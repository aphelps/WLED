#pragma once
//
// rs485_bridge_http.h — the pure, host-testable half of the HTTP command endpoint.
//
// The endpoint's job is to accept an HMTL frame over HTTP, route it exactly as the UDP path does,
// and — for a frame that expects an answer — hold the request open until the RS485 reply comes back.
// Everything in this file is that logic WITHOUT AsyncWebServer: body decoding, and the pending-
// request table that pairs a reply with the request waiting for it.
//
// The split exists because the interesting failure modes are all in this half (a reply attributed
// to the wrong request, a request completed after its client vanished, a timeout that never fires),
// and none of them are reachable in a test that needs a web server and a live bus.
//
// Nothing here allocates or blocks; the table is a fixed array sized at compile time.
//
#include <stdint.h>
#include <string.h>
#include "rs485_bridge_protocol.h"

// --- Body decoding ----------------------------------------------------------------------------
// The frame arrives as text so it survives JSON. Hex and base64 are both accepted: hex is what a
// human pastes out of a protocol dump, base64 is what a script produces. Both decode into the same
// byte buffer and take the same validation path afterwards — there is deliberately no third
// "structured command JSON" form, which would be a second parser to keep in agreement with the
// first.

// Decode `len` hex characters into `out`. Whitespace between bytes is skipped so a pasted dump with
// spaces works. Returns the byte count, or -1 on any invalid character or odd digit count.
// An odd digit count is an ERROR rather than a leading-zero pad: "abc" is far more likely to be a
// truncated paste than a request for 0x0a 0xbc, and silently accepting truncation is how a short
// frame reaches the bus.
static inline int rs485b_decode_hex(const char *in, int len, uint8_t *out, int outCap) {
  int n = 0;
  int hi = -1;
  for (int i = 0; i < len; i++) {
    char c = in[i];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ':' || c == ',') {
      if (hi >= 0) return -1;          // separator mid-byte: "a b" is not a byte
      continue;
    }
    int v;
    if      (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else return -1;
    if (hi < 0) { hi = v; }
    else {
      if (n >= outCap) return -1;      // would overflow the caller's buffer
      out[n++] = (uint8_t)((hi << 4) | v);
      hi = -1;
    }
  }
  if (hi >= 0) return -1;              // trailing nibble: odd digit count
  return n;
}

// Is every non-separator character in the hex alphabet? Used to decide whether a failed hex decode
// should fall back to base64. An all-hex string that will not decode as hex is broken hex — saying
// so is far more useful than silently reinterpreting it as base64 and blaming the frame.
static inline bool rs485b_looks_like_hex(const char *in, int len) {
  if (in == 0) return false;
  bool any = false;
  for (int i = 0; i < len; i++) {
    char c = in[i];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ':' || c == ',') continue;
    bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!isHex) return false;
    any = true;
  }
  return any;
}

static inline int rs485b_b64_val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

// Decode standard base64 (with or without '=' padding) into `out`. Returns the byte count, or -1 on
// an invalid character, a truncated group, or overflow. Whitespace is skipped so a wrapped line
// works.
static inline int rs485b_decode_base64(const char *in, int len, uint8_t *out, int outCap) {
  uint32_t acc = 0;
  int bits = 0, n = 0;
  for (int i = 0; i < len; i++) {
    char c = in[i];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
    if (c == '=') break;                     // padding: the group is complete
    int v = rs485b_b64_val(c);
    if (v < 0) return -1;
    acc = (acc << 6) | (uint32_t)v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (n >= outCap) return -1;
      out[n++] = (uint8_t)((acc >> bits) & 0xFF);
    }
  }
  // A leftover of 6 bits means a single stray character — not a valid group.
  if (bits >= 6) return -1;
  return n;
}

// --- The pending-request table ----------------------------------------------------------------
//
// A request that expects a reply cannot be answered inside the HTTP handler: AsyncWebServer runs
// handlers on the async TCP task, and an RS485 reply takes milliseconds to hundreds of milliseconds
// (MessageHandler staggers broadcast POLL responses by delay(address * 2), up to ~510 ms at address
// 255). Blocking there stalls the web server, WLED's own UI included. So the handler records what it
// is waiting for and returns; loop() completes the response when the reply arrives or the deadline
// passes.
//
// CORRELATION, and why it is not "the next frame on the bus": HTTP and UDP share one bus, and the
// bridge listens promiscuously. Matching on arrival order would hand an HTTP caller a reply to
// someone else's poll. A pending entry is therefore keyed on the DESTINATION we sent to (the reply's
// source address) plus the message type we expect back, and an unmatched reply stays unmatched —
// it is relayed to the UDP peer as before, never attributed to the oldest waiting request.
//
// Deliberately NOT keyed on a sequence number: HMTL's 8-byte header has no request id, so there is
// nothing to correlate on that the wire actually carries. Two concurrent HTTP requests to the same
// address for the same type are therefore genuinely ambiguous; the table resolves that by matching
// the OLDEST such entry first (FIFO), which is the same answer the UDP path gives by construction.

#ifndef RS485B_HTTP_MAX_PENDING
#define RS485B_HTTP_MAX_PENDING 4
#endif

// How long a request waits for its reply before the endpoint answers "accepted, unconfirmed".
// Sized above the worst-case staggered POLL response (~510 ms at address 255) with margin, and well
// under any sane HTTP client timeout.
#ifndef RS485B_HTTP_REPLY_TIMEOUT_MS
#define RS485B_HTTP_REPLY_TIMEOUT_MS 1200
#endif

enum RS485BPendingState : uint8_t {
  RS485B_PEND_FREE = 0,
  RS485B_PEND_WAITING,   // sent; waiting for a reply or the deadline
};

struct RS485BPending {
  RS485BPendingState state;
  uint16_t addr;         // address we sent to == address a reply comes FROM
  uint8_t  msgType;      // the type we expect back
  uint32_t deadlineMs;
  uint32_t seq;          // monotonic issue order, for FIFO matching among equal keys
  void    *ctx;          // opaque handle to the async request (never dereferenced here)
};

struct RS485BPendingTable {
  RS485BPending slot[RS485B_HTTP_MAX_PENDING];
  uint32_t      nextSeq;
};

static inline void rs485b_pending_init(RS485BPendingTable &t) {
  memset(&t, 0, sizeof(t));
  t.nextSeq = 1;
}

// Claim a slot. Returns the slot index, or -1 when the table is full — the caller then answers
// immediately with the "accepted, unconfirmed" shape rather than queueing. Refusing is deliberate:
// a queue here would hold async request objects for an unbounded time, which is the leak this
// fixed-size table exists to prevent.
static inline int rs485b_pending_add(RS485BPendingTable &t, uint16_t addr, uint8_t msgType,
                                     uint32_t nowMs, uint32_t timeoutMs, void *ctx) {
  for (int i = 0; i < RS485B_HTTP_MAX_PENDING; i++) {
    if (t.slot[i].state != RS485B_PEND_FREE) continue;
    t.slot[i].state      = RS485B_PEND_WAITING;
    t.slot[i].addr       = addr;
    t.slot[i].msgType    = msgType;
    t.slot[i].deadlineMs = nowMs + timeoutMs;
    t.slot[i].seq        = t.nextSeq++;
    t.slot[i].ctx        = ctx;
    return i;
  }
  return -1;
}

// Find the entry a reply belongs to: same source address, same type, oldest first. Returns the slot
// index, or -1 if nothing is waiting for this frame — in which case the caller must treat the reply
// as unsolicited (relay it) rather than guessing.
static inline int rs485b_pending_match(const RS485BPendingTable &t, uint16_t fromAddr,
                                       uint8_t msgType) {
  int best = -1;
  for (int i = 0; i < RS485B_HTTP_MAX_PENDING; i++) {
    const RS485BPending &p = t.slot[i];
    if (p.state != RS485B_PEND_WAITING) continue;
    if (p.addr != fromAddr || p.msgType != msgType) continue;
    if (best < 0 || p.seq < t.slot[best].seq) best = i;
  }
  return best;
}

// Release a slot. Used on completion, on timeout, and when a client disconnects — the last is what
// keeps loop() from writing into a freed request object, the classic AsyncWebServer footgun.
static inline void rs485b_pending_release(RS485BPendingTable &t, int idx) {
  if (idx < 0 || idx >= RS485B_HTTP_MAX_PENDING) return;
  memset(&t.slot[idx], 0, sizeof(t.slot[idx]));
}

// Find one entry whose deadline has passed, or -1. Signed comparison so a millis() wrap does not
// make every deadline look infinitely far away.
static inline int rs485b_pending_expired(const RS485BPendingTable &t, uint32_t nowMs) {
  for (int i = 0; i < RS485B_HTTP_MAX_PENDING; i++) {
    if (t.slot[i].state != RS485B_PEND_WAITING) continue;
    if ((int32_t)(nowMs - t.slot[i].deadlineMs) >= 0) return i;
  }
  return -1;
}

// Cancel by opaque handle, for the disconnect callback, which knows the request but not the slot.
static inline int rs485b_pending_cancel_ctx(RS485BPendingTable &t, void *ctx) {
  if (ctx == 0) return -1;
  for (int i = 0; i < RS485B_HTTP_MAX_PENDING; i++) {
    if (t.slot[i].state == RS485B_PEND_WAITING && t.slot[i].ctx == ctx) {
      rs485b_pending_release(t, i);
      return i;
    }
  }
  return -1;
}

// --- What the caller was promised --------------------------------------------------------------
// The response body must say which guarantee the caller got, because the whole reason to prefer
// HTTP over the fire-and-forget UDP path is knowing whether the device acted. A caller that cannot
// tell "the module replied" from "we put it on the wire and stopped waiting" is back to the UDP
// guarantee with extra steps.
enum RS485BHttpOutcome : uint8_t {
  RS485B_HTTP_LOCAL = 0,   // addressed to this node; applied here, nothing went on the bus
  RS485B_HTTP_REPLIED,     // forwarded, and the module's reply is included
  RS485B_HTTP_ACCEPTED,    // forwarded, no reply expected or none arrived before the deadline
  RS485B_HTTP_REJECTED,    // never reached the bus — see the RS485BFrameResult
};

static inline const char *rs485b_http_outcome_str(RS485BHttpOutcome o) {
  switch (o) {
    case RS485B_HTTP_LOCAL:    return "applied-locally";
    case RS485B_HTTP_REPLIED:  return "replied";
    case RS485B_HTTP_ACCEPTED: return "accepted-unconfirmed";
    default:                   return "rejected";
  }
}
