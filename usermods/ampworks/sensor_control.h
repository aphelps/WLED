#pragma once
//
// sensor_control.h — the control plane's PURE logic: validating a control frame off the wire, and
// deciding whether it should be applied. No Arduino, no WLED globals, no I/O, so all of it is
// exercised by the host tests in tests/sensor_control_test.cpp.
//
// This lives on the WLED (edge) side rather than in sensor_sync_protocol.h because only edges
// resolve control conflicts — a router relays control frames without inspecting their payload. The
// shared header carries the numbers and the struct; the decisions live with whoever makes them,
// the same split M3c established for the router's relay/election logic.
//
#include <stdint.h>
#include <string.h>
#include "sensor_sync_protocol.h"

// Total order over commands: higher Lamport clock wins, deviceId breaks ties. Plain comparisons,
// because a Lamport clock only ever increases — so unlike a wall clock this needs no wraparound
// trickery, and crucially it IS transitive. An RFC-1982-style compare is only an order within a
// half-range window: with values spread wider than 2^31 you can have a<b and b<c but not a<c, and
// two nodes seeing the same commands in different orders then settle on different states forever.
//
// This deliberately does not use the header's `timestamp`. That is `millis() + Segment::timebase`,
// a per-node quantity — nothing assigns `timebase` from the mesh leader, WLED's own sync rewrites
// it, and `resetTimebase()` zeroes it whenever the strip switches on — so it is not a shared clock
// and ordering by it would be unsound in exactly the way described above.
static inline bool ss_ctrl_newer(uint32_t aClock, uint32_t aDev, uint32_t bClock, uint32_t bDev) {
  if (aClock != bClock) return aClock > bClock;
  return aDev > bDev;
}

// What a node remembers about the last command it applied, plus its own logical clock. No
// per-origin table: the order is total, so only the current winner needs remembering.
struct ControlState {
  uint32_t lastClock;
  uint32_t lastDev;
  uint32_t clock;      // our Lamport clock
  bool     have;
};

static inline ControlState ss_ctrl_init() {
  return ControlState{ 0, 0, 0, false };
}

// Stamp an outgoing command: bump our clock and return the value to put on the wire.
// Saturates at u32 max rather than wrapping: a wrap puts the clock back near 0 and mutes this
// node against every peer (the ceiling already saturates for the same reason). Reaching the top
// honestly takes ~4e9 commands; a node driven there by an adopted maximum stays parked at max —
// its commands tie rather than win — instead of wedging the whole installation.
static inline uint32_t ss_ctrl_tick(ControlState &st) {
  if (st.clock == 0xFFFFFFFFUL) return st.clock;
  return ++st.clock;
}

// Raise our clock to at least what we just heard, so our next command sorts after it. Called for
// every received command, INCLUDING ones we decline to apply — otherwise a node that lost a race
// would keep issuing commands that lose, and a node that had rebooted (clock back to 0) could
// never catch up.
// The clamp is not defensive tidiness. Without it one frame carrying lamport = 0xFFFFFFFF wraps
// every command this node subsequently originates back to 0, so every peer rejects them forever:
// a one-packet permanent DoS on an unauthenticated open broadcast. A jump of more than
// SS_CTRL_MAX_JUMP past our own clock is not a real installation getting ahead of us, so we decline
// to follow it — a genuine peer that far ahead would have to have issued 2^20 commands unheard.
#define SS_CTRL_MAX_JUMP  0x00100000UL   // ~1M commands

static inline void ss_ctrl_observe(ControlState &st, uint32_t heardClock) {
  if (heardClock <= st.clock) return;
  if (heardClock - st.clock > SS_CTRL_MAX_JUMP) return;   // implausible: ignore rather than follow
  st.clock = heardClock;
}

// ---- clock durability across reboots (plan decision 5, Adam 2026-08-07) ---------------------
//
// The clock used to be RAM-only, so a rebooted gateway restarted at 1 and — because control frames
// are deliberately never re-broadcast — had nothing to hear and stayed permanently muted whenever
// it was the only source of commands. Two mechanisms together, because neither alone is enough:
//
//   persisted reservation   handles "I have been here before"     — but not a NEW node (nothing
//                                                                   persisted), nor one that fell
//                                                                   far behind while away
//   solicited query reply   handles "tell me where the mesh is"   — but assumes clock 1 when the
//                                                                   reply is lost or nobody answers
//
//   resume at  max(persisted_ceiling, consensus_of_replies)
//
// tick() pre-increments, so the first command published is that value + 1.
#define SS_CTRL_RESERVE  1000UL   // clock values claimed per NVS write

// Adopt a clock learned from a reply to a query WE issued, inside the window we opened for it.
// Deliberately bypasses SS_CTRL_MAX_JUMP. That clamp exists for UNSOLICITED frames from anyone;
// applying it here would defeat the case this whole mechanism exists for, because a node more than
// ~1M commands behind would refuse to believe the mesh and stay locked out by our own anti-DoS
// rule — ~14 years at tap rates, but ~29 hours at 10 commands/sec, so a node off for a long
// weekend would never rejoin. The corroboration rule below is what replaces the clamp's protection.
static inline void ss_ctrl_adopt(ControlState &st, uint32_t clock) {
  if (clock > st.clock) st.clock = clock;
}

// A reply as collected: the claimed clock plus the header deviceId it arrived under.
// The deviceId is what corroboration counts — see below.
struct SsCtrlReply {
  uint32_t clock;
  uint32_t deviceId;
};

// Decide which reply to believe. Bypassing the clamp re-opens the poison-frame DoS on this path —
// one reply of 0xFFFFFFFF would set our clock to the maximum and park it there, which is the
// exact bug the clamp was added for. So corroboration replaces the clamp: with two or more
// DISTINCT senders a lone outlier more than SS_CTRL_MAX_JUMP above the runner-up is discarded and
// the true maximum is used, so a single liar cannot move us and honest spread (peers differing by
// a command or two because of packet loss) costs us nothing.
//
// Corroboration counts SENDERS, not frames: the CTRL_QUERY broadcast announces on-air that the
// reply window just opened, so one liar answering twice with two nearby values must remain ONE
// voice — replies are deduped by deviceId first, keeping each sender's highest claim. The header
// deviceId is spoofable on this unauthenticated channel, so this is not a defence against a
// deliberate multi-identity attacker (only auth would be); it restores the stated guarantee
// against the realistic threat, a single buggy or half-upgraded sender.
//
// With exactly ONE distinct sender there is nothing to corroborate against, so we fall back to
// the unsolicited rule and refuse a jump beyond the clamp. That trades the long-absent-node
// recovery away in single-peer meshes only; with two or more peers answering, recovery is
// unbounded.
static inline uint32_t ss_ctrl_reply_consensus(const SsCtrlReply *replies, int n,
                                               uint32_t persisted) {
  if (replies == 0 || n <= 0) return 0;
  // Dedup by sender, keeping each sender's highest claim. n is small (reply buffer is 8).
  uint32_t voice[16]; uint32_t voiceDev[16]; int voices = 0;
  for (int i = 0; i < n && voices < 16; i++) {
    int j = 0;
    for (; j < voices; j++) {
      if (voiceDev[j] == replies[i].deviceId) {
        if (replies[i].clock > voice[j]) voice[j] = replies[i].clock;
        break;
      }
    }
    if (j == voices) { voiceDev[voices] = replies[i].deviceId; voice[voices++] = replies[i].clock; }
  }
  uint32_t hi = 0, second = 0;
  for (int i = 0; i < voices; i++) {
    uint32_t v = voice[i];
    if (v > hi)          { second = hi; hi = v; }
    else if (v > second) { second = v; }
  }
  if (voices == 1) {
    if (hi > persisted && hi - persisted > SS_CTRL_MAX_JUMP) return persisted;
    return hi;
  }
  if (hi - second > SS_CTRL_MAX_JUMP) return second;   // lone outlier: discard it
  return hi;
}

// The clock to resume at after a reboot.
static inline uint32_t ss_ctrl_start(uint32_t persistedCeiling, uint32_t consensus) {
  return persistedCeiling > consensus ? persistedCeiling : consensus;
}

// The ceiling to persist when claiming the next block. Saturates rather than wrapping: wrapping
// would put the clock back near 0 and mute this node against every peer, which is the failure this
// file exists to prevent.
static inline uint32_t ss_ctrl_next_ceiling(uint32_t clock) {
  if (clock > 0xFFFFFFFFUL - SS_CTRL_RESERVE) return 0xFFFFFFFFUL;
  return clock + SS_CTRL_RESERVE;
}

// Must we claim another block before publishing? Checked before tick(), not after, so the clock
// never advances past a ceiling that was actually written to flash.
static inline bool ss_ctrl_needs_reserve(uint32_t clock, uint32_t ceiling) {
  return clock >= ceiling;
}

// Record a command this node originated. The originator MUST do this: it is competing in the same
// total order as everyone else, and skipping it leaves `have` false so the next stale command to
// arrive is accepted here while every peer correctly rejects it — a divergence that persists.
static inline void ss_ctrl_record_own(ControlState &st, uint32_t clock, uint32_t selfId) {
  st.lastClock = clock;
  st.lastDev   = selfId;
  st.have      = true;
}

// Should this command be applied? Rejects, in order:
//   - our own command coming back to us. The router's dedup stops a frame looping through the
//     backbone, but nothing stops an edge that re-broadcasts what it applied from re-originating
//     it, so the edge has to decline to apply its own echo.
//   - anything not strictly newer than what we already applied, which is what makes a late or
//     duplicated frame from a slower path harmless.
// Updates `st` only when it returns true.
static inline bool ss_ctrl_should_apply(ControlState &st, uint32_t clock, uint32_t dev,
                                        uint32_t selfId) {
  ss_ctrl_observe(st, clock);   // even a rejected command advances our clock
  if (dev == selfId) return false;
  if (st.have && !ss_ctrl_newer(clock, dev, st.lastClock, st.lastDev)) return false;
  st.lastClock = clock;
  st.lastDev   = dev;
  st.have      = true;
  return true;
}

// Validate a CTRL_QUERY (no payload) or a CTRL_CLOCK reply. Split from ss_parse_control because
// these carry a clock, never a command, and must never reach the apply path.
static inline bool ss_parse_ctrl_query(const uint8_t *pkt, int len, uint32_t selfId,
                                       SensorSyncHeader &h) {
  if (pkt == 0 || len < (int)sizeof(SensorSyncHeader)) return false;
  memcpy(&h, pkt, sizeof(h));
  if (!(h.magic[0] == 'A' && h.magic[1] == 'M' && h.magic[2] == 'P' && h.magic[3] == 'S'))
    return false;
  if (h.version != SENSOR_SYNC_VERSION) return false;
  if (h.msgType != SENSOR_SYNC_MSG_CTRL_QUERY) return false;
  if (h.deviceId == selfId) return false;   // never answer our own query
  return true;
}

static inline bool ss_parse_ctrl_clock(const uint8_t *pkt, int len, uint32_t selfId,
                                       SensorSyncHeader &h, SensorControlClock &out) {
  if (pkt == 0 || len < (int)sizeof(SensorSyncHeader)) return false;
  memcpy(&h, pkt, sizeof(h));
  if (!(h.magic[0] == 'A' && h.magic[1] == 'M' && h.magic[2] == 'P' && h.magic[3] == 'S'))
    return false;
  if (h.version != SENSOR_SYNC_VERSION) return false;
  if (h.msgType != SENSOR_SYNC_MSG_CTRL_CLOCK) return false;
  if (h.deviceId == selfId) return false;
  if ((int)sizeof(SensorSyncHeader) + (int)h.dataLen > len) return false;
  if (h.dataLen < (int)sizeof(SensorControlClock)) return false;
  memcpy(&out, pkt + sizeof(SensorSyncHeader), sizeof(out));
  return true;
}

// Validate a control datagram and extract both header and payload. Mirrors ss_parse_header's
// checks but gates on msgType CONTROL; ss_parse_header itself is deliberately left alone, since
// widening it would loosen the sensor RX path to admit frames it has no handler for.
//
// dataLen greater than the struct is ACCEPTED, copying only the bytes this build understands, so a
// future release can extend SensorControl without stranding older nodes. Unknown `fields` bits are
// likewise ignored rather than rejected — an old node applies the parts it knows.
static inline bool ss_parse_control(const uint8_t *pkt, int len, uint32_t selfId,
                                    SensorSyncHeader &h, SensorControl &c) {
  if (pkt == 0) return false;
  if (len < (int)sizeof(SensorSyncHeader)) return false;
  memcpy(&h, pkt, sizeof(h));
  if (!(h.magic[0] == 'A' && h.magic[1] == 'M' && h.magic[2] == 'P' && h.magic[3] == 'S'))
    return false;
  if (h.version != SENSOR_SYNC_VERSION) return false;
  if (h.msgType != SENSOR_SYNC_MSG_CONTROL) return false;
  if (h.deviceId == selfId) return false;
  if ((int)sizeof(SensorSyncHeader) + (int)h.dataLen > len) return false;  // truncated payload
  if (h.dataLen < (int)sizeof(SensorControl)) return false;                // too short to be one
  memcpy(&c, pkt + sizeof(SensorSyncHeader), sizeof(c));
  return true;
}
