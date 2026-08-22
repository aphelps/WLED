// Host unit test for the HTTP command endpoint's pure half (rs485_bridge_http.h): body decoding
// and the pending-request table that makes the deferred response correct.
//
// Builds with a normal host compiler — no Arduino, no WLED, no AsyncWebServer. From the super-repo
// root:
//
//   c++ -std=c++11 -Wall -Wextra
//       -I HMTL/Libraries/HMTLprotocol -I ArduinoLibs/Socket
//       -o /tmp/rs485_http_test WLED/usermods/rs485_bridge/tests/rs485_bridge_http_test.cpp
//     && /tmp/rs485_http_test
//
// The point of the split this file tests: every interesting way the endpoint can be wrong is in
// here — a reply handed to the wrong request, a request completed after its client vanished, a
// deadline that never fires — and none of them need a web server or a bus to reproduce.
//
// Exits 0 on success, 1 if any assertion failed.
//
#include "../rs485_bridge_http.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

int main() {
  // --- hex decoding -----------------------------------------------------------------------
  {
    uint8_t out[16];
    const char *s = "0a1b2c";
    CHECK(rs485b_decode_hex(s, 6, out, sizeof(out)) == 3, "three bytes from six digits");
    CHECK(out[0] == 0x0a && out[1] == 0x1b && out[2] == 0x2c, "hex bytes land in order");

    const char *spaced = "0a 1b 2c";
    CHECK(rs485b_decode_hex(spaced, 8, out, sizeof(out)) == 3, "spaces between bytes are skipped");
    const char *upper = "AABBCC";
    CHECK(rs485b_decode_hex(upper, 6, out, sizeof(out)) == 3 && out[0] == 0xAA, "uppercase works");

    // An odd digit count is rejected rather than zero-padded. A truncated paste is far more likely
    // than a deliberate request for a leading zero, and a silently shortened frame is one that
    // reaches the bus malformed.
    CHECK(rs485b_decode_hex("abc", 3, out, sizeof(out)) == -1, "odd digit count is an error");
    CHECK(rs485b_decode_hex("0a 1", 4, out, sizeof(out)) == -1, "trailing nibble is an error");
    CHECK(rs485b_decode_hex("0a1g", 4, out, sizeof(out)) == -1, "a non-hex character is an error");
    CHECK(rs485b_decode_hex("0 a", 3, out, sizeof(out)) == -1, "a separator mid-byte is an error");

    // Overflow must be refused, not truncated: the buffer this fills is the one handed to
    // validation and then to the bus.
    uint8_t tiny[2];
    CHECK(rs485b_decode_hex("0a1b2c", 6, tiny, sizeof(tiny)) == -1, "overflow is refused");
    CHECK(rs485b_decode_hex("", 0, out, sizeof(out)) == 0, "empty input decodes to zero bytes");
  }

  // --- base64 decoding --------------------------------------------------------------------
  {
    uint8_t out[16];
    // "Man" -> TWFu ; the classic padding cases.
    CHECK(rs485b_decode_base64("TWFu", 4, out, sizeof(out)) == 3, "4 chars -> 3 bytes");
    CHECK(memcmp(out, "Man", 3) == 0, "base64 round-trips a known vector");
    CHECK(rs485b_decode_base64("TWE=", 4, out, sizeof(out)) == 2, "one pad char -> 2 bytes");
    CHECK(memcmp(out, "Ma", 2) == 0, "padded vector decodes");
    CHECK(rs485b_decode_base64("TQ==", 4, out, sizeof(out)) == 1, "two pad chars -> 1 byte");
    CHECK(out[0] == 'M', "single byte decodes");
    CHECK(rs485b_decode_base64("TWFu\n", 5, out, sizeof(out)) == 3, "wrapped lines are fine");
    CHECK(rs485b_decode_base64("TW*u", 4, out, sizeof(out)) == -1, "an invalid character errors");
    CHECK(rs485b_decode_base64("T", 1, out, sizeof(out)) == -1, "a lone character is not a group");
    uint8_t tiny[1];
    CHECK(rs485b_decode_base64("TWFu", 4, tiny, sizeof(tiny)) == -1, "overflow is refused");
  }

  // --- the hex-before-base64 ordering, which is easy to reverse and silent when wrong -------
  {
    // The hex alphabet is a SUBSET of the base64 alphabet, so a hex string is also decodable as
    // base64 — into completely different bytes. If the endpoint's auto-detect ever tries base64
    // first, every hex frame silently becomes a plausible wrong frame that then fails CRC, and the
    // symptom ("it rejects everything") points nowhere near the decoder.
    //
    // This asserts the property that makes the order matter, so it fails if either decoder is
    // changed such that they stop disagreeing — which is what would make the ordering look safe.
    const char *s = "0a1b2c";
    uint8_t asHex[16], asB64[16];
    int nHex = rs485b_decode_hex(s, 6, asHex, sizeof(asHex));
    int nB64 = rs485b_decode_base64(s, 6, asB64, sizeof(asB64));
    CHECK(nHex == 3, "the sample decodes as hex");
    CHECK(nB64 > 0, "and the SAME string is also valid base64 — this is the trap");
    CHECK(!(nHex == nB64 && memcmp(asHex, asB64, (size_t)nHex) == 0),
          "the two decoders genuinely disagree, so trying them in the wrong order is destructive");
    // And the value hex must win with: 0a 1b 2c, not base64's interpretation.
    CHECK(asHex[0] == 0x0a && asHex[1] == 0x1b && asHex[2] == 0x2c,
          "hex-first must yield the hex bytes");
  }

  // --- the hex/base64 fallback guard (found on hardware, not in a unit test) -------------------
  {
    // "abc" and "zzzz" both failed hex and then SUCCEEDED as base64, producing garbage bytes that
    // failed frame validation — so the endpoint told the caller their FRAME was bad when their
    // ENCODING was bad. The guard is: an all-hex string that will not decode as hex is broken hex,
    // and must not be reinterpreted.
    CHECK(rs485b_looks_like_hex("abc", 3), "odd-length hex is still recognisably hex");
    CHECK(rs485b_looks_like_hex("0a 1b", 5), "separators do not disqualify it");
    CHECK(rs485b_looks_like_hex("DEADBEEF", 8), "uppercase counts");
    CHECK(!rs485b_looks_like_hex("zzzz", 4), "z is outside the hex alphabet");
    CHECK(!rs485b_looks_like_hex("TWFu", 4), "a real base64 payload is not mistaken for hex");
    CHECK(!rs485b_looks_like_hex("", 0), "empty is not hex");
    CHECK(!rs485b_looks_like_hex("   ", 3), "separators alone are not hex");

    // The property that matters: for an all-hex-but-invalid string, base64 WOULD have accepted it.
    // That is exactly why the guard has to exist.
    uint8_t out[16];
    CHECK(rs485b_decode_hex("abc", 3, out, sizeof(out)) == -1, "hex rejects it");
    CHECK(rs485b_decode_base64("abc", 3, out, sizeof(out)) > 0,
          "but base64 accepts it — the misdirection this guard prevents");
  }

  // --- the correlation property, which is the reason this table exists --------------------
  {
    RS485BPendingTable t; rs485b_pending_init(t);
    int dummyA = 1, dummyB = 2;

    int a = rs485b_pending_add(t, 71, MSG_TYPE_POLL, 1000, 1200, &dummyA);
    int b = rs485b_pending_add(t, 72, MSG_TYPE_POLL, 1000, 1200, &dummyB);
    CHECK(a >= 0 && b >= 0 && a != b, "two requests take distinct slots");

    // A reply from 72 must go to the request that addressed 72 — not to the older entry. Matching
    // on arrival order would hand one caller another caller's answer, and the bus is shared with
    // the UDP path, so this is the normal case rather than a contrived one.
    CHECK(rs485b_pending_match(t, 72, MSG_TYPE_POLL) == b, "a reply matches its own address");
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_POLL) == a, "and the other still matches its own");

    // An unsolicited frame matches NOTHING. It must be relayed as before, never attributed to
    // whichever request happens to be waiting.
    CHECK(rs485b_pending_match(t, 99, MSG_TYPE_POLL) == -1, "an unrelated address matches nothing");
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_SENSOR) == -1, "the wrong type matches nothing");

    // Same address AND type: genuinely ambiguous on the wire (HMTL's header carries no request id),
    // resolved oldest-first so behaviour is defined rather than arbitrary.
    rs485b_pending_release(t, a);
    rs485b_pending_release(t, b);
    int first  = rs485b_pending_add(t, 71, MSG_TYPE_POLL, 1000, 1200, &dummyA);
    int second = rs485b_pending_add(t, 71, MSG_TYPE_POLL, 1010, 1200, &dummyB);
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_POLL) == first, "FIFO among identical keys");
    rs485b_pending_release(t, first);
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_POLL) == second, "then the next one");
  }

  // --- timeout, including the millis() wrap -----------------------------------------------
  {
    RS485BPendingTable t; rs485b_pending_init(t);
    int ctx = 1;
    int i = rs485b_pending_add(t, 71, MSG_TYPE_POLL, 1000, 1200, &ctx);
    CHECK(i >= 0, "slot claimed");
    CHECK(rs485b_pending_expired(t, 1000) == -1, "not expired at issue time");
    CHECK(rs485b_pending_expired(t, 2199) == -1, "not expired one ms early");
    CHECK(rs485b_pending_expired(t, 2200) == i, "expired exactly on the deadline");
    CHECK(rs485b_pending_expired(t, 5000) == i, "and stays expired");
    rs485b_pending_release(t, i);
    CHECK(rs485b_pending_expired(t, 5000) == -1, "a released slot never expires");

    // millis() wraps every ~49.7 days. Unsigned comparison would read every deadline as
    // impossibly distant across the wrap and hold requests open forever; signed difference is what
    // makes this correct, so assert it across the boundary rather than trusting the idiom.
    RS485BPendingTable w; rs485b_pending_init(w);
    const uint32_t nearMax = 0xFFFFFF00u;
    int j = rs485b_pending_add(w, 71, MSG_TYPE_POLL, nearMax, 1200, &ctx);
    CHECK(rs485b_pending_expired(w, nearMax + 100) == -1, "before deadline, across the wrap");
    CHECK(rs485b_pending_expired(w, nearMax + 1200) == j, "expires correctly after wrapping");
  }

  // --- client disconnect: the AsyncWebServer footgun ---------------------------------------
  {
    RS485BPendingTable t; rs485b_pending_init(t);
    int reqA = 1, reqB = 2;
    int a = rs485b_pending_add(t, 71, MSG_TYPE_POLL, 1000, 1200, &reqA);
    rs485b_pending_add(t, 72, MSG_TYPE_POLL, 1000, 1200, &reqB);

    // When a client vanishes its request object is freed. If the slot outlived it, loop() would
    // complete a response into freed memory the moment the reply arrived — so cancellation must
    // make the entry unmatchable, not merely mark it.
    CHECK(rs485b_pending_cancel_ctx(t, &reqA) == a, "cancel finds the entry by request handle");
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_POLL) == -1,
          "a cancelled request is never completed afterwards");
    CHECK(rs485b_pending_expired(t, 9999) != a, "nor does it later time out");
    CHECK(rs485b_pending_match(t, 72, MSG_TYPE_POLL) >= 0, "the other request is untouched");
    CHECK(rs485b_pending_cancel_ctx(t, &reqA) == -1, "cancelling twice is harmless");
    CHECK(rs485b_pending_cancel_ctx(t, 0) == -1, "a null handle matches nothing");
  }

  // --- table exhaustion --------------------------------------------------------------------
  {
    RS485BPendingTable t; rs485b_pending_init(t);
    int ctx[RS485B_HTTP_MAX_PENDING + 1];
    for (int i = 0; i < RS485B_HTTP_MAX_PENDING; i++)
      CHECK(rs485b_pending_add(t, (uint16_t)(71 + i), MSG_TYPE_POLL, 1000, 1200, &ctx[i]) >= 0,
            "table fills");
    // Refusing is the designed answer: the caller then replies "accepted, unconfirmed" instead of
    // holding an async request object for an unbounded time.
    CHECK(rs485b_pending_add(t, 99, MSG_TYPE_POLL, 1000, 1200, &ctx[RS485B_HTTP_MAX_PENDING]) == -1,
          "a full table refuses rather than evicting");
    // Refusal must not have disturbed the existing entries — evicting one would silently strand
    // whichever caller was unlucky.
    CHECK(rs485b_pending_match(t, 71, MSG_TYPE_POLL) >= 0, "existing entries survive a refusal");
  }

  // --- the caller is told which guarantee they got ------------------------------------------
  {
    // Distinct strings, because a caller that cannot tell "the module replied" from "we put it on
    // the wire and stopped waiting" has the UDP guarantee with extra steps — which is the entire
    // reason this endpoint exists.
    CHECK(strcmp(rs485b_http_outcome_str(RS485B_HTTP_REPLIED),
                 rs485b_http_outcome_str(RS485B_HTTP_ACCEPTED)) != 0,
          "replied and accepted-unconfirmed are distinguishable");
    CHECK(strcmp(rs485b_http_outcome_str(RS485B_HTTP_LOCAL), "applied-locally") == 0,
          "local application is named");
    CHECK(strcmp(rs485b_http_outcome_str(RS485B_HTTP_REJECTED), "rejected") == 0,
          "rejection is named");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
