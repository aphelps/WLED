// Host unit test for the SPSC RX ring (SpscByteRing) used by the ESP-NOW transport.
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure ring header:
//   c++ -std=c++11 -Wall -o /tmp/ss_ring_test sensor_sync_ring_test.cpp && /tmp/ss_ring_test
// Exits 0 on success, 1 on the first failed assertion.
//
// Covers the SPSC semantics: fill-to-capacity, full->drop-newest, FIFO drain order,
// wrap-around across the modulus, empty->returns 0, and bad-slot skip (never halts draining).
//
#include "../sensor_sync_ring.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

// Small ring: RING=4 -> capacity 3 usable slots, SLOT_LEN=8.
typedef SpscByteRing<4, 8> Ring;

// Push a one-byte "tagged" frame; returns whether it was stored.
static bool push_tag(Ring &r, uint8_t tag) {
  uint8_t b[1] = { tag };
  return r.push(b, 1);
}
// Pop one frame; returns the tag byte, or -1 if empty.
static int pop_tag(Ring &r) {
  uint8_t b[8];
  int n = r.pop(b, sizeof(b));
  return n > 0 ? (int)b[0] : -1;
}

int main() {
  // 1) Empty ring returns 0 / -1.
  {
    Ring r;
    CHECK(r.empty(), "new ring empty");
    CHECK(r.size() == 0, "new ring size 0");
    CHECK(pop_tag(r) == -1, "empty pop returns 0");
  }

  // 2) Fill to capacity, then full -> drop-newest.
  {
    Ring r;
    CHECK(Ring::capacity() == 3, "capacity is RING-1");
    CHECK(push_tag(r, 10), "push 1 of 3");
    CHECK(push_tag(r, 11), "push 2 of 3");
    CHECK(push_tag(r, 12), "push 3 of 3");
    CHECK(r.full(), "ring full at capacity");
    CHECK(r.size() == 3, "size == capacity");
    CHECK(!push_tag(r, 99), "push when full is dropped");        // drop-newest
    CHECK(r.size() == 3, "size unchanged after dropped push");
    // Drain in FIFO order — the dropped-newest (99) is never seen.
    CHECK(pop_tag(r) == 10, "FIFO drain 1");
    CHECK(pop_tag(r) == 11, "FIFO drain 2");
    CHECK(pop_tag(r) == 12, "FIFO drain 3");
    CHECK(r.empty(), "empty after full drain");
    CHECK(pop_tag(r) == -1, "empty pop after drain");
  }

  // 3) Wrap-around across the modulus: interleave many push/pop so head/tail wrap several times.
  {
    Ring r;
    uint8_t expect = 0;
    for (uint8_t round = 0; round < 20; round++) {
      CHECK(push_tag(r, (uint8_t)(round * 2)), "wrap push a");
      CHECK(push_tag(r, (uint8_t)(round * 2 + 1)), "wrap push b");
      CHECK(pop_tag(r) == expect++, "wrap pop a");
      CHECK(pop_tag(r) == expect++, "wrap pop b");
    }
    CHECK(r.empty(), "empty after wrap sequence");
  }

  // 4) Oversized / garbage frames on push are dropped (never stored).
  {
    Ring r;
    uint8_t big[16];
    memset(big, 0xEE, sizeof(big));
    CHECK(!r.push(big, 16), "oversized push dropped");           // 16 > SLOT_LEN(8)
    CHECK(!r.push(big, 0), "zero-len push dropped");
    CHECK(!r.push(nullptr, 4), "null push dropped");
    CHECK(r.empty(), "ring still empty after bad pushes");
  }

  // 5) Bad/oversized slot on pop is SKIPPED, draining CONTINUES (regression: must not return 0
  //    and strand the frames behind it). Simulate by popping into a buffer too small for a
  //    stored frame — pop must skip it and return the next good frame, not halt at the bad one.
  {
    Ring r;
    uint8_t four[4] = { 1, 2, 3, 4 };       // a 4-byte frame
    uint8_t one[1]  = { 42 };               // a 1-byte frame
    CHECK(r.push(four, 4), "push 4-byte frame");
    CHECK(r.push(one, 1), "push 1-byte frame");
    // Pop with maxLen=1: the 4-byte frame does NOT fit -> it must be skipped, and the 1-byte
    // frame returned, rather than returning 0 (which would strand the queue).
    uint8_t out[1];
    int n = r.pop(out, 1);
    CHECK(n == 1 && out[0] == 42, "bad-fit slot skipped, next frame drained");
    CHECK(r.empty(), "ring drained past the skipped slot");
    CHECK(r.pop(out, 1) == 0, "empty after skip+drain");
  }

  // 6) Producer/consumer independence: fill partially, drain partially, refill past the wrap
  //    to confirm no stale count desync (the whole point of the two-index design).
  {
    Ring r;
    push_tag(r, 1); push_tag(r, 2);
    CHECK(pop_tag(r) == 1, "partial drain 1");
    push_tag(r, 3);                          // head wraps ahead of tail
    push_tag(r, 4);                          // now full (2,3,4)
    CHECK(r.full(), "full after refill across partial drain");
    CHECK(!push_tag(r, 5), "drop when full again");
    CHECK(pop_tag(r) == 2, "drain 2");
    CHECK(pop_tag(r) == 3, "drain 3");
    CHECK(pop_tag(r) == 4, "drain 4");
    CHECK(r.empty(), "empty at end");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
