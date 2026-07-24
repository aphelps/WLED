#pragma once
//
// sensor_sync_ring.h — a correct single-producer / single-consumer (SPSC) lock-free ring
// for the ESP-NOW RX path of SensorSync.
//
// Why this exists: QuickEspNow delivers its receive callback from a dedicated FreeRTOS task,
// which reaches EspNowSensorTransport::feed() (the PRODUCER). Meanwhile poll() runs on the
// Arduino loop() task (the CONSUMER). They run CONCURRENTLY. A shared non-atomic `count`
// written by both sides is a data race (torn read-modify-write, cached-value risk). This ring
// removes that: it derives full/empty purely from two indices, each written by ONE side only.
//
// SPSC invariant:
//   - `head` (write index) is written ONLY by the producer feed().
//   - `tail` (read index)  is written ONLY by the consumer poll().
//   - EMPTY  <=> head == tail
//   - FULL   <=> (head + 1) % N == tail   (one slot is always left free to distinguish the two)
//   Neither side ever writes the other's index, so there is no shared read-modify-write and no
//   lock/portMUX is needed. Both indices are `volatile` so each side always observes the other's
//   latest published value rather than a cached copy. On the release path (feed) the slot bytes
//   are written BEFORE `head` advances; on the acquire path (poll) `head` is read BEFORE the slot
//   bytes are read — so a consumer that sees an advanced `head` also sees the fully-written slot.
//   We make that ordering explicit with a minimal release/acquire fence pair.
//
// This header is deliberately WLED/Arduino-free (only <stdint.h>/<string.h>/<atomic>) so the ring
// can be compiled and unit-tested on a host build (tests/sensor_sync_ring_test.cpp).
//
#include <stdint.h>
#include <string.h>
#include <atomic>

// Fixed-capacity SPSC ring of byte slots. Capacity is (RING - 1) usable entries (one slot is
// reserved to keep FULL and EMPTY distinguishable without a shared counter).
template <uint8_t RING, uint16_t SLOT_LEN>
class SpscByteRing {
 public:
  void clear() {
    // Only safe to call while no producer/consumer is racing (e.g. begin()/end()).
    head = 0;
    tail = 0;
  }

  // PRODUCER side. Copy one frame in; publish by advancing head. Returns true if stored, false
  // if the frame was dropped (oversized/garbage, or ring full). Policy on full: DROP-NEWEST —
  // the incoming frame is discarded (a keyframe re-broadcast re-syncs state), which keeps the
  // consumer draining the older queued frames uninterrupted.
  bool push(const uint8_t *data, int len) {
    if (data == nullptr || len <= 0 || len > (int)SLOT_LEN) return false;  // ignore oversized/garbage
    const uint8_t h = head;                       // producer owns head; plain read of own index
    const uint8_t next = (uint8_t)((h + 1) % RING);
    if (next == tail) return false;               // FULL (tail is written by consumer) — drop-newest
    Slot &s = slots[h];
    memcpy(s.buf, data, (size_t)len);
    s.len = (uint16_t)len;
    std::atomic_thread_fence(std::memory_order_release);  // slot bytes complete before head advances
    head = next;                                  // publish
    return true;
  }

  // CONSUMER side. Copy the next frame out into buf. Returns bytes copied (>0), or 0 when the
  // ring is EMPTY. A slot that does not fit `maxLen` is DROPPED and draining CONTINUES to the
  // next slot (it never returns 0 for a bad slot — that would strand queued frames behind it).
  int pop(uint8_t *buf, int maxLen) {
    for (;;) {
      const uint8_t t = tail;                     // consumer owns tail; plain read of own index
      if (t == head) return 0;                    // EMPTY (head is written by producer)
      std::atomic_thread_fence(std::memory_order_acquire);  // pair with feed's release fence
      Slot &s = slots[t];
      const int n = (int)s.len;
      const uint8_t next = (uint8_t)((t + 1) % RING);
      if (n <= 0 || n > maxLen) {                 // bad/oversized slot: skip it, keep draining
        tail = next;
        continue;
      }
      memcpy(buf, s.buf, (size_t)n);
      tail = next;                                // consume
      return n;
    }
  }

  // Test/introspection helper — number of occupied slots. Only meaningful when not racing.
  uint8_t size() const {
    return (uint8_t)((head + RING - tail) % RING);
  }
  bool empty() const { return head == tail; }
  bool full()  const { return (uint8_t)((head + 1) % RING) == tail; }
  static constexpr uint8_t capacity() { return RING - 1; }

 private:
  struct Slot { uint8_t buf[SLOT_LEN]; uint16_t len; };
  Slot slots[RING] = {};
  volatile uint8_t head = 0;   // write index — written ONLY by producer push()
  volatile uint8_t tail = 0;   // read index  — written ONLY by consumer pop()
};
