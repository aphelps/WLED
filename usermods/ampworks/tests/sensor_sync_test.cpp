// Host unit test for the SensorSync wire protocol + dispatch logic.
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure protocol header:
//   c++ -std=c++11 -Wall -o /tmp/ss_test sensor_sync_test.cpp && /tmp/ss_test
// Exits 0 on success, 1 on the first failed assertion.
//
#include "../sensor_sync_protocol.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

// Serialize a snapshot/sample message into buf; returns total length.
static int build_mask(uint8_t *buf, uint8_t sensorType, uint32_t dev, uint16_t seq, uint16_t mask) {
  SensorSyncHeader h{};
  memcpy(h.magic, "AMPS", 4);
  h.version = SENSOR_SYNC_VERSION; h.msgType = SENSOR_SYNC_MSG_SNAPSHOT;
  h.sensorType = sensorType; h.dataLen = sizeof(SensorSnapshot);
  h.deviceId = dev; h.seq = seq; h.reserved = 0; h.timestamp = 1000 + seq;
  SensorSnapshot s{mask};
  memcpy(buf, &h, sizeof(h)); memcpy(buf + sizeof(h), &s, sizeof(s));
  return sizeof(h) + sizeof(s);
}
static int build_sample(uint8_t *buf, uint8_t sensorType, uint32_t dev, uint16_t seq,
                        uint8_t ch, int16_t val) {
  SensorSyncHeader h{};
  memcpy(h.magic, "AMPS", 4);
  h.version = SENSOR_SYNC_VERSION; h.msgType = SENSOR_SYNC_MSG_SNAPSHOT;
  h.sensorType = sensorType; h.dataLen = sizeof(SensorSample);
  h.deviceId = dev; h.seq = seq; h.reserved = 0; h.timestamp = 2000 + seq;
  SensorSample s{ch, val};
  memcpy(buf, &h, sizeof(h)); memcpy(buf + sizeof(h), &s, sizeof(s));
  return sizeof(h) + sizeof(s);
}

// Run one datagram through parse+dispatch; returns #events (or -1 if header rejected).
static int feed(const uint8_t *pkt, int len, uint32_t selfId,
                SensorPeer *peers, uint8_t maxPeers, RemoteSensorEvent *out, uint8_t cap) {
  SensorSyncHeader h;
  if (!ss_parse_header(pkt, len, selfId, h)) return -1;
  SensorEventSink sink{out, cap, 0};
  ss_dispatch(h, pkt + sizeof(SensorSyncHeader), h.dataLen, peers, maxPeers, sink);
  return sink.count;
}

int main() {
  const uint32_t SELF = 0x1111, A = 0xAAAA, B = 0xBBBB;
  uint8_t pkt[64];
  RemoteSensorEvent ev[32];
  SensorPeer peers[4];

  // 1) First touch snapshot from A: mask 0b0101 -> two presses (channels 0 and 2), resync from 0.
  memset(peers, 0, sizeof(peers));
  int n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 1, 0b0101), SELF, peers, 4, ev, 32);
  CHECK(n == 2, "first snapshot: 2 presses");
  CHECK(ev[0].channel == 0 && ev[0].value == 1, "ch0 press");
  CHECK(ev[1].channel == 2 && ev[1].value == 1, "ch2 press");

  // 2) Next snapshot: ch2 released, ch1 pressed -> one release (2), one press (1).
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 2, 0b0011), SELF, peers, 4, ev, 32);
  CHECK(n == 2, "diff snapshot: 2 edges");
  bool sawRel2 = false, sawPress1 = false;
  for (int i = 0; i < n; i++) {
    if (ev[i].channel == 2 && ev[i].value == 0) sawRel2 = true;
    if (ev[i].channel == 1 && ev[i].value == 1) sawPress1 = true;
  }
  CHECK(sawRel2 && sawPress1, "release ch2 + press ch1");

  // 3) Duplicate seq (2) -> dropped (dedup), no events even though mask differs.
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 2, 0b1111), SELF, peers, 4, ev, 32);
  CHECK(n == 0, "duplicate seq dropped");

  // 4) Reordered / older seq (1) -> dropped.
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 1, 0b1111), SELF, peers, 4, ev, 32);
  CHECK(n == 0, "older seq dropped");

  // 5) Newer seq with wraparound (seq 65535 then 0) is accepted as newer.
  memset(peers, 0, sizeof(peers));
  feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 65535, 0b0001), SELF, peers, 4, ev, 32);
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 0, 0b0011), SELF, peers, 4, ev, 32);
  CHECK(n == 1 && ev[0].channel == 1 && ev[0].value == 1, "seq wraparound accepted");

  // 6) Our own packet (deviceId == SELF) is rejected at header parse.
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, SELF, 5, 0b0001), SELF, peers, 4, ev, 32);
  CHECK(n == -1, "self packet rejected");

  // 7) dataLen too short for the struct -> no events (payload rejected).
  {
    int len = build_mask(pkt, SS_SENSOR_TOUCH, B, 1, 0b0001);
    SensorSyncHeader h; memcpy(&h, pkt, sizeof(h));
    h.dataLen = 1;                 // claim only 1 byte of data
    memcpy(pkt, &h, sizeof(h));
    memset(peers, 0, sizeof(peers));
    n = feed(pkt, len, SELF, peers, 4, ev, 32);
    CHECK(n == 0, "short dataLen -> no events");
  }

  // 8) Proximity sample surfaces as a scalar event (no edge derivation).
  memset(peers, 0, sizeof(peers));
  n = feed(pkt, build_sample(pkt, SS_SENSOR_PROXIMITY, B, 1, 3, 200), SELF, peers, 4, ev, 32);
  CHECK(n == 1 && ev[0].sensorType == SS_SENSOR_PROXIMITY && ev[0].channel == 3 && ev[0].value == 200,
        "proximity sample surfaces");

  // 9) Temperature sample (negative centi-degrees) surfaces intact.
  n = feed(pkt, build_sample(pkt, SS_SENSOR_TEMP, B, 2, 0, -1550), SELF, peers, 4, ev, 32);
  CHECK(n == 1 && ev[0].sensorType == SS_SENSOR_TEMP && ev[0].value == -1550, "temp sample surfaces");

  // 10) Switch bitmask behaves like touch (edge-derived).
  memset(peers, 0, sizeof(peers));
  n = feed(pkt, build_mask(pkt, SS_SENSOR_SWITCH, A, 1, 0b0010), SELF, peers, 4, ev, 32);
  CHECK(n == 1 && ev[0].sensorType == SS_SENSOR_SWITCH && ev[0].channel == 1 && ev[0].value == 1,
        "switch edge");

  // 11) Unknown sensor type -> ignored, and does not advance peer seq.
  memset(peers, 0, sizeof(peers));
  n = feed(pkt, build_mask(pkt, 99, A, 1, 0b0001), SELF, peers, 4, ev, 32);
  CHECK(n == 0, "unknown sensor type ignored");
  n = feed(pkt, build_mask(pkt, SS_SENSOR_TOUCH, A, 1, 0b0001), SELF, peers, 4, ev, 32);
  CHECK(n == 1, "seq not consumed by ignored type");

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
