// Host unit test for the control plane: frame validation, the conflict/ordering rule, and echo
// suppression.
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure headers:
//   c++ -std=c++11 -Wall -o /tmp/ss_ctrl_test sensor_control_test.cpp && /tmp/ss_ctrl_test
// Exits 0 on success, 1 on the first failed assertion.
//
#include "../sensor_control.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

// Serialize a control message into buf; returns total length.
static int build_ctrl(uint8_t *buf, uint32_t dev, uint16_t seq, uint32_t lamport,
                      uint8_t fields, uint8_t presetId, uint8_t brightness) {
  SensorSyncHeader h{};
  memcpy(h.magic, "AMPS", 4);
  h.version = SENSOR_SYNC_VERSION; h.msgType = SENSOR_SYNC_MSG_CONTROL;
  h.sensorType = 0; h.dataLen = sizeof(SensorControl);
  h.deviceId = dev; h.seq = seq; h.timestamp = 1234; h.ttl = SS_DEFAULT_TTL;
  SensorControl c{};
  c.fields = fields; c.presetId = presetId; c.brightness = brightness; c.lamport = lamport;
  memcpy(buf, &h, sizeof(h)); memcpy(buf + sizeof(h), &c, sizeof(c));
  return (int)(sizeof(h) + sizeof(c));
}

int main() {
  const uint32_t SELF = 0x11111111, A = 0x22222222, B = 0x33333333;
  uint8_t pkt[128];

  // ---- wire size: the whole point of the compact-command decision ----
  {
    CHECK(sizeof(SensorControl) == 16, "SensorControl is 16 bytes");
    CHECK(sizeof(SensorSyncHeader) + sizeof(SensorControl) <= 20 + 64,
          "control frame fits the 64-byte payload budget");
    // Guards the reason the budget exists at all.
    CHECK(sizeof(SensorSyncHeader) + sizeof(SensorControl) <= 250,
          "control frame fits an ESP-NOW datagram");
    // The u32s must stay 4-aligned inside the struct or a packed copy costs us on ESP32.
    SensorControl c{};
    CHECK(((const uint8_t*)&c.colour   - (const uint8_t*)&c) % 4 == 0, "colour is 4-aligned");
    CHECK(((const uint8_t*)&c.lamport  - (const uint8_t*)&c) % 4 == 0, "lamport is 4-aligned");
  }

  // ---- parse/validation ----
  {
    SensorSyncHeader h; SensorControl c;
    int len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 7, 0);
    CHECK(ss_parse_control(pkt, len, SELF, h, c), "well-formed control accepted");
    CHECK(c.presetId == 7, "payload round-trips");
    CHECK(h.deviceId == A, "header round-trips");

    CHECK(!ss_parse_control(0, len, SELF, h, c), "null packet rejected");
    CHECK(!ss_parse_control(pkt, (int)sizeof(SensorSyncHeader) - 1, SELF, h, c),
          "short-of-header rejected");

    // truncated payload: header claims a full struct, buffer stops early
    CHECK(!ss_parse_control(pkt, len - 1, SELF, h, c), "truncated payload rejected");

    // our own command must not parse as inbound
    int lenSelf = build_ctrl(pkt, SELF, 1, 5000, SS_CTRL_PRESET, 7, 0);
    CHECK(!ss_parse_control(pkt, lenSelf, SELF, h, c), "own deviceId rejected");

    // wrong msgType — a snapshot must not be mistaken for a command
    len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 7, 0);
    SensorSyncHeader bad; memcpy(&bad, pkt, sizeof(bad));
    bad.msgType = SENSOR_SYNC_MSG_SNAPSHOT; memcpy(pkt, &bad, sizeof(bad));
    CHECK(!ss_parse_control(pkt, len, SELF, h, c), "snapshot msgType rejected");

    // wrong version
    len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 7, 0);
    memcpy(&bad, pkt, sizeof(bad)); bad.version = SENSOR_SYNC_VERSION + 1;
    memcpy(pkt, &bad, sizeof(bad));
    CHECK(!ss_parse_control(pkt, len, SELF, h, c), "wrong version rejected");

    // wrong magic
    len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 7, 0);
    pkt[0] = 'X';
    CHECK(!ss_parse_control(pkt, len, SELF, h, c), "wrong magic rejected");

    // dataLen too small to hold a SensorControl
    len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 7, 0);
    memcpy(&bad, pkt, sizeof(bad)); bad.dataLen = sizeof(SensorControl) - 1;
    memcpy(pkt, &bad, sizeof(bad));
    CHECK(!ss_parse_control(pkt, len, SELF, h, c), "undersized dataLen rejected");
  }

  // ---- forward compatibility: a newer node's larger payload must still apply ----
  {
    SensorSyncHeader h; SensorControl c;
    int len = build_ctrl(pkt, A, 1, 5000, SS_CTRL_PRESET, 9, 0);
    SensorSyncHeader big; memcpy(&big, pkt, sizeof(big));
    big.dataLen = sizeof(SensorControl) + 8;          // a future, longer struct
    memcpy(pkt, &big, sizeof(big));
    memset(pkt + sizeof(big) + sizeof(SensorControl), 0xAB, 8);
    len += 8;
    CHECK(ss_parse_control(pkt, len, SELF, h, c), "oversized payload accepted (forward compat)");
    CHECK(c.presetId == 9, "known prefix still decoded from an oversized payload");

    // unknown field bits are ignored, not rejected
    len = build_ctrl(pkt, A, 2, 5001, SS_CTRL_PRESET | 0x80, 4, 0);
    CHECK(ss_parse_control(pkt, len, SELF, h, c), "unknown field bit accepted");
    CHECK((c.fields & SS_CTRL_PRESET) != 0, "known field bit survives an unknown one");
  }

  // ---- ordering rule: must be a TOTAL order ----
  {
    CHECK(ss_ctrl_newer(100, A, 50, A), "higher clock wins");
    CHECK(!ss_ctrl_newer(50, A, 100, A), "lower clock loses");
    CHECK(!ss_ctrl_newer(100, A, 100, A), "identical command is not newer than itself");
    CHECK(ss_ctrl_newer(100, B, 100, A), "tie broken by higher deviceId");
    CHECK(!ss_ctrl_newer(100, A, 100, B), "tie-break is antisymmetric");

    // TRANSITIVITY across the full u32 range. This is the property a wraparound-style compare
    // does NOT have, and its absence is not academic: without it two nodes fed the same commands
    // in different orders settle on different states permanently. The values below are exactly
    // the counterexample that broke the previous timestamp-based rule (0, 2^30, 2^31).
    const uint32_t x = 0, y = 0x40000000UL, z = 0x80000000UL;
    CHECK(ss_ctrl_newer(y, A, x, A) && ss_ctrl_newer(z, A, y, A) && ss_ctrl_newer(z, A, x, A),
          "ordering is transitive across the full u32 range");
    // ...and no pair is ever mutually not-newer (the old rule stalemated at exactly 2^31 apart).
    CHECK(ss_ctrl_newer(z, A, x, A) || ss_ctrl_newer(x, A, z, A),
          "no mutual stalemate between distinct clocks");

    // Antisymmetry over a spread of pairs: exactly one direction holds for any two distinct keys.
    const uint32_t clocks[] = { 0, 1, 0x7FFFFFFFUL, 0x80000000UL, 0xFFFFFFFFUL };
    for (unsigned i = 0; i < 5; i++)
      for (unsigned j = 0; j < 5; j++) {
        if (i == j) continue;
        bool ij = ss_ctrl_newer(clocks[i], A, clocks[j], A);
        bool ji = ss_ctrl_newer(clocks[j], A, clocks[i], A);
        CHECK(ij != ji, "exactly one direction of newer() holds for distinct clocks");
      }
  }

  // ---- Lamport clock mechanics ----
  {
    ControlState st = ss_ctrl_init();
    CHECK(ss_ctrl_tick(st) == 1, "first tick is 1");
    CHECK(ss_ctrl_tick(st) == 2, "clock increments");

    // Hearing a higher clock raises ours, so our next command sorts after it.
    ss_ctrl_observe(st, 100);
    CHECK(ss_ctrl_tick(st) == 101, "clock catches up to what was heard");
    ss_ctrl_observe(st, 5);
    CHECK(ss_ctrl_tick(st) == 102, "a lower heard clock does not move us backwards");

    // A REJECTED command must still advance our clock, or a node that lost a race keeps issuing
    // commands that lose, and a rebooted node (clock 0) never catches up.
    ControlState st2 = ss_ctrl_init();
    ss_ctrl_should_apply(st2, 500, A, SELF);          // applied
    ss_ctrl_should_apply(st2, 400, B, SELF);          // rejected as older
    CHECK(ss_ctrl_tick(st2) == 501, "rejected command still advanced the clock");

    // Rebooted node: clock back to 0, but one heard command is enough to catch up.
    ControlState fresh = ss_ctrl_init();
    ss_ctrl_should_apply(fresh, 9000, A, SELF);
    CHECK(ss_ctrl_tick(fresh) > 9000, "rebooted node catches up after hearing one command");
  }

  // ---- apply decision + echo suppression ----
  {
    ControlState st = ss_ctrl_init();
    CHECK(!st.have, "fresh state has applied nothing");

    CHECK(ss_ctrl_should_apply(st, 1000, A, SELF), "first command applies");
    CHECK(st.have && st.lastClock == 1000 && st.lastDev == A, "state records the winner");

    CHECK(!ss_ctrl_should_apply(st, 1000, A, SELF), "exact duplicate does not re-apply");
    CHECK(!ss_ctrl_should_apply(st, 999, A, SELF), "older command does not apply");
    CHECK(ss_ctrl_should_apply(st, 1001, A, SELF), "newer command applies");

    // echo suppression: our own command must never be applied, even though it is newest
    CHECK(!ss_ctrl_should_apply(st, 99999, SELF, SELF), "own echo suppressed");
    CHECK(st.lastClock == 1001, "suppressed echo did not disturb state");

    // a rejected command must not advance state, or the next legitimate one gets swallowed
    ControlState st2 = ss_ctrl_init();
    CHECK(ss_ctrl_should_apply(st2, 500, A, SELF), "baseline applies");
    CHECK(!ss_ctrl_should_apply(st2, 400, B, SELF), "older from another gateway rejected");
    CHECK(st2.lastClock == 500 && st2.lastDev == A, "rejection left state untouched");
    CHECK(ss_ctrl_should_apply(st2, 501, B, SELF), "next newer command still applies");
  }

  // ---- the originator competes in the same order it imposes on everyone else ----
  {
    // Regression: publishControl once applied locally WITHOUT recording, leaving have=false. A
    // stale command arriving next was then accepted by the originator and rejected by every peer.
    ControlState orig = ss_ctrl_init(), peer = ss_ctrl_init();
    const uint32_t clk = ss_ctrl_tick(orig);
    ss_ctrl_record_own(orig, clk, SELF);      // what publishControl does
    ss_ctrl_should_apply(peer, clk, SELF, B); // the peer receives it

    CHECK(!ss_ctrl_should_apply(orig, 0, A, SELF), "originator rejects a stale command");
    CHECK(!ss_ctrl_should_apply(peer, 0, A, B),    "peer rejects the same stale command");
    CHECK(orig.lastDev == peer.lastDev && orig.lastClock == peer.lastClock,
          "originator and peer agree on the winner");
  }

  // ---- three commands, every arrival order, must converge (needs transitivity) ----
  {
    // Two commands can converge even under a non-transitive rule; three is where it breaks.
    struct Cmd { uint32_t clock, dev; };
    const Cmd cmds[3] = { {0, A}, {0x40000000UL, A}, {0x80000000UL, A} };
    const int orders[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    uint32_t wonClock = 0, wonDev = 0;
    for (int o = 0; o < 6; o++) {
      ControlState st = ss_ctrl_init();
      for (int k = 0; k < 3; k++)
        ss_ctrl_should_apply(st, cmds[orders[o][k]].clock, cmds[orders[o][k]].dev, SELF);
      if (o == 0) { wonClock = st.lastClock; wonDev = st.lastDev; }
      CHECK(st.lastClock == wonClock && st.lastDev == wonDev,
            "all six arrival orders converge on the same winner");
    }
  }

  // ---- two gateways converge on the same winner regardless of arrival order ----
  {
    // Same two commands, opposite arrival orders, on two different nodes.
    ControlState n1 = ss_ctrl_init(), n2 = ss_ctrl_init();
    ss_ctrl_should_apply(n1, 2000, A, SELF);
    ss_ctrl_should_apply(n1, 2001, B, SELF);

    ss_ctrl_should_apply(n2, 2001, B, SELF);
    ss_ctrl_should_apply(n2, 2000, A, SELF);   // arrives late, must lose

    CHECK(n1.lastClock == n2.lastClock && n1.lastDev == n2.lastDev,
          "nodes converge on the same command despite different arrival order");
    CHECK(n1.lastDev == B, "the genuinely newer command is the one that stuck");
  }

  // ---- simultaneous commands from two gateways converge by tie-break ----
  {
    ControlState n1 = ss_ctrl_init(), n2 = ss_ctrl_init();
    ss_ctrl_should_apply(n1, 3000, A, SELF);
    ss_ctrl_should_apply(n1, 3000, B, SELF);

    ss_ctrl_should_apply(n2, 3000, B, SELF);
    ss_ctrl_should_apply(n2, 3000, A, SELF);

    CHECK(n1.lastDev == n2.lastDev, "same-millisecond commands still converge");
    CHECK(n1.lastDev == B, "higher deviceId is the agreed winner");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
