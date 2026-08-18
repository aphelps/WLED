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
  const uint32_t SELF = 0x11111111, A = 0x22222222, B = 0x33333333, C = 0x44444444;
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

  // ---- clock durability across reboots (plan decision 5) ----
  //
  // Each case below is a scenario that muted a gateway under one of the rejected designs.

  // The bug this decision exists to fix: sole gateway reboots, peer still latched.
  {
    ControlState peer = ss_ctrl_init();
    ss_ctrl_should_apply(peer, 20, A, SELF);          // peer has seen clock 20 from gateway A

    ControlState gw = ss_ctrl_init();                 // A reboots: RAM clock back to 0
    SsCtrlReply replies[] = { {20, B}, {20, C} };
    gw.clock = ss_ctrl_start(0, ss_ctrl_reply_consensus(replies, 2, 0));
    CHECK(ss_ctrl_should_apply(peer, ss_ctrl_tick(gw), A, SELF),
          "rebooted sole gateway is heard again (was 0/20 accepted before)");
  }

  // A NEW node becoming sole gateway: nothing persisted. This is the case a reservation alone
  // cannot fix, and that (bootCount, lamport) actively breaks.
  {
    ControlState peer = ss_ctrl_init();
    ss_ctrl_should_apply(peer, 5000, A, SELF);

    ControlState fresh = ss_ctrl_init();
    SsCtrlReply replies[] = { {5000, A}, {5000, C} };
    fresh.clock = ss_ctrl_start(0, ss_ctrl_reply_consensus(replies, 2, 0));   // persisted = 0
    CHECK(ss_ctrl_should_apply(peer, ss_ctrl_tick(fresh), B, SELF),
          "a brand-new node with nothing persisted is accepted immediately");
  }

  // A node returning after a long outage, far enough behind that the clamp would lock it out.
  {
    ControlState peer = ss_ctrl_init();
    ss_ctrl_should_apply(peer, 9000000, A, SELF);     // mesh moved ~9M commands on

    ControlState back = ss_ctrl_init();
    back.clock = 20;                                   // its persisted ceiling, far behind
    SsCtrlReply replies[] = { {9000000, A}, {9000000, C} };
    uint32_t c = ss_ctrl_reply_consensus(replies, 2, 20);
    CHECK(c == 9000000, "a corroborated reply is believed however far ahead it is");
    back.clock = ss_ctrl_start(20, c);
    CHECK(ss_ctrl_should_apply(peer, ss_ctrl_tick(back), B, SELF),
          "a long-absent node rejoins rather than being locked out by our own clamp");

    ControlState clamped = ss_ctrl_init();
    clamped.clock = 20;
    ss_ctrl_observe(clamped, 9000000);
    CHECK(clamped.clock == 20,
          "...while an UNSOLICITED frame that far ahead is still refused (clamp intact)");
  }

  // Bypassing the clamp re-opens the poison-frame DoS on this path; corroboration replaces it.
  {
    SsCtrlReply liar[] = { {0xFFFFFFFFUL, A}, {21, B}, {22, C} };
    CHECK(ss_ctrl_reply_consensus(liar, 3, 0) == 22,
          "a lone 0xFFFFFFFF reply is discarded, the true maximum is used");

    SsCtrlReply honest[] = { {20, A}, {21, B}, {22, C} };
    CHECK(ss_ctrl_reply_consensus(honest, 3, 0) == 22,
          "honest spread costs nothing — we do not undershoot to the runner-up");

    SsCtrlReply alone[] = { {0xFFFFFFFFUL, A} };
    CHECK(ss_ctrl_reply_consensus(alone, 1, 20) == 20,
          "with nothing to corroborate against, a single wild reply falls back to the clamp rule");

    SsCtrlReply sane_alone[] = { {900, A} };
    CHECK(ss_ctrl_reply_consensus(sane_alone, 1, 20) == 900,
          "a single plausible reply is still believed");

    // NEW-A: corroboration counts SENDERS, not frames. The query broadcast announces the reply
    // window on-air, so one liar answering twice with two nearby maxima is still ONE voice and
    // falls back to the single-sender clamp rule.
    SsCtrlReply twice[] = { {0xFFFFFFFFUL, A}, {0xFFFFFFFEUL, A} };
    CHECK(ss_ctrl_reply_consensus(twice, 2, 20) == 20,
          "one sender answering twice cannot corroborate itself past the clamp");
    SsCtrlReply twice_sane[] = { {900, A}, {880, A} };
    CHECK(ss_ctrl_reply_consensus(twice_sane, 2, 20) == 900,
          "...but a repeated plausible sender still counts as one believable voice");
    SsCtrlReply mixed[] = { {0xFFFFFFFFUL, A}, {0xFFFFFFFEUL, A}, {30, B} };
    CHECK(ss_ctrl_reply_consensus(mixed, 3, 20) == 30,
          "a double-voting liar plus one honest peer: the liar is a lone outlier, honesty wins");
  }

  // No replies at all: the persisted ceiling must carry it, and must never regress.
  {
    CHECK(ss_ctrl_reply_consensus(0, 0, 500) == 0, "no replies yields no consensus value");
    // NEW-B: raise-only re-attach. Observation legitimately carries the clock past the ceiling;
    // a re-run of the query path (WiFi re-attach) must adopt, never assign, or a window that
    // closes with zero replies regresses the clock and re-mutes the node.
    {
      ControlState peer = ss_ctrl_init();
      ControlState node = ss_ctrl_init();
      node.clock = 1000;                       // resumed at its ceiling
      ss_ctrl_observe(node, 5000);             // then heard the mesh move on
      CHECK(node.clock == 5000, "observation raised the clock past the ceiling");
      ss_ctrl_should_apply(peer, 5000, A, SELF);   // a peer is latched at 5000

      // Re-attach: begin (floor to ceiling) + finish (zero replies -> consensus 0).
      ss_ctrl_adopt(node, 1000);                       // begin: raise-only floor
      ss_ctrl_adopt(node, ss_ctrl_start(1000, 0));     // finish: adopt, never assign
      CHECK(node.clock == 5000, "the re-attach did not regress what observation learned");
      CHECK(ss_ctrl_should_apply(peer, ss_ctrl_tick(node), B, SELF),
            "the latched peer accepts the node's next command after the re-attach");
    }
    // Saturating tick: the top of the clock parks rather than wrapping to 0.
    {
      ControlState st = ss_ctrl_init();
      st.clock = 0xFFFFFFFFUL;
      CHECK(ss_ctrl_tick(st) == 0xFFFFFFFFUL, "tick saturates at u32 max");
      CHECK(st.clock == 0xFFFFFFFFUL, "...and the stored clock does not wrap to 0");
    }
    CHECK(ss_ctrl_start(500, 0) == 500, "persisted ceiling is used when nobody answers");
    CHECK(ss_ctrl_start(500, 40) == 500, "and never regresses below it");
  }

  // The reservation itself.
  {
    CHECK(ss_ctrl_next_ceiling(0) == SS_CTRL_RESERVE, "first boot claims one block");
    CHECK(ss_ctrl_next_ceiling(1000) == 2000, "each claim advances by one block");
    CHECK(ss_ctrl_next_ceiling(0xFFFFFFFFUL - 10) == 0xFFFFFFFFUL,
          "saturates instead of wrapping — a wrapped clock is the mute we are preventing");
    CHECK(!ss_ctrl_needs_reserve(999, 1000), "inside the block, no flash write");
    CHECK(ss_ctrl_needs_reserve(1000, 1000), "at the ceiling, claim before publishing");
  }

  // Adoption only ever raises.
  {
    ControlState st = ss_ctrl_init();
    st.clock = 500;
    ss_ctrl_adopt(st, 400);
    CHECK(st.clock == 500, "a lower reply never drags the clock backwards");
    ss_ctrl_adopt(st, 600);
    CHECK(st.clock == 600, "a higher reply raises it");

    // The whole point of adopt() existing separately from observe(): same input, and the
    // solicited path must follow it while the unsolicited path must not. Without this, adopt()
    // could be quietly reimplemented as observe() and every test above would still pass.
    const uint32_t far = 600 + SS_CTRL_MAX_JUMP + 1;
    ControlState solicited = ss_ctrl_init();   solicited.clock = 600;
    ControlState unsolicited = ss_ctrl_init(); unsolicited.clock = 600;
    ss_ctrl_adopt(solicited, far);
    ss_ctrl_observe(unsolicited, far);
    CHECK(solicited.clock == far, "adopt() follows a jump beyond the clamp (solicited reply)");
    CHECK(unsolicited.clock == 600, "observe() still refuses the identical jump (unsolicited)");
  }

  // ---- query / reply frame parsing ----
  {
    uint8_t pkt[sizeof(SensorSyncHeader) + sizeof(SensorControlClock)];
    SensorSyncHeader h{}; SensorControlClock cl{};
    h.magic[0]='A'; h.magic[1]='M'; h.magic[2]='P'; h.magic[3]='S';
    h.version = SENSOR_SYNC_VERSION; h.deviceId = A;

    // A CTRL_CLOCK reply must not be readable as a command, and vice versa: the whole point of
    // splitting the parsers is that a clock can never reach the apply path.
    h.msgType = SENSOR_SYNC_MSG_CTRL_CLOCK; h.dataLen = sizeof(cl); cl.clock = 4242;
    memcpy(pkt, &h, sizeof(h)); memcpy(pkt + sizeof(h), &cl, sizeof(cl));

    SensorSyncHeader oh; SensorControlClock oc; SensorControl bogus;
    CHECK(ss_parse_ctrl_clock(pkt, sizeof(pkt), SELF, oh, oc), "a well-formed reply parses");
    CHECK(oc.clock == 4242, "the clock survives the round trip");
    CHECK(!ss_parse_control(pkt, sizeof(pkt), SELF, oh, bogus),
          "a CTRL_CLOCK reply is NOT accepted as a command");
    CHECK(!ss_parse_ctrl_query(pkt, sizeof(pkt), SELF, oh), "nor as a query");

    CHECK(!ss_parse_ctrl_clock(pkt, sizeof(SensorSyncHeader) + 1, SELF, oh, oc),
          "a truncated reply is rejected");
    h.deviceId = SELF; memcpy(pkt, &h, sizeof(h));
    CHECK(!ss_parse_ctrl_clock(pkt, sizeof(pkt), SELF, oh, oc), "our own reply is ignored");

    h.deviceId = A; h.msgType = SENSOR_SYNC_MSG_CTRL_QUERY; h.dataLen = 0;
    memcpy(pkt, &h, sizeof(h));
    CHECK(ss_parse_ctrl_query(pkt, sizeof(SensorSyncHeader), SELF, oh), "a query parses");
    CHECK(!ss_parse_control(pkt, sizeof(SensorSyncHeader), SELF, oh, bogus),
          "a query is not a command either");
    h.deviceId = SELF; memcpy(pkt, &h, sizeof(h));
    CHECK(!ss_parse_ctrl_query(pkt, sizeof(SensorSyncHeader), SELF, oh),
          "we never answer our own query");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
