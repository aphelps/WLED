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
static inline uint32_t ss_ctrl_tick(ControlState &st) {
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
