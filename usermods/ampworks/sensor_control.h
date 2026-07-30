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

// Wraparound-safe "is a strictly newer than b" for the header's 32-bit timestamp, matching what
// ss_seq_newer does for 16-bit sequence numbers. Without this a timestamp rolling through zero
// would make every subsequent command look older than the last one applied, and the installation
// would stop responding until it caught back up.
static inline bool ss_ts_newer(uint32_t a, uint32_t b) {
  return a != b && (uint32_t)(a - b) < 0x80000000UL;
}

// Total order over commands, so that two gateways issuing conflicting commands converge on the
// same winner rather than fighting. Primary key is the leader-owned timestamp; deviceId breaks
// ties, which matters because two phones acting in the same millisecond is not exotic when a
// timestamp has millisecond resolution. Deterministic and stateless: every node picks the same
// winner without agreeing on anything beyond the frames themselves.
static inline bool ss_ctrl_newer(uint32_t aTs, uint32_t aDev, uint32_t bTs, uint32_t bDev) {
  if (aTs != bTs) return ss_ts_newer(aTs, bTs);
  return aDev > bDev;
}

// What a node remembers about the last command it applied. Two u32s and a flag — no per-origin
// table, because the order is total: only the current winner needs remembering.
struct ControlState {
  uint32_t lastTs;
  uint32_t lastDev;
  bool     have;
};

static inline ControlState ss_ctrl_init() {
  return ControlState{ 0, 0, false };
}

// Should this command be applied? Rejects, in order:
//   - our own command coming back to us. The router's dedup stops a frame looping through the
//     backbone, but nothing stops an edge that re-broadcasts what it applied from re-originating
//     it, so the edge has to decline to apply its own echo.
//   - anything not strictly newer than what we already applied, which is what makes a late or
//     duplicated frame from a slower path harmless.
// Updates `st` only when it returns true.
static inline bool ss_ctrl_should_apply(ControlState &st, uint32_t ts, uint32_t dev,
                                        uint32_t selfId) {
  if (dev == selfId) return false;
  if (st.have && !ss_ctrl_newer(ts, dev, st.lastTs, st.lastDev)) return false;
  st.lastTs  = ts;
  st.lastDev = dev;
  st.have    = true;
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
