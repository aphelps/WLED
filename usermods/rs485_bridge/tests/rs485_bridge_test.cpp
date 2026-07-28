// Host unit test for the RS485/HMTL bridge wire format + decision logic.
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure protocol header:
//   c++ -std=c++11 -Wall -Wextra -o /tmp/rs485_test rs485_bridge_test.cpp && /tmp/rs485_test
// Exits 0 on success, 1 on the first failed assertion.
//
#include "../rs485_bridge_protocol.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
// Records a failure and keeps going, so one run reports every broken assertion rather than only
// the first — much faster to work through when a wire-format change breaks several at once.
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

static const uint16_t SELF_ADDR = 0x0007;   // the address the simulated bridge listens on
static const uint16_t SELF_DEV  = 0x1234;   // its HMTL device id (SET_ADDRESS is keyed on this)

// ---------------------------------------------------------------------------------------------
// Frame builders (mirror HMTL's hmtl_*_fmt helpers)
// ---------------------------------------------------------------------------------------------
// These construct frames the way a legacy HMTL module would, independently of the header's own
// rs485b_hmtl_fmt(), so the tests exercise the parser against externally-shaped input rather than
// only against what this code itself emits. Each returns the total frame length and leaves the
// crc byte at 0 (valid — stock HMTL builds do not use CRC); call stamp_crc() to check that path.

// Write just an HMTL header. `len` is the TOTAL frame length the header will declare.
static uint8_t build_hdr(uint8_t *buf, uint16_t addr, uint8_t len, uint8_t type, uint8_t flags) {
  HmtlMsgHdr h;
  h.startcode = HMTL_MSG_START;
  h.crc = 0; h.version = HMTL_MSG_VERSION; h.length = len;
  h.type = type; h.flags = flags; h.address = addr;
  memcpy(buf, &h, sizeof(h));
  return len;
}

// Build an OUTPUT/RGB frame setting `output` to (r, g, b).
static uint8_t build_rgb(uint8_t *buf, uint16_t addr, uint8_t output,
                         uint8_t r, uint8_t g, uint8_t b) {
  const uint8_t len = sizeof(HmtlMsgHdr) + sizeof(HmtlMsgRgb);
  build_hdr(buf, addr, len, HMTL_MSG_TYPE_OUTPUT, 0);
  HmtlMsgRgb m;
  m.hdr.type = HMTL_OUTPUT_RGB; m.hdr.output = output;
  m.values[0] = r; m.values[1] = g; m.values[2] = b;
  memcpy(buf + sizeof(HmtlMsgHdr), &m, sizeof(m));
  return len;
}

// Build an OUTPUT/VALUE frame. `value` is masked to the 13 bits the wire field actually carries.
static uint8_t build_value(uint8_t *buf, uint16_t addr, uint8_t output, uint16_t value) {
  const uint8_t len = sizeof(HmtlMsgHdr) + sizeof(HmtlMsgValue);
  build_hdr(buf, addr, len, HMTL_MSG_TYPE_OUTPUT, 0);
  HmtlMsgValue m;
  memset(&m, 0, sizeof(m));
  m.hdr.type = HMTL_OUTPUT_VALUE; m.hdr.output = output;
  m.value = value & 0x1FFF; m.flags = 0;
  memcpy(buf + sizeof(HmtlMsgHdr), &m, sizeof(m));
  return len;
}

// Build an OUTPUT/PROGRAM frame.
// `tail` is how many program value bytes to actually transmit: msg_program_t is a fixed 35 bytes
// in the HMTL headers, but real senders truncate the unused tail, so the parser must cope with
// any length from zero values upward.
static uint8_t build_program(uint8_t *buf, uint16_t addr, uint8_t output, uint8_t progType,
                             const uint8_t *vals, uint8_t tail) {
  const uint8_t len = (uint8_t)(sizeof(HmtlMsgHdr) + sizeof(HmtlOutputHdr) + 1 + tail);
  build_hdr(buf, addr, len, HMTL_MSG_TYPE_OUTPUT, 0);
  HmtlOutputHdr oh; oh.type = HMTL_OUTPUT_PROGRAM; oh.output = output;
  memcpy(buf + sizeof(HmtlMsgHdr), &oh, sizeof(oh));
  buf[sizeof(HmtlMsgHdr) + sizeof(HmtlOutputHdr)] = progType;
  if (tail) memcpy(buf + sizeof(HmtlMsgHdr) + sizeof(HmtlOutputHdr) + 1, vals, tail);
  return len;
}

// Build a SET_ADDR frame. `devId` is the selector (0 == any device), `newAddr` the address to
// take on; `addr` is still the ordinary HMTL destination.
static uint8_t build_set_addr(uint8_t *buf, uint16_t addr, uint16_t devId, uint16_t newAddr) {
  const uint8_t len = sizeof(HmtlMsgHdr) + sizeof(HmtlMsgSetAddr);
  build_hdr(buf, addr, len, HMTL_MSG_TYPE_SET_ADDR, 0);
  HmtlMsgSetAddr m; m.device_id = devId; m.address = newAddr;
  memcpy(buf + sizeof(HmtlMsgHdr), &m, sizeof(m));
  return len;
}

// Build a SENSOR frame carrying exactly one sensor record.
static uint8_t build_sensor(uint8_t *buf, uint16_t addr, uint8_t sensorType,
                            const uint8_t *data, uint8_t dataLen) {
  const uint8_t len = (uint8_t)(sizeof(HmtlMsgHdr) + sizeof(HmtlSensorData) + dataLen);
  build_hdr(buf, addr, len, HMTL_MSG_TYPE_SENSOR, 0);
  HmtlSensorData s; s.sensor_type = sensorType; s.data_len = dataLen;
  memcpy(buf + sizeof(HmtlMsgHdr), &s, sizeof(s));
  memcpy(buf + sizeof(HmtlMsgHdr) + sizeof(HmtlSensorData), data, dataLen);
  return len;
}

// Fill in a frame's crc byte the way a CRC-enabled HMTL sender would.
static void stamp_crc(uint8_t *buf, uint8_t len) { buf[1] = rs485b_hmtl_crc(buf, len); }

// Run every test group in order and report. Groups are numbered and self-contained: each builds
// its own frames in the shared `buf` and asserts on the result, so they can be read (or deleted)
// independently.
int main() {
  uint8_t buf[128];   // HMTL_MAX_MSG_LEN — big enough for any legal frame

  // 1) Wire layout is what legacy HMTL modules expect.
  {
    CHECK(sizeof(HmtlMsgHdr) == 8, "msg_hdr_t is 8B");
    CHECK(sizeof(HmtlMsgRgb) == 5, "msg_rgb_t is 5B");
    CHECK(sizeof(HmtlMsgValue) == 4, "msg_value_t is 4B");
    CHECK(sizeof(HmtlMsgProgram) == 35, "msg_program_t is 35B");
    // Field offsets: length at 3, address at 6 — the bridge indexes buf[3] directly.
    HmtlMsgHdr h; memset(&h, 0, sizeof(h));
    CHECK((size_t)((uint8_t *)&h.length  - (uint8_t *)&h) == 3, "length at offset 3");
    CHECK((size_t)((uint8_t *)&h.address - (uint8_t *)&h) == 6, "address at offset 6");
  }

  // 2) CRC-8 matches ArduinoLibs' EEPROM_crc (poly 0xD8, init 0, MSB-first).
  {
    const uint8_t one[1] = { 0x00 };
    CHECK(rs485b_crc8(one, 1) == 0x00, "crc8 of 0x00 is 0");
    const uint8_t v[3] = { 0x01, 0x02, 0x03 };
    // Reference value computed by running the EEPROM_crc algorithm by hand.
    uint8_t rem = 0;
    for (int i = 0; i < 3; i++) {
      rem ^= v[i];
      for (int b = 8; b > 0; b--) rem = (rem & 0x80) ? (uint8_t)((rem << 1) ^ 0xD8)
                                                     : (uint8_t)(rem << 1);
    }
    CHECK(rs485b_crc8(v, 3) == rem, "crc8 matches the reference loop");
    CHECK(rs485b_crc8(v, 0) == 0, "crc8 of an empty buffer is 0");
  }

  // 3) rs485b_hmtl_crc ignores the crc byte itself, so stamping is idempotent.
  {
    uint8_t len = build_rgb(buf, SELF_ADDR, 0, 1, 2, 3);
    uint8_t c1 = rs485b_hmtl_crc(buf, len);
    buf[1] = c1;
    uint8_t c2 = rs485b_hmtl_crc(buf, len);
    CHECK(c1 == c2, "hmtl crc is independent of the crc field");
    CHECK(rs485b_validate(buf, len) == RS485B_OK, "stamped frame validates");
  }

  // 4) Validation rejects every malformed shape, and accepts crc == 0 (stock HMTL builds have
  //    HMTL_USE_CRC off and transmit a zero crc).
  {
    uint8_t len = build_rgb(buf, SELF_ADDR, 0, 9, 9, 9);
    CHECK(buf[1] == 0, "unstamped frame has crc 0");
    CHECK(rs485b_validate(buf, len) == RS485B_OK, "crc 0 accepted (legacy no-CRC sender)");

    CHECK(rs485b_validate(buf, sizeof(HmtlMsgHdr) - 1) == RS485B_ERR_SHORT, "short buffer");
    CHECK(rs485b_validate(nullptr, 64) == RS485B_ERR_SHORT, "null buffer");
    CHECK(rs485b_validate(buf, (uint16_t)(len - 1)) == RS485B_ERR_SHORT, "avail < hdr.length");

    uint8_t bad[16]; memcpy(bad, buf, len);
    bad[0] = 0x00;
    CHECK(rs485b_validate(bad, len) == RS485B_ERR_START, "bad start code");

    memcpy(bad, buf, len); bad[2] = 99;
    CHECK(rs485b_validate(bad, len) == RS485B_ERR_VERSION, "bad version");

    memcpy(bad, buf, len); bad[3] = 3;
    CHECK(rs485b_validate(bad, len) == RS485B_ERR_LENGTH, "hdr.length below the header size");

    memcpy(bad, buf, len); bad[3] = HMTL_MAX_MSG_LEN + 1;
    CHECK(rs485b_validate(bad, 200) == RS485B_ERR_LENGTH, "hdr.length above HMTL_MAX_MSG_LEN");

    memcpy(bad, buf, len); stamp_crc(bad, len); bad[4] ^= 0xFF;   // flip the type byte
    CHECK(rs485b_validate(bad, len) == RS485B_ERR_CRC, "corrupted body fails the crc");
  }

  // 5) Addressing: exact match or broadcast.
  {
    CHECK(rs485b_addressed_to(SELF_ADDR, SELF_ADDR), "exact address matches");
    CHECK(rs485b_addressed_to(HMTL_ADDR_BROADCAST, SELF_ADDR), "broadcast matches");
    CHECK(!rs485b_addressed_to(0x0099, SELF_ADDR), "other address does not match");
  }

  // 6) RGB command addressed to us.
  {
    uint8_t len = build_rgb(buf, SELF_ADDR, 0, 0x11, 0x22, 0x33);
    stamp_crc(buf, len);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_RGB, "rgb -> SET_RGB");
    CHECK(d.rgb[0] == 0x11 && d.rgb[1] == 0x22 && d.rgb[2] == 0x33, "rgb values decoded");
    CHECK(d.output == 0, "output index decoded");
  }

  // 7) Broadcast RGB is also ours; a frame for another node is relay-only.
  {
    uint8_t len = build_rgb(buf, HMTL_ADDR_BROADCAST, HMTL_ALL_OUTPUTS, 1, 2, 3);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_RGB, "broadcast rgb is handled locally");
    CHECK(d.output == HMTL_ALL_OUTPUTS, "ALL_OUTPUTS preserved");

    len = build_rgb(buf, 0x0099, 0, 1, 2, 3);
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_RELAY_ONLY, "frame for another node -> relay only");
    CHECK(d.destAddress == 0x0099, "dest address reported for relaying");
  }

  // 8) VALUE command, including the 13-bit field boundary.
  {
    uint8_t len = build_value(buf, SELF_ADDR, 0, 8191);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_VALUE, "value -> SET_VALUE");
    CHECK(d.value == 8191, "13-bit value survives the round trip");

    len = build_value(buf, SELF_ADDR, 0, 0);
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_VALUE && d.value == 0, "zero value decoded");
  }

  // 9) PROGRAM commands: mapped, unmapped, and truncated payloads.
  {
    const uint8_t vals[3] = { 0xAA, 0xBB, 0xCC };
    uint8_t len = build_program(buf, SELF_ADDR, 0, HMTL_PROGRAM_SPARKLE, vals, 3);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_PROGRAM, "program -> PROGRAM");
    CHECK(d.programType == HMTL_PROGRAM_SPARKLE, "program type decoded");
    CHECK(d.programLen == 3 && d.programVals[0] == 0xAA, "program payload decoded");
    CHECK(rs485b_program_to_mode(d.programType) == RS485B_WLED_MODE_SPARKLE, "sparkle mapped");

    // A program with no values at all is still well-formed.
    len = build_program(buf, SELF_ADDR, 0, HMTL_PROGRAM_BLINK, nullptr, 0);
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_PROGRAM && d.programLen == 0, "valueless program accepted");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_BLINK) == RS485B_WLED_MODE_BLINK, "blink mapped");

    // Unmapped programs are reported so the caller can count-and-ignore them.
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_SOUND_VALUE) == -1, "sound_value unmapped");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_SEQUENCE) == -1, "sequence unmapped");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_BRIGHTNESS) == -1, "brightness is not a mode");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_NONE) == RS485B_WLED_MODE_STATIC, "none -> static");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_FADE) == RS485B_WLED_MODE_FADE, "fade mapped");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_CIRCULAR) == RS485B_WLED_MODE_CHASE, "circ mapped");
  }

  // 10) An OUTPUT frame whose payload is too short for its declared output type is unsupported,
  //     never a buffer over-read.
  {
    uint8_t len = (uint8_t)(sizeof(HmtlMsgHdr) + 1);
    build_hdr(buf, SELF_ADDR, len, HMTL_MSG_TYPE_OUTPUT, 0);
    buf[sizeof(HmtlMsgHdr)] = HMTL_OUTPUT_RGB;
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "output header truncated -> unsupported");

    // Full output header but a truncated RGB body.
    len = (uint8_t)(sizeof(HmtlMsgHdr) + sizeof(HmtlOutputHdr) + 1);
    build_hdr(buf, SELF_ADDR, len, HMTL_MSG_TYPE_OUTPUT, 0);
    HmtlOutputHdr oh; oh.type = HMTL_OUTPUT_RGB; oh.output = 0;
    memcpy(buf + sizeof(HmtlMsgHdr), &oh, sizeof(oh));
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "truncated rgb body -> unsupported");

    // Unknown output type.
    len = build_rgb(buf, SELF_ADDR, 0, 1, 2, 3);
    buf[sizeof(HmtlMsgHdr)] = HMTL_OUTPUT_XBEE;
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "unknown output type -> unsupported");
  }

  // 11) POLL is answered; the RESPONSE flag is surfaced.
  {
    uint8_t len = build_hdr(buf, SELF_ADDR, sizeof(HmtlMsgHdr), HMTL_MSG_TYPE_POLL,
                            HMTL_MSG_FLAG_RESPONSE);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_POLL, "poll -> POLL");
    CHECK(d.wantsResponse, "RESPONSE flag decoded");
  }

  // 12) SET_ADDRESS honours the device-id filter.
  {
    uint8_t len = build_set_addr(buf, SELF_ADDR, SELF_DEV, 0x0042);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_ADDRESS, "matching device id -> SET_ADDRESS");
    CHECK(d.newAddress == 0x0042, "new address decoded");

    len = build_set_addr(buf, SELF_ADDR, 0, 0x0043);
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SET_ADDRESS, "device id 0 means any device");

    len = build_set_addr(buf, SELF_ADDR, 0x9999, 0x0044);
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "other device id is not us");
  }

  // 13) SENSOR payloads iterate cleanly and never run past the buffer.
  {
    const uint8_t payload[2] = { 0x10, 0x20 };
    uint8_t len = build_sensor(buf, HMTL_ADDR_BROADCAST, HMTL_SENSOR_SOUND, payload, 2);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SENSOR, "sensor -> SENSOR");
    CHECK(d.sensorLen == sizeof(HmtlSensorData) + 2, "sensor payload length");

    uint8_t type = 0, dlen = 0;
    const uint8_t *dptr = nullptr;
    uint8_t used = rs485b_next_sensor(d.sensorData, d.sensorLen, 0, &type, &dptr, &dlen);
    CHECK(used == sizeof(HmtlSensorData) + 2, "one sensor record consumed");
    CHECK(type == HMTL_SENSOR_SOUND && dlen == 2 && dptr[0] == 0x10, "sensor record decoded");
    CHECK(rs485b_next_sensor(d.sensorData, d.sensorLen, used, &type, &dptr, &dlen) == 0,
          "no second record");

    // A record whose declared data_len runs past the payload is refused.
    uint8_t trunc[4] = { HMTL_SENSOR_LIGHT, 200, 0, 0 };
    CHECK(rs485b_next_sensor(trunc, 4, 0, &type, &dptr, &dlen) == 0, "over-long record refused");
  }

  // 14) Unknown message types are counted, not acted on.
  {
    uint8_t len = build_hdr(buf, SELF_ADDR, sizeof(HmtlMsgHdr), HMTL_MSG_TYPE_TIMESYNC, 0);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "timesync is outside the v1 set");
  }

  // 15) A malformed frame decides to DROP and reports why.
  {
    uint8_t len = build_rgb(buf, SELF_ADDR, 0, 1, 2, 3);
    buf[0] = 0x00;
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_DROP && d.err == RS485B_ERR_START, "drop reports the reason");
  }

  // 16) UDP ingress rejects anything the socket send buffer cannot carry.
  {
    uint8_t len = build_rgb(buf, 0x0005, 0, 1, 2, 3);
    CHECK(rs485b_validate_udp_ingress(buf, len, RS485B_TX_SLOT_LEN) == RS485B_OK,
          "small frame accepted for forwarding");
    // Declare a frame longer than the socket data buffer.
    uint8_t big[128];
    build_hdr(big, 0x0005, 100, HMTL_MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(HmtlMsgHdr), 0, 100 - sizeof(HmtlMsgHdr));
    CHECK(rs485b_validate_udp_ingress(big, 100, RS485B_TX_SLOT_LEN) == RS485B_ERR_OVERSIZE,
          "oversized frame rejected before sendMsgTo");
    // Exactly at the limit is fine.
    build_hdr(big, 0x0005, RS485B_TX_SLOT_LEN, HMTL_MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(HmtlMsgHdr), 0, RS485B_TX_SLOT_LEN - sizeof(HmtlMsgHdr));
    CHECK(rs485b_validate_udp_ingress(big, RS485B_TX_SLOT_LEN, RS485B_TX_SLOT_LEN) == RS485B_OK,
          "frame exactly at the buffer limit accepted");
    // A datagram that claims more than it delivers is short, not oversize.
    build_hdr(big, 0x0005, 40, HMTL_MSG_TYPE_OUTPUT, 0);
    CHECK(rs485b_validate_udp_ingress(big, 20, RS485B_TX_SLOT_LEN) == RS485B_ERR_SHORT,
          "truncated datagram rejected");
  }

  // 17) TX queue: FIFO order, bounded, drop-oldest, never over-runs a slot.
  {
    RS485BTxQueue q;
    uint8_t frame[RS485B_TX_SLOT_LEN];
    memset(frame, 0, sizeof(frame));
    uint16_t dest = 0; uint8_t outLen = 0;
    uint8_t out[RS485B_TX_SLOT_LEN];

    CHECK(q.empty(), "queue starts empty");
    CHECK(!q.pop(&dest, out, &outLen), "pop on an empty queue fails");

    for (uint8_t i = 0; i < RS485B_TX_SLOTS; i++) {
      frame[0] = i;
      CHECK(q.push((uint16_t)(100 + i), frame, 8), "push within capacity keeps everything");
    }
    CHECK(q.full(), "queue reports full");

    frame[0] = 0xEE;
    CHECK(!q.push(999, frame, 8), "push on a full queue reports the drop");

    // The oldest entry (i == 0) was discarded; the rest kept their order.
    CHECK(q.pop(&dest, out, &outLen) && dest == 101 && out[0] == 1, "oldest dropped, FIFO kept");
    CHECK(q.pop(&dest, out, &outLen) && dest == 102 && out[0] == 2, "FIFO order 2");
    CHECK(q.pop(&dest, out, &outLen) && dest == 103 && out[0] == 3, "FIFO order 3");
    CHECK(q.pop(&dest, out, &outLen) && dest == 999 && out[0] == 0xEE, "newest survived");
    CHECK(q.empty(), "queue drained");

    // Oversized / degenerate pushes are refused outright, leaving the queue untouched.
    CHECK(!q.push(1, frame, RS485B_TX_SLOT_LEN + 1), "over-long frame refused");
    CHECK(!q.push(1, frame, 0), "zero-length frame refused");
    CHECK(!q.push(1, nullptr, 4), "null frame refused");
    CHECK(q.empty(), "refused pushes did not enqueue");

    // Wrap-around: push/pop far more than the capacity.
    for (uint8_t i = 0; i < 20; i++) {
      frame[0] = i;
      q.push(i, frame, 4);
      CHECK(q.pop(&dest, out, &outLen) && dest == i && outLen == 4, "wrap-around round trip");
    }
    CHECK(q.empty(), "queue empty after wrap-around");
  }

  // 18) Counters classify each failure into its own bucket.
  {
    RS485BCounters c;
    c.countError(RS485B_ERR_SHORT);
    c.countError(RS485B_ERR_CRC);
    c.countError(RS485B_ERR_CRC);
    c.countError(RS485B_ERR_OVERSIZE);
    c.countError(RS485B_OK);
    CHECK(c.errShort == 1 && c.errCrc == 2 && c.errOversize == 1, "errors bucketed");
    CHECK(c.errStart == 0 && c.errVersion == 0 && c.errLength == 0, "unrelated buckets untouched");
  }

  // 19) rs485b_hmtl_fmt produces a frame that validates, and refuses bad sizes.
  {
    uint8_t out[64];
    uint8_t len = rs485b_hmtl_fmt(out, sizeof(out), 0x0021, sizeof(HmtlMsgHdr),
                                  HMTL_MSG_TYPE_POLL, HMTL_MSG_FLAG_ACK);
    CHECK(len == sizeof(HmtlMsgHdr), "fmt returns the frame length");
    CHECK(rs485b_validate(out, len) == RS485B_OK, "formatted frame validates");
    HmtlMsgHdr h; memcpy(&h, out, sizeof(h));
    CHECK(h.address == 0x0021 && h.type == HMTL_MSG_TYPE_POLL, "fmt fields land");
    CHECK(rs485b_hmtl_fmt(out, sizeof(out), 0, 3, HMTL_MSG_TYPE_POLL, 0) == 0,
          "fmt refuses a sub-header length");
    CHECK(rs485b_hmtl_fmt(out, 4, 0, sizeof(HmtlMsgHdr), HMTL_MSG_TYPE_POLL, 0) == 0,
          "fmt refuses a too-small buffer");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
