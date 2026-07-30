// Host unit test for the RS485/HMTL bridge wire format + decision logic.
//
// Builds with a normal host compiler — no Arduino, no WLED, no PlatformIO. Run from the
// super-repo root:
//
//   c++ -std=c++11 -Wall -Wextra
//       -I HMTL/Libraries/HMTLprotocol -I ArduinoLibs/Socket
//       -o /tmp/rs485_test WLED/usermods/rs485_bridge/tests/rs485_bridge_test.cpp
//     && /tmp/rs485_test
//
// (No trailing backslashes above: a line continuation inside a // comment is what -Wcomment
// warns about, and this file is compiled warning-free under clang and g++ >= 9 alike.)
//
// The two -I flags are the *real* HMTL and ArduinoLibs headers: this test is therefore also the
// acceptance check that HMTLWireFormat.h is genuinely dependency-free. Any Arduino.h or
// RS485Utils.h leaking into it would fail this compile immediately.
//
// Run it twice — plain, and with -fpack-struct=1 for the AVR-like layout — plus once under a
// g++ >= 9 (clang and GCC disagree about which portability mistakes are errors). Every wire size
// and offset must come out the same; that is the cross-ABI guarantee HMTLWireFormat.h's packed
// structs exist to provide.
//
// Exits 0 on success, 1 if any assertion failed.
//
#include "../rs485_bridge_protocol.h"
#include <cstddef>   // offsetof — used to pin every wire struct's field offsets, see group 1
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
  msg_hdr_t h;
  h.startcode = HMTL_MSG_START;
  h.crc = 0; h.version = HMTL_MSG_VERSION; h.length = len;
  h.type = type; h.flags = flags; h.address = addr;
  memcpy(buf, &h, sizeof(h));
  return len;
}

// Build an OUTPUT/RGB frame setting `output` to (r, g, b).
static uint8_t build_rgb(uint8_t *buf, uint16_t addr, uint8_t output,
                         uint8_t r, uint8_t g, uint8_t b) {
  const uint8_t len = sizeof(msg_hdr_t) + sizeof(msg_rgb_t);
  build_hdr(buf, addr, len, MSG_TYPE_OUTPUT, 0);
  msg_rgb_t m;
  m.hdr.type = HMTL_OUTPUT_RGB; m.hdr.output = output;
  m.values[0] = r; m.values[1] = g; m.values[2] = b;
  memcpy(buf + sizeof(msg_hdr_t), &m, sizeof(m));
  return len;
}

// Build an OUTPUT/VALUE frame. `value` is masked to the 13 bits the wire field actually carries.
static uint8_t build_value(uint8_t *buf, uint16_t addr, uint8_t output, uint16_t value) {
  const uint8_t len = sizeof(msg_hdr_t) + sizeof(msg_value_t);
  build_hdr(buf, addr, len, MSG_TYPE_OUTPUT, 0);
  msg_value_t m;
  memset(&m, 0, sizeof(m));
  m.hdr.type = HMTL_OUTPUT_VALUE; m.hdr.output = output;
  m.value = value & 0x1FFF; m.flags = 0;
  memcpy(buf + sizeof(msg_hdr_t), &m, sizeof(m));
  return len;
}

// Build an OUTPUT/PROGRAM frame.
// `tail` is how many program value bytes to actually transmit: msg_program_t is a fixed 35 bytes
// in the HMTL headers, but real senders truncate the unused tail, so the parser must cope with
// any length from zero values upward.
static uint8_t build_program(uint8_t *buf, uint16_t addr, uint8_t output, uint8_t progType,
                             const uint8_t *vals, uint8_t tail) {
  const uint8_t len = (uint8_t)(sizeof(msg_hdr_t) + sizeof(output_hdr_t) + 1 + tail);
  build_hdr(buf, addr, len, MSG_TYPE_OUTPUT, 0);
  output_hdr_t oh; oh.type = HMTL_OUTPUT_PROGRAM; oh.output = output;
  memcpy(buf + sizeof(msg_hdr_t), &oh, sizeof(oh));
  buf[sizeof(msg_hdr_t) + sizeof(output_hdr_t)] = progType;
  if (tail) memcpy(buf + sizeof(msg_hdr_t) + sizeof(output_hdr_t) + 1, vals, tail);
  return len;
}

// Build a SET_ADDR frame. `devId` is the selector (0 == any device), `newAddr` the address to
// take on; `addr` is still the ordinary HMTL destination.
static uint8_t build_set_addr(uint8_t *buf, uint16_t addr, uint16_t devId, uint16_t newAddr) {
  const uint8_t len = sizeof(msg_hdr_t) + sizeof(msg_set_addr_t);
  build_hdr(buf, addr, len, MSG_TYPE_SET_ADDR, 0);
  msg_set_addr_t m; m.device_id = devId; m.address = newAddr;
  memcpy(buf + sizeof(msg_hdr_t), &m, sizeof(m));
  return len;
}

// Build a SENSOR frame carrying exactly one sensor record.
static uint8_t build_sensor(uint8_t *buf, uint16_t addr, uint8_t sensorType,
                            const uint8_t *data, uint8_t dataLen) {
  const uint8_t len = (uint8_t)(sizeof(msg_hdr_t) + sizeof(msg_sensor_data_t) + dataLen);
  build_hdr(buf, addr, len, MSG_TYPE_SENSOR, 0);
  // msg_sensor_data_t ends in a flexible array member, so its two fixed fields are written by
  // hand rather than through a local of that type.
  buf[sizeof(msg_hdr_t)]     = sensorType;   // msg_sensor_data_t::sensor_type
  buf[sizeof(msg_hdr_t) + 1] = dataLen;      // msg_sensor_data_t::data_len
  memcpy(buf + sizeof(msg_hdr_t) + sizeof(msg_sensor_data_t), data, dataLen);
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
    // The RS485 socket header wraps every HMTL frame on the wire, and its size is what places the
    // payload at both ends. It is 7 bytes because RS485Utils packs the struct; unpacked it would be 7
    // on AVR and 8 on this host, and a bridge built with 8 misparses every frame a real ATMega328
    // module sends (and vice versa).
    //
    // This file cannot include RS485Utils.h (it pulls in Arduino.h), so the check is against a
    // replica of the declaration rather than the struct itself. Be precise about what that does and
    // does not prove:
    //   * it DOES prove that this field list, packed, is 7 bytes with these offsets under BOTH the
    //     native and the -fpack-struct=1 (AVR-like) ABI — which is the cross-ABI claim the wire
    //     format depends on, and it fails if anyone edits the replica or the constant out of step
    //   * it does NOT observe the real rs485_socket_hdr_t. If a field is added to the real struct,
    //     this replica goes stale and stays green. The only check tying the constant to the real
    //     declaration is the static_assert at usermod_rs485_bridge.cpp:19, and that compiles ONLY in
    //     [env:ampworks] — an env no CI workflow builds — so it is a local guard, not an automated
    //     one. Keep them in step by hand.
    // (An earlier version of this asserted `RS485B_SOCKET_HDR_LEN == 7` on its own. That is 7 == 7:
    // identical under both ABIs and green even with the packing attribute deleted. Caught in review.)
    struct __attribute__((__packed__)) socketHdrReplica {
      uint8_t  ID;
      uint8_t  length;
      uint16_t source;    // socket_addr_t
      uint16_t address;   // socket_addr_t
      uint8_t  flags;
    };
    CHECK(sizeof(socketHdrReplica) == RS485B_SOCKET_HDR_LEN,
          "RS485 socket header is RS485B_SOCKET_HDR_LEN bytes under this ABI");
    CHECK(RS485B_SOCKET_HDR_LEN == 7, "RS485 socket header is 7B (the AVR layout) on every target");
    CHECK(offsetof(socketHdrReplica, ID)      == 0, "socket hdr ID at 0");
    CHECK(offsetof(socketHdrReplica, length)  == 1, "socket hdr length at 1");
    CHECK(offsetof(socketHdrReplica, source)  == 2, "socket hdr source at 2");
    CHECK(offsetof(socketHdrReplica, address) == 4, "socket hdr address at 4");
    CHECK(offsetof(socketHdrReplica, flags)   == 6, "socket hdr flags at 6");

    CHECK(sizeof(msg_hdr_t) == 8, "msg_hdr_t is 8B");
    CHECK(sizeof(msg_rgb_t) == 5, "msg_rgb_t is 5B");
    CHECK(sizeof(msg_value_t) == 4, "msg_value_t is 4B");
    CHECK(sizeof(msg_program_t) == 35, "msg_program_t is 35B");
    // Field offsets: length at 3, address at 6 — the bridge indexes buf[3] directly.
    msg_hdr_t h; memset(&h, 0, sizeof(h));
    CHECK((size_t)((uint8_t *)&h.length  - (uint8_t *)&h) == 3, "length at offset 3");
    CHECK((size_t)((uint8_t *)&h.address - (uint8_t *)&h) == 6, "address at offset 6");

    // msg_poll_response_t used to be the one struct rs485_bridge_protocol.h's static_assert block
    // exempted, because its size was ABI-dependent: config_hdr_t is 10 bytes and, unpacked, a
    // 2-byte-aligned ABI added a trailing pad after msg_version, making the struct 15 B on AVR and
    // 16 B here — so HMTL_MSG_POLL_MIN_LEN was 23 or 24 depending on which end of the bus you
    // asked. HMTLWireFormat.h now packs every wire struct, so the sizes below are exact rather
    // than bounded, and this test asserts the *same* numbers a legacy ATMega328 master would.
    // Run it under -fpack-struct=1 too (see the plan's Test Plan): the AVR-like layout must give
    // identical answers, which is the whole claim.
    CHECK(offsetof(msg_poll_response_t, config)           == 0,  "poll resp config at 0");
    CHECK(offsetof(msg_poll_response_t, object_type)      == 10, "poll resp object_type at 10");
    CHECK(offsetof(msg_poll_response_t, recv_buffer_size) == 12, "poll resp recv_buffer_size at 12");
    CHECK(offsetof(msg_poll_response_t, msg_version)      == 14, "poll resp msg_version at 14");
    CHECK(offsetof(msg_poll_response_t, data)             == 15, "poll resp data at 15");
    CHECK(sizeof(msg_poll_response_t) == 15, "poll resp is 15B on every ABI (packed)");
    // The wire length the bridge actually emits follows from the above: header + response = 23.
    CHECK(HMTL_MSG_POLL_MIN_LEN == sizeof(msg_hdr_t) + sizeof(msg_poll_response_t),
          "HMTL_MSG_POLL_MIN_LEN is msg_hdr_t + msg_poll_response_t");
    CHECK(HMTL_MSG_POLL_MIN_LEN == 23, "poll response is 23B on every ABI");

    // config_hdr_v2_t is the other struct the packing fixed, and it was the worse of the two:
    // interior padding, not trailing. Unpacked it is 8 B with address at offset 3 on AVR but 10 B
    // with address at offset 4 on a 32-bit target, so a v2 EEPROM blob written by a deployed
    // module misparses when read through the struct here. Compile-time dead under
    // HMTL_CONFIG_VERSION 3, pinned anyway because the blobs are not dead.
    CHECK(sizeof(config_hdr_v2_t) == 8, "config_hdr_v2_t is 8B on every ABI (packed)");
    CHECK(offsetof(config_hdr_v2_t, address)     == 3, "config v2 address at 3, no interior pad");
    CHECK(offsetof(config_hdr_v2_t, reserved)    == 5, "config v2 reserved at 5");
    CHECK(offsetof(config_hdr_v2_t, num_outputs) == 6, "config v2 num_outputs at 6");
    CHECK(offsetof(config_hdr_v2_t, flags)       == 7, "config v2 flags at 7");

    // Every remaining wire struct, so "all offsets match across ABIs" is executable rather than
    // asserted in prose. Sizes are already static_asserted in rs485_bridge_protocol.h.
    CHECK(offsetof(config_hdr_v3_t, device_id) == 6, "config v3 device_id at 6");
    CHECK(offsetof(config_hdr_v3_t, address)   == 8, "config v3 address at 8");
    CHECK(offsetof(msg_rgb_t, values)          == 2, "rgb values at 2");
    CHECK(offsetof(msg_program_t, type)        == 2, "program type at 2");
    CHECK(offsetof(msg_program_t, values)      == 3, "program values at 3");
    CHECK(offsetof(msg_set_addr_t, address)    == 2, "set_addr address at 2");
    CHECK(offsetof(msg_sensor_data_t, data)    == 2, "sensor data at 2");
    CHECK(offsetof(output_hdr_t, output)       == 1, "output_hdr output at 1");

    // Packing a struct that holds bitfields is the one case where the attribute could have moved
    // more than padding, so check the 13/3 split at byte level rather than trusting sizeof.
    msg_value_t mv; memset(&mv, 0, sizeof(mv));
    mv.value = 0x1FFF;
    const uint8_t *mvb = (const uint8_t *)&mv;
    CHECK(mvb[2] == 0xFF && mvb[3] == 0x1F, "13-bit value fills bits 0..12 of the unit at offset 2");
    mv.value = 0; mv.flags = 0x7;
    CHECK(mvb[2] == 0x00 && mvb[3] == 0xE0, "3-bit flags occupy bits 13..15");
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

    CHECK(rs485b_validate(buf, sizeof(msg_hdr_t) - 1) == RS485B_ERR_SHORT, "short buffer");
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
    CHECK(rs485b_addressed_to(SOCKET_ADDR_ANY, SELF_ADDR), "broadcast matches");
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
    uint8_t len = build_rgb(buf, SOCKET_ADDR_ANY, HMTL_ALL_OUTPUTS, 1, 2, 3);
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
    CHECK(rs485b_program_to_mode(PROGRAM_BRIGHTNESS) == -1, "brightness is not a mode");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_NONE) == RS485B_WLED_MODE_STATIC, "none -> static");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_FADE) == RS485B_WLED_MODE_FADE, "fade mapped");
    CHECK(rs485b_program_to_mode(HMTL_PROGRAM_CIRCULAR) == RS485B_WLED_MODE_CHASE, "circ mapped");
  }

  // 10) An OUTPUT frame whose payload is too short for its declared output type is unsupported,
  //     never a buffer over-read.
  {
    uint8_t len = (uint8_t)(sizeof(msg_hdr_t) + 1);
    build_hdr(buf, SELF_ADDR, len, MSG_TYPE_OUTPUT, 0);
    buf[sizeof(msg_hdr_t)] = HMTL_OUTPUT_RGB;
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "output header truncated -> unsupported");

    // Full output header but a truncated RGB body.
    len = (uint8_t)(sizeof(msg_hdr_t) + sizeof(output_hdr_t) + 1);
    build_hdr(buf, SELF_ADDR, len, MSG_TYPE_OUTPUT, 0);
    output_hdr_t oh; oh.type = HMTL_OUTPUT_RGB; oh.output = 0;
    memcpy(buf + sizeof(msg_hdr_t), &oh, sizeof(oh));
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "truncated rgb body -> unsupported");

    // Unknown output type.
    len = build_rgb(buf, SELF_ADDR, 0, 1, 2, 3);
    buf[sizeof(msg_hdr_t)] = HMTL_OUTPUT_XBEE;
    d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_UNSUPPORTED, "unknown output type -> unsupported");
  }

  // 11) POLL is answered; the RESPONSE flag is surfaced.
  {
    uint8_t len = build_hdr(buf, SELF_ADDR, sizeof(msg_hdr_t), MSG_TYPE_POLL,
                            MSG_FLAG_RESPONSE);
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
    uint8_t len = build_sensor(buf, SOCKET_ADDR_ANY, HMTL_SENSOR_SOUND, payload, 2);
    RS485BDecision d = rs485b_decide(buf, len, SELF_ADDR, SELF_DEV);
    CHECK(d.action == RS485B_ACT_SENSOR, "sensor -> SENSOR");
    CHECK(d.sensorLen == sizeof(msg_sensor_data_t) + 2, "sensor payload length");

    uint8_t type = 0, dlen = 0;
    const uint8_t *dptr = nullptr;
    uint8_t used = rs485b_next_sensor(d.sensorData, d.sensorLen, 0, &type, &dptr, &dlen);
    CHECK(used == sizeof(msg_sensor_data_t) + 2, "one sensor record consumed");
    CHECK(type == HMTL_SENSOR_SOUND && dlen == 2 && dptr[0] == 0x10, "sensor record decoded");
    CHECK(rs485b_next_sensor(d.sensorData, d.sensorLen, used, &type, &dptr, &dlen) == 0,
          "no second record");

    // A record whose declared data_len runs past the payload is refused.
    uint8_t trunc[4] = { HMTL_SENSOR_LIGHT, 200, 0, 0 };
    CHECK(rs485b_next_sensor(trunc, 4, 0, &type, &dptr, &dlen) == 0, "over-long record refused");
  }

  // 14) Unknown message types are counted, not acted on.
  {
    uint8_t len = build_hdr(buf, SELF_ADDR, sizeof(msg_hdr_t), MSG_TYPE_TIMESYNC, 0);
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
    build_hdr(big, 0x0005, 100, MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(msg_hdr_t), 0, 100 - sizeof(msg_hdr_t));
    CHECK(rs485b_validate_udp_ingress(big, 100, RS485B_TX_SLOT_LEN) == RS485B_ERR_OVERSIZE,
          "oversized frame rejected before sendMsgTo");
    // Exactly at the limit is fine.
    build_hdr(big, 0x0005, RS485B_TX_SLOT_LEN, MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(msg_hdr_t), 0, RS485B_TX_SLOT_LEN - sizeof(msg_hdr_t));
    CHECK(rs485b_validate_udp_ingress(big, RS485B_TX_SLOT_LEN, RS485B_TX_SLOT_LEN) == RS485B_OK,
          "frame exactly at the buffer limit accepted");
    // The ceiling in absolute terms, not just relative to the constant.
    //
    // Every check above uses RS485B_TX_SLOT_LEN symbolically, so they follow the constant wherever it
    // goes -- which is right for the relationship and useless for pinning the NUMBER. These two pin it:
    // 57 is the largest payload a receiver on the default 64-byte budget can actually take once the
    // 7-byte socket header is counted, and 58 is one too many. The constant read 64 until this was
    // fixed, so a frame the bridge happily forwarded was dropped on arrival by every peer.
    CHECK(RS485B_TX_SLOT_LEN == 57, "the payload ceiling is 57, not the buffer size");
    CHECK(RS485B_TX_SLOT_LEN + RS485B_SOCKET_HDR_LEN == RS485B_RECV_BUFFER_LEN,
          "payload + socket header exactly fills the receive budget");
    build_hdr(big, 0x0005, 57, MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(msg_hdr_t), 0, 57 - sizeof(msg_hdr_t));
    CHECK(rs485b_validate_udp_ingress(big, 57, RS485B_TX_SLOT_LEN) == RS485B_OK,
          "a 57-byte frame is accepted");
    build_hdr(big, 0x0005, 58, MSG_TYPE_OUTPUT, 0);
    memset(big + sizeof(msg_hdr_t), 0, 58 - sizeof(msg_hdr_t));
    CHECK(rs485b_validate_udp_ingress(big, 58, RS485B_TX_SLOT_LEN) == RS485B_ERR_OVERSIZE,
          "a 58-byte frame is refused as oversize, not accepted then dropped on the wire");

    // A datagram that claims more than it delivers is short, not oversize.
    build_hdr(big, 0x0005, 40, MSG_TYPE_OUTPUT, 0);
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
    uint8_t len = rs485b_hmtl_fmt(out, sizeof(out), 0x0021, sizeof(msg_hdr_t),
                                  MSG_TYPE_POLL, MSG_FLAG_ACK);
    CHECK(len == sizeof(msg_hdr_t), "fmt returns the frame length");
    CHECK(rs485b_validate(out, len) == RS485B_OK, "formatted frame validates");
    msg_hdr_t h; memcpy(&h, out, sizeof(h));
    CHECK(h.address == 0x0021 && h.type == MSG_TYPE_POLL, "fmt fields land");
    CHECK(rs485b_hmtl_fmt(out, sizeof(out), 0, 3, MSG_TYPE_POLL, 0) == 0,
          "fmt refuses a sub-header length");
    CHECK(rs485b_hmtl_fmt(out, 4, 0, sizeof(msg_hdr_t), MSG_TYPE_POLL, 0) == 0,
          "fmt refuses a too-small buffer");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
