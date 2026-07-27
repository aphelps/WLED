#pragma once
//
// rs485_bridge_protocol.h — the HMTL wire format + the pure bridge decision logic.
//
// This header is deliberately free of any WLED / Arduino / ArduinoLibs dependencies (only
// <stdint.h> / <string.h>), so it compiles and unit-tests on a host build. The Arduino usermod
// (usermod_rs485_bridge.{h,cpp}) includes this and wires it to RS485Socket + WiFiUDP + the
// segment layer; the host test (tests/rs485_bridge_test.cpp) includes only this file.
//
// ---------------------------------------------------------------------------------------------
// Why the wire format is vendored instead of #included from the HMTL submodule
// ---------------------------------------------------------------------------------------------
// The HMTL submodule (../../../HMTL) is the source of truth for this format, but its libraries
// are not buildable inside WLED 16: HMTLTypes.cpp pulls in Debug.h / EEPromUtils.h / PixelUtil.h /
// MPR121.h / XBeeSocket.h, and HMTLMessaging.cpp pulls in HMTLPrograms.h -> FastLED.h (removed in
// WLED 16). Only the *wire format* is needed here, so it is copied — keep it byte-compatible with:
//   HMTL/Libraries/HMTLMessaging/HMTLMessaging.h   (msg_hdr_t and the msg_* payload structs)
//   HMTL/Libraries/HMTLTypes/HMTLTypes.h           (output_hdr_t, HMTL_OUTPUT_*, HMTL_NO_ADDRESS)
//   HMTL/Libraries/HMTLMessaging/HMTLPrograms.h    (HMTL_PROGRAM_*)
//   ArduinoLibs/EEPromUtils/EEPromUtils.cpp:32     (the CRC-8 polynomial)
//
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------------------------
// RS485 socket layer (mirrors ArduinoLibs RS485Utils' rs485_socket_hdr_t) — size math only.
// ---------------------------------------------------------------------------------------------
// { byte ID; byte length; uint16_t source; uint16_t address; byte flags; } with natural
// alignment == 8 bytes. Used to bounds-check RS485Socket::getLength() against the payload
// length before dereferencing (RS485Utils' own checks are compiled out in release builds:
// they sit inside `#if DEBUG_LEVEL >= DEBUG_TRACE`).
#define RS485B_SOCKET_HDR_LEN 8

// ---------------------------------------------------------------------------------------------
// HMTL message framing
// ---------------------------------------------------------------------------------------------
#define HMTL_MSG_START    0xFC
#define HMTL_MSG_VERSION  2
#define HMTL_MAX_MSG_LEN  128

// Message type codes (msg_hdr_t.type)
#define HMTL_MSG_TYPE_OUTPUT      0x01
#define HMTL_MSG_TYPE_POLL        0x02
#define HMTL_MSG_TYPE_SET_ADDR    0x03
#define HMTL_MSG_TYPE_SENSOR      0x04
#define HMTL_MSG_TYPE_TIMESYNC    0x05
#define HMTL_MSG_TYPE_DUMP_CONFIG 0xE0

// Message flags (msg_hdr_t.flags)
#define HMTL_MSG_FLAG_ACK       (1 << 0)
#define HMTL_MSG_FLAG_RESPONSE  (1 << 1)   // sender expects a response
#define HMTL_MSG_FLAG_MORE_DATA (1 << 2)
#define HMTL_MSG_FLAG_ERROR     (1 << 3)

// Output types (output_hdr_t.type)
#define HMTL_OUTPUT_VALUE   0x1
#define HMTL_OUTPUT_RGB     0x2
#define HMTL_OUTPUT_PROGRAM 0x3
#define HMTL_OUTPUT_PIXELS  0x4
#define HMTL_OUTPUT_MPR121  0x5
#define HMTL_OUTPUT_RS485   0x6
#define HMTL_OUTPUT_XBEE    0x7

#define HMTL_NO_OUTPUT      ((uint8_t)-1)   // 0xFF
#define HMTL_ALL_OUTPUTS    ((uint8_t)-2)   // 0xFE

// Addresses. SOCKET_ADDR_ANY (0xFFFF) doubles as HMTL's broadcast / "no address".
#define HMTL_ADDR_BROADCAST ((uint16_t)0xFFFF)
#define HMTL_ADDR_INVALID   ((uint16_t)0xFFFE)

// HMTL programs (HMTLPrograms.h)
#define HMTL_PROGRAM_NONE         0x00
#define HMTL_PROGRAM_BLINK        0x01
#define HMTL_PROGRAM_TIMED_CHANGE 0x02
#define HMTL_PROGRAM_LEVEL_VALUE  0x03
#define HMTL_PROGRAM_SOUND_VALUE  0x04
#define HMTL_PROGRAM_FADE         0x05
#define HMTL_PROGRAM_SPARKLE      0x06
#define HMTL_PROGRAM_SOUND_PIXELS 0x07
#define HMTL_PROGRAM_CIRCULAR     0x08
#define HMTL_PROGRAM_SEQUENCE     0x09
#define HMTL_PROGRAM_BRIGHTNESS   0x30   // one-shot: set brightness, no mode change
#define HMTL_PROGRAM_COLOR        0x31   // one-shot: set colour, no mode change

// Sensor types carried by HMTL_MSG_TYPE_SENSOR
#define HMTL_SENSOR_SOUND 0x1
#define HMTL_SENSOR_LIGHT 0x2
#define HMTL_SENSOR_POT   0x3

// The WLED effect ids this bridge maps HMTL programs onto. Duplicated as literals so this
// header stays dependency-free; usermod_rs485_bridge.cpp static_asserts them against FX.h.
#define RS485B_WLED_MODE_STATIC  0    // FX_MODE_STATIC
#define RS485B_WLED_MODE_BLINK   1    // FX_MODE_BLINK
#define RS485B_WLED_MODE_FADE    12   // FX_MODE_FADE
#define RS485B_WLED_MODE_SPARKLE 20   // FX_MODE_SPARKLE
#define RS485B_WLED_MODE_CHASE   28   // FX_MODE_CHASE_COLOR

// 8 bytes: 6 x uint8_t then a naturally-aligned uint16_t. Matches HMTL's msg_hdr_t on both AVR
// and Xtensa; `packed` makes that explicit rather than assumed.
struct __attribute__((packed)) HmtlMsgHdr {
  uint8_t  startcode;  // HMTL_MSG_START
  uint8_t  crc;        // CRC-8 over the whole message with this field zeroed; 0 == "not used"
  uint8_t  version;    // HMTL_MSG_VERSION
  uint8_t  length;     // TOTAL message length, header included
  uint8_t  type;       // HMTL_MSG_TYPE_*
  uint8_t  flags;      // HMTL_MSG_FLAG_*
  uint16_t address;    // destination (HMTL_ADDR_BROADCAST == everyone)
};

struct __attribute__((packed)) HmtlOutputHdr {
  uint8_t type;        // HMTL_OUTPUT_*
  uint8_t output;      // output index, or HMTL_ALL_OUTPUTS
};

struct __attribute__((packed)) HmtlMsgValue {
  HmtlOutputHdr hdr;
  uint16_t      value : 13;   // 0..8191
  uint16_t      flags : 3;
};

struct __attribute__((packed)) HmtlMsgRgb {
  HmtlOutputHdr hdr;
  uint8_t       values[3];
};

#define HMTL_MAX_PROGRAM_VAL 32
struct __attribute__((packed)) HmtlMsgProgram {
  HmtlOutputHdr hdr;
  uint8_t       type;                        // HMTL_PROGRAM_*
  uint8_t       values[HMTL_MAX_PROGRAM_VAL];
};

struct __attribute__((packed)) HmtlMsgSetAddr {
  uint16_t device_id;  // 0 == "any device"; otherwise only that device id reacts
  uint16_t address;    // the new RS485 socket address
};

struct __attribute__((packed)) HmtlSensorData {
  uint8_t sensor_type; // HMTL_SENSOR_*
  uint8_t data_len;
  // uint8_t data[data_len] follows
};

// Poll response payload (HMTL config_hdr_v3_t + poll fields). Emitted, never consumed.
struct __attribute__((packed)) HmtlConfigHdrV3 {
  uint8_t  magic;             // HMTL_CONFIG_MAGIC
  uint8_t  protocol_version;  // HMTL_CONFIG_VERSION
  uint8_t  hardware_version;
  uint8_t  baud;              // baud / 1200
  uint8_t  num_outputs;
  uint8_t  flags;
  uint16_t device_id;
  uint16_t address;
};

struct __attribute__((packed)) HmtlMsgPollResponse {
  HmtlConfigHdrV3 config;
  uint16_t        object_type;
  uint16_t        recv_buffer_size;
  uint8_t         msg_version;
};

#define HMTL_CONFIG_MAGIC        0x5C
#define HMTL_CONFIG_VERSION      3
#define HMTL_BAUD_TO_BYTE(b)     ((uint8_t)((b) / 1200))
#define HMTL_OBJECT_TYPE_WLED    0x57   // 'W' — not a stock HMTL object type; identifies this bridge

// ---------------------------------------------------------------------------------------------
// CRC-8 (poly 0xD8, init 0, MSB-first) — ArduinoLibs' EEPROM_crc, which HMTL uses for msg_hdr.crc
// ---------------------------------------------------------------------------------------------
static inline uint8_t rs485b_crc8(const uint8_t *data, uint16_t len) {
  uint8_t remainder = 0;
  for (uint16_t i = 0; i < len; i++) {
    remainder ^= data[i];
    for (uint8_t bit = 8; bit > 0; bit--) {
      if (remainder & 0x80) remainder = (uint8_t)((remainder << 1) ^ 0xD8);
      else                  remainder = (uint8_t)(remainder << 1);
    }
  }
  return remainder;
}

// CRC of an HMTL message as the sender computes it: the crc byte itself counted as zero.
static inline uint8_t rs485b_hmtl_crc(const uint8_t *msg, uint8_t len) {
  uint8_t remainder = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = (i == 1) ? 0 : msg[i];   // offset 1 == HmtlMsgHdr::crc
    remainder ^= b;
    for (uint8_t bit = 8; bit > 0; bit--) {
      if (remainder & 0x80) remainder = (uint8_t)((remainder << 1) ^ 0xD8);
      else                  remainder = (uint8_t)(remainder << 1);
    }
  }
  return remainder;
}

// Stamp startcode/version/length/type/flags/address + CRC into a buffer. Returns `length`,
// or 0 if the buffer is too small.
static inline uint8_t rs485b_hmtl_fmt(uint8_t *buf, uint16_t bufSize, uint16_t address,
                                      uint8_t length, uint8_t type, uint8_t flags) {
  if (length < sizeof(HmtlMsgHdr) || bufSize < length) return 0;
  HmtlMsgHdr h;
  h.startcode = HMTL_MSG_START;
  h.crc       = 0;
  h.version   = HMTL_MSG_VERSION;
  h.length    = length;
  h.type      = type;
  h.flags     = flags;
  h.address   = address;
  memcpy(buf, &h, sizeof(h));
  buf[1] = rs485b_hmtl_crc(buf, length);
  return length;
}

// ---------------------------------------------------------------------------------------------
// Frame validation
// ---------------------------------------------------------------------------------------------
enum RS485BFrameResult : uint8_t {
  RS485B_OK = 0,
  RS485B_ERR_SHORT,     // fewer bytes available than an HMTL header, or than hdr.length
  RS485B_ERR_START,     // bad start code
  RS485B_ERR_VERSION,   // unsupported protocol version
  RS485B_ERR_LENGTH,    // hdr.length < header, or > HMTL_MAX_MSG_LEN
  RS485B_ERR_CRC,       // non-zero crc that does not match
  RS485B_ERR_OVERSIZE,  // valid frame, but longer than the transmit buffer can carry
};

// Validate an HMTL frame sitting in `buf` with `avail` readable bytes.
// A zero crc field is accepted unchecked: HMTL_USE_CRC is off in stock HMTL builds, so legacy
// modules transmit crc == 0. A non-zero crc must match.
static inline RS485BFrameResult rs485b_validate(const uint8_t *buf, uint16_t avail) {
  if (buf == nullptr || avail < sizeof(HmtlMsgHdr)) return RS485B_ERR_SHORT;
  HmtlMsgHdr h;
  memcpy(&h, buf, sizeof(h));
  if (h.startcode != HMTL_MSG_START)  return RS485B_ERR_START;
  if (h.version   != HMTL_MSG_VERSION) return RS485B_ERR_VERSION;
  if (h.length < sizeof(HmtlMsgHdr) || h.length > HMTL_MAX_MSG_LEN) return RS485B_ERR_LENGTH;
  if (avail < h.length) return RS485B_ERR_SHORT;
  if (h.crc != 0 && h.crc != rs485b_hmtl_crc(buf, h.length)) return RS485B_ERR_CRC;
  return RS485B_OK;
}

// Validate a datagram arriving on the WiFi side before it is handed to RS485Socket::sendMsgTo.
// `maxPayload` is the socket's usable data size (RS485Socket::initBuffer's data_size, default
// RS485_RECV_BUFFER == 64). sendMsgTo takes a `byte` length and writes the socket header in
// FRONT of the caller's buffer, so an oversized datagram would overflow it.
static inline RS485BFrameResult rs485b_validate_udp_ingress(const uint8_t *buf, uint16_t len,
                                                            uint16_t maxPayload) {
  RS485BFrameResult r = rs485b_validate(buf, len);
  if (r != RS485B_OK) return r;
  uint8_t frameLen = buf[3];   // HmtlMsgHdr::length
  if (frameLen > maxPayload || frameLen > 255) return RS485B_ERR_OVERSIZE;
  return RS485B_OK;
}

// Is a message with destination `msgAddr` meant for a node listening on `selfAddr`?
static inline bool rs485b_addressed_to(uint16_t msgAddr, uint16_t selfAddr) {
  return msgAddr == HMTL_ADDR_BROADCAST || msgAddr == selfAddr;
}

// ---------------------------------------------------------------------------------------------
// Bridge decision logic — "what should the usermod do with this frame?"
// ---------------------------------------------------------------------------------------------
enum RS485BAction : uint8_t {
  RS485B_ACT_DROP = 0,     // malformed — see RS485BDecision::err
  RS485B_ACT_RELAY_ONLY,   // well-formed but addressed elsewhere: relay to the WiFi peer only
  RS485B_ACT_UNSUPPORTED,  // addressed to us but not in the v1 command set: count and ignore
  RS485B_ACT_SET_RGB,
  RS485B_ACT_SET_VALUE,
  RS485B_ACT_PROGRAM,
  RS485B_ACT_POLL,
  RS485B_ACT_SET_ADDRESS,
  RS485B_ACT_SENSOR,
};

struct RS485BDecision {
  RS485BAction      action;
  RS485BFrameResult err;          // meaningful when action == RS485B_ACT_DROP
  uint16_t          destAddress;  // hdr.address as received
  uint8_t           output;       // HMTL output index, or HMTL_ALL_OUTPUTS
  bool              wantsResponse;
  uint8_t           rgb[3];       // SET_RGB
  uint16_t          value;        // SET_VALUE (0..8191)
  uint8_t           programType;  // PROGRAM: HMTL_PROGRAM_*
  const uint8_t    *programVals;  // PROGRAM: payload bytes after `programType`
  uint8_t           programLen;
  uint16_t          newAddress;   // SET_ADDRESS
  uint16_t          deviceId;     // SET_ADDRESS: 0 == "any device"
  const uint8_t    *sensorData;   // SENSOR: payload bytes after the HMTL header
  uint8_t           sensorLen;
};

// Decide what to do with one received HMTL frame.
//   selfAddr     — this node's RS485 socket address
//   selfDeviceId — this node's HMTL device id (SET_ADDRESS is ignored unless it matches, or the
//                  message carries device_id == 0 meaning "any device")
static inline RS485BDecision rs485b_decide(const uint8_t *buf, uint16_t avail,
                                           uint16_t selfAddr, uint16_t selfDeviceId) {
  RS485BDecision d;
  memset(&d, 0, sizeof(d));
  d.output = HMTL_NO_OUTPUT;

  d.err = rs485b_validate(buf, avail);
  if (d.err != RS485B_OK) { d.action = RS485B_ACT_DROP; return d; }

  HmtlMsgHdr h;
  memcpy(&h, buf, sizeof(h));
  d.destAddress   = h.address;
  d.wantsResponse = (h.flags & HMTL_MSG_FLAG_RESPONSE) != 0;

  if (!rs485b_addressed_to(h.address, selfAddr)) { d.action = RS485B_ACT_RELAY_ONLY; return d; }

  const uint8_t *payload    = buf + sizeof(HmtlMsgHdr);
  const uint8_t  payloadLen = (uint8_t)(h.length - sizeof(HmtlMsgHdr));

  switch (h.type) {
    case HMTL_MSG_TYPE_OUTPUT: {
      if (payloadLen < sizeof(HmtlOutputHdr)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      HmtlOutputHdr oh;
      memcpy(&oh, payload, sizeof(oh));
      d.output = oh.output;
      switch (oh.type) {
        case HMTL_OUTPUT_RGB: {
          if (payloadLen < sizeof(HmtlMsgRgb)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          HmtlMsgRgb m;
          memcpy(&m, payload, sizeof(m));
          d.rgb[0] = m.values[0]; d.rgb[1] = m.values[1]; d.rgb[2] = m.values[2];
          d.action = RS485B_ACT_SET_RGB;
          return d;
        }
        case HMTL_OUTPUT_VALUE: {
          if (payloadLen < sizeof(HmtlMsgValue)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          HmtlMsgValue m;
          memcpy(&m, payload, sizeof(m));
          d.value  = m.value;
          d.action = RS485B_ACT_SET_VALUE;
          return d;
        }
        case HMTL_OUTPUT_PROGRAM:
        case HMTL_OUTPUT_PIXELS: {
          // msg_program_t is a fixed 35 bytes on the wire, but senders may truncate the unused
          // tail; only the output header + program type byte are required.
          if (payloadLen < sizeof(HmtlOutputHdr) + 1) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          d.programType = payload[sizeof(HmtlOutputHdr)];
          d.programVals = payload + sizeof(HmtlOutputHdr) + 1;
          d.programLen  = (uint8_t)(payloadLen - sizeof(HmtlOutputHdr) - 1);
          d.action      = RS485B_ACT_PROGRAM;
          return d;
        }
        default:
          d.action = RS485B_ACT_UNSUPPORTED;
          return d;
      }
    }
    case HMTL_MSG_TYPE_POLL:
      d.action = RS485B_ACT_POLL;
      return d;

    case HMTL_MSG_TYPE_SET_ADDR: {
      if (payloadLen < sizeof(HmtlMsgSetAddr)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      HmtlMsgSetAddr m;
      memcpy(&m, payload, sizeof(m));
      if (m.device_id != 0 && m.device_id != selfDeviceId) {
        d.action = RS485B_ACT_UNSUPPORTED;   // addressed to us, but a different device id
        return d;
      }
      d.deviceId   = m.device_id;
      d.newAddress = m.address;
      d.action     = RS485B_ACT_SET_ADDRESS;
      return d;
    }
    case HMTL_MSG_TYPE_SENSOR:
      d.sensorData = payload;
      d.sensorLen  = payloadLen;
      d.action     = RS485B_ACT_SENSOR;
      return d;

    default:
      d.action = RS485B_ACT_UNSUPPORTED;
      return d;
  }
}

// Iterate the HmtlSensorData records packed into a SENSOR payload.
// Returns the number of bytes consumed (0 == no further record fits).
static inline uint8_t rs485b_next_sensor(const uint8_t *data, uint8_t len, uint8_t offset,
                                         uint8_t *outType, const uint8_t **outData,
                                         uint8_t *outLen) {
  if (offset + sizeof(HmtlSensorData) > len) return 0;
  HmtlSensorData s;
  memcpy(&s, data + offset, sizeof(s));
  if (offset + sizeof(HmtlSensorData) + s.data_len > len) return 0;
  *outType = s.sensor_type;
  *outData = data + offset + sizeof(HmtlSensorData);
  *outLen  = s.data_len;
  return (uint8_t)(sizeof(HmtlSensorData) + s.data_len);
}

// HMTL program -> WLED effect id. Returns -1 for programs with no v1 mapping (the caller counts
// and ignores those), and for the two one-shot programs that are not modes at all.
static inline int rs485b_program_to_mode(uint8_t program) {
  switch (program) {
    case HMTL_PROGRAM_NONE:    return RS485B_WLED_MODE_STATIC;
    case HMTL_PROGRAM_BLINK:   return RS485B_WLED_MODE_BLINK;
    case HMTL_PROGRAM_FADE:    return RS485B_WLED_MODE_FADE;
    case HMTL_PROGRAM_SPARKLE: return RS485B_WLED_MODE_SPARKLE;
    case HMTL_PROGRAM_CIRCULAR:return RS485B_WLED_MODE_CHASE;
    default:                   return -1;
  }
}

// ---------------------------------------------------------------------------------------------
// Transmit queue — bounded, drop-oldest.
// ---------------------------------------------------------------------------------------------
// RS485Socket::sendMsgTo blocks for the whole frame (it flushes the UART while holding DE high:
// ~48 ms for a 64-byte payload at 28000 baud once Gammon's byte-stuffing is counted), so the
// usermod forwards at most one frame per loop() iteration and parks the rest here. Dropping the
// oldest keeps a UDP flood from stalling the LED refresh or tripping the watchdog.
#define RS485B_TX_SLOTS    4
#define RS485B_TX_SLOT_LEN 64

struct RS485BTxQueue {
  uint8_t  buf[RS485B_TX_SLOTS][RS485B_TX_SLOT_LEN];
  uint8_t  len[RS485B_TX_SLOTS];
  uint16_t dest[RS485B_TX_SLOTS];
  uint8_t  head  = 0;   // next slot to pop
  uint8_t  tail  = 0;   // next slot to write
  uint8_t  count = 0;

  void clear() { head = tail = count = 0; }
  bool empty() const { return count == 0; }
  bool full()  const { return count == RS485B_TX_SLOTS; }

  // Returns false if an older entry had to be discarded to make room (the new one is still
  // queued) or if the frame does not fit a slot (nothing is queued in that case).
  bool push(uint16_t destAddr, const uint8_t *data, uint8_t dataLen) {
    if (data == nullptr || dataLen == 0 || dataLen > RS485B_TX_SLOT_LEN) return false;
    bool dropped = false;
    if (full()) {
      head = (uint8_t)((head + 1) % RS485B_TX_SLOTS);
      count--;
      dropped = true;
    }
    memcpy(buf[tail], data, dataLen);
    len[tail]  = dataLen;
    dest[tail] = destAddr;
    tail = (uint8_t)((tail + 1) % RS485B_TX_SLOTS);
    count++;
    return !dropped;
  }

  // Copies the oldest frame into `out` (must be >= RS485B_TX_SLOT_LEN). Returns false if empty.
  bool pop(uint16_t *destAddr, uint8_t *out, uint8_t *outLen) {
    if (empty()) return false;
    *destAddr = dest[head];
    *outLen   = len[head];
    memcpy(out, buf[head], len[head]);
    head = (uint8_t)((head + 1) % RS485B_TX_SLOTS);
    count--;
    return true;
  }
};

// ---------------------------------------------------------------------------------------------
// Counters (surfaced in /json/info)
// ---------------------------------------------------------------------------------------------
struct RS485BCounters {
  uint32_t rs485Rx      = 0;   // frames pulled off the bus
  uint32_t rs485Handled = 0;   // frames acted on locally
  uint32_t rs485Relayed = 0;   // frames relayed to the WiFi peer
  uint32_t rs485Tx      = 0;   // frames written to the bus
  uint32_t errShort     = 0;
  uint32_t errStart     = 0;
  uint32_t errVersion   = 0;
  uint32_t errLength    = 0;
  uint32_t errCrc       = 0;
  uint32_t errOversize  = 0;
  uint32_t unsupported  = 0;   // addressed to us but outside the v1 command set
  uint32_t udpRx        = 0;
  uint32_t udpRejected  = 0;
  uint32_t txDropped    = 0;   // queue overflow, drop-oldest

  void countError(RS485BFrameResult r) {
    switch (r) {
      case RS485B_ERR_SHORT:    errShort++;    break;
      case RS485B_ERR_START:    errStart++;    break;
      case RS485B_ERR_VERSION:  errVersion++;  break;
      case RS485B_ERR_LENGTH:   errLength++;   break;
      case RS485B_ERR_CRC:      errCrc++;      break;
      case RS485B_ERR_OVERSIZE: errOversize++; break;
      case RS485B_OK:           break;
    }
  }
};

// Compile-time wire-layout guards. A mismatch here means the vendored copy has drifted from
// HMTL/Libraries and legacy modules would no longer interoperate.
static_assert(sizeof(HmtlMsgHdr)     == 8,  "HMTL msg_hdr_t must be 8 bytes on the wire");
static_assert(sizeof(HmtlOutputHdr)  == 2,  "HMTL output_hdr_t must be 2 bytes on the wire");
static_assert(sizeof(HmtlMsgValue)   == 4,  "HMTL msg_value_t must be 4 bytes on the wire");
static_assert(sizeof(HmtlMsgRgb)     == 5,  "HMTL msg_rgb_t must be 5 bytes on the wire");
static_assert(sizeof(HmtlMsgProgram) == 35, "HMTL msg_program_t must be 35 bytes on the wire");
static_assert(sizeof(HmtlMsgSetAddr) == 4,  "HMTL msg_set_addr_t must be 4 bytes on the wire");
static_assert(sizeof(HmtlConfigHdrV3) == 10, "HMTL config_hdr_v3_t must be 10 bytes on the wire");
