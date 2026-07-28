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

// HMTL's msg_value_t. The 13/3 bit split is little-endian bitfield allocation — value occupies
// the low 13 bits of the uint16_t. Both AVR-GCC and Xtensa-GCC lay it out that way, which is why
// this can be a bitfield at all rather than hand-shifted; the static_assert at the bottom of this
// header pins the resulting size.
struct __attribute__((packed)) HmtlMsgValue {
  HmtlOutputHdr hdr;
  uint16_t      value : 13;   // 0..8191 — HMTL's "level"; the bridge clamps it to 8 bits
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

// Compute the raw CRC-8 of `len` bytes at `data`.
//
// This must stay a byte-for-byte transcription of ArduinoLibs' EEPROM_crc
// (EEPromUtils.cpp:32) — polynomial 0xD8, initial remainder 0, MSB-first, no final XOR and no
// reflection. HMTL modules on the bus compute their crc byte with that exact routine, so any
// deviation (a reflected variant, a different init, the 0x8C LSB-first "Dallas" poly that looks
// superficially similar) produces frames legacy hardware silently discards.
//
// Returns the 8-bit remainder; a zero-length buffer yields 0.
static inline uint8_t rs485b_crc8(const uint8_t *data, uint16_t len) {
  uint8_t remainder = 0;
  for (uint16_t i = 0; i < len; i++) {
    remainder ^= data[i];
    // Shift the whole byte through the register, folding in the polynomial on each 1 bit that
    // falls out of the top.
    for (uint8_t bit = 8; bit > 0; bit--) {
      if (remainder & 0x80) remainder = (uint8_t)((remainder << 1) ^ 0xD8);
      else                  remainder = (uint8_t)(remainder << 1);
    }
  }
  return remainder;
}

// Compute the CRC of a whole HMTL message the way the sender does it.
//
// The crc byte lives *inside* the region being checksummed (HmtlMsgHdr::crc, offset 1), so the
// sender computes the CRC with that byte held at zero and only then writes the result into it.
// Substituting a zero for offset 1 here — rather than checksumming the buffer as-is — is what
// makes the operation idempotent: stamping a frame and re-running this function returns the same
// value, so a receiver can verify without having to clear the field in a scratch copy first.
//
//   msg — the complete frame, starting at the HMTL header
//   len — hdr.length (the TOTAL frame length, header included)
static inline uint8_t rs485b_hmtl_crc(const uint8_t *msg, uint8_t len) {
  uint8_t remainder = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = (i == 1) ? 0 : msg[i];   // offset 1 == HmtlMsgHdr::crc, treated as zero
    remainder ^= b;
    for (uint8_t bit = 8; bit > 0; bit--) {
      if (remainder & 0x80) remainder = (uint8_t)((remainder << 1) ^ 0xD8);
      else                  remainder = (uint8_t)(remainder << 1);
    }
  }
  return remainder;
}

// Write an HMTL header into `buf` and stamp the frame's CRC.
//
//   buf     — destination, must already hold the payload bytes that follow the header
//   bufSize — capacity of `buf`, checked so a caller cannot overflow it via `length`
//   address — destination address (HMTL_ADDR_BROADCAST for everyone)
//   length  — TOTAL frame length, header included (this is what hdr.length carries)
//   type    — HMTL_MSG_TYPE_*
//   flags   — HMTL_MSG_FLAG_*
//
// Returns `length` on success, or 0 if `length` is smaller than a header or larger than `buf`.
//
// The CRC covers the payload as well as the header, so the payload must be in place *before*
// calling this. A caller that fills the payload afterwards has to re-stamp buf[1] itself — see
// sendPollResponse() in usermod_rs485_bridge.cpp, which does exactly that.
static inline uint8_t rs485b_hmtl_fmt(uint8_t *buf, uint16_t bufSize, uint16_t address,
                                      uint8_t length, uint8_t type, uint8_t flags) {
  // Reject both under- and over-sized requests: `length` goes on the wire as the authoritative
  // frame size, so it must describe a real header and must fit the buffer we are writing.
  if (length < sizeof(HmtlMsgHdr) || bufSize < length) return 0;
  HmtlMsgHdr h;
  h.startcode = HMTL_MSG_START;
  h.crc       = 0;                     // zero while the CRC is computed; filled in below
  h.version   = HMTL_MSG_VERSION;
  h.length    = length;
  h.type      = type;
  h.flags     = flags;
  h.address   = address;
  memcpy(buf, &h, sizeof(h));          // build in a local, then blit: `buf` may be unaligned
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

// Validate an HMTL frame sitting in `buf`, of which `avail` bytes are actually readable.
//
// Every later function in this header (and every caller in the usermod) treats a RS485B_OK frame
// as safe to index up to hdr.length, so this is the single choke point that has to establish that
// invariant. The checks run in an order that never reads a byte it has not already justified:
//
//   1. `avail` covers a header       — nothing below may touch buf[0..7] until this holds
//   2. start code / version          — cheapest way to discard bus noise and foreign protocols
//   3. hdr.length is self-consistent — at least a header, at most HMTL_MAX_MSG_LEN, so the
//                                      declared size can never exceed a receive buffer
//   4. `avail` covers hdr.length     — the frame is all here, not a truncated read
//   5. CRC                           — only now, since it checksums hdr.length bytes
//
// A zero crc field is accepted unchecked, deliberately: HMTL_USE_CRC is commented out in stock
// HMTL builds, so legacy modules transmit crc == 0 (HMTLMessaging.cpp:245) and rejecting those
// would make the bridge unable to talk to the hardware it exists for. A non-zero crc must match.
static inline RS485BFrameResult rs485b_validate(const uint8_t *buf, uint16_t avail) {
  if (buf == nullptr || avail < sizeof(HmtlMsgHdr)) return RS485B_ERR_SHORT;
  HmtlMsgHdr h;
  memcpy(&h, buf, sizeof(h));          // copy out: `buf` has no alignment guarantee
  if (h.startcode != HMTL_MSG_START)  return RS485B_ERR_START;
  if (h.version   != HMTL_MSG_VERSION) return RS485B_ERR_VERSION;
  // A length below the header or above the protocol maximum is either corruption or a hostile
  // frame; either way it must not become a read bound.
  if (h.length < sizeof(HmtlMsgHdr) || h.length > HMTL_MAX_MSG_LEN) return RS485B_ERR_LENGTH;
  if (avail < h.length) return RS485B_ERR_SHORT;   // declared more than it delivered
  if (h.crc != 0 && h.crc != rs485b_hmtl_crc(buf, h.length)) return RS485B_ERR_CRC;
  return RS485B_OK;
}

// Validate a datagram arriving on the WiFi side before it is handed to RS485Socket::sendMsgTo.
//
//   buf        — the datagram as read off the socket
//   len        — bytes actually read
//   maxPayload — the socket's usable data size, i.e. what RS485Socket::initBuffer() handed back
//                (RS485_RECV_BUFFER == 64 by default)
//
// Beyond ordinary frame validation this enforces the transmit-side bound, and that bound is a
// memory-safety requirement rather than a policy: sendMsgTo() takes its length as a `byte` and
// writes the RS485 socket header *in front of* the caller's data pointer, into the same backing
// array. Handing it a frame longer than the region initBuffer() reserved therefore overruns that
// array. Rejecting here — before the frame is ever queued — is the only place the caller still
// has the information to say no.
//
// Note for bring-up: the usermod's UDP read buffer is only RS485B_TX_SLOT_LEN bytes, so a
// legal-but-long HMTL frame (up to HMTL_MAX_MSG_LEN == 128) is truncated by udp.read() before it
// reaches here and is counted as RS485B_ERR_SHORT rather than RS485B_ERR_OVERSIZE.
static inline RS485BFrameResult rs485b_validate_udp_ingress(const uint8_t *buf, uint16_t len,
                                                            uint16_t maxPayload) {
  RS485BFrameResult r = rs485b_validate(buf, len);
  if (r != RS485B_OK) return r;
  // Safe to read: rs485b_validate() has already established that a full header is present.
  uint8_t frameLen = buf[3];   // HmtlMsgHdr::length
  if (frameLen > maxPayload || frameLen > 255) return RS485B_ERR_OVERSIZE;
  return RS485B_OK;
}

// Is a message with destination `msgAddr` meant for a node listening on `selfAddr`?
// HMTL has no subnet or group concept — a frame is either a unicast to one address or the
// 0xFFFF broadcast every node acts on.
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
//
//   buf          — the frame as it came off the bus (HMTL header first)
//   avail        — readable bytes in `buf`
//   selfAddr     — this node's RS485 socket address
//   selfDeviceId — this node's HMTL device id (SET_ADDRESS is ignored unless it matches, or the
//                  message carries device_id == 0 meaning "any device")
//
// This is the whole slave-path policy in one pure function, which is what makes it host
// unit-testable: it never touches hardware, allocates nothing, and returns a decision the
// usermod then executes. Pointers in the returned decision (programVals, sensorData) alias into
// `buf`, so the caller must act on the decision before that buffer is reused.
//
// Every payload branch re-checks `payloadLen` against the struct it is about to read. That is
// not redundant with rs485b_validate(): validation only proved hdr.length bytes are present, not
// that the sender declared a length consistent with the message type it claimed. Without these
// checks a frame announcing HMTL_OUTPUT_RGB but carrying two payload bytes would read past the
// end of the received data.
static inline RS485BDecision rs485b_decide(const uint8_t *buf, uint16_t avail,
                                           uint16_t selfAddr, uint16_t selfDeviceId) {
  RS485BDecision d;
  memset(&d, 0, sizeof(d));
  d.output = HMTL_NO_OUTPUT;           // "no output specified" unless a payload says otherwise

  d.err = rs485b_validate(buf, avail);
  if (d.err != RS485B_OK) { d.action = RS485B_ACT_DROP; return d; }

  HmtlMsgHdr h;
  memcpy(&h, buf, sizeof(h));
  d.destAddress   = h.address;
  d.wantsResponse = (h.flags & HMTL_MSG_FLAG_RESPONSE) != 0;

  // Frames for other nodes are not errors — the bridge listens promiscuously (RS485_ADDR_ANY) so
  // it can mirror the whole bus back to the WiFi peer. They just get no local handling.
  if (!rs485b_addressed_to(h.address, selfAddr)) { d.action = RS485B_ACT_RELAY_ONLY; return d; }

  const uint8_t *payload    = buf + sizeof(HmtlMsgHdr);
  // Safe subtraction: validate() guaranteed h.length >= sizeof(HmtlMsgHdr).
  const uint8_t  payloadLen = (uint8_t)(h.length - sizeof(HmtlMsgHdr));

  switch (h.type) {
    case HMTL_MSG_TYPE_OUTPUT: {
      // An OUTPUT message starts with an output header naming the target output and its type.
      if (payloadLen < sizeof(HmtlOutputHdr)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      HmtlOutputHdr oh;
      memcpy(&oh, payload, sizeof(oh));
      d.output = oh.output;            // index, or HMTL_ALL_OUTPUTS; the bridge has one output
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
          d.value  = m.value;          // 13-bit bitfield; widened to uint16_t here
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
          // PIXELS-style outputs the bridge has no equivalent for (MPR121, RS485, XBee...).
          d.action = RS485B_ACT_UNSUPPORTED;
          return d;
      }
    }
    case HMTL_MSG_TYPE_POLL:
      // A poll needs no payload — the caller answers with this node's config block so a master
      // can enumerate the bus.
      d.action = RS485B_ACT_POLL;
      return d;

    case HMTL_MSG_TYPE_SET_ADDR: {
      if (payloadLen < sizeof(HmtlMsgSetAddr)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      HmtlMsgSetAddr m;
      memcpy(&m, payload, sizeof(m));
      // SET_ADDRESS is normally sent to the broadcast address during commissioning, so the
      // device id is the real selector. device_id == 0 means "whoever hears this", which is how
      // a single unconfigured module on the bench gets its first address; anything else must
      // match us exactly or we would steal another module's address.
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
      // Sensor broadcasts are handed over whole; rs485b_next_sensor() walks the records. v1 only
      // relays them (the sensorsync-rs485-transport follow-up consumes them).
      d.sensorData = payload;
      d.sensorLen  = payloadLen;
      d.action     = RS485B_ACT_SENSOR;
      return d;

    default:
      // TIMESYNC, DUMP_CONFIG and anything newer: outside the v1 set, counted by the caller.
      d.action = RS485B_ACT_UNSUPPORTED;
      return d;
  }
}

// Step over one HmtlSensorData record in a SENSOR payload.
//
// A SENSOR message carries a run of variable-length records back to back, each a
// { sensor_type, data_len } header followed by data_len bytes. Call this with offset 0, then
// with the running total of the returned sizes, until it returns 0.
//
//   data    — the SENSOR payload (RS485BDecision::sensorData)
//   len     — its length (RS485BDecision::sensorLen)
//   offset  — byte offset of the record to read
//   outType/outData/outLen — receive the record's type, data pointer (aliases `data`) and length
//
// Returns the number of bytes this record occupies, or 0 when no complete record starts at
// `offset`. The two bounds checks are what make a hostile or corrupt `data_len` harmless: a
// record claiming more bytes than the payload holds is refused rather than handed out as a
// pointer/length pair the caller would read past the end of.
static inline uint8_t rs485b_next_sensor(const uint8_t *data, uint8_t len, uint8_t offset,
                                         uint8_t *outType, const uint8_t **outData,
                                         uint8_t *outLen) {
  // Is there even a record header left? (Arithmetic promotes to int, so this cannot wrap.)
  if (offset + sizeof(HmtlSensorData) > len) return 0;
  HmtlSensorData s;
  memcpy(&s, data + offset, sizeof(s));
  // Does the body the header claims actually fit inside the payload?
  if (offset + sizeof(HmtlSensorData) + s.data_len > len) return 0;
  *outType = s.sensor_type;
  *outData = data + offset + sizeof(HmtlSensorData);
  *outLen  = s.data_len;
  return (uint8_t)(sizeof(HmtlSensorData) + s.data_len);
}

// Map an HMTL program id onto the WLED effect id the bridge runs for it.
//
// The v1 mapping is deliberately shallow — the bridge's deliverable is the transport, not a full
// HMTL program emulation. Returns -1 when there is no mapping, which covers two cases the caller
// treats identically (count as unsupported, change nothing):
//   * programs with no WLED analogue (SOUND_VALUE, SEQUENCE, TIMED_CHANGE, ...)
//   * HMTL_PROGRAM_BRIGHTNESS / HMTL_PROGRAM_COLOR, which are one-shot property sets rather than
//     modes — applyProgram() handles those before it ever consults this table.
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
// Why this queue exists at all: RS485Socket::sendMsgTo() is *blocking*. On the ESP32 hardware
// serial path it raises the driver-enable pin, writes the frame and then calls serial->flush(),
// busy-waiting until the last bit is on the wire before it can drop DE — it has to, because
// releasing DE early truncates the frame on a half-duplex bus. At the HMTL-compatible 28000 baud
// a 64-byte payload becomes roughly 135 wire bytes once Gammon's byte-stuffing and the socket
// header are counted, i.e. about 48 ms spent inside WLED's loop().
//
// WLED's loop() also drives the LED refresh and feeds the task watchdog, so an unbounded forward
// path would let a UDP flood stall the strip or reset the board. The bridge therefore transmits
// at most one frame per loop() pass and parks the rest here.
//
// The queue is intentionally tiny and drop-oldest rather than block-the-sender: RS485 carries
// live lighting commands, so if the bus cannot keep up, the freshest command is the one worth
// delivering and a backlog of stale ones is worse than useless. Fixed-size slots keep it free of
// heap allocation, which matters in a usermod running for months at a time.
#define RS485B_TX_SLOTS    4
#define RS485B_TX_SLOT_LEN 64

struct RS485BTxQueue {
  uint8_t  buf[RS485B_TX_SLOTS][RS485B_TX_SLOT_LEN];
  uint8_t  len[RS485B_TX_SLOTS];
  uint16_t dest[RS485B_TX_SLOTS];
  uint8_t  head  = 0;   // next slot to pop
  uint8_t  tail  = 0;   // next slot to write
  uint8_t  count = 0;   // occupancy — head == tail is ambiguous without it

  // Discard everything queued (used when the bridge stops).
  void clear() { head = tail = count = 0; }
  // True when there is nothing to transmit.
  bool empty() const { return count == 0; }
  // True when the next push() will have to drop the oldest frame.
  bool full()  const { return count == RS485B_TX_SLOTS; }

  // Queue one frame for transmission to `destAddr`.
  //
  // Returns true when the frame was queued with nothing lost. Returns false in two distinct
  // situations, both of which the caller counts as a drop:
  //   * the frame is degenerate or longer than a slot — nothing is queued (a partial copy would
  //     put a truncated, CRC-invalid frame on the bus)
  //   * the queue was full — the new frame IS queued, but the oldest one was discarded
  bool push(uint16_t destAddr, const uint8_t *data, uint8_t dataLen) {
    if (data == nullptr || dataLen == 0 || dataLen > RS485B_TX_SLOT_LEN) return false;
    bool dropped = false;
    if (full()) {
      // Drop-oldest: advance head past the stalest frame to free its slot.
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

  // Take the oldest queued frame.
  //
  // Copies it into `out`, which must have room for RS485B_TX_SLOT_LEN bytes, and reports its
  // destination and length. Returns false (leaving the outputs untouched) if the queue is empty.
  // The copy is what lets the caller hand the frame to sendMsgTo()'s buffer, which sits in front
  // of a socket header and so cannot be the queue's own storage.
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
// The bridge never logs per-frame during normal operation — at 28000 baud the serial console
// would itself become the bottleneck — so these counters are the only bring-up diagnostic. They
// are split per failure mode on purpose: "bad start code" points at wiring or a baud mismatch,
// "bad CRC" at line noise or termination, "unsupported" at a command outside the v1 set, and
// "tx-drop" at the loop() rate limit above.
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

  // Bump the bucket matching a validation result. RS485B_OK is accepted and ignored so callers
  // can pass a result through unconditionally; the enum is listed exhaustively (no `default`) so
  // adding a failure mode without a bucket is a compiler warning rather than a silent miscount.
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
