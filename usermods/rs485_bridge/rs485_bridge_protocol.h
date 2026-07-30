#pragma once
//
// rs485_bridge_protocol.h — the pure bridge/framing logic that sits on top of the HMTL wire
// format.
//
// This header is deliberately free of any WLED / Arduino / ArduinoLibs-transport dependencies, so
// it compiles and unit-tests on a host build. The Arduino usermod (usermod_rs485_bridge.{h,cpp})
// includes this and wires it to RS485Socket + WiFiUDP + the segment layer; the host test
// (tests/rs485_bridge_test.cpp) includes only this file.
//
// ---------------------------------------------------------------------------------------------
// Where the wire format comes from
// ---------------------------------------------------------------------------------------------
// It is IMPORTED, not copied: HMTLWireFormat.h in the HMTL submodule
// (../../../HMTL/Libraries/HMTLprotocol/, on the build path via lib_extra_dirs) is the single
// source of truth for msg_hdr_t, the MSG_TYPE_* / MSG_FLAG_* codes, the msg_* payload structs,
// output_hdr_t, config_hdr_t and the HMTL_OUTPUT_* / HMTL_PROGRAM_* codes. That header was
// extracted upstream (aphelps/HMTL, branch rs485-bridge-wire-format) precisely so consumers like
// this one do not have to vendor the declarations: it pulls in only <stdint.h> and Socket.h and
// never Arduino.h, which is why the host test below still builds with a plain host compiler.
//
// Everything defined *here* is bridge-specific and has no HMTL equivalent: the RS485-socket
// bounds constants, the CRC helper, frame validation, the pure receive-path decision function,
// the transmit queue and the counters.
//
#include <stdint.h>
#include <string.h>

#include "HMTLWireFormat.h"   // HMTL submodule — the wire format, imported (see above)

// ---------------------------------------------------------------------------------------------
// RS485 socket layer (mirrors ArduinoLibs RS485Utils' rs485_socket_hdr_t) — size math only.
// ---------------------------------------------------------------------------------------------
// { byte ID; byte length; uint16_t source; uint16_t address; byte flags; } == 7 bytes, on every
// target: the struct is __attribute__((__packed__)) in RS485Utils.h. It has to be. Unpacked it is 7
// bytes on AVR (alignment 1) but 8 on a 2-byte-aligned ABI, and that size is what positions the
// payload at both ends — so this bridge emitted [7 header][1 pad][payload] while a legacy ATMega328
// module read the payload from offset 7, misparsing every frame in both directions. This constant
// read 8 for exactly as long as that bug existed.
//
// Used to bounds-check RS485Socket::getLength() against the payload length before dereferencing
// (RS485Utils' own checks are compiled out in release builds: they sit inside
// `#if DEBUG_LEVEL >= DEBUG_TRACE`). Declared as a literal rather than taken from RS485Utils.h
// because that header pulls in Arduino.h. Two checks keep it honest, and neither is automated:
//   * usermod_rs485_bridge.cpp:19 static_asserts it against the REAL struct — the only check that
//     does — but compiles solely in [env:ampworks], which no CI workflow builds.
//   * tests/rs485_bridge_test.cpp group 1 asserts this value against a packed *replica* of the
//     declaration under both the native and the -fpack-struct=1 ABI. That proves the field list is
//     7 bytes on both, and fails if the packing is dropped; it cannot see the real struct, so a
//     field added there leaves the replica stale and green.
// Keep all three in step by hand.
#define RS485B_SOCKET_HDR_LEN 7

// The object type this bridge reports in a POLL response. Not a stock HMTL object type — it is
// how a master tells a WLED bridge apart from a real HMTL module.
#define HMTL_OBJECT_TYPE_WLED 0x57   // 'W'

// The WLED effect ids this bridge maps HMTL programs onto. Duplicated as literals so this
// header stays dependency-free; usermod_rs485_bridge.cpp static_asserts them against FX.h.
#define RS485B_WLED_MODE_STATIC  0    // FX_MODE_STATIC
#define RS485B_WLED_MODE_BLINK   1    // FX_MODE_BLINK
#define RS485B_WLED_MODE_FADE    12   // FX_MODE_FADE
#define RS485B_WLED_MODE_SPARKLE 20   // FX_MODE_SPARKLE
#define RS485B_WLED_MODE_CHASE   28   // FX_MODE_CHASE_COLOR

// ---------------------------------------------------------------------------------------------
// CRC-8 (poly 0xD8, init 0, MSB-first) — ArduinoLibs' EEPROM_crc, which HMTL uses for msg_hdr.crc
// ---------------------------------------------------------------------------------------------
// Reimplemented here rather than imported because ArduinoLibs' EEPROM_crc lives in
// EEPromUtils.cpp behind EEPROM.h (AVR-only) and is not inline, so neither the ESP32 firmware nor
// the host test can link against it. The algorithm — not a declaration — is what is duplicated,
// and the test below pins it against an independently written reference loop.

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
// The crc byte lives *inside* the region being checksummed (msg_hdr_t::crc, offset 1), so the
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
    uint8_t b = (i == 1) ? 0 : msg[i];   // offset 1 == msg_hdr_t::crc, treated as zero
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
//   address — destination address (SOCKET_ADDR_ANY for everyone)
//   length  — TOTAL frame length, header included (this is what hdr.length carries)
//   type    — MSG_TYPE_*
//   flags   — MSG_FLAG_*
//
// Returns `length` on success, or 0 if `length` is smaller than a header or larger than `buf`.
//
// This is the host-testable equivalent of HMTL's hmtl_msg_fmt(), which cannot be linked here: it
// lives in HMTLMessaging.cpp behind Debug.h and HMTLPrograms.h -> FastLED.h.
//
// The CRC covers the payload as well as the header, so the payload must be in place *before*
// calling this. A caller that fills the payload afterwards has to re-stamp buf[1] itself — see
// sendPollResponse() in usermod_rs485_bridge.cpp, which does exactly that.
static inline uint8_t rs485b_hmtl_fmt(uint8_t *buf, uint16_t bufSize, uint16_t address,
                                      uint8_t length, uint8_t type, uint8_t flags) {
  // Reject both under- and over-sized requests: `length` goes on the wire as the authoritative
  // frame size, so it must describe a real header and must fit the buffer we are writing.
  if (length < sizeof(msg_hdr_t) || bufSize < length) return 0;
  msg_hdr_t h;
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
  if (buf == nullptr || avail < sizeof(msg_hdr_t)) return RS485B_ERR_SHORT;
  msg_hdr_t h;
  memcpy(&h, buf, sizeof(h));          // copy out: `buf` has no alignment guarantee
  if (h.startcode != HMTL_MSG_START)  return RS485B_ERR_START;
  if (h.version   != HMTL_MSG_VERSION) return RS485B_ERR_VERSION;
  // A length below the header or above the protocol maximum is either corruption or a hostile
  // frame; either way it must not become a read bound.
  if (h.length < sizeof(msg_hdr_t) || h.length > HMTL_MAX_MSG_LEN) return RS485B_ERR_LENGTH;
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
  // msg_hdr_t::length is a uint8_t on the wire, so the 255 ceiling sendMsgTo's `byte` length
  // imposes is enforced by the type itself — maxPayload is the only bound left to check. (An
  // explicit `frameLen > 255` used to sit here; g++ -Wtype-limits correctly calls it dead.)
  uint8_t frameLen = buf[3];   // msg_hdr_t::length
  if (frameLen > maxPayload) return RS485B_ERR_OVERSIZE;
  return RS485B_OK;
}

// Is a message with destination `msgAddr` meant for a node listening on `selfAddr`?
// HMTL has no subnet or group concept — a frame is either a unicast to one address or the
// SOCKET_ADDR_ANY (0xFFFF) broadcast every node acts on.
static inline bool rs485b_addressed_to(uint16_t msgAddr, uint16_t selfAddr) {
  return msgAddr == SOCKET_ADDR_ANY || msgAddr == selfAddr;
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

  msg_hdr_t h;
  memcpy(&h, buf, sizeof(h));
  d.destAddress   = h.address;
  d.wantsResponse = (h.flags & MSG_FLAG_RESPONSE) != 0;

  // Frames for other nodes are not errors — the bridge listens promiscuously (RS485_ADDR_ANY) so
  // it can mirror the whole bus back to the WiFi peer. They just get no local handling.
  if (!rs485b_addressed_to(h.address, selfAddr)) { d.action = RS485B_ACT_RELAY_ONLY; return d; }

  const uint8_t *payload    = buf + sizeof(msg_hdr_t);
  // Safe subtraction: validate() guaranteed h.length >= sizeof(msg_hdr_t).
  const uint8_t  payloadLen = (uint8_t)(h.length - sizeof(msg_hdr_t));

  switch (h.type) {
    case MSG_TYPE_OUTPUT: {
      // An OUTPUT message starts with an output header naming the target output and its type.
      if (payloadLen < sizeof(output_hdr_t)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      output_hdr_t oh;
      memcpy(&oh, payload, sizeof(oh));
      d.output = oh.output;            // index, or HMTL_ALL_OUTPUTS; the bridge has one output
      switch (oh.type) {
        case HMTL_OUTPUT_RGB: {
          if (payloadLen < sizeof(msg_rgb_t)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          msg_rgb_t m;
          memcpy(&m, payload, sizeof(m));
          d.rgb[0] = m.values[0]; d.rgb[1] = m.values[1]; d.rgb[2] = m.values[2];
          d.action = RS485B_ACT_SET_RGB;
          return d;
        }
        case HMTL_OUTPUT_VALUE: {
          if (payloadLen < sizeof(msg_value_t)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          msg_value_t m;
          memcpy(&m, payload, sizeof(m));
          d.value  = m.value;          // 13-bit bitfield; widened to uint16_t here
          d.action = RS485B_ACT_SET_VALUE;
          return d;
        }
        case HMTL_OUTPUT_PROGRAM:
        case HMTL_OUTPUT_PIXELS: {
          // msg_program_t is a fixed 35 bytes on the wire, but senders may truncate the unused
          // tail; only the output header + program type byte are required.
          if (payloadLen < sizeof(output_hdr_t) + 1) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
          d.programType = payload[sizeof(output_hdr_t)];
          d.programVals = payload + sizeof(output_hdr_t) + 1;
          d.programLen  = (uint8_t)(payloadLen - sizeof(output_hdr_t) - 1);
          d.action      = RS485B_ACT_PROGRAM;
          return d;
        }
        default:
          // PIXELS-style outputs the bridge has no equivalent for (MPR121, RS485, XBee...).
          d.action = RS485B_ACT_UNSUPPORTED;
          return d;
      }
    }
    case MSG_TYPE_POLL:
      // A poll needs no payload — the caller answers with this node's config block so a master
      // can enumerate the bus.
      d.action = RS485B_ACT_POLL;
      return d;

    case MSG_TYPE_SET_ADDR: {
      if (payloadLen < sizeof(msg_set_addr_t)) { d.action = RS485B_ACT_UNSUPPORTED; return d; }
      msg_set_addr_t m;
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
    case MSG_TYPE_SENSOR:
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

// Step over one msg_sensor_data_t record in a SENSOR payload.
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
//
// This is the bounds-checked counterpart of HMTL's hmtl_next_sensor(), which lives in
// HMTLMessaging.cpp (not linkable here) and does no such checking.
static inline uint8_t rs485b_next_sensor(const uint8_t *data, uint8_t len, uint8_t offset,
                                         uint8_t *outType, const uint8_t **outData,
                                         uint8_t *outLen) {
  // msg_sensor_data_t ends in a flexible array member, so sizeof() is just the two-byte record
  // header — which is exactly the quantity wanted here.
  const size_t recHdr = sizeof(msg_sensor_data_t);
  // Is there even a record header left? (sizeof() makes this size_t arithmetic; the uint8_t
  // operands are far too small to wrap it.)
  if (offset + recHdr > len) return 0;
  // Does the body the header claims actually fit inside the payload?
  const uint8_t sensorType = data[offset];       // msg_sensor_data_t::sensor_type
  const uint8_t dataLen    = data[offset + 1];   // msg_sensor_data_t::data_len
  if (offset + recHdr + dataLen > len) return 0;
  *outType = sensorType;
  *outData = data + offset + recHdr;
  *outLen  = dataLen;
  return (uint8_t)(recHdr + dataLen);
}

// Map an HMTL program id onto the WLED effect id the bridge runs for it.
//
// The v1 mapping is deliberately shallow — the bridge's deliverable is the transport, not a full
// HMTL program emulation. Returns -1 when there is no mapping, which covers two cases the caller
// treats identically (count as unsupported, change nothing):
//   * programs with no WLED analogue (SOUND_VALUE, SEQUENCE, TIMED_CHANGE, ...)
//   * PROGRAM_BRIGHTNESS / PROGRAM_COLOR, which are one-shot property sets rather than
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
  uint32_t rs485Handled = 0;   // frames that reached a v1 action (see handleDecision's caveat)
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

// Compile-time wire-layout guards on the imported HMTL structs.
//
// HMTLWireFormat.h declares every wire struct __attribute__((__packed__)), precisely so an
// ATMega328 module and this ESP32 bridge — which share the bus and read each other's config
// blobs — agree on every size and every field offset. Two structs really did differ before that:
// config_hdr_v2_t was 8 B on AVR with address at offset 3 but 10 B with address at offset 4 on
// 32-bit targets (interior padding), and msg_poll_response_t was 15 B versus 16 B (trailing),
// which made HMTL_MSG_POLL_MIN_LEN 23 or 24 depending on the target. There is therefore no longer
// an "ABI-dependent" struct here to exempt: every number below holds on avr-gcc, Xtensa and
// x86-64 alike, so a single set of constants is correct for both ends of the wire.
//
// What these assertions still buy: they fail the firmware build if an upstream HMTLWireFormat.h
// edit drops the packed attribute, reorders a field, or resizes one — i.e. if interoperability
// with deployed modules breaks. Sizes are asserted here (offsetof is not usable in this header,
// which must stay <stddef.h>-free for the flagless builds); tests/rs485_bridge_test.cpp group 1
// pins the field offsets as well, and HMTL's own platformio/HMTL_Test/test/test_wire_format does
// the same on the AVR side.
static_assert(sizeof(msg_hdr_t)       == 8,  "HMTL msg_hdr_t must be 8 bytes on the wire");
static_assert(sizeof(output_hdr_t)    == 2,  "HMTL output_hdr_t must be 2 bytes on the wire");
static_assert(sizeof(msg_value_t)     == 4,  "HMTL msg_value_t must be 4 bytes on the wire");
static_assert(sizeof(msg_rgb_t)       == 5,  "HMTL msg_rgb_t must be 5 bytes on the wire");
static_assert(sizeof(msg_program_t)   == 35, "HMTL msg_program_t must be 35 bytes on the wire");
static_assert(sizeof(msg_set_addr_t)  == 4,  "HMTL msg_set_addr_t must be 4 bytes on the wire");
static_assert(sizeof(config_hdr_v1_t) == 5,  "HMTL config_hdr_v1_t must be 5 bytes on the wire");
static_assert(sizeof(config_hdr_v2_t) == 8,  "HMTL config_hdr_v2_t must be 8 bytes on the wire");
static_assert(sizeof(config_hdr_v3_t) == 10, "HMTL config_hdr_v3_t must be 10 bytes on the wire");
static_assert(sizeof(msg_sensor_data_t) == 2, "HMTL msg_sensor_data_t header must be 2 bytes");
static_assert(sizeof(msg_sensor_response_t) == 0, "HMTL msg_sensor_response_t must be a bare FAM");
static_assert(sizeof(msg_dumpconfig_response_t) == 0,
              "HMTL msg_dumpconfig_response_t must be a bare FAM");
// The struct that used to be exempt, now pinned like the rest: 15 B, so the POLL response the
// bridge emits is 23 B — byte-for-byte what a legacy AVR master builds and length-checks.
static_assert(sizeof(msg_poll_response_t) == 15,
              "HMTL msg_poll_response_t must be 15 bytes on the wire (packed, AVR-identical)");
static_assert(HMTL_MSG_POLL_MIN_LEN == 23, "HMTL POLL response must be 23 bytes on every target");
