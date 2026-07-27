#include "usermod_rs485_bridge.h"

#if RS485_BRIDGE_BUILD

// The protocol header carries the WLED effect ids as literals so it stays dependency-free.
// If FX.h ever renumbers, fail the build here rather than silently mapping HMTL programs to the
// wrong effect.
static_assert(RS485B_WLED_MODE_STATIC  == FX_MODE_STATIC,      "FX_MODE_STATIC renumbered");
static_assert(RS485B_WLED_MODE_BLINK   == FX_MODE_BLINK,       "FX_MODE_BLINK renumbered");
static_assert(RS485B_WLED_MODE_FADE    == FX_MODE_FADE,        "FX_MODE_FADE renumbered");
static_assert(RS485B_WLED_MODE_SPARKLE == FX_MODE_SPARKLE,     "FX_MODE_SPARKLE renumbered");
static_assert(RS485B_WLED_MODE_CHASE   == FX_MODE_CHASE_COLOR, "FX_MODE_CHASE_COLOR renumbered");

// A queue slot must be able to hold the largest frame the socket can carry.
static_assert(RS485B_TX_SLOT_LEN <= RS485_RECV_BUFFER,
              "TX slot larger than the RS485 socket data buffer");

// The protocol header hardcodes the socket header size for its bounds checks.
static_assert(RS485B_SOCKET_HDR_LEN == sizeof(rs485_socket_hdr_t),
              "RS485B_SOCKET_HDR_LEN out of sync with RS485Utils' rs485_socket_hdr_t");

const char UsermodRS485Bridge::_name[]    PROGMEM = "RS485Bridge";
const char UsermodRS485Bridge::_enabled[] PROGMEM = "enabled";

// ---------------------------------------------------------------------------------------------
// setup / teardown
// ---------------------------------------------------------------------------------------------
uint16_t UsermodRS485Bridge::effectiveDeviceId() const {
  if (deviceId) return deviceId;
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  return (uint16_t)((mac[4] << 8) | mac[5]);
}

bool UsermodRS485Bridge::allocatePins() {
  if (rxPin < 0 || txPin < 0 || enPin < 0) { failReason = PSTR("pins unset"); return false; }
  // GPIO 16/17 are the WROVER PSRAM pins: PinManager::isPinOk() rejects them whenever PSRAM is
  // present, so the RS485 defaults (RX16/TX17) must be reconfigured on such boards.
  if (!PinManager::isPinOk(rxPin, false) || !PinManager::isPinOk(txPin, true) ||
      !PinManager::isPinOk(enPin, true)) {
    failReason = PSTR("pin not usable (PSRAM board? move off GPIO 16/17)");
    return false;
  }
  const managed_pin_type pins[] = { { rxPin, false }, { txPin, true }, { enPin, true } };
  // All-or-nothing: never begin() the UART on a half-allocated pin set.
  if (!PinManager::allocateMultiplePins(pins, 3, PinOwner::UM_RS485_BRIDGE)) {
    failReason = PSTR("pin already in use");
    return false;
  }
  return true;
}

void UsermodRS485Bridge::releasePins() {
  const managed_pin_type pins[] = { { rxPin, false }, { txPin, true }, { enPin, true } };
  PinManager::deallocateMultiplePins(pins, 3, PinOwner::UM_RS485_BRIDGE);
}

void UsermodRS485Bridge::setup() {
  initDone = true;
  if (!enabled) return;
  if (uartNum < 1 || uartNum >= SOC_UART_NUM) { failReason = PSTR("invalid UART"); return; }
  if (!allocatePins()) {
    DEBUG_PRINTF_P(PSTR("[RS485Bridge] disabled: %s\n"), failReason);
    return;
  }

  port = new HardwareSerial(uartNum);
  port->begin(baud, SERIAL_8N1, rxPin, txPin);

  // Hand the already-open port to RS485Socket. The (recvPin, xmitPin, ...) overload is
  // deliberately avoided: on ESP32 it hardcodes DEFAULT_BAUD and takes the UART from the
  // compile-time RS485_HARDWARE_SERIAL macro, which makes runtime baud/UART config impossible.
  // RS485Socket::setup() skips serial->begin() under ESP32, so it will not undo the settings.
  rs485.init(port, (byte)enPin, address, RS485_RECV_BUFFER, false);
  rs485.setup();
  txData = rs485.initBuffer(rawTxBuf, sizeof(rawTxBuf));

  running    = true;
  failReason = nullptr;
  DEBUG_PRINTF_P(PSTR("[RS485Bridge] UART%u rx=%d tx=%d en=%d baud=%u addr=%u udp=%u\n"),
                 uartNum, rxPin, txPin, enPin, (unsigned)baud, address, udpPort);
}

void UsermodRS485Bridge::connected() {
  udpStarted = false;   // rebind on (re)connection
}

// ---------------------------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------------------------
void UsermodRS485Bridge::loop() {
  if (!running || strip.isUpdating()) return;
  serviceRs485();
  serviceTx();
  serviceUdp();
}

void UsermodRS485Bridge::serviceRs485() {
  for (uint8_t i = 0; i < RX_PER_LOOP; i++) {
    unsigned int retlen = 0;
    // RS485_ADDR_ANY: take every frame off the bus, then decide locally whether it is ours.
    // Frames for other nodes are still useful — they are relayed back to the WiFi peer.
    const byte *data = rs485.getMsg(RS485_ADDR_ANY, &retlen);
    if (data == nullptr) return;

    counters.rs485Rx++;

    // RS485Utils' own length checks live inside `#if DEBUG_LEVEL >= DEBUG_TRACE` and are compiled
    // out of release builds, so validate here before dereferencing anything.
    const uint8_t socketLen = rs485.getLength();
    if (socketLen < RS485B_SOCKET_HDR_LEN ||
        (uint16_t)retlen + RS485B_SOCKET_HDR_LEN > (uint16_t)socketLen ||
        retlen == 0 || retlen > RS485_RECV_BUFFER) {
      counters.countError(RS485B_ERR_SHORT);
      continue;
    }

    const uint16_t sourceAddr = rs485.sourceFromData((void *)data);
    RS485BDecision d = rs485b_decide(data, (uint16_t)retlen, address, effectiveDeviceId());
    handleDecision(d, data, (uint8_t)retlen, sourceAddr);
  }
}

void UsermodRS485Bridge::serviceTx() {
  // At most one frame per loop(): sendMsgTo busy-waits on serial->flush() while holding the
  // driver-enable pin high (~48 ms for a 64-byte payload at 28000 baud with byte-stuffing).
  if (txQueue.empty() || txData == nullptr) return;
  uint16_t dest = 0;
  uint8_t  len  = 0;
  uint8_t  frame[RS485B_TX_SLOT_LEN];
  if (!txQueue.pop(&dest, frame, &len)) return;
  memcpy(txData, frame, len);
  rs485.sendMsgTo(dest, txData, len);
  counters.rs485Tx++;
}

void UsermodRS485Bridge::serviceUdp() {
  if (!Network.isConnected()) return;
  if (!udpStarted) {
    if (!udp.begin(udpPort)) return;
    udpStarted = true;
  }
  for (uint8_t i = 0; i < UDP_PER_LOOP; i++) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;
    uint8_t buf[UDP_BUF_LEN];
    int len = udp.read(buf, sizeof(buf));
    // Remember who spoke last so RS485 responses can be relayed back to them.
    peerIp   = udp.remoteIP();
    peerPort = udp.remotePort();
    counters.udpRx++;
    if (len <= 0) { counters.udpRejected++; continue; }

    RS485BFrameResult r = rs485b_validate_udp_ingress(buf, (uint16_t)len, RS485B_TX_SLOT_LEN);
    if (r != RS485B_OK) {
      counters.countError(r);
      counters.udpRejected++;
      continue;
    }
    HmtlMsgHdr h;
    memcpy(&h, buf, sizeof(h));
    if (!sendHmtlFrame(h.address, buf, h.length)) counters.txDropped++;
  }
}

bool UsermodRS485Bridge::sendHmtlFrame(uint16_t destAddr, const uint8_t *frame, uint8_t frameLen) {
  if (!running) return false;
  return txQueue.push(destAddr, frame, frameLen);
}

// ---------------------------------------------------------------------------------------------
// Slave path — act on frames addressed to this node
// ---------------------------------------------------------------------------------------------
void UsermodRS485Bridge::handleDecision(const RS485BDecision &d, const uint8_t *frame,
                                        uint8_t frameLen, uint16_t sourceAddr) {
  switch (d.action) {
    case RS485B_ACT_DROP:
      counters.countError(d.err);
      return;

    case RS485B_ACT_RELAY_ONLY:
      relayToPeer(frame, frameLen);
      return;

    case RS485B_ACT_UNSUPPORTED:
      counters.unsupported++;
      relayToPeer(frame, frameLen);
      return;

    case RS485B_ACT_SET_RGB:
      applyRgb(d.rgb);
      break;

    case RS485B_ACT_SET_VALUE:
      applyValue(d.value);
      break;

    case RS485B_ACT_PROGRAM:
      applyProgram(d);
      break;

    case RS485B_ACT_POLL:
      sendPollResponse(sourceAddr);
      break;

    case RS485B_ACT_SET_ADDRESS:
      // The address is the one setting that can change without re-init(): getMsg()/sendMsgTo()
      // read it straight out of RS485Socket::sourceAddress.
      address = d.newAddress;
      rs485.sourceAddress = address;
      serializeConfigToFS();
      break;

    case RS485B_ACT_SENSOR:
      // v1: sensor broadcasts are relayed to the WiFi peer and counted. Feeding them into the
      // SensorSync bus is the sensorsync-rs485-transport follow-up.
      relayToPeer(frame, frameLen);
      break;
  }
  counters.rs485Handled++;
}

void UsermodRS485Bridge::applyRgb(const uint8_t rgb[3]) {
  Segment &seg = strip.getMainSegment();
  seg.setColor(0, RGBW32(rgb[0], rgb[1], rgb[2], 0));
  seg.setMode(FX_MODE_STATIC);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

void UsermodRS485Bridge::applyValue(uint16_t value) {
  // HMTL VALUE drives all three channels to the same level (hmtl_handle_output_msg), i.e. a
  // white level — mapped here onto WLED's master brightness. The 13-bit field is clamped.
  bri = (uint8_t)(value > 255 ? 255 : value);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

void UsermodRS485Bridge::applyProgram(const RS485BDecision &d) {
  Segment &seg = strip.getMainSegment();
  switch (d.programType) {
    case HMTL_PROGRAM_BRIGHTNESS:
      if (d.programLen < 1) { counters.unsupported++; return; }
      bri = d.programVals[0];
      break;
    case HMTL_PROGRAM_COLOR:
      if (d.programLen < 3) { counters.unsupported++; return; }
      seg.setColor(0, RGBW32(d.programVals[0], d.programVals[1], d.programVals[2], 0));
      break;
    default: {
      int mode = rs485b_program_to_mode(d.programType);
      if (mode < 0) { counters.unsupported++; return; }   // no v1 mapping: count and ignore
      seg.setMode((uint8_t)mode);
      break;
    }
  }
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

void UsermodRS485Bridge::sendPollResponse(uint16_t to) {
  const uint8_t len = (uint8_t)(sizeof(HmtlMsgHdr) + sizeof(HmtlMsgPollResponse));
  uint8_t frame[sizeof(HmtlMsgHdr) + sizeof(HmtlMsgPollResponse)];
  if (!rs485b_hmtl_fmt(frame, sizeof(frame), address, len, HMTL_MSG_TYPE_POLL,
                       HMTL_MSG_FLAG_ACK)) return;

  HmtlMsgPollResponse resp;
  memset(&resp, 0, sizeof(resp));
  resp.config.magic            = HMTL_CONFIG_MAGIC;
  resp.config.protocol_version = HMTL_CONFIG_VERSION;
  resp.config.hardware_version = 1;
  resp.config.baud             = HMTL_BAUD_TO_BYTE(baud);
  resp.config.num_outputs      = 1;
  resp.config.flags            = 0;
  resp.config.device_id        = effectiveDeviceId();
  resp.config.address          = address;
  resp.object_type             = HMTL_OBJECT_TYPE_WLED;
  resp.recv_buffer_size        = RS485_RECV_BUFFER;
  resp.msg_version             = HMTL_MSG_VERSION;
  memcpy(frame + sizeof(HmtlMsgHdr), &resp, sizeof(resp));
  frame[1] = rs485b_hmtl_crc(frame, len);   // re-stamp: the payload changed after fmt()

  if (!sendHmtlFrame(to, frame, len)) counters.txDropped++;
}

void UsermodRS485Bridge::relayToPeer(const uint8_t *frame, uint8_t frameLen) {
  if (!udpStarted || peerPort == 0 || frameLen == 0) return;
  if (udp.beginPacket(peerIp, peerPort) != 1) return;
  udp.write(frame, frameLen);
  udp.endPacket();
  counters.rs485Relayed++;
}

// ---------------------------------------------------------------------------------------------
// Info / config
// ---------------------------------------------------------------------------------------------
void UsermodRS485Bridge::addToJsonInfo(JsonObject &root) {
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");

  JsonArray state = user.createNestedArray(F("RS485 Bridge"));
  if (!enabled) {
    state.add(F("disabled"));
  } else if (!running) {
    state.add(F("error: "));
    state.add(failReason ? failReason : PSTR("not initialised"));
  } else {
    state.add(address);
    state.add(F(" (addr)"));

    JsonArray rx = user.createNestedArray(F("RS485 rx/tx"));
    rx.add(counters.rs485Rx);
    char buf[32];
    snprintf_P(buf, sizeof(buf), PSTR(" / %u"), (unsigned)counters.rs485Tx);
    rx.add(buf);

    JsonArray udpInfo = user.createNestedArray(F("RS485 udp in/relayed"));
    udpInfo.add(counters.udpRx);
    char buf2[32];
    snprintf_P(buf2, sizeof(buf2), PSTR(" / %u"), (unsigned)counters.rs485Relayed);
    udpInfo.add(buf2);

    const uint32_t errs = counters.errShort + counters.errStart + counters.errVersion +
                          counters.errLength + counters.errCrc + counters.errOversize;
    if (errs || counters.unsupported || counters.txDropped || counters.udpRejected) {
      JsonArray bad = user.createNestedArray(F("RS485 dropped"));
      bad.add(errs);
      char buf3[64];
      snprintf_P(buf3, sizeof(buf3), PSTR(" bad, %u unsupported, %u tx-drop, %u udp-rej"),
                 (unsigned)counters.unsupported, (unsigned)counters.txDropped,
                 (unsigned)counters.udpRejected);
      bad.add(buf3);
    }
  }
}

void UsermodRS485Bridge::addToConfig(JsonObject &root) {
  JsonObject top = root.createNestedObject(FPSTR(_name));
  top[FPSTR(_enabled)] = enabled;
  top["uart"]  = uartNum;
  top["rx"]    = rxPin;
  top["tx"]    = txPin;
  top["en"]    = enPin;
  top["baud"]  = baud;
  top["addr"]  = address;
  top["devId"] = deviceId;
  top["port"]  = udpPort;
}

bool UsermodRS485Bridge::readFromConfig(JsonObject &root) {
  JsonObject top = root[FPSTR(_name)];
  if (top.isNull()) return false;
  getJsonValue(top[FPSTR(_enabled)], enabled);
  getJsonValue(top["uart"],  uartNum);
  getJsonValue(top["rx"],    rxPin);
  getJsonValue(top["tx"],    txPin);
  getJsonValue(top["en"],    enPin);
  getJsonValue(top["baud"],  baud);
  getJsonValue(top["addr"],  address);
  getJsonValue(top["devId"], deviceId);
  getJsonValue(top["port"],  udpPort);
  if (initDone && running) {
    // UART / pin / baud changes need a reboot (RS485Utils leaks on re-init); the socket address
    // and the UDP port can be applied live.
    rs485.sourceAddress = address;
    udpStarted = false;
  }
  return true;
}

// WLED 16.x self-registration.
static UsermodRS485Bridge rs485_bridge_usermod;
REGISTER_USERMOD(rs485_bridge_usermod);

#else  // !RS485_BRIDGE_BUILD

// Builds without -D USERMOD_RS485_BRIDGE / -D RS485_HARDWARE_SERIAL (the `[env:usermods]`
// wildcard env and the per-usermod CI matrix) guard the bridge itself out — see the comment on
// RS485_BRIDGE_BUILD in usermod_rs485_bridge.h. The pure protocol header has no such
// dependencies, so it is still compiled here: its static_asserts are the wire-layout regression
// check, and they give this usermod real coverage in those builds.
//
// An inert placeholder usermod stands in for the real one, for two reasons:
//   * pio-scripts/validate_modules.py fails the build for any enrolled usermod that contributes
//     no DWARF compilation unit to the linked ELF, and a translation unit whose only symbol is
//     unreferenced is dropped by --gc-sections, leaving no CU. REGISTER_USERMOD's pointer lands
//     in the KEEP'd .dynarray.usermods section, so the CU survives.
//   * It makes the situation legible at runtime instead of the usermod silently not existing.
#include "rs485_bridge_protocol.h"

class UsermodRS485BridgeUnavailable : public Usermod {
 public:
  void setup() override {}
  void loop() override {}
  uint16_t getId() override { return USERMOD_ID_RS485_BRIDGE; }
  void addToJsonInfo(JsonObject &root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray state = user.createNestedArray(F("RS485 Bridge"));
    state.add(F("not built"));
    state.add(F(" (needs -D USERMOD_RS485_BRIDGE -D RS485_HARDWARE_SERIAL on ESP32)"));
  }
};

static UsermodRS485BridgeUnavailable rs485_bridge_usermod_unavailable;
REGISTER_USERMOD(rs485_bridge_usermod_unavailable);

#endif  // RS485_BRIDGE_BUILD
