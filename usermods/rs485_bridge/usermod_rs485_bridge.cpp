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
// This node's HMTL device id — the identity SET_ADDRESS and POLL responses are keyed on.
//
// A device id must be stable across reboots and unique on the bus, but asking the user to invent
// one for every board is friction. So a configured non-zero value wins, and 0 means "derive it":
// the low two bytes of the WiFi MAC are unique per ESP32 and survive reflashing.
uint16_t UsermodRS485Bridge::effectiveDeviceId() const {
  if (deviceId) return deviceId;
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  return (uint16_t)((mac[4] << 8) | mac[5]);
}

// Reserve the RS485 GPIOs with WLED's PinManager. Returns true only if all three are ours.
//
// WLED shares its GPIOs between LED buses, buttons, relays, IR and usermods, and PinManager is
// the only thing that knows who holds what — an unregistered pin is one the LED-bus settings page
// will happily hand to a strip, after which two drivers fight over it. Registering also makes the
// conflict visible in the UI instead of appearing as mysteriously dead output.
//
// On failure `failReason` is set for the info page and nothing is allocated, so setup() can leave
// the bridge cleanly disabled.
bool UsermodRS485Bridge::allocatePins() {
  if (rxPin < 0 || txPin < 0 || enPin < 0) { failReason = PSTR("pins unset"); return false; }
  // isPinOk() screens for pins that exist and are safe to use in the requested direction (the
  // second argument is "used as output"). The one that bites here: GPIO 16/17 are the WROVER
  // PSRAM pins, and isPinOk() returns !psramFound() for them (pin_manager.cpp:249-254). The
  // documented RS485 defaults of RX16/TX17 therefore fail on any PSRAM board, which is why the
  // pins are user-configurable and the readme calls this out rather than the code trying to
  // pick alternatives on the board's behalf.
  if (!PinManager::isPinOk(rxPin, false) || !PinManager::isPinOk(txPin, true) ||
      !PinManager::isPinOk(enPin, true)) {
    failReason = PSTR("pin not usable (PSRAM board? move off GPIO 16/17)");
    return false;
  }
  const managed_pin_type pins[] = { { rxPin, false }, { txPin, true }, { enPin, true } };
  // allocateMultiplePins() is all-or-nothing — it rolls back its own partial allocations — and
  // that matters: a UART begun on a half-owned pin set would drive a line another peripheral
  // believes it owns, and there would be no clean way to give the pins back.
  if (!PinManager::allocateMultiplePins(pins, 3, PinOwner::UM_RS485_BRIDGE)) {
    failReason = PSTR("pin already in use");
    return false;
  }
  return true;
}

// Hand the RS485 GPIOs back to PinManager.
//
// Only meaningful while the pins are actually held (running == true); PinManager ignores a
// deallocation whose owner does not match, so calling it otherwise cannot steal another
// subsystem's pins.
void UsermodRS485Bridge::releasePins() {
  const managed_pin_type pins[] = { { rxPin, false }, { txPin, true }, { enPin, true } };
  PinManager::deallocateMultiplePins(pins, 3, PinOwner::UM_RS485_BRIDGE);
}

// Bring the bridge up: claim the pins, open the UART, and attach the RS485 socket.
//
// This is the ONLY place HardwareSerial::begin() and RS485Socket::init() are ever called.
// RS485Utils keeps its serial pointer in a file-scope global and RS485Socket::init_general()
// unconditionally re-news the channel without freeing the previous one, so a re-init at runtime
// leaks both the RS485 object and the HardwareSerial. Hence the v1 policy: UART / pin / baud /
// enable-pin changes take effect on the next boot only (see readFromConfig()).
//
// Any failure leaves `running` false with `failReason` set, so the usermod is inert rather than
// half-initialised and the reason shows up in /json/info.
void UsermodRS485Bridge::setup() {
  initDone = true;
  if (!enabled) return;
  // UART 0 is the console/programming port — taking it over would cut off serial debugging and
  // break Adalight/TPM2. SOC_UART_NUM is the per-chip count, so this also rejects UART 2 on the
  // C3/S2 parts that do not have one.
  if (uartNum < 1 || uartNum >= SOC_UART_NUM) { failReason = PSTR("invalid UART"); return; }
  if (!allocatePins()) {
    DEBUG_PRINTF_P(PSTR("[RS485Bridge] disabled: %s\n"), failReason);
    return;
  }

  // Own the port ourselves so baud and pins come from config rather than compile-time macros.
  port = new HardwareSerial(uartNum);
  port->begin(baud, SERIAL_8N1, rxPin, txPin);

  // Hand the already-open port to RS485Socket. The (recvPin, xmitPin, ...) overload is
  // deliberately avoided: on ESP32 it hardcodes DEFAULT_BAUD and takes the UART from the
  // compile-time RS485_HARDWARE_SERIAL macro, which makes runtime baud/UART config impossible.
  // RS485Socket::setup() skips serial->begin() under ESP32, so it will not undo the settings.
  rs485.init(port, (byte)enPin, address, RS485_RECV_BUFFER, false);
  rs485.setup();
  // initBuffer() carves rawTxBuf into [socket header][data] and returns a pointer to the data
  // region. sendMsgTo() writes its header immediately in front of the pointer it is given, so
  // transmissions must go through this pointer and no other.
  txData = rs485.initBuffer(rawTxBuf, sizeof(rawTxBuf));

  running    = true;
  failReason = nullptr;
  DEBUG_PRINTF_P(PSTR("[RS485Bridge] UART%u rx=%d tx=%d en=%d baud=%u addr=%u udp=%u\n"),
                 uartNum, rxPin, txPin, enPin, (unsigned)baud, address, udpPort);
}

// WLED calls this whenever the WiFi link comes up (including after a reconnect).
// The UDP socket is bound to an interface that no longer exists after a reconnect, so it is
// marked for rebinding rather than used blind — serviceUdp() re-begin()s it on the next pass.
void UsermodRS485Bridge::connected() {
  udpStarted = false;   // rebind on (re)connection
}

// ---------------------------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------------------------
// Per-iteration service of both directions, in receive → transmit → ingest order. That order is
// not load-bearing for the rate limit — serviceTx()'s one-frame-per-call budget is, and it is
// called exactly once per pass — it just means a frame ingested this pass waits until the next
// one to go out, costing a single iteration of latency.
//
// The strip.isUpdating() bail-out matters: the LED drivers are timing-critical while shifting
// data out, and this usermod's transmit step blocks for tens of milliseconds. Yielding the whole
// iteration is cheap — RS485 traffic is not latency-sensitive at 28000 baud — and it keeps the
// bridge from ever being the cause of a visible glitch.
void UsermodRS485Bridge::loop() {
  if (!running || strip.isUpdating()) return;
  serviceRs485();
  serviceTx();
  serviceUdp();
}

// Slave path, receive side: drain up to RX_PER_LOOP frames off the RS485 bus and act on them.
//
// Receiving is cheap (RS485Utils' reader is non-blocking and the frames are tens of bytes), so a
// small batch per loop() keeps the bus from backing up without risking the watchdog. The loop
// returns rather than breaks on the first empty read — there is nothing more to drain.
void UsermodRS485Bridge::serviceRs485() {
  for (uint8_t i = 0; i < RX_PER_LOOP; i++) {
    unsigned int retlen = 0;
    // RS485_ADDR_ANY: take every frame off the bus, then decide locally whether it is ours.
    // Frames for other nodes are still useful — they are relayed back to the WiFi peer, which is
    // what lets a WiFi client observe replies from the legacy modules it commanded.
    const byte *data = rs485.getMsg(RS485_ADDR_ANY, &retlen);
    if (data == nullptr) return;

    counters.rs485Rx++;

    // Bounds-check the socket layer before trusting `data`/`retlen`. RS485Utils does have its own
    // length checks, but they sit inside `#if DEBUG_LEVEL >= DEBUG_TRACE` and so are compiled out
    // of exactly the release builds this ships as. `retlen` is the payload length the sender
    // declared and `getLength()` is the total frame length actually received, so the three tests
    // are: the frame is at least a socket header; the declared payload plus that header does not
    // claim more than was received; and the payload is neither empty nor larger than the receive
    // buffer it supposedly came out of. Only after all three does the payload become safe to read.
    const uint8_t socketLen = rs485.getLength();
    if (socketLen < RS485B_SOCKET_HDR_LEN ||
        (uint16_t)retlen + RS485B_SOCKET_HDR_LEN > (uint16_t)socketLen ||
        retlen == 0 || retlen > RS485_RECV_BUFFER) {
      counters.countError(RS485B_ERR_SHORT);
      continue;
    }

    // The socket-layer source address is where a reply has to go; the HMTL header's address is
    // the destination and cannot be used for that.
    const uint16_t sourceAddr = rs485.sourceFromData((void *)data);
    RS485BDecision d = rs485b_decide(data, (uint16_t)retlen, address, effectiveDeviceId());
    handleDecision(d, data, (uint8_t)retlen, sourceAddr);
  }
}

// Master path, transmit side: write at most ONE queued frame to the bus per loop() iteration.
//
// The one-per-iteration cap is the rate limit that makes the bridge safe to run alongside the LED
// engine. sendMsgTo() holds the driver-enable pin high and busy-waits on serial->flush() until
// the frame has physically left the UART — it cannot release DE earlier without truncating the
// frame on a half-duplex bus — which is ~48 ms for a 64-byte payload at 28000 baud once Gammon's
// byte-stuffing is counted. Transmitting the whole queue here would multiply that by four and
// stall the strip refresh (and, with a sustained UDP flood, trip the task watchdog). Everything
// else waits in RS485BTxQueue, which drops oldest-first if the sender outruns the wire.
void UsermodRS485Bridge::serviceTx() {
  if (txQueue.empty() || txData == nullptr) return;
  uint16_t dest = 0;
  uint8_t  len  = 0;
  uint8_t  frame[RS485B_TX_SLOT_LEN];
  if (!txQueue.pop(&dest, frame, &len)) return;
  // The frame has to be copied into the socket's own buffer: sendMsgTo() writes the RS485 socket
  // header into the bytes immediately preceding `txData`, which only rawTxBuf has room for.
  memcpy(txData, frame, len);
  rs485.sendMsgTo(dest, txData, len);   // blocks until the frame is on the wire — see above
  counters.rs485Tx++;
}

// Master path, ingest side: accept HMTL frames arriving as raw UDP datagrams and queue them for
// the bus. Handles up to UDP_PER_LOOP datagrams per iteration.
//
// Binding is lazy rather than done in setup(): WiFi may not be up yet at usermod setup time, and
// connected() clears udpStarted so a reconnect rebinds.
void UsermodRS485Bridge::serviceUdp() {
  if (!Network.isConnected()) return;
  if (!udpStarted) {
    if (!udp.begin(udpPort)) return;   // retry on the next pass rather than giving up
    udpStarted = true;
  }
  for (uint8_t i = 0; i < UDP_PER_LOOP; i++) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;       // nothing pending
    // Reading into a fixed UDP_BUF_LEN buffer truncates anything larger, which is intentional:
    // nothing bigger than a transmit slot can be forwarded anyway, and the truncated frame then
    // fails validation below instead of being copied somewhere it does not fit.
    uint8_t buf[UDP_BUF_LEN];
    int len = udp.read(buf, sizeof(buf));
    // Remember who spoke last so RS485 responses can be relayed back to them. Recorded before
    // validation so that even a malformed sender stays reachable for subsequent replies.
    peerIp   = udp.remoteIP();
    peerPort = udp.remotePort();
    counters.udpRx++;
    if (len <= 0) { counters.udpRejected++; continue; }

    // Validate before the frame can reach sendMsgTo(): this is the check that keeps an oversized
    // datagram from overflowing the socket transmit buffer (see rs485b_validate_udp_ingress).
    RS485BFrameResult r = rs485b_validate_udp_ingress(buf, (uint16_t)len, RS485B_TX_SLOT_LEN);
    if (r != RS485B_OK) {
      counters.countError(r);
      counters.udpRejected++;
      continue;
    }
    // Forward using the frame's OWN destination address — the bridge is a transport here and
    // does not rewrite addressing. h.length (not `len`) is the authoritative frame size, so any
    // trailing padding in the datagram is not put on the wire.
    msg_hdr_t h;
    memcpy(&h, buf, sizeof(h));
    if (!sendHmtlFrame(h.address, buf, h.length)) counters.txDropped++;
  }
}

// Queue an HMTL frame for transmission on the RS485 bus.
//
// The single entry point to the transmit queue, used by both the UDP ingest path and the
// locally-generated poll responses. Returns false if the bridge is not running, if the frame does
// not fit a queue slot, or if an older queued frame had to be dropped to make room — callers
// treat all three as a dropped transmission.
bool UsermodRS485Bridge::sendHmtlFrame(uint16_t destAddr, const uint8_t *frame, uint8_t frameLen) {
  if (!running) return false;
  return txQueue.push(destAddr, frame, frameLen);
}

// ---------------------------------------------------------------------------------------------
// Slave path — act on frames addressed to this node
// ---------------------------------------------------------------------------------------------
// Execute the decision rs485b_decide() reached for one received frame.
//
// The split is deliberate: rs485b_decide() is pure and host-tested, this function is the only
// part that touches WLED state or the bus. `frame`/`frameLen` are passed through because the
// relay cases forward the original bytes unmodified, and `sourceAddr` is the socket-layer sender
// (the HMTL header carries the destination, not the source, so replies need this).
//
// Note the counting policy: DROP/RELAY_ONLY/UNSUPPORTED return early and are NOT counted as
// handled — rs485Handled counts frames that reached a v1 action. Caveat: a PROGRAM frame whose
// program is unmapped or truncated bails inside applyProgram() (counted unsupported) but is
// still counted handled here, and SENSOR frames are only relayed.
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
      // Well-formed and ours, but outside the v1 command set. Still relayed: a WiFi client may
      // understand HMTL commands this bridge does not.
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
      // read it straight out of the public RS485Socket::sourceAddress on every call, so writing
      // it takes effect immediately. Persisted right away because HMTL commissioning assigns an
      // address once and expects it to survive the next power cycle.
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

// Apply an HMTL RGB command to the local LEDs.
//
// HMTL modules drive a single physical output, so the closest WLED equivalent is the main
// segment. The mode is forced to STATIC because an RGB command means "be this colour now" — left
// under a running effect the colour change would be invisible or immediately overwritten.
void UsermodRS485Bridge::applyRgb(const uint8_t rgb[3]) {
  Segment &seg = strip.getMainSegment();
  seg.setColor(0, RGBW32(rgb[0], rgb[1], rgb[2], 0));
  seg.setMode(FX_MODE_STATIC);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);   // notify the UI/MQTT/sync like any other state change
}

// Apply an HMTL VALUE command to the local LEDs.
//
// HMTL VALUE drives all three channels of an output to the same level (hmtl_handle_output_msg),
// i.e. a white level — mapped here onto WLED's master brightness, which is the closest thing a
// multi-segment strip has to "one output's level". The wire field is 13 bits (0..8191) while
// WLED brightness is 8, so values above 255 saturate rather than wrap.
void UsermodRS485Bridge::applyValue(uint16_t value) {
  bri = (uint8_t)(value > 255 ? 255 : value);
  stateUpdated(CALL_MODE_DIRECT_CHANGE);
}

// Apply an HMTL PROGRAM command.
//
// Two of HMTL's programs are one-shot property sets rather than modes and are handled directly;
// everything else goes through rs485b_program_to_mode(), whose small v1 table is documented in
// the readme. Programs with no mapping, and programs whose sender truncated the values this
// bridge needs, are counted as unsupported and change nothing — a partial application would be
// worse than none, since the operator would see *something* happen and assume the command worked.
void UsermodRS485Bridge::applyProgram(const RS485BDecision &d) {
  Segment &seg = strip.getMainSegment();
  switch (d.programType) {
    case PROGRAM_BRIGHTNESS:
      if (d.programLen < 1) { counters.unsupported++; return; }
      bri = d.programVals[0];
      break;
    case PROGRAM_COLOR:
      if (d.programLen < 3) { counters.unsupported++; return; }   // needs a full RGB triple
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

// Answer an HMTL POLL by describing this node, so a master can enumerate the bus.
//
//   to — the socket-layer address the poll came from
//
// The payload is HMTL's config_hdr_v3_t plus the poll extras, which is what legacy masters parse.
// Fields are reported honestly where an equivalent exists (baud, address, device id) and pinned
// to a minimal truth where it does not: one output, hardware version 1, and a non-stock object
// type so a master can tell a WLED bridge from a real HMTL module.
//
// The length is HMTL's own HMTL_MSG_POLL_MIN_LEN, i.e. exactly what hmtl_poll_fmt() puts on the
// wire for this target: 24 bytes here, where uint16_t alignment adds a trailing pad byte to
// msg_poll_response_t, versus 23 on AVR. Only trailing padding differs, so every field lands at
// the same offset for a legacy master either way.
void UsermodRS485Bridge::sendPollResponse(uint16_t to) {
  const uint8_t len = (uint8_t)HMTL_MSG_POLL_MIN_LEN;
  uint8_t frame[HMTL_MSG_POLL_MIN_LEN];
  // ACK marks this as a response to a poll rather than a poll of our own.
  if (!rs485b_hmtl_fmt(frame, sizeof(frame), address, len, MSG_TYPE_POLL,
                       MSG_FLAG_ACK)) return;

  msg_poll_response_t resp;
  memset(&resp, 0, sizeof(resp));   // zero first: every reserved/unused field goes out as 0
  resp.config.magic            = HMTL_CONFIG_MAGIC;
  resp.config.protocol_version = HMTL_CONFIG_VERSION;
  resp.config.hardware_version = 1;
  resp.config.baud             = BAUD_TO_BYTE(baud);
  resp.config.num_outputs      = 1;   // the bridge presents the strip as a single HMTL output
  resp.config.flags            = 0;
  resp.config.device_id        = effectiveDeviceId();
  resp.config.address          = address;
  resp.object_type             = HMTL_OBJECT_TYPE_WLED;
  resp.recv_buffer_size        = RS485_RECV_BUFFER;
  resp.msg_version             = HMTL_MSG_VERSION;
  memcpy(frame + sizeof(msg_hdr_t), &resp, sizeof(resp));
  // The CRC covers header AND payload, but rs485b_hmtl_fmt() ran before the payload was written,
  // so its stamp is stale. Re-stamp now or the response fails validation at any CRC-checking
  // receiver.
  frame[1] = rs485b_hmtl_crc(frame, len);

  // Goes through the normal transmit queue rather than straight to the wire: a poll response is
  // subject to the same one-frame-per-loop() rate limit as anything else.
  if (!sendHmtlFrame(to, frame, len)) counters.txDropped++;
}

// Forward a frame received on RS485 to the last WiFi peer that talked to the bridge.
//
// The bridge tracks a single peer rather than a subscriber list: the WiFi side is a
// request/response client (send a command, read the replies), so "whoever spoke most recently" is
// the right recipient and needs no configuration. Silently does nothing if no peer has ever been
// seen or the UDP socket is not bound — RS485 traffic is not held for a peer that may never come.
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
// Publish bridge status into /json/info, which is what the WLED info page renders.
//
// Three states, deliberately distinguishable at a glance: "disabled" (the user turned it off),
// "error: <reason>" (enabled but setup() could not bring it up — almost always a pin conflict),
// and the running case with the live counters. The counter rows only appear when running, and
// the drop row only when something has actually been dropped, so a healthy bridge stays quiet.
//
// Each row is a JSON array of [value, suffix] because that is how WLED's info page renders a
// number with a unit; the string halves therefore carry the leading space.
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

// Serialise the usermod's settings into cfg.json (and the Settings → Usermods form).
//
// The keys here must match readFromConfig() exactly — a mismatch does not error, it silently
// reverts to defaults on the next boot, which is why the config round-trip is an explicit item in
// the plan's Testing Required list.
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

// Load the usermod's settings from cfg.json.
//
// Called once before setup() at boot, and again after every save from the settings page.
// Returns false when the section is absent, which is how WLED detects a config predating this
// usermod and knows to write the defaults out.
//
// getJsonValue() leaves the target untouched when the key is missing, so a config written by an
// older build keeps this build's defaults for the new fields rather than zeroing them.
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
  // Post-boot save (initDone) while the bridge is up: apply what can safely be applied live.
  // UART / pins / baud / enable pin are NOT among them — RS485Socket::init_general() re-news its
  // channel without freeing the old one and overwrites RS485Utils' file-scope serial pointer, so
  // re-initialising leaks on every save. Those fields are stored now and take effect on the next
  // boot, which the settings help text and readme both state. The socket address is safe because
  // it is read per call, and clearing udpStarted makes serviceUdp() rebind to the new port.
  if (initDone && running) {
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
// RS485_BRIDGE_BUILD in usermod_rs485_bridge.h.
//
// rs485_bridge_protocol.h is NOT included here: it imports the wire format from the HMTL
// submodule, which is only on the build path in the RS485-enabled env (and is not cloned by the
// CI workflows at all). Its wire-layout static_asserts are covered instead by `pio run -e
// ampworks` and by the host test, which compiles the same HMTL header with a plain c++.
//
// An inert placeholder usermod stands in for the real one, for two reasons:
//   * pio-scripts/validate_modules.py fails the build for any enrolled usermod that contributes
//     no DWARF compilation unit to the linked ELF, and a translation unit whose only symbol is
//     unreferenced is dropped by --gc-sections, leaving no CU. REGISTER_USERMOD's pointer lands
//     in the KEEP'd .dynarray.usermods section, so the CU survives.
//   * It makes the situation legible at runtime instead of the usermod silently not existing.

class UsermodRS485BridgeUnavailable : public Usermod {
 public:
  void setup() override {}   // nothing to bring up
  void loop() override {}    // nothing to service
  // Usermod id (USERMOD_ID_RS485_BRIDGE) — what UsermodManager::lookup() finds this bridge by.
  uint16_t getId() override { return USERMOD_ID_RS485_BRIDGE; }
  // Report the not-built state on the info page, so a board flashed with the wrong env says so
  // instead of the usermod simply not appearing.
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
