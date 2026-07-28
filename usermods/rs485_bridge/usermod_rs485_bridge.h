#pragma once

#include "wled.h"

// ---------------------------------------------------------------------------------------------
// Build guard
// ---------------------------------------------------------------------------------------------
// This usermod owns a hardware UART and depends on ArduinoLibs' RS485Utils, so it can only be
// compiled when the whole set is present:
//   USERMOD_RS485_BRIDGE   — opt-in flag from the env's build_flags
//   RS485_HARDWARE_SERIAL  — the UART number; also what makes RS485Utils' SERIAL_TYPE resolve to
//                            HardwareSerial instead of AVR-only SoftwareSerial
//   ARDUINO_ARCH_ESP32     — UART2 / HardwareSerial(n) with remappable pins
// Creating usermods/rs485_bridge/library.json enrols this directory in `[env:usermods]`
// (custom_usermods = *) and in the per-usermod CI matrix (usermods_esp32 / _esp32c3 / _esp32s2 /
// _esp32s3), none of which define those flags — there the whole translation unit compiles to
// nothing. See readme.md ("Why the build guard exists").
#if defined(USERMOD_RS485_BRIDGE) && defined(RS485_HARDWARE_SERIAL) && defined(ARDUINO_ARCH_ESP32)
  #define RS485_BRIDGE_BUILD 1
#else
  #define RS485_BRIDGE_BUILD 0
#endif

#if RS485_BRIDGE_BUILD

#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include "RS485Utils.h"                // ArduinoLibs — RS485Socket over Nick Gammon's protocol
#include "rs485_bridge_protocol.h"     // dependency-free HMTL framing + bridge decision logic

#ifndef RS485_BRIDGE_UDP_PORT
  // Follows the SENSOR_SYNC_PORT precedent (usermods/ampworks/usermod_sensor_sync.h). Must avoid
  // WLED's 21324/65506 notifier ports, 5568 (E1.31), 4048 (DDP), 6454 (Art-Net) and 21330
  // (SensorSync).
  #define RS485_BRIDGE_UDP_PORT 21331
#endif

/*
 * UsermodRS485Bridge — turns a WLED ESP32 into a WiFi <-> RS485 bridge for legacy HMTL modules.
 *
 *   Master path (WiFi -> RS485): a UDP listener accepts raw HMTL-framed datagrams, validates
 *   them, and queues them for transmission onto the RS485 bus. At most one frame is written per
 *   loop() iteration because RS485Socket::sendMsgTo blocks while it flushes the UART.
 *
 *   Slave path (RS485 -> WLED): frames addressed to this node (or broadcast) are parsed and
 *   acted on — RGB / VALUE sets, a small set of PROGRAM commands mapped to WLED effects, POLL
 *   and SET_ADDRESS so the bridge behaves like a proper HMTL bus citizen, and SENSOR broadcasts.
 *   Everything else is counted and ignored. Frames addressed elsewhere are relayed to the last
 *   WiFi peer, as are responses to forwarded commands.
 *
 * The wire format and every parsing/queueing decision live in rs485_bridge_protocol.h, which has
 * no WLED/Arduino dependencies and is host unit-tested (tests/rs485_bridge_test.cpp).
 *
 * Runtime reconfiguration: UART / pins / baud changes take effect on the NEXT BOOT only.
 * RS485Utils keeps its serial pointer in a file-scope global and RS485Socket::init_general
 * unconditionally re-news the channel without freeing the previous one, so re-init() leaks.
 * setup() is the sole caller of HardwareSerial::begin() and RS485Socket::init().
 * (The RS485 address is the exception: it lives in RS485Socket::sourceAddress and can be
 * changed live, which is what makes HMTL SET_ADDRESS work.)
 */
class UsermodRS485Bridge : public Usermod {
 public:
  static const char _name[];      // config section name, also the settings-page heading
  static const char _enabled[];   // config key for the enable flag

  // Claim the GPIOs, open the UART and attach RS485Socket. Sole caller of begin()/init().
  void setup() override;
  // WiFi (re)connected — marks the UDP socket for rebinding.
  void connected() override;
  // Per-iteration service of both directions; see the .cpp for the rate-limiting rationale.
  void loop() override;

  uint16_t getId() override { return USERMOD_ID_RS485_BRIDGE; }
  // Publish state + counters into /json/info.
  void addToJsonInfo(JsonObject &root) override;
  // Write the settings into cfg.json / the Settings -> Usermods form.
  void addToConfig(JsonObject &root) override;
  // Read the settings back; returns false when the config predates this usermod.
  bool readFromConfig(JsonObject &root) override;

  // Queue an HMTL frame for transmission on the RS485 bus. Returns false if the bridge is not
  // running, the frame did not fit a queue slot, or an older queued frame had to be dropped to
  // make room. Public so other usermods can push HMTL traffic onto the bus.
  bool sendHmtlFrame(uint16_t destAddr, const uint8_t *frame, uint8_t frameLen);

  bool isRunning() const { return running; }        // true once pins + socket are up
  uint16_t getAddress() const { return address; }   // this node's RS485 socket address

 private:
  // Work budgets per loop() iteration. Reception is cheap (non-blocking reads of tens of bytes);
  // transmission is not, which is why serviceTx() has a budget of exactly one and needs no
  // constant here. UDP_BUF_LEN matches a transmit slot because nothing larger can be forwarded.
  static const uint8_t  RX_PER_LOOP  = 4;
  static const uint8_t  UDP_PER_LOOP = 4;
  static const uint16_t UDP_BUF_LEN  = RS485B_TX_SLOT_LEN;

  // config
  bool     enabled    = false;
  uint8_t  uartNum    = RS485_HARDWARE_SERIAL;
  int8_t   rxPin      = 16;
  int8_t   txPin      = 17;
  int8_t   enPin      = 18;
  uint32_t baud       = RS485Socket::DEFAULT_BAUD;   // 28000 — interoperates with legacy modules
  uint16_t address    = 1;
  uint16_t udpPort    = RS485_BRIDGE_UDP_PORT;
  uint16_t deviceId   = 0;                            // 0 == derive from the MAC

  // runtime
  bool           initDone   = false;
  bool           running    = false;    // pins allocated + socket initialised
  bool           udpStarted = false;
  const char    *failReason = nullptr;  // why setup() left the bridge disabled
  HardwareSerial *port      = nullptr;
  RS485Socket     rs485;
  WiFiUDP         udp;
  IPAddress       peerIp;
  uint16_t        peerPort = 0;

  // RS485Socket::sendMsgTo writes the socket header in FRONT of the caller's buffer, so the
  // backing array needs room for both. initBuffer() hands back the data region.
  uint8_t  rawTxBuf[RS485_BUFFER_TOTAL(RS485B_TX_SLOT_LEN)];
  uint8_t *txData = nullptr;

  RS485BTxQueue  txQueue;
  RS485BCounters counters;

  bool allocatePins();      // reserve {rx, tx, en} with PinManager, all-or-nothing
  void releasePins();       // hand them back
  void serviceRs485();      // slave path: drain and act on up to RX_PER_LOOP received frames
  void serviceTx();         // master path: write ONE queued frame (the blocking-send rate limit)
  void serviceUdp();        // master path: accept up to UDP_PER_LOOP HMTL datagrams from WiFi
  // Execute the decision rs485b_decide() reached; `sourceAddr` is the socket-layer sender.
  void handleDecision(const RS485BDecision &d, const uint8_t *frame, uint8_t frameLen,
                      uint16_t sourceAddr);
  void applyRgb(const uint8_t rgb[3]);      // HMTL RGB   -> main segment colour, mode STATIC
  void applyValue(uint16_t value);          // HMTL VALUE -> master brightness (clamped to 8 bit)
  void applyProgram(const RS485BDecision &d);   // HMTL PROGRAM -> effect / brightness / colour
  void sendPollResponse(uint16_t to);       // answer a POLL with this node's config block
  void relayToPeer(const uint8_t *frame, uint8_t frameLen);   // RS485 -> last WiFi peer
  uint16_t effectiveDeviceId() const;       // configured id, or one derived from the MAC
};

#endif  // RS485_BRIDGE_BUILD
