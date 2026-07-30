#pragma once

#include "wled.h"
#include "sensor_sync_protocol.h"   // wire format + pure ss_parse_header/ss_dispatch
#include "sensor_sync_ring.h"       // dependency-free SPSC RX ring (host-tested)
#include "sensor_control.h"         // pure control-frame parse + ordering/echo logic

#ifndef SENSOR_SYNC_PORT
  #define SENSOR_SYNC_PORT 21330   // no external meaning; avoids WLED's 21324 notifier / 65506 node list
#endif

/*
 * UsermodSensorSync — a typed multi-sensor event bus across WLED devices on the local network
 * (no router required). Producers broadcast full sensor-state snapshots; receivers derive
 * discrete RemoteSensorEvents (edges for bitmask sensors, direct samples for scalar sensors).
 *
 * Architecture overview + send/receive flow charts: see SENSOR_SYNC.md (keep it in sync).
 *
 * The wire format + dispatch logic live in the dependency-free sensor_sync_protocol.h (host
 * unit-tested). This class wires that to a transport (UDP or ESP-NOW, both behind
 * ISensorTransport) and to the effect layer.
 *
 * Producer API (any usermod/effect can publish onto the bus):
 *   ss->publishSnapshot(SS_SENSOR_SWITCH, mask);          // bitmask sensors (edge-derived)
 *   ss->publishSample(SS_SENSOR_TEMP, channel, centiDeg); // scalar sensors (delivered as-is)
 * The built-in MPR121 touch poll is the first producer.
 *
 * Consumer API (each consumer keeps its own cursor, so multiple segments/effects independently
 * see every event — no single-drain contention):
 *   SensorCursor cur = ss->subscribe();                   // once (e.g. in SEGENV on call==0)
 *   RemoteSensorEvent ev[8];
 *   uint8_t n = ss->drain(cur, ev, 8);                    // each frame; advances cur
 *
 * Config (cfg.json): enabled (default true), port (default 21330), id (device-id override;
 * 0 = auto from MAC), keyframeMs (periodic full-snapshot re-broadcast; 0 = off).
 */

// Minimal transport seam: broadcast one datagram, or poll the next received one. Transport-specific
// packet-boundary handling stays inside each implementation (UDP and ESP-NOW, below).
struct ISensorTransport {
  virtual ~ISensorTransport() {}
  virtual bool begin(uint16_t port) = 0;
  virtual void end() = 0;
  virtual bool broadcast(const uint8_t *buf, int len) = 0;
  virtual int  poll(uint8_t *buf, int maxLen) = 0;   // >0 = bytes read, 0 = nothing pending
};

class UdpSensorTransport : public ISensorTransport {
 public:
  bool begin(uint16_t port) override;
  void end() override;
  bool broadcast(const uint8_t *buf, int len) override;
  int  poll(uint8_t *buf, int maxLen) override;
 private:
  WiFiUDP  udp;
  uint16_t boundPort = 0;
  bool     started   = false;
};

// The demux classifier ss_is_our_frame() lives in the dependency-free sensor_sync_protocol.h so
// it is host-testable and shared with the RX hook below.

#ifndef WLED_DISABLE_ESPNOW
// ESP-NOW transport: broadcast via quickEspNow; RX is callback-driven, so inbound frames are
// fed (from UsermodSensorSync::onEspNowMessage) into a small fixed ring that poll() drains,
// preserving the poll() semantics receiveLoop() expects. The WLED radio is already started in
// wled.cpp — we never (re)init it here, we just start accepting frames on begin().
class EspNowSensorTransport : public ISensorTransport {
 public:
  bool begin(uint16_t port) override;   // port is vestigial for ESP-NOW; ignored (logged)
  void end() override;
  bool broadcast(const uint8_t *buf, int len) override;
  int  poll(uint8_t *buf, int maxLen) override;

  bool isStarted() const { return started; }
  // Push an inbound frame into the RX ring (the PRODUCER side; called from the ESP-NOW receive
  // hook, which QuickEspNow runs on a dedicated FreeRTOS task). Silently drops if not started,
  // oversized, or the ring is full (drop-newest — a keyframe re-broadcast re-syncs state).
  void feed(const uint8_t *data, int len);

 private:
  // Lock-free SPSC ring: producer feed() (ESP-NOW RX task) vs consumer poll() (loop() task).
  // Correctness comes from the two-index (head/tail) design in SpscByteRing — no shared count,
  // no mutex. Depth RX_RING gives (RX_RING - 1) usable slots.
  static const uint8_t  RX_RING     = 7;                       // 6 usable slots (one reserved)
  static const uint16_t RX_SLOT_LEN = sizeof(SensorSyncHeader) + 64;
  SpscByteRing<RX_RING, RX_SLOT_LEN> rxRing;
  bool     started = false;
};
#endif

class UsermodSensorSync : public Usermod {
 public:
  static const char _name[];
  static const char _enabled[];

  void setup() override;
  void connected() override;
  void loop() override;

  // --- Producer API ---
  bool publishSnapshot(uint8_t sensorType, uint16_t mask);        // TOUCH / SWITCH
  bool publishSample(uint8_t sensorType, uint8_t channel, int16_t value); // PROXIMITY / TEMP

  // --- Control plane (M4) ---
  // Put a UI/preset command onto the mesh. This is the gateway path: a node that took a command
  // locally (its own web UI / JSON API) fans it out to the installation. Applies locally too, so
  // the originator and everyone else converge on the same state.
  bool publishControl(const SensorControl &c);
  // Convenience for the common case, so callers don't hand-assemble the field mask.
  bool publishPreset(uint8_t presetId);

  // --- Consumer API (per-consumer cursor over the shared event ring) ---
  SensorCursor subscribe() const { return ring.subscribe(); }
  uint8_t drain(SensorCursor &cur, RemoteSensorEvent *out, uint8_t maxOut) {
    return ring.drain(cur, out, maxOut);
  }

  uint32_t getDeviceId() const { return deviceId; }

  // WLED fires this from stateUpdated() on every state change. This is the gateway path: a
  // user-driven change here is put onto the mesh. See the implementation for which call modes
  // count as user-driven and why that is what suppresses echo.
  void onStateChange(uint8_t mode) override;

  uint16_t getId() override;
  void addToJsonInfo(JsonObject &root) override;
  void addToConfig(JsonObject &root) override;
  bool readFromConfig(JsonObject &root) override;

#ifndef WLED_DISABLE_ESPNOW
  // WLED routes inbound ESP-NOW here before its own linked-remote handling. We claim (return
  // true) only OUR SensorSync frames when the ESP-NOW transport is active; everything else falls
  // through untouched so WLED's own sync traffic is never disturbed.
  bool onEspNowMessage(uint8_t *sender, uint8_t *payload, uint8_t len) override;
#endif

 private:
  static const uint8_t  EVENT_RING   = 32;   // per-consumer cursors read from this monotonic ring
  static const uint8_t  MAX_PEERS     = 8;    // per-device snapshot state for edge derivation
  static const uint16_t RX_BUF_LEN    = sizeof(SensorSyncHeader) + 64;
  static const uint32_t PEER_TIMEOUT_MS = 30000;   // free peer state idle longer than this

  bool     initDone   = false;
  bool     enabled    = true;
  uint16_t port       = SENSOR_SYNC_PORT;
  uint32_t configId   = 0;       // 0 = auto-derive
  uint32_t deviceId   = 0;
  uint32_t keyframeMs = 3000;    // periodic full-snapshot re-broadcast (0 = off)
  bool     useEspNow  = false;   // false = UDP (default, unchanged behavior); true = ESP-NOW
  // Off by default: with this on, every local UI/API change is broadcast to the whole
  // installation, which is a large behaviour change to opt into rather than inherit on upgrade.
  bool     gateway    = false;   // act as a control gateway (publish local state changes)
  // Set by onStateChange (which may run on the AsyncTCP task), consumed by loop().
  volatile bool controlPending = false;

  // Both transports are owned; `transport` points at the active one (defaults to UDP, so an
  // unconfigured device broadcasts over UDP). Selection swaps the pointer.
  UdpSensorTransport udpTransport;
#ifndef WLED_DISABLE_ESPNOW
  EspNowSensorTransport espNowTransport;
#endif
  ISensorTransport *transport = &udpTransport;
  bool     started    = false;
  uint16_t txSeq      = 0;
  uint32_t txCount    = 0;

  // Producer change-detection for the built-in touch snapshot.
  uint16_t prevLocalMask   = 0;
  uint32_t lastTouchSendMs = 0;

  // Monotonic multi-consumer event ring (pure logic in SensorEventRing; each consumer cursor
  // reads [writeSeq-EVENT_RING, writeSeq)).
  RemoteSensorEvent events[EVENT_RING];
  SensorEventRing   ring{ events, EVENT_RING, 0 };

  // Peer snapshot table (pure struct) + parallel last-seen stamps for aging.
  SensorPeer peers[MAX_PEERS] = {};
  uint32_t   peerLastSeen[MAX_PEERS] = {};
  uint32_t   lastPeerSweepMs = 0;

  // Last control command applied, for the total order + echo suppression (sensor_control.h).
  ControlState control = ss_ctrl_init();

  static uint32_t deriveDeviceId();
  // msgType is explicit so control frames draw from the SAME txSeq counter as snapshots:
  // the router deduplicates per origin on a single seq space, so a separate counter would
  // make each type look like a duplicate of the other and get dropped mid-backbone.
  bool sendMessage(uint8_t msgType, uint8_t sensorType, const uint8_t *data, uint8_t dataLen);
  void receiveLoop();
  // Split deliberately: applyControl() owns the DECISION for a remote frame (ordering + echo
  // suppression, which touches `control`), applyControlFields() owns the EFFECT on WLED state and
  // touches no ordering state. The gateway path needs the effect without the decision.
  void applyControl(const SensorSyncHeader &h, const SensorControl &c);
  void applyControlFields(const SensorControl &c);
  void publishPendingControl();
  void broadcastLocalState();
  void sweepPeers();
};
