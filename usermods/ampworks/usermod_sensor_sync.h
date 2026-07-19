#pragma once

#include "wled.h"
#include "sensor_sync_protocol.h"   // wire format + pure ss_parse_header/ss_dispatch

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
 * unit-tested). This class wires that to a transport (UDP now; ESP-NOW later via ISensorTransport)
 * and to the effect layer.
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

// Minimal transport seam: broadcast one datagram, or poll the next received one. UDP-specific
// packet-boundary handling stays inside the implementation; M3 adds an ESP-NOW impl.
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

  // --- Consumer API (per-consumer cursor over the shared event ring) ---
  SensorCursor subscribe() const { return ring.subscribe(); }
  uint8_t drain(SensorCursor &cur, RemoteSensorEvent *out, uint8_t maxOut) {
    return ring.drain(cur, out, maxOut);
  }

  uint32_t getDeviceId() const { return deviceId; }

  uint16_t getId() override;
  void addToJsonInfo(JsonObject &root) override;
  void addToConfig(JsonObject &root) override;
  bool readFromConfig(JsonObject &root) override;

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

  UdpSensorTransport transport;
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

  static uint32_t deriveDeviceId();
  bool sendMessage(uint8_t sensorType, const uint8_t *data, uint8_t dataLen);
  void receiveLoop();
  void broadcastLocalState();
  void sweepPeers();
};
