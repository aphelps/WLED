#pragma once

#include "wled.h"

/*
 * UsermodSensorSync — broadcasts local sensor *edge events* to, and receives them from, other
 * WLED devices on the same local network (no router required). Renamed from "touch_sync": the
 * bus is sensor-agnostic (touch, switches, proximity, temperature, …), so the name reflects that.
 *
 * Design (per review on WLED#3):
 *   - The PRODUCER derives rising/falling edges once (diffing its own previous state) and
 *     transmits the edges. Consumers do NOT re-derive edges — they read them directly. This is
 *     cheaper (one diff at the source vs. one per consumer) and removes per-peer snapshot state.
 *   - All sensors that changed in a given tick are sent in ONE message (a list of edge records),
 *     not one packet per channel.
 *   - GENERIC header common to all sensor types + a per-type record; header.count = number of
 *     edge records following. Add a sensor type = new SS_SENSOR_* tag (+ handling) — no header change.
 *   - Transport: UDP limited broadcast on SENSOR_SYNC_PORT. ESP-NOW / multi-hop come in later milestones.
 *
 * Implementation lives in usermod_sensor_sync.cpp (this header is declarations only).
 *
 * Consumers look this usermod up directly:
 *   #ifdef USERMOD_SENSOR_SYNC
 *   UsermodSensorSync *ss = (UsermodSensorSync*) UsermodManager::lookup(USERMOD_ID_SENSOR_SYNC);
 *   RemoteSensorEvent ev;
 *   while (ss && ss->popRemoteEvent(ev)) { ... }
 *   #endif
 *
 * Config (cfg.json): enabled (default true), port (default 21330, no external meaning — chosen
 * to avoid WLED's 21324 notifier / 65506 node-list), id (device id override; 0 = auto from MAC).
 */

#ifndef SENSOR_SYNC_PORT
  #define SENSOR_SYNC_PORT 21330
#endif

#define SENSOR_SYNC_VERSION   3   // wire protocol version (producer-side edges)
#define SENSOR_SYNC_MSG_EDGES 0   // msgType: a batch of sensor edge records

// Sensor type tags (extend as new producers are added).
#define SS_SENSOR_TOUCH   0   // MPR121 electrode/proximity channel; value = 1 active / 0 inactive

// Generic header, common to every sensor type. 16 bytes, naturally 4-byte aligned.
struct __attribute__((packed)) SensorSyncHeader {
  char     magic[4];   // {'A','M','P','S'} — AMPWorks sensor sync
  uint8_t  version;    // SENSOR_SYNC_VERSION
  uint8_t  msgType;    // SENSOR_SYNC_MSG_*
  uint8_t  sensorType; // SS_SENSOR_* — what kind of sensor these edges are from
  uint8_t  count;      // number of SensorEdge records following the header
  uint16_t deviceId;   // origin device id
  uint16_t seq;        // per-sender sequence number (future dedup/loss tracking)
  uint32_t timestamp;  // millis() + strip.timebase at origin
};

// One edge: channel changed to `value`. Producer emits these; consumers read them directly.
struct __attribute__((packed)) SensorEdge {
  uint8_t channel;     // sensor channel index
  uint8_t value;       // 1 = became active (press), 0 = became inactive (release)
};

// What effects/consumers see after draining the receive queue.
struct RemoteSensorEvent {
  uint8_t  sensorType;
  uint8_t  channel;
  uint8_t  value;
  uint16_t deviceId;
  uint32_t timestamp;
};

class UsermodSensorSync : public Usermod {
 public:
  static const char _name[];
  static const char _enabled[];

  void setup() override;
  void connected() override;
  void loop() override;

  // Consumer API: pop the oldest received remote event; false when the queue is empty.
  bool popRemoteEvent(RemoteSensorEvent &out);
  uint16_t getDeviceId() const;

  uint16_t getId() override;
  void addToJsonInfo(JsonObject &root) override;
  void addToConfig(JsonObject &root) override;
  bool readFromConfig(JsonObject &root) override;

 private:
  static const uint8_t RX_QUEUE_LEN = 32;
  static const uint8_t RX_BUF_LEN   = sizeof(SensorSyncHeader) + 64;  // header + max edge batch

  bool     initDone   = false;
  bool     enabled    = true;
  uint16_t port       = SENSOR_SYNC_PORT;
  uint16_t configId   = 0;       // 0 = auto-derive
  uint16_t deviceId   = 0;

  WiFiUDP  udp;
  bool     udpStarted = false;
  uint16_t txSeq      = 0;
  uint32_t txCount    = 0;
  uint16_t prevLocalTouched = 0; // producer-side previous state for edge derivation

  RemoteSensorEvent rxQueue[RX_QUEUE_LEN];
  uint8_t  rxHead = 0, rxTail = 0, rxCount = 0;

  static uint16_t deriveDeviceId();
  void enqueue(uint8_t sensorType, uint8_t channel, uint8_t value, uint16_t dev, uint32_t ts);
  void dispatchMessage(const SensorSyncHeader &h, const uint8_t *data, int dataLen);
  void receiveLoop();
  void sendEdges(uint8_t sensorType, const SensorEdge *edges, uint8_t n);
  void broadcastLocalState();
};
