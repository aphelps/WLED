#pragma once

#include "wled.h"

/*
 * UsermodSensorSync — broadcasts local sensor *state snapshots* to, and receives them from,
 * other WLED devices on the same local network (no router required). Sensor-agnostic bus
 * (touch, switches, proximity, …).
 *
 * Design:
 *   - The PRODUCER broadcasts a full snapshot of its sensor state (e.g. the touched bitmask)
 *     whenever that state changes — all channels in one message.
 *   - The CONSUMER keeps the last snapshot seen per origin device and derives rising/falling
 *     edges by diffing the new snapshot against it, enqueuing discrete RemoteSensorEvents.
 *     Snapshots are self-healing: a dropped UDP datagram is corrected by the next snapshot
 *     (no permanent edge loss), which is why edge derivation lives at the consumer.
 *   - GENERIC header common to all sensor types + a per-type data struct; header.dataLen =
 *     the data struct's byte length. Add a sensor type = new SS_SENSOR_* tag (+ a data struct
 *     and a dispatchMessage() branch) — no header change.
 *   - Transport: UDP limited broadcast on SENSOR_SYNC_PORT. ESP-NOW / multi-hop come later.
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

#define SENSOR_SYNC_VERSION      4   // wire protocol version (snapshot + consumer-side edges)
#define SENSOR_SYNC_MSG_SNAPSHOT 0   // msgType: a full sensor-state snapshot

// Sensor type tags (extend as new producers are added).
#define SS_SENSOR_TOUCH   0   // MPR121 channels; SensorSnapshot.mask bit e = channel e active

// Generic header, common to every sensor type. 16 bytes, naturally 4-byte aligned.
struct __attribute__((packed)) SensorSyncHeader {
  char     magic[4];   // {'A','M','P','S'} — AMPWorks sensor sync
  uint8_t  version;    // SENSOR_SYNC_VERSION
  uint8_t  msgType;    // SENSOR_SYNC_MSG_*
  uint8_t  sensorType; // SS_SENSOR_* — selects the data struct that follows
  uint8_t  dataLen;    // number of sensor-data bytes following the header
  uint16_t deviceId;   // origin device id
  uint16_t seq;        // per-sender sequence number (future dedup/loss tracking)
  uint32_t timestamp;  // millis() + strip.timebase at origin
};

// Sensor-data struct for SS_SENSOR_TOUCH: full state of every channel in one snapshot.
struct __attribute__((packed)) SensorSnapshot {
  uint16_t mask;       // bit e set = channel e currently active (electrodes + proximity)
};

// What effects/consumers see after the consumer derives an edge from a remote snapshot.
struct RemoteSensorEvent {
  uint8_t  sensorType;
  uint8_t  channel;
  uint8_t  value;      // 1 = became active (press), 0 = became inactive (release)
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

  // Consumer API: pop the oldest derived remote event; false when the queue is empty.
  bool popRemoteEvent(RemoteSensorEvent &out);
  uint16_t getDeviceId() const;

  uint16_t getId() override;
  void addToJsonInfo(JsonObject &root) override;
  void addToConfig(JsonObject &root) override;
  bool readFromConfig(JsonObject &root) override;

 private:
  static const uint8_t RX_QUEUE_LEN = 32;
  static const uint8_t MAX_PEERS    = 8;   // per-device last-snapshot state for edge derivation
  static const uint8_t RX_BUF_LEN   = sizeof(SensorSyncHeader) + 64;  // header + largest data struct

  bool     initDone   = false;
  bool     enabled    = true;
  uint16_t port       = SENSOR_SYNC_PORT;
  uint16_t configId   = 0;       // 0 = auto-derive
  uint16_t deviceId   = 0;

  WiFiUDP  udp;
  bool     udpStarted = false;
  uint16_t txSeq      = 0;
  uint32_t txCount    = 0;
  uint16_t prevLocalMask = 0;    // last snapshot we broadcast (change detection; not edges)

  RemoteSensorEvent rxQueue[RX_QUEUE_LEN];
  uint8_t  rxHead = 0, rxTail = 0, rxCount = 0;

  // Per-peer last snapshot, so the consumer can derive edges from full-state messages.
  struct PeerSnapshot { uint16_t deviceId; uint16_t mask; bool used; };
  PeerSnapshot peers[MAX_PEERS] = {};

  static uint16_t deriveDeviceId();
  void enqueue(uint8_t sensorType, uint8_t channel, uint8_t value, uint16_t dev, uint32_t ts);
  PeerSnapshot *peerSlot(uint16_t dev);
  void dispatchMessage(const SensorSyncHeader &h, const uint8_t *data, int dataLen);
  void receiveLoop();
  bool sendSnapshot(uint8_t sensorType, uint16_t mask);
  void broadcastLocalState();
};
