#pragma once

#include "wled.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

/*
 * UsermodTouchSync — broadcasts local sensor state to, and receives remote sensor state
 * from, other WLED devices on the same local network (no router required).
 *
 * M0 prototype scope (see todo_plans/multi-device-wled-mesh-sync-with-shared-touch-input.md):
 *   - Transport: UDP limited broadcast (255.255.255.255) on TOUCH_SYNC_PORT. Works over a
 *     WiFi SoftAP + stations setup with no internet/router. ESP-NOW + multi-hop mesh come later.
 *   - Producer: polls the MPR121 usermod each loop and, whenever the full touch state changes,
 *     broadcasts ALL sensors of that device in a single message (one snapshot, not per-edge).
 *   - Consumer: receives remote snapshots, derives per-device rising/falling edges, and enqueues
 *     discrete RemoteSensorEvents into an in-RAM ring buffer that effects drain via popRemoteEvent().
 *     Touch Pond (ampworks.cpp) spawns a distinct-colored wave per remote press.
 *
 * Wire format (little-endian; all devices are ESP32, same ABI):
 *   [ TouchSyncHeader (16 B, 4-byte aligned) ][ sensor-data struct, header.dataLen bytes ]
 * The header is GENERIC across sensor types; header.sensorType selects which data struct follows
 * and header.dataLen is its byte length. To add a sensor type (switch/proximity/temp), define a
 * new TS_SENSOR_* tag + a packed data struct and handle it in dispatchMessage() — no header change.
 *
 * Device ID is derived from the low 2 bytes of the station MAC (override via config "id", 0=auto).
 * A device ignores messages bearing its own ID, so broadcast loopback is harmless.
 *
 * Consumers look this usermod up directly:
 *   #ifdef USERMOD_TOUCH_SYNC
 *   UsermodTouchSync *ts = (UsermodTouchSync*) UsermodManager::lookup(USERMOD_ID_TOUCH_SYNC);
 *   RemoteSensorEvent ev;
 *   while (ts && ts->popRemoteEvent(ev)) { ... }
 *   #endif
 *
 * NOTE (M0 limitation): popRemoteEvent() drains a single global queue — correct for one active
 * Touch Pond segment; generalize to per-consumer cursors in M1.
 *
 * Config (cfg.json):
 *   enabled — run the sync usermod (default true)
 *   port    — UDP port (default 21330; no external/standard meaning, chosen to avoid WLED's
 *             21324 notifier and 65506 node-list ports)
 *   id      — device id override; 0 = auto-derive from MAC (default 0)
 */

#ifndef TOUCH_SYNC_PORT
  #define TOUCH_SYNC_PORT 21330
#endif

#define TOUCH_SYNC_VERSION   2   // wire protocol version (bumped for the generic-header layout)
#define TOUCH_SYNC_MSG_EVENT 0   // msgType: a sensor-state snapshot

// Sensor type tags — one packed data struct per type (extend in M1).
#define TS_SENSOR_TOUCH   0   // MPR121: TsTouchData (bitmask of all electrodes + proximity)

// Generic header, common to every sensor type. 16 bytes, naturally 4-byte aligned
// (4-byte magic first keeps the whole struct and the uint32 timestamp aligned).
struct __attribute__((packed)) TouchSyncHeader {
  char     magic[4];   // {'A','T','S','1'} — AMPWorks Touch Sync, v1 wire family
  uint8_t  version;    // TOUCH_SYNC_VERSION
  uint8_t  msgType;    // TOUCH_SYNC_MSG_*
  uint8_t  sensorType; // TS_SENSOR_* — selects the data struct that follows
  uint8_t  dataLen;    // number of sensor-data bytes following the header
  uint16_t deviceId;   // origin device id
  uint16_t seq;        // per-sender sequence number (future dedup/loss tracking)
  uint32_t timestamp;  // millis() + strip.timebase at origin
};

// Sensor-data struct for TS_SENSOR_TOUCH: one snapshot covers all MPR121 channels at once.
struct __attribute__((packed)) TsTouchData {
  uint16_t touched;    // bit e set = channel e currently touched (0..MPR121::TOTAL_SENSORS-1)
};

// What effects/consumers see after the receiver derives an edge from a remote snapshot.
struct RemoteSensorEvent {
  uint8_t  sensorType;
  uint8_t  channel;
  uint8_t  value;      // 1 = became active (press), 0 = became inactive (release)
  uint16_t deviceId;
  uint32_t timestamp;
};

class UsermodTouchSync : public Usermod {
 public:
  static const char _name[];
  static const char _enabled[];

  void setup() override {
    deviceId = configId ? configId : deriveDeviceId();
    initDone = true;
  }

  void connected() override {
    udpStarted = false;  // re-bind on (re)connection; lazy begin in loop() also covers SoftAP-only
  }

  void loop() override {
    if (!enabled || strip.isUpdating()) return;

    if (!(Network.isConnected() || apActive)) { udpStarted = false; return; }
    if (!udpStarted) {
      if (!udp.begin(port)) return;
      udpStarted = true;
    }

    receiveLoop();
    broadcastLocalState();
  }

  // ── Consumer API ────────────────────────────────────────────────────────────
  bool popRemoteEvent(RemoteSensorEvent &out) {
    if (rxCount == 0) return false;
    out = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_LEN;
    rxCount--;
    return true;
  }

  uint16_t getDeviceId() const { return deviceId; }

  uint16_t getId() override { return USERMOD_ID_TOUCH_SYNC; }

  void addToJsonInfo(JsonObject &root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray arr = user.createNestedArray("Touch Sync");
    arr.add(udpStarted ? txCount : 0);
    arr.add(udpStarted ? " sent" : " (offline)");
  }

  void addToConfig(JsonObject &root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = enabled;
    top["port"]          = port;
    top["id"]            = configId;
  }

  bool readFromConfig(JsonObject &root) override {
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) return false;
    uint16_t portPrev = port;
    getJsonValue(top[FPSTR(_enabled)], enabled);
    getJsonValue(top["port"], port);
    getJsonValue(top["id"],   configId);
    if (initDone) {
      deviceId = configId ? configId : deriveDeviceId();
      if (port != portPrev) udpStarted = false;  // rebind on next loop
    }
    return true;
  }

 private:
  static const uint8_t RX_QUEUE_LEN = 32;
  static const uint8_t MAX_PEERS    = 8;   // per-device snapshot state for edge detection
  static const uint8_t RX_BUF_LEN   = sizeof(TouchSyncHeader) + 64;  // header + largest data struct

  bool     initDone   = false;
  bool     enabled    = true;
  uint16_t port       = TOUCH_SYNC_PORT;
  uint16_t configId   = 0;       // 0 = auto-derive
  uint16_t deviceId   = 0;

  WiFiUDP  udp;
  bool     udpStarted = false;
  uint16_t txSeq      = 0;
  uint32_t txCount    = 0;
  uint16_t prevLocalTouched = 0; // last broadcast local snapshot (change detection)

  RemoteSensorEvent rxQueue[RX_QUEUE_LEN];
  uint8_t  rxHead = 0, rxTail = 0, rxCount = 0;

  // Per-peer previous touch snapshot, so we can derive edges from full-state messages.
  struct PeerTouch { uint16_t deviceId; uint16_t touched; bool used; };
  PeerTouch peers[MAX_PEERS] = {};

  static uint16_t deriveDeviceId() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    uint16_t id = ((uint16_t)mac[4] << 8) | mac[5];
    return id ? id : 1;  // never 0 (0 means "auto")
  }

  void enqueue(uint8_t sensorType, uint8_t channel, uint8_t value, uint16_t dev, uint32_t ts) {
    RemoteSensorEvent &slot = rxQueue[rxHead];
    slot.sensorType = sensorType;
    slot.channel    = channel;
    slot.value      = value;
    slot.deviceId   = dev;
    slot.timestamp  = ts;
    rxHead = (rxHead + 1) % RX_QUEUE_LEN;
    if (rxCount < RX_QUEUE_LEN) rxCount++;
    else rxTail = (rxTail + 1) % RX_QUEUE_LEN;  // overwrite oldest
  }

  // Find (or allocate) the stored snapshot for a peer. Overwrites the first free/LRU slot.
  PeerTouch *peerSlot(uint16_t dev) {
    for (uint8_t i = 0; i < MAX_PEERS; i++)
      if (peers[i].used && peers[i].deviceId == dev) return &peers[i];
    for (uint8_t i = 0; i < MAX_PEERS; i++)
      if (!peers[i].used) { peers[i] = {dev, 0, true}; return &peers[i]; }
    peers[0] = {dev, 0, true};  // all full: evict slot 0 (rare at M0 scale; revisit for fleet)
    return &peers[0];
  }

  void dispatchMessage(const TouchSyncHeader &h, const uint8_t *data, int dataLen) {
    if (h.sensorType == TS_SENSOR_TOUCH && dataLen >= (int)sizeof(TsTouchData)) {
      TsTouchData td;
      memcpy(&td, data, sizeof(td));
      PeerTouch *p = peerSlot(h.deviceId);
      uint16_t changed = td.touched ^ p->touched;
      for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++) {
        if (!(changed & (1u << e))) continue;
        enqueue(TS_SENSOR_TOUCH, e, (td.touched & (1u << e)) ? 1 : 0, h.deviceId, h.timestamp);
      }
      p->touched = td.touched;
    }
  }

  void receiveLoop() {
    int sz = udp.parsePacket();
    while (sz > 0) {
      if (sz >= (int)sizeof(TouchSyncHeader) && sz <= (int)RX_BUF_LEN) {
        uint8_t buf[RX_BUF_LEN];
        int n = udp.read(buf, sizeof(buf));
        TouchSyncHeader h;
        if (n >= (int)sizeof(h)) {
          memcpy(&h, buf, sizeof(h));
          if (h.magic[0] == 'A' && h.magic[1] == 'T' && h.magic[2] == 'S' && h.magic[3] == '1' &&
              h.version == TOUCH_SYNC_VERSION && h.msgType == TOUCH_SYNC_MSG_EVENT &&
              h.deviceId != deviceId &&
              (int)sizeof(h) + (int)h.dataLen <= n) {
            dispatchMessage(h, buf + sizeof(h), h.dataLen);
          }
        }
      } else {
        udp.flush();
      }
      sz = udp.parsePacket();
    }
  }

  void sendTouchSnapshot(uint16_t touched) {
    uint8_t buf[sizeof(TouchSyncHeader) + sizeof(TsTouchData)];
    TouchSyncHeader h;
    h.magic[0] = 'A'; h.magic[1] = 'T'; h.magic[2] = 'S'; h.magic[3] = '1';
    h.version    = TOUCH_SYNC_VERSION;
    h.msgType    = TOUCH_SYNC_MSG_EVENT;
    h.sensorType = TS_SENSOR_TOUCH;
    h.dataLen    = sizeof(TsTouchData);
    h.deviceId   = deviceId;
    h.seq        = txSeq++;
    h.timestamp  = millis() + strip.timebase;
    TsTouchData td; td.touched = touched;
    memcpy(buf, &h, sizeof(h));
    memcpy(buf + sizeof(h), &td, sizeof(td));

    if (udp.beginPacket(IPAddress(255, 255, 255, 255), port)) {
      udp.write(buf, sizeof(buf));
      udp.endPacket();
      txCount++;
    }
  }

  void broadcastLocalState() {
#ifdef USERMOD_MPR121
    UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
    if (!mpr || !mpr->isSensorFound()) return;

    uint16_t cur = 0;
    for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++)
      if (mpr->touched(e)) cur |= (1u << e);

    if (cur != prevLocalTouched) {
      sendTouchSnapshot(cur);   // one message carries all sensors of this device
      prevLocalTouched = cur;
    }
#endif
  }
};

const char UsermodTouchSync::_name[]    PROGMEM = "TouchSync";
const char UsermodTouchSync::_enabled[] PROGMEM = "enabled";
