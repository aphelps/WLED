#pragma once

#include "wled.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

/*
 * UsermodTouchSync — broadcasts local sensor events to, and receives remote sensor
 * events from, other WLED devices on the same local network (no router required).
 *
 * M0 prototype scope (see todo_plans/multi-device-wled-mesh-sync-with-shared-touch-input.md):
 *   - Transport: UDP limited broadcast (255.255.255.255) on TOUCH_SYNC_PORT. Works over a
 *     WiFi SoftAP + stations setup with no internet/router. ESP-NOW + multi-hop mesh come later.
 *   - Producer: polls the MPR121 usermod each loop and broadcasts a packet on every
 *     press/release edge.
 *   - Consumer: receives remote packets into an in-RAM ring buffer that effects drain via
 *     popRemoteEvent(). Touch Pond (ampworks.cpp) spawns a distinct-colored wave per remote press.
 *
 * The packet layout is deliberately a TYPED SENSOR EVENT (sensorType + channel + value), not
 * touch-specific, so M1 can add switches/proximity/temperature producers without a wire change.
 *
 * Device ID is derived from the low 2 bytes of the station MAC (override via config "id", 0=auto).
 * A device ignores packets bearing its own ID, so broadcast loopback is harmless.
 *
 * Other code accesses received events by looking up this usermod directly:
 *   #ifdef USERMOD_TOUCH_SYNC
 *   UsermodTouchSync *ts = (UsermodTouchSync*) UsermodManager::lookup(USERMOD_ID_TOUCH_SYNC);
 *   RemoteSensorEvent ev;
 *   while (ts && ts->popRemoteEvent(ev)) { ... }
 *   #endif
 *
 * NOTE (M0 limitation): popRemoteEvent() drains a single global queue. With one active segment
 * running Touch Pond this is correct; with multiple consuming segments the first to run would
 * drain the others. Generalize to per-consumer cursors in M1.
 *
 * Config (cfg.json):
 *   enabled — run the sync usermod (default true)
 *   port    — UDP port (default 21330; avoids 21324 notifier / 65506 node-list)
 *   id      — device id override; 0 = auto-derive from MAC (default 0)
 */

#ifndef TOUCH_SYNC_PORT
  #define TOUCH_SYNC_PORT 21330
#endif

// Sensor type tags (extend in M1)
#define TS_SENSOR_TOUCH   0   // MPR121 electrode; value = 1 press / 0 release

// Wire packet — fixed layout, packed so it is identical across all devices.
struct __attribute__((packed)) TouchSyncPacket {
  uint8_t  magic[3];   // 'A','T','S'
  uint8_t  version;    // protocol version (1)
  uint8_t  msgType;    // 0 = sensor event
  uint8_t  sensorType; // TS_SENSOR_* (which kind of sensor)
  uint8_t  channel;    // electrode / channel index
  uint8_t  value;      // event value (e.g. 1=press, 0=release)
  uint16_t deviceId;   // origin device id
  uint16_t seq;        // per-sender sequence number (for future dedup/loss tracking)
  uint32_t timestamp;  // millis() + strip.timebase at origin
};

#define TOUCH_SYNC_VERSION  1
#define TOUCH_SYNC_MSG_EVENT 0

// What effects/consumers see after draining the receive queue.
struct RemoteSensorEvent {
  uint8_t  sensorType;
  uint8_t  channel;
  uint8_t  value;
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
    // Re-bind on (re)connection; lazy begin in loop() also covers SoftAP-only bring-up.
    udpStarted = false;
  }

  void loop() override {
    if (!enabled || strip.isUpdating()) return;

    // Need a usable network (station connected OR SoftAP active) before any UDP.
    if (!(Network.isConnected() || apActive)) { udpStarted = false; return; }
    if (!udpStarted) {
      if (!udp.begin(port)) return;
      udpStarted = true;
    }

    receiveLoop();
    broadcastLocalEdges();
  }

  // ── Consumer API ────────────────────────────────────────────────────────────
  // Pops the oldest received remote event. Returns false when the queue is empty.
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

  bool     initDone   = false;
  bool     enabled    = true;
  uint16_t port       = TOUCH_SYNC_PORT;
  uint16_t configId   = 0;       // 0 = auto-derive
  uint16_t deviceId   = 0;

  WiFiUDP  udp;
  bool     udpStarted = false;
  uint16_t txSeq      = 0;
  uint32_t txCount    = 0;

  RemoteSensorEvent rxQueue[RX_QUEUE_LEN];
  uint8_t  rxHead = 0;   // next write
  uint8_t  rxTail = 0;   // next read
  uint8_t  rxCount = 0;

  uint16_t prevTouched = 0;  // MPR121 edge-detection state

  static uint16_t deriveDeviceId() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    uint16_t id = ((uint16_t)mac[4] << 8) | mac[5];
    return id ? id : 1;  // never 0 (0 means "auto")
  }

  void pushRemoteEvent(const TouchSyncPacket &pkt) {
    RemoteSensorEvent &slot = rxQueue[rxHead];
    slot.sensorType = pkt.sensorType;
    slot.channel    = pkt.channel;
    slot.value      = pkt.value;
    slot.deviceId   = pkt.deviceId;
    slot.timestamp  = pkt.timestamp;
    rxHead = (rxHead + 1) % RX_QUEUE_LEN;
    if (rxCount < RX_QUEUE_LEN) rxCount++;
    else rxTail = (rxTail + 1) % RX_QUEUE_LEN;  // overwrite oldest
  }

  void receiveLoop() {
    int sz = udp.parsePacket();
    while (sz > 0) {
      if (sz >= (int)sizeof(TouchSyncPacket)) {
        TouchSyncPacket pkt;
        udp.read((uint8_t*)&pkt, sizeof(pkt));
        if (pkt.magic[0] == 'A' && pkt.magic[1] == 'T' && pkt.magic[2] == 'S' &&
            pkt.version == TOUCH_SYNC_VERSION && pkt.msgType == TOUCH_SYNC_MSG_EVENT &&
            pkt.deviceId != deviceId) {
          pushRemoteEvent(pkt);
        }
      } else {
        udp.flush();
      }
      sz = udp.parsePacket();
    }
  }

  void sendEvent(uint8_t sensorType, uint8_t channel, uint8_t value) {
    TouchSyncPacket pkt;
    pkt.magic[0] = 'A'; pkt.magic[1] = 'T'; pkt.magic[2] = 'S';
    pkt.version    = TOUCH_SYNC_VERSION;
    pkt.msgType    = TOUCH_SYNC_MSG_EVENT;
    pkt.sensorType = sensorType;
    pkt.channel    = channel;
    pkt.value      = value;
    pkt.deviceId   = deviceId;
    pkt.seq        = txSeq++;
    pkt.timestamp  = millis() + strip.timebase;

    if (udp.beginPacket(IPAddress(255, 255, 255, 255), port)) {
      udp.write((uint8_t*)&pkt, sizeof(pkt));
      udp.endPacket();
      txCount++;
    }
  }

  void broadcastLocalEdges() {
#ifdef USERMOD_MPR121
    UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
    if (!mpr || !mpr->isSensorFound()) return;

    uint16_t cur = 0;
    for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++)
      if (mpr->touched(e)) cur |= (1u << e);

    uint16_t changed = cur ^ prevTouched;
    if (changed) {
      for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) {
        if (!(changed & (1u << e))) continue;
        sendEvent(TS_SENSOR_TOUCH, e, (cur & (1u << e)) ? 1 : 0);
      }
      prevTouched = cur;
    }
#endif
  }
};

const char UsermodTouchSync::_name[]    PROGMEM = "TouchSync";
const char UsermodTouchSync::_enabled[] PROGMEM = "enabled";
