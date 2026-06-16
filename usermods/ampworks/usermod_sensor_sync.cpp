#include "usermod_sensor_sync.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

// RX_BUF_LEN reserves 64 bytes for sensor data after the header; the largest per-type data
// struct must fit that allowance (oversized datagrams are flushed in receiveLoop()).
static_assert(sizeof(SensorSnapshot) <= 64, "sensor data struct exceeds RX_BUF_LEN data allowance");

const char UsermodSensorSync::_name[]    PROGMEM = "SensorSync";
const char UsermodSensorSync::_enabled[] PROGMEM = "enabled";

void UsermodSensorSync::setup() {
  deviceId = configId ? configId : deriveDeviceId();
  initDone = true;
}

void UsermodSensorSync::connected() {
  udpStarted = false;  // re-bind on (re)connection; lazy begin in loop() also covers SoftAP-only
}

void UsermodSensorSync::loop() {
  if (!enabled || strip.isUpdating()) return;

  if (!(Network.isConnected() || apActive)) { udpStarted = false; return; }
  if (!udpStarted) {
    if (!udp.begin(port)) return;
    udpStarted = true;
  }

  receiveLoop();
  broadcastLocalState();
}

bool UsermodSensorSync::popRemoteEvent(RemoteSensorEvent &out) {
  if (rxCount == 0) return false;
  out = rxQueue[rxTail];
  rxTail = (rxTail + 1) % RX_QUEUE_LEN;
  rxCount--;
  return true;
}

uint32_t UsermodSensorSync::getDeviceId() const { return deviceId; }

uint16_t UsermodSensorSync::getId() { return USERMOD_ID_SENSOR_SYNC; }

void UsermodSensorSync::addToJsonInfo(JsonObject &root) {
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");
  JsonArray arr = user.createNestedArray("Sensor Sync");
  arr.add(udpStarted ? txCount : 0);
  arr.add(udpStarted ? " sent" : " (offline)");
}

void UsermodSensorSync::addToConfig(JsonObject &root) {
  JsonObject top = root.createNestedObject(FPSTR(_name));
  top[FPSTR(_enabled)] = enabled;
  top["port"]          = port;
  top["id"]            = configId;
}

bool UsermodSensorSync::readFromConfig(JsonObject &root) {
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

uint32_t UsermodSensorSync::deriveDeviceId() {
  // 32-bit FNV-1a hash of the full 6-byte MAC: folds in all device-unique entropy (incl.
  // mac[3], which a low-bytes scheme discards) so auto-IDs are effectively collision-free
  // at fleet scale. Override deterministically via the `id` config field if desired.
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  uint32_t h = 2166136261u;          // FNV offset basis
  for (uint8_t i = 0; i < 6; i++) {
    h ^= mac[i];
    h *= 16777619u;                  // FNV prime
  }
  return h ? h : 1;  // never 0 (0 means "auto")
}

void UsermodSensorSync::enqueue(uint8_t sensorType, uint8_t channel, uint8_t value,
                                uint32_t dev, uint32_t ts) {
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

// Find (or allocate) the stored snapshot for a peer. Reuses a free slot; if all are in use
// it evicts slot 0 (a re-added peer then resyncs from mask=0 — see dispatchMessage). Fine at
// M0's handful of devices; revisit with a real last-seen LRU for the fleet milestone.
UsermodSensorSync::PeerSnapshot *UsermodSensorSync::peerSlot(uint32_t dev) {
  for (uint8_t i = 0; i < MAX_PEERS; i++)
    if (peers[i].used && peers[i].deviceId == dev) return &peers[i];
  for (uint8_t i = 0; i < MAX_PEERS; i++)
    if (!peers[i].used) { peers[i] = {dev, 0, true}; return &peers[i]; }
  peers[0] = {dev, 0, true};
  return &peers[0];
}

// Consumer-side edge derivation: diff the incoming snapshot against the peer's last snapshot
// and enqueue one event per changed channel. First message from a peer (mask 0) resyncs by
// emitting a press for every already-active channel.
void UsermodSensorSync::dispatchMessage(const SensorSyncHeader &h, const uint8_t *data, int dataLen) {
  if (h.sensorType == SS_SENSOR_TOUCH && dataLen >= (int)sizeof(SensorSnapshot)) {
    SensorSnapshot snap;
    memcpy(&snap, data, sizeof(snap));
    PeerSnapshot *p = peerSlot(h.deviceId);
    uint16_t changed = snap.mask ^ p->mask;
    for (uint8_t e = 0; e < 16; e++) {
      if (!(changed & (1u << e))) continue;
      enqueue(SS_SENSOR_TOUCH, e, (snap.mask & (1u << e)) ? 1 : 0, h.deviceId, h.timestamp);
    }
    p->mask = snap.mask;
  }
}

void UsermodSensorSync::receiveLoop() {
  int sz = udp.parsePacket();
  while (sz > 0) {
    if (sz >= (int)sizeof(SensorSyncHeader) && sz <= (int)RX_BUF_LEN) {
      uint8_t buf[RX_BUF_LEN];
      int rd = udp.read(buf, sizeof(buf));
      SensorSyncHeader h;
      if (rd >= (int)sizeof(h)) {
        memcpy(&h, buf, sizeof(h));
        if (h.magic[0] == 'A' && h.magic[1] == 'M' && h.magic[2] == 'P' && h.magic[3] == 'S' &&
            h.version == SENSOR_SYNC_VERSION && h.msgType == SENSOR_SYNC_MSG_SNAPSHOT &&
            h.deviceId != deviceId &&
            (int)sizeof(h) + (int)h.dataLen <= rd) {
          dispatchMessage(h, buf + sizeof(h), h.dataLen);
        }
      }
    } else {
      udp.flush();
    }
    sz = udp.parsePacket();
  }
}

bool UsermodSensorSync::sendSnapshot(uint8_t sensorType, uint16_t mask) {
  uint8_t buf[sizeof(SensorSyncHeader) + sizeof(SensorSnapshot)];
  SensorSyncHeader h;
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version    = SENSOR_SYNC_VERSION;
  h.msgType    = SENSOR_SYNC_MSG_SNAPSHOT;
  h.sensorType = sensorType;
  h.dataLen    = sizeof(SensorSnapshot);
  h.deviceId   = deviceId;
  h.seq        = txSeq++;
  h.reserved   = 0;
  h.timestamp  = millis() + strip.timebase;
  SensorSnapshot snap; snap.mask = mask;
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), &snap, sizeof(snap));

  if (!udp.beginPacket(IPAddress(255, 255, 255, 255), port)) return false;
  udp.write(buf, sizeof(buf));
  if (!udp.endPacket()) return false;
  txCount++;
  return true;
}

void UsermodSensorSync::broadcastLocalState() {
#ifdef USERMOD_MPR121
  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr || !mpr->isSensorFound()) { prevLocalMask = 0; return; }

  uint16_t cur = 0;
  for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++)
    if (mpr->touched(e)) cur |= (1u << e);

  if (cur == prevLocalMask) return;
  // Only advance prevLocalMask on a successful send, so a failed send retries next loop.
  if (sendSnapshot(SS_SENSOR_TOUCH, cur)) prevLocalMask = cur;
#endif
}
