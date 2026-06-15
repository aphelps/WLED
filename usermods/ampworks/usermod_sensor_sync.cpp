#include "usermod_sensor_sync.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

// The largest single message must fit the receive buffer (oversized datagrams are flushed).
static_assert(sizeof(SensorSyncHeader) + 255 * sizeof(SensorEdge) >= sizeof(SensorSyncHeader),
              "edge batch size sanity");

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

uint16_t UsermodSensorSync::getDeviceId() const { return deviceId; }

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

uint16_t UsermodSensorSync::deriveDeviceId() {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  uint16_t id = ((uint16_t)mac[4] << 8) | mac[5];
  return id ? id : 1;  // never 0 (0 means "auto")
}

void UsermodSensorSync::enqueue(uint8_t sensorType, uint8_t channel, uint8_t value,
                                uint16_t dev, uint32_t ts) {
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

// Edges are derived at the producer, so the consumer just forwards each record to the queue.
void UsermodSensorSync::dispatchMessage(const SensorSyncHeader &h, const uint8_t *data, int dataLen) {
  int avail = dataLen / (int)sizeof(SensorEdge);
  int n = (h.count < avail) ? h.count : avail;
  for (int i = 0; i < n; i++) {
    SensorEdge e;
    memcpy(&e, data + i * sizeof(SensorEdge), sizeof(e));
    enqueue(h.sensorType, e.channel, e.value, h.deviceId, h.timestamp);
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
            h.version == SENSOR_SYNC_VERSION && h.msgType == SENSOR_SYNC_MSG_EDGES &&
            h.deviceId != deviceId) {
          dispatchMessage(h, buf + sizeof(h), rd - (int)sizeof(h));
        }
      }
    } else {
      udp.flush();
    }
    sz = udp.parsePacket();
  }
}

void UsermodSensorSync::sendEdges(uint8_t sensorType, const SensorEdge *edges, uint8_t n) {
  if (n == 0) return;
  uint8_t buf[RX_BUF_LEN];
  if (sizeof(SensorSyncHeader) + (size_t)n * sizeof(SensorEdge) > sizeof(buf)) return;
  SensorSyncHeader h;
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version    = SENSOR_SYNC_VERSION;
  h.msgType    = SENSOR_SYNC_MSG_EDGES;
  h.sensorType = sensorType;
  h.count      = n;
  h.deviceId   = deviceId;
  h.seq        = txSeq++;
  h.timestamp  = millis() + strip.timebase;
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), edges, (size_t)n * sizeof(SensorEdge));

  if (udp.beginPacket(IPAddress(255, 255, 255, 255), port)) {
    udp.write(buf, sizeof(h) + (size_t)n * sizeof(SensorEdge));
    udp.endPacket();
    txCount++;
  }
}

void UsermodSensorSync::broadcastLocalState() {
#ifdef USERMOD_MPR121
  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr || !mpr->isSensorFound()) { prevLocalTouched = 0; return; }

  uint16_t cur = 0;
  for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++)
    if (mpr->touched(e)) cur |= (1u << e);

  uint16_t changed = cur ^ prevLocalTouched;
  if (!changed) return;

  // Producer derives edges once and sends all changed channels in one message.
  SensorEdge edges[MPR121::TOTAL_SENSORS];
  uint8_t n = 0;
  for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++) {
    if (!(changed & (1u << e))) continue;
    edges[n].channel = e;
    edges[n].value   = (cur & (1u << e)) ? 1 : 0;
    n++;
  }
  sendEdges(SS_SENSOR_TOUCH, edges, n);
  prevLocalTouched = cur;
#endif
}
