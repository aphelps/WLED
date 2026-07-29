#include "usermod_sensor_sync.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

// The largest per-type data struct must fit the RX data allowance after the header.
static_assert(sizeof(SensorSnapshot) <= 64, "SensorSnapshot exceeds RX_BUF_LEN data allowance");
static_assert(sizeof(SensorSample)   <= 64, "SensorSample exceeds RX_BUF_LEN data allowance");

const char UsermodSensorSync::_name[]    PROGMEM = "SensorSync";
const char UsermodSensorSync::_enabled[] PROGMEM = "enabled";

// ---------------------------------------------------------------------------
// UDP transport
// ---------------------------------------------------------------------------
bool UdpSensorTransport::begin(uint16_t port) {
  if (started && boundPort == port) return true;
  end();
  if (!udp.begin(port)) return false;
  boundPort = port; started = true;
  return true;
}
void UdpSensorTransport::end() {
  if (started) udp.stop();
  started = false;
}
bool UdpSensorTransport::broadcast(const uint8_t *buf, int len) {
  if (!started) return false;
  if (!udp.beginPacket(IPAddress(255, 255, 255, 255), boundPort)) return false;
  udp.write(buf, len);
  return udp.endPacket();
}
int UdpSensorTransport::poll(uint8_t *buf, int maxLen) {
  if (!started) return 0;
  int sz = udp.parsePacket();
  if (sz <= 0) return 0;
  if (sz > maxLen) { udp.flush(); return 0; }   // oversized — drop this datagram
  return udp.read(buf, maxLen);
}

// ---------------------------------------------------------------------------
// ESP-NOW transport (single-hop broadcast; drop-in behind ISensorTransport)
// ---------------------------------------------------------------------------
#ifndef WLED_DISABLE_ESPNOW
bool EspNowSensorTransport::begin(uint16_t port) {
  // ESP-NOW has no ports; the radio is already up (quickEspNow started in wled.cpp). We only
  // flip on and start accepting frames from the RX hook. `port` kept for the seam; ignored.
  (void)port;
  rxRing.clear();
  started = true;
  DEBUG_PRINTLN(F("SensorSync: ESP-NOW transport started"));
  return true;
}
void EspNowSensorTransport::end() {
  started = false;
  rxRing.clear();
}
bool EspNowSensorTransport::broadcast(const uint8_t *buf, int len) {
  if (!started) return false;
  if (statusESPNow != ESP_NOW_STATE_ON) return false;   // radio not up
  // send() returns COMMS_SEND_OK (0) on enqueue success, like udp.cpp treats !err as success.
  return quickEspNow.send(ESPNOW_BROADCAST_ADDRESS, buf, (size_t)len) == COMMS_SEND_OK;
}
void EspNowSensorTransport::feed(const uint8_t *data, int len) {
  // PRODUCER (ESP-NOW RX task). The SPSC ring handles the oversized/garbage/full checks and
  // publishes atomically via its head index — safe against the concurrent poll() consumer.
  if (!started) return;
  rxRing.push(data, len);   // returns false on drop; nothing else to do
}
int EspNowSensorTransport::poll(uint8_t *buf, int maxLen) {
  // CONSUMER (loop() task). pop() skips any bad/oversized slot and keeps draining, so a single
  // malformed frame never returns 0 and strands the frames queued behind it.
  if (!started) return 0;
  return rxRing.pop(buf, maxLen);
}
#endif

// ---------------------------------------------------------------------------
// Usermod
// ---------------------------------------------------------------------------
void UsermodSensorSync::setup() {
  deviceId = configId ? configId : deriveDeviceId();
  initDone = true;
}

void UsermodSensorSync::connected() {
  started = false;  // re-bind on (re)connection
}

void UsermodSensorSync::loop() {
  if (!enabled || strip.isUpdating()) return;

  if (!(Network.isConnected() || apActive)) { started = false; return; }
  if (!started) {
    if (!transport->begin(port)) return;
    started = true;
  }

  receiveLoop();
  broadcastLocalState();
  sweepPeers();
}

uint16_t UsermodSensorSync::getId() { return USERMOD_ID_SENSOR_SYNC; }

// --- device id ---
uint32_t UsermodSensorSync::deriveDeviceId() {
  // 32-bit FNV-1a hash of the full 6-byte MAC (collision-free in practice at fleet scale).
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  uint32_t h = 2166136261u;
  for (uint8_t i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
  return h ? h : 1;  // never 0 (0 means "auto")
}

// --- producer ---
bool UsermodSensorSync::sendMessage(uint8_t sensorType, const uint8_t *data, uint8_t dataLen) {
  if (!started) return false;
  uint8_t buf[RX_BUF_LEN];
  if ((int)sizeof(SensorSyncHeader) + dataLen > (int)sizeof(buf)) return false;
  SensorSyncHeader h;
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version    = SENSOR_SYNC_VERSION;
  h.msgType    = SENSOR_SYNC_MSG_SNAPSHOT;
  h.sensorType = sensorType;
  h.dataLen    = dataLen;
  h.deviceId   = deviceId;
  h.seq        = txSeq++;
  h.ttl        = SS_DEFAULT_TTL;   // origin stamps the hop budget so routers can relay
  h.flags      = 0;
  h.timestamp  = millis() + strip.timebase;
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), data, dataLen);
  if (!transport->broadcast(buf, sizeof(h) + dataLen)) { txSeq--; return false; }  // unwind seq on fail
  txCount++;
  return true;
}

bool UsermodSensorSync::publishSnapshot(uint8_t sensorType, uint16_t mask) {
  SensorSnapshot snap; snap.mask = mask;
  return sendMessage(sensorType, reinterpret_cast<const uint8_t *>(&snap), sizeof(snap));
}

bool UsermodSensorSync::publishSample(uint8_t sensorType, uint8_t channel, int16_t value) {
  SensorSample s; s.channel = channel; s.value = value;
  return sendMessage(sensorType, reinterpret_cast<const uint8_t *>(&s), sizeof(s));
}

// --- receive ---
void UsermodSensorSync::receiveLoop() {
  uint8_t buf[RX_BUF_LEN];
  int rd;
  while ((rd = transport->poll(buf, sizeof(buf))) > 0) {
    SensorSyncHeader h;
    if (!ss_parse_header(buf, rd, deviceId, h)) continue;
    RemoteSensorEvent evbuf[16];
    SensorEventSink sink{ evbuf, 16, 0 };
    ss_dispatch(h, buf + sizeof(SensorSyncHeader), h.dataLen, peers, MAX_PEERS, sink);
    for (uint8_t i = 0; i < sink.count; i++) ring.push(evbuf[i]);
    // stamp the peer slot's last-seen for aging
    for (uint8_t i = 0; i < MAX_PEERS; i++)
      if (peers[i].used && peers[i].deviceId == h.deviceId) { peerLastSeen[i] = millis(); break; }
  }
}

#ifndef WLED_DISABLE_ESPNOW
// WLED calls this for every inbound ESP-NOW datagram (before its own linked-remote handling).
// Only feed the ring — and claim the frame — when ESP-NOW is our active transport AND the payload
// is one of OUR frames (ss_is_our_frame demuxes AMPS from WLED's 'W' sync + short/garbage). Any
// other frame: return false so WLED's own handling proceeds undisturbed.
bool UsermodSensorSync::onEspNowMessage(uint8_t *sender, uint8_t *payload, uint8_t len) {
  (void)sender;
  if (!enabled || !useEspNow || !espNowTransport.isStarted()) return false;
  if (!ss_is_our_frame(payload, (int)len)) return false;
  espNowTransport.feed(payload, (int)len);
  return true;   // claimed — it's ours; don't let WLED process it
}
#endif

void UsermodSensorSync::broadcastLocalState() {
#ifdef USERMOD_MPR121
  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr || !mpr->isSensorFound()) { prevLocalMask = 0; return; }  // reset on sensor loss

  uint16_t cur = 0;
  for (uint8_t e = 0; e < MPR121::TOTAL_SENSORS; e++)
    if (mpr->touched(e)) cur |= (1u << e);

  uint32_t now = millis();
  bool changed  = (cur != prevLocalMask);
  bool keyframe = keyframeMs && (now - lastTouchSendMs >= keyframeMs);
  if (!changed && !keyframe) return;

  if (publishSnapshot(SS_SENSOR_TOUCH, cur)) { prevLocalMask = cur; lastTouchSendMs = now; }
#endif
}

// Free peer snapshot state that has gone silent, so a returning device resyncs cleanly and
// slots aren't held by departed peers at fleet scale.
void UsermodSensorSync::sweepPeers() {
  uint32_t now = millis();
  if (now - lastPeerSweepMs < 1000) return;   // sweep at most ~1 Hz
  lastPeerSweepMs = now;
  for (uint8_t i = 0; i < MAX_PEERS; i++) {
    if (peers[i].used && (now - peerLastSeen[i] > PEER_TIMEOUT_MS)) {
      peers[i] = SensorPeer{};
      peerLastSeen[i] = 0;
    }
  }
}

// --- info / config ---
void UsermodSensorSync::addToJsonInfo(JsonObject &root) {
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");
  JsonArray arr = user.createNestedArray("Sensor Sync");
  arr.add(started ? txCount : 0);
  if (!started)      arr.add(" (offline)");
  else if (useEspNow) arr.add(" sent (ESP-NOW)");
  else                arr.add(" sent (UDP)");
}

void UsermodSensorSync::addToConfig(JsonObject &root) {
  JsonObject top = root.createNestedObject(FPSTR(_name));
  top[FPSTR(_enabled)] = enabled;
  top["port"]       = port;
  top["id"]         = configId;
  top["keyframeMs"] = keyframeMs;
  top["useEspNow"]  = useEspNow;   // false = UDP (default); true = ESP-NOW broadcast
}

bool UsermodSensorSync::readFromConfig(JsonObject &root) {
  JsonObject top = root[FPSTR(_name)];
  if (top.isNull()) return false;
  uint16_t portPrev      = port;
  bool     useEspNowPrev = useEspNow;
  getJsonValue(top[FPSTR(_enabled)], enabled);
  getJsonValue(top["port"], port);
  getJsonValue(top["id"],   configId);
  getJsonValue(top["keyframeMs"], keyframeMs);
  getJsonValue(top["useEspNow"], useEspNow);
#ifdef WLED_DISABLE_ESPNOW
  useEspNow = false;   // no ESP-NOW in this build — force UDP regardless of cfg
#endif
  bool changed = (port != portPrev) || (useEspNow != useEspNowPrev);
  // If the active transport is changing, tear down the OLD one (still pointed to by `transport`)
  // before swapping, so it releases its resources cleanly.
  if (initDone && changed && started) transport->end();

  // Point `transport` at the selected impl (UDP by default). Cheap to redo every read.
#ifndef WLED_DISABLE_ESPNOW
  transport = useEspNow ? (ISensorTransport*)&espNowTransport : (ISensorTransport*)&udpTransport;
#else
  transport = &udpTransport;
#endif
  if (initDone) {
    deviceId = configId ? configId : deriveDeviceId();
    if (changed) started = false;   // rebind on next loop (mirrors port!=portPrev)
  }
  return true;
}

// WLED 16.x self-registration.
static UsermodSensorSync sensor_sync_usermod;
REGISTER_USERMOD(sensor_sync_usermod);
