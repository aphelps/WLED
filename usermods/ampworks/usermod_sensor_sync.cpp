#include "usermod_sensor_sync.h"
#ifdef ARDUINO_ARCH_ESP32
  #include <Preferences.h>          // NVS-backed control-clock ceiling (plan decision 5)
  static const char *SS_NVS_NS  = "ampsync";
  static const char *SS_NVS_KEY = "ctrlceil";
#endif
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
    // Ask where the mesh's clock is. Done here rather than in setup() because it needs the
    // transport up, and repeated on every re-attach: connected() clears `started`, and a node that
    // dropped out may have missed enough commands to be behind on its return.
    beginClockQuery();
  }

  // Poll BEFORE closing the window so replies that arrived during it are counted; a window that
  // expires in the same pass that would have read its replies is a window of zero length.
  receiveLoop();
  if (!clockReady && (int32_t)(millis() - queryDeadline) >= 0) finishClockQuery();

  publishPendingControl();
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
bool UsermodSensorSync::sendMessage(uint8_t msgType, uint8_t sensorType, const uint8_t *data,
                                    uint8_t dataLen) {
  if (!started) return false;
  uint8_t buf[RX_BUF_LEN];
  if ((int)sizeof(SensorSyncHeader) + dataLen > (int)sizeof(buf)) return false;
  SensorSyncHeader h;
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version    = SENSOR_SYNC_VERSION;
  h.msgType    = msgType;
  h.sensorType = sensorType;
  h.dataLen    = dataLen;
  h.deviceId   = deviceId;
  h.seq        = txSeq++;
  h.ttl        = SS_DEFAULT_TTL;   // origin stamps the hop budget so routers can relay
  h.flags      = 0;
  h.timestamp  = millis() + strip.timebase;
  memcpy(buf, &h, sizeof(h));
  // Guarded: CTRL_QUERY has no payload and passes (nullptr, 0). memcpy with a null source is
  // undefined behaviour even for a zero length — it happens to work, and sanitizers rightly
  // complain. A payload-free message type is legitimate, so the guard belongs here.
  if (dataLen) memcpy(buf + sizeof(h), data, dataLen);
  if (!transport->broadcast(buf, sizeof(h) + dataLen)) { txSeq--; return false; }  // unwind seq on fail
  txCount++;
  return true;
}

bool UsermodSensorSync::publishSnapshot(uint8_t sensorType, uint16_t mask) {
  SensorSnapshot snap; snap.mask = mask;
  return sendMessage(SENSOR_SYNC_MSG_SNAPSHOT, sensorType,
                     reinterpret_cast<const uint8_t *>(&snap), sizeof(snap));
}

bool UsermodSensorSync::publishSample(uint8_t sensorType, uint8_t channel, int16_t value) {
  SensorSample s; s.channel = channel; s.value = value;
  return sendMessage(SENSOR_SYNC_MSG_SNAPSHOT, sensorType,
                     reinterpret_cast<const uint8_t *>(&s), sizeof(s));
}

// --- control plane: gateway path ---
// Apply locally first, then broadcast. Local-first means the phone sees its own node respond at
// once instead of waiting for a round trip, and it costs nothing in convergence: our own frame is
// stamped with this node's deviceId, so ss_ctrl_should_apply rejects it if it loops back to us.
//
// It does not route through applyControl(), which owns the decision for a REMOTE frame; the
// originator records itself explicitly instead (below). Recording is not optional — an earlier
// version skipped it on the theory that it would block a same-clock higher-deviceId command, which
// is simply false: the deviceId tie-break admits that command either way.
//
// INVARIANT — nothing may broadcast control state except an explicit local command.
// This is what makes roaming safe: when a phone moves to another node, that node must not announce
// its own idea of the state and undo something newer. It holds by construction today (this is the
// only producer, and `connected()` merely rebinds), NOT by a check, so it is easy to break by
// accident. In particular, do not give control frames the periodic keyframe re-broadcast that
// snapshots get for reliability: a re-sent old command would be indistinguishable from a new one to
// a node that had rebooted and lost its ordering state, and would resurrect state the user had
// already moved on from.
// --- control-clock durability (plan decision 5) -------------------------------------------------
//
// A reservation, not the clock itself: we persist a CEILING and spend values below it from RAM, so
// flash sees one write per boot and per SS_CTRL_RESERVE commands rather than one per command.
// Persisting the clock directly would tie flash lifetime to how often the user taps — fine at human
// rates, ~73 days at 10 commands/sec, which is a trap for any later automation.
uint32_t UsermodSensorSync::loadCeiling() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (!p.begin(SS_NVS_NS, true)) return 0;     // never written yet: a new node, ceiling 0
  uint32_t v = p.getUInt(SS_NVS_KEY, 0);
  p.end();
  return v;
#else
  return 0;
#endif
}

void UsermodSensorSync::storeCeiling(uint32_t v) {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (!p.begin(SS_NVS_NS, false)) return;
  p.putUInt(SS_NVS_KEY, v);
  p.end();
#else
  (void)v;
#endif
}

// Claim the next block. Called BEFORE the clock crosses the ceiling, never after, so a value is
// only ever spent once it is backed by flash — otherwise a power cut between spending and writing
// would let the next boot reissue clocks this node has already published.
bool UsermodSensorSync::reserveClockBlock() {
  uint32_t next = ss_ctrl_next_ceiling(control.clock);
  if (next == clockCeiling) return false;      // saturated at u32 max: refuse rather than wrap
  clockCeiling = next;
  storeCeiling(clockCeiling);
  return true;
}

// Ask the mesh where the clock is. Broadcast once on (re)start; replies land in receiveLoop.
void UsermodSensorSync::beginClockQuery() {
  clockCeiling    = loadCeiling();
  // Raise-only, never assign: a WiFi re-attach re-runs this, and observation may have already
  // carried the clock far past the ceiling. Overwriting would regress it and re-mute the node —
  // the exact symptom this mechanism removes.
  ss_ctrl_adopt(control, clockCeiling);        // floor: never regress below what we already spent
  clockReplyCount = 0;
  clockReady      = false;
  queryDeadline   = millis() + SS_CTRL_QUERY_WINDOW_MS;
  sendMessage(SENSOR_SYNC_MSG_CTRL_QUERY, 0, nullptr, 0);
}

// Window closed: believe the replies, or fall back to what we persisted. Both inputs matter — a
// new node has no ceiling and needs the replies; a lone node hears nothing and needs the ceiling.
void UsermodSensorSync::finishClockQuery() {
  uint32_t consensus = ss_ctrl_reply_consensus(clockReplies, clockReplyCount, clockCeiling);
  // Adopt (raise-only), never assign: anything observed DURING the window is already ours and
  // must survive a window that closes with zero or losing replies.
  ss_ctrl_adopt(control, ss_ctrl_start(clockCeiling, consensus));
  // The mesh may be far past anything we have flash-backed; re-anchor the ceiling before spending.
  if (ss_ctrl_needs_reserve(control.clock, clockCeiling)) reserveClockBlock();
  clockReady = true;
}

bool UsermodSensorSync::publishControl(const SensorControl &cmd) {
  if (!started) return false;
  // Until the query window closes we do not know where the mesh's clock is. Publishing anyway
  // would stamp a low value that every peer rejects — the user's tap would silently do nothing,
  // which is the exact symptom this mechanism exists to remove.
  if (!clockReady) return false;
  // Claim flash-backed headroom BEFORE spending the value, so a power cut can never let the next
  // boot reissue a clock this node already published.
  if (ss_ctrl_needs_reserve(control.clock + 1, clockCeiling) && !reserveClockBlock()) return false;
  SensorControl c = cmd;
  c.lamport = ss_ctrl_tick(control);      // our command competes in the same total order as any other

  // Broadcast BEFORE recording or applying. Ordering matters: recording a command the mesh never
  // received would leave this node holding state nobody else has, and then rejecting the peer
  // command everyone else applied — the same divergence as not recording at all, moved to the
  // failure path. On a send failure we change nothing locally, so the node stays consistent with
  // the installation and the user simply taps again.
  //
  // There is deliberately no retry queue: control is user-driven, and a queue would need its own
  // ordering story against commands issued while it drained.
  if (!sendMessage(SENSOR_SYNC_MSG_CONTROL, 0,
                   reinterpret_cast<const uint8_t *>(&c), sizeof(c))) return false;

  // Record our own command: this node competes in the same total order it imposes on everyone
  // else. Skipping it left `have` false, so the next stale command was accepted here while every
  // peer correctly rejected it.
  ss_ctrl_record_own(control, c.lamport, deviceId);
  applyControlFields(c);
  return true;
}

bool UsermodSensorSync::publishPreset(uint8_t presetId) {
  SensorControl c{};
  c.fields   = SS_CTRL_PRESET;
  c.presetId = presetId;
  return publishControl(c);      // publishControl stamps c.lamport
}

// --- control plane: apply path ---
// Mutate WLED state from a command. No ordering state is touched here, so this is equally usable
// for a remote command (via applyControl) and for our own (via publishControl).
//
// CALL_MODE_NO_NOTIFY throughout: this state change is already being distributed by the mesh, and
// letting WLED's own notifier re-announce it would put the same change onto the network twice by a
// second path, with its own echo characteristics we do not control.
void UsermodSensorSync::applyControlFields(const SensorControl &c) {
  // A preset carries a whole look, so it is applied first and alone — the individual fields below
  // would otherwise be applied on top of it and partially override what the preset just set.
  if (c.fields & SS_CTRL_PRESET) {
    if (c.presetId >= 1 && c.presetId <= 250) applyPreset(c.presetId, CALL_MODE_NO_NOTIFY);
    return;
  }

  bool changed = false;
  if (c.fields & SS_CTRL_EFFECT) {
    if (c.effectId < strip.getModeCount()) { strip.getMainSegment().setMode(c.effectId); changed = true; }
  }
  if (c.fields & SS_CTRL_PALETTE) {
    if (c.paletteId < getPaletteCount()) { strip.getMainSegment().setPalette(c.paletteId); changed = true; }
  }
  if (c.fields & SS_CTRL_COLOUR) {
    strip.getMainSegment().setColor(0, c.colour);
    changed = true;
  }
  if (c.fields & SS_CTRL_BRIGHTNESS) {
    bri = c.brightness;
    changed = true;
  }
  if (changed) stateUpdated(CALL_MODE_NO_NOTIFY);

}

// Decide whether a REMOTE command should be applied, then apply it. The decision is pure and lives
// in sensor_control.h, so the conflict rule and echo suppression are host-tested rather than only
// exercised on hardware.
void UsermodSensorSync::applyControl(const SensorSyncHeader &h, const SensorControl &c) {
  // Ordering is on c.lamport, NOT h.timestamp — see sensor_control.h for why the header timestamp
  // is not a shared clock. should_apply also advances our own clock from what we heard.
  if (!ss_ctrl_should_apply(control, c.lamport, h.deviceId, deviceId)) return;
  applyControlFields(c);
  // Deliberately NOT re-broadcast. The router's per-origin dedup stops a frame circulating the
  // backbone, but nothing stops an edge from re-originating what it just applied under its OWN
  // deviceId — which would defeat that dedup entirely and flood the mesh.
}

// --- control plane: gateway hook ---
// WLED calls this from stateUpdated() for every state change, with the callMode that caused it.
// Only genuinely user-driven modes originate a mesh command.
//
// Filtering on callMode is also what suppresses echo at this layer, and it is why applyControlFields
// uses CALL_MODE_NO_NOTIFY: a command we applied from the mesh comes back through here as
// NO_NOTIFY and is ignored, so it cannot be re-originated under our own deviceId. Likewise
// CALL_MODE_NOTIFICATION (WLED's own sync) is excluded, so the two sync mechanisms do not feed
// each other.
void UsermodSensorSync::onStateChange(uint8_t mode) {
  if (!enabled || !gateway || !started) return;
  if (mode != CALL_MODE_DIRECT_CHANGE && mode != CALL_MODE_BUTTON && mode != CALL_MODE_ALEXA) return;

  // Do NOT publish from here. This runs wherever the state change came from, and for the most
  // common case — POST /json/state — that is the AsyncTCP task, not loop(). Publishing inline would
  // race txSeq, the ControlState, and the transport against loop()'s own use of them. Set a flag
  // and let loop() do the work on the task that owns those.
  //
  // Coalescing to a single pending flag is deliberate: several state fields changing in one
  // interaction should reach the mesh as one command, not one per field.
  controlPending = true;
}

// Runs on the loop() task. Reads the state as it now stands, which is also why coalescing is
// correct — whatever the final state of a burst of changes is, that is what the mesh should get.
void UsermodSensorSync::publishPendingControl() {
  // The clockReady gate is known-temporary (a query window is ~1.5s), so a tap landing inside it
  // stays pending instead of being consumed against a publish that refuses — state is read at
  // publish time, so deferred taps coalesce for free.
  if (!controlPending || !clockReady) return;
  controlPending = false;

  Segment &seg = strip.getMainSegment();
  SensorControl c{};
  c.fields     = SS_CTRL_EFFECT | SS_CTRL_PALETTE | SS_CTRL_BRIGHTNESS | SS_CTRL_COLOUR;
  c.effectId   = seg.mode;
  c.paletteId  = seg.palette;
  c.brightness = bri;
  c.colour     = seg.colors[0];
  publishControl(c);
}

// --- receive ---
void UsermodSensorSync::receiveLoop() {
  uint8_t buf[RX_BUF_LEN];
  int rd;
  while ((rd = transport->poll(buf, sizeof(buf))) > 0) {
    // Control frames are a separate wire path with their own validation; ss_parse_header is left
    // gating strictly on snapshots so the sensor path cannot be handed a frame it has no
    // handler for.
    {
      SensorSyncHeader ch;
      SensorControl    cc;
      if (ss_parse_control(buf, rd, deviceId, ch, cc)) { applyControl(ch, cc); continue; }

      // A peer asking where the clock is. Any node can answer, not just gateways: ss_ctrl_observe
      // keeps `control.clock` current on every command a node hears, including ones it rejected,
      // so a pure listener holds the same value a publisher does. That is what makes recovery work
      // in the single-gateway topology, where the only node that publishes is the one asking.
      SensorControlClock qc;
      if (ss_parse_ctrl_query(buf, rd, deviceId, ch)) {
        qc.clock = control.clock;
        sendMessage(SENSOR_SYNC_MSG_CTRL_CLOCK, 0,
                    reinterpret_cast<const uint8_t *>(&qc), sizeof(qc));
        continue;
      }

      // A reply to our own query. Only collected while our window is open — outside it we have
      // already committed to a clock, and adopting late would let a stale reply move us backwards
      // relative to commands we have since published.
      if (ss_parse_ctrl_clock(buf, rd, deviceId, ch, qc)) {
        if (!clockReady && clockReplyCount < SS_CTRL_MAX_REPLIES)
          clockReplies[clockReplyCount++] = SsCtrlReply{ qc.clock, ch.deviceId };
        continue;
      }
    }
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
  top["gateway"]    = gateway;
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
  getJsonValue(top["gateway"], gateway);
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
