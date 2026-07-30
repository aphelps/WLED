#pragma once
//
// sensor_sync_protocol.h — the SensorSync wire format + pure message-dispatch logic.
//
// This header is deliberately free of any WLED / Arduino dependencies (only <stdint.h> /
// <string.h>), so the protocol can be compiled and unit-tested on a host build. The Arduino
// usermod (usermod_sensor_sync.{h,cpp}) includes this and wires it to WiFiUDP + the effect bus;
// the host test (tests/sensor_sync_test.cpp) includes only this file and exercises ss_dispatch().
//
// Architecture overview + send/receive flow charts: see SENSOR_SYNC.md (keep it in sync).
//
// Wire compatibility: the header + touch snapshot are unchanged from version 5, so devices on
// older builds interoperate for touch. New sensor types are additive — a peer that doesn't know
// a sensorType simply ignores it (ss_dispatch drops unknown types).
//
// Routing fields: the header's `ttl` + `flags` bytes occupy what was a 16-bit `reserved` padding
// word, so they carry routing state without changing the version. Senders that predate the split
// zeroed `reserved`, so their frames arrive as ttl=0/flags=0 — the router relay logic reads ttl==0
// as "unset" and injects SS_DEFAULT_TTL, so those frames still relay. The version stays 5
// deliberately: `ss_parse_header`/`ss_is_our_frame` gate on version==5, so raising it would break
// interoperation across the whole touch path.
//
#include <stdint.h>
#include <string.h>

#define SENSOR_SYNC_VERSION      5   // wire protocol version (32-bit deviceId; additive types)

// msgType numbers. SNAPSHOT is the only type that travels multi-hop; every number above it is a
// single-hop control frame that the receiving node consumes rather than re-broadcasts, so a new
// control type can be added without touching the relay rule.
#define SENSOR_SYNC_MSG_SNAPSHOT   0   // a full sensor-state snapshot (the multi-hop payload)
#define SENSOR_SYNC_MSG_BEACON     1   // router leader-election beacon (router to router)
#define SENSOR_SYNC_MSG_ROUTER_ADV 2   // router heartbeat advertising its routing metric
#define SENSOR_SYNC_MSG_ATTACH     3   // node announcing itself to a router (attach + keepalive)
#define SENSOR_SYNC_MSG_ATTACH_ACK 4   // router accepting an attach, granting a membership lease

// Multi-hop relay. TTL bounds flood diameter; per-origin seq dedup terminates loops. The
// origin stamps SS_DEFAULT_TTL; each relay re-broadcasts with ttl-1 and drops once ttl would reach 0.
#define SS_DEFAULT_TTL           8    // max hops from origin (mesh diameter budget)
#define SS_FLAG_RELAYED          0x01 // diagnostics-only: a router sets this on frames it re-broadcast.
                                       // NOT consumed by relay/dispatch (dedup+ttl handle loops); it
                                       // exists purely so a sniffer can tell origin frames from relays.

// Sensor type tags. TOUCH/SWITCH carry a 16-bit channel bitmask; PROXIMITY/TEMP carry a
// per-channel scalar sample (no edge derivation — the level/reading is delivered as-is).
#define SS_SENSOR_TOUCH      0   // MPR121 touch electrodes; SensorSnapshot.mask bit e = active
#define SS_SENSOR_SWITCH     1   // GPIO switches; SensorSnapshot.mask bit e = closed
#define SS_SENSOR_PROXIMITY  2   // proximity level; SensorSample.channel/value (0..255)
#define SS_SENSOR_TEMP       3   // temperature; SensorSample.channel/value (centi-degrees C)

// Generic header, common to every sensor type. 20 bytes, naturally 4-byte aligned.
struct __attribute__((packed)) SensorSyncHeader {
  char     magic[4];   // {'A','M','P','S'}
  uint8_t  version;    // SENSOR_SYNC_VERSION
  uint8_t  msgType;    // SENSOR_SYNC_MSG_*
  uint8_t  sensorType; // SS_SENSOR_* — selects the data struct that follows
  uint8_t  dataLen;    // number of sensor-data bytes following the header
  uint32_t deviceId;   // origin device id (32-bit MAC hash)
  uint16_t seq;        // per-sender sequence number (dedup / loss tracking)
  uint8_t  ttl;        // remaining hops (0 = unset, e.g. a pre-routing sender; relay injects default)
  uint8_t  flags;      // SS_FLAG_* bits (0 from senders that predate the field)
  uint32_t timestamp;  // millis()+timebase at origin
};

// Bitmask snapshot — SS_SENSOR_TOUCH and SS_SENSOR_SWITCH.
struct __attribute__((packed)) SensorSnapshot {
  uint16_t mask;       // bit e set = channel e active/closed
};

// Scalar per-channel sample — SS_SENSOR_PROXIMITY (0..255) and SS_SENSOR_TEMP (centi-deg C).
struct __attribute__((packed)) SensorSample {
  uint8_t  channel;    // which sensor of this type on the origin device
  int16_t  value;      // proximity level, or temperature in centi-degrees
};

// --- Attach protocol payloads -----------------------------------------------------------------
// A node binds to one router so the router knows who depends on it and the node knows where its
// traffic is bridged. All three frames are single-hop (ttl = 1): a router advertises its distance
// to the timebase leader, a node attaches to the router it ranks best, and the router acks with a
// membership lease. The selection + membership LOGIC lives in the esp-now-router repo; only these
// structs and their msgType numbers are shared, because both sides parse them off the wire.

#define SS_HOP_UNREACHABLE 255   // hopCost meaning "this router has no path to the leader"

// Payload of SENSOR_SYNC_MSG_ROUTER_ADV. The advertising router's own id is the header deviceId.
struct __attribute__((packed)) RouterAdvert {
  uint32_t leaderId;    // timebase leader this router routes toward (0 = none known yet)
  uint16_t leaderTerm;  // leadership term the metric was computed in (stale-advert guard)
  uint8_t  hopCost;     // hops from this router to the leader; 0 = it is the leader itself
  uint8_t  memberCount; // nodes currently attached to it (load, the secondary ranking term)
};

// Payload of SENSOR_SYNC_MSG_ATTACH. Sent by the attaching node; its id is the header deviceId.
struct __attribute__((packed)) NodeAttach {
  uint32_t routerId;    // the router being attached to; must name one router, never 0
};

// Payload of SENSOR_SYNC_MSG_ATTACH_ACK. Broadcast by the router; addressed by nodeId.
struct __attribute__((packed)) AttachAck {
  uint32_t nodeId;      // the node this ack answers
  uint32_t leaseMs;     // membership is held this long without a further attach
};

// What consumers see after the receiver derives an event from a remote message.
struct RemoteSensorEvent {
  uint8_t  sensorType;
  uint8_t  channel;
  int16_t  value;      // TOUCH/SWITCH: 1=became active, 0=became inactive. PROX/TEMP: the sample.
  uint32_t deviceId;
  uint32_t timestamp;
};

// Per-peer state the receiver maintains — pure data so ss_dispatch stays testable.
struct SensorPeer {
  uint32_t deviceId;
  uint16_t mask;       // last bitmask snapshot seen (for edge derivation)
  uint16_t lastSeq;    // last accepted seq (for dedup / reorder rejection)
  bool     used;
  bool     haveSeq;    // false until the first message sets lastSeq
};

// Caller-provided event sink for the pure dispatch (fixed buffer, no allocation).
struct SensorEventSink {
  RemoteSensorEvent *buf;
  uint8_t            cap;
  uint8_t            count;
};

// Opaque per-consumer read cursor over a SensorEventRing. A consumer obtains one from
// subscribe() (typically stored in its SEGENV data) and passes it to drain() each frame.
struct SensorCursor {
  uint32_t readSeq;
};

// Monotonic multi-consumer event ring (pure — host-testable). Producers push(); each consumer
// holds its own SensorCursor and drain()s independently, so every consumer sees every event.
// A consumer that falls more than `cap` events behind skips the gap (never blocks the producer).
// `buf` is caller-owned (cap entries); writeSeq is the total number of events ever pushed.
struct SensorEventRing {
  RemoteSensorEvent *buf;
  uint8_t            cap;
  uint32_t           writeSeq;   // total events pushed; 32-bit wraparound (~4e9 events) is unreachable in practice

  void push(const RemoteSensorEvent &e) {
    buf[writeSeq % cap] = e;
    writeSeq++;
  }
  SensorCursor subscribe() const { return SensorCursor{ writeSeq }; }  // start from "now"
  uint8_t drain(SensorCursor &cur, RemoteSensorEvent *out, uint8_t maxOut) const {
    uint32_t oldest = (writeSeq > cap) ? (writeSeq - cap) : 0;
    if (cur.readSeq < oldest) cur.readSeq = oldest;    // fell behind -> skip overwritten events
    uint8_t n = 0;
    while (cur.readSeq < writeSeq && n < maxOut) {
      out[n++] = buf[cur.readSeq % cap];
      cur.readSeq++;
    }
    return n;
  }
};

static inline void ss_sink_push(SensorEventSink &s, const RemoteSensorEvent &e) {
  if (s.count < s.cap) s.buf[s.count++] = e;
}

// Wraparound-safe "is a strictly newer than b" for 16-bit sequence numbers (RFC 1982 style).
static inline bool ss_seq_newer(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) > 0;
}

// --- Routing logic lives in the router repo ---------------------------------------------------
// The relay, leader-election and attach LOGIC (ss_router_should_relay / ss_router_relay_slot /
// SensorRouterPeer / ss_router_beacon_better / RouterBeacon, and the membership table + route
// selection) is NOT here — the WLED edge never calls it. It lives in the esp-now-router repo
// (`src/router_relay.h`, `src/router_election.h`, `src/router_attach.h`), which includes this
// header for the shared wire types. Only the on-the-wire *format* stays shared: the
// SensorSyncHeader ttl/flags fields, SS_DEFAULT_TTL (the edge sender stamps it), the attach
// payload structs, and the reserved msgType/flag numbers above.

// Find (or allocate) the peer slot for dev. Reuses a free slot; if full, evicts the peer with
// the oldest lastSeq activity is not tracked here, so evict slot 0 (documented resync-from-0).
static inline SensorPeer *ss_peer_slot(SensorPeer *peers, uint8_t maxPeers, uint32_t dev) {
  for (uint8_t i = 0; i < maxPeers; i++)
    if (peers[i].used && peers[i].deviceId == dev) return &peers[i];
  for (uint8_t i = 0; i < maxPeers; i++)
    if (!peers[i].used) { peers[i] = SensorPeer{dev, 0, 0, true, false}; return &peers[i]; }
  peers[0] = SensorPeer{dev, 0, 0, true, false};
  return &peers[0];
}

// Cheap demux classifier: is `pkt` (len bytes) one of OUR SensorSync frames? Checks length,
// magic ('AMPS'), version, and that the claimed payload fits — nothing about a specific sender.
// Used by the ESP-NOW RX hook to separate our frames from WLED's own sync traffic (magic 'W')
// and from short/garbage buffers, WITHOUT rejecting our own broadcasts (no selfId check here).
// Pure/host-testable; ss_parse_header layers the selfId + msgType checks on top for the RX path.
static inline bool ss_is_our_frame(const uint8_t *pkt, int len) {
  if (pkt == 0) return false;
  if (len < (int)sizeof(SensorSyncHeader)) return false;
  SensorSyncHeader h;
  memcpy(&h, pkt, sizeof(h));
  if (!(h.magic[0] == 'A' && h.magic[1] == 'M' && h.magic[2] == 'P' && h.magic[3] == 'S'))
    return false;
  if (h.version != SENSOR_SYNC_VERSION) return false;
  if ((int)sizeof(SensorSyncHeader) + (int)h.dataLen > len) return false;
  return true;
}

// Validate a datagram's header. Returns true and fills `out` if the packet is a well-formed
// SensorSync snapshot from another device. `selfId` is rejected so we ignore our own broadcasts.
static inline bool ss_parse_header(const uint8_t *pkt, int len, uint32_t selfId,
                                   SensorSyncHeader &out) {
  if (len < (int)sizeof(SensorSyncHeader)) return false;
  memcpy(&out, pkt, sizeof(out));
  if (!(out.magic[0] == 'A' && out.magic[1] == 'M' && out.magic[2] == 'P' && out.magic[3] == 'S'))
    return false;
  if (out.version != SENSOR_SYNC_VERSION) return false;
  if (out.msgType != SENSOR_SYNC_MSG_SNAPSHOT) return false;
  if (out.deviceId == selfId) return false;                 // ignore our own packets
  if ((int)sizeof(SensorSyncHeader) + (int)out.dataLen > len) return false;  // truncated payload
  return true;
}

// Pure dispatch: given a validated header + its data payload + the peer table, derive events
// into `sink`. Updates peer mask/seq state. Returns the number of events appended.
//
//  - Duplicate/reordered snapshots (seq not newer than the peer's last) are dropped.
//  - TOUCH/SWITCH: diff the bitmask against the peer's last snapshot; emit one event per changed
//    channel. First message from a peer (haveSeq=false) derives against mask=0, i.e. resyncs by
//    emitting a press/closed event for each already-active channel.
//  - PROXIMITY/TEMP: deliver the scalar sample directly (no edge derivation).
//  - Unknown sensor types are ignored.
static inline int ss_dispatch(const SensorSyncHeader &h, const uint8_t *data, int dataLen,
                              SensorPeer *peers, uint8_t maxPeers, SensorEventSink &sink) {
  SensorPeer *p = ss_peer_slot(peers, maxPeers, h.deviceId);

  // Dedup / reorder rejection (but always accept the very first message from a peer).
  if (p->haveSeq && !ss_seq_newer(h.seq, p->lastSeq)) return 0;

  uint8_t before = sink.count;

  if (h.sensorType == SS_SENSOR_TOUCH || h.sensorType == SS_SENSOR_SWITCH) {
    if (dataLen < (int)sizeof(SensorSnapshot)) return 0;
    SensorSnapshot snap; memcpy(&snap, data, sizeof(snap));
    uint16_t changed = snap.mask ^ p->mask;
    for (uint8_t e = 0; e < 16; e++) {
      if (!(changed & (1u << e))) continue;
      RemoteSensorEvent ev{ h.sensorType, e,
                            (int16_t)((snap.mask & (1u << e)) ? 1 : 0), h.deviceId, h.timestamp };
      ss_sink_push(sink, ev);
    }
    p->mask = snap.mask;
  } else if (h.sensorType == SS_SENSOR_PROXIMITY || h.sensorType == SS_SENSOR_TEMP) {
    if (dataLen < (int)sizeof(SensorSample)) return 0;
    SensorSample s; memcpy(&s, data, sizeof(s));
    RemoteSensorEvent ev{ h.sensorType, s.channel, s.value, h.deviceId, h.timestamp };
    ss_sink_push(sink, ev);
  } else {
    return 0;  // unknown sensor type — ignore, do not advance seq
  }

  p->lastSeq = h.seq;
  p->haveSeq = true;
  return sink.count - before;
}
