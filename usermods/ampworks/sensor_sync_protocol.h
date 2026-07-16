#pragma once
//
// sensor_sync_protocol.h — the SensorSync wire format + pure message-dispatch logic.
//
// This header is deliberately free of any WLED / Arduino dependencies (only <stdint.h> /
// <string.h>), so the protocol can be compiled and unit-tested on a host build. The Arduino
// usermod (usermod_sensor_sync.{h,cpp}) includes this and wires it to WiFiUDP + the effect bus;
// the host test (tests/sensor_sync_test.cpp) includes only this file and exercises ss_dispatch().
//
// Wire compatibility: the header + touch snapshot are unchanged from version 5, so devices on
// older builds interoperate for touch. New sensor types are additive — a peer that doesn't know
// a sensorType simply ignores it (ss_dispatch drops unknown types).
//
#include <stdint.h>
#include <string.h>

#define SENSOR_SYNC_VERSION      5   // wire protocol version (32-bit deviceId; additive types)
#define SENSOR_SYNC_MSG_SNAPSHOT 0   // msgType: a full sensor-state snapshot

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
  uint16_t reserved;   // padding; reserved
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
