# SensorSync — cross-device sensor event bus

Engineer-oriented guide to how the AMPWorks **SensorSync** bus works and is structured.
Keep this document in sync with the code — update it in the same PR whenever the wire
format, the dispatch logic, the consumer/producer API, or the transport changes.

## What it is

SensorSync lets multiple WLED devices on the same LAN share sensor activity **without a
router or broker**. A device broadcasts a full snapshot of its local sensor state (e.g. which
MPR121 electrodes are touched); peers receive the snapshot, derive discrete events (a press, a
release, a proximity/temperature sample) and hand them to effects. This is the substrate for
the installation's "touch on one device ripples across all devices" behavior.

Design goals: sensor-agnostic (touch, switches, proximity, temperature, …), tolerant of packet
loss (snapshot model self-heals), multi-consumer (several effects/segments each see every
event), and transport-swappable (UDP today; ESP-NOW / multi-hop at milestone M3).

## File map

| File | Role |
|------|------|
| `sensor_sync_protocol.h` | **Pure** wire format + logic. No WLED/Arduino deps (only `<stdint.h>`/`<string.h>`). Structs, `ss_parse_header`, `ss_dispatch`, and the multi-consumer `SensorEventRing`. This is what the host test compiles. |
| `usermod_sensor_sync.h` / `.cpp` | The WLED **usermod**: wires the pure protocol to a transport (`UdpSensorTransport`), the local producer (MPR121 poll + keyframe), config, and the consumer-facing `subscribe()`/`drain()` API. |
| `tests/sensor_sync_test.cpp` | Host unit test — includes only `sensor_sync_protocol.h`; builds with a stock `c++` compiler. |
| `ampworks.cpp` (consumers) | Effects **Touch Pond** and **Touch Grid** consume the bus: each holds its own `SensorCursor` in `SEGENV` data and `drain()`s remote events to spawn waves/chasers. |

The pure/impure split is deliberate: all protocol logic (parsing, edge derivation, dedup, the
ring) lives in the dependency-free header so it can be unit-tested off-device.

## Wire format

Every datagram is a 20-byte `SensorSyncHeader` followed by a per-type data struct
(`header.dataLen` bytes). `SENSOR_SYNC_VERSION = 5`.

```
SensorSyncHeader (20B): magic "AMPS" | version | msgType | sensorType | dataLen
                        | deviceId(u32) | seq(u16) | reserved(u16) | timestamp(u32)
```

| `sensorType` | payload | semantics |
|--------------|---------|-----------|
| `SS_SENSOR_TOUCH` (0)     | `SensorSnapshot{ u16 mask }` | bit *e* = electrode *e* active; **edge-derived** |
| `SS_SENSOR_SWITCH` (1)    | `SensorSnapshot{ u16 mask }` | bit *e* = switch *e* closed; **edge-derived** |
| `SS_SENSOR_PROXIMITY` (2) | `SensorSample{ u8 ch; i16 value }` | proximity level 0..255; delivered as-is |
| `SS_SENSOR_TEMP` (3)      | `SensorSample{ u8 ch; i16 value }` | temperature in centi-°C; delivered as-is |

`deviceId` is a 32-bit FNV-1a hash of the full MAC (override via the `id` config field). The
header/touch layout is unchanged from version 5, so older builds still interoperate for touch;
new sensor types are additive (an older peer ignores a `sensorType` it doesn't know).

## Data model

- **Snapshot, not deltas.** Producers send the *entire* current state on change. A dropped
  datagram self-corrects on the next snapshot — no permanent edge loss. This is why edge
  derivation lives on the **receiver**.
- **Edge derivation (bitmask sensors).** The receiver keeps the last mask per peer and XORs the
  new mask against it, emitting one event per changed bit (value 1 = became active, 0 = released).
- **Scalar sensors** (proximity/temp) aren't edge-derived — the sample is delivered directly.
- **Dedup.** Each peer's last `seq` is tracked; a snapshot whose `seq` is not newer
  (RFC-1982 wraparound-safe) is dropped, so relay/retransmit duplicates don't double-fire.
- **Keyframe.** Producers re-broadcast the current snapshot every `keyframeMs` (config, default
  3 s, 0 = off) so a node that booted mid-touch, or missed a change during loss, resyncs.
- **Peer aging.** Peer state idle > 30 s is freed (`sweepPeers`) so departed devices don't hold slots.

## Send path (producer)

```mermaid
flowchart TD
    A["loop() — MPR121 poll<br/>build 16-bit touch mask"] --> B{"mask changed<br/>OR keyframe due?"}
    B -- no --> Z["(nothing sent)"]
    B -- yes --> C["publishSnapshot(SS_SENSOR_TOUCH, mask)"]
    EXT["Any usermod/effect:<br/>publishSnapshot() / publishSample()"] --> C2["sendMessage(type, data, len)"]
    C --> C2
    C2 --> D["build SensorSyncHeader<br/>seq = txSeq++ · deviceId · timestamp"]
    D --> E["transport.broadcast(buf)"]
    E --> F["UdpSensorTransport →<br/>UDP 255.255.255.255 : port"]
    E -- send failed --> G["txSeq-- (unwind) · retry next loop"]
```

The MPR121 poll is the built-in producer; `publishSnapshot`/`publishSample` are public so any
other usermod or effect can emit onto the same bus (switches, proximity, temperature, …).

## Receive + act path (consumer)

```mermaid
flowchart TD
    A["loop() → receiveLoop()"] --> B["transport.poll(buf) — next datagram"]
    B --> C["ss_parse_header()<br/>magic/version/msgType OK?<br/>not our own deviceId?<br/>length ≥ header+dataLen?"]
    C -- reject --> B
    C -- ok --> D["ss_dispatch(header, data, peers, sink)"]
    D --> E{"seq newer than<br/>peer.lastSeq?"}
    E -- no (dup/reorder) --> B
    E -- yes --> F{"sensorType"}
    F -- TOUCH/SWITCH --> G["XOR mask vs peer.mask<br/>→ one RemoteSensorEvent per changed bit"]
    F -- PROX/TEMP --> H["one RemoteSensorEvent<br/>with the scalar value"]
    G --> I["ring.push(event)"]
    H --> I
    I --> J["... other datagrams ..."]
    J --> B
    K["Effect frame (Touch Pond / Touch Grid):<br/>subscribe() once → drain(cursor, out, n)"] --> L["for each event:<br/>spawn wave / chaser<br/>(remote = dimmer/distinct hue)"]
    I -.->|"consumers read the ring<br/>on their own frame"| K
```

Receiver logic (`ss_parse_header` + `ss_dispatch`) is pure and host-tested. Events land in a
monotonic ring; consumers read via cursors (below).

## Consumer API (per-consumer cursors)

Each consumer holds its own `SensorCursor` (typically in its `SEGENV` data, zero-initialized so
it subscribes lazily on the first frame). Multiple segments/effects therefore each see **every**
event — no single global drain contention. A consumer that falls more than the ring capacity
behind skips the overwritten gap rather than blocking the producer.

```cpp
UsermodSensorSync *ss = (UsermodSensorSync*) UsermodManager::lookup(USERMOD_ID_SENSOR_SYNC);
if (ss) {
  if (!data->remoteSubscribed) { data->remoteCursor = ss->subscribe(); data->remoteSubscribed = true; }
  RemoteSensorEvent ev[8];
  uint8_t n;
  while ((n = ss->drain(data->remoteCursor, ev, 8)) > 0)
    for (uint8_t i = 0; i < n; i++)
      if (ev[i].sensorType == SS_SENSOR_TOUCH && ev[i].value) { /* act on it */ }
}
```

## Producer API

```cpp
ss->publishSnapshot(SS_SENSOR_SWITCH, mask);          // bitmask sensors (edge-derived by peers)
ss->publishSample(SS_SENSOR_TEMP, channel, centiDeg); // scalar sensors (delivered as-is)
```

## Transport seam

`ISensorTransport { begin/end/broadcast/poll }` isolates the wire. `UdpSensorTransport` is the
current impl (UDP limited broadcast; packet-boundary handling stays inside it). Milestone **M3**
adds an ESP-NOW / multi-hop implementation behind the same interface — producers, consumers, and
dispatch don't change.

## Testing

```bash
cd usermods/ampworks
c++ -std=c++11 -Wall -Wextra -o /tmp/ss_test tests/sensor_sync_test.cpp && /tmp/ss_test
```

The host test exercises the pure protocol: edge derivation, seq dedup + wraparound, self-packet
rejection, short-`dataLen` rejection, proximity/temperature scalars, switch edges, unknown-type
ignore, and the ring's two-consumer independence + lag-skip. On-device (multi-device hardware)
verification is tracked as `## Testing Required` on the task.

## Extending

- **New sensor type:** add an `SS_SENSOR_*` tag + (if needed) a data struct in
  `sensor_sync_protocol.h`, add a branch to `ss_dispatch`, add a host test case, and a
  `publish*` call from the producer. No header/version change if the layout is unchanged.
- **New consumer:** look up the usermod, `subscribe()` once (store the cursor in your effect's
  state), `drain()` each frame. Filter by `sensorType`/`channel`.

## Maintenance

This README is the human-facing map of the bus. **Update it in the same change** that alters the
wire format, `ss_dispatch`, the producer/consumer API, the transport, or the sensor-type table
above. The code headers point here; keep the flowcharts accurate.
