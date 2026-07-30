# RS485 Bridge (AMPWorks)

Turns a WLED ESP32 into a **WiFi ↔ RS485 bridge** for legacy [HMTL](https://github.com/aphelps/HMTL)
modules, using `RS485Socket` from [aphelps/ArduinoLibs](https://github.com/aphelps/ArduinoLibs).

Two directions, both in v1:

| Direction | What happens |
|-----------|--------------|
| **Master — WiFi → RS485** | A UDP listener (default port **21331**) accepts raw HMTL-framed datagrams, validates them, and transmits them onto the RS485 bus. |
| **Slave — RS485 → WLED** | Frames addressed to this node (or broadcast) are parsed and acted on: colour/brightness sets, a small set of HMTL programs mapped to WLED effects, POLL and SET_ADDRESS. Frames for other nodes — and responses to forwarded commands — are relayed back to the last WiFi peer. |

SensorSync-over-RS485 is **not** part of this usermod; it is the follow-up task
`sensorsync-rs485-transport`. Sensor broadcasts seen on the bus are relayed and counted only.

## Enabling it

Only the `ampworks` env (and anything that extends it, e.g. `apa102_mpr121`) builds the bridge:

```ini
custom_usermods = audioreactive ampworks mpr121 rs485_bridge
build_flags = … -D USERMOD_RS485_BRIDGE -D RS485_HARDWARE_SERIAL=2
lib_extra_dirs = ${PROJECT_DIR}/../ArduinoLibs
lib_deps = …
  ${PROJECT_DIR}/../ArduinoLibs/Socket
  ${PROJECT_DIR}/../ArduinoLibs/RS485_non_blocking
  ${PROJECT_DIR}/../ArduinoLibs/RS485Utils
```

Both flags are load-bearing:

* `USERMOD_RS485_BRIDGE` opts the usermod in.
* `RS485_HARDWARE_SERIAL=<uart>` is what makes RS485Utils' `SERIAL_TYPE` resolve to
  `HardwareSerial`. Without it the library falls back to AVR-only `SoftwareSerial` and the
  build fails. (The usermod reads the *runtime* UART number from its own config; the macro
  only selects the type.)

```bash
cd WLED && pio run -e ampworks
```

### ⚠ After editing anything in `ArduinoLibs`, clear `.pio/libdeps` first

`lib_deps` lists the ArduinoLibs directories by path, so PlatformIO **copies** them into
`.pio/libdeps/<env>/<Lib>/` and then builds against that copy. It does not reliably re-sync the copy
when the source changes, so a header edit in `ArduinoLibs/` can be silently invisible:

```bash
rm -rf .pio/libdeps/ampworks/RS485Utils     # and any other ArduinoLibs dir you touched
pio run -e ampworks
```

This is not theoretical. While fixing the socket-header packing, `pio run -e ampworks` reported
SUCCESS against a `RS485B_SOCKET_HDR_LEN` that was deliberately wrong — the build was reading a
cached, pre-fix copy of `RS485Utils.h`, so the `static_assert` meant to catch exactly that mismatch
never saw the new struct. Recompiling the usermod (`touch`ing the `.cpp`) was not enough; only
deleting the `libdeps` copy was. **Before flashing anything you intend to test on hardware, clear the
copy** — otherwise the binary can disagree with the source you are reading.

### Why the build guard exists

Adding `usermods/rs485_bridge/library.json` automatically enrols this directory in two builds
that set **none** of those flags:

* `[env:usermods]` (`custom_usermods = *`), built by `.github/workflows/build.yml`
* the per-usermod matrix in `.github/workflows/usermods.yml`
  (`usermods_esp32` / `_esp32c3` / `_esp32s2` / `_esp32s3`)

and UART 2 does not exist on ESP32-C3/S2 at all. So the whole translation unit is wrapped in

```c
#if defined(USERMOD_RS485_BRIDGE) && defined(RS485_HARDWARE_SERIAL) && defined(ARDUINO_ARCH_ESP32)
```

In those builds the bridge compiles to nothing and an **inert placeholder usermod** takes its
place, reporting `RS485 Bridge: not built` on the info page. The placeholder is not cosmetic:
`pio-scripts/validate_modules.py` fails the build for any enrolled usermod that contributes no
compilation unit to the linked ELF, and a translation unit with no reachable symbol is dropped by
`--gc-sections`. `rs485_bridge_protocol.h` is *not* compiled there — it imports the wire format
from the HMTL submodule, which is only on the build path in the RS485-enabled env — so its
wire-layout `static_assert`s are covered by `pio run -e ampworks` and by the host test instead.

`ArduinoLibs/RS485Utils` carries a matching guard (`RS485UTILS_SUPPORTED`) so it too compiles to
nothing when no serial backend is available — the CI workflows put ArduinoLibs on
`PLATFORMIO_LIB_EXTRA_DIRS`, which is enough for the library dependency finder to pull it in.

The build guard alone is **not** enough to keep RS485Utils out of `[env:usermods]`, though.
PlatformIO's dependency finder runs in `chain` mode, which scans `#include` lines *without*
evaluating preprocessor conditionals: it sees `#include "RS485Utils.h"` inside the disabled
`#if RS485_BRIDGE_BUILD` block regardless, pulls RS485Utils out of ArduinoLibs and compiles it —
and `RS485Utils.cpp` includes `<RS485_non_blocking.h>` above its own guard, so the build fails
outright if that sibling library is missing. The CI workflows clone ArduinoLibs' *default* branch,
which need not have it. `[env:usermods]` therefore sets `lib_ignore = RS485Utils`: with the bridge
compiled out, no translation unit in that env can reference an RS485Utils symbol, so dropping the
library from the dependency graph is what actually matches reality. If the RS485 build flags are
ever added to that env, the ignore has to go with them.

## Wiring

A 3.3 V-tolerant RS485 transceiver (MAX3485 / SP3485 / SN65HVD75) with a single driver-enable
line:

| Transceiver | ESP32 (default) | Notes |
|-------------|-----------------|-------|
| `RO` (receiver out)  | GPIO 16 → UART RX | |
| `DI` (driver in)     | GPIO 17 ← UART TX | |
| `DE` + `/RE` (tied)  | GPIO 18           | HIGH while transmitting |
| `A` / `B`            | bus pair          | 120 Ω termination at both ends of the run |
| `VCC` / `GND`        | 3.3 V / GND       | share GND with every node on the bus |

> **⚠ GPIO 16/17 fail on PSRAM boards.** `PinManager::isPinOk()` returns `!psramFound()` for
> GPIO 16 and 17 (`wled00/pin_manager.cpp`) because those are the WROVER PSRAM pins. On any
> board with PSRAM the default pins will fail allocation and the bridge will stay disabled with
> `error: pin not usable` on the info page — pick different pins in the settings.

Pins are registered with WLED's `PinManager` under `PinOwner::UM_RS485_BRIDGE`, so the LED-bus /
button / relay settings pages report a conflict instead of silently fighting over them.
Allocation is all-or-nothing: if any of the three pins is unavailable the UART is never opened.

## Settings (Config → Usermods → RS485Bridge)

| Field | Key | Default | Notes |
|-------|-----|---------|-------|
| Enabled | `enabled` | `false` | Off unless explicitly enabled. |
| UART | `uart` | `2` | 1 or 2 (UART 0 is the serial console). **Reboot required.** |
| RX pin | `rx` | `16` | **Reboot required.** |
| TX pin | `tx` | `17` | **Reboot required.** |
| DE/RE pin | `en` | `18` | **Reboot required.** |
| Baud | `baud` | `28000` | `RS485Socket::DEFAULT_BAUD` — what existing HMTL modules use. **Reboot required.** |
| RS485 address | `addr` | `1` | This node's HMTL/socket address. Applied live. |
| Device id | `devId` | `0` | HMTL device id; `0` derives one from the MAC. |
| UDP port | `port` | `21331` | WiFi ingress port. Applied live (rebinds). |

**Why "reboot required".** `RS485Utils` keeps its serial pointer in a file-scope global and
`RS485Socket::init_general()` unconditionally re-runs `new RS485(...)` without freeing the
previous channel or `HardwareSerial`, so re-`init()`ing after a settings change leaks. `setup()`
is therefore the only caller of `HardwareSerial::begin()` and `RS485Socket::init()`. The RS485
address is the exception — it lives in `RS485Socket::sourceAddress` and is safe to change live,
which is what makes HMTL `SET_ADDRESS` work.

Verify a config round-trip with:

```bash
curl -s http://<ip>/json/cfg | python3 -m json.tool | grep -A9 RS485Bridge
```

## HMTL command subset (v1)

Everything below is accepted when the frame's destination address is this node's address or the
broadcast address `0xFFFF`.

| HMTL message | Payload | Effect on WLED |
|--------------|---------|----------------|
| `MSG_TYPE_OUTPUT` / `HMTL_OUTPUT_RGB` | `msg_rgb_t` | Main segment colour 0 ← RGB, effect forced to `FX_MODE_STATIC`. |
| `MSG_TYPE_OUTPUT` / `HMTL_OUTPUT_VALUE` | `msg_value_t` | Master brightness ← value (13-bit field clamped to 255). HMTL drives all three channels from this field, i.e. a white level. |
| `MSG_TYPE_OUTPUT` / `HMTL_OUTPUT_PROGRAM` or `_PIXELS` | `msg_program_t` | See the program map below. |
| `MSG_TYPE_POLL` | — | **A request** (no `MSG_FLAG_ACK`) is answered with an HMTL poll response (config v3 header, `object_type` `0x57`, `recv_buffer_size` 64) addressed to the **requester** — unless the poll's socket-layer source is `0xFFFF` (an unconfigured module), in which case it is refused and counted as `poll-refused`, because an ACK'd response addressed to the broadcast address would make every stock module on the bus answer it — a stock `HMTL_Module` discards a response addressed to the sender's own address. **A response** (`MSG_FLAG_ACK` set, which is how a module's own poll reply arrives, since it is sent to the poll's socket-layer source) is relayed to the WiFi peer, never answered — including when broadcast, so one broadcast ACK cannot make every node on the bus reply at once. **`recv_buffer_size` is the whole-frame budget, not the sendable payload** — subtract the 7-byte socket header for that, so 57. A master that takes 64 as a payload size and sends 64 gets `RS485B_ERR_OVERSIZE` from the node that advertised it; that conflation is in the HMTL wire format, not in this bridge. |
| `MSG_TYPE_SET_ADDR` | `msg_set_addr_t` | Adopts the new address if `device_id` is 0 ("any device") or matches this node's device id; persists it to `cfg.json`. |
| `MSG_TYPE_SENSOR` | `msg_sensor_data_t` records | Relayed to the WiFi peer and counted. Local handling is the `sensorsync-rs485-transport` follow-up. |

### Program → effect map

| HMTL program | Value | WLED effect |
|--------------|-------|-------------|
| `HMTL_PROGRAM_NONE` | `0x00` | `FX_MODE_STATIC` (0) |
| `HMTL_PROGRAM_BLINK` | `0x01` | `FX_MODE_BLINK` (1) |
| `HMTL_PROGRAM_FADE` | `0x05` | `FX_MODE_FADE` (12) |
| `HMTL_PROGRAM_SPARKLE` | `0x06` | `FX_MODE_SPARKLE` (20) |
| `HMTL_PROGRAM_CIRCULAR` | `0x08` | `FX_MODE_CHASE_COLOR` (28) |
| `PROGRAM_BRIGHTNESS` | `0x30` | Master brightness ← `values[0]` (one-shot, no mode change) |
| `PROGRAM_COLOR` | `0x31` | Main segment colour 0 ← `values[0..2]` (one-shot, no mode change) |

`TIMED_CHANGE`, `LEVEL_VALUE`, `SOUND_VALUE`, `SOUND_PIXELS` and `SEQUENCE` have **no** v1
mapping: they are counted as `unsupported` and ignored. Deeper effect mapping is future work —
the transport is the deliverable here.

Anything else addressed to this node (unknown message types, unknown output types, truncated
payloads) is counted and ignored; nothing is ever dereferenced past the validated frame length.

## Bridge semantics

* **Validation.** Every frame is checked for start code `0xFC`, protocol version 2, a
  `length` between the 8-byte header and `HMTL_MAX_MSG_LEN`, enough bytes actually present, and
  the CRC. A **zero CRC is accepted unchecked** — stock HMTL builds have `HMTL_USE_CRC`
  commented out and transmit `crc == 0`. A non-zero CRC must match (CRC-8, poly `0xD8`, init 0,
  computed over the whole message with the CRC byte treated as zero).
* **Socket-level bounds.** `RS485Utils` validates the socket-layer length itself (its checks used to
  live inside `#if DEBUG_LEVEL >= DEBUG_TRACE` and so were compiled out of release builds); a
  rejection is reported through `getRejectCount()` and counted here as a bad frame. The usermod still
  re-checks `getLength()` against the socket header + payload length before touching the payload —
  defence in depth against a library built without that fix, and the "payload must not be empty" test
  is the usermod's own.
* **Transmit is rate-bounded.** `RS485Socket::sendMsgTo` blocks: it calls `serial->flush()` while
  holding DE high, so a max-size 64-byte frame at 28000 baud is ≈47 ms of busy-wait once Gammon's
  byte-stuffing is counted. The bridge therefore transmits **at most one frame per `loop()`
  iteration** and parks the rest in a 4-slot ring buffer, dropping the oldest on overflow. A UDP
  flood cannot stall the LED refresh or trip the watchdog.
* **Ingress size cap — 57 bytes of payload, not 64.** `RS485_RECV_BUFFER` (64 B) is the budget for
  the whole *frame*, socket header included, so the usable payload is `64 - 7 = 57`. A datagram
  declaring more is rejected as `RS485B_ERR_OVERSIZE` before it reaches `sendMsgTo`, whose
  `datalength` argument is a `byte` and which writes the socket header *in front of* the caller's
  buffer. The cap used to be 64, which meant a full-size payload went on the wire as 71 bytes and
  was silently dropped by every peer on the default buffer, including this bridge's own receiver.
  `RS485B_TX_SLOT_LEN` is now derived as `RS485B_RECV_BUFFER_LEN - RS485B_SOCKET_HDR_LEN` so the two
  cannot drift apart again.
* **Relay target.** Responses and non-local traffic go to the IP/port of the most recent WiFi
  datagram. There is no peer table in v1.

## Info page counters

`Info → RS485 Bridge` shows the address (or `disabled` / `not built` / `error: <reason>`), then
`RS485 rx/tx`, `RS485 udp in/relayed`, and — only when non-zero — `RS485 dropped`, broken down into
seven figures: bad frames, unsupported commands, transmit-queue drops, rejected datagrams, receive
timeouts, framing errors, and refused polls.

The last two are worth reading carefully, because they mean different things from the rest:

* **`timeout`** — partial frames abandoned by the receive timeout. A truncated or interrupted
  transmission, not a corrupt one.
* **`framing`** — reported by `RS485_non_blocking`, which lumps together a byte failing the
  nibble-complement form check, a bad CRC, **and** a receive-buffer overflow, so it cannot tell them
  apart. On a real bus a steady climb most likely means line noise or missing termination rather than
  anything in software. It also **cannot** see a frame *this* node sent being dropped by a peer whose
  buffer was too small — that overflow happens in the peer's channel, and a half-duplex sender does
  not hear itself.

## Testing

Pure protocol/decision logic is host-testable with no Arduino toolchain:

```bash
# from the super-repo root
c++ -std=c++11 -Wall -Wextra -I HMTL/Libraries/HMTLprotocol -I ArduinoLibs/Socket \
  -o /tmp/rs485_test WLED/usermods/rs485_bridge/tests/rs485_bridge_test.cpp \
  && /tmp/rs485_test
```

The two `-I` flags point at the *real* HMTL and ArduinoLibs headers, so this doubles as the
acceptance check that `HMTLWireFormat.h` is genuinely dependency-free. Run it under a g++ >= 9 as
well as clang — the two disagree about which host-portability mistakes are diagnosable.

`tests/` is excluded from the firmware build via `library.json`'s `srcFilter`.

## Files

| File | Role |
|------|------|
| `rs485_bridge_protocol.h` | CRC-8, frame validation, the bridge decision function, the transmit ring buffer and the counters. Imports the wire format from HMTL (below). **No WLED/Arduino dependencies** — host unit-tested. |
| `usermod_rs485_bridge.h` | Usermod class + the `RS485_BRIDGE_BUILD` guard. |
| `usermod_rs485_bridge.cpp` | Transport wiring (HardwareSerial + RS485Socket + WiFiUDP), segment/brightness actions, config, and the inert placeholder for flagless builds. |
| `tests/rs485_bridge_test.cpp` | Host unit test. |

### Relationship to the HMTL submodule

`HMTL/` (super-repo root) is the source of truth for the wire format, and the bridge **imports**
it rather than copying it: `rs485_bridge_protocol.h` includes
`HMTL/Libraries/HMTLprotocol/HMTLWireFormat.h` for `msg_hdr_t`, the `MSG_TYPE_*` / `MSG_FLAG_*`
codes, the `msg_*` payload structs, `output_hdr_t`, `config_hdr_t` and the `HMTL_OUTPUT_*` /
`HMTL_PROGRAM_*` codes. There is no second copy to keep in sync.

`HMTLWireFormat.h` was extracted upstream for exactly this purpose (aphelps/HMTL, branch
`rs485-bridge-wire-format`): it takes only `<stdint.h>` and `Socket.h`, never `Arduino.h` or
`RS485Utils.h`, which is why the host unit test can compile it with a plain `c++`. The rest of
HMTL stays off the build path — `HMTLTypes.cpp` pulls in `Debug.h`, `EEPromUtils.h`,
`PixelUtil.h`, `MPR121.h` and `XBeeSocket.h`, and `HMTLMessaging.cpp` pulls in `HMTLPrograms.h`
→ `FastLED.h`, which WLED 16 removed — so nothing here includes `HMTLTypes.h` or
`HMTLMessaging.h`.

Two things remain bridge-local by necessity rather than by choice, both reimplementations of
*algorithms* (never of declarations), and both pinned by the host test:

* `rs485b_crc8()` — ArduinoLibs' `EEPROM_crc` lives in `EEPromUtils.cpp` behind AVR-only
  `EEPROM.h` and is not inline, so it cannot be linked from either the ESP32 firmware or the host
  test.
* `rs485b_hmtl_fmt()` / `rs485b_next_sensor()` — the HMTL equivalents (`hmtl_msg_fmt`,
  `hmtl_next_sensor`) live in `HMTLMessaging.cpp`, behind `Debug.h` and `FastLED.h`.

Build wiring: `${PROJECT_DIR}/../HMTL/Libraries` on `lib_extra_dirs` plus
`${PROJECT_DIR}/../HMTL/Libraries/HMTLprotocol` in `lib_deps` (`[env:ampworks]`). `HMTLprotocol`
carries a `library.json` declaring `espressif32`, which WLED's `lib_compat_mode = strict`
requires.

**Local-edit gotcha.** `lib_deps` names `HMTLprotocol` by `file://` path, so PlatformIO *copies*
it into `.pio/libdeps/<env>/HMTLprotocol` and caches it by version. Editing
`HMTL/Libraries/HMTLprotocol/HMTLWireFormat.h` in the submodule therefore has **no effect** on the
next `pio run` until that copy is cleared:

```bash
rm -rf .pio/libdeps/*/HMTLprotocol && pio run -e ampworks
```

This matters because the wire-layout `static_assert`s below are the guard against an incompatible
wire-format change — a stale copy silently checks the old layout. Fresh clones and CI copy the
current file, so it only bites in-place development.

### Wire structs are packed — sizes are the same on AVR and ESP32

Every struct in `HMTLWireFormat.h` carries `__attribute__((__packed__))`, because an ATMega328
module and this bridge talk to each other. Without it, `config_hdr_v2_t` is 8 B with `address` at
offset 3 under avr-gcc but 10 B with `address` at offset 4 on a 32-bit target (interior padding),
and `msg_poll_response_t` is 15 B versus 16 B, which made `HMTL_MSG_POLL_MIN_LEN` 23 or 24
depending on which end of the bus you asked — so an AVR master length-checking a poll response
against its own constant could reject a valid one from the bridge. Packing is layout-neutral under
avr-gcc (alignment is already 1), so deployed modules are unaffected; it makes the ESP32 layout
equal the AVR layout that is already on the wire.

The consequence for this usermod is that a single set of constants is correct for both ends:
`rs485_bridge_protocol.h`'s `static_assert` block pins every wire struct's size (there is no
longer an ABI-dependent one to exempt) and `tests/rs485_bridge_test.cpp` group 1 pins every field
offset. Run the host test under both alignment models to check the claim rather than assume it:

```bash
c++ -std=c++11 -Wall -Wextra -I ../../../HMTL/Libraries/HMTLprotocol -I ../../../ArduinoLibs/Socket \
  -o /tmp/rs485_test tests/rs485_bridge_test.cpp && /tmp/rs485_test
# and again with -fpack-struct=1 added, for the AVR-like layout
```
