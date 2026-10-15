# AMPWorks usermod

Custom LED effects for AMPWorks installations (AMP AI, HMTL Sparkle, the MPR121 touch
effects, ...). Enabled by the `ampworks` PlatformIO env and the envs that extend it.

## Pinned effect ids

Each AMPWorks effect is registered at a **fixed effect id**, so saved presets keep running
the right effect when upstream WLED adds built-in effects.

| id  | effect          |
|-----|-----------------|
| 240 | AMP AI          |
| 241 | AMP AI Audio    |
| 242 | AMP Moving SIN  |
| 243 | HMTL Sparkle    |
| 244 | Touch Pond      |
| 245 | Touch Grid      |
| 246 | reserved for AMP Test (not registered) |

The table lives in `AMP_FX_SLOTS` in `ampworks.cpp`.

### Why

Presets store the numeric effect id (`fx`). The effects used to be added with
`strip.addEffect(255, ...)`, which puts them right after WLED's built-in list, so every
upstream release that grew that list renumbered them and presets silently started a different
effect (Touch Pond 169 -> 220 on TouchTower, HMTL Sparkle 161 -> 171 on Triangle Tower).

### How it works

`fx_slot.h` (`pinEffect()`) uses only WLED's stock `addEffect()`: it pads the effect list with
hidden filler entries up to the pinned id, then adds the effect exactly there. Nothing under
`wled00/` is changed, so merging upstream releases stays easy.

- The filler entries are named `RSVD AMPWorks pad`. WLED hides any effect whose name contains
  `RSVD` from the UI effect list, and `Segment::setMode` skips over ids whose name starts with
  `RSVD`, so they can't be selected.
- With 220 upstream built-ins (WLED 16, `MODE_COUNT`), ids 220-239 are padding and the effect
  count (`fxcount` in `/json/info`) is 246.
- Known side effects of the padding: a random effect (`fx:"r"`, IR random) that lands in the
  padding runs AMP AI instead, and stepping backwards (`fx:"~-"`) from 240 stays on 240.
- `/json/eff` itself still lists the 20 `RSVD AMPWorks pad` rows. The WLED web UI filters
  them, but other clients that read the raw list (WLED phone app, Home Assistant, HMTL
  tooling) may show them. The name has to contain `RSVD` for WLED's own filtering to work, so
  renaming them is not an option; at worst they are harmless, unselectable rows.

If upstream ever grows past 240 built-ins, the pins collide. The boot check then reports it
instead of failing silently: `/json/info` gets a line under `u` like
`"AMPWorks: effect Touch Pond": ["failed to pin at 244 (got 255)"]` (and a debug-serial
`ERROR AMPWorks: ...` line in `WLED_DEBUG` builds). Fix it by moving the whole block to
higher ids (and remapping presets, below).

### Adding a new effect

1. Take the **next unused id** after the current block (247, 248, ...). 246 is held for AMP Test.
2. **Never reuse or renumber an id**, even after deleting an effect: a preset somewhere still
   stores the old number.
3. Add the row to `AMP_FX_SLOTS` in `ampworks.cpp` and to the table above.

Ids must stay below 255 (255 means "no effect / next free" to WLED).

## Preset remap on upload

`tools/remap_presets.py` remaps presets **by effect name** when a firmware update changes
effect ids (the one-time move to the pinned ids, or any future mismatch).

`tools/upload_wled.py` (the HTTP OTA upload for `pio run -e ampworks -t upload`) runs it
automatically:

1. Before flashing: snapshot `/json/eff` and `/presets.json` to
   `.pio/build/<env>/preset_remap/<ip>/`.
2. After a successful (HTTP 2xx) flash: wait for the device to come back with a fresh uptime,
   map each preset's old fx id -> effect name -> new id, upload the rewritten `presets.json`,
   and reboot once if anything changed. Each change is printed, e.g.
   `preset 5 (Sparkle) seg 0: HMTL Sparkle 171 -> 243`.

Set `WLED_NO_PRESET_REMAP=1` to skip it. The post-flash wait is 60 s; set
`WLED_REMAP_TIMEOUT=<seconds>` for devices slow to rejoin WiFi. A remap problem only prints a
warning (the firmware is already flashed). If the wait times out, run the printed
`remap_presets.py apply ...` command once the device is up. A snapshot that was never applied
is kept (not overwritten) by the next upload's snapshot step when the device's effect table
has changed since, so simply re-running the upload also remaps; `snapshot --force` discards it.
(The snapshot lives under `.pio/build/<env>/`, so a `pio run -t clean` in between deletes it:
apply it before cleaning.) If the device has a settings PIN the upload is refused (HTTP 401); the
warning names the rewritten file so you can upload it by hand after unlocking.

Manual use:

```bash
python3 tools/remap_presets.py snapshot 192.168.1.72 /tmp/tt-snap   # before flashing
python3 tools/remap_presets.py apply 192.168.1.72 /tmp/tt-snap --dry-run
python3 tools/remap_presets.py apply 192.168.1.72 /tmp/tt-snap --reboot
python3 tools/remap_presets.py --self-test                          # offline test
```

Only integer `fx` values inside preset segments are rewritten. Left alone (and reported where
relevant): relative/random string values (`~`, `~-`, `r`), playlists, presets whose HTTP-API
`win` string sets `FX=`, names not in the new effect list, ambiguous names, and `RSVD`
placeholders. Re-running is a no-op, and `apply` refuses to overwrite presets that changed on
the device since the snapshot (`--force` overrides).

## Host tests

```bash
cd usermods/ampworks/tests
c++ -std=c++11 -Wall -Wextra -o /tmp/effect_slot_test effect_slot_test.cpp && /tmp/effect_slot_test
```
