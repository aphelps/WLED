#pragma once

/*
 * Pin a custom effect to a fixed effect ID, using only WLED's stock public API.
 *
 * Why: strip.addEffect(255, ...) puts a custom effect in the next free slot, right after
 * WLED's built-in table. Every upstream release that adds built-ins renumbers the custom
 * effects, and saved presets (which store the numeric fx id) silently start a different
 * effect. Pinning each AMPWorks effect to a fixed high slot (240+) keeps the ids stable.
 *
 * How: stock WS2812FX::addEffect(id, ...) only honours an explicit id when id < size(); past
 * the end it push_back()s at the next index. So we first pad the effect list with filler
 * entries up to `id` (each one added at exactly getModeCount(), i.e. the push_back branch),
 * then add the real effect at `id`, which is now exactly the end of the list.
 *
 * Filler entries are named "RSVD..." so WLED treats them as hidden: the UI drops names
 * containing "RSVD" (data/index.js) and Segment::setMode skips ids whose name starts with
 * "RSVD" (FX_fcn.cpp). They are NOT FX.cpp's _data_RESERVED pointer, so a later
 * addEffect(255, ...) from another usermod appends past our block instead of landing in
 * our padding.
 *
 * Header-only and free of WLED includes so it can be unit tested on the host
 * (tests/effect_slot_test.cpp). No changes under wled00/ are needed.
 */

#include <stdint.h>

#ifndef PROGMEM
  #define PROGMEM
#endif

// Name for padding entries. Must start with "RSVD" (see above). Deliberately different text
// from FX.cpp's "RSVD" so the two can never be merged into one object by the toolchain.
static const char _data_AMP_RESERVED[] PROGMEM = "RSVD AMPWorks pad";

// Returned when an effect could not be placed (same convention as stock addEffect).
static const uint8_t FX_SLOT_FAILED = 255;

// Place `fn`/`name` at exactly effect id `id`. `filler` is the effect function used for the
// padding entries (it never runs: setMode skips RSVD ids). Returns the id actually used,
// which is `id` on success. Returns FX_SLOT_FAILED (255) if `id` is already taken (e.g.
// upstream grew its built-in table past it) or the list cannot grow.
//
// Strip needs: uint8_t getModeCount() const; uint8_t addEffect(uint8_t, Fn, const char*).
template<class Strip, class Fn>
uint8_t pinEffect(Strip& strip, uint8_t id, Fn fn, const char* name, Fn filler) {
  if (id == FX_SLOT_FAILED) return FX_SLOT_FAILED;   // 255 means "next free" to addEffect
  while (strip.getModeCount() < id) {
    const uint8_t before = strip.getModeCount();
    const uint8_t got = strip.addEffect(before, filler, _data_AMP_RESERVED);
    // Stop rather than spin if the list did not grow by exactly one entry at `before`
    // (allocation failure, a full list, or getModeCount() out of step with the vector).
    if (got != before || strip.getModeCount() != before + 1) return FX_SLOT_FAILED;
  }
  return strip.addEffect(id, fn, name);
}
