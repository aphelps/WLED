#pragma once
//
// fastled_compat.h — WLED 16.x removed the FastLED dependency; its `fastled_slim` replacement
// keeps the color helpers (scale8/qadd8/qsub8/color_fade) but drops the random/map/beat helpers,
// and the core migrated its own call sites to hw_random8()/beatsin8_t(). The AMPWorks effects
// predate that, so this header re-exposes just the five removed names by forwarding them to the
// WLED-16 natives — keeping the effect code itself unchanged. Include after "wled.h"/"colors.h".
//
#include <math.h>

// random8()/random16() -> WLED's hardware PRNG (matching overloads: (), (lim), (min,lim)).
#define random8  hw_random8
#define random16 hw_random16
// beatsin8(bpm,lo,hi) -> beatsin8_t(bpm,lo,hi,timebase=0,phase=0).
#define beatsin8 beatsin8_t

// map8/sqrt16 have no WLED-16 native; express them via the still-present helpers.
static inline uint8_t  map8(uint8_t x, uint8_t lo, uint8_t hi) { return lo + scale8(x, (uint8_t)(hi - lo)); }
static inline uint16_t sqrt16(uint16_t x) { return (uint16_t)sqrtf((float)x); }
