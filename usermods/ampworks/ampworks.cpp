#include "ampworks.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif

/*
 * AMP's initial test mode
 */
uint16_t mode_amp_test(void) {
  uint32_t cycleTime = 1000 + (255 - SEGMENT.speed)*100;
  uint32_t it = strip.now / cycleTime;
  if (it != SEGENV.step) {
    SEGENV.aux0 = (SEGENV.aux0 +1) % 256;
    SEGENV.step = it;
  }
  SEGMENT.fill(SEGMENT.color_from_palette(SEGENV.aux0,
                                     true,
                                        (strip.paletteBlend == 1 || strip.paletteBlend == 3),
                                        0));

  return FRAMETIME;
}
static const char _data_FX_MODE_AMP_TEST[] PROGMEM = "AMP Test Usermod v2.1@!;!,!;!;01";


/* This effect sets every third pixel of the segment to red */
uint16_t mode_amp_ai(void) {
  // if segment length is 0 just return
  if (SEGLEN == 0) return FRAMETIME;

  // spacing is controlled by SEGMENT.custom1 (c1). If unset (0) use default 3.
  uint16_t spacing = SEGMENT.custom1 ? SEGMENT.custom1 : 3;
  if (spacing == 0) spacing = 1; // guard
  if (spacing > SEGLEN) spacing = SEGLEN; // clamp to segment length

  // cycle time scales with speed (higher SEGMENT.speed -> faster shifting)
  uint32_t cycleTime = max(16u, 600u + (255 - SEGMENT.speed) * 8u); // tuned constant, min 16ms
  uint32_t it = strip.now / cycleTime;
  if (it != SEGENV.step) {
    SEGENV.aux0 = (SEGENV.aux0 + 1) % spacing; // shift pattern within spacing
    SEGENV.step = it;
  }

  // set every N-th pixel (with shifting offset) to segment primary color, clear others
  uint32_t col = SEGCOLOR(0);
  for (uint16_t i = 0; i < SEGLEN; i++) {
    if (((i + SEGENV.aux0) % spacing) == 0) {
      SEGMENT.setPixelColor(i, col);
    } else {
      SEGMENT.setPixelColor(i, 0);
    }
  }

  return FRAMETIME;
}
// expose 3 sliders: speed, intensity, Spacing (maps to custom1), and default c1=3
static const char _data_FX_MODE_AMP_AI[] PROGMEM = "AMP AI@!,!,Spacing;!,!;!;01;c1=3";

/* This effect sets all pixels to the same color, with the brightness based on
 * the current audio input volume
 * */
uint16_t mode_amp_ai_audio(void) {
  um_data_t *um_data = nullptr;
  // try to get audio data from audioreactive usermod, fall back to simulated sound
  if (!UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE)) {
    um_data = simulateSound(SEGMENT.soundSim);
  }
  if (!um_data) return FRAMETIME; // nothing to do

  // audio data layout (as used throughout FX.cpp):
  // u_data[0] -> float volumeSmth
  // u_data[1] -> int16_t volumeRaw
  float volumeSmth = *(float*) um_data->u_data[0];
  int16_t volumeRaw = *(int16_t*) um_data->u_data[1];

  // Normalize volumeSmth: if it's in 0..1 range scale up to 0..255
  float vol = volumeSmth;
  if (vol <= 2.0f) vol *= 255.0f;
  if (vol < 0.0f) vol = 0.0f;

  // combine smoothed float and raw int for extra dynamics (weighted)
  float combined = (vol + (float)volumeRaw) * 0.5f;

  // scale by segment intensity (0-255) and clamp
  uint8_t bri = (uint8_t)constrain((int)(combined * (float)SEGMENT.intensity / 255.0f), 0, 255);

  uint32_t col = color_blend(BLACK, SEGCOLOR(0), bri);
  SEGMENT.fill(col);

  return FRAMETIME;
}
// PROGMEM description for audio mode
static const char _data_FX_MODE_AMP_AI_AUDIO[] PROGMEM = "AMP AI Audio@!;!,!;!;01";

struct Moving_Point {
  uint16_t center;
  uint8_t width;
  int8_t direction; /* Direction of travel, -1 or 1 */
};

#define MAX_POINTS 10
struct Moving_Sin_Data {
  uint8_t count;
  struct Moving_Point points[MAX_POINTS];
};

static void init_moving_sin_point(struct Moving_Point *data) {
  data->center = random16(SEGMENT.width());
  data->direction = (random8() & 1) ? 1 : -1;
  data->width = 1;
}

static void set_moving_sin_point(struct Moving_Point *data, uint32_t color) {
  uint16_t trailing;
  if (data->center < data->width) trailing = SEGMENT.width() - (data->width - data->center);
  else trailing = data->center - data->width;
  for (uint16_t offset = 0; offset < data->width * 2; offset++) {
    SEGMENT.setPixelColor((trailing + offset) % SEGMENT.width(), color);
  }
}

uint16_t mode_amp_moving_sin(void) {
  uint8_t points = map8(SEGMENT.custom1, 1, 10);
  uint8_t i;

  if (!SEGENV.allocateData(sizeof(struct Moving_Sin_Data))) return FRAMETIME;

  struct Moving_Sin_Data *data = (struct Moving_Sin_Data *)SEGENV.data;
  if (SEGENV.call == 0) {
    /* Initialization */
    SEGMENT.fill(0);
    data->count = points;
    for (i = 0; i < data->count; i++)
      init_moving_sin_point(&data->points[i]);
  }

  /* Clear the trailing pixels */
  for (i = 0; i < data->count; i++)
    set_moving_sin_point(&data->points[i], 0);

  if (data->count != points) {
    /* Need to redo points */
    data->count = points;
    for (i = 0; i < data->count; i++)
      init_moving_sin_point(&data->points[i]);
  }

  /* Update values from the sliders */
  uint8_t speed = map8(SEGMENT.speed, 1, 10);
  uint8_t width = map8(SEGMENT.intensity, 1, 10);

  for (i = 0; i < data->count; i++) {
    auto *point = &data->points[i];
    point->width = width;

    if (point->direction > 0) {
      /* Add the speed, wrapping around the end of the segment */
      point->center = (point->center + speed) % SEGMENT.width();
    } else {
      /* Add the speed, wrapping around the beginning of the segment */
      if (speed > point->center)
        point->center = SEGMENT.width() - (speed - point->center);
      else
        point->center = point->center - speed;
    }

    set_moving_sin_point(point, SEGCOLOR(i));
  }

  return FRAMETIME;
}
// PROGMEM description for audio mode
static const char _data_FX_MODE_AMP_MOVING_SIN[] PROGMEM = "AMP Moving SIN@Speed,Width,Points;!,!;!;01;sx=32,c1=1";


/*
 * HMTL Sparkle: reproduces the HMTL "sparkle" program with smooth fading.
 *
 * Each update period, every pixel independently rolls a random chance to:
 *   - Become a new sparkle color drawn from the active palette
 *   - Reset to the background color (Color 2 slot)
 *   - Stay unchanged
 *
 * Instead of snapping instantly, each channel steps toward the target by at
 * most `fade_step` per frame. Channel-delta stepping guarantees monotonic
 * convergence at all fade speeds, avoiding the blendPixelColor integer-
 * truncation stall that causes fade-to-black artifacts at small blend values.
 * Current color is read back via getPixelColor (returns raw set value) so no
 * separate per-pixel current-state buffer is needed.
 *
 * Parameters:
 *   Speed      → update rate (high = faster flicker)
 *   Intensity  → sparkle probability per pixel per frame (0–100%)
 *   c1         → background-reset probability per pixel per frame (0–100%)
 *   c2         → fade speed (0 = instant snap, 255 = very slow dissolve; cubic curve)
 *   Color 2    → background color
 *   Palette    → sparkle color source
 */
uint16_t mode_hmtl_sparkle(void) {
  if (SEGLEN == 0) return FRAMETIME;

  unsigned dataSize = sizeof(uint32_t) * SEGLEN;
  if (!SEGENV.allocateData(dataSize)) { SEGMENT.fill(SEGCOLOR(1)); return FRAMETIME; }
  uint32_t* targets = reinterpret_cast<uint32_t*>(SEGENV.data);

  if (SEGENV.call == 0) {
    uint32_t bg = SEGCOLOR(1);
    for (uint16_t i = 0; i < SEGLEN; i++) targets[i] = bg;
    SEGMENT.fill(bg);
  }

  // On each cycle tick, assign new targets via per-pixel dice roll
  uint32_t cycleTime = 10 + (255 - SEGMENT.speed) * 2;
  uint32_t it = strip.now / cycleTime;
  if (it != SEGENV.step) {
    SEGENV.step = it;

    // sparkle_thresh: per-pixel % chance to become a fresh sparkle color (0–100)
    uint8_t sparkle_thresh = map8(SEGMENT.intensity, 0, 100);
    // bg_thresh: pixels between sparkle_thresh and bg_thresh reset to background
    uint8_t bg_thresh = sparkle_thresh + map8(SEGMENT.custom1, 0, 100);

    for (uint16_t i = 0; i < SEGLEN; i++) {
      uint8_t r = random8(100);
      if (r < sparkle_thresh) {
        targets[i] = SEGMENT.color_from_palette(random8(), true, false, 255);
      } else if (r < bg_thresh) {
        targets[i] = SEGCOLOR(1);
      }
      // else: target unchanged — pixel continues toward existing target
    }
  }

  // Cubic curve: inv³/255² stretches the slow end over more slider range than
  // quadratic. c2=0 → fade_step=255 (instant snap); c2≈212 → fade_step=1
  // (~8.5 s full range at 30 fps). Integer-only, no powf.
  uint8_t inv = 255 - SEGMENT.custom2;
  uint8_t fade_step = (uint8_t)max(1u, (uint32_t)inv * inv * inv / (255u * 255u));

  // Step each channel toward target. getPixelColor returns the raw value last
  // written by setPixelColor, so reading it back loses no precision.
  for (uint16_t i = 0; i < SEGLEN; i++) {
    uint32_t cur = SEGMENT.getPixelColor(i);
    uint8_t r = R(cur), g = G(cur), b = B(cur);
    uint8_t tr = R(targets[i]), tg = G(targets[i]), tb = B(targets[i]);

    auto step_ch = [](uint8_t from, uint8_t to, uint8_t step) -> uint8_t {
      if (from == to) return to;
      if (from < to) return (uint8_t)((to - from <= step) ? to : from + step);
      return (uint8_t)((from - to <= step) ? to : from - step);
    };

    SEGMENT.setPixelColor(i, RGBW32(step_ch(r, tr, fade_step),
                                     step_ch(g, tg, fade_step),
                                     step_ch(b, tb, fade_step), 0));
  }

  return FRAMETIME;
}
// Speed = update rate; Intensity = sparkle %; c1 = BG reset %; c2 = fade speed (0=snap, 255=slow)
// Color 1 = sparkle base (palette), Color 2 = background
static const char _data_FX_MODE_HMTL_SPARKLE[] PROGMEM = "HMTL Sparkle@Rate,Sparkle,BG Reset,Fade;!,!;!;01;sx=128,ix=50,c1=20,c2=200";


/*
 * Touch Ripple: reacts to MPR121 capacitive touch sensor data (USERMOD_ID_MPR121).
 *
 * Each of the 12 electrodes maps to an evenly-spaced anchor along the segment.
 * A new touch spawns an expanding wave: two wavefronts radiate outward in opposite
 * directions from the anchor, fading as they travel. Up to 8 waves coexist.
 * The touched anchor stays lit while held. Background fades toward black each frame.
 * Each electrode gets a distinct hue spread across the active palette.
 *
 * Speed     → wave travel speed (low=slow lingering waves, high=fast energetic)
 * Intensity → peak wave brightness
 * c1        → MPR121 poll rate (Hz)
 * c2        → trail length (low=short, high=long; default 200)
 */

#define MAX_TOUCH_WAVES 8

struct TouchWave {
  uint16_t origin;   // anchor pixel index
  uint8_t  age;      // frames elapsed since spawn
  uint8_t  maxAge;   // 0 = inactive
  uint8_t  color;    // palette index
};

struct TouchRippleData {
  uint16_t   prevTouched;               // bitmask from last frame
  TouchWave  waves[MAX_TOUCH_WAVES];
};

/*
 * Touch Pond: each new touch spawns an expanding comet wave.
 *
 * Touch onset → two wavefronts radiate outward from the electrode anchor.
 * The wavefront is a bright 3-pixel spike; behind it, a "comet tail" fills
 * the interior of the expanding disc at reduced brightness. Up to 8 waves
 * coexist, each with a distinct palette hue. While held, the anchor pixel
 * pulses with a slow beat. Background fades to black each frame.
 *
 * Speed     → wave lifetime (low=slow lingering, high=fast energetic)
 * Intensity → peak wave brightness
 * c1        → MPR121 poll rate (Hz)
 * c2        → background fade rate (low=short trail, high=long trail)
 */
uint16_t mode_touch_ripple(void) {
#ifndef USERMOD_MPR121
  return FRAMETIME;
#else
  if (SEGLEN == 0) return FRAMETIME;

  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr || !mpr->isSensorFound()) return FRAMETIME;

  mpr->setUpdateHz(map8(SEGMENT.custom1, 1, 100));

  if (!SEGENV.allocateData(sizeof(TouchRippleData))) return FRAMETIME;
  TouchRippleData *data = reinterpret_cast<TouchRippleData*>(SEGENV.data);
  if (SEGENV.call == 0) memset(data, 0, sizeof(TouchRippleData));

  // Fade background toward black; c2 controls trail length
  uint8_t fadeRate = map8(SEGMENT.custom2, 180, 250);
  for (uint16_t i = 0; i < SEGLEN; i++)
    SEGMENT.setPixelColor(i, color_fade(SEGMENT.getPixelColor(i), fadeRate, false));

  // Detect new touches (rising edges) and spawn waves
  uint16_t curTouched = 0;
  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++)
    if (mpr->touched(e)) curTouched |= (1u << e);

  uint16_t newTouches = curTouched & ~data->prevTouched;
  data->prevTouched = curTouched;

  uint8_t maxAge = map8(255 - SEGMENT.speed, 50, 200);
  uint16_t maxRadius = (uint16_t)SEGLEN;  // allow wave to fill full strip

  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) {
    if (!(newTouches & (1u << e))) continue;
    int slot = -1;
    uint8_t oldestAge = 0; int oldestSlot = 0;
    for (int w = 0; w < MAX_TOUCH_WAVES; w++) {
      if (data->waves[w].maxAge == 0) { slot = w; break; }
      if (data->waves[w].age >= oldestAge) { oldestAge = data->waves[w].age; oldestSlot = w; }
    }
    if (slot < 0) slot = oldestSlot;
    data->waves[slot] = {
      (uint16_t)((uint32_t)e * SEGLEN / MPR121::MAX_SENSORS),
      0,
      maxAge,
      (uint8_t)(e * (256 / MPR121::MAX_SENSORS))
    };
  }

  // Draw active waves: bright wavefront + comet tail filling toward origin
  for (int w = 0; w < MAX_TOUCH_WAVES; w++) {
    TouchWave &wave = data->waves[w];
    if (wave.maxAge == 0) continue;

    uint8_t frac = map(wave.age, 0, wave.maxAge, 0, 255);
    uint8_t frontBri = scale8(255 - frac, SEGMENT.intensity);
    uint32_t col = SEGMENT.color_from_palette(wave.color, false, false, 255);
    uint16_t radius = (uint32_t)wave.age * maxRadius / wave.maxAge;

    for (uint16_t p = 0; p < SEGLEN; p++) {
      uint16_t dist = (p >= wave.origin) ? (p - wave.origin) : (wave.origin - p);
      if (dist > radius + 1) continue;

      uint8_t pBri;
      if (radius <= 1 || dist + 1 >= radius) {
        // Wavefront spike: 3 pixels centered at radius from origin
        uint16_t frontDist = (dist >= radius) ? (dist - radius) : (radius - dist);
        pBri = (frontDist == 0) ? frontBri : scale8(frontBri, 150);
      } else {
        // Comet tail: dims from wavefront back toward origin
        // dist=0 (origin) = dim, dist near radius = ~half brightness
        pBri = scale8(scale8(frontBri, 100), (uint8_t)map(dist, 0, radius, 60, 200));
      }
      SEGMENT.blendPixelColor(p, col, pBri);
    }

    wave.age++;
    if (wave.age >= wave.maxAge) wave.maxAge = 0;
  }

  // Held-electrode pulse: beatsin that makes the anchor glow rhythmically
  uint8_t beat = beatsin8(30, 80, 220);  // 30 bpm, brightness range 80–220
  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) {
    if (!(curTouched & (1u << e))) continue;
    uint16_t anchor = (uint32_t)e * SEGLEN / MPR121::MAX_SENSORS;
    uint32_t col = SEGMENT.color_from_palette(e * (256 / MPR121::MAX_SENSORS), false, false, 255);
    uint8_t anchorBri = scale8(beat, SEGMENT.intensity >> 1);
    for (int8_t d = -1; d <= 1; d++) {
      int16_t pos = (int16_t)anchor + d;
      if (pos < 0 || pos >= (int16_t)SEGLEN) continue;
      uint8_t taperedBri = (d == 0) ? anchorBri : scale8(anchorBri, 160);
      SEGMENT.blendPixelColor((uint16_t)pos, col, taperedBri);
    }
  }

  return FRAMETIME;
#endif
}
static const char _data_FX_MODE_TOUCH_RIPPLE[] PROGMEM =
  "Touch Pond@Speed,Intensity,Hz,Trail;!;!;01;sx=40,ix=220,c1=50,c2=200";


// add more strings here to reduce flash memory usage
const char AMPWorks::_name[]    PROGMEM = "AMPWorks";
const char AMPWorks::_enabled[] PROGMEM = "enabled";

void AMPWorks::setup() {
  //Serial.println("Hello from my usermod!");

  // strip.addEffect(255, &mode_amp_test, _data_FX_MODE_AMP_TEST);
  strip.addEffect(255, &mode_amp_ai, _data_FX_MODE_AMP_AI); // register AMP AI mode
  strip.addEffect(255, &mode_amp_ai_audio, _data_FX_MODE_AMP_AI_AUDIO); // register AMP AI audio mode
  strip.addEffect(255, &mode_amp_moving_sin, _data_FX_MODE_AMP_MOVING_SIN);
  strip.addEffect(255, &mode_hmtl_sparkle, _data_FX_MODE_HMTL_SPARKLE);
  strip.addEffect(255, &mode_touch_ripple, _data_FX_MODE_TOUCH_RIPPLE);

  initDone = true;
}
