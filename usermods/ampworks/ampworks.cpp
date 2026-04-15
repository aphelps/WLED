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
 * HMTL Sparkle: reproduces the HMTL "sparkle" program.
 *
 * Each update period, every pixel independently rolls a random chance to:
 *   - Become a new sparkle color drawn from the active palette
 *   - Reset to the background color (Color 2 slot)
 *   - Stay unchanged (providing persistence/decay between frames)
 *
 * Parameters:
 *   Speed     → update rate (high = faster flicker)
 *   Intensity → sparkle probability per pixel per frame (0–100%)
 *   c1        → background-reset probability per pixel per frame (0–100%)
 *   Color 2   → background color
 *   Palette   → sparkle color source
 */
uint16_t mode_hmtl_sparkle(void) {
  if (SEGLEN == 0) return FRAMETIME;

  // On first call fill with the background color so pixels start clean
  if (SEGENV.call == 0) {
    SEGMENT.fill(SEGCOLOR(1));
  }

  // Higher speed → shorter cycle time (faster updates); matches HMTL default ~50ms at mid-speed
  uint32_t cycleTime = 10 + (255 - SEGMENT.speed) * 2;
  uint32_t it = strip.now / cycleTime;
  if (it == SEGENV.step) return FRAMETIME; // period not yet elapsed
  SEGENV.step = it;

  // sparkle_thresh: per-pixel % chance to become a fresh sparkle color (0–100)
  uint8_t sparkle_thresh = map8(SEGMENT.intensity, 0, 100);
  // bg_thresh: combined upper bound — pixels below sparkle_thresh sparkle,
  //            pixels between sparkle_thresh and bg_thresh reset to background,
  //            pixels above bg_thresh are left unchanged (HMTL behaviour)
  uint8_t bg_thresh = sparkle_thresh + map8(SEGMENT.custom1, 0, 100);

  for (uint16_t i = 0; i < SEGLEN; i++) {
    uint8_t r = random8(100);
    if (r < sparkle_thresh) {
      SEGMENT.setPixelColor(i, SEGMENT.color_from_palette(random8(), true, false, 255));
    } else if (r < bg_thresh) {
      SEGMENT.setPixelColor(i, SEGCOLOR(1));
    }
    // else: leave pixel colour unchanged from previous frame
  }

  return FRAMETIME;
}
// Speed = update rate; Intensity = sparkle probability; c1 = BG reset probability
// Color 1 = sparkle base (palette), Color 2 = background
static const char _data_FX_MODE_HMTL_SPARKLE[] PROGMEM = "HMTL Sparkle@Rate,Sparkle,BG Reset;!,!;!;01;sx=128,ix=50,c1=20";


/*
 * Touch Ripple: reacts to MPR121 capacitive touch sensor data (USERMOD_ID_MPR121).
 *
 * Each of the 12 electrodes maps to an evenly-spaced anchor point along the segment.
 * Touching an electrode radiates a bright pulse outward with linear falloff.
 * Electrode 12 (proximity) adds a dim global glow proportional to proximity.
 * fade_out() provides natural inter-frame decay.
 *
 * Requires the mpr121 usermod to be enabled and configured.
 */
uint16_t mode_touch_ripple(void) {
#ifndef USERMOD_MPR121
  return FRAMETIME;
#else
  if (SEGLEN == 0) return FRAMETIME;

  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr || !mpr->isSensorFound()) return FRAMETIME;

  mpr->setUpdateHz(map8(SEGMENT.custom1, 1, 100));

  SEGMENT.fade_out(220);

  uint8_t rippleHalf = (uint8_t)max(1u, (uint32_t)map8(SEGMENT.speed, 2, SEGLEN / 4));

  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) {
    if (!mpr->touched(e)) continue;
    uint16_t anchor = (uint32_t)e * SEGLEN / MPR121::MAX_SENSORS;
    for (int16_t d = -(int16_t)rippleHalf; d <= (int16_t)rippleHalf; d++) {
      int16_t pos = (int16_t)anchor + d;
      if (pos < 0 || pos >= (int16_t)SEGLEN) continue;
      uint8_t bri = scale8((uint8_t)map(abs(d), 0, rippleHalf, 255, 0), SEGMENT.intensity);
      SEGMENT.addPixelColor((uint16_t)pos, color_blend(BLACK, SEGCOLOR(e % 3), bri));
    }
  }

  // Proximity electrode: dim global glow proportional to proximity reading
  uint16_t prox = mpr->getFiltered(MPR121::PROX_SENSOR);
  if (prox > 100) {
    uint8_t proxBri = scale8((uint8_t)constrain(map(prox, 100, 800, 0, 80), 0, 80), SEGMENT.intensity);
    uint32_t proxCol = color_blend(BLACK, SEGCOLOR(2), proxBri);
    for (uint16_t i = 0; i < SEGLEN; i++) SEGMENT.addPixelColor(i, proxCol);
  }

  return FRAMETIME;
#endif
}
static const char _data_FX_MODE_TOUCH_RIPPLE[] PROGMEM =
  "Touch Ripple@Speed,Intensity,Hz;!,!,!;!;01;c1=50";


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
