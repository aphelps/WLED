#include "ampworks.h"
#ifdef USERMOD_MPR121
  #include "../usermods/mpr121/usermod_mpr121.h"
#endif
#ifdef USERMOD_SENSOR_SYNC
  #include "../usermods/ampworks/usermod_sensor_sync.h"
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

#define MAX_TOUCH_WAVES  8
#define MAX_HISTORY      16
#define IDLE_THRESHOLD   90    // frames (~3s at 30fps) before idle replay starts
#define REPLAY_SCALE      4    // timeDelta units = this many frames
#define REPLAY_PAUSE    150    // frames to pause between replay cycles

struct TouchEvent {
  uint8_t electrode;   // 0-11
  uint8_t timeDelta;   // frames since previous touch / REPLAY_SCALE, capped to 255
};

struct TouchWave {
  uint16_t origin;    // anchor pixel index
  uint8_t  age;       // frames elapsed since spawn
  uint8_t  maxAge;    // 0 = inactive
  uint8_t  color;     // palette index
  uint8_t  ghostBri;  // 255 = real touch; ~80 = ghost replay wave
};

struct TouchRippleData {
  uint16_t   prevTouched;                // bitmask from last frame
  TouchWave  waves[MAX_TOUCH_WAVES];
  TouchEvent history[MAX_HISTORY];       // ring buffer of recent touch events
  uint8_t    histHead;                   // next write position in ring
  uint8_t    histCount;                  // valid entries (0..MAX_HISTORY)
  uint16_t   idleFrames;                 // frames since last real touch
  uint8_t    replayIdx;                  // next history index to replay
  uint16_t   replayWait;                 // countdown frames until next ghost event
  uint8_t    agcPeak;                    // AGC: running peak for volume normalization
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
#ifdef USERMOD_MPR121
// Default palette hue for an electrode's wave (local/ghost/audio waves).
static inline uint8_t touchWaveColor(uint8_t e) {
  return (uint8_t)(e * (256 / MPR121::MAX_SENSORS));
}

static void spawnTouchWave(TouchRippleData *data, uint8_t e, uint16_t segLen, uint8_t maxAge, uint8_t ghostBri, uint8_t color) {
  int slot = -1;
  uint8_t oldestAge = 0; int oldestSlot = 0;
  for (int w = 0; w < MAX_TOUCH_WAVES; w++) {
    if (data->waves[w].maxAge == 0) { slot = w; break; }
    if (data->waves[w].age >= oldestAge) { oldestAge = data->waves[w].age; oldestSlot = w; }
  }
  if (slot < 0) slot = oldestSlot;
  data->waves[slot] = {
    (uint16_t)((uint32_t)e * segLen / MPR121::MAX_SENSORS),
    0, maxAge,
    color,
    ghostBri
  };
}
#endif

uint16_t mode_touch_ripple(void) {
#ifndef USERMOD_MPR121
  return FRAMETIME;
#else
  if (SEGLEN == 0) return FRAMETIME;

  // mpr may report no physical sensor (display-only node): the touched() wrappers then
  // return false, so local-touch logic is inert but remote/audio waves still render.
  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (!mpr) return FRAMETIME;

  if (mpr->isSensorFound()) mpr->setUpdateHz(map8(SEGMENT.custom1, 1, 100));

  if (!SEGENV.allocateData(sizeof(TouchRippleData))) return FRAMETIME;
  TouchRippleData *data = reinterpret_cast<TouchRippleData*>(SEGENV.data);
  if (SEGENV.call == 0) { memset(data, 0, sizeof(TouchRippleData)); data->agcPeak = 128; }

  // Fade background toward black; c2 controls trail length
  uint8_t fadeRate = map8(SEGMENT.custom2, 180, 250);
  for (uint16_t i = 0; i < SEGLEN; i++)
    SEGMENT.setPixelColor(i, color_fade(SEGMENT.getPixelColor(i), fadeRate, false));

  // Detect touched electrodes and rising edges
  uint16_t curTouched = 0;
  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++)
    if (mpr->touched(e)) curTouched |= (1u << e);

  uint16_t newTouches = curTouched & ~data->prevTouched;
  data->prevTouched = curTouched;

  uint8_t maxAge = map8(255 - SEGMENT.speed, 50, 200);
  uint16_t maxRadius = (uint16_t)SEGLEN;

  // Record new touches to history ring buffer and spawn real waves
  for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) {
    if (!(newTouches & (1u << e))) continue;
    uint8_t delta = (uint8_t)min((uint32_t)255, (uint32_t)data->idleFrames / REPLAY_SCALE);
    data->history[data->histHead] = {e, delta};
    data->histHead = (data->histHead + 1) % MAX_HISTORY;
    if (data->histCount < MAX_HISTORY) data->histCount++;
    spawnTouchWave(data, e, SEGLEN, maxAge, 255, touchWaveColor(e));
  }

  // Remote touches from other devices (sensor-sync usermod): spawn a wave with a
  // distinct hue (offset half the palette) so remote interaction is visibly different.
  // M0 limitation: drains a single global queue — correct for one active Touch Pond segment.
#ifdef USERMOD_SENSOR_SYNC
  {
    UsermodSensorSync *ss = (UsermodSensorSync*) UsermodManager::lookup(USERMOD_ID_SENSOR_SYNC);
    if (ss) {
      RemoteSensorEvent ev;
      while (ss->popRemoteEvent(ev)) {
        if (ev.sensorType == SS_SENSOR_TOUCH && ev.value && ev.channel < MPR121::MAX_SENSORS)
          spawnTouchWave(data, ev.channel, SEGLEN, maxAge, 255, (uint8_t)(touchWaveColor(ev.channel) + 128));
      }
    }
  }
#endif

  // Update idle counter; reset and cancel replay on any new real touch
  if (newTouches) {
    data->idleFrames = 0;
    data->replayIdx = 0;
    data->replayWait = 0;
  } else {
    if (data->idleFrames < 0xFFFF) data->idleFrames++;
  }

  // Idle ghost replay: replay stored touch history as dim echo waves, preserving rhythm
  if (data->idleFrames > IDLE_THRESHOLD && data->histCount > 0) {
    if (data->replayWait > 0) {
      data->replayWait--;
    } else {
      uint8_t oldest = (data->histHead + MAX_HISTORY - data->histCount) % MAX_HISTORY;
      uint8_t idx = (oldest + data->replayIdx) % MAX_HISTORY;
      spawnTouchWave(data, data->history[idx].electrode, SEGLEN, maxAge, 80, touchWaveColor(data->history[idx].electrode));

      uint8_t nextIdx = (data->replayIdx + 1) % data->histCount;
      if (nextIdx == 0) {
        data->replayWait = REPLAY_PAUSE;
      } else {
        uint8_t nextHistIdx = (oldest + nextIdx) % MAX_HISTORY;
        uint16_t wait = (uint16_t)data->history[nextHistIdx].timeDelta * REPLAY_SCALE;
        data->replayWait = (wait > 0) ? wait : 1;
      }
      data->replayIdx = nextIdx;
    }
  }

  // Audio-reactive wave spawning: c3 controls sensitivity (sqrt curve so half = ~70% of max)
  uint8_t audioMix = SEGMENT.custom3;
  if (audioMix > 0) {
    um_data_t *um_data = nullptr;
    if (!UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE))
      um_data = simulateSound(SEGMENT.soundSim);
    if (um_data) {
      float volF = *(float*)um_data->u_data[0];
      if (volF <= 2.0f) volF *= 255.0f;
      uint8_t vol = (uint8_t)constrain((int)volF, 0, 255);

      // AGC: instant attack, slow exponential decay (~8s half-life at 30fps)
      if (vol > data->agcPeak) {
        data->agcPeak = vol;
      } else {
        data->agcPeak = max((uint8_t)1, (uint8_t)((uint16_t)data->agcPeak * 253 >> 8));
      }
      // Normalize volume relative to recent peak (0-255 = silent to loudest-seen)
      // Floor of 30 prevents ambient noise from triggering after peak decays down
      uint8_t normVol = (data->agcPeak > 30) ?
        (uint8_t)min(255u, (uint32_t)vol * 255u / data->agcPeak) : 0;

      // Piecewise curve: lower half (0-128) sqrt-compressed so 128 = "good" zone;
      // upper half (128-255) linear to preserve full control range.
      uint8_t curvedMix = (audioMix <= 128)
        ? (uint8_t)sqrt16((uint16_t)audioMix * 100u)
        : (uint8_t)(113u + (uint16_t)(audioMix - 128u) * 142u / 127u);

      // Threshold: fraction of recent peak needed to spawn; lower slider = higher bar
      uint8_t threshold = 255 - scale8(curvedMix, 165);

      if (SEGENV.aux0 > 0) {
        SEGENV.aux0--;
      } else if (normVol > threshold) {
        // Brightness scales with amplitude: louder = brighter (80-220 range)
        uint8_t audioBri = 80 + scale8(normVol, scale8(curvedMix, 140));
        uint8_t audioE = random8(MPR121::MAX_SENSORS);
        spawnTouchWave(data, audioE, SEGLEN, maxAge, audioBri, touchWaveColor(audioE));
        SEGENV.aux0 = map8(255 - curvedMix, 8, 40);
      }
    }
  }

  // Draw active waves: bright wavefront spike + comet tail; ghost waves dimmed via ghostBri
  for (int w = 0; w < MAX_TOUCH_WAVES; w++) {
    TouchWave &wave = data->waves[w];
    if (wave.maxAge == 0) continue;

    uint8_t frac = map(wave.age, 0, wave.maxAge, 0, 255);
    uint8_t frontBri = scale8(scale8(255 - frac, SEGMENT.intensity), wave.ghostBri);
    uint32_t col = SEGMENT.color_from_palette(wave.color, false, false, 255);
    uint16_t radius = (uint32_t)wave.age * maxRadius / wave.maxAge;

    for (uint16_t p = 0; p < SEGLEN; p++) {
      uint16_t dist = (p >= wave.origin) ? (p - wave.origin) : (wave.origin - p);
      if (dist > radius + 1) continue;

      uint8_t pBri;
      if (radius <= 1 || dist + 1 >= radius) {
        uint16_t frontDist = (dist >= radius) ? (dist - radius) : (radius - dist);
        pBri = (frontDist == 0) ? frontBri : scale8(frontBri, 150);
      } else {
        pBri = scale8(scale8(frontBri, 100), (uint8_t)map(dist, 0, radius, 60, 200));
      }
      SEGMENT.blendPixelColor(p, col, pBri);
    }

    wave.age++;
    if (wave.age >= wave.maxAge) wave.maxAge = 0;
  }

  // Held-electrode pulse: beatsin that makes the anchor glow rhythmically
  uint8_t beat = beatsin8(30, 80, 220);
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
  "Touch Pond@Speed,Intensity,Hz,Trail,Audio;!;!;01;sx=40,ix=220,c1=50,c2=200,c3=50";


/*
 * Touch Grid: 2D effect for square LED grids (matrix 8x8, touch-box 5x5).
 *  - Outer ring (perimeter) is touch-reactive: each touch spawns a colored dot that chases
 *    clockwise around the ring. Local touches (this device's MPR121) are bright; remote
 *    touches (other devices, via the sensor-sync bus) are dimmer.
 *  - Interior square runs a Fire2012-style fire (rising toward the top row).
 *
 * Requires a 2D matrix segment with W,H >= 3. Perimeter = 2W+2H-4 cells; interior = (W-2)x(H-2).
 * Speed -> chase speed | Fire(ix) -> spark rate | Hz(c1) -> MPR121 poll | Cool(c2) -> fire cooling
 * Tail(c3) -> chaser tail length | Palette -> chaser colors.
 */
#define TG_MAX_CHASERS 16

struct TouchChaser {
  float    pos;    // ring index (clockwise) [0, perim)
  float    speed;  // ring units per frame
  uint8_t  hue;    // palette index
  uint8_t  bri;    // brightness scale (255 local / dimmer remote)
  uint16_t life;   // frames remaining (0 = inactive)
};

struct GridFireData {
  uint16_t    prevTouched;             // local MPR121 edge state
  TouchChaser chasers[TG_MAX_CHASERS];
  uint8_t     heat[];                  // (W-2)*(H-2) interior heat (flexible array)
};

// Clockwise ring index -> (x,y) on a W x H border. r in [0, 2W+2H-4).
static void tgRingXY(int r, int W, int H, int &x, int &y) {
  const int top = W, right = H - 1, bottom = W - 1;
  if      (r < top)                  { x = r;                          y = 0; }
  else if (r < top + right)          { x = W - 1;                      y = r - top + 1; }
  else if (r < top + right + bottom) { x = W - 2 - (r - top - right);  y = H - 1; }
  else                               { x = 0;                          y = H - 1 - (r - top - right - bottom); }
}

// FastLED-style heat -> fire color (black -> red -> yellow -> white).
static uint32_t tgHeatColor(uint8_t heat) {
  uint8_t t192 = scale8(heat, 191);
  uint8_t ramp = (uint8_t)((t192 & 0x3F) << 2);
  if      (t192 & 0x80) return RGBW32(255, 255, ramp, 0);
  else if (t192 & 0x40) return RGBW32(255, ramp, 0, 0);
  else                  return RGBW32(ramp, 0, 0, 0);
}

static void tgSpawnChaser(GridFireData *d, uint8_t channel, uint8_t nch, int perim, uint8_t bri, uint8_t speedSlider) {
  int slot = -1; uint16_t oldest = 0xFFFF; int oslot = 0;
  for (int i = 0; i < TG_MAX_CHASERS; i++) {
    if (d->chasers[i].life == 0) { slot = i; break; }
    if (d->chasers[i].life <= oldest) { oldest = d->chasers[i].life; oslot = i; }
  }
  if (slot < 0) slot = oslot;
  if (nch == 0) nch = 1;
  TouchChaser &c = d->chasers[slot];
  c.pos   = (float)((uint32_t)channel * perim / nch);     // start near the touched electrode
  c.speed = 0.04f + (float)speedSlider / 255.0f * 0.40f;  // 0.04 .. 0.44 ring units / frame
  c.hue   = (uint8_t)(channel * (256 / nch));
  c.bri   = bri;
  c.life  = 1200;
}

uint16_t mode_touch_grid(void) {
  if (!SEGMENT.is2D()) return FRAMETIME;
  const int W = SEGMENT.virtualWidth();
  const int H = SEGMENT.virtualHeight();
  if (W < 3 || H < 3) return FRAMETIME;

  const int perim = 2 * W + 2 * H - 4;
  const int iW = W - 2, iH = H - 2;        // interior dims
  const int heatBytes = iW * iH;

  if (!SEGENV.allocateData(sizeof(GridFireData) + heatBytes)) return FRAMETIME;
  GridFireData *d = reinterpret_cast<GridFireData*>(SEGENV.data);
  if (SEGENV.call == 0) memset(d, 0, sizeof(GridFireData) + heatBytes);

  // --- collect touches -> spawn chasers (local bright, remote dim) ---
#ifdef USERMOD_MPR121
  UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
  if (mpr) {
    if (mpr->isSensorFound()) mpr->setUpdateHz(map8(SEGMENT.custom1, 1, 100));
    uint16_t cur = 0;
    for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++) if (mpr->touched(e)) cur |= (1u << e);
    uint16_t newTouch = cur & ~d->prevTouched;
    d->prevTouched = cur;
    for (uint8_t e = 0; e < MPR121::MAX_SENSORS; e++)
      if (newTouch & (1u << e)) tgSpawnChaser(d, e, MPR121::MAX_SENSORS, perim, 255, SEGMENT.speed);
  }
  #ifdef USERMOD_SENSOR_SYNC
  UsermodSensorSync *ss = (UsermodSensorSync*) UsermodManager::lookup(USERMOD_ID_SENSOR_SYNC);
  if (ss) {
    RemoteSensorEvent ev;
    while (ss->popRemoteEvent(ev))
      if (ev.sensorType == SS_SENSOR_TOUCH && ev.value)
        tgSpawnChaser(d, ev.channel, MPR121::MAX_SENSORS, perim, 90, SEGMENT.speed);
  }
  #endif
#endif

  // --- audio reactivity (optional): livelier sparks with volume, a bright base flash on beats.
  //     Fire still runs at the baseline rate when silent / no AudioReactive usermod. ---
  uint8_t audioSpark = 0;    // extra spark probability from loudness
  uint8_t beatTarget = 0;    // beat -> brighten the flame base toward this heat (0 = no beat)
  {
    um_data_t *um_data = nullptr;
    if (!UsermodManager::getUMData(&um_data, USERMOD_ID_AUDIOREACTIVE))
      um_data = simulateSound(SEGMENT.soundSim);  // honours the descriptor's volume flag
    if (um_data) {
      float    volumeSmth = *(float*)   um_data->u_data[0];
      uint8_t *fftResult  =  (uint8_t*) um_data->u_data[2];
      uint8_t  samplePeak = *(uint8_t*) um_data->u_data[3];
      uint8_t  vol  = (volumeSmth > 255.0f) ? 255 : (volumeSmth < 0.0f ? 0 : (uint8_t)volumeSmth);
      uint8_t  bass = fftResult ? fftResult[0] : 0;     // low-frequency energy
      audioSpark = scale8(qadd8(vol, bass >> 1), 90);   // louder/bassier -> somewhat more sparks
      if (samplePeak) beatTarget = 200 + scale8(vol, 55); // onset -> bright base flash (capped <=255)
    }
  }

  // --- interior fire (rises toward y=0; flame base = row y=iH-1) ---
  //     Tuned for short columns: strong per-cell cooling + a lossy rise hold a black->red->
  //     orange->yellow gradient instead of saturating the whole interior to white. Sparks SET
  //     the base toward a target (no qadd runaway), so only beats briefly flash it bright.
  if (iW > 0 && iH > 0) {
    uint8_t cooling  = map8(SEGMENT.custom2, 12, 60);                  // per-cell heat loss / frame
    uint8_t sparking = qadd8(map8(SEGMENT.intensity, 40, 180), audioSpark);
    for (int x = 0; x < iW; x++) {
      for (int y = 0; y < iH; y++)
        d->heat[y * iW + x] = qsub8(d->heat[y * iW + x], random8(cooling + 1));
      for (int y = 0; y < iH - 1; y++)
        d->heat[y * iW + x] = scale8(d->heat[(y + 1) * iW + x], 205); // rise one row, lose ~20%
      int base = (iH - 1) * iW + x;
      if (random8() < sparking) {
        uint8_t s = random8(150, 210);
        if (s > d->heat[base]) d->heat[base] = s;
      }
      if (beatTarget && beatTarget > d->heat[base]) d->heat[base] = beatTarget; // beat flash
    }
    for (int y = 0; y < iH; y++)
      for (int x = 0; x < iW; x++)
        SEGMENT.setPixelColorXY(1 + x, 1 + y, tgHeatColor(d->heat[y * iW + x]));
  }

  // --- perimeter chasers (clear ring, then draw heads + fading tails) ---
  for (int r = 0; r < perim; r++) { int x, y; tgRingXY(r, W, H, x, y); SEGMENT.setPixelColorXY(x, y, BLACK); }
  uint8_t tail = map8(SEGMENT.custom3, 0, 5);
  for (int i = 0; i < TG_MAX_CHASERS; i++) {
    TouchChaser &c = d->chasers[i];
    if (c.life == 0) continue;
    c.pos += c.speed;
    while (c.pos >= perim) c.pos -= perim;
    c.life--;
    uint8_t lifeBri = (c.life > 60) ? 255 : (uint8_t)map((int)c.life, 0, 60, 0, 255);
    uint8_t headBri = scale8(c.bri, lifeBri);
    uint32_t col = SEGMENT.color_from_palette(c.hue, false, false, 255);
    int head = (int)c.pos;
    int hx, hy; tgRingXY(((head % perim) + perim) % perim, W, H, hx, hy);
    SEGMENT.addPixelColorXY(hx, hy, color_fade(col, headBri, true));
    for (uint8_t t = 1; t <= tail; t++) {
      int r = ((head - (int)t) % perim + perim) % perim;
      int tx, ty; tgRingXY(r, W, H, tx, ty);
      uint8_t tb = scale8(headBri, (uint8_t)(180 - t * (140 / (tail ? tail : 1))));
      SEGMENT.addPixelColorXY(tx, ty, color_fade(col, tb, true));
    }
  }

  return FRAMETIME;
}
static const char _data_FX_MODE_TOUCH_GRID[] PROGMEM =
  "Touch Grid@Speed,Fire,Hz,Cool,Tail;;!;2v;sx=40,ix=120,c1=50,c2=128,c3=120,si=0";


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
  strip.addEffect(255, &mode_touch_grid, _data_FX_MODE_TOUCH_GRID);

  initDone = true;
}
