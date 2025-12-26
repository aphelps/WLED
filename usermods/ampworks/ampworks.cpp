#include "ampworks.h"

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

struct Moving_Sin {
  uint16_t center;
  uint8_t width;
};

uint16_t mode_amp_moving_sin(void) {
  if (!SEGENV.allocateData(sizeof(struct Moving_Sin))) return FRAMETIME;

  struct Moving_Sin *data = (struct Moving_Sin*)SEGENV.data;
  if (SEGENV.call == 0) {
    /* Initialization */
    data->center = random16(SEGMENT.width());
    SEGMENT.fill(0);
  }

  /* Clear the trailing pixels */
  uint16_t trailing;
  if (data->center < data->width) trailing = SEGMENT.width() - (data->width - data->center);
  else trailing = data->center - data->width;
  for (uint16_t offset = 0; offset < speed; offset++) {
    SEGMENT.setPixelColor((trailing + offset) % SEGMENT.width(), 0);
  }

  uint8_t speed = map8(SEGMENT.speed, 1, 10);
  data->width = map8(SEGMENT.intensity, 1, 10);

  data->center = (data->center + speed) % SEGMENT.width();
  if (data->width >= data->center) trailing = 0;
  else trailing = data->center - data->width;

  uint16_t lead = data->center + data->width;
  if ((lead >= SEGMENT.width()) || (lead < data->center))
    lead = SEGMENT.width() - 1;

  for (uint16_t i = trailing; i <= lead ; i++) {
    SEGMENT.setPixelColor(i, SEGCOLOR(0));
  }

  return FRAMETIME;
}
// PROGMEM description for audio mode
static const char _data_FX_MODE_AMP_MOVING_SIN[] PROGMEM = "AMP Moving SIN@Speed,Width;!,!;!;01";


// add more strings here to reduce flash memory usage
const char AMPWorks::_name[]    PROGMEM = "AMPWorks";
const char AMPWorks::_enabled[] PROGMEM = "enabled";

void AMPWorks::setup() {
  //Serial.println("Hello from my usermod!");

  // strip.addEffect(255, &mode_amp_test, _data_FX_MODE_AMP_TEST);
  strip.addEffect(255, &mode_amp_ai, _data_FX_MODE_AMP_AI); // register AMP AI mode
  strip.addEffect(255, &mode_amp_ai_audio, _data_FX_MODE_AMP_AI_AUDIO); // register AMP AI audio mode
  strip.addEffect(255, &mode_amp_moving_sin, _data_FX_MODE_AMP_MOVING_SIN);

  initDone = true;
}
