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

  // cycle time scales with speed (higher SEGMENT.speed -> faster shifting)
  uint32_t cycleTime = 600 + (255 - SEGMENT.speed) * 8; // tuned constant
  uint32_t it = strip.now / cycleTime;
  if (it != SEGENV.step) {
    SEGENV.aux0 = (SEGENV.aux0 + 1) % 3; // shift pattern 0..2
    SEGENV.step = it;
  }

  // set every third pixel (with shifting offset) to red, clear others
  for (uint16_t i = 0; i < SEGLEN; i++) {
    if (((i + SEGENV.aux0) % 3) == 0) {
      SEGMENT.setPixelColor(i, RGBW32(255, 0, 0, 0));
    } else {
      SEGMENT.setPixelColor(i, 0);
    }
  }

  return FRAMETIME;
}
static const char _data_FX_MODE_AMP_AI[] PROGMEM = "AMP AI v0.1@!;!,!;!;01";

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


// add more strings here to reduce flash memory usage
const char AMPWorks::_name[]    PROGMEM = "AMPWorks";
const char AMPWorks::_enabled[] PROGMEM = "enabled";

void AMPWorks::setup() {
  //Serial.println("Hello from my usermod!");

  strip.addEffect(255, &mode_amp_test, _data_FX_MODE_AMP_TEST);
  strip.addEffect(255, &mode_amp_ai, _data_FX_MODE_AMP_AI); // register AMP AI mode
  strip.addEffect(255, &mode_amp_ai_audio, _data_FX_MODE_AMP_AI_AUDIO); // register AMP AI audio mode

  initDone = true;
}
