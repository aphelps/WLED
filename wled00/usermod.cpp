#include "wled.h"
/*
 * This v1 usermod file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * If you just need 8 bytes, use 2551-2559 (you do not need to increase EEPSIZE)
 *
 * Consider the v2 usermod API if you need a more advanced feature set!
 */

//Use userVar0 and userVar1 (API calls &U0=,&U1=, uint16_t)

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
static const char _data_FX_MODE_AMP_TEST[] PROGMEM = "AMP Test Usermod@!;!,!;!;01";


//gets called once at boot. Do all initialization that doesn't depend on network here
void userSetup()
{
  WS2812FX *fx = WS2812FX::getInstance();

  fx->addEffect(255, &mode_amp_test, _data_FX_MODE_AMP_TEST);

}

//gets called every time WiFi is (re-)connected. Initialize own network interfaces here
void userConnected()
{

}

//loop. You can use "if (WLED_CONNECTED)" to check for successful connection
void userLoop()
{

}
