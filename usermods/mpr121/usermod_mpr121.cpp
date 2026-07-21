#include "wled.h"
#include "usermod_mpr121.h"

// File-scope definitions moved out of the header so the header can be included by other
// translation units (e.g. ampworks.cpp) without multiple-definition errors.
const char UsermodMPR121::_name[]    PROGMEM = "MPR121";
const char UsermodMPR121::_enabled[] PROGMEM = "enabled";

// WLED 16.x self-registration (replaces the old central usermods_list.cpp).
static UsermodMPR121 mpr121_usermod;
REGISTER_USERMOD(mpr121_usermod);
