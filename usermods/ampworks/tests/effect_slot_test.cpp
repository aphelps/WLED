// Host unit test for fx_slot.h: pinning custom effects to fixed effect ids through WLED's
// stock addEffect().
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure header:
//   c++ -std=c++11 -Wall -Wextra -o /tmp/effect_slot_test effect_slot_test.cpp && /tmp/effect_slot_test
// Exits 0 on success, 1 on any failed assertion.
//
// FakeStrip::addEffect / setupEffectData replicate stock WLED (wled00/FX.cpp at origin/main
// eb14ddb7, WS2812FX::addEffect and the gap-filling head of setupEffectData) line for line,
// with std::vector in place of the strip's vectors. If upstream changes addEffect, update
// this copy and re-run: the test is only as good as the replica.
#include "../fx_slot.h"
#include <cstdio>
#include <cstring>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

typedef void (*mode_ptr)();
static void mode_static() {}
static void mode_builtin() {}
static void mode_custom() {}
static void mode_filler() {}
static void mode_other_usermod() {}

static const char _data_RESERVED[] = "RSVD";   // FX.cpp's own reserved marker
static const char _data_BUILTIN[]  = "Builtin";
static const char _data_STATIC[]   = "Solid";
static const char _data_CUSTOM[]   = "Custom@!;!;!;01";
static const char _data_OTHER[]    = "Other usermod";

static const uint8_t UPSTREAM_MODE_COUNT = 220;  // MODE_COUNT at eb14ddb7

struct FakeStrip {
  std::vector<mode_ptr> _mode;
  std::vector<const char*> _modeData;
  uint8_t _modeCount;

  explicit FakeStrip(uint8_t modeCount) : _modeCount(modeCount) {}

  uint8_t getModeCount() const { return _modeCount; }
  const char *getModeData(unsigned id = 0) const { return (id && id < _modeCount) ? _modeData[id] : "Solid"; }

  // ---- verbatim logic of stock WS2812FX::addEffect ----
  uint8_t addEffect(uint8_t id, mode_ptr mode_fn, const char *mode_name) {
    if (id == 255) { // find empty slot
      for (size_t i=1; i<_mode.size(); i++) if (_modeData[i] == _data_RESERVED) { id = i; break; }
    }
    if (id < _mode.size()) {
      if (_modeData[id] != _data_RESERVED) return 255; // do not overwrite an already added effect
      _mode[id]     = mode_fn;
      _modeData[id] = mode_name;
      return id;
    } else if (_mode.size() < 255) { // 255 is reserved for indicating the effect wasn't added
      _mode.push_back(mode_fn);
      _modeData.push_back(mode_name);
      if (_modeCount < _mode.size()) _modeCount++;
      return _mode.size() - 1;
    } else {
      return 255; // The vector is full so return 255
    }
  }

  // ---- head of stock setupEffectData, then the built-ins filled ----
  // `gap` (0 = none) leaves one built-in id as _data_RESERVED, like the #ifdef-guarded
  // addEffect calls in stock setupEffectData (WLED_DISABLE_2D, WLED_ENABLE_GIF,
  // WLED_PS_DONT_REPLACE_*_FX, ...) do in a real build.
  void setupEffectData(uint8_t gap = 0) {
    _mode.push_back(&mode_static);
    _modeData.push_back(_data_STATIC);
    for (size_t i=1; i<_modeCount; i++) {
      _mode.push_back(&mode_static);
      _modeData.push_back(_data_RESERVED);
    }
    for (uint8_t i = 1; i < _modeCount; i++) if (i != gap) addEffect(i, &mode_builtin, _data_BUILTIN);
  }
};

static bool startsRSVD(const char *s) { return strncmp(s, "RSVD", 4) == 0; }

// Pin past the end: pads with RSVD-named filler and lands exactly at id.
static void test_pin_past_end() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  CHECK(s.getModeCount() == 220 && s._mode.size() == 220, "stock setup leaves 220 effects");
  uint8_t got = pinEffect(s, 240, &mode_custom, _data_CUSTOM, &mode_filler);
  CHECK(got == 240, "pin 240 lands at 240");
  CHECK(s.getModeCount() == 241, "getModeCount()==id+1 after pin");
  CHECK(s._mode.size() == 241, "vector size matches getModeCount");
  CHECK(s._modeData[240] == _data_CUSTOM && s._mode[240] == &mode_custom, "effect stored at 240");
  for (unsigned i = 220; i < 240; i++) {
    CHECK(startsRSVD(s.getModeData(i)), "padded index name begins RSVD");
    CHECK(s._modeData[i] != _data_RESERVED, "padding is not FX.cpp's _data_RESERVED pointer");
    CHECK(s._mode[i] == &mode_filler, "padding uses the filler effect");
  }
  for (unsigned i = 1; i < 220; i++) CHECK(s._modeData[i] == _data_BUILTIN, "built-ins untouched");
}

// Pinning the whole AMPWorks block 240..245 ascending: every one lands. Run both on a
// gap-free built-in table and on one with a stock _data_RESERVED gap below 220 (gap != 0).
static void test_pin_block_ascending(uint8_t gap) {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData(gap);
  for (uint8_t id = 240; id <= 245; id++) {
    uint8_t got = pinEffect(s, id, &mode_custom, _data_CUSTOM, &mode_filler);
    CHECK(got == id, "block pin lands at its id");
  }
  CHECK(s.getModeCount() == 246, "fxcount 246 after the block");
  CHECK(s._mode.size() <= 255, "size stays <= 255");

  // A later addEffect(255) from another usermod never lands in our 220..245 block (padding or
  // pins). Where it does land is build-dependent: the first stock _data_RESERVED gap if the
  // build has one, else appended at 246 — so assert the build-agnostic property, plus the
  // exact slot only for the two fixtures whose layout we control.
  uint8_t other = s.addEffect(255, &mode_other_usermod, _data_OTHER);
  CHECK(other != 255, "addEffect(255) still finds a slot");
  CHECK(other < 220 || other > 245, "addEffect(255) does not land in the pinned block or padding");
  CHECK(other == (gap ? gap : 246), "fixture: fills the stock gap, else appends at 246");
  for (unsigned i = 220; i < 240; i++) CHECK(s._modeData[i] == _data_AMP_RESERVED, "padding still intact");
}

// Pinning an id that is already occupied (e.g. upstream grew its table past it) → 255.
static void test_pin_occupied() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  uint8_t got = pinEffect(s, 10, &mode_custom, _data_CUSTOM, &mode_filler);
  CHECK(got == 255, "occupied built-in id refuses");
  CHECK(s._modeData[10] == _data_BUILTIN, "built-in not overwritten");
  CHECK(s.getModeCount() == 220, "nothing added on refusal");

  // Pinning the same id twice: second one refuses.
  CHECK(pinEffect(s, 240, &mode_custom, _data_CUSTOM, &mode_filler) == 240, "first pin ok");
  CHECK(pinEffect(s, 240, &mode_other_usermod, _data_OTHER, &mode_filler) == 255, "re-pin refuses");
  CHECK(s._modeData[240] == _data_CUSTOM, "first pin kept");
}

// Upstream grew past our id: size already > id and the slot is a real built-in → 255.
static void test_upstream_grew_past_pin() {
  FakeStrip s(250); s.setupEffectData();
  CHECK(s.getModeCount() == 250, "future upstream with 250 built-ins");
  uint8_t got = pinEffect(s, 243, &mode_custom, _data_CUSTOM, &mode_filler);
  CHECK(got == 255, "pin below upstream size refuses");
  CHECK(s.getModeCount() == 250, "no growth on refusal");
}

// Pinning exactly at the end (no padding needed) lands without adding filler.
static void test_pin_at_end() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  CHECK(pinEffect(s, 220, &mode_custom, _data_CUSTOM, &mode_filler) == 220, "pin at end");
  CHECK(s.getModeCount() == 221, "one entry added");
}

// Another usermod appended before us (addEffect(255) at 220): padding starts after it.
static void test_other_usermod_first() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  CHECK(s.addEffect(255, &mode_other_usermod, _data_OTHER) == 220, "other usermod at 220");
  CHECK(pinEffect(s, 240, &mode_custom, _data_CUSTOM, &mode_filler) == 240, "pin still lands");
  CHECK(s._modeData[220] == _data_OTHER, "other usermod kept");
  for (unsigned i = 221; i < 240; i++) CHECK(startsRSVD(s.getModeData(i)), "padding after it");
}

// 255 is "next free" to addEffect, never a pinnable id.
static void test_pin_255_refused() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  CHECK(pinEffect(s, 255, &mode_custom, _data_CUSTOM, &mode_filler) == 255, "id 255 refused");
  CHECK(s.getModeCount() == 220, "nothing added for 255");
}

// Highest legal id: list can hold 255 entries (ids 0..254).
static void test_pin_254_fills_list() {
  FakeStrip s(UPSTREAM_MODE_COUNT); s.setupEffectData();
  CHECK(pinEffect(s, 254, &mode_custom, _data_CUSTOM, &mode_filler) == 254, "pin 254");
  CHECK(s._mode.size() == 255, "list full at 255 entries");
  CHECK(s.addEffect(255, &mode_other_usermod, _data_OTHER) == 255, "full list refuses more");
}

// WS2812FX's constructor sets _modeCount=1 and skips setupEffectData when allocation fails.
// pinEffect must give up rather than spin forever.
static void test_alloc_failure_terminates() {
  FakeStrip s(1);   // no setupEffectData(): empty vectors, _modeCount 1
  uint8_t got = pinEffect(s, 240, &mode_custom, _data_CUSTOM, &mode_filler);
  CHECK(got == 255, "alloc-failure strip refuses without looping");
}

int main() {
  test_pin_past_end();
  test_pin_block_ascending(0);
  test_pin_block_ascending(120);  // e.g. a WLED_DISABLE_2D-style hole among the built-ins
  test_pin_occupied();
  test_upstream_grew_past_pin();
  test_pin_at_end();
  test_other_usermod_first();
  test_pin_255_refused();
  test_pin_254_fills_list();
  test_alloc_failure_terminates();
  if (g_fail) { printf("effect_slot_test: FAILED\n"); return 1; }
  printf("effect_slot_test: all passed\n");
  return 0;
}
