#pragma once

/*
 * AMPWorks WLED usermod.
 *
 * Registers custom LED effects with the WLED effect engine. Effects are
 * implemented in ampworks.cpp and added to the strip in setup().
 *
 * Enabled via -D USERMOD_AMPWORKS in the ampworks PlatformIO environment.
 * Effects that use the MPR121 touch sensor also require -D USERMOD_MPR121.
 */

#include "wled.h"

class AMPWorks : public Usermod {

  private:
    bool initDone = false;
    static const char _name[];
    static const char _enabled[];

  public:
    void setup() override;

    void loop() override {}
    void connected() override {}

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
    }

    void addToConfig(JsonObject& root) override {
      root.createNestedObject(FPSTR(_name));
    }

    bool readFromConfig(JsonObject& root) override {
      return !root[FPSTR(_name)].isNull();
    }

    uint16_t getId() override { return USERMOD_ID_EXAMPLE; }
};
