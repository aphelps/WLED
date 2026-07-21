#pragma once

#include "wled.h"
#include <Wire.h>
#include <MPR121.h>

/*
 * MPR121 library: https://github.com/AMPWorks/ArduinoLibs/tree/master/MPR121
 *
 * UsermodMPR121 — standalone usermod for the MPR121 12-channel capacitive touch sensor.
 *
 * Exposes sensor state via inline wrapper methods over the MPR121 library accessors.
 * Effects and other usermods access touch data by looking up this usermod directly:
 *
 *   #ifdef USERMOD_MPR121
 *   UsermodMPR121 *mpr = (UsermodMPR121*) UsermodManager::lookup(USERMOD_ID_MPR121);
 *   if (mpr && mpr->touched(3)) { ... }
 *   uint16_t prox = mpr ? mpr->getFiltered(MPR121::PROX_SENSOR) : 0;
 *   #endif
 *
 * The usermod's loop() handles all I2C reads at ~20 Hz. Callers just invoke the
 * wrappers — the cached library state is always up to date.
 *
 * Config (persisted in cfg.json):
 *   enabled        — whether to initialize/run the sensor (default true)
 *   i2cAddress     — I2C address in decimal (default 90 = 0x5A)
 *   touchThresh    — touch threshold per electrode (default 10 = 0x0A)
 *   releaseThresh  — release threshold per electrode (default 15 = 0x0F)
 *   irqPin         — GPIO for interrupt pin; -1 = polling mode (default -1)
 *   updateHz       — sensor poll/read rate in Hz, 1–100 (default 20)
 */
class UsermodMPR121 : public Usermod {
 public:
  static const char _name[];
  static const char _enabled[];

  void setup() override {
    if (enabled) initSensor();
    initDone = true;
  }

  void loop() override {
    if (!enabled || !sensorFound || !mpr121 || strip.isUpdating()) return;
    if (millis() - lastTime < (1000u / updateHz)) return;
    lastTime = millis();

    // If no IRQ pin, set triggered manually so readTouchInputs() polls via I2C
    if (irqPin < 0) mpr121->triggered = true;

    mpr121->readTouchInputs();  // reads I2C only when triggered; clears triggered
    mpr121->getFilteredAll();   // refreshes filteredData cache inside library
  }

  // Wrappers over MPR121 library accessors — safe to call even when sensor not found
  inline bool touched(uint8_t electrode) {
    return sensorFound && mpr121 && mpr121->touched(electrode);
  }
  inline bool changed(uint8_t electrode) {
    return sensorFound && mpr121 && mpr121->changed(electrode);
  }
  inline uint16_t getFiltered(uint8_t electrode) {
    return (sensorFound && mpr121) ? mpr121->getFiltered(electrode) : 0;
  }
  inline bool isSensorFound() { return sensorFound; }
  inline void setUpdateHz(uint8_t hz) { updateHz = constrain(hz, 1, 100); }

  uint16_t getId() override { return USERMOD_ID_MPR121; }

  void addToJsonInfo(JsonObject &root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray arr = user.createNestedArray("MPR121");
    if (!sensorFound) {
      arr.add("not found");
    } else {
      uint16_t states = 0;
      for (uint8_t i = 0; i < MPR121::TOTAL_SENSORS; i++) {
        if (touched(i)) states |= (1 << i);
      }
      arr.add(states);
      arr.add(" touched");
    }
  }

  void addToConfig(JsonObject &root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)]  = enabled;
    top["i2cAddress"]     = i2cAddress;
    top["touchThresh"]    = touchThresh;
    top["releaseThresh"]  = releaseThresh;
    top["irqPin"]         = irqPin;
    top["updateHz"]       = updateHz;
  }

  bool readFromConfig(JsonObject &root) override {
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) return false;

    bool en_prev = enabled;
    getJsonValue(top[FPSTR(_enabled)], enabled);
    getJsonValue(top["i2cAddress"],    i2cAddress);
    getJsonValue(top["touchThresh"],   touchThresh);
    getJsonValue(top["releaseThresh"], releaseThresh);
    getJsonValue(top["irqPin"],        irqPin);
    uint8_t hz = updateHz;
    if (getJsonValue(top["updateHz"], hz)) updateHz = constrain(hz, 1, 100);

    if (initDone && enabled != en_prev) {
      if (enabled) initSensor();
      else         sensorFound = false;
    }
    return true;
  }

 private:
  bool       initDone      = false;
  bool       enabled       = true;
  bool       sensorFound   = false;
  uint8_t    i2cAddress    = 0x5A;
  uint8_t    touchThresh   = 0x0A;
  uint8_t    releaseThresh = 0x0F;
#ifndef MPR121_IRQ_PIN
  int8_t     irqPin        = -1;
#else
  int8_t     irqPin        = MPR121_IRQ_PIN;
#endif
  uint8_t    updateHz      = 20;  // poll rate 1–100 Hz

  MPR121    *mpr121        = nullptr;
  uint32_t   lastTime      = 0;

  void initSensor() {
    sensorFound = false;
    if (i2c_scl < 0 || i2c_sda < 0) {
      DEBUG_PRINTLN(F("MPR121: I2C pins not configured"));
      return;
    }

    // Probe I2C address — if nothing ACKs, skip init
    Wire.beginTransmission((uint8_t)i2cAddress);
    if (Wire.endTransmission() != 0) {
      DEBUG_PRINTF("MPR121: no device found at I2C address 0x%02X\n", i2cAddress);
      return;
    }

    bool useIRQ = (irqPin >= 0);
    mpr121 = new MPR121((byte)(useIRQ ? irqPin : 0), useIRQ,
                        (byte)i2cAddress, /*times=*/false, /*filtered=*/true, /*auto_enabled=*/false);
    if (!mpr121) {
      DEBUG_PRINTLN(F("MPR121: failed to allocate sensor object"));
      return;
    }
    mpr121->setThresholds((byte)touchThresh, (byte)releaseThresh);
    sensorFound = true;
    DEBUG_PRINTF("MPR121: initialized at 0x%02X, IRQ pin %d\n", i2cAddress, irqPin);
  }
};

const char UsermodMPR121::_name[]    PROGMEM = "MPR121";
const char UsermodMPR121::_enabled[] PROGMEM = "enabled";
