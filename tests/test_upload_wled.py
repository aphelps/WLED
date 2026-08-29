#!/usr/bin/env python3
"""Host tests for tools/upload_wled.py -- the serial-vs-OTA upload decision.

Run: python3 tests/test_upload_wled.py   (registered as `make test-upload`)

These assert the DECISION and the constructed argv. There is no device and no
SCons here; the module is importable because its Import("env") is guarded.
"""
import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location(
    "upload_wled", os.path.join(ROOT, "tools", "upload_wled.py"))
uw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(uw)

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL: {}".format(what))


class FakeEnv(dict):
    """Minimal stand-in: UPLOAD_PORT is the CLI-injected clivar, GetProjectOption
    is the platformio.ini value. The two deliberately disagree in one test."""

    def __init__(self, upload_port="", project_option="192.168.1.55"):
        super().__init__()
        self["UPLOAD_PORT"] = upload_port
        self._project_option = project_option

    def subst(self, s):
        if s == "$UPLOAD_PORT":
            return self.get("UPLOAD_PORT", "")
        return s

    def GetProjectOption(self, name, default=None):
        if name == "upload_port":
            return self._project_option
        return default


# --- classification ---------------------------------------------------------
for t in ("/dev/cu.usbserial-0001", "/dev/ttyUSB0", "/dev/ttyACM0",
          "/dev/serial/by-id/usb-Silicon_Labs", "COM7", "com7", "COM10",
          "\\\\.\\COM10",
          "/dev/cu.SET-UPLOAD-PORT"):
    check(uw.is_serial_target(t), "serial: {!r}".format(t))

for t in ("192.168.1.55", "wled-matrix.local", "wled-matrix", "", None, "   "):
    check(not uw.is_serial_target(t), "OTA: {!r}".format(t))

# The led_driver_lora1ch sentinel must stay serial so that env keeps failing fast
# rather than silently OTA-ing to a placeholder.
check(uw.is_serial_target("/dev/cu.SET-UPLOAD-PORT"),
      "led_driver_lora1ch sentinel classifies as serial")


# --- CLI precedence: the actual root cause ----------------------------------
# platformio.ini says 192.168.1.55; the CLI passed --upload-port /dev/...
# The old code read GetProjectOption and so always chose OTA. This test fails
# against that implementation, which is the point of having it.
env = FakeEnv(upload_port="/dev/cu.usbserial-0001", project_option="192.168.1.55")
resolved = uw.resolve_target(env, environ={})
check(resolved == "/dev/cu.usbserial-0001",
      "CLI --upload-port wins over platformio.ini (got {!r})".format(resolved))
check(uw.is_serial_target(resolved), "CLI-supplied device path classifies as serial")

# With no CLI value, fall back to the project option (OTA default preserved).
env2 = FakeEnv(upload_port="", project_option="192.168.1.55")
check(uw.resolve_target(env2, environ={}) == "192.168.1.55",
      "falls back to platformio.ini upload_port when no CLI value")

# WLED_IP forces OTA even when a serial port is present.
env3 = FakeEnv(upload_port="/dev/cu.usbserial-0001")
check(uw.resolve_target(env3, environ={"WLED_IP": "10.0.0.9"}) == "10.0.0.9",
      "WLED_IP overrides and forces OTA")


# --- OTA argv unchanged -----------------------------------------------------
argv = uw.build_curl_argv("/tmp/firmware.bin", "192.168.1.55")
check(argv[0] == "curl", "OTA still uses curl")
check("http://192.168.1.55/update" in argv, "OTA posts to the /update endpoint")
check("update=@/tmp/firmware.bin" in argv, "OTA attaches the firmware")
check("--max-time" in argv and "60" in argv, "OTA keeps its 60s cap")
check("--connect-timeout" in argv,
      "OTA fails fast on an unreachable host instead of waiting the full cap")

if failures:
    print("\n{} test(s) failed".format(len(failures)))
    sys.exit(1)
print("ALL TESTS PASSED")
