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

check("-w" in argv and any("HTTP %{http_code}" in a for a in argv),
      "OTA keeps the -w HTTP-code token that _upload_wled() parses")
check(uw.parse_http_code("\nHTTP 200\n") == 200, "parses a 2xx status line")
check(uw.parse_http_code("\nHTTP 500\n") == 500, "parses a 5xx status line")
check(uw.parse_http_code("") == 0, "missing status line parses as 0 (not success)")


# --- SCons wiring: configure() across pre/post x serial/OTA -----------------
# The decision functions above can all be right while the wiring is wrong; this
# drives configure() the way PlatformIO does (once per registration, sharing env).
BOTH = ["pre:tools/set_build_info.py", "pre:tools/upload_wled.py",
        "post:tools/upload_wled.py"]


class RecordingEnv(FakeEnv):
    def __init__(self, upload_port, extra_scripts=BOTH):
        super().__init__(upload_port=upload_port)
        self.extra_scripts = extra_scripts
        self.replaced = []

    def GetProjectOption(self, name, default=None):
        if name == "extra_scripts":
            return list(self.extra_scripts)
        return super().GetProjectOption(name, default)

    def Replace(self, **kw):
        self.replaced.append(kw)
        self.update(kw)


SERIAL, OTA = "/dev/cu.usbserial-0001", "192.168.1.55"

e = RecordingEnv(SERIAL)
uw.configure(e, environ={})
check(e.replaced == [{"UPLOAD_PROTOCOL": "esptool"}],
      "pre+serial: switches protocol to esptool only (got {!r})".format(e.replaced))
check("UPLOADCMD" not in e, "pre+serial: UPLOADCMD untouched")
e.replaced = []
uw.configure(e, environ={})
check(e.replaced == [], "post+serial: no Replace at all (got {!r})".format(e.replaced))

e = RecordingEnv(OTA)
uw.configure(e, environ={})
check(e.replaced == [], "pre+OTA: no Replace (got {!r})".format(e.replaced))
uw.configure(e, environ={})
check(len(e.replaced) == 1 and callable(e.replaced[0].get("UPLOADCMD")),
      "post+OTA: UPLOADCMD is the HTTP uploader (got {!r})".format(e.replaced))
check(e.get("UPLOADCMD") is uw._upload_wled, "post+OTA: UPLOADCMD is _upload_wled")

# WLED_IP with a serial --upload-port: OTA wins, so no esptool switch and the
# HTTP uploader (and with it the preset remap) targets the IP.
e = RecordingEnv(SERIAL)
uw.configure(e, environ={"WLED_IP": "10.0.0.9"})
uw.configure(e, environ={"WLED_IP": "10.0.0.9"})
check(e.get("UPLOAD_PROTOCOL") is None and e.get("UPLOADCMD") is uw._upload_wled,
      "WLED_IP + serial port: OTA wiring")

# Registration guard: post-only (the led_driver_lora1ch restating pattern gone
# wrong) must fail loudly instead of the silent exit-0 no-op flash.
for scripts, label in ((["post:tools/upload_wled.py"], "post-only"),
                       (["pre:tools/upload_wled.py"], "pre-only"),
                       (["pre:tools/set_build_info.py"], "absent")):
    e = RecordingEnv(SERIAL, extra_scripts=scripts)
    try:
        uw.configure(e, environ={})
        check(False, "{} registration must exit".format(label))
    except SystemExit as ex:
        check("pre: and post:" in str(ex.code),
              "{} registration exits naming the requirement".format(label))
    check(e.replaced == [], "{} registration: nothing replaced".format(label))

# --- _upload_wled() itself: the 2xx gate on remap apply, and curl failure ------
# The wiring matrix proves _upload_wled is installed only for OTA; this drives the
# uploader with subprocess.run and run_remap stubbed, so the gate that keeps a
# remap `apply` (which reboots the device) away from a rejected flash is pinned.
import types


class UploadEnv(FakeEnv):
    def subst(self, s):
        if s == "$BUILD_DIR":
            return self["BUILD_DIR"]
        return super().subst(s)


def drive_upload(curl_rc, curl_stdout, snapshot_rc=0, environ=None):
    """Run _upload_wled() against an OTA FakeEnv. Returns (remap_calls, curl_argvs,
    exit_code) where exit_code is None unless it raised SystemExit."""
    remap_calls, curl_argvs = [], []

    def fake_run(argv, **kw):
        curl_argvs.append(list(argv))
        return types.SimpleNamespace(returncode=curl_rc, stdout=curl_stdout)

    def fake_remap(env, args, ok=(0,)):
        remap_calls.append(list(args))
        return snapshot_rc if args[0] == "snapshot" else 0

    saved_run, saved_remap = uw.subprocess.run, uw.run_remap
    keys = ("WLED_IP", "WLED_NO_PRESET_REMAP", "WLED_REMAP_TIMEOUT")
    saved_env = {k: os.environ.get(k) for k in keys}
    for k in keys:
        os.environ.pop(k, None)
    os.environ.update(environ or {})
    uw.subprocess.run, uw.run_remap = fake_run, fake_remap
    env = UploadEnv(upload_port=OTA)
    env["BUILD_DIR"] = "/tmp/build"
    exit_code = None
    try:
        uw._upload_wled(["/tmp/firmware.bin"], None, env)
    except SystemExit as ex:
        exit_code = ex.code
    finally:
        uw.subprocess.run, uw.run_remap = saved_run, saved_remap
        for k, v in saved_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    return remap_calls, curl_argvs, exit_code


def steps(calls):
    return [c[0] for c in calls]


calls, argvs, rc = drive_upload(0, "\nHTTP 200\n")
check(steps(calls) == ["snapshot", "apply"],
      "HTTP 200: snapshot then apply (got {!r})".format(steps(calls)))
check(rc is None, "HTTP 200: no exit")
check(argvs == [uw.build_curl_argv("/tmp/firmware.bin", OTA)],
      "HTTP 200: curl ran once with the build_curl_argv() argv")
if len(calls) == 2:
    check(calls[1][1] == OTA and "--wait-since" in calls[1] and "--reboot" in calls[1],
          "HTTP 200: apply targets the host, waits for the reboot, reboots once")
    check(calls[0][2] == calls[1][2] == os.path.join("/tmp/build", "preset_remap", OTA),
          "snapshot and apply share the per-host snapdir")

for status in ("HTTP 500", "HTTP 404", ""):
    calls, argvs, rc = drive_upload(0, "\n{}\n".format(status))
    check(steps(calls) == ["snapshot"],
          "{!r}: snapshot only, NO apply (got {!r})".format(status, steps(calls)))
    check(rc is None, "{!r}: curl rc 0 does not exit".format(status))

calls, argvs, rc = drive_upload(1, "")
check(rc == 1, "curl rc 1 -> exit 1 (got {!r})".format(rc))
check(steps(calls) == ["snapshot"], "curl rc 1: no apply (got {!r})".format(steps(calls)))
calls, argvs, rc = drive_upload(28, "\nHTTP 200\n")   # timeout, even with a stray 200 line
check(rc == 1, "curl rc 28 -> exit 1 even if stdout mentions 200 (got {!r})".format(rc))
check("apply" not in steps(calls), "curl rc 28: no apply")

# Snapshot outcomes: a kept earlier snapshot (rc 3) still proceeds to apply; a failed
# snapshot skips the remap entirely but the upload still runs.
calls, argvs, rc = drive_upload(0, "\nHTTP 200\n", snapshot_rc=uw.SNAPSHOT_KEPT)
check(steps(calls) == ["snapshot", "apply"],
      "kept snapshot (rc 3): apply still runs (got {!r})".format(steps(calls)))
calls, argvs, rc = drive_upload(0, "\nHTTP 200\n", snapshot_rc=2)
check(steps(calls) == ["snapshot"], "failed snapshot: no apply (got {!r})".format(steps(calls)))
check(len(argvs) == 1, "failed snapshot: upload still runs")

# Opt-out: no remap calls at all.
calls, argvs, rc = drive_upload(0, "\nHTTP 200\n", environ={"WLED_NO_PRESET_REMAP": "1"})
check(calls == [] and len(argvs) == 1, "WLED_NO_PRESET_REMAP=1: upload, no remap")


if failures:
    print("\n{} test(s) failed".format(len(failures)))
    sys.exit(1)
print("ALL TESTS PASSED")
