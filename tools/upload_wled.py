"""
PlatformIO custom upload script: flash WLED over HTTP (the /update endpoint) OR over
USB serial, chosen from the resolved upload target.

Uses curl (not Python urllib) for the HTTP path, to work around macOS Sequoia
local-network permission restrictions that block UDP (espota) and Python's urllib
but not system binaries.

The OTA target is upload_port from platformio.ini, overridden by --upload-port on the
command line, or forced via the WLED_IP environment variable:

    WLED_IP=192.168.1.99 pio run -e ampworks -t upload            # force OTA
    pio run -e ampworks -t upload --upload-port /dev/cu.usbserial-0001   # serial

WHY THIS FILE IS REGISTERED AS BOTH pre: AND post:
------------------------------------------------
PlatformIO runs pre: scripts, then the platform build script, then post: scripts.

  * The SERIAL decision must happen in the PRE pass. The espressif32 builder reads
    $UPLOAD_PROTOCOL and, for "esptool", assembles the whole upload command --
    correct --chip, $ESP32_APP_OFFSET, FLASH_EXTRA_IMAGES for the bootloader /
    partitions / boot_app0, and a BeforeUpload port-autodetect action. Setting the
    protocol after that has run is too late.

  * The OTA replacement must happen in the POST pass. The platform builder forces
    upload_protocol to "espota" whenever UPLOAD_PORT looks like an IP or *.local,
    and that branch Replace()s UPLOADCMD with espota.py. Replacing UPLOADCMD before
    the builder runs would simply be clobbered.

Doing only one of the two does not work. In particular, leaving UPLOADCMD unset
under `upload_protocol = custom` does NOT fall back to esptool: $UPLOADCMD
substitutes to empty, the upload target runs an empty action and EXITS 0 --
a silent no-op "flash" on a board that was never written.

Preset remap (OTA only): a firmware update can renumber effects (presets store the
numeric fx id). Around an HTTP flash this script runs tools/remap_presets.py: snapshot
/json/eff + /presets.json before uploading, then after a successful (2xx) upload wait
for the device to come back, remap presets by effect name, and reboot once if anything
changed. The snapshot is kept under $BUILD_DIR/preset_remap/<host>; an earlier snapshot
that was never applied (e.g. the post-flash wait timed out) is kept and used instead.
Opt out with WLED_NO_PRESET_REMAP=1. The post-flash readiness wait defaults to 60 s;
override with WLED_REMAP_TIMEOUT=<seconds>. A remap problem only warns: the firmware is
already flashed, so it never fails the upload. The remap lives entirely inside
_upload_wled(), which the post pass installs only for an OTA target, so a serial flash never
touches it.
"""
import os
import re
import subprocess
import sys
import time

# Windows names ports above COM9 as \\.\COM10; a bare COM\d+ rule misses those.
_COM_RE = re.compile(r"^COM\d+$", re.IGNORECASE)

DEFAULT_OTA_TARGET = "192.168.1.55"

SNAPSHOT_KEPT = 3  # remap_presets.py snapshot: kept an earlier un-applied snapshot


def is_serial_target(target):
    """True if `target` names a serial port rather than a network host.

    Serial: POSIX device paths (/dev/cu.*, /dev/ttyUSB*, /dev/serial/by-id/*),
    COM1..COM9, and the \\.\COMnn form Windows requires above COM9.
    Everything else -- IPs, *.local, bare hostnames, empty -- is treated as OTA.

    Ambiguity resolves toward SERIAL on purpose: a hostname misread as serial
    fails immediately and legibly in esptool, whereas a device path misread as
    OTA burns the full curl timeout, which is the bug this file exists to fix.
    """
    if not target:
        return False
    t = str(target).strip()
    if not t:
        return False
    if t.startswith("/dev/"):
        return True
    if t.startswith("\\\\.\\"):          # \\.\COM10
        return True
    return bool(_COM_RE.match(t))


def resolve_target(env=None, environ=None):
    """Resolve the upload target, honouring the CLI.

    WLED_IP wins and always means OTA.

    Otherwise read $UPLOAD_PORT from the SCons environment -- NOT
    GetProjectOption("upload_port"). GetProjectOption reads platformio.ini via
    ProjectConfig and is blind to the command line; `--upload-port` arrives as the
    SCons clivar UPLOAD_PORT and lands only in env. Reading the project option is
    why `--upload-port /dev/cu.usbserial-0001` was previously ignored.
    """
    environ = os.environ if environ is None else environ
    forced = environ.get("WLED_IP")
    if forced:
        return forced
    if env is not None:
        target = ""
        try:
            target = env.subst("$UPLOAD_PORT") or ""
        except Exception:
            target = ""
        if not target:
            target = env.get("UPLOAD_PORT", "") or ""
        if target:
            return str(target)
        try:
            return str(env.GetProjectOption("upload_port", DEFAULT_OTA_TARGET))
        except Exception:
            pass
    return DEFAULT_OTA_TARGET


def build_curl_argv(firmware, host):
    """The OTA command. Split out so a test can assert the argv without a device.

    The -w "HTTP %{http_code}" token is load-bearing: curl exits 0 on an HTTP 4xx/5xx,
    so _upload_wled() parses that line to tell a real flash from a rejected one.
    """
    return [
        "curl", "-X", "POST", "http://{}/update".format(host),
        "-F", "update=@{}".format(firmware),
        "--progress-bar",
        "--max-time", "60",
        "--connect-timeout", "5",
        "-w", "\nHTTP %{http_code}\n",
        "-o", "/dev/null",
    ]


def parse_http_code(stdout):
    """The status from build_curl_argv()'s -w line, or 0 if absent."""
    m = re.search(r"HTTP (\d{3})", stdout or "")
    return int(m.group(1)) if m else 0


def run_remap(env, args, ok=(0,)):
    """Run tools/remap_presets.py with args; warn (never exit) on failure. Returns the exit
    code, or None if it could not run."""
    script = os.path.join(env["PROJECT_DIR"], "tools", "remap_presets.py")
    cmd = [sys.executable or "python3", script] + args
    sys.stdout.flush()  # keep our lines ordered with the child's output when piped
    try:
        rc = subprocess.run(cmd).returncode
    except OSError as e:
        print(f"WARNING: preset remap could not run ({e})")
        return None
    if rc not in ok:
        print(f"WARNING: preset remap step '{args[0]}' exited {rc} (firmware upload unaffected)")
    return rc


def _upload_wled(source, target, env):
    """OTA upload (the post pass installs this only for a non-serial target)."""
    firmware = str(source[0])
    host = resolve_target(env)
    url = f"http://{host}/update"

    remap = os.environ.get("WLED_NO_PRESET_REMAP", "") not in ("1", "true", "yes")
    snapdir = os.path.join(env.subst("$BUILD_DIR"), "preset_remap", host)
    if remap:
        print(f"\nSnapshotting effects + presets from {host} for the post-flash preset remap ...")
        if run_remap(env, ["snapshot", host, snapdir], ok=(0, SNAPSHOT_KEPT)) not in (0, SNAPSHOT_KEPT):
            print("WARNING: preset snapshot failed; preset remap skipped, continuing with upload")
            remap = False

    print(f"\nUploading {firmware} to {url} ...")
    sys.stdout.flush()
    # The progress bar goes to stderr (still shown); stdout carries the "HTTP <code>" line.
    result = subprocess.run(build_curl_argv(firmware, host), stdout=subprocess.PIPE,
                            universal_newlines=True)
    t0 = time.time()
    sys.stdout.write(result.stdout or "")
    if result.returncode != 0:
        print(
            "curl failed with exit code {}.\n"
            "  target was: {}\n"
            "  override with: WLED_IP=<ip> pio run -e <env> -t upload\n"
            "  or flash over the cable: pio run -e <env> -t upload "
            "--upload-port /dev/cu.usbserial-XXXX".format(result.returncode, host)
        )
        sys.exit(1)
    code = parse_http_code(result.stdout)
    print("Upload successful — device is rebooting." if 200 <= code < 300
          else f"Upload returned HTTP {code} — device may NOT have been flashed.")

    if not remap:
        return
    if not 200 <= code < 300:
        print(f"WARNING: firmware upload returned HTTP {code}; preset remap skipped")
        return
    # apply waits until /json/info reports an uptime shorter than the time since t0 (the
    # device rebooted into the new firmware), remaps, uploads, and reboots once if changed.
    timeout = os.environ.get("WLED_REMAP_TIMEOUT", "60")
    try:
        timeout = str(float(timeout))
    except ValueError:
        print(f"WARNING: WLED_REMAP_TIMEOUT={timeout!r} is not a number; using 60")
        timeout = "60"
    run_remap(env, ["apply", host, snapdir, "--wait-since", f"{t0:.3f}", "--timeout", timeout,
                    "--reboot"])


# --- SCons wiring -----------------------------------------------------------
# Guarded so the module stays importable by tests outside SCons, where Import()
# and env do not exist.
try:
    Import("env")  # noqa: F821  (injected by SCons)
except NameError:
    env = None

if env is not None:
    _target = resolve_target(env)
    _serial = is_serial_target(_target)
    if not env.get("_WLED_UPLOAD_PRE_DONE"):
        # PRE pass: hand serial uploads to the platform's own esptool path.
        env["_WLED_UPLOAD_PRE_DONE"] = True
        if _serial:
            env.Replace(UPLOAD_PROTOCOL="esptool")
    elif not _serial:
        # POST pass: OTA only. Never replace UPLOADCMD for a serial target -- that
        # is what the platform builder has just assembled.
        env.Replace(UPLOADCMD=_upload_wled)
