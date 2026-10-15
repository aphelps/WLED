"""
PlatformIO custom upload script: flash WLED over HTTP using the /update endpoint.
Uses curl (not Python urllib) to work around macOS Sequoia local-network permission
restrictions that block UDP (espota) and Python's urllib but not system binaries.

Target IP is taken from upload_port in platformio.ini, or overridden via the
WLED_IP environment variable:
    WLED_IP=192.168.1.99 pio run -e ampworks -t upload

Preset remap: a firmware update can renumber effects (presets store the numeric fx id).
Around the flash this script runs tools/remap_presets.py: snapshot /json/eff +
/presets.json before uploading, then after a successful upload wait for the device to come
back, remap presets by effect name, and reboot once if anything changed. The snapshot is kept
under $BUILD_DIR/preset_remap/<ip>; an earlier snapshot that was never applied (e.g. the
post-flash wait timed out) is kept and used instead. Opt out with WLED_NO_PRESET_REMAP=1. The
post-flash readiness wait defaults to 60 s; override with WLED_REMAP_TIMEOUT=<seconds>. A remap
problem only warns: the firmware is already flashed, so it never fails the upload.
"""
import os
import re
import subprocess
import sys
import time
Import("env")


SNAPSHOT_KEPT = 3  # remap_presets.py snapshot: kept an earlier un-applied snapshot


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


def upload_wled(source, target, env):
    firmware = str(source[0])
    ip = os.environ.get("WLED_IP", env.GetProjectOption("upload_port", "192.168.1.55"))
    url = f"http://{ip}/update"

    remap = os.environ.get("WLED_NO_PRESET_REMAP", "") not in ("1", "true", "yes")
    snapdir = os.path.join(env.subst("$BUILD_DIR"), "preset_remap", ip)
    if remap:
        print(f"\nSnapshotting effects + presets from {ip} for the post-flash preset remap ...")
        if run_remap(env, ["snapshot", ip, snapdir], ok=(0, SNAPSHOT_KEPT)) not in (0, SNAPSHOT_KEPT):
            print("WARNING: preset snapshot failed; preset remap skipped, continuing with upload")
            remap = False

    print(f"\nUploading {firmware} to {url} ...")

    cmd = [
        "curl", "-X", "POST", url,
        "-F", f"update=@{firmware}",
        "--progress-bar",
        "--max-time", "60",
        "-w", "\nHTTP %{http_code}\n",
        "-o", "/dev/null",
    ]

    sys.stdout.flush()
    # The progress bar goes to stderr (still shown); stdout carries the "HTTP <code>" line.
    result = subprocess.run(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    t0 = time.time()
    sys.stdout.write(result.stdout or "")
    if result.returncode != 0:
        print(f"curl failed with exit code {result.returncode}")
        sys.exit(1)
    m = re.search(r"HTTP (\d{3})", result.stdout or "")
    code = int(m.group(1)) if m else 0
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
    run_remap(env, ["apply", ip, snapdir, "--wait-since", f"{t0:.3f}", "--timeout", timeout,
                    "--reboot"])


env.Replace(UPLOADCMD=upload_wled)
