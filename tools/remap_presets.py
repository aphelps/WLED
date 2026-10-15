#!/usr/bin/env python3
"""
Name-based WLED preset remap: keep saved presets pointing at the same effect after a
firmware update renumbers effects.

Presets store the numeric effect id (`fx`). When an update changes which id an effect has
(e.g. custom usermod effects moving because upstream added built-ins), every preset on a
moved effect silently starts a different one. This tool remaps by NAME:

    snapshot <host> <dir> [--force]       save /json/eff and /presets.json before flashing
    apply    <host> <dir> [--dry-run]     after flashing: old id -> name -> new id, upload
             [--wait-since EPOCH] [--timeout S] [--reboot] [--force]
    --self-test                           offline fixture test, no device needed

What `apply` rewrites: only integer `fx` values inside a preset's `seg` (a dict or a list of
dicts). Left untouched and reported where relevant: string fx values (`~`, `~-`, `r`, `w~`),
playlist presets, presets whose `win` HTTP-API string sets `FX=`, names missing from the new
effect table, names that are ambiguous in the new table, and RSVD (placeholder) names.

`apply` rewrites the SNAPSHOT's presets and compares the result with what the device holds
now: equal -> nothing to do (idempotent, no upload); device changed since the snapshot ->
refuses unless --force (so a stale snapshot can never clobber newer edits).

A snapshot stays PENDING (marker file `<dir>/pending`) until an `apply` resolves it (uploaded,
nothing to change, or a confirmed reboot into an identical effect table). `snapshot` will not
overwrite a pending snapshot whose effect table differs from the device's current one -- that
snapshot is the only record of the old id -> name mapping (e.g. the post-flash wait timed out
and the operator simply re-ran the upload). It keeps it, exits 3, and the next `apply` uses
it. `snapshot --force` discards it.

All HTTP goes through the system `curl` binary, never urllib/http.client: macOS Sequoia's
local-network permission blocks Python's own sockets but not system binaries (see
upload_wled.py).

Exit status: 0 = done (including "nothing to change"), 1 = error / nothing applied,
2 = usage, 3 = snapshot: kept an earlier un-applied snapshot (apply will use it).
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time

CURL_TIMEOUT = 15  # seconds per request


# ----------------------------------------------------------------------------------------
# HTTP (curl only)
# ----------------------------------------------------------------------------------------

def curl(args, timeout=CURL_TIMEOUT):
    """Run curl with `args`; return (http_code, body_bytes). http_code is 0 when curl could
    not get a response at all (connection refused, timeout, ...)."""
    fd, out = tempfile.mkstemp(prefix="remap_presets_")
    os.close(fd)
    try:
        cmd = ["curl", "-s", "--max-time", str(timeout), "-o", out, "-w", "%{http_code}"] + args
        try:
            r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               timeout=timeout + 5)
        except subprocess.TimeoutExpired:
            return 0, b""
        try:
            code = int((r.stdout or b"0").decode().strip() or 0)
        except ValueError:
            code = 0
        with open(out, "rb") as f:
            body = f.read()
        return code, body
    finally:
        os.unlink(out)


def get_json(host, path, timeout=CURL_TIMEOUT):
    """GET http://host/path and parse JSON. Returns (code, obj or None)."""
    code, body = curl([f"http://{host}{path}"], timeout=timeout)
    if code != 200:
        return code, None
    try:
        return code, json.loads(body.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return code, None


def upload_presets(host, path):
    code, body = curl(["-X", "POST", "-F", f"data=@{path};filename=/presets.json",
                       f"http://{host}/upload"], timeout=30)
    return code, body.decode("utf-8", "replace").strip()


def reboot(host):
    code, _ = curl(["-X", "POST", "-H", "Content-Type: application/json",
                    "-d", '{"rb":true}', f"http://{host}/json/state"])
    return code


def wait_for_reboot(host, since_epoch, timeout, interval=2.0):
    """Poll /json/info until the device answers with an uptime shorter than the time since
    `since_epoch` (proof it rebooted after the flash: WLED replies to /update and restarts
    afterwards, so an early poll can still reach the OLD firmware). Returns True on success."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        code, info = get_json(host, "/json/info", timeout=5)
        if code == 200 and isinstance(info, dict):
            uptime = info.get("uptime")
            if isinstance(uptime, (int, float)) and uptime < time.time() - since_epoch:
                return True
        time.sleep(interval)
    return False


# ----------------------------------------------------------------------------------------
# Pure remap logic (no I/O; exercised by --self-test)
# ----------------------------------------------------------------------------------------

def is_reserved(name):
    return isinstance(name, str) and name.startswith("RSVD")


def build_name_to_id(eff):
    """name -> id for names that are unique in `eff` and not RSVD. Returns (map, ambiguous)."""
    seen = {}
    for i, name in enumerate(eff):
        seen.setdefault(name, []).append(i)
    mapping, ambiguous = {}, set()
    for name, ids in seen.items():
        if is_reserved(name):
            continue
        if len(ids) == 1:
            mapping[name] = ids[0]
        else:
            ambiguous.add(name)
    return mapping, ambiguous


def iter_segs(preset):
    """Yield (index_label, seg_dict) for every seg dict in a preset."""
    seg = preset.get("seg")
    if isinstance(seg, dict):
        yield "0", seg
    elif isinstance(seg, list):
        for i, s in enumerate(seg):
            if isinstance(s, dict):
                yield str(s.get("id", i)), s


def remap(old_eff, new_eff, presets):
    """Return (new_presets, changes, notes). `changes` lists rewrites as strings; `notes`
    lists everything reported but deliberately left alone. Does not mutate `presets`."""
    out = json.loads(json.dumps(presets))  # deep copy, preserving key order
    changes, notes = [], []
    name_to_id, ambiguous = build_name_to_id(new_eff)

    def pid_key(k):
        return (0, int(k)) if str(k).isdigit() else (1, str(k))

    for pid in sorted(out.keys(), key=pid_key):
        p = out[pid]
        if not isinstance(p, dict):
            continue
        label = f"preset {pid}" + (f" ({p['n']})" if isinstance(p.get("n"), str) else "")
        win = p.get("win")
        if isinstance(win, str) and "FX=" in win.upper():
            notes.append(f"{label}: HTTP-API string sets FX= ({win!r}); NOT rewritten, check by hand")
        for seg_label, seg in iter_segs(p):
            fx = seg.get("fx")
            if isinstance(fx, bool) or not isinstance(fx, int):
                continue  # absent, or a string like "~", "~-", "r", "w~": relative/random, leave
            where = f"{label} seg {seg_label}"
            if fx < 0 or fx >= len(old_eff):
                notes.append(f"{where}: fx {fx} not in the old effect table; left as is")
                continue
            name = old_eff[fx]
            if is_reserved(name):
                notes.append(f"{where}: fx {fx} was a reserved slot ({name}); left as is")
                continue
            if name in ambiguous:
                notes.append(f"{where}: effect {name!r} is ambiguous in the new table; left at {fx}")
                continue
            if name not in name_to_id:
                notes.append(f"{where}: effect {name!r} (fx {fx}) is not in the new table; left as is")
                continue
            new_id = name_to_id[name]
            if new_id != fx:
                seg["fx"] = new_id
                changes.append(f"{where}: {name} {fx} -> {new_id}")
    return out, changes, notes


def same_table_warning(old_eff, new_eff, snapdir):
    if old_eff == new_eff:
        return ("effect list unchanged since the snapshot: device may not have rebooted into "
                f"new firmware; presets NOT remapped, snapshot at {snapdir}")
    return None


# ----------------------------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------------------------

def write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, separators=(",", ":"))
    os.replace(tmp, path)


PENDING = "pending"
SNAPSHOT_KEPT = 3


def pending_path(snapdir):
    return os.path.join(snapdir, PENDING)


def clear_pending(snapdir):
    try:
        os.unlink(pending_path(snapdir))
    except OSError:
        pass


def cmd_snapshot(host, snapdir, force=False):
    code, eff = get_json(host, "/json/eff")
    if not isinstance(eff, list):
        print(f"remap_presets: could not read /json/eff from {host} (HTTP {code})")
        return 1
    if not force and os.path.exists(pending_path(snapdir)):
        try:
            with open(os.path.join(snapdir, "eff.json")) as f:
                prev_eff = json.load(f)
        except (OSError, ValueError):
            prev_eff = None
        if isinstance(prev_eff, list) and prev_eff != eff:
            print(f"remap_presets: keeping the earlier snapshot in {snapdir}: it was never "
                  f"applied and {host}'s effect table has changed since, so it is the only "
                  "record of the old effect ids. The next apply will use it "
                  "(snapshot --force discards it).")
            return SNAPSHOT_KEPT
    code, presets = get_json(host, "/presets.json")
    if code == 404:
        presets = {}
    elif not isinstance(presets, dict):
        print(f"remap_presets: could not read /presets.json from {host} (HTTP {code})")
        return 1
    os.makedirs(snapdir, exist_ok=True)
    write_json(os.path.join(snapdir, "eff.json"), eff)
    write_json(os.path.join(snapdir, "presets.json"), presets)
    with open(pending_path(snapdir), "w") as f:
        f.write(f"{time.time():.3f}\n")
    print(f"remap_presets: snapshot of {host} ({len(eff)} effects, {len(presets)} presets) "
          f"saved to {snapdir}")
    return 0


def cmd_apply(host, snapdir, dry_run=False, wait_since=None, timeout=60, do_reboot=False,
              force=False):
    try:
        with open(os.path.join(snapdir, "eff.json")) as f:
            old_eff = json.load(f)
        with open(os.path.join(snapdir, "presets.json")) as f:
            snap_presets = json.load(f)
    except (OSError, ValueError) as e:
        print(f"remap_presets: no usable snapshot in {snapdir}: {e}")
        return 1

    if wait_since is not None:
        print(f"remap_presets: waiting up to {timeout}s for {host} to come back from reboot ...")
        if not wait_for_reboot(host, wait_since, timeout):
            print(f"remap_presets: WARNING {host} did not come back with a fresh uptime within "
                  f"{timeout}s; presets NOT remapped, snapshot at {snapdir}. Once the device "
                  f"is up, run `python3 tools/remap_presets.py apply {host} {snapdir} --reboot` "
                  "before flashing again (a re-flash keeps this snapshot, but don't rely on it).")
            return 1

    code, new_eff = get_json(host, "/json/eff")
    if not isinstance(new_eff, list):
        print(f"remap_presets: could not read /json/eff from {host} (HTTP {code}); "
              f"presets NOT remapped, snapshot at {snapdir}")
        return 1
    if old_eff == new_eff:
        if wait_since is not None:
            # The uptime check proved a reboot into the flashed firmware: same build re-flashed
            # (or no effect changes), so there is genuinely nothing to remap.
            print("remap_presets: effect list unchanged (reboot confirmed); nothing to remap")
            clear_pending(snapdir)
        else:
            print(f"remap_presets: WARNING {same_table_warning(old_eff, new_eff, snapdir)}")
        return 0

    new_presets, changes, notes = remap(old_eff, new_eff, snap_presets)
    for n in notes:
        print(f"remap_presets: note: {n}")

    code, current = get_json(host, "/presets.json")
    if code == 404:
        current = {}
    elif not isinstance(current, dict):
        print(f"remap_presets: could not read current /presets.json from {host} (HTTP {code}); "
              f"presets NOT remapped, snapshot at {snapdir}")
        return 1
    if current == new_presets:
        print("remap_presets: presets already match the new effect table; nothing to upload")
        if not dry_run:
            clear_pending(snapdir)
        return 0
    if current != snap_presets and not force:
        print("remap_presets: WARNING presets on the device changed since the snapshot; not "
              f"overwriting them (re-run with --force to do so). Snapshot at {snapdir}")
        return 1
    for c in changes:
        print(f"remap_presets: {'would change' if dry_run else 'change'}: {c}")
    if dry_run:
        print("remap_presets: dry run, nothing uploaded")
        return 0

    out = os.path.join(snapdir, "presets.remapped.json")
    write_json(out, new_presets)
    code, body = upload_presets(host, out)
    if code == 401:
        print(f"remap_presets: WARNING upload refused (HTTP 401): presets NOT remapped (PIN "
              f"locked). Unlock the settings PIN and upload {out} by hand, or re-run apply. "
              f"Snapshot at {snapdir}")
        return 1
    if not 200 <= code < 300:
        print(f"remap_presets: WARNING upload failed (HTTP {code} {body!r}); presets NOT "
              f"remapped. Rewritten file at {out}, snapshot at {snapdir}")
        return 1
    print(f"remap_presets: uploaded remapped presets ({len(changes)} change(s)) to {host}")
    clear_pending(snapdir)
    if do_reboot:
        rc = reboot(host)
        if 200 <= rc < 300:
            print(f"remap_presets: rebooting {host} to load the new presets")
        else:
            print(f"remap_presets: WARNING reboot request failed (HTTP {rc}); reboot {host} by hand")
    return 0


# ----------------------------------------------------------------------------------------
# Self-test (offline fixture)
# ----------------------------------------------------------------------------------------

def self_test():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # Old firmware: built-ins 0..160, custom effects appended at 161.. (the pre-pin layout).
    old_eff = [f"Builtin {i}" for i in range(161)]
    old_eff[51] = "Gradient"
    old_eff[60] = "Twin"            # will be duplicated in the new table -> ambiguous
    old_eff[70] = "RSVD"            # a placeholder in the old table
    old_eff += ["AMP AI", "AMP AI Audio", "AMP Moving SIN", "Touch Pond", "Touch Grid",
                "Retired FX", "Builtin 166x", "Builtin 167x", "Builtin 168x", "Builtin 169x",
                "HMTL Sparkle"]  # HMTL Sparkle at 171
    assert old_eff[161] == "AMP AI" and old_eff[171] == "HMTL Sparkle"

    # New firmware: same built-ins (+ a duplicate "Twin"), RSVD padding, customs pinned at 240+.
    new_eff = [f"Builtin {i}" for i in range(161)]
    new_eff[51] = "Gradient"
    new_eff[60] = "Twin"
    new_eff += [f"Builtin {i}x" for i in range(166, 170)]
    new_eff += ["Twin"]                                  # ambiguous name
    new_eff += ["RSVD"] * (240 - len(new_eff))
    new_eff += ["AMP AI", "AMP AI Audio", "AMP Moving SIN", "HMTL Sparkle", "Touch Pond",
                "Touch Grid"]
    assert new_eff.index("HMTL Sparkle") == 243 and len(new_eff) == 246

    presets = {
        "0": {},
        "1": {"n": "Sparkle", "on": True, "seg": [{"id": 0, "fx": 171}, {"id": 1, "fx": 161}]},
        "2": {"n": "Grad", "seg": {"id": 0, "fx": 51}},                 # seg as a dict
        "3": {"n": "Pond", "seg": [{"id": 0, "fx": 164}]},
        "4": {"n": "Next", "seg": [{"fx": "~"}, {"fx": "r"}, {"fx": "~-"}]},
        "5": {"n": "Gone", "seg": [{"fx": 166}]},                       # Retired FX: not in new
        "6": {"n": "Api", "win": "&FX=161&SX=10"},
        "7": {"n": "Dup", "seg": [{"fx": 60}]},                         # ambiguous in new
        "8": {"n": "Rsvd", "seg": [{"fx": 70}]},                        # RSVD in old
        "9": {"n": "List", "playlist": {"ps": [1, 2], "dur": [100, 100]}},
        "10": {"n": "Range", "seg": [{"fx": 250}]},
    }
    orig = json.loads(json.dumps(presets))

    new_presets, changes, notes = remap(old_eff, new_eff, presets)
    check(presets == orig, "input presets were mutated")
    check(new_presets["1"]["seg"][0]["fx"] == 243, "HMTL Sparkle 171 -> 243")
    check(new_presets["1"]["seg"][1]["fx"] == 240, "AMP AI 161 -> 240")
    check(new_presets["2"]["seg"]["fx"] == 51, "stock 51 unchanged (seg as dict)")
    check(new_presets["3"]["seg"][0]["fx"] == 244, "Touch Pond 164 -> 244")
    check([s["fx"] for s in new_presets["4"]["seg"]] == ["~", "r", "~-"], "string fx untouched")
    check(new_presets["5"]["seg"][0]["fx"] == 166, "unknown name left untouched")
    check(any("Retired FX" in n and "not in the new table" in n for n in notes),
          "unknown name reported")
    check(new_presets["6"] == orig["6"], "win preset not rewritten")
    check(any("preset 6" in n and "FX=" in n for n in notes), "win FX= preset reported")
    check(new_presets["7"]["seg"][0]["fx"] == 60, "ambiguous name left untouched")
    check(any("'Twin'" in n and "ambiguous" in n for n in notes), "ambiguous name reported")
    check(new_presets["8"]["seg"][0]["fx"] == 70, "RSVD old slot left untouched")
    check(any("reserved slot" in n for n in notes), "RSVD old slot reported")
    check(new_presets["9"] == orig["9"], "playlist untouched")
    check(new_presets["10"]["seg"][0]["fx"] == 250, "out-of-range fx left untouched")
    check(len(changes) == 3, f"expected 3 changes, got {len(changes)}: {changes}")
    check("RSVD" not in build_name_to_id(new_eff)[0], "RSVD never a mapping target")

    # Identical old/new effect lists -> the "may not have rebooted" warning.
    w = same_table_warning(new_eff, list(new_eff), "/snap")
    check(w is not None and "may not have rebooted" in w and "/snap" in w,
          "identical tables warn")
    check(same_table_warning(old_eff, new_eff, "/snap") is None, "different tables do not warn")

    self_test_commands(check, old_eff, new_eff, presets, new_presets)

    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        print("remap_presets self-test: FAILED")
        return 1
    print("remap_presets self-test: all passed")
    return 0


class _FakeDevice:
    """Scripted WLED for the command self-tests: serves /json/eff and /presets.json, records
    uploads and reboots. Patched over the module's HTTP functions by self_test_commands."""

    def __init__(self, eff, presets, upload_code=200, reboot_code=200, comes_back=True):
        self.eff, self.presets = list(eff), json.loads(json.dumps(presets))
        self.upload_code, self.reboot_code, self.comes_back = upload_code, reboot_code, comes_back
        self.calls = []

    def get_json(self, host, path, timeout=CURL_TIMEOUT):
        self.calls.append(("GET", path))
        if path == "/json/eff":
            return 200, list(self.eff)
        if path == "/presets.json":
            return (404, None) if self.presets is None else (200, json.loads(json.dumps(self.presets)))
        return 404, None

    def upload_presets(self, host, path):
        with open(path) as f:
            body = json.load(f)
        self.calls.append(("UPLOAD", body))
        if 200 <= self.upload_code < 300:
            self.presets = body
        return self.upload_code, ""

    def reboot(self, host):
        self.calls.append(("REBOOT",))
        return self.reboot_code

    def wait_for_reboot(self, host, since_epoch, timeout, interval=2.0):
        self.calls.append(("WAIT",))
        return self.comes_back

    def did(self, kind):
        return [c for c in self.calls if c[0] == kind]


def self_test_commands(check, old_eff, new_eff, presets, new_presets):
    """cmd_snapshot / cmd_apply decision tree against a scripted device (no network)."""
    import contextlib
    import io
    import shutil

    mod = sys.modules[__name__]
    names = ("get_json", "upload_presets", "reboot", "wait_for_reboot")
    saved = {n: getattr(mod, n) for n in names}
    root = tempfile.mkdtemp(prefix="remap_selftest_")

    def run(dev, fn, *a, **kw):
        for n in names:
            setattr(mod, n, getattr(dev, n))
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                rc = fn("dev", *a, **kw)
        finally:
            for n, f in saved.items():
                setattr(mod, n, f)
        return rc, buf.getvalue()

    def fresh(tag, dev):
        d = os.path.join(root, tag)
        rc, _ = run(dev, cmd_snapshot, d)
        check(rc == 0 and os.path.exists(pending_path(d)), f"{tag}: snapshot taken, pending")
        dev.calls.clear()
        return d

    try:
        # Normal flash: old table -> new table, remap uploads, then reboots (after the 2xx).
        dev = _FakeDevice(old_eff, presets)
        d = fresh("normal", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 0, "normal: rc 0")
        check(len(dev.did("UPLOAD")) == 1 and dev.did("UPLOAD")[0][1] == new_presets,
              "normal: uploads the remapped presets")
        kinds = [c[0] for c in dev.calls]
        check("REBOOT" in kinds and kinds.index("REBOOT") > kinds.index("UPLOAD"),
              "normal: reboot only after the upload")
        check(not os.path.exists(pending_path(d)), "normal: pending cleared")

        # Idempotent re-upload of the same build (the real second-run path): new snapshot holds
        # the new table + remapped presets; reboot confirmed; identical tables -> info, no-op.
        rc, _ = run(dev, cmd_snapshot, d)
        dev.calls.clear()
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 0 and "nothing to remap" in out and "WARNING" not in out,
              "re-flash same build: info line, not a warning")
        check(not dev.did("UPLOAD") and not dev.did("REBOOT"), "re-flash same build: no upload/reboot")
        check(not os.path.exists(pending_path(d)), "re-flash same build: pending cleared")

        # Identical tables WITHOUT a confirmed reboot (manual apply) -> warning, pending kept.
        dev = _FakeDevice(new_eff, new_presets)
        d = fresh("manual-same", dev)
        rc, out = run(dev, cmd_apply, d)
        check(rc == 0 and "may not have rebooted" in out, "manual identical tables: warning")
        check(not dev.did("UPLOAD"), "manual identical tables: no upload")
        check(os.path.exists(pending_path(d)), "manual identical tables: pending kept")

        # Wait failure -> 1, nothing uploaded, tells the operator to apply before re-flashing.
        dev = _FakeDevice(old_eff, presets, comes_back=False)
        d = fresh("waitfail", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 1 and "before flashing again" in out, "wait failure: rc 1 + apply hint")
        check(not dev.did("UPLOAD") and not dev.did("REBOOT"), "wait failure: no upload/reboot")
        # ... and the operator just re-runs the upload: snapshot must KEEP the old table.
        rc, out = run(dev, cmd_snapshot, d)
        check(rc == SNAPSHOT_KEPT, "re-snapshot after failed apply: kept (rc 3)")
        with open(os.path.join(d, "eff.json")) as f:
            check(json.load(f) == old_eff, "re-snapshot after failed apply: old table preserved")
        dev.comes_back = True
        dev.calls.clear()
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 0 and dev.did("UPLOAD") and dev.did("UPLOAD")[0][1] == new_presets,
              "apply after kept snapshot: presets still remapped")
        # snapshot --force discards a pending snapshot.
        dev2 = _FakeDevice(old_eff, presets)
        d2 = fresh("force", dev2)
        dev2.eff = list(new_eff)
        rc, _ = run(dev2, cmd_snapshot, d2, force=True)
        with open(os.path.join(d2, "eff.json")) as f:
            check(rc == 0 and json.load(f) == new_eff, "snapshot --force overwrites")

        # Device already holds the remapped presets (old snapshot) -> no-op, no upload.
        dev = _FakeDevice(old_eff, presets)
        d = fresh("noop", dev)
        dev.eff, dev.presets = list(new_eff), json.loads(json.dumps(new_presets))
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 0 and "nothing to upload" in out, "already remapped: no-op rc 0")
        check(not dev.did("UPLOAD") and not dev.did("REBOOT"), "already remapped: no upload/reboot")

        # Presets edited on the device since the snapshot -> refuse (1), unless --force.
        dev = _FakeDevice(old_eff, presets)
        d = fresh("changed", dev)
        dev.eff = list(new_eff)
        dev.presets["11"] = {"n": "New since snapshot"}
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 1 and "changed since the snapshot" in out, "changed since snapshot: refuse")
        check(not dev.did("UPLOAD") and not dev.did("REBOOT"), "changed since snapshot: no upload")
        check(os.path.exists(pending_path(d)), "changed since snapshot: pending kept")
        rc, _ = run(dev, cmd_apply, d, force=True)
        check(rc == 0 and dev.did("UPLOAD"), "changed since snapshot + --force: uploads")

        # PIN-locked upload (401) -> 1, PIN message, never reboots.
        dev = _FakeDevice(old_eff, presets, upload_code=401)
        d = fresh("pin", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 1 and "PIN" in out and d in out, "401: rc 1, PIN message with snapshot path")
        check(not dev.did("REBOOT"), "401: no reboot")
        check(os.path.exists(pending_path(d)), "401: pending kept")

        # Other upload failure (500) -> 1, no reboot.
        dev = _FakeDevice(old_eff, presets, upload_code=500)
        d = fresh("500", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 1 and not dev.did("REBOOT"), "500: rc 1, no reboot")

        # Successful upload without --reboot -> no reboot; dry run -> no upload.
        dev = _FakeDevice(old_eff, presets)
        d = fresh("noreboot", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, dry_run=True)
        check(rc == 0 and not dev.did("UPLOAD") and "would change" in out, "dry run: no upload")
        check(os.path.exists(pending_path(d)), "dry run: pending kept")
        rc, out = run(dev, cmd_apply, d)
        check(rc == 0 and dev.did("UPLOAD") and not dev.did("REBOOT"), "no --reboot: no reboot")

        # Device with no presets.json (404) snapshots as {} and applies as a no-op.
        dev = _FakeDevice(old_eff, None)
        d = fresh("nopresets", dev)
        dev.eff = list(new_eff)
        rc, out = run(dev, cmd_apply, d, wait_since=0.0, do_reboot=True)
        check(rc == 0 and not dev.did("UPLOAD"), "no presets.json: no-op")
    finally:
        shutil.rmtree(root, ignore_errors=True)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if argv == ["--self-test"]:
        return self_test()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    s = sub.add_parser("snapshot", help="save /json/eff and /presets.json")
    s.add_argument("host")
    s.add_argument("dir")
    s.add_argument("--force", action="store_true",
                   help="overwrite an earlier snapshot even if it was never applied")
    a = sub.add_parser("apply", help="remap presets by effect name and upload")
    a.add_argument("host")
    a.add_argument("dir")
    a.add_argument("--dry-run", action="store_true", help="report changes, upload nothing")
    a.add_argument("--wait-since", type=float, metavar="EPOCH",
                   help="first wait for the device to report an uptime shorter than now-EPOCH")
    a.add_argument("--timeout", type=float, default=60, help="readiness wait timeout (s)")
    a.add_argument("--reboot", action="store_true", help="reboot the device after an upload")
    a.add_argument("--force", action="store_true",
                   help="overwrite presets even if they changed on the device since the snapshot")
    args = ap.parse_args(argv)
    if args.cmd == "snapshot":
        return cmd_snapshot(args.host, args.dir, force=args.force)
    if args.cmd == "apply":
        return cmd_apply(args.host, args.dir, dry_run=args.dry_run, wait_since=args.wait_since,
                         timeout=args.timeout, do_reboot=args.reboot, force=args.force)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
