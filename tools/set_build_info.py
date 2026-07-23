"""
PlatformIO pre-build script: inject git commit hash, branch, and build timestamp
as global CPPDEFINES so they can be embedded in the firmware binary.

Defines injected (all string literals):
  WLED_GIT_HASH   — short commit hash, e.g. "abc1234" or "abc1234+dirty"
  WLED_GIT_BRANCH — branch name, e.g. "main"
  WLED_BUILD_TIME — UTC ISO-8601 timestamp, e.g. "2026-04-14T22:30:00Z"

Consumed in wled00/json.cpp serializeInfo() when WLED_GIT_HASH is defined.
"""
Import("env")
import subprocess
from datetime import datetime, timezone

def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None

commit     = run(["git", "rev-parse", "--short", "HEAD"]) or "unknown"
branch     = run(["git", "rev-parse", "--abbrev-ref", "HEAD"]) or "unknown"
dirty      = bool(run(["git", "status", "--porcelain"]))
hash_str   = commit + ("+dirty" if dirty else "")
build_time = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

print(f"Build info: {hash_str}  branch={branch}  time={build_time}")

env.Append(CPPDEFINES=[
    ("WLED_GIT_HASH",   f'\\\"{hash_str}\\\"'),
    ("WLED_GIT_BRANCH", f'\\\"{branch}\\\"'),
    ("WLED_BUILD_TIME", f'\\\"{build_time}\\\"'),
])
