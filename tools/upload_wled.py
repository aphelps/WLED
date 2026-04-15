"""
PlatformIO custom upload script: flash WLED over HTTP using the /update endpoint.
Uses curl (not Python urllib) to work around macOS Sequoia local-network permission
restrictions that block UDP (espota) and Python's urllib but not system binaries.

Target IP is taken from upload_port in platformio.ini, or overridden via the
WLED_IP environment variable:
    WLED_IP=192.168.1.99 pio run -e ampworks -t upload
"""
import os
import subprocess
import sys
Import("env")

def upload_wled(source, target, env):
    firmware = str(source[0])
    ip = os.environ.get("WLED_IP", env.GetProjectOption("upload_port", "192.168.1.55"))
    url = f"http://{ip}/update"

    print(f"\nUploading {firmware} to {url} ...")

    cmd = [
        "curl", "-X", "POST", url,
        "-F", f"update=@{firmware}",
        "--progress-bar",
        "--max-time", "60",
        "-w", "\nHTTP %{http_code}\n",
        "-o", "/dev/null",
    ]

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"curl failed with exit code {result.returncode}")
        sys.exit(1)
    print("Upload successful — device is rebooting.")

env.Replace(UPLOADCMD=upload_wled)
