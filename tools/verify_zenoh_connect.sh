#!/usr/bin/env bash
#
# Probe wuji_sdk SdkManager.connect(options=ConnectOptions(enable_bridge=...))
# inside the wuji-hand-teleop container. Pairs with tools/zenoh_capture.sh to
# verify whether enable_bridge actually suppresses the zenoh device-bridge
# on the wire — i.e. the glove is not advertised on the LAN.
#
# Run (host):
#   # B-round (expect zenoh_capture PASS / 0 packets):
#   ./tools/verify_zenoh_connect.sh WG1KA01260301058 45
#
#   # A-round (intentional regression — expect zenoh_capture FAIL / >0 packets):
#   ./tools/verify_zenoh_connect.sh WG1KA01260301058 45 true
#
# Args:
#   sn        glove serial number (required)
#   duration  seconds to hold connection (default 30)
#   bridge    false (default) | true — value of ConnectOptions.enable_bridge

set -euo pipefail

SN="${1:-}"
DURATION="${2:-30}"
BRIDGE="${3:-false}"
CONTAINER="${CONTAINER:-wuji-hand-teleop}"

[[ -n "${SN}" ]] || { echo "usage: $0 <sn> [duration] [bridge]" >&2; exit 2; }
[[ "${BRIDGE}" =~ ^(true|false)$ ]] || { echo "error: bridge must be true|false" >&2; exit 2; }
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" \
  || { echo "error: container '${CONTAINER}' not running" >&2; exit 2; }

docker exec -i \
  -e SN="${SN}" \
  -e DURATION="${DURATION}" \
  -e BRIDGE="${BRIDGE}" \
  "${CONTAINER}" python3 - <<'PY'
import os
import time
from wuji_sdk import SdkManager, ConnectOptions

sn = os.environ["SN"]
duration = float(os.environ["DURATION"])
enable_bridge = os.environ["BRIDGE"].lower() == "true"

print(f"[verify] sn={sn} bridge={enable_bridge}", flush=True)
opt = ConnectOptions(enable_bridge=enable_bridge)
try:
    device = SdkManager.instance().connect(sn=sn, device_name="verify", options=opt)
    print(f"[verify] connect OK: {device!r}", flush=True)
except Exception as exc:
    print(f"[verify] connect failed: {exc!r}", flush=True)

print(f"[verify] holding {duration:.0f}s", flush=True)
time.sleep(duration)
print("[verify] done", flush=True)
PY
