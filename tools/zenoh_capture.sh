#!/usr/bin/env bash
#
# Capture outbound zenoh egress from THIS host to verify wuji_sdk
# connect() with enable_bridge=False does not advertise the connected
# glove on the LAN.
#
# Run (host, as root):
#   sudo apt-get install -y tcpdump            # one-time
#   sudo ./tools/zenoh_capture.sh eth0 70      # 70s, exit 0=PASS / 1=FAIL
#
# Pair with: ./tools/verify_zenoh_connect.sh (drives the connection).
#
# Filter rationale:
#   - src host <ourIP>     : only count packets WE emit (ignore zenoh
#                            multicast other machines spray at us)
#   - tcp port 7447        : zenoh peer/router unicast
#   - udp port 7446        : zenoh scouting unicast
#   - udp dst 224.0.0.224  : zenoh scouting multicast (TTL=1, link-local)

set -euo pipefail

IFACE="${1:-}"
DURATION="${2:-30}"

[[ -n "${IFACE}" ]] || { echo "usage: $0 <iface> [duration_seconds]" >&2; exit 2; }
[[ ${EUID} -eq 0 ]] || { echo "error: must run as root (use sudo)" >&2; exit 2; }
command -v tcpdump >/dev/null || { echo "error: tcpdump missing (apt-get install tcpdump)" >&2; exit 2; }

IPS="$(ip -4 -o addr show dev "${IFACE}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
[[ -n "${IPS}" ]] || { echo "error: no IPv4 on ${IFACE}" >&2; exit 2; }

SRC=""
for ip in ${IPS}; do
  [[ -z "${SRC}" ]] && SRC="src host ${ip}" || SRC="${SRC} or src host ${ip}"
done
FILTER="(${SRC}) and ((tcp port 7447) or (udp port 7446) or (udp and dst host 224.0.0.224))"

echo "info: iface=${IFACE} src=[${IPS//$'\n'/ }] duration=${DURATION}s" >&2
OUT="$(timeout "${DURATION}" tcpdump -i "${IFACE}" -nn -l "${FILTER}" 2>/dev/null || true)"
COUNT="$(printf '%s' "${OUT}" | grep -c '^' || true)"
COUNT="${COUNT:-0}"

[[ -n "${OUT}" ]] && echo "${OUT}"

echo
echo "===== zenoh egress report ====="
echo "interface : ${IFACE}"
echo "src IPs   : ${IPS//$'\n'/ }"
echo "duration  : ${DURATION}s"
echo "filter    : ${FILTER}"
echo "packets   : ${COUNT}"

if [[ "${COUNT}" -gt 0 ]]; then
  echo "result    : FAIL — outbound zenoh traffic detected"
  exit 1
fi
echo "result    : PASS — no outbound zenoh traffic"
