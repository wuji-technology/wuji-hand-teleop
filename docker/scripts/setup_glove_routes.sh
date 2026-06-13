#!/bin/bash
# setup_glove_routes.sh — make Wuji Glove IPs reachable on multi-NIC harnesses.
#
# THE PROBLEM
#   Some harnesses wire each glove receiver to its own NIC, with every NIC on
#   the SAME subnet (e.g. three ports all on 192.168.1.0/24). Linux then sends
#   unicast to a glove out the single lowest-metric route — one NIC — which may
#   not be the NIC the glove actually sits behind. ARP never resolves, so the
#   SDK times out with "Connection timeout" *even though it can DISCOVER the
#   glove over UDP broadcast* (broadcast goes out every NIC; unicast does not).
#
#   Worse, the two ports of a dual-port NIC (enpXsYf0 / enpXsYf1) can be
#   enumerated in a different order on each boot, so any glove->NIC route that
#   hardcodes an interface NAME is correct only half the time.
#
# THE FIX (what this does)
#   For each glove IP: if it is already reachable via the default route, do
#   nothing. Otherwise probe each candidate NIC with `ping -I <nic>` to find the
#   one that actually reaches it, and pin a /32 route to that NIC. This adapts
#   to interface renames automatically (it re-probes every run) and is a no-op
#   on ordinary single-NIC setups.
#
#   Idempotent — safe to re-run any time (e.g. after powering the gloves on):
#       bash setup_glove_routes.sh
#
# Requires NET_ADMIN (the docker-compose service sets cap_add: NET_ADMIN).
set -u

# Glove IPs to ensure reachability for. Factory convention: .100 = left,
# .101 = right on the operator's harness LAN. Override with e.g.
#   WUJI_GLOVE_IPS="192.168.1.100 192.168.1.101 192.168.1.102"
GLOVE_IPS="${WUJI_GLOVE_IPS:-192.168.1.100 192.168.1.101}"

log() { echo "[glove-routes] $*"; }
have() { command -v "$1" >/dev/null 2>&1; }

# Pin <ip>/32 to <nic>, replacing any stale route from a previous (swapped) boot.
add_route() {
    local ip="$1" nic="$2"
    if have ip; then
        ip route replace "${ip}/32" dev "$nic"
    else
        route del -host "$ip" 2>/dev/null || true   # drop a stale/wrong pin
        route add -host "$ip" dev "$nic"
    fi
}

# Up NICs (excluding loopback) that hold an IPv4 in the glove's /24 — the only
# interfaces worth probing.
candidate_nics() {
    local prefix="${1%.*}."                      # 192.168.1.100 -> 192.168.1.
    local esc="${prefix//./\\.}"
    local d n
    for d in /sys/class/net/*; do
        n=$(basename "$d")
        [ "$n" = "lo" ] && continue
        [ "$(cat "$d/operstate" 2>/dev/null)" = "up" ] || continue
        if have ip; then
            ip -4 addr show "$n" 2>/dev/null | grep -qE "inet ${esc}[0-9]+" && echo "$n"
        else
            ifconfig "$n" 2>/dev/null | grep -qE "inet (addr:)?${esc}[0-9]+" && echo "$n"
        fi
    done
}

for ip in $GLOVE_IPS; do
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        log "$ip reachable via default route — ok"
        continue
    fi
    pinned=""
    for nic in $(candidate_nics "$ip"); do
        if ping -c1 -W1 -I "$nic" "$ip" >/dev/null 2>&1; then
            if add_route "$ip" "$nic"; then
                log "pinned $ip -> $nic"
                pinned=1
            else
                log "WARN: failed to pin $ip -> $nic (need NET_ADMIN?)"
            fi
            break
        fi
    done
    [ -z "$pinned" ] && log "$ip not reachable on any NIC — glove powered off / not connected?"
done
