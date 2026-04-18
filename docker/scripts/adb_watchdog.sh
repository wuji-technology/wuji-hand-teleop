#!/bin/bash
# =============================================================================
# ADB Reverse Port Monitor
# =============================================================================
# PICO USB ADB reverse is session-level, lost after disconnection.
# This script runs as a background process, checking PICO USB connection status every 5 seconds:
#   - Connected: automatically establish reverse port mapping
#   - Disconnected: silently wait for reconnection
#
# Ports:
#   63901 → PC-Service control channel (PICO Connect button)
#   13579 → XRoboCompatServer camera command channel
#
# No USB device in HTC/WiFi mode, idle with no side effects.
# =============================================================================

# Ignore SIGINT/SIGTERM — do not exit with ros2 launch Ctrl+C
trap '' INT TERM

PORTS="63901 13579"
INTERVAL=5
LAST_STATE=""

while true; do
    if adb devices 2>/dev/null | grep -q "device$"; then
        # Check reverse ports exist on every loop (may be cleared by PICO disconnection)
        MISSING=false
        for PORT in $PORTS; do
            if ! adb reverse --list 2>/dev/null | grep -q "tcp:$PORT"; then
                MISSING=true
                break
            fi
        done

        if [ "$MISSING" = "true" ]; then
            SERIAL=$(adb devices 2>/dev/null | grep "device$" | head -1 | cut -f1)
            [ "$LAST_STATE" != "connected" ] && echo "[ADB watchdog] PICO detected ($SERIAL)"
            for PORT in $PORTS; do
                adb reverse tcp:$PORT tcp:$PORT 2>/dev/null
            done
            echo "[ADB watchdog] Reverse ports set: $PORTS"
        fi
        LAST_STATE="connected"
    else
        if [ "$LAST_STATE" = "connected" ]; then
            echo "[ADB watchdog] PICO disconnected, waiting for reconnect..."
        fi
        LAST_STATE="disconnected"
    fi
    sleep $INTERVAL
done
