#!/bin/bash
# launch_ui_docker.sh — host-side desktop launcher for any wuji_teleop_monitor UI.
#
# Usage:  launch_ui_docker.sh {monitor|brake|camera}
#
# Docker is the only supported deployment, so the desktop icons run the GUI
# inside the wuji-hand-teleop container via `docker exec`. This wrapper:
#   1. Allows the container to reach the host X server (xhost +local:docker).
#   2. Verifies the container is running and reports clearly when it isn't.
#   3. Spawns `ros2 run wuji_teleop_monitor <ui>` inside the container with
#      DISPLAY forwarded so Qt5 paints on the host desktop.
#
# Override the container name with WUJI_TELEOP_CONTAINER if you renamed the
# service in docker-compose.yml.

set -u

UI="${1:-monitor}"
case "$UI" in
    monitor|brake|camera) ;;
    *)
        echo "[ERROR] Unknown UI: '$UI' (expected: monitor | brake | camera)" >&2
        exit 2
        ;;
esac

CONTAINER="${WUJI_TELEOP_CONTAINER:-wuji-hand-teleop}"

# Idempotent X11 grant for the container's user.
xhost +local:docker >/dev/null 2>&1 || true

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q '^true$'; then
    msg="Wuji Teleop container ($CONTAINER) is not running.

Start it first:
  cd <repo>/docker && docker compose up -d"
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --title="Wuji Teleop" --text="$msg" 2>/dev/null || true
    elif command -v notify-send >/dev/null 2>&1; then
        notify-send "Wuji Teleop" "$msg" 2>/dev/null || true
    fi
    echo "[ERROR] $msg" >&2
    exit 1
fi

exec docker exec \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    "$CONTAINER" \
    bash -lc '
        source /opt/ros/humble/setup.bash
        source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
        exec ros2 run wuji_teleop_monitor "$1"
    ' bash "$UI"
