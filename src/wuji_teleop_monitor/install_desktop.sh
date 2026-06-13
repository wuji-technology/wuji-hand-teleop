#!/bin/bash
# install_desktop.sh — install all three Wuji Teleop UI desktop shortcuts.
#
#   teleop-monitor.desktop  → Monitor (one-click teleop launch)
#   brake-control.desktop   → direct-SDK arm brake / recovery
#   camera-preview.desktop  → three-feed camera preview
#
# Usage (run on the host, not inside the container):
#   cd <repo>/src/wuji_teleop_monitor
#   ./install_desktop.sh
#
# What it does:
#   - Resolves the desktop directory via xdg-user-dir (locale-aware).
#   - Substitutes {{HOME}}, {{PACKAGE_PATH}}, {{LAUNCHER}} into each .desktop
#     template, copies the result onto the desktop, chmod +x, and marks it
#     trusted under GNOME via `gio set ... metadata::trusted true`.
#   - All three shortcuts dispatch into the wuji-hand-teleop container via
#     scripts/launch_ui_docker.sh <monitor|brake|camera> — Docker is the
#     only supported deployment.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"

if [ ! -d "$DESKTOP_DIR" ]; then
    echo "[ERROR] Desktop directory not found: $DESKTOP_DIR" >&2
    exit 1
fi

PACKAGE_PATH="$SCRIPT_DIR/wuji_teleop_monitor"
LAUNCHER="$SCRIPT_DIR/scripts/launch_ui_docker.sh"

if [ ! -f "$LAUNCHER" ]; then
    echo "[ERROR] Launcher script not found: $LAUNCHER" >&2
    exit 1
fi
if [ ! -f "$PACKAGE_PATH/wuji.svg" ]; then
    echo "[WARN] Icon not found: $PACKAGE_PATH/wuji.svg (.desktop will fall back to a generic icon)" >&2
fi

chmod +x "$LAUNCHER"
# Keep the back-compat shim executable too, in case an older .desktop still points at it.
[ -f "$SCRIPT_DIR/scripts/launch_monitor_docker.sh" ] && chmod +x "$SCRIPT_DIR/scripts/launch_monitor_docker.sh"

install_shortcut() {
    local ui_name="$1"
    local template_name="$2"
    local output_name="$3"
    local template="$SCRIPT_DIR/$template_name"
    local output="$DESKTOP_DIR/$output_name"

    if [ ! -f "$template" ]; then
        echo "[ERROR] Template not found: $template" >&2
        return 1
    fi
    sed -e "s|{{HOME}}|$HOME|g" \
        -e "s|{{PACKAGE_PATH}}|$PACKAGE_PATH|g" \
        -e "s|{{LAUNCHER}}|$LAUNCHER $ui_name|g" \
        "$template" > "$output"
    chmod +x "$output"
    gio set "$output" metadata::trusted true 2>/dev/null || true
    echo "[OK] $output"
}

install_shortcut monitor teleop-monitor.desktop.template teleop-monitor.desktop
install_shortcut brake   brake-control.desktop.template  brake-control.desktop
install_shortcut camera  camera-preview.desktop.template camera-preview.desktop

echo
echo "Three shortcuts installed under $DESKTOP_DIR."
echo "Launcher:  $LAUNCHER"
echo "Icon:      $PACKAGE_PATH/wuji.svg"
echo
echo "Prerequisites for the shortcuts to work:"
echo "  - The wuji-hand-teleop container must be running:"
echo "      cd <repo>/docker && docker compose up -d"
echo "  - The host X server must be reachable from the container (the launcher"
echo "    runs 'xhost +local:docker' automatically on every click)."
