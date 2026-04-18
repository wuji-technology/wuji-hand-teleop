#!/bin/bash

# =============================================================================
# Teleop Monitor Desktop Shortcut Installation Script
# =============================================================================
#
# Usage:
#   cd ~/ros2_ws/src/wuji-hand-teleop-ros2/src/wuji_teleop_monitor
#   ./install_desktop.sh
#
# Features:
#   - Creates a Teleop Monitor shortcut on the user's desktop
#   - Auto-detects desktop path (supports both Chinese and English desktop names)
#   - Auto-configures correct paths and permissions
#
# =============================================================================

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Get user desktop path
DESKTOP_DIR=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")

# Check if desktop directory exists
if [ ! -d "$DESKTOP_DIR" ]; then
    echo "Error: Desktop directory does not exist: $DESKTOP_DIR"
    exit 1
fi

# Package Python module path (where logo is located)
PACKAGE_PATH="$SCRIPT_DIR/wuji_teleop_monitor"

# Check if logo exists
if [ ! -f "$PACKAGE_PATH/wuji.jpg" ]; then
    echo "Warning: Logo file not found: $PACKAGE_PATH/wuji.jpg"
fi

# Check template file
TEMPLATE_FILE="$SCRIPT_DIR/teleop-monitor.desktop.template"
if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "Error: Template file not found: $TEMPLATE_FILE"
    exit 1
fi

# Generate .desktop file
OUTPUT_FILE="$DESKTOP_DIR/teleop-monitor.desktop"

sed -e "s|{{HOME}}|$HOME|g" \
    -e "s|{{PACKAGE_PATH}}|$PACKAGE_PATH|g" \
    "$TEMPLATE_FILE" > "$OUTPUT_FILE"

# Add execute permission
chmod +x "$OUTPUT_FILE"

# Mark as trusted (required for GNOME desktop)
gio set "$OUTPUT_FILE" metadata::trusted true 2>/dev/null || true

echo "Desktop shortcut installed to: $OUTPUT_FILE"
echo "Done!"
