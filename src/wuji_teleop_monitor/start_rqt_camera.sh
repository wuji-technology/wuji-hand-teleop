#!/bin/bash
# ============================================================================
# One-Click Launch rqt Camera Monitor
# ============================================================================
#
# Description:
#   Automatically loads 3 camera views in a single rqt window
#   - Stereo camera left eye
#   - Left wrist camera
#   - Right wrist camera
#
# Usage:
#   ./start_rqt_camera.sh
#
# After launch:
#   - Three camera feeds will be displayed automatically
#   - You can drag to adjust the layout
#   - Press Ctrl+C or close the window to exit
#
# ============================================================================

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Infer workspace path from script location (3 levels up: wuji_teleop_monitor -> src -> wuji-hand-teleop -> ros2_ws)
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Source ROS 2 base environment
source /opt/ros/humble/setup.bash

# Source workspace (if it exists)
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo "Workspace loaded: ${WS_DIR}"
else
    echo "Warning: Workspace not found at ${WS_DIR}/install/setup.bash"
fi

echo "Starting camera monitor..."
echo ""
echo "Camera topics:"
echo "  - /stereo/left/compressed (stereo camera left eye)"
echo "  - /cam_left_wrist/color/image_rect_raw/compressed (left wrist)"
echo "  - /cam_right_wrist/color/image_rect_raw/compressed (right wrist)"
echo ""
echo "Layout: stereo camera on top, left/right wrist cameras on bottom"
echo "Press 'q' or ESC to exit"
echo ""

# Launch Python camera viewer
python3 "${SCRIPT_DIR}/camera_viewer.py"

