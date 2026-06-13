#!/bin/bash
# Wuji Teleop Monitor — desktop launcher (single entry)
#
# Usage:
#   bash launcher.sh                # menu prompt
#   bash launcher.sh teleop         # one-click teleop launcher UI
#   bash launcher.sh brake          # arm brake / recovery UI (standalone)
#   bash launcher.sh camera         # camera preview UI (standalone)

set -euo pipefail

# Source ROS2 + workspace before invoking the entry points.
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

APP="${1:-}"

if [ -z "$APP" ]; then
    echo "Select an app to launch:"
    echo "  1) teleop  - one-click launcher UI"
    echo "  2) brake   - arm brake / recovery UI"
    echo "  3) camera  - camera preview UI"
    read -rp "> " choice
    case "$choice" in
        1|teleop) APP="teleop" ;;
        2|brake)  APP="brake" ;;
        3|camera) APP="camera" ;;
        *) echo "Unknown choice: $choice" >&2; exit 1 ;;
    esac
fi

case "$APP" in
    teleop) exec ros2 run wuji_teleop_monitor monitor ;;
    brake)  exec ros2 run wuji_teleop_monitor brake ;;
    camera) exec ros2 run wuji_teleop_monitor camera ;;
    *) echo "Unknown app: $APP" >&2; exit 1 ;;
esac
