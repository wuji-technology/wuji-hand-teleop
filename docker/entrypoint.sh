#!/bin/bash
set -e

WS="$HOME/ros2_ws"

# ============================================
# 1. Source ROS2 base environment
# ============================================
source /opt/ros/humble/setup.bash

# ============================================
# 2. Check source code mount
# ============================================
if [ ! -d "$WS/src" ] || [ -z "$(ls -A "$WS/src" 2>/dev/null)" ]; then
    echo ""
    echo "[ERROR] src/ directory is empty or not mounted"
    echo "  Please configure volumes in docker-compose.yml:"
    echo "    - ../src:/home/wuji/ros2_ws/src:rw"
    echo ""
    exec "$@"
fi

# ============================================
# 3. First-time build (auto-execute when install/ does not exist)
# ============================================
if [ ! -f "$WS/install/setup.bash" ]; then
    echo ""
    echo "[INFO] First startup, building ROS2 workspace..."
    echo ""

    # wuji_hand_description URDF conflict (already installed via pip)
    find "$WS/src" -path "*/mujoco-sim/wuji_hand_description" -exec touch {}/COLCON_IGNORE \; 2>/dev/null || true

    # Git LFS detection: skip manus_ros2 when .so is a pointer file
    MANUS_SO="$WS/src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK.so"
    if [ -f "$MANUS_SO" ] && head -1 "$MANUS_SO" | grep -q "git-lfs"; then
        echo "[WARN] ManusSDK .so is a Git LFS pointer file, skipping manus_ros2 build"
        echo "[WARN] Run on host: git lfs install && git lfs pull"
        touch "$WS/src/input_devices/manus_input/COLCON_IGNORE"
    fi

    # Detect packages with missing external dependencies, auto-skip
    IGNORE_PKGS=""
    # wujihand_driver requires wujihandcpp C++ SDK (deb package, installed in Dockerfile)
    if ! dpkg -l wujihandcpp >/dev/null 2>&1; then
        echo "[INFO] wujihandcpp not installed, skipping wujihand_driver build"
        IGNORE_PKGS="$IGNORE_PKGS --packages-ignore wujihand_driver"
    fi

    # rosdep + colcon build
    (rosdep update || true)
    sudo apt-get update
    (rosdep install --from-paths "$WS/src" --ignore-src -r -y || true)
    sudo rm -rf /var/lib/apt/lists/*

    cd "$WS"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        $IGNORE_PKGS \
        || echo "[WARN] Some packages failed to build, core functionality is not affected"

    echo ""
    echo "[INFO] Build complete"
fi

# ============================================
# 3b. Ensure wuji_teleop_bringup is built
# ============================================
# When wujihand_driver etc. fail in the main build, colcon may skip bringup packages in the dependency chain
if [ -f "$WS/install/setup.bash" ] && [ ! -d "$WS/install/wuji_teleop_bringup" ]; then
    echo ""
    echo "[INFO] wuji_teleop_bringup missing, building separately..."
    cd "$WS"
    colcon build --symlink-install --packages-select wuji_teleop_bringup \
        || echo "[WARN] wuji_teleop_bringup build failed"
    source "$WS/install/setup.bash" 2>/dev/null || true
fi

# ============================================
# 4. Shared library linking (vendored .so → /usr/local/lib/)
# ============================================
NEED_LDCONFIG=false

# PICO SDK
PICO_SO="$WS/src/input_devices/pico_input/prebuilt/x86_64/libPXREARobotSDK.so"
if [ -f "$PICO_SO" ] && [ ! -f /usr/local/lib/libPXREARobotSDK.so ]; then
    sudo cp "$PICO_SO" /usr/local/lib/
    NEED_LDCONFIG=true
fi

# Manus SDK
MANUS_DIR="$WS/src/input_devices/manus_input/manus_ros2/ManusSDK/lib"
if [ -d "$MANUS_DIR" ] && [ ! -f /usr/local/lib/libManusSDK.so ]; then
    sudo cp "$MANUS_DIR"/*.so /usr/local/lib/ 2>/dev/null || true
    NEED_LDCONFIG=true
fi

# Manus USB dongle udev rule (allows non-root access to vendor 3325)
MANUS_UDEV="$WS/src/input_devices/manus_input/config/udev/99-manus-libusb.rules"
if [ -f "$MANUS_UDEV" ] && { [ ! -f /etc/udev/rules.d/99-manus-libusb.rules ] || ! cmp -s "$MANUS_UDEV" /etc/udev/rules.d/99-manus-libusb.rules; }; then
    sudo cp "$MANUS_UDEV" /etc/udev/rules.d/
    sudo udevadm control --reload-rules 2>/dev/null || true
    sudo udevadm trigger 2>/dev/null || true
fi

# Tianji SDK
TIANJI_DIR="$WS/src/output_devices/tianji_output/tianji_output/_internal/lib"
if [ -d "$TIANJI_DIR" ] && [ ! -f /usr/local/lib/libMarvinSDK.so ]; then
    sudo cp "$TIANJI_DIR"/*.so /usr/local/lib/ 2>/dev/null || true
    NEED_LDCONFIG=true
fi

if $NEED_LDCONFIG; then
    sudo ldconfig
fi

# ROS2 + workspace library paths (registered via ldconfig for all users)
# Ensures ManusSDK and other shared libraries are found without LD_LIBRARY_PATH
if [ ! -f /etc/ld.so.conf.d/ros2_ws.conf ]; then
    {
        echo "/opt/ros/humble/lib"
        echo "/opt/ros/humble/lib/x86_64-linux-gnu"
        find "$WS/install" -name "lib" -maxdepth 3 -type d 2>/dev/null
    } | sudo tee /etc/ld.so.conf.d/ros2_ws.conf >/dev/null
    sudo ldconfig
fi

# ============================================
# 5. OpenVR path registration (SteamVR null driver mode)
# ============================================
# Host ~/.steam mounted to container /home/wuji/.steam, paths differ so regeneration is needed
STEAMVR_RT="$HOME/.steam/debian-installation/steamapps/common/SteamVR"
if [ -d "$STEAMVR_RT" ]; then
    # Host .steam mounted as ro, but vrserver uses host paths (HOST_HOME)
    # Container needs: 1) path registration using host paths  2) symlinks to ensure paths are reachable
    HOST_HOME="${HOST_HOME:-$HOME}"
    if [ "$HOST_HOME" != "$HOME" ] && [ ! -e "$HOST_HOME" ]; then
        sudo ln -sf "$HOME" "$HOST_HOME"
    fi

    mkdir -p "$HOME/.config/openvr" "$HOME/.openvr-logs"
    cat > "$HOME/.config/openvr/openvrpaths.vrpath" <<EOVR
{
    "config": ["${HOST_HOME}/.steam/debian-installation/config"],
    "external_drivers": null,
    "jsonid": "vrpathreg",
    "log": ["$HOME/.openvr-logs"],
    "runtime": ["${HOST_HOME}/.steam/debian-installation/steamapps/common/SteamVR"],
    "version": 1
}
EOVR
    echo "[INFO] OpenVR paths registered (SteamVR null driver)"
fi

# ============================================
# 6. Source workspace
# ============================================
source "$WS/install/setup.bash" 2>/dev/null || true

# ============================================
# 7. Start XRoboToolkit PC-Service
# ============================================
if [ -f /opt/apps/roboticsservice/runService.sh ]; then
    (cd /opt/apps/roboticsservice && bash runService.sh) &
fi

# ============================================
# 8. ADB watchdog (PICO wired connection auto-management)
# ============================================
# ADB reverse is USB session-level, lost after disconnection.
# Watchdog runs in background every 5s to detect and auto-recover. Idle with no side effects in HTC/WiFi mode.
# ADB restart + watchdog all run in background, non-blocking for entrypoint.
ADB_CONNECTED=false
if command -v adb &>/dev/null; then
    {
        sudo adb kill-server 2>/dev/null || true
        sudo adb start-server 2>/dev/null || true
        WATCHDOG="/entrypoint-scripts/adb_watchdog.sh"
        [ -f "$WATCHDOG" ] && exec bash "$WATCHDOG"
    } &

    # Check current status (only for startup info display, ADB may not be ready yet)
    sleep 0.1
    if adb devices 2>/dev/null | grep -q "device$"; then
        ADB_SERIAL=$(adb devices 2>/dev/null | grep "device$" | head -1 | cut -f1)
        ADB_CONNECTED=true
    fi
else
    echo "[WARN] ADB not installed, PICO wired mode unavailable"
fi

# ============================================
# 9. Startup information
# ============================================
echo ""
echo "============================================"
echo "  Wuji Teleop ROS2 Docker"
echo "============================================"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID:-0}"
echo "  RMW:            ${RMW_IMPLEMENTATION}"
echo "============================================"
echo ""
echo "SDK Status:"

# PICO
python3 -c "import xrobotoolkit_sdk; print('  [OK] PICO SDK')" 2>/dev/null \
    || echo "  [--] PICO SDK"
[ -f /opt/apps/roboticsservice/runService.sh ] \
    && echo "  [OK] PICO PC-Service" \
    || echo "  [--] PICO PC-Service"
if [ "$ADB_CONNECTED" = "true" ]; then
    echo "  [OK] ADB ($ADB_SERIAL, wired mode)"
else
    echo "  [--] ADB (not connected)"
fi

# Manus
[ -f /usr/local/lib/libManusSDK.so ] \
    && echo "  [OK] Manus SDK" \
    || echo "  [--] Manus SDK"

# Tianji
[ -f /usr/local/lib/libMarvinSDK.so ] \
    && echo "  [OK] Tianji SDK" \
    || echo "  [--] Tianji SDK"

# WujiHand
dpkg -l wujihandcpp >/dev/null 2>&1 \
    && echo "  [OK] WujiHand SDK" \
    || echo "  [--] WujiHand SDK"

# RealSense
dpkg -l ros-humble-realsense2-camera >/dev/null 2>&1 \
    && echo "  [OK] RealSense Driver" \
    || echo "  [--] RealSense Driver"

# OpenVR / SteamVR
if [ -f "$HOME/.config/openvr/openvrpaths.vrpath" ]; then
    echo "  [OK] OpenVR (SteamVR null driver)"
else
    echo "  [--] OpenVR (SteamVR not mounted)"
fi

echo ""
echo "Launch:"
echo "  ros2 run wuji_teleop_monitor monitor          # Monitor GUI (recommended)"
echo "  ros2 launch wuji_teleop_bringup wuji_teleop.launch.py  # Hand + Arm"
echo "  ros2 launch wuji_teleop_bringup pico_teleop.launch.py  # PICO solution"
echo "  ros2 launch camera camera_launch.py            # Camera"
echo "============================================"
echo ""

exec "$@"
