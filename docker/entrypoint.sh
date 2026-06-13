#!/bin/bash
set -e

WS="$HOME/ros2_ws"

# Detect a Git LFS pointer file (~130-byte text stub instead of the real binary).
# Hosts that cloned without `git lfs pull` end up with these in place of the
# real .so files — copying them to /usr/local/lib/ produces a green-looking
# ldconfig but a runtime ctypes/ld crash with no useful message.
is_lfs_pointer() {
    [ -f "$1" ] && head -1 "$1" 2>/dev/null | grep -q "git-lfs"
}

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
# ============================================
# 2b. Seed per-machine config yamls from their .template siblings if missing
# ============================================
# Tracked `*.yaml.template` files carry placeholders (YOUR_LEFT_HAND_SERIAL,
# etc.); the live `<file>.yaml` is gitignored and holds real SNs / IPs.
# Auto-cp only when `<file>.yaml` is absent so we never clobber operator edits.
# Runs every container start (cheap + idempotent) — picks up newly added
# templates after `git pull` without a full rebuild.
find "$WS/src" \( -name '*.yaml.template' -o -name '*.rules.template' \) -type f 2>/dev/null | while read -r tpl; do
    live="${tpl%.template}"
    if [ ! -e "$live" ]; then
        cp "$tpl" "$live"
        echo "[INFO] Seeded $(basename "$live") from .template (edit with real SNs before launch)"
    fi
done

if [ ! -f "$WS/install/setup.bash" ]; then
    echo ""
    echo "[INFO] First startup, building ROS2 workspace..."
    echo ""

    # wuji-description URDF conflict: wujihandros2 provides wuji_description as a ROS2 package;
    # wuji-retargeting bundles its own copy via pip. Mark legacy/duplicate paths to skip colcon build.
    find "$WS/src" -path "*/mujoco-sim/wuji_hand_description" -exec touch {}/COLCON_IGNORE \; 2>/dev/null || true
    find "$WS/src" -path "*/wuji_retargeting/wuji-description" -exec touch {}/COLCON_IGNORE \; 2>/dev/null || true

    # Git LFS detection: skip manus_ros2 when .so is a pointer file
    MANUS_SO="$WS/src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK.so"
    if is_lfs_pointer "$MANUS_SO"; then
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
if is_lfs_pointer "$PICO_SO"; then
    echo "[WARN] libPXREARobotSDK.so is a Git LFS pointer; skipping cp. Run on host: git lfs install && git lfs pull"
elif [ -f "$PICO_SO" ] && [ ! -f /usr/local/lib/libPXREARobotSDK.so ]; then
    sudo cp "$PICO_SO" /usr/local/lib/
    NEED_LDCONFIG=true
fi

# Manus SDK
MANUS_DIR="$WS/src/input_devices/manus_input/manus_ros2/ManusSDK/lib"
if [ -d "$MANUS_DIR" ] && [ ! -f /usr/local/lib/libManusSDK.so ]; then
    # Skip any .so under MANUS_DIR that is an LFS pointer — copying a 130-byte
    # stub would silently fool the startup check below.
    for so in "$MANUS_DIR"/*.so; do
        [ -f "$so" ] || continue
        is_lfs_pointer "$so" && continue
        sudo cp "$so" /usr/local/lib/
        NEED_LDCONFIG=true
    done
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
    for so in "$TIANJI_DIR"/*.so; do
        [ -f "$so" ] || continue
        is_lfs_pointer "$so" && continue
        sudo cp "$so" /usr/local/lib/
        NEED_LDCONFIG=true
    done
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
# 4b. Wuji Studio calibration mount (~/.wuji)
# ============================================
# docker auto-creates ${HOME}/.wuji on the host as root if it doesn't exist
# before `docker compose up`; that breaks SDK writes from inside the container.
# Reclaim ownership so the wuji user inside can read/write calibration files.
if [ ! -w "$HOME/.wuji" ]; then
    sudo mkdir -p "$HOME/.wuji/sdk/params"
    sudo chown -R wuji:wuji "$HOME/.wuji"
fi

# ============================================
# 4c. Wuji Glove multi-NIC route pinning
# ============================================
# Harnesses that wire each glove receiver to its own NIC on the same subnet hit
# a routing quirk: the kernel sends unicast to a glove out one (default) NIC,
# which may not be the NIC the glove sits behind, so the SDK times out on
# connect even though it can discover the glove over broadcast. Pin a /32 route
# to the NIC that actually reaches each glove. No-op on single-NIC setups; needs
# cap_add: NET_ADMIN (set in docker-compose.yml). Re-runnable after powering the
# gloves on: bash /entrypoint-scripts/setup_glove_routes.sh
if [ -f /entrypoint-scripts/setup_glove_routes.sh ]; then
    bash /entrypoint-scripts/setup_glove_routes.sh || true
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
echo "  Wuji Hand Teleop ROS2 Docker"
echo "============================================"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID:-0}"
echo "  RMW:            ${RMW_IMPLEMENTATION}"
echo "============================================"
echo ""
echo "Required SDKs (hand pipeline — container exits if any fails):"

# Hard-required for the hand-only main flow (the README default). A regression
# here is the silent-failure mode QA hit on bbb8b8f: container boots, teleop
# starts, glove->hand just doesn't track. Hard-fail at entrypoint instead —
# stderr is intentionally NOT redirected so the operator sees the real
# ImportError / OSError, not a sanitized "[--]" line.

# WujiHand SDK — required by wujihand_driver to talk to the hand hardware.
if dpkg -l wujihandcpp >/dev/null 2>&1; then
    echo "  [OK] WujiHand SDK"
else
    echo "  [FAIL] WujiHand SDK (wujihandcpp .deb not installed; rebuild image)" >&2
    exit 1
fi

# In-repo bundled kinematics .so. fx_kine.py loads via ctypes from this exact
# path; the bug fixed in beddb9c was an off-by-one .gitignore whitelist.
_KINE_SO=/home/wuji/ros2_ws/src/output_devices/tianji_output/tianji_output/_internal/lib/libKine.so
if [ ! -f "$_KINE_SO" ]; then
    echo "  [FAIL] Tianji kine — libKine.so missing at $_KINE_SO" >&2
    echo "         Check .gitignore whitelist (must keep _internal/lib/*.so)" >&2
    echo "         and run \`git lfs pull\` on the host." >&2
    exit 1
elif is_lfs_pointer "$_KINE_SO"; then
    echo "  [FAIL] Tianji kine — libKine.so is a Git LFS pointer at $_KINE_SO" >&2
    echo "         Run on the host: git lfs install && git lfs pull" >&2
    exit 1
else
    echo "  [OK] Tianji kine (libKine.so tracked in repo)"
fi

# Tianji SDK — Marvin protocol shared library, loaded by the controller.
if [ -f /usr/local/lib/libMarvinSDK.so ]; then
    echo "  [OK] Tianji SDK (libMarvinSDK.so)"
else
    echo "  [FAIL] Tianji SDK — /usr/local/lib/libMarvinSDK.so missing" >&2
    exit 1
fi

# pinocchio — hand IK depends on it via wuji_retargeting. The 3.9.0->4.0.0 bump
# in Dockerfile §5 fixes the urdfdom soname mismatch. Let Python print the
# real ImportError on failure (no `2>/dev/null`).
echo -n "  [..] pinocchio (urdfdom soname + NumPy 2 ABI) ... "
if python3 -c "import pinocchio" 2>&1; then
    echo "OK"
else
    echo ""
    echo "  [FAIL] pinocchio import failed — see traceback above" >&2
    echo "         Likely Dockerfile §5 \`pin==\` got regressed; HEAD pins 4.0.0." >&2
    exit 1
fi

# wuji_retargeting — Wuji's hand-pose retargeting algorithm. If pinocchio just
# passed, this almost always passes too — but a missing submodule init would
# slip past the pinocchio check.
echo -n "  [..] wuji_retargeting (hand IK) ... "
if python3 -c "import wuji_retargeting" 2>&1; then
    echo "OK"
else
    echo ""
    echo "  [FAIL] wuji_retargeting import failed — see traceback above" >&2
    echo "         Did \`git submodule update --init --recursive\` run on the host" >&2
    echo "         before \`docker compose build\`? See Dockerfile §6." >&2
    exit 1
fi

echo ""
echo "Optional SDKs (per-device — missing rows are fine if the device isn't used):"

# Below: optional inputs / sensors. Operator may run hand-only without these
# and not care that they're [--]. Dashboard format intentional. stderr passes
# through so a *failed import* (different from "not installed") is still loud.

# PICO (arm input alternative)
if python3 -c "import xrobotoolkit_sdk" 2>/dev/null; then
    echo "  [OK] PICO SDK"
else
    echo "  [--] PICO SDK (only needed on the PICO arm path)"
fi
if [ -f /opt/apps/roboticsservice/runService.sh ]; then
    echo "  [OK] PICO PC-Service"
else
    echo "  [--] PICO PC-Service (only needed on the PICO arm path)"
fi
if [ "$ADB_CONNECTED" = "true" ]; then
    echo "  [OK] ADB ($ADB_SERIAL, wired mode)"
else
    echo "  [--] ADB (no PICO headset connected over USB)"
fi

# Manus (community-supported hand input alternative)
if [ -f /usr/local/lib/libManusSDK.so ]; then
    echo "  [OK] Manus SDK"
else
    echo "  [--] Manus SDK (only needed for MANUS Glove path)"
fi

# RealSense (wrist cameras)
if dpkg -l ros-humble-realsense2-camera >/dev/null 2>&1; then
    echo "  [OK] RealSense Driver"
else
    echo "  [--] RealSense Driver (only needed for D405 wrist cameras)"
fi

# OpenVR / SteamVR (HTC Vive Tracker arm path)
if [ -f "$HOME/.config/openvr/openvrpaths.vrpath" ]; then
    echo "  [OK] OpenVR (SteamVR null driver)"
else
    echo "  [--] OpenVR (only needed for HTC Vive Tracker arm path)"
fi

echo ""
echo "Launch:"
echo "  ros2 run wuji_teleop_monitor monitor                      # Monitor GUI — one-click hand teleop (recommended)"
echo "  ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py  # CLI alternative: dual Wuji Hands"
echo "  ros2 launch camera camera_launch.py                       # Cameras (head stereo + wrist D405)"
echo ""
echo "  # Arm teleop — see docs/STEAMVR.md (HTC) or docs/PICO.md (PICO):"
echo "  #   ros2 launch wuji_teleop_bringup wuji_teleop.launch.py arm_input:=tracker"
echo "  #   ros2 launch wuji_teleop_bringup pico_teleop.launch.py"
echo "============================================"
echo ""

exec "$@"
