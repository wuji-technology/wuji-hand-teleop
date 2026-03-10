#!/bin/bash
set -e

WS="$HOME/ros2_ws"

# ============================================
# 1. Source ROS2 基础环境
# ============================================
source /opt/ros/humble/setup.bash

# ============================================
# 2. 检查源码挂载
# ============================================
if [ ! -d "$WS/src" ] || [ -z "$(ls -A "$WS/src" 2>/dev/null)" ]; then
    echo ""
    echo "[ERROR] src/ 目录为空或未挂载"
    echo "  请在 docker-compose.yml 中配置 volumes:"
    echo "    - ../src:/home/wuji/ros2_ws/src:rw"
    echo ""
    exec "$@"
fi

# ============================================
# 3. 首次构建 (install/ 不存在时自动执行)
# ============================================
if [ ! -f "$WS/install/setup.bash" ]; then
    echo ""
    echo "[INFO] 首次启动, 开始构建 ROS2 工作空间..."
    echo ""

    # wuji_hand_description URDF 冲突 (已由 pip 安装)
    find "$WS/src" -path "*/mujoco-sim/wuji_hand_description" -exec touch {}/COLCON_IGNORE \; 2>/dev/null || true

    # Git LFS 检测: .so 为指针文件时跳过 manus_ros2
    MANUS_SO="$WS/src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK.so"
    if [ -f "$MANUS_SO" ] && head -1 "$MANUS_SO" | grep -q "git-lfs"; then
        echo "[WARN] ManusSDK .so 是 Git LFS 指针文件, 跳过 manus_ros2 构建"
        echo "[WARN] 宿主机执行: git lfs install && git lfs pull"
        touch "$WS/src/input_devices/manus_input/COLCON_IGNORE"
    fi

    # 检测缺失外部依赖的包, 自动跳过
    IGNORE_PKGS=""
    # wujihand_driver 需要 wujihandcpp C++ SDK (deb 包, Dockerfile 已安装)
    if ! dpkg -l wujihandcpp >/dev/null 2>&1; then
        echo "[INFO] wujihandcpp 未安装, 跳过 wujihand_driver 构建"
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
        || echo "[WARN] 部分包构建失败, 核心功能不受影响"

    echo ""
    echo "[INFO] 构建完成"
fi

# ============================================
# 3b. 确保 wuji_teleop_bringup 已构建
# ============================================
# 主构建中 wujihand_driver 等失败时，colcon 可能跳过依赖链上的 bringup 包
if [ -f "$WS/install/setup.bash" ] && [ ! -d "$WS/install/wuji_teleop_bringup" ]; then
    echo ""
    echo "[INFO] wuji_teleop_bringup 缺失, 单独构建..."
    cd "$WS"
    colcon build --symlink-install --packages-select wuji_teleop_bringup \
        || echo "[WARN] wuji_teleop_bringup 构建失败"
    source "$WS/install/setup.bash" 2>/dev/null || true
fi

# ============================================
# 4. 共享库链接 (vendored .so → /usr/local/lib/)
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

# Tianji SDK
TIANJI_DIR="$WS/src/output_devices/tianji_output/tianji_output/_internal/lib"
if [ -d "$TIANJI_DIR" ] && [ ! -f /usr/local/lib/libMarvinSDK.so ]; then
    sudo cp "$TIANJI_DIR"/*.so /usr/local/lib/ 2>/dev/null || true
    NEED_LDCONFIG=true
fi

if $NEED_LDCONFIG; then
    sudo ldconfig
fi

# ROS2 + workspace library paths for sudo (ManusSDK needs root for USB dongle)
# sudo strips LD_LIBRARY_PATH, so we register all paths via ldconfig
if [ ! -f /etc/ld.so.conf.d/ros2_ws.conf ]; then
    {
        echo "/opt/ros/humble/lib"
        echo "/opt/ros/humble/lib/x86_64-linux-gnu"
        find "$WS/install" -name "lib" -maxdepth 3 -type d 2>/dev/null
    } | sudo tee /etc/ld.so.conf.d/ros2_ws.conf >/dev/null
    sudo ldconfig
fi

# ============================================
# 5. OpenVR 路径注册 (SteamVR 无头显模式)
# ============================================
# 宿主机 ~/.steam 挂载到容器 /home/wuji/.steam, 路径不同需重新生成
STEAMVR_RT="$HOME/.steam/debian-installation/steamapps/common/SteamVR"
if [ -d "$STEAMVR_RT" ]; then
    # 宿主机 .steam 以 ro 挂载, 但 vrserver 使用宿主机路径 (HOST_HOME)
    # 容器内需要: 1) 路径注册用宿主机路径  2) 符号链接保证路径可达
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
# 6. Source 工作空间
# ============================================
source "$WS/install/setup.bash" 2>/dev/null || true

# ============================================
# 7. 启动 XRoboToolkit PC-Service
# ============================================
if [ -f /opt/apps/roboticsservice/runService.sh ]; then
    (cd /opt/apps/roboticsservice && bash runService.sh) &
fi

# ============================================
# 8. ADB watchdog (PICO 有线连接自动管理)
# ============================================
# ADB reverse 是 USB session 级别的, 断连后丢失。
# watchdog 后台每 5s 检测, 自动恢复。HTC/WiFi 模式下空转无副作用。
# ADB restart + watchdog 全部后台运行, 不阻塞 entrypoint。
ADB_CONNECTED=false
if command -v adb &>/dev/null; then
    {
        sudo adb kill-server 2>/dev/null || true
        sudo adb start-server 2>/dev/null || true
        WATCHDOG="/entrypoint-scripts/adb_watchdog.sh"
        [ -f "$WATCHDOG" ] && exec bash "$WATCHDOG"
    } &

    # 检测当前状态 (仅用于启动信息展示, ADB 可能尚未就绪)
    sleep 0.1
    if adb devices 2>/dev/null | grep -q "device$"; then
        ADB_SERIAL=$(adb devices 2>/dev/null | grep "device$" | head -1 | cut -f1)
        ADB_CONNECTED=true
    fi
else
    echo "[WARN] ADB not installed, PICO wired mode unavailable"
fi

# ============================================
# 9. 启动信息
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
    echo "  [OK] ADB ($ADB_SERIAL, 有线模式)"
else
    echo "  [--] ADB (未连接)"
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
    echo "  [--] OpenVR (SteamVR 未挂载)"
fi

echo ""
echo "Launch:"
echo "  ros2 run wuji_teleop_monitor monitor          # Monitor GUI (推荐)"
echo "  ros2 launch wuji_teleop_bringup wuji_teleop.launch.py  # 手+臂"
echo "  ros2 launch wuji_teleop_bringup pico_teleop.launch.py  # PICO 方案"
echo "  ros2 launch camera camera_launch.py            # 相机"
echo "============================================"
echo ""

exec "$@"
