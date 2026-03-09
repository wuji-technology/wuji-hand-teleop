#!/bin/bash
# =============================================================================
# 遥操作相机设备配置脚本
# =============================================================================
# 功能:
#   1. 安装 udev 规则 → 固定相机设备符号链接
#   2. 扫描当前连接的相机
#   3. 可选: 更新 RealSense 序列号
#
# 使用: bash setup_cameras.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_RULES="$SCRIPT_DIR/config/udev/99-teleop-cameras.rules"
DEST="/etc/udev/rules.d/99-teleop-cameras.rules"

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "============================================================"
echo "  遥操作相机设备配置"
echo "============================================================"
echo ""

# --- 1. 扫描当前相机 ---
echo -e "${GREEN}[1/3] 扫描 USB 相机...${NC}"
echo ""

# ZED 系列
if lsusb | grep -qi "stereolabs\|zed"; then
    echo -e "  ${GREEN}✓${NC} ZED 相机已连接:"
    lsusb | grep -i "stereolabs\|zed" | sed 's/^/    /'
else
    echo -e "  ${YELLOW}✗${NC} 未检测到 ZED 相机"
fi

# RealSense 系列
if command -v rs-enumerate-devices &>/dev/null; then
    RS_DEVICES=$(rs-enumerate-devices 2>/dev/null | grep -A1 "Device info" | grep "Serial Number" | awk '{print $NF}')
    if [ -n "$RS_DEVICES" ]; then
        echo -e "  ${GREEN}✓${NC} RealSense 相机:"
        rs-enumerate-devices 2>/dev/null | grep -E "Name|Serial Number" | sed 's/^/    /'
    else
        echo -e "  ${YELLOW}✗${NC} 未检测到 RealSense 相机"
    fi
else
    echo -e "  ${YELLOW}!${NC} rs-enumerate-devices 未安装，跳过 RealSense 扫描"
    echo "    安装: sudo apt install librealsense2-utils"
fi

# /dev/video* 设备
echo ""
echo "  当前视频设备:"
for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    name=$(cat /sys/class/video4linux/$(basename $dev)/name 2>/dev/null || echo "unknown")
    echo "    $dev → $name"
done
[ ! -e /dev/video0 ] && echo "    (无)"

# --- 2. 安装 udev 规则 ---
echo ""
echo -e "${GREEN}[2/3] 安装 udev 规则...${NC}"

if [ ! -f "$UDEV_RULES" ]; then
    echo -e "  ${RED}错误: 规则文件不存在: $UDEV_RULES${NC}"
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "  需要 sudo 权限安装 udev 规则"
    sudo cp "$UDEV_RULES" "$DEST"
else
    cp "$UDEV_RULES" "$DEST"
fi

sudo udevadm control --reload-rules
sudo udevadm trigger
echo -e "  ${GREEN}✓${NC} 规则已安装到 $DEST"

# --- 3. 验证符号链接 ---
echo ""
echo -e "${GREEN}[3/3] 验证设备符号链接...${NC}"
sleep 1  # 等待 udev 处理

check_link() {
    local link=$1
    local label=$2
    if [ -e "$link" ]; then
        local target=$(readlink -f "$link")
        echo -e "  ${GREEN}✓${NC} $link → $target  ($label)"
    else
        echo -e "  ${YELLOW}✗${NC} $link 未创建  ($label 未连接)"
    fi
}

check_link "/dev/stereo_camera"    "头部 ZED"
check_link "/dev/cam_left_wrist"   "左腕 RealSense"
check_link "/dev/cam_right_wrist"  "右腕 RealSense"

# --- 完成 ---
echo ""
echo "============================================================"
echo "  配置完成!"
echo ""
echo "  设备映射:"
echo "    /dev/stereo_camera   ← ZED Mini (头部双目)"
echo "    /dev/cam_left_wrist  ← RealSense D405/D435 (左腕)"
echo "    /dev/cam_right_wrist ← RealSense D405/D435 (右腕)"
echo ""
echo "  Docker 挂载示例:"
echo "    docker run \\"
echo "      --device /dev/stereo_camera \\"
echo "      --device /dev/cam_left_wrist \\"
echo "      --device /dev/cam_right_wrist \\"
echo "      ..."
echo ""
echo "  更换 RealSense 时修改序列号:"
echo "    1. rs-enumerate-devices | grep 'Serial Number'"
echo "    2. 编辑 config/udev/99-teleop-cameras.rules"
echo "    3. 重新运行本脚本"
echo "============================================================"
