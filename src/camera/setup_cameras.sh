#!/bin/bash
# =============================================================================
# Teleoperation camera device setup script
# =============================================================================
# Functions:
#   1. Install udev rules → fix camera device symlinks
#   2. Scan currently connected cameras
#   3. Optional: update RealSense serial numbers
#
# Usage: bash setup_cameras.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_RULES="$SCRIPT_DIR/config/udev/99-teleop-cameras.rules"
DEST="/etc/udev/rules.d/99-teleop-cameras.rules"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "============================================================"
echo "  Teleoperation Camera Device Setup"
echo "============================================================"
echo ""

# --- 1. Scan current cameras ---
echo -e "${GREEN}[1/3] Scanning USB cameras...${NC}"
echo ""

# ZED series
if lsusb | grep -qi "stereolabs\|zed"; then
    echo -e "  ${GREEN}✓${NC} ZED camera connected:"
    lsusb | grep -i "stereolabs\|zed" | sed 's/^/    /'
else
    echo -e "  ${YELLOW}✗${NC} No ZED camera detected"
fi

# RealSense series
if command -v rs-enumerate-devices &>/dev/null; then
    RS_DEVICES=$(rs-enumerate-devices 2>/dev/null | grep -A1 "Device info" | grep "Serial Number" | awk '{print $NF}')
    if [ -n "$RS_DEVICES" ]; then
        echo -e "  ${GREEN}✓${NC} RealSense cameras:"
        rs-enumerate-devices 2>/dev/null | grep -E "Name|Serial Number" | sed 's/^/    /'
    else
        echo -e "  ${YELLOW}✗${NC} No RealSense camera detected"
    fi
else
    echo -e "  ${YELLOW}!${NC} rs-enumerate-devices not installed, skipping RealSense scan"
    echo "    Install: sudo apt install librealsense2-utils"
fi

# /dev/video* devices
echo ""
echo "  Current video devices:"
for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    name=$(cat /sys/class/video4linux/$(basename "$dev")/name 2>/dev/null || echo "unknown")
    echo "    $dev → $name"
done
[ ! -e /dev/video0 ] && echo "    (none)"

# --- 2. Install udev rules ---
echo ""
echo -e "${GREEN}[2/3] Installing udev rules...${NC}"

if [ ! -f "$UDEV_RULES" ]; then
    echo -e "  ${RED}Error: rules file not found: $UDEV_RULES${NC}"
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "  sudo permissions required to install udev rules"
    sudo cp "$UDEV_RULES" "$DEST"
else
    cp "$UDEV_RULES" "$DEST"
fi

sudo udevadm control --reload-rules
sudo udevadm trigger
echo -e "  ${GREEN}✓${NC} Rules installed to $DEST"

# --- 3. Verify symlinks ---
echo ""
echo -e "${GREEN}[3/3] Verifying device symlinks...${NC}"
sleep 1  # Wait for udev processing

check_link() {
    local link=$1
    local label=$2
    if [ -e "$link" ]; then
        local target=$(readlink -f "$link")
        echo -e "  ${GREEN}✓${NC} $link → $target  ($label)"
    else
        echo -e "  ${YELLOW}✗${NC} $link not created  ($label not connected)"
    fi
}

check_link "/dev/stereo_camera"    "Head ZED"
check_link "/dev/cam_left_wrist"   "Left wrist RealSense"
check_link "/dev/cam_right_wrist"  "Right wrist RealSense"

# --- Done ---
echo ""
echo "============================================================"
echo "  Setup complete!"
echo ""
echo "  Device mapping:"
echo "    /dev/stereo_camera   ← ZED Mini (head stereo)"
echo "    /dev/cam_left_wrist  ← RealSense D405/D435 (left wrist)"
echo "    /dev/cam_right_wrist ← RealSense D405/D435 (right wrist)"
echo ""
echo "  Docker mount example:"
echo "    docker run \\"
echo "      --device /dev/stereo_camera \\"
echo "      --device /dev/cam_left_wrist \\"
echo "      --device /dev/cam_right_wrist \\"
echo "      ..."
echo ""
echo "  Update serial number when replacing RealSense:"
echo "    1. rs-enumerate-devices | grep 'Serial Number'"
echo "    2. Edit config/udev/99-teleop-cameras.rules"
echo "    3. Re-run this script"
echo "============================================================"
