# MANUS Input

ROS2 input packages for MANUS gloves. This directory contains the MANUS C++ driver, the Python keypoint bridge, calibration assets, and the USB udev rule used by the public teleoperation workflow.

## Package Layout

```text
src/input_devices/manus_input/
|- config/udev/99-manus-libusb.rules        # USB access rule for the MANUS dongle
|- manus_ros2/                              # C++ driver, calibration files, helper scripts
|- manus_ros2_msgs/                         # ROS2 message definitions
`- manus_input_py/                          # Python bridge to /hand_input
```

## Quick Start - MANUS gloves controlling WG110

Use this when you want the MANUS-specific setup in one place. The output hand in the public repo is the WG110 first-generation Wuji Hand.

```bash
# 1. Clone
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/wuji-technology/wuji-hand-teleop.git
cd wuji-hand-teleop
git lfs install && git lfs pull

# 2. Install dependencies
sudo apt install ros-humble-desktop ros-humble-ament-cmake ros-humble-rclpy ros-humble-std-msgs ros-humble-tf2-ros libncurses-dev python3-pip
python3 -m pip install numpy scipy pyyaml PyQt5 openvr
wget https://github.com/wuji-technology/wujihandpy/releases/download/v1.5.1/wujihandcpp-1.5.1-amd64.deb
sudo apt install ./wujihandcpp-1.5.1-amd64.deb
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/wuji-technology/wuji-retargeting.git
cd wuji-retargeting && python3 -m pip install .
touch COLCON_IGNORE    # Prevent "Duplicate package names" with wujihandros2
cd ~/ros2_ws/src/wuji-hand-teleop

# 3. Install the MANUS USB dongle udev rule
sudo cp src/input_devices/manus_input/config/udev/99-manus-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. Copy MANUS calibration files exported from MANUS Core
cp /path/to/left_calibration.mcal  src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal
cp /path/to/right_calibration.mcal src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal

# 5. Configure glove IDs
#    Edit: src/input_devices/manus_input/manus_input_py/manus_input_py/config/manus_input.yaml
#    include_right_hand: true
#    include_left_hand:  true
#    left_glove_id:  0
#    right_glove_id: 1

# 6. Configure WG110 hand serial numbers
#    Find your serials:
#      lsusb -v -d 0483:2000 | grep iSerial
#    Then edit:
#      src/output_devices/wujihand_output/config/wujihand_ik.yaml
#      left_hand:  serial_number: "YOUR_LEFT_HAND_SERIAL"   (or null to disable)
#      right_hand: serial_number: "YOUR_RIGHT_HAND_SERIAL"  (or null to disable)

# 7. Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
# Expected: "Summary: 18 packages finished [xx.xs]" with 0 failed

# 8. Launch
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
# Expected: "Calibration loaded successfully for Left/Right glove"
#           "Publishing hand data on '/hand_input' at 120.0 Hz"
# Verify:   ros2 topic hz /hand_input  (should show ~50-120 Hz)
```

> **Using Docker?** See the [Docker Setup Guide](../../../docker/README.md).

## Calibration

Calibration is required per user.

1. Download **MANUS Core** on a Windows PC.
2. Connect and calibrate both gloves in MANUS Core.
3. Export left and right `.mcal` files.
4. Copy them into `manus_ros2/calibration/` using the filenames above.

The detailed calibration note from the package is here: [CALIBRATION_GUIDE.md](manus_ros2/CALIBRATION_GUIDE.md).

## Glove ID Configuration

Edit `manus_input_py/manus_input_py/config/manus_input.yaml`:

```yaml
include_right_hand: true
include_left_hand: true
left_glove_id: 0
right_glove_id: 1
```

If left and right appear swapped, adjust `left_glove_id` / `right_glove_id` and relaunch.

## Verify and Troubleshooting

```bash
# Confirm the dongle is visible
lsusb -d 3325:

# Inspect the MANUS keypoint topic (BEST_EFFORT QoS)
ros2 topic echo /hand_input --qos-reliability best_effort

# Check the publish rate
ros2 topic hz /hand_input
```

Common issues:

- `LIBUSB_ERROR_ACCESS` or no glove data: reinstall the udev rule, then replug the dongle or reboot.
- `manus_ros2` link error: run `git lfs pull` so the SDK `.so` files are real binaries instead of LFS placeholders.
- `Duplicate package names`: ensure `COLCON_IGNORE` exists in the `wuji-retargeting` checkout.
