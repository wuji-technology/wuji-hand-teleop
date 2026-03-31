# wuji-hand-teleop

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)  [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-hand-teleop)](https://github.com/wuji-technology/wuji-hand-teleop/releases)

ROS2-based teleoperation system for Wuji Hand and Tianji Arm. Supports multiple input devices including MANUS Gloves, HTC Vive Trackers, PICO VR, and custom devices through a standardized topic interface. Features a Monitor GUI for one-click launch and real-time device monitoring.

> [!WARNING]
> This project is **not actively maintained** and **no after-sales support** is provided. If you encounter any issues, please [open an issue](https://github.com/wuji-technology/wuji-hand-teleop/issues) — but responses are not guaranteed. **Product version coming soon.**

**Get started with [Quick Start](#quick-start). For detailed documentation, please refer to [Teleop User Guide](https://docs.wuji.technology/docs/en/wuji-hand/latest/teleop-user-guide/introduction/) on Wuji Docs Center.**

## Repository Structure

```text
├── src/
│   ├── wuji_teleop_bringup/       // Launch files for various teleoperation modes
│   ├── wuji_teleop_monitor/       // Monitor GUI for device monitoring and one-click launch
│   ├── controller/                // Controller nodes for Wuji Hand and Tianji Arm
│   ├── input_devices/
│   │   ├── common_input/          // Base interface
│   │   ├── manus_input/           // MANUS Glove driver (C++ SDK + Python wrapper)
│   │   ├── openvr_input/          // HTC Vive Tracker (SteamVR headless)
│   │   └── pico_input/            // PICO VR integration
│   ├── output_devices/
│   │   ├── wujihand_output/       // Wuji Hand retargeting + IK controller
│   │   ├── tianji_output/         // Tianji Arm IK controller
│   │   └── tianji_world_output/   // Tianji Arm world-frame controller
│   ├── camera/                    // RealSense + StereoVR integration
│   ├── wujihand_urdf/             // URDF models for RViz visualization
│   └── wujihandros2/              // Submodule: Wuji Hand C++ ROS2 driver
├── docs/                          // Documentation and technical reports
├── docker/                        // Docker container setup
└── README.md
```

## Quick Start

### Installation

```bash
# Install ROS2 dependencies
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ament-cmake ros-humble-rclpy ros-humble-std-msgs ros-humble-tf2-ros
pip install numpy scipy pyyaml PyQt5 openvr

# Install hand retargeting algorithm
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/wuji-technology/wuji-retargeting.git
cd wuji-retargeting && pip install -e .

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running

```bash
# Full teleoperation (Manus hand + Tracker arm)
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker

# With RViz visualization
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker enable_rviz:=true

# Hand-only control
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus

# Arm-only control
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker

# Monitor GUI (recommended)
ros2 run wuji_teleop_monitor monitor
```

## Citation

If you find our project useful, please cite it as follows:

```bibtex
@software{wuji2025handteleop,
  title   = {Wuji Hand Teleop: ROS2 Teleoperation for Dexterous Hands and Robot Arms},
  author  = {Guanqi He and Wentao Zhang and Liang Zhu},
  year    = {2025},
  url     = {https://github.com/wuji-technology/wuji-hand-teleop}
}
```

## Appendix

- **Documentation**: [Teleop User Guide](https://docs.wuji.technology/docs/en/wuji-hand/latest/teleop-user-guide/introduction/)
- **Hardware BOM**: [Bill of Materials](https://docs.google.com/document/d/19Md8R5tw9OyTvOUD-JKt7S6xMivlHVSCSNAKuoZr1eo/edit?tab=t.0)
- **Related Projects**:
  - [wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting) — Hand pose retargeting algorithm
  - [wujihandros2](https://github.com/wuji-technology/wujihandros2) — Wuji Hand ROS2 driver
  - [pico-ros2-bridge](https://github.com/wuji-technology/pico-ros2-bridge) — PICO VR to ROS2 bridge
- **Acknowledgements**:
  - StereoVR stereo vision module — Liang ZHU (lzhu686@connect.hkust-gz.edu.cn)
  - Tianji Arm controller — based on [TJ_FX_ROBOT_CONTRL_SDK](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK)

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
