[中文](README.zh.md) | English

# Wuji Hand Teleop ROS2

A ROS2-based teleoperation system for Wuji Hand, supporting Apple Vision Pro and MANUS Gloves as input devices.

> **Coming Soon**: Robot arm output support (Tianji Arm)

We welcome contributions to the Wuji ecosystem!

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Installation](#3-installation)
4. [Quick Start](#4-quick-start)
5. [Configuration](#5-configuration)
6. [MANUS Glove Setup](#6-manus-glove-setup)
7. [Topic Interface](#7-topic-interface)
8. [Directory Structure](#8-directory-structure)
9. [Troubleshooting](#9-troubleshooting)
10. [Acknowledgments](#10-acknowledgments)

---

## 1. Overview

### Supported Devices

| Input Device | Output | Status |
|--------------|--------|--------|
| Apple Vision Pro | Wuji Hand | ✅ Supported |
| MANUS Glove | Wuji Hand | ✅ Supported |
| Custom Device | Wuji Hand | ✅ Supported (via `/hand_input` topic) |
| Apple Vision Pro / HTC Vive | Tianji Arm | 🚧 Coming Soon |

### System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Hardware**: Wuji Hand (USB connection)

---

## 2. System Architecture

### Data Flow

```
Input Devices                    ROS Topics
─────────────                    ──────────

Apple Vision Pro  ───────────►  /hand_input (Float32MultiArray)
                                    │
MANUS Glove  ────────────────►      │
                                    │
Custom Device  ──────────────►      │
                                    ▼
                               wujihand_ik
                                    │
                                    ▼
                               Wuji Hand (Hardware)
```

### Custom Input Device

Simply publish your input node output in the following format:

- **Topic**: `/hand_input`
- **Type**: `std_msgs/Float32MultiArray`
- **Format**: MediaPipe 21-point format (see [Topic Interface](#7-topic-interface))

---

## 3. Installation

### 3.1 Prerequisites

```bash
# Install ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Install ROS2 build dependencies
sudo apt install python3-colcon-common-extensions
```

### 3.2 Python Dependencies

```bash
# Wuji Hand SDK
pip3 install --user wujihandpy

# Hand retargeting algorithm (required)
pip3 install --user wuji-retargeting

# For Apple Vision Pro input
pip3 install --user avp-stream

# Other dependencies
pip3 install --user numpy pyyaml
```

### 3.3 Clone and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/wuji-technology/wuji-hand-teleop-ros2.git

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 4. Quick Start

### 4.1 Launch Commands

```bash
# Source workspace first
source ~/ros2_ws/install/setup.bash

# Using Apple Vision Pro
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=avp

# Using MANUS Gloves
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 4.2 Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_input` | `avp` | Input device: `avp` or `manus` |
| `hand_config` | (default path) | Path to wujihand_ik.yaml |

---

## 5. Configuration

### 5.1 Wuji Hand Configuration

**Get Wuji Hand Serial Number:**
```bash
lsusb -v -d 0483:2000 | grep iSerial
```

**File**: `src/wujihand_ik/wujihand_ik/config/wujihand_ik.yaml`

```yaml
# Hand serial numbers (set to null to disable)
right_hand_serial: "347B38703433"  # (for example)
left_hand_serial: "3472387D3433"  # (for example)

# Input mode (false = use retargeting, true = direct joint angles)
use_joint_input: false

# Input source: "avp" or "manus"
input_source: "avp"
```

### 5.2 Apple Vision Pro Configuration

**File**: `src/input_devices/avp_input/avp_input/config/avp_input.yaml`

```yaml
avp_ip: "192.168.2.13"     # Your Vision Pro IP address
publish_rate_hz: 200.0      # Publishing rate
include_right_hand: true
include_left_hand: true
```

### 5.3 Retargeting Configuration

**Files**: `src/wujihand_ik/wujihand_ik/config/`

| Input Source | Config File | Description |
|--------------|-------------|-------------|
| AVP | `retarget_avp.yaml` | Left/right hands share config |
| MANUS | `retarget_manus_right.yaml` | Right hand (z-rotation: -15°) |
| MANUS | `retarget_manus_left.yaml` | Left hand (z-rotation: +15°) |

> **Note**: MANUS requires separate configs for left/right hands due to coordinate system differences.

---

## 6. MANUS Glove Setup

### 6.1 About MANUS SDK Files

The MANUS ROS2 driver (`manus_ros2` package) is based on the official MANUS SDK from [manus-meta.com](https://www.manus-meta.com/). We have made modifications to adapt it for ROS2 integration.

> **Important**:
> - The SDK may need updates when MANUS Core software version changes
> - If you download SDK directly from MANUS, you may need to adapt it yourself

### 6.2 Calibration

**⚠️ Calibration is required for accurate hand tracking!**

**Calibration Process:**

1. **Download MANUS Core 3** from [manus-meta.com](https://www.manus-meta.com/resources/downloads) (Windows only)
2. **Connect your MANUS Gloves** via Bluetooth dongle on Windows
3. **Run calibration** in MANUS Core 3 GUI following the on-screen instructions
4. **Export calibration file** (`.mcal` file)
5. **Copy calibration file** to this repository:
   ```bash
   # Replace the default calibration file
   cp /path/to/YourCalibration.mcal \
      src/input_devices/manus_input/manus_ros2/calibration/Calibration.mcal
   ```

### 6.3 MANUS Configuration

**File**: `src/input_devices/manus_input/manus_input_py/manus_input_py/config/manus_input.yaml`

```yaml
# Enable hands
include_right_hand: true
include_left_hand: true

# Glove IDs (assigned by MANUS Core, default: 0=left, 1=right)
left_glove_id: 0
right_glove_id: 1
```

### 6.4 Running MANUS Input

```bash
# Build MANUS packages (first time or after changes)
cd ~/ros2_ws
colcon build --packages-select manus_ros2_msgs manus_ros2 manus_input_py
source install/setup.bash

# Launch with MANUS input
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 6.5 MANUS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | Left glove raw data |
| `/manus_glove_1` | `manus_ros2_msgs/ManusGlove` | Right glove raw data |
| `/hand_input` | `Float32MultiArray` | Converted MediaPipe format |

---

## 7. Topic Interface

### 7.1 Main Input Topic

| Topic | Type | Description |
|-------|------|-------------|
| `/hand_input` | `std_msgs/Float32MultiArray` | Hand keypoints (MediaPipe format) |

**Data Format:**
- Single hand: 63 values (21 keypoints × 3 coordinates)
- Dual hands: 126 values (right hand first, then left)

**Keypoint Order (21 points per hand):**
```
0: WRIST
1-4: THUMB (CMC, MCP, IP, TIP)
5-8: INDEX (MCP, PIP, DIP, TIP)
9-12: MIDDLE (MCP, PIP, DIP, TIP)
13-16: RING (MCP, PIP, DIP, TIP)
17-20: PINKY (MCP, PIP, DIP, TIP)
```

### 7.2 AVP-specific Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/right_wrist` | `Float32MultiArray` | Right wrist transform (4×4 matrix) |
| `/left_wrist` | `Float32MultiArray` | Left wrist transform (4×4 matrix) |
| `/head_pose` | `Float32MultiArray` | Head pose transform (4×4 matrix) |

---

## 8. Directory Structure

```
wuji-hand-teleop-ros2/
├── README.md
├── LICENSE
├── src/
│   ├── wuji_teleop_bringup/           # Launch files
│   │   └── launch/
│   │       └── wuji_teleop_hand.launch.py
│   │
│   ├── input_devices/
│   │   ├── avp_input/                 # Apple Vision Pro input
│   │   │   ├── avp_input/
│   │   │   │   ├── avp_input_node.py
│   │   │   │   └── config/avp_input.yaml
│   │   │   └── launch/
│   │   │
│   │   └── manus_input/               # MANUS Glove input
│   │       ├── manus_input_py/        # Python node (format conversion)
│   │       ├── manus_ros2/            # C++ SDK driver
│   │       │   ├── ManusSDK/          # MANUS SDK libraries
│   │       │   └── calibration/       # Calibration files (.mcal)
│   │       └── manus_ros2_msgs/       # Custom message definitions
│   │
│   └── wujihand_ik/                   # Hand IK and retargeting
│       ├── wujihand_ik/
│       │   ├── ik_node.py
│       │   └── config/
│       │       ├── wujihand_ik.yaml
│       │       ├── retarget_avp.yaml
│       │       ├── retarget_manus_left.yaml
│       │       └── retarget_manus_right.yaml
│       └── package.xml
```

---

## 9. Troubleshooting

| Problem | Solution |
|---------|----------|
| Cannot connect to Vision Pro | 1. Ensure devices on same network<br>2. Check `avp_ip` in config<br>3. Run avp_stream app on Vision Pro |
| Hand serial not found | Run `lsusb -v -d 0483:2000 \| grep iSerial` |
| `ImportError: wuji_retargeting` | `pip3 install wuji-retargeting` |
| `ImportError: wujihandpy` | `pip3 install wujihandpy` |
| Package not found after build | `source ~/ros2_ws/install/setup.bash` |
| MANUS glove not detected | 1. Check Bluetooth connection<br>2. Verify calibration file exists<br>3. Check glove IDs in config |
| Poor hand tracking accuracy | Re-run calibration in MANUS Core 3 |

**Debug Logging:**
```bash
ros2 run wujihand_ik wujihand_retargeting --ros-args --log-level debug
```

---

## 10. Acknowledgments

### MANUS

The MANUS Glove integration in this project uses the official MANUS SDK. We thank [MANUS](https://www.manus-meta.com/) for providing the SDK and documentation.

> Note: The ROS2 SDK adapter may require updates when MANUS Core version changes. Users downloading SDK directly from MANUS website may need to make their own adaptations.

### Related Projects

- **[wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting)** - Hand retargeting algorithm
- **[wujihandpy](https://pypi.org/project/wujihandpy/)** - Wuji Hand control SDK
- **[avp-stream](https://pypi.org/project/avp-stream/)** - Apple Vision Pro streaming library

---

## License

MIT License - see [LICENSE](LICENSE)

## Maintainer

Wuji Technology (support@wuji.tech)

## Contributors

- Guanqi He
- Wentao Zhang
- Liang Zhu

---

Issues and PRs welcome at: https://github.com/wuji-technology/wuji-hand-teleop-ros2
