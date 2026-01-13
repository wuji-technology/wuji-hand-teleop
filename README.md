# Wuji Hand Teleop ROS2

A ROS2-based teleoperation system for Wuji Hand, supporting Apple Vision Pro and MANUS Gloves as input devices.

基于 ROS2 的 Wuji Hand 遥操作系统，支持 Apple Vision Pro 和 MANUS 数据手套作为输入设备。

> **Coming Soon / 即将推出**: 机械臂输出支持（Tianji Arm）

We welcome contributions to the Wuji ecosystem!

我们欢迎朋友们一起进行二次开发，一起打造 Wuji 生态。

---

## Table of Contents / 目录

1. [Overview / 概述](#1-overview--概述)
2. [System Architecture / 系统架构](#2-system-architecture--系统架构)
3. [Installation / 环境配置](#3-installation--环境配置)
4. [Quick Start / 快速开始](#4-quick-start--快速开始)
5. [Configuration / 配置文件](#5-configuration--配置文件)
6. [MANUS Glove Setup / MANUS 手套设置](#6-manus-glove-setup--manus-手套设置)
7. [Topic Interface / 话题接口](#7-topic-interface--话题接口)
8. [Directory Structure / 目录结构](#8-directory-structure--目录结构)
9. [Troubleshooting / 常见问题](#9-troubleshooting--常见问题)
10. [Acknowledgments / 致谢](#10-acknowledgments--致谢)

---

## 1. Overview / 概述

### Supported Devices / 支持的设备

| Input Device / 输入设备 | Output / 输出 | Status / 状态 |
|------------------------|---------------|---------------|
| Apple Vision Pro | Wuji Hand | ✅ Supported |
| MANUS Glove | Wuji Hand | ✅ Supported |
| Custom Device | Wuji Hand | ✅ Supported (via `/hand_input` topic) |
| Apple Vision Pro / HTC Vive | Tianji Arm | 🚧 Coming Soon |

### System Requirements / 系统要求

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Hardware**: Wuji Hand (USB connection)

---

## 2. System Architecture / 系统架构

### Data Flow / 数据流

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

### Custom Input Device / 自定义输入设备

只需将您的输入节点输出维护为以下格式：

- **Topic**: `/hand_input`
- **Type**: `std_msgs/Float32MultiArray`
- **Format**: MediaPipe 21-point format (详见 [Topic Interface](#7-topic-interface--话题接口))

---

## 3. Installation / 环境配置

### 3.1 Prerequisites / 前置条件

```bash
# Install ROS2 Humble (Ubuntu 22.04)
# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install ROS2 build dependencies
# 安装 ROS2 构建依赖
sudo apt install python3-colcon-common-extensions
```

### 3.2 Python Dependencies / Python 依赖

```bash
# Wuji Hand SDK
pip3 install --user wujihandpy

# Hand retargeting algorithm (required)
# 手部重定向算法（必需）
pip3 install --user wuji-retargeting

# For Apple Vision Pro input
# 用于 Vision Pro 输入
pip3 install --user avp-stream

# Other dependencies
pip3 install --user numpy pyyaml
```

### 3.3 Clone and Build / 克隆与编译

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

## 4. Quick Start / 快速开始

### 4.1 Launch Commands / 启动命令

```bash
# Source workspace first
source ~/ros2_ws/install/setup.bash

# Using Apple Vision Pro
# 使用 Vision Pro
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=avp

# Using MANUS Gloves
# 使用 MANUS 手套
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 4.2 Launch Parameters / 启动参数

| Parameter | Default | Description / 说明 |
|-----------|---------|-------------------|
| `hand_input` | `avp` | Input device: `avp` or `manus` |
| `hand_config` | (default path) | Path to wujihand_ik.yaml |

---

## 5. Configuration / 配置文件

### 5.1 Wuji Hand Configuration / Wuji Hand 配置

**Get Wuji Hand Serial Number / 获取序列号:**
```bash
lsusb -v -d 0483:2000 | grep iSerial
```

**File / 文件**: `src/wujihand_ik/wujihand_ik/config/wujihand_ik.yaml`

```yaml
# Hand serial numbers (set to null to disable)
# 手部序列号（设为 null 禁用该手）
right_hand_serial: "347B38703433"  # (for example)
left_hand_serial: "3472387D3433"  # (for example)

# Input mode (false = use retargeting, true = direct joint angles)
use_joint_input: false

# Input source: "avp" or "manus"
input_source: "avp"
```

### 5.2 Apple Vision Pro Configuration / Vision Pro 配置

**File / 文件**: `src/input_devices/avp_input/avp_input/config/avp_input.yaml`

```yaml
avp_ip: "192.168.2.13"     # Your Vision Pro IP address
publish_rate_hz: 200.0      # Publishing rate
include_right_hand: true
include_left_hand: true
```

### 5.3 Retargeting Configuration / 重定向配置

**Files / 文件**: `src/wujihand_ik/wujihand_ik/config/`

| Input Source | Config File | Description |
|--------------|-------------|-------------|
| AVP | `retarget_avp.yaml` | Left/right hands share config |
| MANUS | `retarget_manus_right.yaml` | Right hand (z-rotation: -15°) |
| MANUS | `retarget_manus_left.yaml` | Left hand (z-rotation: +15°) |

> **Note / 说明**: MANUS requires separate configs for left/right hands due to coordinate system differences.

---

## 6. MANUS Glove Setup / MANUS 手套设置

### 6.1 About MANUS SDK Files / 关于 MANUS SDK 文件

The MANUS ROS2 driver (`manus_ros2` package) is based on the official MANUS SDK from [manus-meta.com](https://www.manus-meta.com/). We have made modifications to adapt it for ROS2 integration.

本仓库中的 MANUS ROS2 驱动（`manus_ros2` 包）基于 [MANUS 官网](https://www.manus-meta.com/) 提供的官方 SDK，并进行了适配 ROS2 的修改。

> **Important / 重要**:
> - The SDK may need updates when MANUS Core software version changes
> - SDK 可能需要随 MANUS Core 软件版本更新而调整
> - If you download SDK directly from MANUS, you may need to adapt it yourself
> - 如果您从 MANUS 官网直接下载 SDK，可能需要自行调整适配

### 6.2 Calibration / 标定

**⚠️ Calibration is required for accurate hand tracking! / 标定对于准确的手部追踪至关重要！**

**Calibration Process / 标定流程:**

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

**标定流程:**

1. 从 [MANUS 官网](https://www.manus-meta.com/resources/downloads) 下载 **MANUS Core 3**（仅 Windows）
2. 通过蓝牙适配器在 Windows 上连接 MANUS 手套
3. 在 MANUS Core 3 图形界面中按照提示完成标定
4. 导出标定文件（`.mcal` 文件）
5. 将标定文件复制到本仓库的指定位置（替换默认文件）

### 6.3 MANUS Configuration / MANUS 配置

**File / 文件**: `src/input_devices/manus_input/manus_input_py/manus_input_py/config/manus_input.yaml`

```yaml
# Enable hands
include_right_hand: true
include_left_hand: true

# Glove IDs (assigned by MANUS Core, default: 0=left, 1=right)
# 手套 ID（由 MANUS Core 自动分配，默认 0=左手, 1=右手）
left_glove_id: 0
right_glove_id: 1
```

### 6.4 Running MANUS Input / 运行 MANUS 输入

```bash
# Build MANUS packages (first time or after changes)
cd ~/ros2_ws
colcon build --packages-select manus_ros2_msgs manus_ros2 manus_input_py
source install/setup.bash

# Launch with MANUS input
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 6.5 MANUS Topics / MANUS 话题

| Topic | Type | Description |
|-------|------|-------------|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | Left glove raw data |
| `/manus_glove_1` | `manus_ros2_msgs/ManusGlove` | Right glove raw data |
| `/hand_input` | `Float32MultiArray` | Converted MediaPipe format |

---

## 7. Topic Interface / 话题接口

### 7.1 Main Input Topic / 主输入话题

| Topic | Type | Description |
|-------|------|-------------|
| `/hand_input` | `std_msgs/Float32MultiArray` | Hand keypoints (MediaPipe format) |

**Data Format / 数据格式:**
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

### 7.2 AVP-specific Topics / AVP 专用话题

| Topic | Type | Description |
|-------|------|-------------|
| `/right_wrist` | `Float32MultiArray` | Right wrist transform (4×4 matrix) |
| `/left_wrist` | `Float32MultiArray` | Left wrist transform (4×4 matrix) |
| `/head_pose` | `Float32MultiArray` | Head pose transform (4×4 matrix) |

---

## 8. Directory Structure / 目录结构

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

## 9. Troubleshooting / 常见问题

| Problem / 问题 | Solution / 解决方案 |
|----------------|---------------------|
| Cannot connect to Vision Pro | 1. Ensure devices on same network<br>2. Check `avp_ip` in config<br>3. Run avp_stream app on Vision Pro |
| Hand serial not found | Run `lsusb -v -d 0483:2000 \| grep iSerial` |
| `ImportError: wuji_retargeting` | `pip3 install wuji-retargeting` |
| `ImportError: wujihandpy` | `pip3 install wujihandpy` |
| Package not found after build | `source ~/ros2_ws/install/setup.bash` |
| MANUS glove not detected | 1. Check Bluetooth connection<br>2. Verify calibration file exists<br>3. Check glove IDs in config |
| Poor hand tracking accuracy | Re-run calibration in MANUS Core 3 |

**Debug Logging / 调试日志:**
```bash
ros2 run wujihand_ik wujihand_retargeting --ros-args --log-level debug
```

---

## 10. Acknowledgments / 致谢

### MANUS

The MANUS Glove integration in this project uses the official MANUS SDK. We thank [MANUS](https://www.manus-meta.com/) for providing the SDK and documentation.

本项目中的 MANUS 手套集成使用了 MANUS 官方 SDK。感谢 [MANUS](https://www.manus-meta.com/) 提供 SDK 和文档支持。

> Note: The ROS2 SDK adapter may require updates when MANUS Core version changes. Users downloading SDK directly from MANUS website may need to make their own adaptations.

### Related Projects / 相关项目

- **[wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting)** - Hand retargeting algorithm
- **[wujihandpy](https://pypi.org/project/wujihandpy/)** - Wuji Hand control SDK
- **[avp-stream](https://pypi.org/project/avp-stream/)** - Apple Vision Pro streaming library

---

## License / 许可证

MIT License - see [LICENSE](LICENSE)

## Maintainer / 维护者

Wuji Technology (support@wuji.tech)

## Contributors / 贡献者

- Guanqi He
- Wentao Zhang
- Liang Zhu

---

Issues and PRs welcome at: https://github.com/wuji-technology/wuji-hand-teleop-ros2
