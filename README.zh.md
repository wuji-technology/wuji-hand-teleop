中文 | [English](README.md)

# Wuji Hand Teleop ROS2

基于 ROS2 的 Wuji Hand 遥操作系统，支持 Apple Vision Pro 和 MANUS 数据手套作为输入设备。

> **即将推出**: 机械臂输出支持（Tianji Arm）

我们欢迎朋友们一起进行二次开发，一起打造 Wuji 生态。

---

## 目录

1. [概述](#1-概述)
2. [系统架构](#2-系统架构)
3. [环境配置](#3-环境配置)
4. [快速开始](#4-快速开始)
5. [配置文件](#5-配置文件)
6. [MANUS 手套设置](#6-manus-手套设置)
7. [话题接口](#7-话题接口)
8. [目录结构](#8-目录结构)
9. [常见问题](#9-常见问题)
10. [致谢](#10-致谢)

---

## 1. 概述

### 支持的设备

| 输入设备 | 输出 | 状态 |
|----------|------|------|
| Apple Vision Pro | Wuji Hand | ✅ 已支持 |
| MANUS 手套 | Wuji Hand | ✅ 已支持 |
| 自定义设备 | Wuji Hand | ✅ 已支持（通过 `/hand_input` 话题） |
| Apple Vision Pro / HTC Vive | Tianji Arm | 🚧 即将推出 |

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **硬件**: Wuji Hand（USB 连接）

---

## 2. 系统架构

### 数据流

```
输入设备                         ROS 话题
────────                         ────────

Apple Vision Pro  ───────────►  /hand_input (Float32MultiArray)
                                    │
MANUS 手套  ─────────────────►      │
                                    │
自定义设备  ─────────────────►      │
                                    ▼
                               wujihand_ik
                                    │
                                    ▼
                               Wuji Hand（硬件）
```

### 自定义输入设备

只需将您的输入节点输出维护为以下格式：

- **话题**: `/hand_input`
- **类型**: `std_msgs/Float32MultiArray`
- **格式**: MediaPipe 21 点格式（详见 [话题接口](#7-话题接口)）

---

## 3. 环境配置

### 3.1 前置条件

```bash
# 安装 ROS2 Humble（Ubuntu 22.04）
sudo apt update
sudo apt install ros-humble-desktop

# 安装 ROS2 构建依赖
sudo apt install python3-colcon-common-extensions
```

### 3.2 Python 依赖

```bash
# Wuji Hand SDK
pip3 install --user wujihandpy

# 手部重定向算法（必需）
pip3 install --user wuji-retargeting

# 用于 Vision Pro 输入
pip3 install --user avp-stream

# 其他依赖
pip3 install --user numpy pyyaml
```

### 3.3 克隆与编译

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/wuji-technology/wuji-hand-teleop-ros2.git

# 编译
cd ~/ros2_ws
colcon build --symlink-install

# 加载工作空间
source install/setup.bash
```

---

## 4. 快速开始

### 4.1 启动命令

```bash
# 首先加载工作空间
source ~/ros2_ws/install/setup.bash

# 使用 Vision Pro
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=avp

# 使用 MANUS 手套
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 4.2 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hand_input` | `avp` | 输入设备：`avp` 或 `manus` |
| `hand_config` | （默认路径） | wujihand_ik.yaml 路径 |

---

## 5. 配置文件

### 5.1 Wuji Hand 配置

**获取序列号：**
```bash
lsusb -v -d 0483:2000 | grep iSerial
```

**文件**: `src/wujihand_ik/wujihand_ik/config/wujihand_ik.yaml`

```yaml
# 手部序列号（设为 null 禁用该手）
right_hand_serial: "347B38703433"  # （示例）
left_hand_serial: "3472387D3433"  # （示例）

# 输入模式（false = 使用重定向，true = 直接关节角度）
use_joint_input: false

# 输入源："avp" 或 "manus"
input_source: "avp"
```

### 5.2 Apple Vision Pro 配置

**文件**: `src/input_devices/avp_input/avp_input/config/avp_input.yaml`

```yaml
avp_ip: "192.168.2.13"     # 您的 Vision Pro IP 地址
publish_rate_hz: 200.0      # 发布频率
include_right_hand: true
include_left_hand: true
```

### 5.3 重定向配置

**文件**: `src/wujihand_ik/wujihand_ik/config/`

| 输入源 | 配置文件 | 说明 |
|--------|----------|------|
| AVP | `retarget_avp.yaml` | 左右手共用配置 |
| MANUS | `retarget_manus_right.yaml` | 右手（z 轴旋转：-15°） |
| MANUS | `retarget_manus_left.yaml` | 左手（z 轴旋转：+15°） |

> **说明**：由于坐标系差异，MANUS 需要为左右手分别配置。

---

## 6. MANUS 手套设置

### 6.1 关于 MANUS SDK 文件

本仓库中的 MANUS ROS2 驱动（`manus_ros2` 包）基于 [MANUS 官网](https://www.manus-meta.com/) 提供的官方 SDK，并进行了适配 ROS2 的修改。

> **重要**：
> - SDK 可能需要随 MANUS Core 软件版本更新而调整
> - 如果您从 MANUS 官网直接下载 SDK，可能需要自行调整适配

### 6.2 标定

**⚠️ 标定对于准确的手部追踪至关重要！**

**标定流程：**

1. 从 [MANUS 官网](https://www.manus-meta.com/resources/downloads) 下载 **MANUS Core 3**（仅 Windows）
2. 通过蓝牙适配器在 Windows 上连接 MANUS 手套
3. 在 MANUS Core 3 图形界面中按照提示完成标定
4. 导出标定文件（`.mcal` 文件）
5. 将标定文件复制到本仓库的指定位置：
   ```bash
   # 替换默认标定文件
   cp /path/to/YourCalibration.mcal \
      src/input_devices/manus_input/manus_ros2/calibration/Calibration.mcal
   ```

### 6.3 MANUS 配置

**文件**: `src/input_devices/manus_input/manus_input_py/manus_input_py/config/manus_input.yaml`

```yaml
# 启用手部
include_right_hand: true
include_left_hand: true

# 手套 ID（由 MANUS Core 自动分配，默认 0=左手, 1=右手）
left_glove_id: 0
right_glove_id: 1
```

### 6.4 运行 MANUS 输入

```bash
# 编译 MANUS 包（首次或更改后）
cd ~/ros2_ws
colcon build --packages-select manus_ros2_msgs manus_ros2 manus_input_py
source install/setup.bash

# 使用 MANUS 输入启动
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

### 6.5 MANUS 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | 左手套原始数据 |
| `/manus_glove_1` | `manus_ros2_msgs/ManusGlove` | 右手套原始数据 |
| `/hand_input` | `Float32MultiArray` | 转换后的 MediaPipe 格式 |

---

## 7. 话题接口

### 7.1 主输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/hand_input` | `std_msgs/Float32MultiArray` | 手部关键点（MediaPipe 格式） |

**数据格式：**
- 单手：63 个值（21 个关键点 × 3 个坐标）
- 双手：126 个值（先右手，后左手）

**关键点顺序（每只手 21 个点）：**
```
0: 手腕
1-4: 拇指（CMC、MCP、IP、TIP）
5-8: 食指（MCP、PIP、DIP、TIP）
9-12: 中指（MCP、PIP、DIP、TIP）
13-16: 无名指（MCP、PIP、DIP、TIP）
17-20: 小指（MCP、PIP、DIP、TIP）
```

### 7.2 AVP 专用话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/right_wrist` | `Float32MultiArray` | 右手腕变换（4×4 矩阵） |
| `/left_wrist` | `Float32MultiArray` | 左手腕变换（4×4 矩阵） |
| `/head_pose` | `Float32MultiArray` | 头部姿态变换（4×4 矩阵） |

---

## 8. 目录结构

```
wuji-hand-teleop-ros2/
├── README.md
├── LICENSE
├── src/
│   ├── wuji_teleop_bringup/           # 启动文件
│   │   └── launch/
│   │       └── wuji_teleop_hand.launch.py
│   │
│   ├── input_devices/
│   │   ├── avp_input/                 # Apple Vision Pro 输入
│   │   │   ├── avp_input/
│   │   │   │   ├── avp_input_node.py
│   │   │   │   └── config/avp_input.yaml
│   │   │   └── launch/
│   │   │
│   │   └── manus_input/               # MANUS 手套输入
│   │       ├── manus_input_py/        # Python 节点（格式转换）
│   │       ├── manus_ros2/            # C++ SDK 驱动
│   │       │   ├── ManusSDK/          # MANUS SDK 库
│   │       │   └── calibration/       # 标定文件（.mcal）
│   │       └── manus_ros2_msgs/       # 自定义消息定义
│   │
│   └── wujihand_ik/                   # 手部 IK 和重定向
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

## 9. 常见问题

| 问题 | 解决方案 |
|------|----------|
| 无法连接 Vision Pro | 1. 确保设备在同一网络<br>2. 检查配置中的 `avp_ip`<br>3. 在 Vision Pro 上运行 avp_stream 应用 |
| 找不到手部序列号 | 运行 `lsusb -v -d 0483:2000 \| grep iSerial` |
| `ImportError: wuji_retargeting` | `pip3 install wuji-retargeting` |
| `ImportError: wujihandpy` | `pip3 install wujihandpy` |
| 编译后找不到包 | `source ~/ros2_ws/install/setup.bash` |
| MANUS 手套未检测到 | 1. 检查蓝牙连接<br>2. 确认标定文件存在<br>3. 检查配置中的手套 ID |
| 手部追踪精度差 | 在 MANUS Core 3 中重新标定 |

**调试日志：**
```bash
ros2 run wujihand_ik wujihand_retargeting --ros-args --log-level debug
```

---

## 10. 致谢

### MANUS

本项目中的 MANUS 手套集成使用了 MANUS 官方 SDK。感谢 [MANUS](https://www.manus-meta.com/) 提供 SDK 和文档支持。

> 说明：ROS2 SDK 适配器可能需要随 MANUS Core 版本更新而调整。从 MANUS 官网直接下载 SDK 的用户可能需要自行适配。

### 相关项目

- **[wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting)** - 手部重定向算法
- **[wujihandpy](https://pypi.org/project/wujihandpy/)** - Wuji Hand 控制 SDK
- **[avp-stream](https://pypi.org/project/avp-stream/)** - Apple Vision Pro 串流库

---

## 许可证

MIT License - 见 [LICENSE](LICENSE)

## 维护者

无极科技 (support@wuji.tech)

## 贡献者

- Guanqi He
- Wentao Zhang
- Liang Zhu

---

欢迎提交 Issue 和 PR：https://github.com/wuji-technology/wuji-hand-teleop-ros2
