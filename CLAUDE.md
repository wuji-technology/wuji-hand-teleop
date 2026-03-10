# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2-based teleoperation system for Wuji Hand and Tianji Arm, supporting multiple input devices (MANUS Gloves, HTC Vive Trackers, PICO VR). The system uses a modular topic-based architecture where all input sources abstract to standard ROS2 interfaces.

**Requirements:** Ubuntu 22.04 LTS, ROS2 Humble, Python 3.10+

## Build Commands

```bash
# Build all packages
colcon build --symlink-install
source install/setup.bash

# Build specific package
colcon build --packages-select <package_name> --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Launch Commands

```bash
# Full teleoperation (hand + arm) — SteamVR 方案, 使用 tianji_output
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker        # ✓ 已测试
ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py hand_input:=manus arm_input:=tracker  # ✓ 已测试

# Single side only
ros2 launch wuji_teleop_bringup wuji_teleop_single.launch.py side:=right hand_input:=manus arm_input:=tracker

# Hand only / Arm only
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker enable_rviz:=true

# PICO VR teleoperation — PICO 方案, 使用 tianji_world_output
ros2 launch wuji_teleop_bringup pico_teleop.launch.py                # ⚠ 未经真机测试
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py        # ✓ 已测试
```

## Testing & Debugging

```bash
# Check topics
ros2 topic list
ros2 topic echo /hand_input
ros2 topic hz /pico/tracker/left_wrist

# View TF tree
ros2 run tf2_tools view_frames

# Debug logging
ros2 run tianji_output tianji_output_node --ros-args --log-level debug

# Get device serial numbers
lsusb -v -d 0483:2000 | grep iSerial  # Wuji Hand
```

## Architecture

### Data Flow
```
Input Devices (MANUS/HTC Vive/PICO)
    ↓
Input Nodes (manus_input, openvr_input, pico_input)
    ↓
Standard ROS2 Topics (/hand_input, TF: left_wrist/right_wrist)
    ↓
Processing (wujihand_ik for hand retargeting, tianji_output for arm IK)
    ↓
Hardware Output (Wuji Hand, Tianji Arm)
```

### Topic Interface

**Hand control:** Publish to `/hand_input` (`Float32MultiArray`, MediaPipe 21-point × 3D = 63 values per hand, 126 for both)

**Arm control:** Publish TF transforms to `left_wrist`/`right_wrist`/`chest` frames

### TF Tree Structure
```
world
├── head
├── chest (computed from head, -0.3m Z)
│   ├── left_chest (static transform)
│   └── right_chest (static transform)
├── left_wrist → tianji_left
├── right_wrist → tianji_right
├── left_arm (upper arm tracker for elbow angle)
└── right_arm (upper arm tracker for elbow angle)
```

**Key insight:** `tianji_output_node` queries `left_chest -> tianji_left` and ROS2 TF automatically computes the full transform chain.

### Package Structure

| Package | Purpose |
|---------|---------|
| `wuji_teleop_bringup` | Launch files (main entry point) |
| `input_devices/manus_input` | MANUS data glove driver |
| `input_devices/openvr_input` | HTC Vive Tracker support |
| `input_devices/pico_input` | PICO VR controller/tracker |
| `output_devices/tianji_output` | Tianji Arm IK (胸部坐标系, SteamVR 方案) |
| `output_devices/tianji_world_output` | Tianji Arm IK (世界坐标系, PICO 方案) |
| `output_devices/wujihand_output` | Wuji Hand retargeting config + IK |
| `controller` | 统一控制器 (tianji_arm_controller, wujihand_controller) |

## Configuration Files

| Config | Location |
|--------|----------|
| Wuji Hand serials | `src/output_devices/wujihand_output/config/wujihand_ik.yaml` |
| Hand retargeting | `src/output_devices/wujihand_output/config/retarget_*.yaml` |
| MANUS glove IDs | `src/input_devices/manus_input/manus_input_py/config/manus_input.yaml` |
| HTC Tracker serials | `src/input_devices/openvr_input/config/openvr_input.yaml` |
| Tianji Arm IP | `src/output_devices/tianji_output/config/tianji_output.yaml` |

## Wuji Hand Driver Parameters

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_number` | `""` | USB 设备序列号 |
| `publish_rate` | `1000.0` | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | `10.0` | 固件侧 RT_POS LP 截止频率 (Hz, 覆写固件默认 3Hz) |
| `diagnostics_rate` | `10.0` | 诊断发布频率 (Hz) |
| `effort_limit` | `-1.0` | 电流限制 (A), <0 = 固件默认 (1.5A), 最大 3.5A |
| `torque_slope_limit` | `-1.0` | 力矩斜率限制, <0 = 固件默认 (20%/cycle) |

## Wuji Hand 控制链路 (完整滤波架构)

```
Manus手套 → retarget (100Hz) → [EMA 5.7Hz] → /joint_commands → [固件 10Hz LP @ 16kHz] → [PID Kp=5 Ki=0.08 Kd=150] → 电机
                                  ↑ 主瓶颈                        ↑ 遥操作时冗余
```

| 层级 | 滤波 | 截止频率 | 采样率 | 代码位置 |
|------|------|---------|--------|---------|
| Controller EMA | α=0.3 | **5.7Hz** | 100Hz | `wujihand_node.py:49-51` |
| 固件 RT_POS | 1阶 IIR | **10Hz** | 16kHz | SDK SDO 0x05:19 覆写 |
| 固件 PID | 三环级联 | — | 16kHz | Kp=5.0, Ki=0.08, Kd=150.0 (硬编码) |

### 实测性能 (固件 v1.2.1)
- **5° 阶跃**: 过冲 3.6-6.9%, 上升时间 27-39ms, 稳态振荡 0.24-0.42° P-P
- **30° 阶跃**: 电流饱和, 只达 37-62% (积分项需更长时间)
- **正弦追踪 ±5°**: 0.5Hz 完美, 2Hz 可用, 5Hz 崩坏 (RMS=3.5°)
- **正弦追踪 ±15°**: 所有频率幅值衰减 72-93% (力矩饱和)
- **振荡根因**: 邻间保持 (相邻关节机械耦合), 固件团队确认为预期行为
- 测试工具: `src/output_devices/wujihand_output/test/step_response_test.py`

## External Dependencies

```bash
# wujihandcpp C++ SDK — wujihand_driver (ROS2 C++ 节点) 的编译依赖
# ROS2 驱动仓库: https://github.com/wuji-technology/wujihandros2
# deb 下载: https://github.com/wuji-technology/wujihandpy/releases (Docker 已包含)

# wuji_retargeting (requires submodules)
git clone --recurse-submodules https://github.com/Wuji-Technology-Co-Ltd/wuji_retargeting.git
cd wuji_retargeting && pip install -e .
```

## Adding Custom Input Devices

1. Create node that publishes to standard topic interface:
   - Hand: `/hand_input` (Float32MultiArray, MediaPipe 21-point format)
   - Arm: TF to `left_wrist`/`right_wrist` frames
2. Add entry point in `setup.py`
3. Update launch files if needed

## Related Projects

- **pico-ros2-bridge**: Docker container bridging PICO XR devices to ROS2 (separate repo)
- **XRoboToolkit-PC-Service-Pybind**: Python bindings for XRoboToolkit SDK
- **wuji_retargeting**: Hand motion mapping algorithm library

## PICO Teleop Coordinate System Convention

**Single Source of Truth:** `tianji_robot.yaml` (`src/output_devices/tianji_world_output/config/tianji_robot.yaml`)

### Chest Coordinate Systems (World → Chest Rotation)

```
Left Chest (绕 X 轴 +90°):
  +X = +X_world (前)
  +Y = -Z_world (向下)     ← 关键: Y 控制上下！
  +Z = +Y_world (向左)

Right Chest (绕 X 轴 -90°):
  +X = +X_world (前)
  +Y = +Z_world (向上)     ← 关键: Y 与 Left 相反！
  +Z = -Y_world (向右)
```

**四元数 (YAML: world→chest 方向, TF 发布需用共轭):**
```yaml
world_to_chest_quat:
  left:  [0.7071, 0.0, 0.0, 0.7071]   # 绕 X 轴 +90°
  right: [0.7071, 0.0, 0.0, -0.7071]  # 绕 X 轴 -90°
```

TF 发布用共轭: `get_tf_quaternion()` → `[-qx, -qy, -qz, qw]`

### 臂角 (zsp_para) — 参考臂角平面参数 (NOT 肘部方向!)

**重要: zsp_para 不是肘部方向，而是"参考臂角平面参数"，Y 分量方向与重力方向相反!**

经 FK→IK 闭环验证 (`test/tool/diagnose_zsp_para.py`):
- 左臂 GRV=[0,+9.81,0] (重力=+Y) → zsp_para Y 取负值
- 右臂 GRV=[0,-9.81,0] (重力=-Y) → zsp_para Y 取正值

```yaml
default_zsp_para:
  left:  [0, -1, -0.5, 0, 0, 0]   # Y- (反向重力), Z- (外侧)
  right: [0, 1, -0.5, 0, 0, 0]    # Y+ (反向重力), Z- (外侧)
```

**验证数据 (与 init_joints 的 IK 闭环误差):**
| 值 | 左臂误差 | 右臂误差 | 说明 |
|----|---------|---------|------|
| `[0, -1, -0.5]`/`[0, +1, -0.5]` | 5.1° | 5.1° | 经验最优 ✓ |
| `[0, -1, -1]`/`[0, +1, -1]` | 14.8° | 14.8° | 旧值,可用但偏差较大 |
| `[0, +1, -1]`/`[0, -1, -1]` | 120° | 120° | 错误! 肘部翻转 |

### Arm Tracker 初始位置 (物理正确值，肘部在肩下方)

```python
# 肘部参考位置 (Chest 坐标系), 物理位置: 肩下方30cm + 外侧10cm
left:  [0.2,  0.3,  0.1]  # Left Chest: Y+=下30cm, Z+=左10cm(外侧)
right: [0.2, -0.3,  0.1]  # Right Chest: Y-=下30cm, Z+=右10cm(外侧)
```

几何计算的肘部偏移方向指向重力(下方)，发布给 IK 前需取反: `ik_direction = -direction`。
取反后与 default_zsp_para 方向对齐 (dot ≈ 0.78)。

### Y 分量符号规则 (CRITICAL — 经 IK 闭环验证)

zsp_para 的 Y 分量符号与重力方向 **相反** (NOT 同向!):
- **左臂: Y 负** (GRV=+Y, zsp_para 取反)
- **右臂: Y 正** (GRV=-Y, zsp_para 取反)

注意: 这与 Chest 坐标系中"向下=左Y正/右Y负"的直觉 **相反**。
此规则仅适用于 zsp_para。arm_init_pos 使用物理正确值(肘部在肩下方)，
几何方向发布给 IK 前需取反: `ik_direction = -direction` (在 pico_input_node.py 和 step4 中)。

### zsp_para 涉及的文件清单 (全部从 tianji_robot.yaml 加载!)

| 文件 | 用途 | 加载方式 |
|------|------|---------|
| `tianji_robot.yaml` | **配置源** (Single Source of Truth) | — |
| `config_loader.py` | 配置加载器 | 解析 YAML |
| `robot_config.py` (test/common) | 测试兼容层 | `TianjiConfig.load()` → 导出 `ZSP_PARA`, `DEFAULT_ZSP_DIRECTION` |
| `tianji_world_output_node.py` | 节点默认值 | `TianjiConfig.load()` |
| `cartesian_controller.py` (tianji_world_output) | IK 默认值 | `TianjiConfig.load()` |
| `cartesian_controller.py` (tianji_output) | IK 默认值 | `TianjiConfig.load()` (fallback 硬编码) |
| `pico_input_node.py` | PICO 运行时默认值 | `get_tianji_config()` → `get_default_zsp_direction()` |
| `debug_arm_axis.py` | 调试工具 (被 TF 覆盖) | `TianjiConfig.load()` (fallback 硬编码) |
| `step2_pose_topic_control.py` | 测试: 发布肘部方向 | `DEFAULT_ZSP_DIRECTION` (from robot_config) |
| `step4_visualize_recorded_data.py` | 测试: 可视化 + arm_init_pos | `DEFAULT_ZSP_DIRECTION`, `ARM_INIT_POS` |
| `step5_incremental_control_with_robot.py` | 测试: 增量控制 + arm_init_pos | `DEFAULT_ZSP_DIRECTION`, `ARM_INIT_POS` |
| `step6_arm_angle_stability_test.py` | 测试: `elbow_direction_from_angles()` | 动态计算 |

### arm_init_pos 涉及的文件清单 (全部从 tianji_robot.yaml 加载!)

| 文件 | 加载方式 |
|------|---------|
| `tianji_robot.yaml` | **配置源**: left=[0.2, 0.3, 0.1], right=[0.2, -0.3, 0.1] |
| `pico_input_node.py` | `_tianji_config.arm_init_pos` |
| `step4_visualize_recorded_data.py` | `ARM_INIT_POS` (from robot_config) |
| `step5_incremental_control_with_robot.py` | `ARM_INIT_POS` (from robot_config) |

### PICO Test Scripts

测试脚本在 `src/input_devices/pico_input/test/` 目录，直接从源码运行 (无需 colcon build)。

| Step | 用途 | 需要硬件 |
|------|------|---------|
| step1 | 直接关节控制 | 机器人 |
| step2 | ROS2 Topic 位姿控制 | 机器人 + ROS2 |
| step3 | RViz 可视化 | 机器人 + ROS2 |
| step4 | 可视化录制数据 | ROS2 (无需硬件) |
| step5 | 增量控制真机 | 机器人 |
| step6 | 臂角稳定性测试 | 机器人 (可 dry-run) |

### Shared Library Architecture (共享库架构)

**唯一权威实现**: `tianji_world_output/tianji_world_output/transform_utils.py`

所有坐标/旋转变换函数只在此文件中实现。所有消费者都从这里导入:

| 消费者 | 导入方式 |
|--------|---------|
| `pico_input_node.py` | `from tianji_world_output.transform_utils import ...` |
| `test/common/transform_utils.py` | Re-export wrapper (添加 sys.path 后 `from tianji_world_output.transform_utils import ...`) |
| `step1-6 测试脚本` | `from common.transform_utils import ...` (通过 re-export wrapper) |

**共享库函数一览**:

| 函数 | 用途 |
|------|------|
| `transform_world_to_chest(vec, side)` | World→Chest 位置向量变换 |
| `transform_chest_to_world(vec, side)` | Chest→World 位置向量变换 |
| `get_world_to_chest_rotation(side)` | World→Chest 3x3 旋转矩阵 |
| `get_chest_to_world_rotation(side)` | Chest→World 3x3 旋转矩阵 |
| `get_tf_quaternion(side)` | TF 发布用四元数 (chest→world 共轭) |
| `get_pico_to_robot()` | PICO→Robot 3x3 变换矩阵 (从配置加载) |
| `apply_world_rotation_to_chest_pose(base, delta, side)` | 4步左乘旋转算法 |
| `transform_pico_rotation_to_world(delta, pico_to_robot)` | PICO→World 旋转 (轴角法) |
| `elbow_direction_from_angles(pitch, yaw, side)` | 臂角方向生成 |
| `get_direction_vector_world(mode)` | 运动方向向量查询 |
| `get_rotation_axis_world(mode)` | 旋转轴向量查询 |

**关键设计原则**:
- 零重复: 无内联实现，所有变换都调用共享库
- 单一配置源: `tianji_robot.yaml` (通过 `config_loader.py` 加载)
- 4步左乘旋转算法: `target_chest = R_w2c @ R_delta @ R_c2w @ base_chest`
- PICO→World 旋转使用轴角法 (pico_to_robot 矩阵 det=+1)
- 旋转正方向: 右手定则，从轴正端向原点看逆时针为正 (+X=Roll right, +Y=Pitch down, +Z=Yaw left)
