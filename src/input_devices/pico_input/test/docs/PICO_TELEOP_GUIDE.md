# PICO 遥操作完整指南

**日期**: 2026-02-09
**适用范围**: step4（可视化）和 step5（真机控制）

---

## 目录

1. [概述](#1-概述)
2. [坐标系定义](#2-坐标系定义)
3. [坐标变换链](#3-坐标变换链)
4. [增量控制原理](#4-增量控制原理)
5. [臂角控制](#5-臂角控制)
6. [数学推导详解](#6-数学推导详解)
7. [完整示例](#7-完整示例)
8. [Tracker 映射与配置](#8-tracker-映射与配置)
9. [Step4 与 Step5 对比](#9-step4-与-step5-对比)
10. [常见问题](#10-常见问题)
11. [文件结构](#11-文件结构)

---

## 1. 概述

### 核心数据流

```
PICO Tracker → 坐标变换 → Chest 坐标系 → 机器人控制
```

### 完整转换链

```
PICO OpenXR 增量 → World 坐标系 → Chest 坐标系 → 控制命令
```

这是一个**多级坐标变换链**，每一步都有明确的物理意义。

---

## 2. 坐标系定义

### 2.1 PICO OpenXR 坐标系（右手系）

```
        +Y (上)
         |
         |
         |_________ +X (右)
        /
       /
      +Z (朝向用户)
```

**特点**:
- 以头显为原点
- Tracker 位置在这个坐标系中表示
- 这是 PICO 硬件直接输出的坐标系

### 2.2 Robot/World 坐标系 (ROS REP 103)

```
        +Z (上)
         |
         |
         |_________ +X (前)
        /
       /
      +Y (左)
```

**特点**:
- 固定在机器人基座
- 所有控制命令在这个坐标系中表示
- ROS2 标准坐标系

### 2.3 Chest 坐标系 (机械臂基座)

每只手臂有自己的 chest 坐标系：

**Left Chest** (world_left):
- 位置: `[0, 0.2, 0]` 相对于 World
- 姿态: 绕 X 轴 +90°
  - `+X_chest` = `+X_world` (前)
  - `+Y_chest` = `-Z_world` (下)
  - `+Z_chest` = `+Y_world` (左)

**Right Chest** (world_right):
- 位置: `[0, -0.2, 0]` 相对于 World
- 姿态: 绕 X 轴 -90°
  - `+X_chest` = `+X_world` (前)
  - `+Y_chest` = `+Z_world` (上)
  - `+Z_chest` = `-Y_world` (右)

**四元数表示**:
```python
WORLD_TO_LEFT_QUAT  = [0.7071, 0, 0,  0.7071]  # 绕 X 轴 +90°
WORLD_TO_RIGHT_QUAT = [0.7071, 0, 0, -0.7071]  # 绕 X 轴 -90°
```

**关键理解**: 虽然两个 chest 坐标系内部定义相同，但它们在 world 坐标系中的朝向不同：
- world_left 的 Y+ 在 world 中指向 Z- (向下)
- world_right 的 Y+ 在 world 中指向 Z+ (向上)

这导致"向身体外侧"在两个坐标系中对应不同的 Y 符号。

### 2.4 旋转正方向约定（右手定则）

**判断方法**: 右手大拇指指向轴的正方向，四指弯曲方向 = 正旋转方向

**等价描述**: 从轴的正端向原点看，**逆时针** = 正方向

```
判断示例: 绕 +Y 轴的正旋转

  1. 右手大拇指指向 +Y (向左)
  2. 四指从 +Z (上) 弯曲向 +X (前)
  3. 所以: +Z→+X, +X→-Z
  4. 物理效果: 前方(+X)的东西向下(-Z)移动 = 手指向下压 = Pitch down
```

**Robot World 坐标系旋转效果总结**:

| 旋转 | 正方向变换 | 物理效果 | 记忆 |
|------|-----------|---------|------|
| 绕 +X (Roll) | Y→Z: 左侧→上方 | 手腕顶部向右倾 (Roll right) | 正Roll右倾 |
| 绕 +Y (Pitch) | Z→X: 上方→前方 | 手指尖端向下压 (Pitch down) | 正Pitch低头 |
| 绕 +Z (Yaw) | X→Y: 前方→左方 | 手指尖端向左偏 (Yaw left) | 正Yaw向左 |

**数学验证** (标准旋转矩阵):
```python
Rx(θ)@[0,0,1] = [0, -sinθ, cosθ]   # 上方向右倾 → Roll right  ✓
Ry(θ)@[1,0,0] = [cosθ, 0, -sinθ]   # 前方向下压 → Pitch down  ✓
Rz(θ)@[1,0,0] = [cosθ, sinθ, 0]    # 前方向左偏 → Yaw left    ✓
```

**负方向旋转** (效果相反):

| 旋转 | 物理效果 |
|------|---------|
| 绕 -X | 手腕顶部向左倾 (Roll left) |
| 绕 -Y | 手指尖端向上翘 (Pitch up) |
| 绕 -Z | 手指尖端向右偏 (Yaw right) |

---

## 3. 坐标变换链

### 3.1 PICO → World 变换

```python
PICO_TO_ROBOT = [
    [0, 0, -1],   # Robot X = -PICO Z
    [-1, 0, 0],   # Robot Y = -PICO X
    [0, 1, 0]     # Robot Z = +PICO Y
]
```

**物理意义**：

| 用户动作 | PICO 方向 | 机器人方向 | 转换公式 |
|---------|----------|-----------|---------|
| 往前伸手 | -Z (向前) | +X (前方) | Robot_X = -PICO_Z |
| 往右伸手 | +X (向右) | -Y (右方) | Robot_Y = -PICO_X |
| 往上抬手 | +Y (向上) | +Z (上方) | Robot_Z = +PICO_Y |

**旋转轴变换**（使用同一变换矩阵）:

| PICO 旋转轴 | Robot 旋转轴 | 转换公式 |
|------------|-------------|---------|
| 绕 PICO X  | 绕 Robot -Y | axis_robot = PICO_TO_ROBOT @ axis_pico |
| 绕 PICO Y  | 绕 Robot +Z | |
| 绕 PICO Z  | 绕 Robot -X | |

**Robot World 坐标系 (ROS REP 103 右手定则，从轴正端向原点看逆时针为正) 旋转效果**:

| 旋转轴 | 标准命名 | 实机效果 |
|-------|---------|---------|
| 绕 +X 轴 | +Roll | 手腕向右翻滚 (顶部向右倾) |
| 绕 -X 轴 | -Roll | 手腕向左翻滚 (顶部向左倾) |
| 绕 +Y 轴 | +Pitch | 手指向下压 (低头) |
| 绕 -Y 轴 | -Pitch | 手指向上翘 (抬头) |
| 绕 +Z 轴 | +Yaw | 手指向左偏 (从上看逆时针) |
| 绕 -Z 轴 | -Yaw | 手指向右偏 (从上看顺时针) |

**PICO → Robot 旋转效果对照**:

| PICO 旋转 | Robot 轴 | 标准命名 | 实机效果 |
|----------|---------|---------|---------|
| 绕 PICO +X | 绕 Robot -Y | -Pitch | 手指向上翘 |
| 绕 PICO -X | 绕 Robot +Y | +Pitch | 手指向下压 |
| 绕 PICO +Y | 绕 Robot +Z | +Yaw | 手指向左偏 |
| 绕 PICO -Y | 绕 Robot -Z | -Yaw | 手指向右偏 |
| 绕 PICO +Z | 绕 Robot -X | -Roll | 手腕向左翻滚 |
| 绕 PICO -Z | 绕 Robot +X | +Roll | 手腕向右翻滚 |

### 3.2 World → Chest 变换

```python
# 左臂: 绕 X 轴 +90°
R_world_to_left = [
    [1,  0,  0],
    [0,  0, -1],   # Y_chest = -Z_world
    [0,  1,  0]    # Z_chest = Y_world
]

# 右臂: 绕 X 轴 -90°
R_world_to_right = [
    [1,  0,  0],
    [0,  0,  1],   # Y_chest = Z_world
    [0, -1,  0]    # Z_chest = -Y_world
]
```

**姿态转换（共轭变换）**:
```python
Δrot_chest = R_world_to_chest * Δrot_world * R_world_to_chest^(-1)
```

---

## 4. 增量控制原理

### 4.1 为什么用增量控制？

直接映射 PICO 位置到机器人位置会有问题：
1. 用户和机器人的工作空间不同
2. 初始姿态不一致
3. 需要灵活的起始点

**增量控制解决方案**：
```
目标位姿 = 机器人初始位姿 + (当前PICO位姿 - 初始PICO位姿)
```

### 4.2 计算流程

```
┌─────────────────────────────────────────────────────────────────┐
│ Step 1: PICO 坐标系中的增量                                      │
├─────────────────────────────────────────────────────────────────┤
│ 初始化时记录: init_pose_pico (第一帧数据)                        │
│ 运行时读取:   current_pose_pico                                 │
│                                                                 │
│ 计算增量:                                                       │
│   Δpos_pico = current_pos - init_pos                           │
│   Δrot_pico = current_rot * init_rot^(-1)                      │
│                                                                 │
│ 物理意义: 用户手在 PICO 空间中移动了多少                         │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 2: 转换到 World 坐标系                                      │
├─────────────────────────────────────────────────────────────────┤
│ 位置增量:                                                       │
│   Δpos_world = pico_to_robot @ Δpos_pico                       │
│                                                                 │
│ 姿态增量（轴角方法）:                                            │
│   axis_world = pico_to_robot @ axis_pico                       │
│   Δrot_world = R.from_rotvec(axis_world * angle)               │
│                                                                 │
│ 物理意义: 增量在机器人坐标系中的表示                             │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 3: 转换到 Chest 坐标系                                      │
├─────────────────────────────────────────────────────────────────┤
│ 位置增量:                                                       │
│   Δpos_chest = R_world_to_chest @ Δpos_world                   │
│                                                                 │
│ 姿态增量:                                                       │
│   Δrot_chest = R_chest * Δrot_world * R_chest^(-1)             │
│                                                                 │
│ 物理意义: 增量在机械臂基座坐标系中的表示（FK/IK 需要）           │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 4: 应用到机器人初始位姿                                     │
├─────────────────────────────────────────────────────────────────┤
│ 目标位置: target_pos = TIANJI_INIT_POS + Δpos_chest            │
│ 目标姿态: target_rot = TIANJI_INIT_ROT * Δrot_chest            │
│                                                                 │
│ 物理意义: 机器人末端应该到达的目标位姿（在 Chest 中）            │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 5: 发布控制命令                                             │
├─────────────────────────────────────────────────────────────────┤
│ Topic 发布: /left_arm_target_pose, /right_arm_target_pose       │
│ 直接控制: controller.move_to_pose_direct(...)                   │
│                                                                 │
│ 接收方: tianji_world_output_node                                │
│   → IK 求解 → 发送关节角到机器人                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. 臂角控制

### 5.1 什么是臂角？

7自由度机械臂到达同一末端位姿有无穷多解（零空间）。
臂角约束指定肘部的期望方向，让机器人选择特定姿态。

```
想象场景：
- 手握住门把手（末端固定）
- 肘部可以朝下（沉肘）或朝外（抬肘）
- 臂角就是控制肘部指向哪个方向
```

### 5.2 几何计算方法

```
            肩膀 (shoulder = [0,0,0])
              ●
             /|\
            / | \
           /  |  \
    肩-肘 /   |   \ 肩-腕
         /    |    \
        ●─────●─────●
     肘部   投影点   手腕
    (elbow) (proj)  (wrist)

        ←────→
      肘部偏移向量
    (elbow_direction)
```

```python
def compute_elbow_direction(shoulder, wrist, elbow):
    # 1. 计算肩-腕单位向量（手臂主轴）
    shoulder_to_wrist = wrist - shoulder
    sw_unit = normalize(shoulder_to_wrist)

    # 2. 将肘部投影到肩-腕连线上
    proj_length = dot(elbow - shoulder, sw_unit)
    proj_point = shoulder + proj_length * sw_unit

    # 3. 肘部偏移向量
    elbow_offset = elbow - proj_point

    # 4. 归一化
    direction = normalize(elbow_offset)

    return direction
```

**重要**: 几何 elbow_direction 与 IK 的 zsp_para **不是同一个东西**！
- 几何方向的 Y 分量指向**重力方向**（沉肘时肘部在下方）
- zsp_para 的 Y 分量指向**反重力方向**（IK 求解器内部约定）
- 因此发布给 IK 前需取反: `ik_direction = -direction`

**各模块使用方式**:
- **pico_input_node**: 动态计算。调用 `compute_elbow_direction()` 取反后发布到 topic
- **step4**: 动态计算。同上，同时发布 MarkerArray 用于 RViz 可视化
- **step5**: 静态默认值。直接使用 `DEFAULT_ZSP_DIRECTION`（经 FK→IK 闭环验证）

### 5.3 沉肘方向详解

**沉肘 = 肘部向下 + 肘部向外侧**

```
俯视图（从上往下看）：

        前 (X+)
          ↑
          │
    ┌─────┼─────┐
    │     │     │
左臂●─────┼─────●右臂
    ↓     │     ↓
  向外侧  身体  向外侧
  (右)         (左)
          │
        后
```

**Chest 坐标系理解（IK 使用的坐标系）：**

```
Chest 坐标系定义:
  Left Chest:  +Y = -Z_world (向下), +Z = +Y_world (向左)
  Right Chest: +Y = +Z_world (向上), +Z = -Y_world (向右)

沉肘方向（Chest 坐标系）:
  左臂: +Y = 向下, -Z = 向右(外侧)  → [0, +1, -1]
  右臂: -Y = 向下, -Z = 向左(外侧)  → [0, -1, -1]
```

**zsp_para 数值（参考臂角平面参数，NOT 肘部方向！）：**

注意: zsp_para 的 Y 分量方向与重力方向 **相反**（经 FK→IK 闭环验证）：
```python
左臂: [0, -1, -0.5, 0, 0, 0]  # Y- (反向重力), Z- (外侧)
右臂: [0, 1, -0.5, 0, 0, 0]   # Y+ (反向重力), Z- (外侧)
```

验证工具: `test/tool/diagnose_zsp_para.py`（FK→IK 闭环测试）

**归一化：**
```python
左臂: [0, -1, -0.5] / norm ≈ [0, -0.894, -0.447]
右臂: [0, +1, -0.5] / norm ≈ [0, +0.894, -0.447]
```

### 5.4 可视化说明 (RViz)

```
红色箭头: 肩 → 腕 (手臂主轴)
绿色箭头: 肩 → 肘 (实际肘部位置)
蓝色箭头: 投影点 → 肘部偏移方向 (发给 IK 的 elbow_direction)
黄色小球: 投影点
紫色线:   投影点 → 实际肘部
```

---

## 6. 数学推导详解

### 6.1 PICO → World 姿态转换（轴角方法）

**为什么使用轴角方法？**

`pico_to_robot` 矩阵是正交矩阵（行列式 = +1），轴角法直接变换旋转轴向量，概念清晰且高效。

**解决方案：轴角方法**

```python
# 1. 将旋转转换为轴角表示
rotvec = Δrot_pico.as_rotvec()  # axis * angle
angle = np.linalg.norm(rotvec)
axis = rotvec / angle if angle > 0 else [0, 0, 0]

# 2. 变换旋转轴（pico_to_robot 正交矩阵 det=+1）
axis_world = pico_to_robot @ axis

# 3. 保持角度不变，重建旋转
rotvec_world = axis_world * angle
Δrot_world = R.from_rotvec(rotvec_world)
```

**物理意义** (根据 ROS REP 103 右手定则，从轴正端向原点看逆时针为正):
- PICO 绕 +X 轴旋转 → 机器人绕 -Y 轴旋转 → 手指向上翘 (Pitch up)
- PICO 绕 -X 轴旋转 → 机器人绕 +Y 轴旋转 → 手指向下压 (Pitch down)
- PICO 绕 Y 轴旋转 → 机器人绕 +Z 轴旋转 (Yaw: 手指向左偏)
- PICO 绕 Z 轴旋转 → 机器人绕 -X 轴旋转 (Roll: 手腕向左翻滚)

---

## 7. 完整示例

### 场景: 用户手向前移动 5cm

**Step 1: PICO 增量**
```python
# 用户手向前（-Z）
init_pos_pico = [0.5, 0.2, 1.0]
current_pos_pico = [0.5, 0.2, 0.95]  # -Z 方向 5cm
Δpos_pico = [0, 0, -0.05]
```

**Step 2: 转换到 World**
```python
Δpos_world = pico_to_robot @ [0, 0, -0.05]
           = [0.05, 0, 0]  # 前方 5cm
```

**Step 3: 转换到 Left Chest**
```python
Δpos_left_chest = R_world_to_left @ [0.05, 0, 0]
                = [0.05, 0, 0]  # X_chest: 前方
```

**Step 4: 应用到机器人初始位姿**
```python
# 机器人初始位置（左臂，来自 tianji_robot.yaml）
TIANJI_INIT_POS['left'] = [0.5733, 0.2237, 0.2762]

# 目标位置
target_pos = [0.5733, 0.2237, 0.2762] + [0.05, 0, 0]
           = [0.6233, 0.2237, 0.2762]  # 向前移动 5cm
```

---

## 8. Tracker 映射与配置

### 8.1 Tracker SN 映射

| Tracker SN | 角色 | 用途 |
|------------|------|------|
| 190058 | pico_left_wrist | 控制左臂末端位姿 |
| 190600 | pico_right_wrist | 控制右臂末端位姿 |
| 190046 | pico_left_arm | 计算左臂肘部方向 |
| 190023 | pico_right_arm | 计算右臂肘部方向 |

### 8.2 Tracker 初始位置（关键！）

**Wrist Tracker 和 Arm Tracker 使用不同的初始位置**：

```python
# Wrist Tracker: 映射到机器人末端位置 (来自 tianji_robot.yaml)
wrist_init_pos = TIANJI_INIT_POS[side]
# 左臂: [0.5733, 0.2237, 0.2762]
# 右臂: [0.5733, -0.2237, 0.2762]

# Arm Tracker: 映射到肘部参考位置 (来自 tianji_robot.yaml 的 arm_init_pos)
# 这些值手动设定，使 arm tracker 在 chest 坐标系中位于肘部下方外侧
arm_init_pos = ARM_INIT_POS[side]
# 左臂: [0.2, 0.3, 0.2]
# 右臂: [0.2, -0.3, 0.2]
```

**为什么不同？**
- Wrist tracker 控制末端位姿，需要映射到机器人手腕位置
- Arm tracker 用于增量控制的参考位置（pico_input_node 中的 arm tracker 增量基准）

**Arm Tracker 初始位置说明（Chest 坐标系）**：
```
Left Chest (+90° 绕 X): +Y = 向下, +Z = 向左(外侧)
Right Chest (-90° 绕 X): +Y = 向上, +Z = 向右(外侧)

左臂: [0.2, 0.3, 0.2]
      # X=0.2:  向前 20cm
      # Y=0.3:  向下 30cm (在 Left Chest 中 +Y = 向下)
      # Z=0.2:  向左 20cm (外侧)

右臂: [0.2, -0.3, 0.2]
      # X=0.2:  向前 20cm
      # Y=-0.3: 向下 30cm (在 Right Chest 中 -Y = 向下)
      # Z=0.2:  向右 20cm (外侧)
```

**臂角控制说明**：

- **pico_input_node / step4**: 从 arm tracker 位置**动态计算**肘部偏移方向
  （几何投影 → 取反 → 发布到 `/left_arm_elbow_direction`）
- **step5**: 使用静态 `DEFAULT_ZSP_DIRECTION`（经 FK→IK 闭环验证的固定值）
- 几何 elbow direction 计算也用于 step4 的 RViz MarkerArray 可视化

---

## 9. Step4 与 Step5 对比

### Step4: 可视化验证

```
目的: 离线验证坐标变换，不需要真机

输入: 录制数据文件 (trackingData_*.txt)
输出:
  - TF: world_left/right → pico_*_wrist, pico_*_arm
  - Topics: /left_arm_target_pose, /right_arm_target_pose
  - Markers: /elbow_angle_visualization

验证要点:
  ✓ pico_left_arm 和 pico_right_arm 都在各自 chest 的下方
  ✓ 蓝色 elbow direction 箭头都指向下方（沉肘）
  ✓ 左右臂对称
```

### Step5: 真机控制

```
目的: 使用录制数据控制真实机器人

输入: 录制数据文件
输出:
  - 机器人末端位姿指令
  - zsp_para 臂角约束

安全措施:
  --dry-run     仅打印，不控制
  --speed 0.3   慢速回放
  --left-only   仅控制左臂
  --right-only  仅控制右臂
```

### Step5 验证测试命令

使用 `trackingData_whole_data.txt` 中的特定帧段验证坐标映射：

**坐标映射速查表**：
| 用户动作 | PICO 变化 | Robot 变化 | 机器人运动 |
|---------|-----------|-----------|-----------|
| 向前伸手 | Z 减小 (-Z) | X 增加 (+X) | 向前 |
| 向后缩手 | Z 增加 (+Z) | X 减小 (-X) | 向后 |
| 向右移动 | X 增加 (+X) | Y 减小 (-Y) | 向右 |
| 向左移动 | X 减小 (-X) | Y 增加 (+Y) | 向左 |
| 向上抬手 | Y 增加 (+Y) | Z 增加 (+Z) | 向上 |
| 向下放手 | Y 减小 (-Y) | Z 减小 (-Z) | 向下 |

**前后方向测试** (PICO Z → Robot X):
```bash
# 向后运动: PICO Z +16cm → Robot X -16cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2180 --end-frame 2230 --left-only --speed 0.5 -v

# 向前运动: PICO Z -10cm → Robot X +10cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2275 --end-frame 2325 --left-only --speed 0.5 -v
```

**左右方向测试** (PICO X → Robot Y):
```bash
# 向右运动: PICO X +18cm → Robot Y -18cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2417 --end-frame 2467 --left-only --speed 0.5 -v

# 向左运动: PICO X -19cm → Robot Y +19cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 1140 --end-frame 1160 --left-only --speed 0.5 -v
```

**上下方向测试** (PICO Y → Robot Z):
```bash
# 向下运动: PICO Y -20cm → Robot Z -20cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2062 --end-frame 2112 --left-only --speed 0.5 -v

# 向上运动: PICO Y +19cm → Robot Z +19cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 1958 --end-frame 2008 --right-only --speed 0.5 -v
```

**运动分析工具**:
```bash
python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt
```

### 与 step2_pose_topic_control.py 的对比

| 方面 | step2 | pico_input_node |
|------|-------|-----------------|
| **frame_id** | `'world_left'` | `'world'` |
| **位姿数值** | 直接在 chest 中 | 增量 + 转换到 chest |
| **控制方式** | 绝对位姿 | 增量位姿 |
| **初始化** | 不需要 | 需要（记录初始位姿） |

**两者都是正确的！** 只是表示方式不同。

---

## 10. 常见问题

### Q1: 为什么 Y 分量符号不同？

因为 world_left 和 world_right 的 Y 轴在 world 坐标系中指向相反方向。
"向身体外侧"这个物理方向，在两个 chest 坐标系中对应不同的 Y 符号。

### Q2: 如何验证坐标变换是否正确？

运行 step4，在 RViz 中检查:
1. pico_left_arm 和 pico_right_arm 都在下方 ✓
2. 蓝色箭头都指向下方 ✓
3. 左右对称 ✓

### Q3: X 分量为什么是 0？

X 轴是向前的方向。沉肘时肘部不向前或向后偏，只向下+向外，所以 X=0。

### Q4: 为什么姿态转换要用轴角方法？

pico_to_robot 是坐标轴映射矩阵（行列式 = +1）。
轴角方法直接变换旋转轴向量，概念清晰且高效。

---

## 11. 文件结构

```
tianji_world_output/tianji_world_output/
├── __init__.py                      # 包入口（公共接口文档）
├── config_loader.py                 # 统一配置加载器 (tianji_robot.yaml)
├── transform_utils.py               # ★ 坐标变换共享库（唯一权威实现）
├── cartesian_controller.py          # 笛卡尔空间控制器
├── tianji_world_output_node.py      # ROS2 输出节点
├── fx_kine.py                       # FK/IK 运动学 (re-export from tianji_output)
├── fx_robot.py                      # 机器人底层控制 (re-export from tianji_output)
├── structure_data.py                # ctypes 结构体 (re-export from tianji_output)
├── robot_structures.py              # 机器人结构体 (re-export from tianji_output)
└── config/
    ├── tianji_robot.yaml            # ★ 统一配置 (Single Source of Truth)
    └── ccs_m6.MvKDCfg              # 运动学配置文件

pico_input/pico_input/
├── __init__.py                      # 包入口（公共接口文档）
├── pico_input_node.py               # ROS2 输入节点（增量控制）
├── incremental_controller.py        # 增量控制器（纯计算，无 ROS2 依赖）
├── data_source/                     # 数据源抽象层 + 实现
│   ├── __init__.py                  # re-export (DataSource, TrackerData, HeadsetData)
│   ├── base.py                      # ABC 基类 (DataSource, TrackerData, HeadsetData)
│   ├── live_data_source.py          # 真实 PICO SDK 数据源
│   └── recorded_data_source.py      # 录制文件回放数据源
└── xrobotoolkit_client.py           # PICO SDK 封装

pico_input/test/
├── common/
│   ├── robot_config.py              # 测试兼容层（从 tianji_robot.yaml 加载）
│   ├── robot_lifecycle.py           # 机器人上电/下电生命周期管理
│   └── transform_utils.py           # Re-export wrapper（委托到 tianji_world_output）
├── docs/
│   └── PICO_TELEOP_GUIDE.md         # 本文档
├── tool/
│   ├── move_to_init_pose.py         # 移动机器人到初始姿态
│   ├── verify_fk_values.py          # 验证 FK 计算值
│   └── diagnose_zsp_para.py         # FK→IK 闭环诊断 zsp_para
├── step1_direct_joint_control.py    # 直接关节控制（基础测试）
├── step2_pose_topic_control.py      # ROS2 Topic 位姿控制
├── step3_visualize_in_rviz.py       # RViz 坐标系可视化
├── step4_visualize_recorded_data.py # PICO 录制数据可视化
├── step5_incremental_control_with_robot.py  # PICO 增量控制真机
└── step6_arm_angle_stability_test.py # 臂角稳定性测试（模拟数据）

pico_input/record/
├── trackingData_sample_static.txt   # 高质量静态测试数据 (150帧)
├── trackingData_whole_data.txt      # 完整测试数据 (5780帧)
├── trackingData_head_tracker.txt    # 头部追踪器数据
├── trackingData_head_tracker_static.txt  # 头部追踪器静态数据
├── analyze_motion_data.py           # 运动数据分析工具
└── visualize_pico_raw.py            # PICO 原始数据可视化工具
```

### 共享库架构

**唯一权威实现**: `tianji_world_output/transform_utils.py`

所有坐标/旋转变换函数只在此文件实现，所有消费者都从这里导入。
`test/common/transform_utils.py` 仅是 re-export wrapper，零实现代码。

### 共享库 (transform_utils.py) 函数索引

编写新程序时，从 `common.transform_utils`（测试脚本）或
`tianji_world_output.transform_utils`（ROS2 节点）导入以下函数：

```python
from common.transform_utils import (
    # 位置变换
    transform_world_to_chest,           # World→Chest 向量: [x,y,z] → [x,-z,y] / [x,z,-y]
    transform_chest_to_world,           # Chest→World 向量 (逆变换)

    # 旋转变换
    get_world_to_chest_rotation,        # World→Chest 旋转矩阵 (3x3)
    get_chest_to_world_rotation,        # Chest→World 旋转矩阵 (3x3)
    apply_world_rotation_to_chest_pose, # 在 World 中旋转，返回 Chest 姿态（4步法）
    transform_pico_rotation_to_world,   # PICO→World 旋转变换（轴角法）

    # TF 发布
    get_tf_quaternion,                  # TF 发布用四元数 (chest→world 共轭)

    # 配置查询
    get_pico_to_robot,                  # PICO→Robot 3x3 变换矩阵 (从配置加载)

    # 臂角控制
    elbow_direction_from_angles,        # 从角度生成 zsp_para 方向向量
)
```

**常用调用示例**:
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. 位置增量: World→Chest
displacement_chest = transform_world_to_chest(np.array([dx, dy, dz]), side)
target_pos = init_pos_chest + displacement_chest

# 2. 姿态旋转: 在 World 中旋转 30°，结果在 Chest 中
R_delta = R.from_rotvec([0, np.radians(30), 0])  # 绕 +Y = Pitch down
target_rot = apply_world_rotation_to_chest_pose(init_rot_chest, R_delta, side)

# 3. PICO→World 旋转变换
R_world = transform_pico_rotation_to_world(R_pico_delta, pico_to_robot)

# 4. 臂角: 默认沉肘
direction = elbow_direction_from_angles(pitch=0, yaw=45, side='left')
zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]
```

### Step 说明

| Step | 功能 | 需要硬件 |
|------|------|---------|
| step1 | 直接关节控制，验证 SDK 连接 | 机器人 |
| step2 | ROS2 Topic 位姿控制 | 机器人 + ROS2 |
| step3 | RViz 可视化机器人位姿 | 机器人 + ROS2 |
| step4 | 可视化 PICO 录制数据 | ROS2（无需硬件）|
| step5 | 使用 PICO 数据控制真机 | 机器人 |
| step6 | 臂角稳定性测试（模拟数据）| 机器人（可 dry-run）|

---

## 总结

### 核心转换链

```
PICO OpenXR 增量
    ↓ (pico_to_robot 矩阵)
天机 World 增量
    ↓ (world_to_chest 旋转)
天机 Chest 增量
    ↓ (加到 TIANJI_INIT_POS)
目标位姿
    ↓ (Topic 发布 / 直接控制)
tianji_world_output_node (IK)
    ↓
关节角控制
```

### 关键点

1. **增量控制**: 用户手移动多少，机器人手移动多少
2. **多级转换**: PICO → World → Chest，每一步都有明确物理意义
3. **坐标系对齐**: 使用转换矩阵对齐不同坐标系的轴方向
4. **姿态转换**: 使用轴角方法变换旋转轴（pico_to_robot det=+1，正交矩阵）
5. **臂角控制**: 通过 zsp_para 指定参考臂角平面（注意: Y 分量与重力方向相反）

---

**核心公式**: `Target = InitPose + Transform_Chain(PICO_Delta)`

**文档更新**: 2026-02-09
