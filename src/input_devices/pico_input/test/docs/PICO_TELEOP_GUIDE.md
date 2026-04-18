
# PICO Teleoperation Complete Guide

**Date**: 2026-02-09
**Scope**: step4 (visualization) and step5 (real robot control)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Coordinate System Definitions](#2-coordinate-system-definitions)
3. [Coordinate Transform Chain](#3-coordinate-transform-chain)
4. [Incremental Control Principle](#4-incremental-control-principle)
5. [Arm Angle Control](#5-arm-angle-control)
6. [Detailed Mathematical Derivation](#6-detailed-mathematical-derivation)
7. [Complete Example](#7-complete-example)
8. [Tracker Mapping and Configuration](#8-tracker-mapping-and-configuration)
9. [Step4 vs Step5 Comparison](#9-step4-vs-step5-comparison)
10. [FAQ](#10-faq)
11. [File Structure](#11-file-structure)

---

## 1. Overview

### Core Data Flow

```text
PICO Tracker → Coordinate Transform → Chest Frame → Robot Control
```

### Complete Transform Chain

```text
PICO OpenXR Delta → World Frame → Chest Frame → Control Command
```

This is a **multi-stage coordinate transform chain**, where each step has a clear physical meaning.

---

## 2. Coordinate System Definitions

### 2.1 PICO OpenXR Coordinate System (Right-Handed)

```text
        +Y (up)
         |
         |
         |_________ +X (right)
        /
       /
      +Z (toward user)
```

**Characteristics**:
- Origin at the headset
- Tracker positions are expressed in this coordinate system
- This is the coordinate system directly output by PICO hardware

### 2.2 Robot/World Coordinate System (ROS REP 103)

```text
        +Z (up)
         |
         |
         |_________ +X (forward)
        /
       /
      +Y (left)
```

**Characteristics**:
- Fixed at the robot base
- All control commands are expressed in this coordinate system
- ROS2 standard coordinate system

### 2.3 Chest Coordinate System (Robotic Arm Base)

Each arm has its own chest coordinate system:

**Left Chest** (world_left):
- Position: `[0, 0.2, 0]` relative to World
- Orientation: +90 degrees rotation around X axis
  - `+X_chest` = `+X_world` (forward)
  - `+Y_chest` = `-Z_world` (down)
  - `+Z_chest` = `+Y_world` (left)

**Right Chest** (world_right):
- Position: `[0, -0.2, 0]` relative to World
- Orientation: -90 degrees rotation around X axis
  - `+X_chest` = `+X_world` (forward)
  - `+Y_chest` = `+Z_world` (up)
  - `+Z_chest` = `-Y_world` (right)

**Quaternion Representation**:
```python
WORLD_TO_LEFT_QUAT  = [0.7071, 0, 0,  0.7071]  # +90° around X axis
WORLD_TO_RIGHT_QUAT = [0.7071, 0, 0, -0.7071]  # -90° around X axis
```

**Key Insight**: Although both chest coordinate systems have identical internal definitions, their orientations in the world coordinate system differ:
- world_left Y+ points toward Z- in world (downward)
- world_right Y+ points toward Z+ in world (upward)

This causes "outward from the body" to correspond to different Y signs in the two coordinate systems.

### 2.4 Positive Rotation Direction Convention (Right-Hand Rule)

**How to determine**: Point the right thumb along the positive axis direction; the curl of the fingers = positive rotation direction

**Equivalent description**: Looking from the positive end of the axis toward the origin, **counterclockwise** = positive direction

```text
Example: Positive rotation around +Y axis

  1. Right thumb points toward +Y (left)
  2. Fingers curl from +Z (up) toward +X (forward)
  3. Therefore: +Z->+X, +X->-Z
  4. Physical effect: Objects in front (+X) move downward (-Z) = fingers press down = Pitch down
```

**Robot World Coordinate System Rotation Effects Summary**:

| Rotation | Positive Direction Transform | Physical Effect | Mnemonic |
|----------|------------------------------|-----------------|----------|
| Around +X (Roll) | Y->Z: left side->upward | Wrist top tilts right (Roll right) | Positive Roll tilts right |
| Around +Y (Pitch) | Z->X: upward->forward | Fingertips press down (Pitch down) | Positive Pitch looks down |
| Around +Z (Yaw) | X->Y: forward->left | Fingertips deflect left (Yaw left) | Positive Yaw turns left |

**Mathematical Verification** (standard rotation matrices):
```python
Rx(θ)@[0,0,1] = [0, -sinθ, cosθ]   # Upward tilts right → Roll right  ✓
Ry(θ)@[1,0,0] = [cosθ, 0, -sinθ]   # Forward presses down → Pitch down  ✓
Rz(θ)@[1,0,0] = [cosθ, sinθ, 0]    # Forward deflects left → Yaw left    ✓
```

**Negative Direction Rotations** (opposite effects):

| Rotation | Physical Effect |
|----------|-----------------|
| Around -X | Wrist top tilts left (Roll left) |
| Around -Y | Fingertips tilt up (Pitch up) |
| Around -Z | Fingertips deflect right (Yaw right) |

---

## 3. Coordinate Transform Chain

### 3.1 PICO -> World Transform

```python
PICO_TO_ROBOT = [
    [0, 0, -1],   # Robot X = -PICO Z
    [-1, 0, 0],   # Robot Y = -PICO X
    [0, 1, 0]     # Robot Z = +PICO Y
]
```

**Physical Meaning**:

| User Action | PICO Direction | Robot Direction | Transform Formula |
|-------------|----------------|-----------------|-------------------|
| Reach forward | -Z (forward) | +X (forward) | Robot_X = -PICO_Z |
| Reach right | +X (right) | -Y (right) | Robot_Y = -PICO_X |
| Raise hand up | +Y (up) | +Z (up) | Robot_Z = +PICO_Y |

**Rotation Axis Transform** (using the same transform matrix):

| PICO Rotation Axis | Robot Rotation Axis | Transform Formula |
|--------------------|---------------------|-------------------|
| Around PICO X | Around Robot -Y | axis_robot = PICO_TO_ROBOT @ axis_pico |
| Around PICO Y | Around Robot +Z | |
| Around PICO Z | Around Robot -X | |

**Robot World Coordinate System (ROS REP 103 right-hand rule, counterclockwise is positive when looking from positive axis end toward origin) Rotation Effects**:

| Rotation Axis | Standard Name | Real Robot Effect |
|---------------|---------------|-------------------|
| Around +X axis | +Roll | Wrist rolls right (top tilts right) |
| Around -X axis | -Roll | Wrist rolls left (top tilts left) |
| Around +Y axis | +Pitch | Fingers press down (look down) |
| Around -Y axis | -Pitch | Fingers tilt up (look up) |
| Around +Z axis | +Yaw | Fingers deflect left (counterclockwise from top view) |
| Around -Z axis | -Yaw | Fingers deflect right (clockwise from top view) |

**PICO -> Robot Rotation Effect Comparison**:

| PICO Rotation | Robot Axis | Standard Name | Real Robot Effect |
|---------------|------------|---------------|-------------------|
| Around PICO +X | Around Robot -Y | -Pitch | Fingers tilt up |
| Around PICO -X | Around Robot +Y | +Pitch | Fingers press down |
| Around PICO +Y | Around Robot +Z | +Yaw | Fingers deflect left |
| Around PICO -Y | Around Robot -Z | -Yaw | Fingers deflect right |
| Around PICO +Z | Around Robot -X | -Roll | Wrist rolls left |
| Around PICO -Z | Around Robot +X | +Roll | Wrist rolls right |

### 3.2 World -> Chest Transform

```python
# Left arm: +90° around X axis
R_world_to_left = [
    [1,  0,  0],
    [0,  0, -1],   # Y_chest = -Z_world
    [0,  1,  0]    # Z_chest = Y_world
]

# Right arm: -90° around X axis
R_world_to_right = [
    [1,  0,  0],
    [0,  0,  1],   # Y_chest = Z_world
    [0, -1,  0]    # Z_chest = -Y_world
]
```

**Orientation Transform (conjugation)**:
```python
Δrot_chest = R_world_to_chest * Δrot_world * R_world_to_chest^(-1)
```

---

## 4. Incremental Control Principle

### 4.1 Why Use Incremental Control?

Directly mapping PICO positions to robot positions has issues:
1. The user and robot have different workspaces
2. Initial poses are inconsistent
3. A flexible starting point is needed

**Incremental Control Solution**:
```python
Target Pose = Robot Initial Pose + (Current PICO Pose - Initial PICO Pose)
```

### 4.2 Computation Flow

```text
┌─────────────────────────────────────────────────────────────────┐
│ Step 1: Delta in PICO Coordinate System                         │
├─────────────────────────────────────────────────────────────────┤
│ Record at initialization: init_pose_pico (first frame data)     │
│ Read at runtime:          current_pose_pico                     │
│                                                                 │
│ Compute delta:                                                  │
│   Δpos_pico = current_pos - init_pos                           │
│   Δrot_pico = current_rot * init_rot^(-1)                      │
│                                                                 │
│ Physical meaning: How much the user's hand moved in PICO space  │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 2: Transform to World Coordinate System                    │
├─────────────────────────────────────────────────────────────────┤
│ Position delta:                                                 │
│   Δpos_world = pico_to_robot @ Δpos_pico                       │
│                                                                 │
│ Orientation delta (axis-angle method):                          │
│   axis_world = pico_to_robot @ axis_pico                       │
│   Δrot_world = R.from_rotvec(axis_world * angle)               │
│                                                                 │
│ Physical meaning: Delta expressed in robot coordinate system    │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 3: Transform to Chest Coordinate System                    │
├─────────────────────────────────────────────────────────────────┤
│ Position delta:                                                 │
│   Δpos_chest = R_world_to_chest @ Δpos_world                   │
│                                                                 │
│ Orientation delta:                                              │
│   Δrot_chest = R_chest * Δrot_world * R_chest^(-1)             │
│                                                                 │
│ Physical meaning: Delta in arm base frame (needed for FK/IK)    │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 4: Apply to Robot Initial Pose                             │
├─────────────────────────────────────────────────────────────────┤
│ Target position: target_pos = TIANJI_INIT_POS + Δpos_chest     │
│ Target orientation: target_rot = TIANJI_INIT_ROT * Δrot_chest  │
│                                                                 │
│ Physical meaning: Target end-effector pose (in Chest frame)     │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Step 5: Publish Control Command                                 │
├─────────────────────────────────────────────────────────────────┤
│ Topic publish: /left_arm_target_pose, /right_arm_target_pose    │
│ Direct control: controller.move_to_pose_direct(...)             │
│                                                                 │
│ Receiver: tianji_world_output_node                              │
│   → IK solve → Send joint angles to robot                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. Arm Angle Control

### 5.1 What is Arm Angle?

A 7-DOF robotic arm has infinitely many solutions (null space) for the same end-effector pose.
The arm angle constraint specifies the desired elbow direction, allowing the robot to choose a specific configuration.

```text
Imagine this scenario:
- Hand grips a door handle (end-effector fixed)
- Elbow can point downward (elbow down) or outward (elbow raised)
- Arm angle controls which direction the elbow points
```

### 5.2 Geometric Calculation Method

```python
            Shoulder (shoulder = [0,0,0])
              ●
             /|\
            / | \
           /  |  \
  Shoulder/   |   \ Shoulder
  -Elbow /    |    \ -Wrist
        ●─────●─────●
     Elbow  Projection Wrist
    (elbow)  (proj)   (wrist)

        ←────→
    Elbow offset vector
    (elbow_direction)
```

```python
def compute_elbow_direction(shoulder, wrist, elbow):
    # 1. Compute shoulder-to-wrist unit vector (arm main axis)
    shoulder_to_wrist = wrist - shoulder
    sw_unit = normalize(shoulder_to_wrist)

    # 2. Project elbow onto the shoulder-wrist line
    proj_length = dot(elbow - shoulder, sw_unit)
    proj_point = shoulder + proj_length * sw_unit

    # 3. Elbow offset vector
    elbow_offset = elbow - proj_point

    # 4. Normalize
    direction = normalize(elbow_offset)

    return direction
```

**Important**: The geometric elbow_direction and the IK zsp_para are **NOT the same thing**!
- The Y component of the geometric direction points toward **gravity** (elbow is below when in elbow-down pose)
- The Y component of zsp_para points toward **anti-gravity** (IK solver internal convention)
- Therefore, negate before publishing to IK: `ik_direction = -direction`

**Usage in each module**:
- **pico_input_node**: Dynamic computation. Calls `compute_elbow_direction()`, negates, then publishes to topic
- **step4**: Dynamic computation. Same as above, also publishes MarkerArray for RViz visualization
- **step5**: Static default value. Directly uses `DEFAULT_ZSP_DIRECTION` (verified through FK->IK closed-loop)

### 5.3 Elbow-Down Direction Explained

**Elbow down = elbow pointing downward + elbow pointing outward**

```text
Top view (looking from above):

        Front (X+)
          ↑
          │
    ┌─────┼─────┐
    │     │     │
Left●─────┼─────●Right
arm ↓     │     ↓ arm
  Outward Body  Outward
  (right)       (left)
          │
        Back
```

**Chest Coordinate System Understanding (coordinate system used by IK):**

```text
Chest coordinate system definition:
  Left Chest:  +Y = -Z_world (down), +Z = +Y_world (left)
  Right Chest: +Y = +Z_world (up), +Z = -Y_world (right)

Elbow-down direction (Chest coordinate system):
  Left arm:  +Y = down, -Z = right (outward)  → [0, +1, -1]
  Right arm: -Y = down, -Z = left (outward)   → [0, -1, -1]
```

**zsp_para Values (reference arm angle plane parameters, NOT elbow direction!):**

Note: The Y component direction of zsp_para is **opposite** to gravity direction (verified through FK->IK closed-loop):
```python
Left arm:  [0, -1, -0.5, 0, 0, 0]  # Y- (anti-gravity), Z- (outward)
Right arm: [0, 1, -0.5, 0, 0, 0]   # Y+ (anti-gravity), Z- (outward)
```

Verification tool: `test/tool/diagnose_zsp_para.py` (FK->IK closed-loop test)

**Normalization:**
```python
Left arm:  [0, -1, -0.5] / norm ≈ [0, -0.894, -0.447]
Right arm: [0, +1, -0.5] / norm ≈ [0, +0.894, -0.447]
```

### 5.4 Visualization Notes (RViz)

```text
Red arrow:    Shoulder → Wrist (arm main axis)
Green arrow:  Shoulder → Elbow (actual elbow position)
Blue arrow:   Projection point → Elbow offset direction (elbow_direction sent to IK)
Yellow sphere: Projection point
Purple line:  Projection point → Actual elbow
```

---

## 6. Detailed Mathematical Derivation

### 6.1 PICO -> World Orientation Transform (Axis-Angle Method)

**Why use the axis-angle method?**

The `pico_to_robot` matrix is an orthogonal matrix (determinant = +1). The axis-angle method directly transforms the rotation axis vector, which is conceptually clear and efficient.

**Solution: Axis-Angle Method**

```python
# 1. Convert rotation to axis-angle representation
rotvec = Δrot_pico.as_rotvec()  # axis * angle
angle = np.linalg.norm(rotvec)
axis = rotvec / angle if angle > 0 else [0, 0, 0]

# 2. Transform rotation axis (pico_to_robot orthogonal matrix det=+1)
axis_world = pico_to_robot @ axis

# 3. Keep angle unchanged, reconstruct rotation
rotvec_world = axis_world * angle
Δrot_world = R.from_rotvec(rotvec_world)
```

**Physical Meaning** (per ROS REP 103 right-hand rule, counterclockwise is positive when looking from positive axis end toward origin):
- PICO rotation around +X axis -> Robot rotation around -Y axis -> Fingers tilt up (Pitch up)
- PICO rotation around -X axis -> Robot rotation around +Y axis -> Fingers press down (Pitch down)
- PICO rotation around Y axis -> Robot rotation around +Z axis (Yaw: fingers deflect left)
- PICO rotation around Z axis -> Robot rotation around -X axis (Roll: wrist rolls left)

---

## 7. Complete Example

### Scenario: User hand moves forward 5cm

**Step 1: PICO Delta**
```python
# User hand moves forward (-Z)
init_pos_pico = [0.5, 0.2, 1.0]
current_pos_pico = [0.5, 0.2, 0.95]  # 5cm in -Z direction
Δpos_pico = [0, 0, -0.05]
```

**Step 2: Transform to World**
```python
Δpos_world = pico_to_robot @ [0, 0, -0.05]
           = [0.05, 0, 0]  # 5cm forward
```

**Step 3: Transform to Left Chest**
```python
Δpos_left_chest = R_world_to_left @ [0.05, 0, 0]
                = [0.05, 0, 0]  # X_chest: forward
```

**Step 4: Apply to Robot Initial Pose**
```python
# Robot initial position (left arm, from tianji_robot.yaml)
TIANJI_INIT_POS['left'] = [0.5733, 0.2237, 0.2762]

# Target position
target_pos = [0.5733, 0.2237, 0.2762] + [0.05, 0, 0]
           = [0.6233, 0.2237, 0.2762]  # Moved 5cm forward
```

---

## 8. Tracker Mapping and Configuration

### 8.1 Tracker SN Mapping

| Tracker SN | Role | Purpose |
|------------|------|---------|
| YOUR_LEFT_WRIST_SN | pico_left_wrist | Controls left arm end-effector pose |
| YOUR_RIGHT_WRIST_SN | pico_right_wrist | Controls right arm end-effector pose |
| YOUR_LEFT_ARM_SN | pico_left_arm | Computes left arm elbow direction |
| YOUR_RIGHT_ARM_SN | pico_right_arm | Computes right arm elbow direction |

### 8.2 Tracker Initial Positions (Critical!)

**Wrist Tracker and Arm Tracker use different initial positions**:

```python
# Wrist Tracker: Maps to robot end-effector position (from tianji_robot.yaml)
wrist_init_pos = TIANJI_INIT_POS[side]
# Left arm:  [0.5733, 0.2237, 0.2762]
# Right arm: [0.5733, -0.2237, 0.2762]

# Arm Tracker: Maps to elbow reference position (from tianji_robot.yaml arm_init_pos)
# These values are manually set so the arm tracker is below and outside the elbow in chest frame
arm_init_pos = ARM_INIT_POS[side]
# Left arm:  [0.2, 0.3, 0.2]
# Right arm: [0.2, -0.3, 0.2]
```

**Why are they different?**
- Wrist tracker controls end-effector pose, needs to map to robot wrist position
- Arm tracker serves as the reference position for incremental control (arm tracker delta baseline in pico_input_node)

**Arm Tracker Initial Position Explanation (Chest Coordinate System)**:
```python
Left Chest (+90° around X): +Y = down, +Z = left (outward)
Right Chest (-90° around X): +Y = up, +Z = right (outward)

Left arm: [0.2, 0.3, 0.2]
      # X=0.2:  20cm forward
      # Y=0.3:  30cm downward (in Left Chest +Y = down)
      # Z=0.2:  20cm to the left (outward)

Right arm: [0.2, -0.3, 0.2]
      # X=0.2:  20cm forward
      # Y=-0.3: 30cm downward (in Right Chest -Y = down)
      # Z=0.2:  20cm to the right (outward)
```

**Arm Angle Control Notes**:

- **pico_input_node / step4**: **Dynamically computes** elbow offset direction from arm tracker position
  (geometric projection -> negate -> publish to `/left_arm_elbow_direction`)
- **step5**: Uses static `DEFAULT_ZSP_DIRECTION` (fixed value verified through FK->IK closed-loop)
- Geometric elbow direction computation is also used for step4's RViz MarkerArray visualization

---

## 9. Step4 vs Step5 Comparison

### Step4: Visualization Verification

```text
Purpose: Offline verification of coordinate transforms, no real robot needed

Input: Recorded data file (trackingData_*.txt)
Output:
  - TF: world_left/right → pico_*_wrist, pico_*_arm
  - Topics: /left_arm_target_pose, /right_arm_target_pose
  - Markers: /elbow_angle_visualization

Verification points:
  ✓ pico_left_arm and pico_right_arm are both below their respective chest frames
  ✓ Blue elbow direction arrows both point downward (elbow-down)
  ✓ Left and right arms are symmetric
```

### Step5: Real Robot Control

```text
Purpose: Control a real robot using recorded data

Input: Recorded data file
Output:
  - Robot end-effector pose commands
  - zsp_para arm angle constraints

Safety measures:
  --dry-run     Print only, no control
  --speed 0.3   Slow playback
  --left-only   Control left arm only
  --right-only  Control right arm only
```

### Step5 Verification Test Commands

Use specific frame segments from `trackingData_whole_data.txt` to verify coordinate mapping:

**Coordinate Mapping Quick Reference**:

| User Action | PICO Change | Robot Change | Robot Motion |
|-------------|-------------|--------------|--------------|
| Reach forward | Z decreases (-Z) | X increases (+X) | Forward |
| Pull back | Z increases (+Z) | X decreases (-X) | Backward |
| Move right | X increases (+X) | Y decreases (-Y) | Right |
| Move left | X decreases (-X) | Y increases (+Y) | Left |
| Raise hand | Y increases (+Y) | Z increases (+Z) | Up |
| Lower hand | Y decreases (-Y) | Z decreases (-Z) | Down |

**Forward/Backward Direction Test** (PICO Z -> Robot X):
```bash
# Backward motion: PICO Z +16cm → Robot X -16cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2180 --end-frame 2230 --left-only --speed 0.5 -v

# Forward motion: PICO Z -10cm → Robot X +10cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2275 --end-frame 2325 --left-only --speed 0.5 -v
```

**Left/Right Direction Test** (PICO X -> Robot Y):
```bash
# Rightward motion: PICO X +18cm → Robot Y -18cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2417 --end-frame 2467 --left-only --speed 0.5 -v

# Leftward motion: PICO X -19cm → Robot Y +19cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 1140 --end-frame 1160 --left-only --speed 0.5 -v
```

**Up/Down Direction Test** (PICO Y -> Robot Z):
```bash
# Downward motion: PICO Y -20cm → Robot Z -20cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 2062 --end-frame 2112 --left-only --speed 0.5 -v

# Upward motion: PICO Y +19cm → Robot Z +19cm
python3 step5_incremental_control_with_robot.py \
  --file ../record/trackingData_whole_data.txt \
  --start-frame 1958 --end-frame 2008 --right-only --speed 0.5 -v
```

**Motion Analysis Tool**:
```bash
python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt
```

### Comparison with step2_pose_topic_control.py

| Aspect | step2 | pico_input_node |
|--------|-------|-----------------|
| **frame_id** | `'world_left'` | `'world'` |
| **Pose values** | Directly in chest frame | Delta + transform to chest |
| **Control method** | Absolute pose | Incremental pose |
| **Initialization** | Not needed | Needed (record initial pose) |

**Both are correct!** They just differ in representation.

---

## 10. FAQ

### Q1: Why are the Y component signs different?

Because the Y axes of world_left and world_right point in opposite directions in the world coordinate system.
The physical direction "outward from the body" corresponds to different Y signs in the two chest coordinate systems.

### Q2: How to verify that coordinate transforms are correct?

Run step4, check in RViz:
1. pico_left_arm and pico_right_arm are both below ✓
2. Blue arrows all point downward ✓
3. Left-right symmetry ✓

### Q3: Why is the X component 0?

The X axis is the forward direction. In the elbow-down pose, the elbow does not deviate forward or backward, only downward + outward, so X=0.

### Q4: Why use the axis-angle method for orientation transforms?

pico_to_robot is an axis mapping matrix (determinant = +1).
The axis-angle method directly transforms the rotation axis vector, which is conceptually clear and efficient.

---

## 11. File Structure

```text
tianji_world_output/tianji_world_output/
├── __init__.py                      # Package entry (public interface docs)
├── config_loader.py                 # Unified config loader (tianji_robot.yaml)
├── transform_utils.py               # ★ Coordinate transform shared library (sole authoritative implementation)
├── cartesian_controller.py          # Cartesian space controller
├── tianji_world_output_node.py      # ROS2 output node
├── fx_kine.py                       # FK/IK kinematics (re-export from tianji_output)
├── fx_robot.py                      # Robot low-level control (re-export from tianji_output)
├── structure_data.py                # ctypes structures (re-export from tianji_output)
├── robot_structures.py              # Robot structures (re-export from tianji_output)
└── config/
    ├── tianji_robot.yaml            # ★ Unified config (Single Source of Truth)
    └── ccs_m6.MvKDCfg              # Kinematics config file

pico_input/pico_input/
├── __init__.py                      # Package entry (public interface docs)
├── pico_input_node.py               # ROS2 input node (incremental control)
├── incremental_controller.py        # Incremental controller (pure computation, no ROS2 dependency)
├── data_source/                     # Data source abstraction layer + implementations
│   ├── __init__.py                  # re-export (DataSource, TrackerData, HeadsetData)
│   ├── base.py                      # ABC base class (DataSource, TrackerData, HeadsetData)
│   ├── live_data_source.py          # Live PICO SDK data source
│   └── recorded_data_source.py      # Recorded file playback data source
└── xrobotoolkit_client.py           # PICO SDK wrapper

pico_input/test/
├── common/
│   ├── robot_config.py              # Test compatibility layer (loads from tianji_robot.yaml)
│   ├── robot_lifecycle.py           # Robot power-on/off lifecycle management
│   └── transform_utils.py           # Re-export wrapper (delegates to tianji_world_output)
├── docs/
│   └── PICO_TELEOP_GUIDE.md         # This document
├── tool/
│   ├── move_to_init_pose.py         # Move robot to initial pose
│   ├── verify_fk_values.py          # Verify FK computed values
│   └── diagnose_zsp_para.py         # FK→IK closed-loop zsp_para diagnostics
├── step1_direct_joint_control.py    # Direct joint control (basic test)
├── step2_pose_topic_control.py      # ROS2 Topic pose control
├── step3_visualize_in_rviz.py       # RViz coordinate system visualization
├── step4_visualize_recorded_data.py # PICO recorded data visualization
├── step5_incremental_control_with_robot.py  # PICO incremental control with real robot
└── step6_arm_angle_stability_test.py # Arm angle stability test (simulated data)

pico_input/record/
├── trackingData_sample_static.txt   # High-quality static test data (150 frames)
├── trackingData_whole_data.txt      # Complete test data (5780 frames)
├── trackingData_head_tracker.txt    # Head tracker data
├── trackingData_head_tracker_static.txt  # Head tracker static data
├── analyze_motion_data.py           # Motion data analysis tool
└── visualize_pico_raw.py            # PICO raw data visualization tool
```

### Shared Library Architecture

**Sole Authoritative Implementation**: `tianji_world_output/transform_utils.py`

All coordinate/rotation transform functions are implemented only in this file; all consumers import from here.
`test/common/transform_utils.py` is merely a re-export wrapper with zero implementation code.

### Shared Library (transform_utils.py) Function Index

When writing new programs, import the following functions from `common.transform_utils` (test scripts) or
`tianji_world_output.transform_utils` (ROS2 nodes):

```python
from common.transform_utils import (
    # Position transforms
    transform_world_to_chest,           # World→Chest vector: [x,y,z] → [x,-z,y] / [x,z,-y]
    transform_chest_to_world,           # Chest→World vector (inverse transform)

    # Rotation transforms
    get_world_to_chest_rotation,        # World→Chest rotation matrix (3x3)
    get_chest_to_world_rotation,        # Chest→World rotation matrix (3x3)
    apply_world_rotation_to_chest_pose, # Rotate in World, return Chest pose (4-step method)
    transform_pico_rotation_to_world,   # PICO→World rotation transform (axis-angle method)

    # TF publishing
    get_tf_quaternion,                  # Quaternion for TF publishing (chest→world conjugate)

    # Configuration query
    get_pico_to_robot,                  # PICO→Robot 3x3 transform matrix (loaded from config)

    # Arm angle control
    elbow_direction_from_angles,        # Generate zsp_para direction vector from angles
)
```

**Common Usage Examples**:
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. Position delta: World→Chest
displacement_chest = transform_world_to_chest(np.array([dx, dy, dz]), side)
target_pos = init_pos_chest + displacement_chest

# 2. Orientation rotation: Rotate 30° in World, result in Chest
R_delta = R.from_rotvec([0, np.radians(30), 0])  # Around +Y = Pitch down
target_rot = apply_world_rotation_to_chest_pose(init_rot_chest, R_delta, side)

# 3. PICO→World rotation transform
R_world = transform_pico_rotation_to_world(R_pico_delta, pico_to_robot)

# 4. Arm angle: Default elbow-down
direction = elbow_direction_from_angles(pitch=0, yaw=45, side='left')
zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]
```

### Step Descriptions

| Step | Function | Hardware Required |
|------|----------|-------------------|
| step1 | Direct joint control, verify SDK connection | Robot |
| step2 | ROS2 Topic pose control | Robot + ROS2 |
| step3 | RViz visualization of robot pose | Robot + ROS2 |
| step4 | Visualize PICO recorded data | ROS2 (no hardware needed) |
| step5 | Control real robot with PICO data | Robot |
| step6 | Arm angle stability test (simulated data) | Robot (can dry-run) |

---

## Summary

### Core Transform Chain

```text
PICO OpenXR Delta
    ↓ (pico_to_robot matrix)
Tianji World Delta
    ↓ (world_to_chest rotation)
Tianji Chest Delta
    ↓ (add to TIANJI_INIT_POS)
Target Pose
    ↓ (Topic publish / direct control)
tianji_world_output_node (IK)
    ↓
Joint Angle Control
```

### Key Points

1. **Incremental control**: The robot hand moves as much as the user's hand moves
2. **Multi-stage transform**: PICO -> World -> Chest, each step has clear physical meaning
3. **Coordinate system alignment**: Transform matrices align axis directions across different coordinate systems
4. **Orientation transform**: Axis-angle method transforms the rotation axis (pico_to_robot det=+1, orthogonal matrix)
5. **Arm angle control**: zsp_para specifies the reference arm angle plane (note: Y component is opposite to gravity direction)

---

**Core Formula**: `Target = InitPose + Transform_Chain(PICO_Delta)`

**Document Updated**: 2026-02-09
