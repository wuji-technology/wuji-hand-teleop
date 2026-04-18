
# tianji_world_output

**ROS REP 103 Standard Tianji Arm Output Package - Designed for PICO Teleoperation and New Incremental Control Algorithm**

## Overview

This is a brand new standalone package, specifically designed for PICO VR teleoperation and incremental control algorithms based on ROS REP 103.

### Differences from tianji_output

| Feature | tianji_output (Original) | tianji_world_output (New) |
|---------|--------------------------|---------------------------|
| **Frame Names** | `left_chest`, `right_chest` | `world_left`, `world_right` |
| **Coordinate Standard** | Custom | ROS REP 103 |
| **Use Case** | Traditional teleoperation | PICO + New incremental control |
| **Control Mode** | Absolute position control | Incremental control optimized |
| **Code Coupling** | Coupled with legacy system | Independent design |

## Core Features

### 1. ROS REP 103 Standard Coordinate System

```
world (robot base, +X forward +Y left +Z up)
  | [fixed rotation]
world_left / world_right (left/right arm chest coordinate frames)
  | [pico_input publishes target_pose Topic]
tianji_world_output (subscribes Topic -> IK -> joint angles -> robot)
```

Note: `left_dh_ee` / `right_dh_ee` are TFs published by test scripts (step3/step4),
used for RViz visualization of robot end-effector poses, not part of the production node data flow.

### 2. Incremental Control Optimization

- Records initial position at startup
- Uses previous target position + increment during runtime
- Avoids repeated FK calls (performance improvement 1-2ms/cycle)
- Suitable for high-frequency control (100Hz+)

### 3. Clear Naming Convention

- `world_*` prefix: indicates everything is under the world coordinate frame hierarchy
- Compliant with ROS REP 103 standard
- Consistent with test scripts naming

## Package Structure

```
tianji_world_output/
├── tianji_world_output/
│   ├── __init__.py
│   ├── tianji_world_output_node.py  <- Main node (subscribes Topic -> IK -> robot)
│   ├── cartesian_controller.py     <- Cartesian controller (IK + robot communication)
│   ├── config_loader.py            <- Unified config loader (tianji_robot.yaml)
│   ├── transform_utils.py          <- Coordinate transform shared library (single authoritative implementation)
│   ├── fx_kine.py                  <- re-export from tianji_output
│   ├── fx_robot.py                 <- re-export from tianji_output
│   ├── structure_data.py           <- re-export from tianji_output
│   ├── robot_structures.py         <- re-export from tianji_output
│   └── config/
│       └── ccs_m6.MvKDCfg           <- Tianji IK configuration file
├── launch/
│   └── tianji_world_output.launch.py
├── config/
│   └── tianji_robot.yaml            <- Robot parameters (Single Source of Truth)
├── tests/
│   ├── conftest.py
│   ├── test_config_loader.py
│   └── test_transform_utils.py
├── package.xml
├── setup.py
└── README.md
```

## Usage

### Installation

```bash
cd ~/Desktop/wuji-hand-teleop
colcon build --packages-select tianji_world_output
source install/setup.bash
```

### Launch Node

```bash
ros2 run tianji_world_output tianji_world_output_node
```

### Integration with PICO System

Use in launch files:

```python
tianji_node = Node(
    package="tianji_world_output",
    executable="tianji_world_output_node",
    name="tianji_world_output_node",
    output="screen",
)
```

## Configuration

### Topic Subscriptions

The node subscribes to the following Topics (published by pico_input):

```
/left_arm_target_pose      (PoseStamped, chest coordinate frame)
/right_arm_target_pose     (PoseStamped, chest coordinate frame)
/left_arm_elbow_direction  (Vector3Stamped, optional)
/right_arm_elbow_direction (Vector3Stamped, optional)
```

### Parameters

```yaml
tianji_world_output:
  ros__parameters:
    control_rate: 90.0        # Control frequency (Hz), default 90
    vel_ratio: 60             # Velocity ratio (%)
    acc_ratio: 60             # Acceleration ratio (%)
    # robot_ip: loaded from tianji_robot.yaml, default "192.168.1.190"
```

## Reference Documentation

- [ROS REP 103](https://www.ros.org/reps/rep-0103.html) - ROS Coordinate Frame Standard
- [PICO_TELEOP_GUIDE.md](../../input_devices/pico_input/test/docs/PICO_TELEOP_GUIDE.md) - Complete Teleoperation Guide
- [tianji_output](../tianji_output) - Original Output Package

## Important Notes

1. **Not compatible with legacy system**: This package uses new naming conventions and is not compatible with legacy systems using `left_chest/right_chest`
2. **Requires matching PICO input**: Must be used with `pico_input` that uses the new naming conventions
3. **Independently maintained**: This package is independent from `tianji_output`; modifications will not affect the original package

## Migration Guide

If migrating from `tianji_output` to `tianji_world_output`:

1. Update the package name in launch files
2. Update TF publishers to use new naming (`world_left/world_right`)
3. Ensure all static TF transforms use the new naming
4. Test and verify the TF tree structure

## Development Log

- 2026-02-04: Initial creation, based on ROS REP 103 standard
- 2026-02-04: Added incremental control optimization support
