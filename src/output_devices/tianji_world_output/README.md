# tianji_world_output

**ROS REP 103 标准的天机臂输出包 - 专为 PICO 遥操作和新增量控制算法设计**

## 📋 概述

这是一个全新的独立包，专门为 PICO VR 遥操作和基于 ROS REP 103 的增量控制算法设计。

### 与 tianji_output 的区别

| 特性 | tianji_output (原包) | tianji_world_output (新包) |
|------|---------------------|---------------------------|
| **坐标系命名** | `left_chest`, `right_chest` | `world_left`, `world_right` ✅ |
| **坐标系标准** | 自定义 | ROS REP 103 ✅ |
| **适用场景** | 传统遥操作 | PICO + 新增量控制 ✅ |
| **控制模式** | 绝对位置控制 | 增量控制优化 ✅ |
| **代码耦合** | 与旧系统耦合 | 独立设计 ✅ |

## 🎯 核心特性

### 1. ROS REP 103 标准坐标系

```
world (机器人基座, +X前 +Y左 +Z上)
  ↓ [固定旋转]
world_left / world_right (左右机械臂 chest 坐标系)
  ↓ [pico_input 发布 target_pose Topic]
tianji_world_output (订阅 Topic → IK → 关节角 → 机器人)
```

注: `left_dh_ee` / `right_dh_ee` 是测试脚本 (step3/step4) 发布的 TF，
用于 RViz 可视化机器人末端位姿，不属于生产节点的数据流。

### 2. 增量控制优化

- ✅ 初始化时记录起始位置
- ✅ 运行时直接使用上次目标位置 + 增量
- ✅ 避免重复调用 FK（性能提升 1-2ms/cycle）
- ✅ 适合高频控制（100Hz+）

### 3. 清晰的命名约定

- `world_*` 前缀：表明都在世界坐标系层级下
- 符合 ROS REP 103 标准
- 与 test scripts 命名一致

## 📦 包结构

```
tianji_world_output/
├── tianji_world_output/
│   ├── __init__.py
│   ├── tianji_world_output_node.py  ← 主节点（订阅 Topic → IK → 机器人）
│   ├── cartesian_controller.py     ← 笛卡尔控制器（IK + 机器人通信）
│   ├── config_loader.py            ← 统一配置加载 (tianji_robot.yaml)
│   ├── transform_utils.py          ← 坐标变换共享库 (唯一权威实现)
│   ├── fx_kine.py                  ← re-export from tianji_output
│   ├── fx_robot.py                 ← re-export from tianji_output
│   ├── structure_data.py           ← re-export from tianji_output
│   ├── robot_structures.py         ← re-export from tianji_output
│   └── config/
│       └── ccs_m6.MvKDCfg           ← 天机 IK 配置文件
├── launch/
│   └── tianji_world_output.launch.py
├── config/
│   └── tianji_robot.yaml            ← 机器人参数 (Single Source of Truth)
├── tests/
│   ├── conftest.py
│   ├── test_config_loader.py
│   └── test_transform_utils.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 使用方法

### 安装

```bash
cd ~/Desktop/wuji-teleop-ros2-private
colcon build --packages-select tianji_world_output
source install/setup.bash
```

### 启动节点

```bash
ros2 run tianji_world_output tianji_world_output_node
```

### 与 PICO 系统集成

在 launch 文件中使用：

```python
tianji_node = Node(
    package="tianji_world_output",
    executable="tianji_world_output_node",
    name="tianji_world_output_node",
    output="screen",
)
```

## 🔧 配置

### Topic 订阅

节点订阅以下 Topics (由 pico_input 发布):

```
/left_arm_target_pose      (PoseStamped, chest 坐标系)
/right_arm_target_pose     (PoseStamped, chest 坐标系)
/left_arm_elbow_direction  (Vector3Stamped, 可选)
/right_arm_elbow_direction (Vector3Stamped, 可选)
```

### 参数

```yaml
tianji_world_output:
  ros__parameters:
    control_rate: 90.0        # 控制频率 (Hz), 默认 90
    vel_ratio: 60             # 速度比例 (%)
    acc_ratio: 60             # 加速度比例 (%)
    # robot_ip: 从 tianji_robot.yaml 加载, 默认 "192.168.1.190"
```

## 📚 参考文档

- [ROS REP 103](https://www.ros.org/reps/rep-0103.html) - ROS 坐标系标准
- [PICO_TELEOP_GUIDE.md](../../input_devices/pico_input/test/docs/PICO_TELEOP_GUIDE.md) - 遥操作完整指南
- [tianji_output](../tianji_output) - 原始输出包

## ⚠️ 注意事项

1. **不兼容旧系统**：此包使用新命名规范，不兼容使用 `left_chest/right_chest` 的旧系统
2. **需要配套 PICO 输入**：必须与使用新命名的 `pico_input` 配合使用
3. **独立维护**：此包独立于 `tianji_output`，修改不会影响原包

## 🔄 迁移指南

如果要从 `tianji_output` 迁移到 `tianji_world_output`：

1. 更新 launch 文件中的包名
2. 更新 TF 发布者使用新命名（`world_left/world_right`）
3. 确保所有 TF 静态变换使用新命名
4. 测试验证 TF 树结构

## 📝 开发日志

- 2026-02-04: 初始创建，基于 ROS REP 103 标准
- 2026-02-04: 添加增量控制优化支持
