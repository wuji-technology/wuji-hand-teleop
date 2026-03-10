"""
tianji_output 包 - 天机臂硬件接口

文件结构：
├── __init__.py                    # 公共 API 导出
├── tianji_arm_controller.py       # 推荐 - 统一控制器（整合笛卡尔和关节空间）
├── cartesian_controller.py        # 笛卡尔空间控制器（向后兼容）
├── joint_controller.py            # 关节空间控制器（向后兼容）
├── _internal/                     # 内部实现（不建议直接导入）
│   ├── fx_robot.py                # 底层机器人通信接口
│   ├── fx_kine.py                 # 运动学解算接口
│   ├── structure_data.py          # C 接口数据结构
│   └── robot_structures.py        # 运动学相关结构体
└── tools/                         # 独立工具脚本
    ├── analyze_recording.py       # 分析机器人记录数据
    ├── debug_arm_axis.py          # 调试坐标轴 ROS2 节点
    └── ankle_angle_plot.py        # 交互式可视化

公共接口：
- TianjiArmController: 统一控制器（推荐，同时支持笛卡尔和关节空间控制）
- CartesianController: 笛卡尔空间控制器（向后兼容）
- JointController: 关节空间控制器（向后兼容）
- Marvin_Robot: 底层机器人通信接口（高级用户）
- Marvin_Kine: 运动学解算接口（高级用户）

使用示例:
    # 统一控制器（推荐）
    from tianji_output import TianjiArmController
    controller = TianjiArmController(robot_ip='192.168.1.190')
    controller.set_impedance_mode(mode='joint')

    # 笛卡尔空间控制（Teleop 模式）
    controller.move_to_pose_direct(left_pose=[...], right_pose=[...], unit='m')

    # 关节空间控制（Inference 模式）
    controller.move_to_joints_direct(left_joints=[...], right_joints=[...])

    # 释放
    controller.disable_and_release()
"""

# 推荐使用的统一控制器
from .tianji_arm_controller import TianjiArmController

# # 向后兼容的接口（不推荐新代码使用）
# from .cartesian_controller import CartesianController
# from .joint_controller import JointController

# 底层接口（高级用户）
from ._internal.fx_robot import Marvin_Robot
from ._internal.fx_kine import Marvin_Kine

__all__ = [
    # 推荐使用的统一控制器
    'TianjiArmController',
    # 向后兼容接口
    # 'CartesianController',
    # 'JointController',
    # 底层接口（高级用户）
    'Marvin_Robot',
    'Marvin_Kine',
]
