"""
controller 包 - 统一控制器节点

提供支持模式切换的状态机控制节点：
- TianjiArmControllerNode: 天机臂控制器
- WujiHandControllerNode: 灵巧手控制器

两种控制模式：
1. TELEOP 模式（默认）：遥操作输入 → 运动学解算 → 硬件
2. INFERENCE 模式：joint_command 话题 → 硬件（跳过运动学解算）

模块结构：
├── common.py           # 共享工具（ControlMode, ROS2LoggerAdapter 等）
├── wujihand_node.py    # 灵巧手控制器节点
└── tianji_arm_node.py  # 天机臂控制器节点
"""

from .common import ControlMode, ROS2LoggerAdapter
from .tianji_arm_node import TianjiArmControllerNode
from .wujihand_node import WujiHandControllerNode

__all__ = [
    'ControlMode',
    'ROS2LoggerAdapter',
    'TianjiArmControllerNode',
    'WujiHandControllerNode',
]

