#!/usr/bin/env python3
"""
机器人配置参数（兼容性包装器）

此文件是向后兼容层，实际配置来源于：
  tianji_world_output/config/tianji_robot.yaml

使用方法（推荐）:
  from tianji_world_output.config_loader import get_config
  config = get_config()

使用方法（兼容旧代码）:
  from common.robot_config import TIANJI_INIT_POS, TIANJI_INIT_ROT, ...
"""

import sys
from pathlib import Path

# 添加 tianji_world_output 到路径（使用相对路径）
_src_dir = Path(__file__).resolve().parents[4]  # common → test → pico_input → input_devices → src
_tianji_path = _src_dir / 'output_devices' / 'tianji_world_output'
if str(_tianji_path) not in sys.path:
    sys.path.insert(0, str(_tianji_path))

# 从统一配置加载器导入
from tianji_world_output.config_loader import get_config

# 加载配置
_config = get_config(use_ros=False)

# =============================================================================
# 导出兼容性变量 (与旧 robot_config.py 接口一致)
# =============================================================================

# 初始关节角 (度)
INIT_JOINTS = {k: v.tolist() for k, v in _config.init_joints.items()}

# 初始位姿 (Chest 坐标系)
TIANJI_INIT_POS = _config.init_pos
TIANJI_INIT_ROT = _config.init_rot

# World → Chest 坐标变换
WORLD_TO_LEFT_QUAT = _config.world_to_chest_quat['left']
WORLD_TO_RIGHT_QUAT = _config.world_to_chest_quat['right']
WORLD_TO_CHEST_TRANS = _config.world_to_chest_trans

# IK 参数
ZSP_TYPE = _config.zsp_type
ZSP_PARA = _config.default_zsp_para
ZSP_ANGLE = _config.zsp_angle
DGR = _config.dgr

# 归一化的默认 zsp 方向向量 (前3个分量，用于 elbow direction 默认值)
DEFAULT_ZSP_DIRECTION = {
    'left': _config.get_default_zsp_direction('left'),
    'right': _config.get_default_zsp_direction('right'),
}

# 前臂 Tracker 初始位置 (Chest 坐标系)
ARM_INIT_POS = _config.arm_init_pos
ARM_INIT_QUAT = _config.arm_init_quat

# 机器人连接参数
ROBOT_IP = _config.robot_ip

# 运动学配置文件路径
KINE_CONFIG_PATH = _config.get_kine_config_path()

# 世界坐标系参考姿态
import numpy as np
WORLD_REFERENCE_ROT = np.eye(3)
