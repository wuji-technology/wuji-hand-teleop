#!/usr/bin/env python3
"""
Robot configuration parameters (compatibility wrapper)

This file is a backward-compatibility layer; the actual configuration comes from:
  tianji_world_output/config/tianji_robot.yaml

Usage (recommended):
  from tianji_world_output.config_loader import get_config
  config = get_config()

Usage (compatible with legacy code):
  from common.robot_config import TIANJI_INIT_POS, TIANJI_INIT_ROT, ...
"""

import sys
from pathlib import Path

# Add tianji_world_output to path (using relative path)
_src_dir = Path(__file__).resolve().parents[4]  # common -> test -> pico_input -> input_devices -> src
_tianji_path = _src_dir / 'output_devices' / 'tianji_world_output'
if str(_tianji_path) not in sys.path:
    sys.path.insert(0, str(_tianji_path))

# Import from unified config loader
from tianji_world_output.config_loader import get_config

# Load configuration
_config = get_config()

# =============================================================================
# Export compatibility variables (matching the old robot_config.py interface)
# =============================================================================

# Initial joint angles (degrees)
INIT_JOINTS = {k: v.tolist() for k, v in _config.init_joints.items()}

# Initial pose (Chest coordinate system)
TIANJI_INIT_POS = _config.init_pos
TIANJI_INIT_ROT = _config.init_rot

# World -> Chest coordinate transform
WORLD_TO_LEFT_QUAT = _config.world_to_chest_quat['left']
WORLD_TO_RIGHT_QUAT = _config.world_to_chest_quat['right']
WORLD_TO_CHEST_TRANS = _config.world_to_chest_trans

# IK parameters
ZSP_TYPE = _config.zsp_type
ZSP_PARA = _config.default_zsp_para
ZSP_ANGLE = _config.zsp_angle
DGR = _config.dgr

# Normalized default zsp direction vector (first 3 components, used as elbow direction default)
DEFAULT_ZSP_DIRECTION = {
    'left': _config.get_default_zsp_direction('left'),
    'right': _config.get_default_zsp_direction('right'),
}

# Forearm Tracker initial position (Chest coordinate system)
ARM_INIT_POS = _config.arm_init_pos
ARM_INIT_QUAT = _config.arm_init_quat

# Robot connection parameters
ROBOT_IP = _config.robot_ip

# Kinematics configuration file path
KINE_CONFIG_PATH = _config.get_kine_config_path()

# World coordinate system reference orientation
import numpy as np
WORLD_REFERENCE_ROT = np.eye(3)
