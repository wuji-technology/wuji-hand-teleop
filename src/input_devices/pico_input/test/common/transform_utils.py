#!/usr/bin/env python3
"""
坐标转换工具函数（向后兼容包装器）

实际实现位于: tianji_world_output/tianji_world_output/transform_utils.py
本文件仅做 re-export，确保 test/ 脚本的 import 路径不变。

使用方法（test 脚本中）:
    from common.transform_utils import (
        transform_world_to_chest,
        apply_world_rotation_to_chest_pose,
        transform_pico_rotation_to_world,
        elbow_direction_from_angles,
        get_pico_to_robot,
        get_tf_quaternion,
        # ... 其它函数
    )
"""

import sys
from pathlib import Path

# 添加 tianji_world_output 到路径（使用相对路径）
_src_dir = Path(__file__).resolve().parents[4]  # common → test → pico_input → input_devices → src
_tianji_path = _src_dir / 'output_devices' / 'tianji_world_output'
if str(_tianji_path) not in sys.path:
    sys.path.insert(0, str(_tianji_path))

# re-export 所有公共函数
from tianji_world_output.transform_utils import (  # noqa: F401, E402
    # 位置变换
    transform_world_to_chest,
    transform_chest_to_world,
    # 旋转矩阵
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    # TF 发布
    get_tf_quaternion,
    # 方向查询
    get_direction_vector_world,
    get_rotation_axis_world,
    # 配置查询
    get_pico_to_robot,
    # 姿态旋转变换
    apply_world_rotation_to_chest_pose,
    transform_pico_rotation_to_world,
    # 臂角控制
    elbow_direction_from_angles,
)
