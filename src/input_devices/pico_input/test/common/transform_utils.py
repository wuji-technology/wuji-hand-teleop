#!/usr/bin/env python3
"""
Coordinate transform utility functions (backward-compatible wrapper)

Actual implementation is in: tianji_world_output/tianji_world_output/transform_utils.py
This file only re-exports, ensuring import paths in test/ scripts remain unchanged.

Usage (in test scripts):
    from common.transform_utils import (
        transform_world_to_chest,
        apply_world_rotation_to_chest_pose,
        transform_pico_rotation_to_world,
        elbow_direction_from_angles,
        get_pico_to_robot,
        get_tf_quaternion,
        # ... other functions
    )
"""

import sys
from pathlib import Path

# Add tianji_world_output to path (using relative path)
_src_dir = Path(__file__).resolve().parents[4]  # common -> test -> pico_input -> input_devices -> src
_tianji_path = _src_dir / 'output_devices' / 'tianji_world_output'
if str(_tianji_path) not in sys.path:
    sys.path.insert(0, str(_tianji_path))

# Re-export all public functions
from tianji_world_output.transform_utils import (  # noqa: F401, E402
    # Position transforms
    transform_world_to_chest,
    transform_chest_to_world,
    # Rotation matrices
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    # TF publishing
    get_tf_quaternion,
    # Direction queries
    get_direction_vector_world,
    get_rotation_axis_world,
    # Configuration queries
    get_pico_to_robot,
    # Pose rotation transforms
    apply_world_rotation_to_chest_pose,
    transform_pico_rotation_to_world,
    # Arm angle control
    elbow_direction_from_angles,
)
