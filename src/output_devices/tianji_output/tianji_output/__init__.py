"""
tianji_output package - Tianji arm hardware interface

File structure:
├── __init__.py                    # Public API exports
├── tianji_arm_controller.py       # Recommended - unified controller (integrates Cartesian and joint space)
├── cartesian_controller.py        # Cartesian space controller (backward compatible)
├── joint_controller.py            # Joint space controller (backward compatible)
├── _internal/                     # Internal implementation (not recommended for direct import)
│   ├── fx_robot.py                # Low-level robot communication interface
│   ├── fx_kine.py                 # Kinematics solver interface
│   ├── structure_data.py          # C interface data structures
│   └── robot_structures.py        # Kinematics-related structures
└── tools/                         # Standalone tool scripts
    ├── analyze_recording.py       # Analyze robot recorded data
    ├── debug_arm_axis.py          # Debug coordinate axes ROS2 node
    └── ankle_angle_plot.py        # Interactive visualization

Public interface:
- TianjiArmController: Unified controller (recommended, supports both Cartesian and joint space control)
- CartesianController: Cartesian space controller (backward compatible)
- JointController: Joint space controller (backward compatible)
- Marvin_Robot: Low-level robot communication interface (advanced users)
- Marvin_Kine: Kinematics solver interface (advanced users)

Usage example:
    # Unified controller (recommended)
    from tianji_output import TianjiArmController
    controller = TianjiArmController(robot_ip='192.168.1.190')
    controller.set_impedance_mode(mode='joint')

    # Cartesian space control (Teleop mode)
    controller.move_to_pose_direct(left_pose=[...], right_pose=[...], unit='m')

    # Joint space control (Inference mode)
    controller.move_to_joints_direct(left_joints=[...], right_joints=[...])

    # Release
    controller.disable_and_release()
"""

# Recommended unified controller
from .tianji_arm_controller import TianjiArmController

# # Backward-compatible interfaces (not recommended for new code)
# from .cartesian_controller import CartesianController
# from .joint_controller import JointController

# Low-level interfaces (advanced users)
from ._internal.fx_robot import Marvin_Robot
from ._internal.fx_kine import Marvin_Kine

__all__ = [
    # Recommended unified controller
    'TianjiArmController',
    # Backward-compatible interfaces
    # 'CartesianController',
    # 'JointController',
    # Low-level interfaces (advanced users)
    'Marvin_Robot',
    'Marvin_Kine',
]
