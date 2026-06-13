"""
tianji_output package - Tianji arm hardware interface

File structure:
├── __init__.py                    # Public API exports
├── tianji_arm_controller.py       # Unified controller (integrates Cartesian and joint space)
├── tianji_chest_driver.py         # Chest-frame driver used by HTC/Tracker teleop
├── fault_codes.py                 # 115-entry CN/EN servo fault-code dictionaries
├── _internal/                     # Internal implementation (not for direct import)
│   ├── fx_robot.py                # Low-level robot communication
│   ├── fx_kine.py                 # Kinematics solver interface
│   ├── structure_data.py          # C interface data structures
│   └── robot_structures.py        # Kinematics-related structures
└── tools/                         # Standalone debugging tools
    └── debug_arm_axis.py          # ROS2 node for debugging coordinate axes

Public interface:
- TianjiArmController: Unified controller (supports both Cartesian and joint space)
- TianjiChestDriver: Chest-frame driver with state-machine extras used by tianji_arm_node
- Marvin_Robot: Low-level robot communication interface (advanced users)
- Marvin_Kine: Kinematics solver interface (advanced users)

Usage example:
    from tianji_output import TianjiChestDriver
    controller = TianjiChestDriver(robot_ip='192.168.1.190')
    controller.set_active(mode='joint')
    controller.move_to_pose_direct(left_pose=[...], right_pose=[...], unit='m')
    controller.disable_and_release()
"""

# Unified controller
from .tianji_arm_controller import TianjiArmController

# Chest-frame driver — used by tianji_arm_node for HTC/Tracker teleop.
# Has the state-machine extras (set_standby/set_active/brake control,
# clear_arm_error) that the controller node needs.
from .tianji_chest_driver import TianjiChestDriver

# Low-level interfaces (advanced users)
from ._internal.fx_robot import Marvin_Robot
from ._internal.fx_kine import Marvin_Kine

__all__ = [
    'TianjiArmController',
    'TianjiChestDriver',
    'Marvin_Robot',
    'Marvin_Kine',
]
