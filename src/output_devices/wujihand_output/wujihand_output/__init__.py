"""
wujihand_output package - Wuji dexterous hand hardware interface

File structure:
    __init__.py                # Public API exports
    wujihand_controller.py     # Unified controller (supports joint angle and IK control)
    _internal/                 # Internal implementation (not recommended for direct import)
        hand_interface.py      # Low-level dexterous hand hardware interface

Public interfaces:
- WujiHandController: Unified controller (supports joint angle and IK control)

Usage example:
    from wujihand_output import WujiHandController
    controller = WujiHandController(left_serial='xxx', right_serial='yyy')

    # Joint angle control
    controller.set_joint_positions(left_positions=[...], right_positions=[...])

    # IK control (requires wuji_retargeting)
    controller.set_keypoints(left_keypoints=[...], right_keypoints=[...])

    controller.disable_and_release()
"""

from .wujihand_controller import WujiHandController

__all__ = ['WujiHandController']
