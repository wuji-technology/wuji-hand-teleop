"""
wujihand_output package — Wuji dexterous-hand hardware interface.

File layout:
├── __init__.py                # public API exports
├── wujihand_controller.py     # unified controller (joint-angle + IK control)
└── _internal/                 # internal implementation (do not import directly)
    └── hand_interface.py      # low-level hardware interface

Public interface:
- WujiHandController: unified controller (supports joint-angle and IK control)

Example:
    from wujihand_output import WujiHandController
    controller = WujiHandController(left_serial='xxx', right_serial='yyy')

    # Joint-angle control
    controller.set_joint_positions(left_positions=[...], right_positions=[...])

    # IK control (requires wuji_retargeting)
    controller.set_keypoints(left_keypoints=[...], right_keypoints=[...])

    controller.disable_and_release()
"""

from .wujihand_controller import WujiHandController

__all__ = ['WujiHandController']
