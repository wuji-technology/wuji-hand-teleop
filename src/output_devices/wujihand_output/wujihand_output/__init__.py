"""
wujihand_output 包 - Wuji 灵巧手硬件接口

文件结构：
├── __init__.py                # 公共 API 导出
├── wujihand_controller.py     # 统一控制器（支持关节角度和 IK 控制）
└── _internal/                 # 内部实现（不建议直接导入）
    └── hand_interface.py      # 底层灵巧手硬件接口

公共接口：
- WujiHandController: 统一控制器（支持关节角度和 IK 控制）

使用示例:
    from wujihand_output import WujiHandController
    controller = WujiHandController(left_serial='xxx', right_serial='yyy')

    # 关节角度控制
    controller.set_joint_positions(left_positions=[...], right_positions=[...])

    # IK 控制（需要 wuji_retargeting）
    controller.set_keypoints(left_keypoints=[...], right_keypoints=[...])

    controller.disable_and_release()
"""

from .wujihand_controller import WujiHandController

__all__ = ['WujiHandController']
