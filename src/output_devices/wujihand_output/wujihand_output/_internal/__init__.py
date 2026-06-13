"""
wujihand_output internal modules — direct import is discouraged.

This directory holds low-level implementation used by the high-level controller:
- hand_interface: dexterous-hand hardware-interface wrapper

Prefer the top-level interface:
    from wujihand_output import WujiHandController
"""

from .hand_interface import WujiHand

__all__ = [
    'WujiHand',
]
