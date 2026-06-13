"""
tianji_output internal module - not recommended for direct import

This directory contains low-level implementations for use by high-level controllers:
- fx_robot: Robot low-level communication interface
- fx_kine: Kinematics solver interface
- structure_data: C interface data structures
- robot_structures: Kinematics-related structures

Recommended to use top-level interfaces:
    from tianji_output import TianjiArmController, TianjiChestDriver
"""

from .fx_robot import Marvin_Robot
from .fx_kine import Marvin_Kine
from .structure_data import DCSS, StateCtr, RT_IN, RT_OUT
from .robot_structures import Vect7, Matrix4, FX_InvKineSolvePara, FX_Jacobi

__all__ = [
    'Marvin_Robot',
    'Marvin_Kine',
    'DCSS',
    'StateCtr',
    'RT_IN',
    'RT_OUT',
    'Vect7',
    'Matrix4',
    'FX_InvKineSolvePara',
    'FX_Jacobi',
]
