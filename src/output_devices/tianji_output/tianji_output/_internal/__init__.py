"""
tianji_output 内部模块 - 不建议直接导入

此目录包含底层实现，供高级控制器使用：
- fx_robot: 机器人底层通信接口
- fx_kine: 运动学解算接口
- structure_data: C 接口数据结构
- robot_structures: 运动学相关结构体

推荐使用顶层接口:
    from tianji_output import CartesianController, JointController
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
