#!/usr/bin/env python3
"""
坐标转换工具函数（权威共享库）

所有坐标/旋转变换函数的唯一实现。
pico_input_node、step1-6 测试脚本统一从此处导入。

配置来源: tianji_world_output/config/tianji_robot.yaml (Single Source of Truth)

=============================================================================
旋转正方向约定（ROS REP 103 右手定则）
=============================================================================

  判断方法: 右手大拇指指向轴的正方向，四指弯曲方向 = 正旋转方向
  等价描述: 从轴的正端向原点看，逆时针方向 = 正方向

  Robot World 坐标系 (X=前, Y=左, Z=上):

    绕 +X 轴正转 (Roll):  Y→Z, 即 左侧→上方 → 手腕顶部向右倾 (Roll right)
    绕 +Y 轴正转 (Pitch): Z→X, 即 上方→前方 → 手指尖端向下压 (Pitch down)
    绕 +Z 轴正转 (Yaw):   X→Y, 即 前方→左方 → 手指尖端向左偏 (Yaw left)

  记忆口诀: "正Roll右倾, 正Pitch低头, 正Yaw向左"

  数学验证 (标准旋转矩阵):
    Rx(θ)@[0,0,1] = [0, -sinθ, cosθ]  → 上方向右倾   → Roll right  ✓
    Ry(θ)@[1,0,0] = [cosθ, 0, -sinθ]  → 前方向下压   → Pitch down  ✓
    Rz(θ)@[1,0,0] = [cosθ, sinθ, 0]   → 前方向左偏   → Yaw left    ✓

=============================================================================
坐标系定义
=============================================================================

  World (ROS REP 103): X=前, Y=左, Z=上
  Left Chest:  X=前, Y=下, Z=左  (World 绕 X 轴 +90°)
  Right Chest: X=前, Y=上, Z=右  (World 绕 X 轴 -90°)

  向量变换:
    Left:  world [x, y, z] → chest [x, -z, y]
    Right: world [x, y, z] → chest [x, z, -y]

=============================================================================
函数索引
=============================================================================

  位置变换:
    transform_world_to_chest()   - World→Chest 向量变换
    transform_chest_to_world()   - Chest→World 向量变换

  旋转变换:
    get_world_to_chest_rotation()  - World→Chest 旋转矩阵
    get_chest_to_world_rotation()  - Chest→World 旋转矩阵
    apply_world_rotation_to_chest_pose()  - 在 World 中旋转，返回 Chest 姿态
    transform_pico_rotation_to_world()    - PICO→World 旋转变换（轴角法）

  TF 发布:
    get_tf_quaternion()  - 获取 TF 发布用四元数

  方向查询:
    get_direction_vector_world()  - 获取运动方向向量
    get_rotation_axis_world()     - 获取旋转轴向量

  配置查询:
    get_pico_to_robot()  - 获取 PICO→Robot 3x3 变换矩阵

  臂角控制:
    elbow_direction_from_angles()  - 从角度生成 zsp_para 方向向量

=============================================================================
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from .config_loader import get_config as _get_config


# =============================================================================
# 内部辅助: 延迟加载配置
# =============================================================================

def _config():
    """延迟加载配置单例"""
    return _get_config(use_ros=False)


# =============================================================================
# 位置变换
# =============================================================================

def transform_world_to_chest(vector_world, side):
    """将 world 坐标系的向量转换到 chest (world_left/world_right) 坐标系

    Args:
        vector_world: world 坐标系中的向量 (位置/方向)
        side: 'left' 或 'right'

    Returns:
        np.array: chest 坐标系中的向量

    Note:
        使用高效的轴映射实现（等价于旋转矩阵变换）

        坐标系定义 (from tianji_robot.yaml):
          World: X=前, Y=左, Z=上
          Left Chest:  X=前, Y=下, Z=左 (绕 World X 轴旋转 +90°)
          Right Chest: X=前, Y=上, Z=右 (绕 World X 轴旋转 -90°)

        向量变换映射（通过 R_world_to_chest = R_chest_to_world^T 推导）：
          左臂: world [x, y, z] → chest [x, -z, y]
          右臂: world [x, y, z] → chest [x, z, -y]
    """
    x, y, z = vector_world[0], vector_world[1], vector_world[2]

    if side == 'left':
        return np.array([x, -z, y])
    else:
        return np.array([x, z, -y])


def transform_chest_to_world(vector_chest, side):
    """将 chest 坐标系的向量转换到 world 坐标系（逆变换）

    Args:
        vector_chest: chest 坐标系中的向量
        side: 'left' 或 'right'

    Returns:
        np.array: world 坐标系中的向量
    """
    x, y, z = vector_chest[0], vector_chest[1], vector_chest[2]

    if side == 'left':
        # 逆映射: [x, z, -y]
        return np.array([x, z, -y])
    else:
        # 逆映射: [x, -z, y]
        return np.array([x, -z, y])


# =============================================================================
# 方向查询
# =============================================================================

def get_direction_vector_world(mode):
    """在 world 坐标系获取运动方向向量

    Args:
        mode: 运动模式字符串

    Returns:
        np.array: world 坐标系中的单位方向向量
    """
    directions = {
        'forward': np.array([1.0, 0.0, 0.0]),   # world +X
        'back': np.array([-1.0, 0.0, 0.0]),     # world -X
        'left': np.array([0.0, 1.0, 0.0]),      # world +Y
        'right': np.array([0.0, -1.0, 0.0]),    # world -Y
        'up': np.array([0.0, 0.0, 1.0]),        # world +Z
        'down': np.array([0.0, 0.0, -1.0]),     # world -Z
    }
    return directions.get(mode, np.array([0.0, 0.0, 0.0]))


def get_rotation_axis_world(mode):
    """在 world 坐标系获取旋转轴向量

    Args:
        mode: 旋转模式字符串

    Returns:
        np.array: world 坐标系中的单位旋转轴向量
    """
    axes = {
        'rotate_x': np.array([1.0, 0.0, 0.0]),  # X轴（前后）
        'rotate_y': np.array([0.0, 1.0, 0.0]),  # Y轴（左右）
        'rotate_z': np.array([0.0, 0.0, 1.0]),  # Z轴（上下）
    }
    return axes.get(mode, np.array([0.0, 0.0, 0.0]))


# =============================================================================
# 旋转矩阵
# =============================================================================

def get_world_to_chest_rotation(side):
    """获取 world → chest 的旋转矩阵

    用于将 world 坐标系中的向量/姿态转换到 chest 坐标系：
      v_chest = R_world_to_chest @ v_world
      R_chest = R_world_to_chest @ R_world @ R_world_to_chest.T

    Args:
        side: 'left' 或 'right'

    Returns:
        np.ndarray: 3x3 旋转矩阵

    Note:
        配置文件中的四元数直接表示 world→chest 变换。
        Left: +90° around X, Right: -90° around X
    """
    return _config().get_world_to_chest_rotation(side)


def get_chest_to_world_rotation(side):
    """获取 chest → world 的旋转矩阵

    用于将 chest 坐标系中的向量/姿态转换到 world 坐标系：
      v_world = R_chest_to_world @ v_chest
      R_world = R_chest_to_world @ R_chest @ R_chest_to_world.T

    Args:
        side: 'left' 或 'right'

    Returns:
        np.ndarray: 3x3 旋转矩阵（world_to_chest 的转置/逆）

    Note:
        这是 get_world_to_chest_rotation 的逆变换。
        Left: -90° around X, Right: +90° around X
    """
    return get_world_to_chest_rotation(side).T


def get_tf_quaternion(side):
    """获取用于 TF 发布的四元数 (chest→world 方向)

    TF 变换描述子坐标系在父坐标系中的姿态。
    对于 world → world_left/world_right，需要 chest→world 方向的旋转，
    即 world_to_chest 的逆。

    Args:
        side: 'left' 或 'right'

    Returns:
        np.ndarray: 四元数 [qx, qy, qz, qw]
    """
    quat = _config().world_to_chest_quat[side]

    # 返回共轭四元数（即逆旋转）
    # 对于单位四元数，逆 = 共轭 = [-qx, -qy, -qz, qw]
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]])


# =============================================================================
# 配置查询
# =============================================================================

def get_pico_to_robot():
    """获取 PICO→Robot 3x3 变换矩阵（从配置加载）

    返回:
        np.ndarray: 3x3 矩阵 (det=+1, 两个轴取反相互抵消)

    默认值:
        [[0, 0, -1],    # Robot X = -PICO Z (前方)
         [-1, 0, 0],    # Robot Y = -PICO X (左方)
         [0, 1, 0]]     # Robot Z = +PICO Y (上方)

    物理意义:
        用户面朝机器人前方时:
          往前伸手 → PICO -Z → Robot +X (前方)
          往右伸手 → PICO +X → Robot -Y (右方)
          往上抬手 → PICO +Y → Robot +Z (上方)
    """
    return _config().pico_to_robot


# =============================================================================
# 姿态旋转变换
# =============================================================================

def apply_world_rotation_to_chest_pose(base_rot_chest, R_delta_world, side):
    """在 World 坐标系中应用旋转增量，返回 Chest 坐标系的目标姿态

    这是遥操作中最核心的姿态变换算法（4步法）。
    保证在 World 坐标系中做旋转（物理意义直观），再转回 Chest 坐标系（IK 需要）。

    算法步骤:
      1. Chest→World: 将基准姿态转到 World 坐标系
      2. 左乘增量:    在 World 坐标系中左乘旋转增量（外旋 = 绕世界轴旋转）
      3. World→Chest: 将目标姿态转回 Chest 坐标系

    数学公式:
      target_chest = R_w2c @ R_delta @ R_c2w @ base_chest

    旋转正方向 (右手定则):
      绕 +X: Roll right (顶部向右倾)
      绕 +Y: Pitch down (手指向下压)
      绕 +Z: Yaw left (手指向左偏)

    Args:
        base_rot_chest: 基准姿态（Chest 坐标系，3x3 旋转矩阵）
        R_delta_world: 增量旋转（World 坐标系，scipy Rotation 对象）
        side: 'left' 或 'right'

    Returns:
        np.ndarray: 目标姿态（Chest 坐标系，3x3 旋转矩阵）

    Example:
        >>> from scipy.spatial.transform import Rotation as R
        >>> # 绕 World +Y 轴旋转 30°（手指向下压 30°）
        >>> R_delta = R.from_rotvec([0, np.radians(30), 0])
        >>> target = apply_world_rotation_to_chest_pose(init_rot, R_delta, 'left')
    """
    R_chest_to_world = get_chest_to_world_rotation(side)
    R_world_to_chest = get_world_to_chest_rotation(side)

    base_rot_in_world = R_chest_to_world @ base_rot_chest
    target_rot_in_world = R_delta_world.as_matrix() @ base_rot_in_world
    target_rot_in_chest = R_world_to_chest @ target_rot_in_world

    return target_rot_in_chest


def transform_pico_rotation_to_world(delta_rot_pico, pico_to_robot):
    """将 PICO 坐标系的旋转增量转换到 Robot World 坐标系（轴角法）

    pico_to_robot 矩阵是坐标轴映射正交矩阵（行列式 = +1，两个轴取反相互抵消）。
    使用轴角法变换旋转：只变换旋转轴方向，保持旋转角度不变。

    PICO→Robot 旋转效果:
      PICO 绕 +X → Robot 绕 -Y → Pitch up (手指向上翘)
      PICO 绕 +Y → Robot 绕 +Z → Yaw left (手指向左偏)
      PICO 绕 +Z → Robot 绕 -X → Roll left (手腕向左翻滚)

    Args:
        delta_rot_pico: PICO 坐标系中的旋转增量（scipy Rotation 对象）
        pico_to_robot: 3x3 变换矩阵（det=+1，从 tianji_robot.yaml 加载）

    Returns:
        scipy.spatial.transform.Rotation: World 坐标系中的旋转增量
    """
    rotvec = delta_rot_pico.as_rotvec()
    angle = np.linalg.norm(rotvec)

    if angle < 1e-10:
        return R.identity()

    axis_pico = rotvec / angle
    axis_world = pico_to_robot @ axis_pico

    return R.from_rotvec(axis_world * angle)


# =============================================================================
# 臂角控制
# =============================================================================

def elbow_direction_from_angles(pitch_deg, yaw_deg, side):
    """从俯仰角和外展角生成 IK 的 zsp_para 方向向量（Chest 坐标系）

    用于生成不同臂角姿态的 zsp_para。结果遵循 IK 求解器的约定：
    Y 分量方向与 Chest 坐标系中的重力方向**相反**。

    重要: zsp_para 不是肘部几何方向！
      - 几何 elbow direction 的 Y 指向重力（肘部物理位置方向）
      - zsp_para 的 Y 指向反重力（IK 参考平面法向量约定）
      - 详见 diagnose_zsp_para.py 的 FK→IK 闭环验证

    角度定义:
      pitch: 0°=不前后偏, 正=肘部向前, 负=肘部向后
      yaw:   0°=纯反重力, 45°=默认沉肘, 90°=纯向外侧

    默认沉肘 (pitch=0, yaw=45):
      左臂: [0, -0.707, -0.707]  (Y-=反重力方向, Z-=外侧)
      右臂: [0, +0.707, -0.707]  (Y+=反重力方向, Z-=外侧)

    Args:
        pitch_deg: 俯仰角（度）
        yaw_deg: 外展角（度）
        side: 'left' 或 'right'

    Returns:
        np.ndarray: 归一化的 3D 方向向量 [x, y, z]（Chest 坐标系）

    Example:
        >>> # 默认沉肘
        >>> d = elbow_direction_from_angles(0, 45, 'left')
        >>> # d ≈ [0, -0.707, -0.707]
        >>> zsp_para = [d[0], d[1], d[2], 0, 0, 0]
    """
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    # X 分量: 前后偏移
    x = np.sin(pitch)

    # 反重力分量（根据 pitch 调整）
    anti_gravity = np.cos(pitch) * np.cos(yaw)
    # 向外侧分量（根据 pitch 调整）
    outward = np.cos(pitch) * np.sin(yaw)

    if side == 'left':
        # Left Chest: Y-=反重力方向, -Z=外侧(向右)
        y = -anti_gravity
        z = -outward
    else:
        # Right Chest: Y+=反重力方向, -Z=外侧(向左)
        y = anti_gravity
        z = -outward

    direction = np.array([x, y, z])

    norm = np.linalg.norm(direction)
    if norm > 1e-6:
        direction = direction / norm

    return direction
