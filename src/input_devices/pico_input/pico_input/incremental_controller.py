#!/usr/bin/env python3
"""
增量控制器 - 核心计算逻辑（无 ROS2 依赖）

从 pico_input_node.py 提取的纯计算类，可独立单元测试。

职责:
  1. 记录 tracker 初始位姿
  2. 计算相对于初始位姿的增量变化 (位置 + 姿态)
  3. 将增量映射到机器人 chest 坐标系
  4. One-Euro Filter 自适应平滑 (位置 + 姿态 + 肘部方向)
  5. 肘部方向计算 (物理约束 + 自适应平滑)

数据流:
  PICO 原始位姿 → IncrementalController → chest 坐标系目标位姿

依赖:
  - numpy, scipy (纯数学)
  - tianji_world_output.config_loader (配置)
  - tianji_world_output.transform_utils (坐标变换共享库)
  - pico_input.one_euro_filter (自适应滤波)
  - 无 ROS2 依赖
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.transform_utils import (
    transform_pico_rotation_to_world,
    apply_world_rotation_to_chest_pose,
    transform_world_to_chest,
)
from pico_input.one_euro_filter import OneEuroFilter, OneEuroFilterQuat


# Beta 内部缩放系数 (将用户的统一 beta 映射到各信号域)
# 位置速度 ~0-2 m/s, 需要大 beta 才能在快速运动时打开截止频率
_POS_BETA_SCALE = 20.0
# 角速度 ~0-10 rad/s, 中等 beta
_ROT_BETA_SCALE = 4.0
# 方向变化速度 ~0-5 unit/s, 中等 beta
_ELBOW_BETA_SCALE = 4.0

# 肘部方向几何奇异阈值 (米)
# 当 arm tracker 到肩-腕轴的垂直距离小于此值时, 方向计算不可靠 (噪声放大)
# 15mm ≈ 前臂长 0.25m × sin(3.4°), 即手臂弯曲角 < 3.4° 时视为几何奇异
# PICO tracker 位置噪声 ~±0.5mm (原始), ~±0.1mm (One-Euro 滤波后)
# 15mm 时噪声引起方向误差 < ±1.9° (One-Euro 后 < ±0.04°)
_ELBOW_SINGULARITY_THRESHOLD = 0.015


class IncrementalController:
    """
    增量位姿控制器（纯计算，无 ROS2 依赖）

    设计原则:
      - 所有状态和计算集中管理
      - 无 I/O、无 ROS2 依赖
      - 构造后通过 initialize() 设定初始位姿
      - 之后每帧调用 compute_target_pose() 获取目标位姿

    滤波策略 (One-Euro Filter):
      - 静止时: 低截止频率 → 强平滑 (抑制传感器抖动)
      - 快速运动时: 高截止频率 → 弱平滑 (低延迟, 跟手)
      - 位置、姿态、肘部方向统一使用 One-Euro Filter
    """

    def __init__(self, config: TianjiConfig,
                 rate: float = 90.0,
                 min_cutoff: float = 1.0,
                 beta: float = 0.7,
                 elbow_min_cutoff: float = 0.3):
        """
        Args:
            config: TianjiConfig 统一配置
            rate: 采样率 (Hz)
            min_cutoff: One-Euro 最小截止频率 (Hz), 控制静止时平滑强度
            beta: One-Euro 速度响应系数 (内部按信号类型缩放)
            elbow_min_cutoff: 肘部方向最小截止频率 (Hz), 通常 < min_cutoff
        """
        # PICO→Robot 变换矩阵
        self.pico_to_robot = config.pico_to_robot

        # 机器人初始位姿 (chest 坐标系)
        self.robot_init_positions = {
            'pico_left_wrist': np.array(config.init_pos['left']),
            'pico_right_wrist': np.array(config.init_pos['right']),
            'pico_left_arm': np.array(config.arm_init_pos['left']),
            'pico_right_arm': np.array(config.arm_init_pos['right']),
        }
        self.robot_init_rotations = {
            'pico_left_wrist': R.from_quat(config.init_quat['left']),
            'pico_right_wrist': R.from_quat(config.init_quat['right']),
            'pico_left_arm': R.from_quat(config.arm_init_quat['left']),
            'pico_right_arm': R.from_quat(config.arm_init_quat['right']),
        }

        # 默认臂角方向 (公开属性，Node 诊断日志需要访问)
        self.default_zsp_direction = {
            'left': config.get_default_zsp_direction('left'),
            'right': config.get_default_zsp_direction('right'),
        }

        # One-Euro Filter 参数 (保存用于创建滤波器实例)
        self._rate = rate
        self._min_cutoff = min_cutoff
        self._beta = beta
        self._elbow_min_cutoff = elbow_min_cutoff

        # 初始化状态
        self.initialized = False
        self.init_tracker_poses = {}   # {role: 4x4 matrix}
        self.init_hmd_pose = None      # 4x4 matrix (仅用于 HMD 可视化)

        # One-Euro Filter 实例 (按 role/side 惰性创建)
        self._pos_filters = {}         # role -> OneEuroFilter
        self._rot_filters = {}         # role -> OneEuroFilterQuat
        self._elbow_filters = {}       # side -> OneEuroFilter

    def _get_pos_filter(self, role: str) -> OneEuroFilter:
        """获取或创建位置滤波器"""
        if role not in self._pos_filters:
            self._pos_filters[role] = OneEuroFilter(
                self._rate, self._min_cutoff,
                self._beta * _POS_BETA_SCALE,
            )
        return self._pos_filters[role]

    def _get_rot_filter(self, role: str) -> OneEuroFilterQuat:
        """获取或创建姿态滤波器"""
        if role not in self._rot_filters:
            self._rot_filters[role] = OneEuroFilterQuat(
                self._rate, self._min_cutoff,
                self._beta * _ROT_BETA_SCALE,
            )
        return self._rot_filters[role]

    def _get_elbow_filter(self, side: str) -> OneEuroFilter:
        """获取或创建肘部方向滤波器"""
        if side not in self._elbow_filters:
            self._elbow_filters[side] = OneEuroFilter(
                self._rate, self._elbow_min_cutoff,
                self._beta * _ELBOW_BETA_SCALE,
            )
        return self._elbow_filters[side]

    def initialize(self, tracker_poses: dict, hmd_pose_array=None) -> set:
        """
        记录初始位姿

        Args:
            tracker_poses: {role: pose_array} 每个 tracker 的初始位姿
                          pose_array = [x, y, z, qx, qy, qz, qw]
            hmd_pose_array: HMD 初始位姿 [x,y,z,qx,qy,qz,qw] (可选, 仅用于可视化)

        Returns:
            set: 成功记录的角色集合
        """
        self.init_tracker_poses = {}
        for role, pose in tracker_poses.items():
            self.init_tracker_poses[role] = self._pose_to_matrix(pose)

        if hmd_pose_array is not None:
            self.init_hmd_pose = self._pose_to_matrix(hmd_pose_array)

        # 清除滤波器状态 (新初始化应从头开始)
        self._pos_filters.clear()
        self._rot_filters.clear()
        self._elbow_filters.clear()

        self.initialized = True
        return set(self.init_tracker_poses.keys())

    def reset(self):
        """重置初始化状态"""
        self.initialized = False
        self.init_tracker_poses = {}
        self.init_hmd_pose = None
        self._pos_filters.clear()
        self._rot_filters.clear()
        self._elbow_filters.clear()

    def compute_target_pose(self, pico_pose, role: str):
        """
        计算增量目标位姿

        Args:
            pico_pose: [x, y, z, qx, qy, qz, qw] 当前 PICO 位姿
            role: tracker 角色 (pico_left_wrist, pico_right_wrist, etc.)

        Returns:
            (position, quaternion) 在 chest 坐标系中, 或 (None, None) 如果未初始化

        数学原理:
            位置: target = robot_init + R_w2c @ pico_to_robot @ (current - init)
            姿态: target = R_w2c @ delta_world @ R_c2w @ robot_init_rot
        """
        if not self.initialized or role not in self.init_tracker_poses:
            return None, None

        current_T = self._pose_to_matrix(pico_pose)
        init_T = self.init_tracker_poses[role]
        side = 'left' if 'left' in role else 'right'

        # ==================== 位置增量 ====================
        delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]
        delta_pos_world = self.pico_to_robot @ delta_pos_pico
        delta_pos_chest = transform_world_to_chest(delta_pos_world, side)
        robot_init_pos = self.robot_init_positions.get(role, np.zeros(3))
        target_pos = robot_init_pos + delta_pos_chest

        # ==================== 姿态增量 ====================
        init_rot = R.from_matrix(init_T[:3, :3])
        current_rot = R.from_matrix(current_T[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()
        delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, self.pico_to_robot)
        robot_init_rot = self.robot_init_rotations.get(role, R.identity())
        target_rot_mat = apply_world_rotation_to_chest_pose(
            robot_init_rot.as_matrix(), delta_rot_world, side
        )
        target_quat = R.from_matrix(target_rot_mat).as_quat()  # [qx, qy, qz, qw]

        # ==================== One-Euro Filter 自适应平滑 ====================
        target_pos = self._get_pos_filter(role)(target_pos)
        target_quat = self._get_rot_filter(role)(target_quat)

        return target_pos, target_quat

    def compute_hmd_world_pose(self, pico_pose):
        """
        计算 HMD 在 world 中的位姿 (仅用于 TF 可视化)

        Returns:
            4x4 matrix 或 None
        """
        if not self.initialized or self.init_hmd_pose is None:
            return None

        current_T = self._pose_to_matrix(pico_pose)

        # 位置增量
        delta_pos_pico = current_T[:3, 3] - self.init_hmd_pose[:3, 3]
        delta_pos_robot = self.pico_to_robot @ delta_pos_pico
        target_pos = np.array([0.0, 0.0, 1.6]) + delta_pos_robot

        # 旋转增量
        current_rot = R.from_matrix(current_T[:3, :3])
        init_rot = R.from_matrix(self.init_hmd_pose[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()
        delta_rot_robot = transform_pico_rotation_to_world(delta_rot_pico, self.pico_to_robot)

        result_T = np.eye(4)
        result_T[:3, 3] = target_pos
        result_T[:3, :3] = delta_rot_robot.as_matrix()
        return result_T

    def compute_elbow_direction(self, shoulder: np.ndarray, wrist: np.ndarray,
                                elbow: np.ndarray, side: str):
        """
        计算肘部偏移方向 + 投影点 (物理约束 + One-Euro 自适应平滑)

        几何原理:
          肘部偏移 = elbow 到 shoulder-wrist 轴的垂直分量
          物理约束: 钳位到解剖学可达半空间，防止奇异穿越翻转
          One-Euro Filter 自适应平滑: 静止时强平滑, 快速变化时弱平滑

        Args:
            shoulder: 肩膀位置 (chest 坐标系, 通常为 [0,0,0])
            wrist: 手腕位置 (chest 坐标系)
            elbow: 前臂 tracker 位置 (chest 坐标系)
            side: 'left' 或 'right'

        Returns:
            (direction, proj_point) - 归一化方向向量和投影点
        """
        default_direction = self.default_zsp_direction[side]
        elbow_filter = self._get_elbow_filter(side)

        shoulder_to_wrist = wrist - shoulder
        sw_length = np.linalg.norm(shoulder_to_wrist)
        if sw_length < 1e-6:
            held = elbow_filter._x_prev if elbow_filter._x_prev is not None else default_direction.copy()
            direction = elbow_filter(held)
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                direction = direction / norm
            return direction, shoulder

        sw_unit = shoulder_to_wrist / sw_length
        shoulder_to_elbow = elbow - shoulder
        proj_length = np.dot(shoulder_to_elbow, sw_unit)
        proj_point = shoulder + proj_length * sw_unit

        elbow_offset = elbow - proj_point
        offset_length = np.linalg.norm(elbow_offset)

        if offset_length < _ELBOW_SINGULARITY_THRESHOLD:
            # 几何奇异: 手臂几乎伸直, 方向计算不可靠 (噪声放大)
            # 使用上一帧有效方向, 但仍调用滤波器保持内部状态鲜活
            # (_dx_prev 自然衰减, 手臂恢复弯曲时滤波器在干净状态等待)
            held = elbow_filter._x_prev if elbow_filter._x_prev is not None else default_direction.copy()
            direction = elbow_filter(held)
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                direction = direction / norm
            return direction, proj_point

        direction = elbow_offset / offset_length

        # 物理约束: 钳位到解剖学可达半空间
        # 防止几何奇异穿越时方向翻转到物理不可能区域 (如肘部朝上/朝前/朝内)
        # Chest 坐标系约束 (由 World 物理预期推导):
        #   X ≤ 0: 肘部在肩腕轴后方 (World -X = 后方)
        #   Y:     肘部在肩腕轴下方 (Left Chest Y+=下, Right Chest Y-=下)
        #   Z ≥ 0: 肘部在肩腕轴外侧 (Left Chest Z+=左/外, Right Chest Z+=右/外)
        direction[0] = min(direction[0], 0.0)
        if side == 'left':
            direction[1] = max(direction[1], 0.0)
        else:
            direction[1] = min(direction[1], 0.0)
        direction[2] = max(direction[2], 0.0)

        # 钳位后不归一化: 保留缩短的向量长度 (norm < 1)
        # 让 One-Euro Filter 感知到更低的变化速度 → 边界处自动增强平滑
        # 全零回退到默认方向 (三轴同时被钳位到 0 的极端情况)
        if np.linalg.norm(direction) < 1e-6:
            direction = default_direction.copy()

        # One-Euro Filter 自适应平滑
        direction = elbow_filter(direction)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm

        return direction, proj_point

    @staticmethod
    def _pose_to_matrix(pose) -> np.ndarray:
        """[x,y,z,qx,qy,qz,qw] → 4x4 变换矩阵"""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
        return T
