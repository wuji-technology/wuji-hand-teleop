#!/usr/bin/env python3
"""
Incremental Controller - Core computation logic (no ROS2 dependency)

Pure computation class extracted from pico_input_node.py, can be unit tested independently.

Responsibilities:
  1. Record tracker initial poses
  2. Compute incremental changes relative to initial poses (position + orientation)
  3. Map increments to robot chest coordinate system
  4. One-Euro Filter adaptive smoothing (position + orientation + elbow direction)
  5. Elbow direction computation (physical constraints + adaptive smoothing)

Data flow:
  PICO raw pose -> IncrementalController -> chest coordinate system target pose

Dependencies:
  - numpy, scipy (pure math)
  - tianji_world_output.config_loader (configuration)
  - tianji_world_output.transform_utils (coordinate transform shared library)
  - pico_input.one_euro_filter (adaptive filtering)
  - No ROS2 dependency
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


# Beta internal scaling coefficients (map user's unified beta to each signal domain)
# Position velocity ~0-2 m/s, needs large beta to open cutoff frequency during fast motion
_POS_BETA_SCALE = 20.0
# Angular velocity ~0-10 rad/s, moderate beta
_ROT_BETA_SCALE = 4.0
# Direction change rate ~0-5 unit/s, moderate beta
_ELBOW_BETA_SCALE = 4.0

# Elbow direction geometric singularity threshold (meters)
# When the perpendicular distance from arm tracker to shoulder-wrist axis is less than this value,
# direction computation is unreliable (noise amplification)
# 15mm ~ forearm length 0.25m x sin(3.4 deg), i.e. arm bend angle < 3.4 deg is considered geometric singularity
# PICO tracker position noise ~+/-0.5mm (raw), ~+/-0.1mm (after One-Euro filter)
# At 15mm, noise causes direction error < +/-1.9 deg (< +/-0.04 deg after One-Euro)
_ELBOW_SINGULARITY_THRESHOLD = 0.015


class IncrementalController:
    """
    Incremental pose controller (pure computation, no ROS2 dependency)

    Design principles:
      - All state and computation centrally managed
      - No I/O, no ROS2 dependency
      - After construction, call initialize() to set initial poses
      - Then call compute_target_pose() each frame to get target pose

    Filtering strategy (One-Euro Filter):
      - At rest: low cutoff frequency -> strong smoothing (suppresses sensor jitter)
      - During fast motion: high cutoff frequency -> weak smoothing (low latency, responsive)
      - Position, orientation, and elbow direction all use One-Euro Filter
    """

    def __init__(self, config: TianjiConfig,
                 rate: float = 90.0,
                 min_cutoff: float = 1.0,
                 beta: float = 0.7,
                 elbow_min_cutoff: float = 0.3):
        """
        Args:
            config: TianjiConfig unified configuration
            rate: Sampling rate (Hz)
            min_cutoff: One-Euro minimum cutoff frequency (Hz), controls smoothing strength at rest
            beta: One-Euro speed response coefficient (internally scaled by signal type)
            elbow_min_cutoff: Elbow direction minimum cutoff frequency (Hz), typically < min_cutoff
        """
        # PICO->Robot transform matrix
        self.pico_to_robot = config.pico_to_robot

        # Robot initial poses (chest coordinate system)
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

        # Default arm angle direction (public attribute, Node diagnostic logs need access)
        self.default_zsp_direction = {
            'left': config.get_default_zsp_direction('left'),
            'right': config.get_default_zsp_direction('right'),
        }

        # One-Euro Filter parameters (saved for creating filter instances)
        self._rate = rate
        self._min_cutoff = min_cutoff
        self._beta = beta
        self._elbow_min_cutoff = elbow_min_cutoff

        # Initialization state
        self.initialized = False
        self.init_tracker_poses = {}   # {role: 4x4 matrix}
        self.init_hmd_pose = None      # 4x4 matrix (only for HMD visualization)

        # One-Euro Filter instances (lazily created per role/side)
        self._pos_filters = {}         # role -> OneEuroFilter
        self._rot_filters = {}         # role -> OneEuroFilterQuat
        self._elbow_filters = {}       # side -> OneEuroFilter

    def _get_pos_filter(self, role: str) -> OneEuroFilter:
        """Get or create position filter"""
        if role not in self._pos_filters:
            self._pos_filters[role] = OneEuroFilter(
                self._rate, self._min_cutoff,
                self._beta * _POS_BETA_SCALE,
            )
        return self._pos_filters[role]

    def _get_rot_filter(self, role: str) -> OneEuroFilterQuat:
        """Get or create orientation filter"""
        if role not in self._rot_filters:
            self._rot_filters[role] = OneEuroFilterQuat(
                self._rate, self._min_cutoff,
                self._beta * _ROT_BETA_SCALE,
            )
        return self._rot_filters[role]

    def _get_elbow_filter(self, side: str) -> OneEuroFilter:
        """Get or create elbow direction filter"""
        if side not in self._elbow_filters:
            self._elbow_filters[side] = OneEuroFilter(
                self._rate, self._elbow_min_cutoff,
                self._beta * _ELBOW_BETA_SCALE,
            )
        return self._elbow_filters[side]

    def initialize(self, tracker_poses: dict, hmd_pose_array=None) -> set:
        """
        Record initial poses

        Args:
            tracker_poses: {role: pose_array} initial pose for each tracker
                          pose_array = [x, y, z, qx, qy, qz, qw]
            hmd_pose_array: HMD initial pose [x,y,z,qx,qy,qz,qw] (optional, visualization only)

        Returns:
            set: Set of successfully recorded roles
        """
        self.init_tracker_poses = {}
        for role, pose in tracker_poses.items():
            self.init_tracker_poses[role] = self._pose_to_matrix(pose)

        if hmd_pose_array is not None:
            self.init_hmd_pose = self._pose_to_matrix(hmd_pose_array)

        # Clear filter state (new initialization should start fresh)
        self._pos_filters.clear()
        self._rot_filters.clear()
        self._elbow_filters.clear()

        self.initialized = True
        return set(self.init_tracker_poses.keys())

    def reset(self):
        """Reset initialization state"""
        self.initialized = False
        self.init_tracker_poses = {}
        self.init_hmd_pose = None
        self._pos_filters.clear()
        self._rot_filters.clear()
        self._elbow_filters.clear()

    def compute_target_pose(self, pico_pose, role: str):
        """
        Compute incremental target pose

        Args:
            pico_pose: [x, y, z, qx, qy, qz, qw] current PICO pose
            role: tracker role (pico_left_wrist, pico_right_wrist, etc.)

        Returns:
            (position, quaternion) in chest coordinate system, or (None, None) if not initialized

        Mathematical principle:
            Position: target = robot_init + R_w2c @ pico_to_robot @ (current - init)
            Orientation: target = R_w2c @ delta_world @ R_c2w @ robot_init_rot
        """
        if not self.initialized or role not in self.init_tracker_poses:
            return None, None

        current_T = self._pose_to_matrix(pico_pose)
        init_T = self.init_tracker_poses[role]
        side = 'left' if 'left' in role else 'right'

        # ==================== Position increment ====================
        delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]
        delta_pos_world = self.pico_to_robot @ delta_pos_pico
        delta_pos_chest = transform_world_to_chest(delta_pos_world, side)
        robot_init_pos = self.robot_init_positions.get(role, np.zeros(3))
        target_pos = robot_init_pos + delta_pos_chest

        # ==================== Orientation increment ====================
        init_rot = R.from_matrix(init_T[:3, :3])
        current_rot = R.from_matrix(current_T[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()
        delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, self.pico_to_robot)
        robot_init_rot = self.robot_init_rotations.get(role, R.identity())
        target_rot_mat = apply_world_rotation_to_chest_pose(
            robot_init_rot.as_matrix(), delta_rot_world, side
        )
        target_quat = R.from_matrix(target_rot_mat).as_quat()  # [qx, qy, qz, qw]

        # ==================== One-Euro Filter adaptive smoothing ====================
        target_pos = self._get_pos_filter(role)(target_pos)
        target_quat = self._get_rot_filter(role)(target_quat)

        return target_pos, target_quat

    def compute_hmd_world_pose(self, pico_pose):
        """
        Compute HMD pose in world frame (for TF visualization only)

        Returns:
            4x4 matrix or None
        """
        if not self.initialized or self.init_hmd_pose is None:
            return None

        current_T = self._pose_to_matrix(pico_pose)

        # Position increment
        delta_pos_pico = current_T[:3, 3] - self.init_hmd_pose[:3, 3]
        delta_pos_robot = self.pico_to_robot @ delta_pos_pico
        target_pos = np.array([0.0, 0.0, 1.6]) + delta_pos_robot

        # Rotation increment
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
        Compute elbow offset direction + projection point (physical constraints + One-Euro adaptive smoothing)

        Geometric principle:
          Elbow offset = perpendicular component from elbow to shoulder-wrist axis
          Physical constraint: clamped to anatomically reachable half-space, prevents singularity crossing flip
          One-Euro Filter adaptive smoothing: strong smoothing at rest, weak smoothing during fast changes

        Args:
            shoulder: Shoulder position (chest coordinate system, typically [0,0,0])
            wrist: Wrist position (chest coordinate system)
            elbow: Forearm tracker position (chest coordinate system)
            side: 'left' or 'right'

        Returns:
            (direction, proj_point) - normalized direction vector and projection point
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
            # Geometric singularity: arm nearly straight, direction computation unreliable (noise amplification)
            # Use last valid direction, but still call filter to keep internal state fresh
            # (_dx_prev decays naturally, filter waits in clean state when arm resumes bending)
            held = elbow_filter._x_prev if elbow_filter._x_prev is not None else default_direction.copy()
            direction = elbow_filter(held)
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                direction = direction / norm
            return direction, proj_point

        direction = elbow_offset / offset_length

        # Physical constraint: clamp to anatomically reachable half-space
        # Prevents direction flipping to physically impossible region during geometric singularity crossing
        # (e.g. elbow pointing up/forward/inward)
        # Chest coordinate system constraints (derived from World physical expectations):
        #   X <= 0: elbow behind shoulder-wrist axis (World -X = backward)
        #   Y:     elbow below shoulder-wrist axis (Left Chest Y+=down, Right Chest Y-=down)
        #   Z >= 0: elbow outside shoulder-wrist axis (Left Chest Z+=left/out, Right Chest Z+=right/out)
        direction[0] = min(direction[0], 0.0)
        if side == 'left':
            direction[1] = max(direction[1], 0.0)
        else:
            direction[1] = min(direction[1], 0.0)
        direction[2] = max(direction[2], 0.0)

        # No normalization after clamping: preserve shortened vector length (norm < 1)
        # Lets One-Euro Filter perceive lower change rate -> automatically enhanced smoothing at boundary
        # Fall back to default direction if all-zero (extreme case where all three axes clamped to 0)
        if np.linalg.norm(direction) < 1e-6:
            direction = default_direction.copy()

        # One-Euro Filter adaptive smoothing
        direction = elbow_filter(direction)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm

        return direction, proj_point

    @staticmethod
    def _pose_to_matrix(pose) -> np.ndarray:
        """[x,y,z,qx,qy,qz,qw] -> 4x4 transform matrix"""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
        return T
