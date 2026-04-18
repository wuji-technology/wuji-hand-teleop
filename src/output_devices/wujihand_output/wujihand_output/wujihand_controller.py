#!/usr/bin/env python3
"""
Wuji Dexterous Hand Unified Controller

Communicates via wujihandros2 driver (C++ wujihandcpp SDK) over ROS2
Supports joint angle control and IK retargeting control
"""
import logging
from pathlib import Path
from typing import Optional, List, Tuple
import numpy as np

try:
    from wujihand_output._internal.hand_interface import WujiHandROS2
except ImportError:
    from ._internal.hand_interface import WujiHandROS2

# IK retargeter (optional)
try:
    from wuji_retargeting import Retargeter
    RETARGETER_AVAILABLE = True
except ImportError:
    RETARGETER_AVAILABLE = False


class WujiHandController:
    """Wuji Dexterous Hand Unified Controller

    Communicates with hardware via wujihandros2 driver (1000Hz control frequency)

    Supports two control modes:
    1. Joint angle control: directly set 20 joint angles
    2. IK control: use hand keypoints (21 points) for inverse kinematics retargeting
    """

    NUM_JOINTS = 20  # 5 fingers x 4 joints
    JOINT_NAMES = [
        "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
        "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
        "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
        "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
        "pinky_joint_0", "pinky_joint_1", "pinky_joint_2", "pinky_joint_3",
    ]

    def __init__(
        self,
        input_source: str = "manus",
        left_retarget_config: Optional[str] = None,
        right_retarget_config: Optional[str] = None,
        enable_ik: bool = True,
        logger=None,
        # ROS2 parameters (wujihandros2)
        node=None,
        left_hand_name: Optional[str] = None,
        right_hand_name: Optional[str] = None,
    ):
        """
        Initialize Wuji dexterous hand controller

        Args:
            input_source: Input source type "manus", used to select IK retarget config
            left_retarget_config: Left hand retarget config file path (optional, for IK control)
            right_retarget_config: Right hand retarget config file path (optional, for IK control)
            enable_ik: Whether to enable IK control (default True)
            logger: External logger (optional)
            node: ROS2 node instance (required)
            left_hand_name: Left hand wujihandros2 driver namespace (e.g. "left_hand")
            right_hand_name: Right hand wujihandros2 driver namespace (e.g. "right_hand")
        """
        # Logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('WujiHandController')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        self.input_source = input_source

        # ROS2 parameters
        self.node = node
        self.left_hand_name = left_hand_name
        self.right_hand_name = right_hand_name

        # Hardware interface (WujiHandROS2)
        self.left_hand = None
        self.right_hand = None

        # IK retargeters
        self.left_retargeter: Optional['Retargeter'] = None
        self.right_retargeter: Optional['Retargeter'] = None
        self._ik_enabled = False

        # Initialize IK retargeters (if enabled and available)
        if enable_ik and RETARGETER_AVAILABLE:
            self._init_retargeters(left_retarget_config, right_retarget_config)
        elif enable_ik and not RETARGETER_AVAILABLE:
            self.logger.warning("wuji_retargeting not installed, IK control unavailable")

        # Initialize hardware (via wujihandros2)
        self._init_hands()

        self.logger.info("Wuji dexterous hand controller initialization complete (wujihandros2 mode)")

    def _resolve_retarget_config(self, side: str) -> Optional[str]:
        """Resolve retarget config path for a given input source and side.

        Lookup order:
          1. retarget_{input_source}_{side}.yaml  (per-side config)
          2. retarget_{input_source}.yaml          (shared config)
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = Path(get_package_share_directory("wujihand_output")) / "config"
        except Exception:
            return None

        per_side = config_dir / f"retarget_{self.input_source}_{side}.yaml"
        if per_side.exists():
            return str(per_side)

        shared = config_dir / f"retarget_{self.input_source}.yaml"
        if shared.exists():
            return str(shared)

        return None

    def _init_retargeters(
        self,
        left_config: Optional[str],
        right_config: Optional[str]
    ) -> None:
        """Initialize IK retargeters"""
        default_left_config = self._resolve_retarget_config("left")
        default_right_config = self._resolve_retarget_config("right")

        # Use user-provided config or default config
        right_config_path = right_config or default_right_config
        left_config_path = left_config or default_left_config

        # Initialize right hand retargeter
        if right_config_path and Path(right_config_path).exists():
            self.right_retargeter = Retargeter.from_yaml(right_config_path, "right")
            self.logger.info(f"Right hand IK retarget config: {Path(right_config_path).name}")
        else:
            self.logger.warning("Right hand IK retarget config file not found")

        # Initialize left hand retargeter
        if left_config_path and Path(left_config_path).exists():
            self.left_retargeter = Retargeter.from_yaml(left_config_path, "left")
            self.logger.info(f"Left hand IK retarget config: {Path(left_config_path).name}")
        else:
            self.logger.warning("Left hand IK retarget config file not found")

        # Mark whether IK is available
        self._ik_enabled = self.left_retargeter is not None or self.right_retargeter is not None

    def _init_hands(self) -> None:
        """Initialize dexterous hand hardware (via wujihandros2 driver)"""
        if self.node is None:
            raise RuntimeError("ROS2 node not provided, cannot initialize wujihandros2 interface")

        self.logger.info("Connecting to dexterous hands via wujihandros2 driver (1000Hz)")

        if self.left_hand_name:
            self.left_hand = WujiHandROS2(
                hand_name=self.left_hand_name,
                side="left",
                node=self.node,
                logger=self.logger
            )
            self.left_hand.connect()
            self.logger.info(f"Left hand ROS2 interface created -> /{self.left_hand_name}")

        if self.right_hand_name:
            self.right_hand = WujiHandROS2(
                hand_name=self.right_hand_name,
                side="right",
                node=self.node,
                logger=self.logger
            )
            self.right_hand.connect()
            self.logger.info(f"Right hand ROS2 interface created -> /{self.right_hand_name}")

    # ==================== Joint Angle Control ====================

    def set_joint_positions(
        self,
        left_positions: Optional[np.ndarray] = None,
        right_positions: Optional[np.ndarray] = None
    ) -> Tuple[bool, bool]:
        """
        Set dual-hand joint angles (non-blocking, for real-time control)

        Args:
            left_positions: Left hand joint angles, shape (20,) or (5, 4), None means no control
            right_positions: Right hand joint angles, shape (20,) or (5, 4), None means no control

        Returns:
            Tuple[bool, bool]: (left_success, right_success)
        """
        left_success = False
        right_success = False

        if left_positions is not None and self.left_hand is not None:
            left_success = self.left_hand.set_joint_positions(left_positions)

        if right_positions is not None and self.right_hand is not None:
            right_success = self.right_hand.set_joint_positions(right_positions)

        return left_success, right_success

    def set_joint_positions_from_flat(
        self,
        flat_positions: np.ndarray
    ) -> Tuple[bool, bool]:
        """
        Set dual-hand joint angles from a flat array

        Args:
            flat_positions: Flattened joint angle array
                - 20 values: Single hand (assigned based on enabled hand)
                - 40 values: Dual hands (right hand first, left hand second)

        Returns:
            Tuple[bool, bool]: (left_success, right_success)
        """
        flat_positions = np.asarray(flat_positions, dtype=np.float32)

        if flat_positions.size == 40:
            # Dual hands: right(0:20) + left(20:40)
            right_pos = flat_positions[:20]
            left_pos = flat_positions[20:]
            return self.set_joint_positions(left_pos, right_pos)
        elif flat_positions.size == 20:
            # Single hand: assign based on enabled hand
            if self.left_hand is not None and self.right_hand is None:
                return self.set_joint_positions(flat_positions, None)
            else:
                return self.set_joint_positions(None, flat_positions)
        else:
            self.logger.error(f"Invalid joint angle count: {flat_positions.size}, expected 20 or 40")
            return False, False

    def get_joint_positions(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get current dual-hand joint angles

        Returns:
            Tuple: (left_hand_joint_angles, right_hand_joint_angles), each is (20,) array or None
        """
        left_pos = None
        right_pos = None

        if self.left_hand is not None:
            left_pos = self.left_hand.get_joint_positions()

        if self.right_hand is not None:
            right_pos = self.right_hand.get_joint_positions()

        return left_pos, right_pos

    # ==================== IK Control ====================

    def is_ik_available(self) -> bool:
        """Check whether IK control is available"""
        return self._ik_enabled

    def retarget(
        self,
        left_keypoints: Optional[np.ndarray] = None,
        right_keypoints: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Retarget hand keypoints to compute joint angles

        Args:
            left_keypoints: Left hand keypoints, shape (21, 3), None means skip
            right_keypoints: Right hand keypoints, shape (21, 3), None means skip

        Returns:
            Tuple: (left_hand_joint_angles, right_hand_joint_angles), each is (20,) array or None
        """
        left_angles = None
        right_angles = None

        if left_keypoints is not None and self.left_retargeter is not None:
            try:
                left_keypoints = np.asarray(left_keypoints, dtype=np.float32)
                if left_keypoints.shape == (63,):
                    left_keypoints = left_keypoints.reshape(21, 3)
                left_angles = self.left_retargeter.retarget(left_keypoints)
            except Exception as e:
                self.logger.error(f"Left hand IK retarget error: {e}")

        if right_keypoints is not None and self.right_retargeter is not None:
            try:
                right_keypoints = np.asarray(right_keypoints, dtype=np.float32)
                if right_keypoints.shape == (63,):
                    right_keypoints = right_keypoints.reshape(21, 3)
                right_angles = self.right_retargeter.retarget(right_keypoints)
            except Exception as e:
                self.logger.error(f"Right hand IK retarget error: {e}")

        return left_angles, right_angles

    def set_keypoints(
        self,
        left_keypoints: Optional[np.ndarray] = None,
        right_keypoints: Optional[np.ndarray] = None
    ) -> Tuple[bool, bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Control dexterous hands using hand keypoints (IK retargeting + hardware control)

        Args:
            left_keypoints: Left hand keypoints, shape (21, 3) or (63,), None means no control
            right_keypoints: Right hand keypoints, shape (21, 3) or (63,), None means no control

        Returns:
            Tuple: (left_success, right_success, left_joint_angles, right_joint_angles)
        """
        # Retarget to compute joint angles
        left_angles, right_angles = self.retarget(left_keypoints, right_keypoints)

        left_success = False
        right_success = False

        # Control left hand
        if left_angles is not None and self.left_hand is not None:
            left_success = self.left_hand.set_joint_positions(left_angles)

        # Control right hand
        if right_angles is not None and self.right_hand is not None:
            right_success = self.right_hand.set_joint_positions(right_angles)

        return left_success, right_success, left_angles, right_angles

    def set_keypoints_from_flat(
        self,
        flat_keypoints: np.ndarray
    ) -> Tuple[bool, bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Set hand keypoints from a flat array

        Args:
            flat_keypoints: Flattened keypoint array
                - 63 values: Single hand (21 * 3), assigned based on enabled hand
                - 126 values: Dual hands (right hand first, left hand second)

        Returns:
            Tuple: (left_success, right_success, left_joint_angles, right_joint_angles)
        """
        flat_keypoints = np.asarray(flat_keypoints, dtype=np.float32)
        single_len = 21 * 3  # 63
        double_len = single_len * 2  # 126

        if flat_keypoints.size == double_len:
            # Dual hands: right(0:63) + left(63:126)
            right_kp = flat_keypoints[:single_len].reshape(21, 3)
            left_kp = flat_keypoints[single_len:].reshape(21, 3)
            return self.set_keypoints(left_kp, right_kp)
        elif flat_keypoints.size == single_len:
            # Single hand: assign based on enabled hand
            kp = flat_keypoints.reshape(21, 3)
            if self.left_hand is not None and self.right_hand is None:
                return self.set_keypoints(kp, None)
            else:
                return self.set_keypoints(None, kp)
        else:
            self.logger.error(f"Invalid keypoint count: {flat_keypoints.size}, expected 63 or 126")
            return False, False, None, None

    # ==================== Status Check and Release ====================

    def is_left_connected(self) -> bool:
        """Check whether left hand is connected"""
        return self.left_hand is not None and self.left_hand.is_connected()

    def is_right_connected(self) -> bool:
        """Check whether right hand is connected"""
        return self.right_hand is not None and self.right_hand.is_connected()

    def get_enabled_hands(self) -> List[str]:
        """Get list of enabled hands"""
        hands = []
        if self.is_left_connected():
            hands.append("left")
        if self.is_right_connected():
            hands.append("right")
        return hands

    def disable_and_release(self) -> None:
        """Disable and release both hands"""
        self.logger.info("Disabling dexterous hands...")

        if self.left_hand is not None:
            self.left_hand.release()
            self.left_hand = None

        if self.right_hand is not None:
            self.right_hand.release()
            self.right_hand = None

        self.logger.info("Safely exited")
