#!/usr/bin/env python3
"""Wuji Hand Controller (single-hand).

ROS2 communication via the wujihandros2 driver (C++ wujihandcpp SDK).
Each instance manages one hand and supports both joint-angle control and
IK-based retargeting control.

Multi-core parallel: each hand runs in its own process with its own
retargeter and its own GIL, fully utilizing multi-core CPUs.
"""
import logging
from pathlib import Path
from typing import Optional, Tuple
import numpy as np

try:
    from wujihand_output._internal.hand_interface import WujiHandROS2
except ImportError:
    from ._internal.hand_interface import WujiHandROS2

try:
    from wuji_retargeting import Retargeter
    RETARGETER_AVAILABLE = True
except ImportError:
    RETARGETER_AVAILABLE = False


class WujiHandController:
    """Single-hand Wuji Hand controller.

    Hardware communication is handled by the wujihandros2 driver
    (1000Hz control rate). One instance == one hand; multi-core
    parallelism is provided by process-level isolation.

    Two control modes are supported:
      1. Joint-angle control: set the 20 joint angles directly.
      2. IK control: drive the hand from 21 hand keypoints via inverse
         kinematics retargeting.
    """

    NUM_JOINTS = 20  # 5 fingers x 4 joints

    def __init__(
        self,
        side: str,
        hand_name: str,
        input_source: str = "manus",
        retarget_config: Optional[str] = None,
        enable_ik: bool = True,
        logger=None,
        node=None,
        nlopt_max_eval: Optional[int] = 25,
    ):
        """Initialize the single-hand controller.

        Args:
            side: "left" or "right".
            hand_name: wujihandros2 driver namespace, e.g. "left_hand".
            input_source: input source type, used to select the IK
                retargeting config file.
            retarget_config: optional explicit path to a retargeting
                config file.
            enable_ik: whether to enable IK control.
            logger: optional logger to inject (defaults to a stdlib
                logger).
            node: ROS2 node instance (required).
            nlopt_max_eval: NLOPT max evaluations per frame; pass 0 or
                a negative value to keep the wuji_retargeting library
                default (50).
        """
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger(f'WujiHandController_{side}')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        self.side = side
        self.hand_name = hand_name
        self.input_source = input_source
        self.node = node
        self._nlopt_max_eval = nlopt_max_eval

        # Hardware interface
        self.hand: Optional[WujiHandROS2] = None

        # IK retargeter
        self.retargeter: Optional['Retargeter'] = None
        self._ik_enabled = False

        if enable_ik and RETARGETER_AVAILABLE:
            self._init_retargeter(retarget_config)
        elif enable_ik and not RETARGETER_AVAILABLE:
            self.logger.warning("wuji_retargeting not installed, IK control unavailable")

        self._init_hand()
        self.logger.info(f"{side} hand controller initialized (wujihandros2)")

    def _resolve_retarget_config(self) -> Optional[str]:
        """Resolve retarget config path.

        Lookup order:
          1. retarget_{input_source}_{side}.yaml
          2. retarget_{input_source}.yaml
        """
        try:
            from ament_index_python.packages import (
                PackageNotFoundError,
                get_package_share_directory,
            )
        except ImportError:
            return None
        try:
            config_dir = Path(get_package_share_directory("wujihand_output")) / "config"
        except PackageNotFoundError:
            return None

        per_side = config_dir / f"retarget_{self.input_source}_{self.side}.yaml"
        if per_side.exists():
            return str(per_side)

        shared = config_dir / f"retarget_{self.input_source}.yaml"
        if shared.exists():
            return str(shared)

        return None

    def _init_retargeter(self, config: Optional[str]) -> None:
        config_path = config or self._resolve_retarget_config()
        if not (config_path and Path(config_path).exists()):
            self.logger.warning("IK retargeting config file not found")
            return
        self.retargeter = Retargeter.from_yaml(config_path, self.side)
        self._ik_enabled = True
        # Override library's hardcoded set_maxeval(50). Lower = faster
        # but extreme poses may regress in accuracy.
        if self._nlopt_max_eval and self._nlopt_max_eval > 0:
            self.retargeter.optimizer.opt.set_maxeval(int(self._nlopt_max_eval))
            self.logger.info(
                f"NLOPT max_eval = {int(self._nlopt_max_eval)} (library default: 50)"
            )
        self.logger.info(f"IK retargeting config: {Path(config_path).name}")

    def _init_hand(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS2 node not provided")
        self.hand = WujiHandROS2(
            hand_name=self.hand_name,
            side=self.side,
            node=self.node,
            logger=self.logger,
        )
        self.hand.connect()
        self.logger.info(f"ROS2 interface created -> /{self.hand_name}")

    # ==================== Joint-angle control ====================

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        if self.hand is not None:
            return self.hand.set_joint_positions(positions)
        return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        if self.hand is not None:
            return self.hand.get_joint_positions()
        return None

    # ==================== IK control ====================

    def is_ik_available(self) -> bool:
        return self._ik_enabled

    def retarget(self, keypoints: np.ndarray) -> Optional[np.ndarray]:
        """Retarget hand keypoints into joint angles.

        Args:
            keypoints: hand keypoints, shape (21, 3) or flat (63,).

        Returns:
            Joint-angle array of shape (20,), or None on failure.
        """
        if self.retargeter is None:
            return None
        try:
            keypoints = np.asarray(keypoints, dtype=np.float32)
            if keypoints.shape == (63,):
                keypoints = keypoints.reshape(21, 3)
            return self.retargeter.retarget(keypoints)
        except Exception as e:
            self.logger.error(f"{self.side} hand IK retargeting error: {e}")
            return None

    def set_keypoints(self, keypoints: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Drive the hand from keypoints (IK retarget + hardware control).

        Returns:
            Tuple: (success, joint angles).
        """
        angles = self.retarget(keypoints)
        if angles is not None and self.hand is not None:
            success = self.hand.set_joint_positions(angles)
            return success, angles
        return False, angles

    # ==================== Status & release ====================

    def is_connected(self) -> bool:
        return self.hand is not None and self.hand.is_connected()

    def disable_and_release(self) -> None:
        self.logger.info(f"Disabling {self.side} hand...")
        if self.hand is not None:
            self.hand.release()
            self.hand = None
        self.logger.info("Shutdown complete")
