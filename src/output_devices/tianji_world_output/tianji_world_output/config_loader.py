#!/usr/bin/env python3
"""
Tianji Robot Configuration Loader

Unified loader for tianji_robot.yaml config file, providing type-safe access interface.

Usage:
    from tianji_world_output.config_loader import TianjiConfig

    # Load config (auto-locates config file)
    config = TianjiConfig.load()

    # Access config
    init_pos = config.init_pos['left']          # numpy array
    init_rot = config.init_rot['left']          # numpy 3x3 matrix
    init_quat = config.init_quat['left']        # numpy array [qx, qy, qz, qw]
    world_to_chest_quat = config.world_to_chest_quat['left']

    # Get raw config dictionary
    raw = config.raw

    # Use in test scripts (no ROS environment needed):
    config = TianjiConfig.load(use_ros=False)
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any, Optional
import numpy as np

try:
    import yaml
except ImportError as exc:
    raise ImportError("PyYAML is required. Install with: pip install pyyaml") from exc


@dataclass
class TianjiConfig:
    """Tianji Robot Configuration"""

    # Raw config dictionary
    raw: Dict[str, Any]

    # Robot connection
    robot_ip: str

    # Initial joint angles
    init_joints: Dict[str, np.ndarray]

    # Initial end-effector pose (Chest coordinate frame)
    init_pos: Dict[str, np.ndarray]     # [x, y, z] meters
    init_rot: Dict[str, np.ndarray]     # 3x3 rotation matrix
    init_quat: Dict[str, np.ndarray]    # [qx, qy, qz, qw]

    # World -> Chest coordinate transform
    world_to_chest_quat: Dict[str, np.ndarray]  # [qx, qy, qz, qw]
    world_to_chest_trans: Dict[str, np.ndarray]  # [x, y, z]

    # Forearm tracker initial position (Chest coordinate frame)
    arm_init_pos: Dict[str, np.ndarray]    # [x, y, z] meters
    arm_init_quat: Dict[str, np.ndarray]   # [qx, qy, qz, qw]

    # PICO -> Robot coordinate transform
    pico_to_robot: np.ndarray  # 3x3 matrix

    # IK parameters
    zsp_type: int
    default_zsp_para: Dict[str, list]
    zsp_angle: float
    dgr: list

    # Tracker mapping
    tracker_serial_map: Dict[str, str]

    # Config file path
    config_path: Path

    @classmethod
    def load(cls, config_path: Optional[str] = None, use_ros: bool = True) -> 'TianjiConfig':
        """
        Load configuration file

        Args:
            config_path: Config file path, None for auto-locate
            use_ros: Whether to use ROS2 package lookup mechanism

        Returns:
            TianjiConfig instance
        """
        if config_path is None:
            config_path = cls._find_config_file(use_ros)

        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")

        with path.open('r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        return cls._parse(raw, path)

    @classmethod
    def _find_config_file(cls, use_ros: bool = True) -> str:
        """Find config file"""
        # Method 1: Use ROS2 package lookup
        if use_ros:
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('tianji_world_output')
                config_path = Path(share_dir) / 'config' / 'tianji_robot.yaml'
                if config_path.exists():
                    return str(config_path)
            except Exception:
                pass

        # Method 2: Search relative to current file
        current_dir = Path(__file__).parent
        possible_paths = [
            current_dir / 'config' / 'tianji_robot.yaml',
            current_dir.parent / 'config' / 'tianji_robot.yaml',
        ]

        for path in possible_paths:
            if path.exists():
                return str(path)

        raise FileNotFoundError(
            "Cannot find tianji_robot.yaml config file.\n"
            "Please ensure the file is located in the tianji_world_output/config/ directory."
        )

    @classmethod
    def _parse(cls, raw: Dict[str, Any], config_path: Path) -> 'TianjiConfig':
        """Parse config dictionary"""

        def to_numpy_dict(d: Dict[str, list]) -> Dict[str, np.ndarray]:
            return {k: np.array(v) for k, v in d.items()}

        return cls(
            raw=raw,
            robot_ip=raw.get('robot_ip', '192.168.1.190'),
            init_joints=to_numpy_dict(raw.get('init_joints', {})),
            init_pos=to_numpy_dict(raw.get('init_pos', {})),
            init_rot=to_numpy_dict(raw.get('init_rot', {})),
            init_quat=to_numpy_dict(raw.get('init_quat', {})),
            world_to_chest_quat=to_numpy_dict(raw.get('world_to_chest_quat', {})),
            world_to_chest_trans=to_numpy_dict(raw.get('world_to_chest_trans', {})),
            arm_init_pos=to_numpy_dict(raw.get('arm_init_pos', {})),
            arm_init_quat=to_numpy_dict(raw.get('arm_init_quat', {})),
            pico_to_robot=np.array(raw.get('pico_to_robot', [[0, 0, -1], [-1, 0, 0], [0, 1, 0]])),
            zsp_type=raw.get('zsp_type', 1),
            default_zsp_para=raw.get('default_zsp_para', {}),
            zsp_angle=raw.get('zsp_angle', 0.0),
            dgr=raw.get('dgr', [5.0, 5.0, 5.0]),
            tracker_serial_map=raw.get('tracker_serial_map', {}),
            config_path=config_path,
        )

    def get_world_to_chest_rotation(self, side: str) -> np.ndarray:
        """Get World -> Chest rotation matrix"""
        from scipy.spatial.transform import Rotation as R
        quat = self.world_to_chest_quat[side]
        return R.from_quat(quat).as_matrix()

    def get_chest_to_world_rotation(self, side: str) -> np.ndarray:
        """Get Chest -> World rotation matrix (inverse transform)"""
        return self.get_world_to_chest_rotation(side).T

    def get_default_zsp_direction(self, side: str) -> np.ndarray:
        """Get normalized default zsp direction vector (first 3 components)"""
        zsp = self.default_zsp_para.get(side, [0, -1, -0.5, 0, 0, 0] if side == 'left' else [0, 1, -0.5, 0, 0, 0])
        direction = np.array(zsp[:3], dtype=float)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        return direction

    def get_kine_config_path(self) -> str:
        """Get kinematics config file path (ccs_m6.MvKDCfg)"""
        # Config file is in the tianji_world_output/config/ directory
        kine_file = self.raw.get('kine_config_file', 'ccs_m6.MvKDCfg')

        # Prefer path relative to this file
        local_path = Path(__file__).parent / 'config' / kine_file
        if local_path.exists():
            return str(local_path)

        # Try from share directory (after ROS2 install)
        try:
            from ament_index_python.packages import get_package_share_directory
            share_path = Path(get_package_share_directory('tianji_world_output')) / 'config' / kine_file
            if share_path.exists():
                return str(share_path)
        except Exception:
            pass

        # Fallback to legacy path
        return str(self.config_path.parent / kine_file)


# Global singleton (lazy-loaded)
_config_instance: Optional[TianjiConfig] = None


def get_config(use_ros: bool = True) -> TianjiConfig:
    """Get config singleton"""
    global _config_instance
    if _config_instance is None:
        _config_instance = TianjiConfig.load(use_ros=use_ros)
    return _config_instance


def reload_config(config_path: Optional[str] = None, use_ros: bool = True) -> TianjiConfig:
    """Reload configuration"""
    global _config_instance
    _config_instance = TianjiConfig.load(config_path=config_path, use_ros=use_ros)
    return _config_instance
