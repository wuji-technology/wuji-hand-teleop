#!/usr/bin/env python3
"""
天机机器人配置加载器

统一加载 tianji_robot.yaml 配置文件，提供类型安全的访问接口。

使用方法:
    from tianji_world_output.config_loader import TianjiConfig

    # 加载配置 (自动查找配置文件)
    config = TianjiConfig.load()

    # 访问配置
    init_pos = config.init_pos['left']          # numpy array
    init_rot = config.init_rot['left']          # numpy 3x3 matrix
    init_quat = config.init_quat['left']        # numpy array [qx, qy, qz, qw]
    world_to_chest_quat = config.world_to_chest_quat['left']

    # 获取完整配置字典
    raw = config.raw

    # 在测试脚本中使用 (不需要 ROS 环境):
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
    """天机机器人配置"""

    # 原始配置字典
    raw: Dict[str, Any]

    # 机器人连接
    robot_ip: str

    # 初始关节角度
    init_joints: Dict[str, np.ndarray]

    # 初始末端位姿 (Chest 坐标系)
    init_pos: Dict[str, np.ndarray]     # [x, y, z] 米
    init_rot: Dict[str, np.ndarray]     # 3x3 旋转矩阵
    init_quat: Dict[str, np.ndarray]    # [qx, qy, qz, qw]

    # World → Chest 坐标变换
    world_to_chest_quat: Dict[str, np.ndarray]  # [qx, qy, qz, qw]
    world_to_chest_trans: Dict[str, np.ndarray]  # [x, y, z]

    # 前臂 Tracker 初始位置 (Chest 坐标系)
    arm_init_pos: Dict[str, np.ndarray]    # [x, y, z] 米
    arm_init_quat: Dict[str, np.ndarray]   # [qx, qy, qz, qw]

    # PICO → Robot 坐标变换
    pico_to_robot: np.ndarray  # 3x3 矩阵

    # IK 参数
    zsp_type: int
    default_zsp_para: Dict[str, list]
    zsp_angle: float
    dgr: list

    # Tracker 映射
    tracker_serial_map: Dict[str, str]

    # 配置文件路径
    config_path: Path

    @classmethod
    def load(cls, config_path: Optional[str] = None, use_ros: bool = True) -> 'TianjiConfig':
        """
        加载配置文件

        Args:
            config_path: 配置文件路径，None 则自动查找
            use_ros: 是否使用 ROS2 包查找机制

        Returns:
            TianjiConfig 实例
        """
        if config_path is None:
            config_path = cls._find_config_file(use_ros)

        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"配置文件不存在: {path}")

        with path.open('r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        return cls._parse(raw, path)

    @classmethod
    def _find_config_file(cls, use_ros: bool = True) -> str:
        """查找配置文件"""
        # 方法1: 使用 ROS2 包查找
        if use_ros:
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('tianji_world_output')
                config_path = Path(share_dir) / 'config' / 'tianji_robot.yaml'
                if config_path.exists():
                    return str(config_path)
            except Exception:
                pass

        # 方法2: 相对于当前文件查找
        current_dir = Path(__file__).parent
        possible_paths = [
            current_dir / 'config' / 'tianji_robot.yaml',
            current_dir.parent / 'config' / 'tianji_robot.yaml',
        ]

        for path in possible_paths:
            if path.exists():
                return str(path)

        raise FileNotFoundError(
            "找不到 tianji_robot.yaml 配置文件。\n"
            "请确保文件位于 tianji_world_output/config/ 目录下。"
        )

    @classmethod
    def _parse(cls, raw: Dict[str, Any], config_path: Path) -> 'TianjiConfig':
        """解析配置字典"""

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
        """获取 World → Chest 的旋转矩阵"""
        from scipy.spatial.transform import Rotation as R
        quat = self.world_to_chest_quat[side]
        return R.from_quat(quat).as_matrix()

    def get_chest_to_world_rotation(self, side: str) -> np.ndarray:
        """获取 Chest → World 的旋转矩阵 (逆变换)"""
        return self.get_world_to_chest_rotation(side).T

    def get_default_zsp_direction(self, side: str) -> np.ndarray:
        """获取归一化的默认 zsp 方向向量 (前3个分量)"""
        zsp = self.default_zsp_para.get(side, [0, -1, -0.5, 0, 0, 0] if side == 'left' else [0, 1, -0.5, 0, 0, 0])
        direction = np.array(zsp[:3], dtype=float)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        return direction

    def get_kine_config_path(self) -> str:
        """获取运动学配置文件路径 (ccs_m6.MvKDCfg)"""
        # 配置文件位于 tianji_world_output/config/ 目录下
        kine_file = self.raw.get('kine_config_file', 'ccs_m6.MvKDCfg')

        # 优先查找相对于此文件的路径
        local_path = Path(__file__).parent / 'config' / kine_file
        if local_path.exists():
            return str(local_path)

        # 尝试从 share 目录查找 (ROS2 安装后)
        try:
            from ament_index_python.packages import get_package_share_directory
            share_path = Path(get_package_share_directory('tianji_world_output')) / 'config' / kine_file
            if share_path.exists():
                return str(share_path)
        except Exception:
            pass

        # 回退到旧路径
        return str(self.config_path.parent / kine_file)


# 全局单例 (延迟加载)
_config_instance: Optional[TianjiConfig] = None


def get_config(use_ros: bool = True) -> TianjiConfig:
    """获取配置单例"""
    global _config_instance
    if _config_instance is None:
        _config_instance = TianjiConfig.load(use_ros=use_ros)
    return _config_instance


def reload_config(config_path: Optional[str] = None, use_ros: bool = True) -> TianjiConfig:
    """重新加载配置"""
    global _config_instance
    _config_instance = TianjiConfig.load(config_path=config_path, use_ros=use_ros)
    return _config_instance
