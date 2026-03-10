"""
公共工具模块 - 控制器节点共享的工具类和函数

包含：
- ControlMode: 控制模式枚举
- ROS2LoggerAdapter: ROS2 日志适配器
- 默认 QoS 配置
- 配置加载工具
"""
from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Dict, Any, Optional

import yaml
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ControlMode(Enum):
    """控制模式枚举

    - TELEOP: 遥操作模式（使用运动学解算）
    - INFERENCE: 推理模式（直接关节控制）
    """
    TELEOP = "teleop"
    INFERENCE = "inference"


class ROS2LoggerAdapter:
    """ROS2 日志适配器

    将 ROS2 logger 适配为标准 logging 接口，
    供底层控制器库使用。
    """

    def __init__(self, ros_logger):
        self._logger = ros_logger

    def info(self, msg: str) -> None:
        self._logger.info(msg)

    def debug(self, msg: str) -> None:
        self._logger.debug(msg)

    def warning(self, msg: str) -> None:
        self._logger.warning(msg)

    def error(self, msg: str) -> None:
        self._logger.error(msg)


def get_default_qos() -> QoSProfile:
    """获取默认 QoS 配置（用于实时控制）"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def load_yaml_config(config_path: str | Path) -> Dict[str, Any]:
    """加载 YAML 配置文件

    Args:
        config_path: 配置文件路径

    Returns:
        配置字典

    Raises:
        FileNotFoundError: 配置文件不存在
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"配置文件不存在: {path}")

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get_package_config_path(package_name: str, config_filename: str) -> Optional[Path]:
    """获取 ROS2 包内的配置文件路径

    Args:
        package_name: ROS2 包名
        config_filename: 配置文件名（在 config 目录下）

    Returns:
        配置文件路径，若包不存在则返回 None
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory(package_name))
        return share_dir / "config" / config_filename
    except Exception:
        return None
