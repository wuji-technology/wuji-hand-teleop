"""
Common utility module - shared utility classes and functions for controller nodes

Contains:
- ControlMode: Control mode enumeration
- ROS2LoggerAdapter: ROS2 logger adapter
- Default QoS configuration
- Configuration loading utilities
"""
from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Dict, Any, Optional

import yaml
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ControlMode(Enum):
    """Control mode enumeration

    - TELEOP: Teleoperation mode (uses kinematic solving)
    - INFERENCE: Inference mode (direct joint control)
    """
    TELEOP = "teleop"
    INFERENCE = "inference"


class ROS2LoggerAdapter:
    """ROS2 logger adapter

    Adapts ROS2 logger to standard logging interface,
    for use by underlying controller libraries.
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
    """Get default QoS configuration (for real-time control)"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def load_yaml_config(config_path: str | Path) -> Dict[str, Any]:
    """Load YAML configuration file

    Args:
        config_path: Configuration file path

    Returns:
        Configuration dictionary

    Raises:
        FileNotFoundError: Configuration file does not exist
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get_package_config_path(package_name: str, config_filename: str) -> Optional[Path]:
    """Get configuration file path within a ROS2 package

    Args:
        package_name: ROS2 package name
        config_filename: Configuration filename (under config directory)

    Returns:
        Configuration file path, returns None if package does not exist
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory(package_name))
        return share_dir / "config" / config_filename
    except Exception:
        return None
