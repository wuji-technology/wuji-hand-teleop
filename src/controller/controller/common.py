"""
Shared utilities — common classes and helpers used by controller nodes.

Includes:
- ArmState: arm-motor control-state enum
- ROS2LoggerAdapter: ROS2 logger adapter
- Default QoS profile
- Config-loading helpers
"""
from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Dict, Any, Optional

import yaml
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ArmState(Enum):
    """Arm-motor control state (mirrors SDK ARM_STATE)."""
    IMPEDANCE = "impedance"  # state=3, compliant (default; suited to teleop)
    POSITION = "position"    # state=1, rigid (suited to payload testing)


class ROS2LoggerAdapter:
    """ROS2 logger adapter.

    Adapts a ROS2 logger to the standard logging interface (with stdlib
    %-format lazy formatting) so it can drive lower-level controller libraries.
    rclpy logger itself does not accept extra args; this adapter lazy-formats
    and then forwards to the ROS2 logger.

    Mirrors the equivalent implementation in
    input_devices/pico_input/pico_input/ros2_logging.py.
    """

    def __init__(self, ros_logger):
        self._logger = ros_logger

    def _format(self, msg, args):
        return msg % args if args else msg

    def info(self, msg, *args):
        self._logger.info(self._format(msg, args))

    def debug(self, msg, *args):
        self._logger.debug(self._format(msg, args))

    def warning(self, msg, *args):
        self._logger.warning(self._format(msg, args))

    def error(self, msg, *args):
        self._logger.error(self._format(msg, args))


def get_default_qos() -> QoSProfile:
    """Default QoS profile (for real-time control)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def load_yaml_config(config_path: str | Path) -> Dict[str, Any]:
    """Load a YAML config file.

    Args:
        config_path: path to the config file.

    Returns:
        Parsed config dict.

    Raises:
        FileNotFoundError: when the config file does not exist.
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get_package_config_path(package_name: str, config_filename: str) -> Optional[Path]:
    """Resolve the path to a config file shipped inside a ROS2 package.

    Args:
        package_name: ROS2 package name.
        config_filename: config filename (under the package's config/ dir).

    Returns:
        Path to the config file, or None if the package cannot be located.
    """
    from ament_index_python.packages import (
        get_package_share_directory,
        PackageNotFoundError,
    )
    try:
        share_dir = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        # Package not installed in the current AMENT_PREFIX_PATH — the
        # caller has to handle this (typically: fall back to a CLI-supplied
        # path). Any other ament error is a real install bug we want
        # propagated.
        return None
    return share_dir / "config" / config_filename
