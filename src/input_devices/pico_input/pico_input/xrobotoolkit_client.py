#!/usr/bin/env python3
"""
XRoboToolkit Python Client - PICO VR device data client using Pybind SDK

Connects to XRoboToolkit PC-Service via xrobotoolkit_sdk (Pybind) to obtain PICO device tracking data.

Data flow:
    PICO headset -> XRoboToolkit APK -> WiFi -> PC-Service -> Pybind SDK -> This client

Usage:
    from pico_input.xrobotoolkit_client import XRoboToolkitClient

    client = XRoboToolkitClient()
    client.init()

    # Get data
    hmd_pose = client.get_headset_pose()       # [x, y, z, qx, qy, qz, qw]
    trackers = client.get_motion_tracker_pose() # [[x,y,z,qx,qy,qz,qw], ...]
    num = client.num_motion_data_available()

    client.close()

Dependencies:
    - xrobotoolkit_sdk (Pybind): vendored at src/input_devices/pico_input/vendor/XRoboToolkit-PC-Service-Pybind/
      (upstream: https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind, MIT)
"""

import logging
from typing import List, Optional

logger = logging.getLogger(__name__)

# Try to import Pybind SDK
try:
    import xrobotoolkit_sdk as xrt
    _USE_PYBIND = True
except ImportError:
    _USE_PYBIND = False
    logger.warning("xrobotoolkit_sdk not installed, unable to connect to PC-Service")
    logger.warning("Installation: cd ~/Desktop/XRoboToolkit-PC-Service-Pybind && pip install .")


class XRoboToolkitClient:
    """
    XRoboToolkit Python Client (Pybind version)

    Connects to PC-Service directly via xrobotoolkit_sdk.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 60061):
        """
        Initialize client

        Args:
            host: PC-Service address (ignored by Pybind version, uses local connection)
            port: PC-Service port (ignored by Pybind version)
        """
        self._connected = False
        # Pybind SDK uses local shared memory, does not need host/port

    def init(self) -> bool:
        """
        Initialize and connect to PC-Service

        Returns:
            True if connection successful
        """
        if not _USE_PYBIND:
            logger.error("xrobotoolkit_sdk not installed")
            return False

        try:
            xrt.init()
            self._connected = True
            logger.info("Connected to PC-Service (Pybind SDK)")
            return True
        except Exception as e:
            logger.error("Initialization failed: %s", e)
            logger.error("Please ensure PC-Service is running: /opt/apps/roboticsservice/runService.sh")
            return False

    def close(self):
        """Close connection"""
        self._connected = False
        logger.info("Connection closed")

    def is_connected(self) -> bool:
        """Check if connected"""
        return self._connected

    # ==================== Pose Retrieval API ====================

    def get_headset_pose(self) -> Optional[List[float]]:
        """Get HMD pose [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_headset_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def get_left_controller_pose(self) -> Optional[List[float]]:
        """Get left controller pose [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_left_controller_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def get_right_controller_pose(self) -> Optional[List[float]]:
        """Get right controller pose [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_right_controller_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def num_motion_data_available(self) -> int:
        """Get number of available Motion Trackers"""
        if not self._connected or not _USE_PYBIND:
            return 0
        try:
            return xrt.num_motion_data_available()
        except Exception:
            return 0

    def get_motion_tracker_pose(self) -> Optional[List[List[float]]]:
        """Get all Motion Tracker poses [[x,y,z,qx,qy,qz,qw], ...]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            poses = xrt.get_motion_tracker_pose()
            return [list(p) for p in poses] if poses else None
        except Exception:
            return None

    def get_motion_tracker_velocity(self) -> Optional[List[List[float]]]:
        """Get all Motion Tracker velocities [[vx,vy,vz,wx,wy,wz], ...]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            velocities = xrt.get_motion_tracker_velocity()
            return [list(v) for v in velocities] if velocities else None
        except Exception:
            return None

    def get_motion_tracker_acceleration(self) -> Optional[List[List[float]]]:
        """Get all Motion Tracker accelerations"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            accel = xrt.get_motion_tracker_acceleration()
            return [list(a) for a in accel] if accel else None
        except Exception:
            return None

    def get_motion_tracker_serial_numbers(self) -> List[str]:
        """Get Motion Tracker serial numbers"""
        if not self._connected or not _USE_PYBIND:
            return []
        try:
            return list(xrt.get_motion_tracker_serial_numbers())
        except Exception:
            return []

    # ==================== Controller Input API ====================

    def get_left_trigger(self) -> float:
        """Get left trigger value [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_left_trigger()
        except Exception:
            return 0.0

    def get_right_trigger(self) -> float:
        """Get right trigger value [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_right_trigger()
        except Exception:
            return 0.0

    def get_left_grip(self) -> float:
        """Get left grip value [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_left_grip()
        except Exception:
            return 0.0

    def get_right_grip(self) -> float:
        """Get right grip value [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_right_grip()
        except Exception:
            return 0.0

    def get_left_axis(self) -> List[float]:
        """Get left joystick [x, y]"""
        if not self._connected or not _USE_PYBIND:
            return [0.0, 0.0]
        try:
            return list(xrt.get_left_axis())
        except Exception:
            return [0.0, 0.0]

    def get_right_axis(self) -> List[float]:
        """Get right joystick [x, y]"""
        if not self._connected or not _USE_PYBIND:
            return [0.0, 0.0]
        try:
            return list(xrt.get_right_axis())
        except Exception:
            return [0.0, 0.0]

    def get_time_stamp_ns(self) -> int:
        """Get timestamp (nanoseconds)"""
        if not self._connected or not _USE_PYBIND:
            return 0
        try:
            return xrt.get_motion_timestamp_ns()
        except Exception:
            return 0


# ==================== Module-level API (compatible with xrobotoolkit_sdk) ====================

_client: Optional[XRoboToolkitClient] = None


def init(host: str = "127.0.0.1", port: int = 60061) -> bool:
    """Initialize SDK"""
    global _client
    if _client is not None:
        return True
    _client = XRoboToolkitClient(host, port)
    return _client.init()


def close():
    """Close SDK"""
    global _client
    if _client is not None:
        _client.close()
        _client = None


def get_headset_pose() -> Optional[List[float]]:
    """Get HMD pose"""
    return _client.get_headset_pose() if _client else None


def get_left_controller_pose() -> Optional[List[float]]:
    """Get left controller pose"""
    return _client.get_left_controller_pose() if _client else None


def get_right_controller_pose() -> Optional[List[float]]:
    """Get right controller pose"""
    return _client.get_right_controller_pose() if _client else None


def num_motion_data_available() -> int:
    """Get number of available Motion Trackers"""
    return _client.num_motion_data_available() if _client else 0


def get_motion_tracker_pose() -> Optional[List[List[float]]]:
    """Get all Motion Tracker poses"""
    return _client.get_motion_tracker_pose() if _client else None


def get_motion_tracker_velocity() -> Optional[List[List[float]]]:
    """Get all Motion Tracker velocities"""
    return _client.get_motion_tracker_velocity() if _client else None


def get_motion_tracker_serial_numbers() -> List[str]:
    """Get Motion Tracker serial numbers"""
    return _client.get_motion_tracker_serial_numbers() if _client else []


def get_left_trigger() -> float:
    """Get left trigger value"""
    return _client.get_left_trigger() if _client else 0.0


def get_right_trigger() -> float:
    """Get right trigger value"""
    return _client.get_right_trigger() if _client else 0.0


def get_left_grip() -> float:
    """Get left grip value"""
    return _client.get_left_grip() if _client else 0.0


def get_right_grip() -> float:
    """Get right grip value"""
    return _client.get_right_grip() if _client else 0.0


def get_left_axis() -> List[float]:
    """Get left joystick"""
    return _client.get_left_axis() if _client else [0.0, 0.0]


def get_right_axis() -> List[float]:
    """Get right joystick"""
    return _client.get_right_axis() if _client else [0.0, 0.0]


def get_time_stamp_ns() -> int:
    """Get timestamp"""
    return _client.get_time_stamp_ns() if _client else 0
