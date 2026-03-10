"""
LiveDataSource - Real-time PICO SDK data source

Wraps the xrobotoolkit_client to provide real-time VR tracking data
through the DataSource interface.
"""

import logging
import time
from typing import List, Optional
import numpy as np

from .base import DataSource, TrackerData, HeadsetData

logger = logging.getLogger(__name__)


class LiveDataSource(DataSource):
    """
    Live Data Source - Real-time PICO SDK

    Connects to PICO PC-Service (XRoboToolkit) to receive live VR tracking data.
    Provides real-time headset and motion tracker poses.

    Args:
        host: PC-Service host address (default: 127.0.0.1)
        port: PC-Service port (default: 60061)

    Example:
        source = LiveDataSource("127.0.0.1", 60061)
        if source.initialize():
            headset = source.get_headset_pose()
            trackers = source.get_tracker_data()

    Note:
        Requires xrobotoolkit_client module and PC-Service running.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 60061):
        self.host = host
        self.port = port
        self._initialized = False
        self._available = False

        # Import xrobotoolkit_client lazily
        try:
            from pico_input import xrobotoolkit_client
            self.sdk = xrobotoolkit_client
        except ImportError as e:
            logger.warning("Failed to import xrobotoolkit_client: %s", e)
            self.sdk = None

    def initialize(self) -> bool:
        """
        Connect to PC-Service

        Returns:
            True if connection successful, False otherwise
        """
        if self.sdk is None:
            logger.error("xrobotoolkit_client not available")
            return False

        try:
            result = self.sdk.init(self.host, self.port)
            if result:
                self._initialized = True
                self._available = True
                return True
            else:
                logger.error("Failed to connect to PC-Service at %s:%d", self.host, self.port)
                return False
        except Exception as e:
            logger.error("SDK initialization error: %s", e)
            return False

    def get_headset_pose(self) -> Optional[HeadsetData]:
        """
        Get current headset (HMD) pose from SDK

        Returns:
            HeadsetData if available and valid, None otherwise
        """
        if not self._initialized or not self.sdk:
            return None

        try:
            pose = self.sdk.get_headset_pose()
            if pose is None or len(pose) < 7:
                return None

            # 直接使用 SDK 返回的列表切片，避免 np.array 开销
            orientation = pose[3:7]
            quat = np.array(orientation, dtype=np.float64)
            if not self.validate_quaternion(quat):
                return None

            return HeadsetData(
                position=np.array(pose[:3], dtype=np.float64),
                orientation=quat,
                is_valid=True,
                timestamp=time.time()
            )

        except Exception as e:
            logger.warning("Error getting headset pose: %s", e)
            return None

    def get_tracker_data(self) -> List[TrackerData]:
        """
        Get all motion tracker data from SDK

        Returns:
            List of TrackerData objects, empty list if unavailable
        """
        if not self._initialized or not self.sdk:
            return []

        try:
            num_trackers = self.sdk.num_motion_data_available()
            if num_trackers <= 0:
                return []

            poses = self.sdk.get_motion_tracker_pose()
            if poses is None:
                return []

            serial_numbers = self.sdk.get_motion_tracker_serial_numbers()
            current_time = time.time()

            result = []
            for i in range(min(num_trackers, len(poses))):
                pose = poses[i]
                if len(pose) < 7:
                    continue

                # Get serial number
                if serial_numbers and i < len(serial_numbers):
                    sn = serial_numbers[i]
                else:
                    sn = f"tracker_{i}"

                # Validate quaternion (使用列表切片，仅在有效时创建 np.array)
                quat_raw = pose[3:7]
                norm_sq = quat_raw[0]**2 + quat_raw[1]**2 + quat_raw[2]**2 + quat_raw[3]**2
                is_valid = 0.9025 < norm_sq < 1.1025  # (1-0.05)² < norm² < (1+0.05)²

                result.append(TrackerData(
                    position=np.array(pose[:3], dtype=np.float64),
                    orientation=np.array(quat_raw, dtype=np.float64),
                    serial_number=sn,
                    is_valid=is_valid,
                    timestamp=current_time
                ))

            return result

        except Exception as e:
            logger.warning("Error getting tracker data: %s", e)
            return []

    def is_available(self) -> bool:
        """
        Check if SDK is available and ready

        Returns:
            True if initialized and ready
        """
        return self._available

    def close(self):
        """
        Close SDK connection and clean up
        """
        if self._initialized and self.sdk:
            try:
                self.sdk.close()
            except Exception as e:
                logger.warning("Error closing SDK: %s", e)

        self._available = False
        self._initialized = False

    def __repr__(self):
        """String representation for debugging"""
        status = "connected" if self._initialized else "disconnected"
        return f"LiveDataSource(host='{self.host}', port={self.port}, status={status})"
