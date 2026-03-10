"""
Data Source Abstraction Layer (OpenXR Style)

Provides abstract interfaces for VR tracking data sources,
following OpenXR-style data structures and patterns.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class TrackerData:
    """
    Single Motion Tracker Data (OpenXR Style)

    Follows OpenXR XrSpacePose and XrPosef conventions:
    - position: 3D position in meters
    - orientation: Quaternion [qx, qy, qz, qw] (scipy format)
    - serial_number: Unique tracker identifier
    - is_valid: Tracking state validity
    - timestamp: Data timestamp in seconds
    """
    position: np.ndarray          # [x, y, z] in meters
    orientation: np.ndarray       # [qx, qy, qz, qw] quaternion
    serial_number: str            # Tracker serial number
    is_valid: bool                # Tracking validity
    timestamp: float              # Timestamp in seconds

    def __post_init__(self):
        """Validate data after initialization"""
        assert len(self.position) == 3, "position must be [x, y, z]"
        assert len(self.orientation) == 4, "orientation must be [qx, qy, qz, qw]"

    def to_pose_array(self) -> np.ndarray:
        """Convert to [x, y, z, qx, qy, qz, qw] array"""
        return np.concatenate([self.position, self.orientation])


@dataclass
class HeadsetData:
    """
    VR Headset (HMD) Data (OpenXR Style)

    Represents the head-mounted display tracking data.
    Similar to TrackerData but specifically for the HMD.
    """
    position: np.ndarray          # [x, y, z] in meters
    orientation: np.ndarray       # [qx, qy, qz, qw] quaternion
    is_valid: bool                # Tracking validity
    timestamp: float              # Timestamp in seconds

    def __post_init__(self):
        """Validate data after initialization"""
        assert len(self.position) == 3, "position must be [x, y, z]"
        assert len(self.orientation) == 4, "orientation must be [qx, qy, qz, qw]"

    def to_pose_array(self) -> np.ndarray:
        """Convert to [x, y, z, qx, qy, qz, qw] array"""
        return np.concatenate([self.position, self.orientation])


class DataSource(ABC):
    """
    Abstract Data Source Interface (OpenXR Style)

    Provides a unified interface for different VR tracking data sources:
    - LiveDataSource: Real-time PICO SDK data
    - RecordedDataSource: Playback from recorded files
    - MockDataSource: Simulated data for testing

    Design Pattern: Strategy Pattern for data source abstraction
    Follows OpenXR runtime abstraction principles.
    """

    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the data source

        Returns:
            bool: True if initialization successful, False otherwise

        Example:
            source = LiveDataSource("127.0.0.1", 60061)
            if source.initialize():
                print("Data source ready")
        """
        pass

    @abstractmethod
    def get_headset_pose(self) -> Optional[HeadsetData]:
        """
        Get current headset (HMD) pose

        Returns:
            HeadsetData if available and valid, None otherwise

        Note:
            Should return None if headset is not tracking or data unavailable.
            Check HeadsetData.is_valid for tracking validity.
        """
        pass

    @abstractmethod
    def get_tracker_data(self) -> List[TrackerData]:
        """
        Get all motion tracker data

        Returns:
            List of TrackerData objects, empty list if no trackers available

        Note:
            - Each tracker is identified by serial_number
            - Check TrackerData.is_valid for individual tracker validity
            - Order may not be consistent across calls
        """
        pass

    @abstractmethod
    def is_available(self) -> bool:
        """
        Check if data source is available and ready

        Returns:
            bool: True if source is ready to provide data

        Note:
            This is a lightweight check, unlike initialize() which may
            perform heavy setup operations.
        """
        pass

    @abstractmethod
    def close(self):
        """
        Close and clean up the data source

        Should release all resources (connections, file handles, etc.)
        After calling close(), the source should not be used again.
        """
        pass

    # Helper methods (shared utilities)

    @staticmethod
    def validate_quaternion(quat: np.ndarray, tolerance: float = 0.05) -> bool:
        """
        Validate quaternion data

        Args:
            quat: Quaternion [qx, qy, qz, qw]
            tolerance: Tolerance for norm deviation from 1.0

        Returns:
            bool: True if quaternion is valid (norm close to 1.0)

        Note:
            Uses tight tolerance (0.05) to reject corrupted data early.
            Valid SDK data typically has |norm - 1| < 0.001.
        """
        if len(quat) != 4:
            return False
        norm = np.linalg.norm(quat)
        return (1.0 - tolerance) < norm < (1.0 + tolerance)

    @staticmethod
    def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
        """
        Normalize a quaternion to unit length

        Args:
            quat: Quaternion [qx, qy, qz, qw]

        Returns:
            Normalized quaternion

        Raises:
            ValueError: If quaternion norm is too close to zero
        """
        norm = np.linalg.norm(quat)
        if norm < 1e-6:
            raise ValueError("Cannot normalize zero-length quaternion")
        return quat / norm
