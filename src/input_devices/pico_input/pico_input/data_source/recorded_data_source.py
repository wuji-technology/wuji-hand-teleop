"""
RecordedDataSource - Playback VR tracking data from recorded files

Reads tracking data from JSON lines format and provides playback
with configurable speed and looping.
"""

import json
import logging
import time
from typing import List, Optional
import numpy as np

from .base import DataSource, TrackerData, HeadsetData

logger = logging.getLogger(__name__)


class RecordedDataSource(DataSource):
    """
    Recorded Data Source - Playback from File

    Reads VR tracking data from recorded JSON lines file and provides
    time-synchronized playback with configurable speed.

    File Format:
        Each line is a JSON object with structure:
        {
            "predictTime": <timestamp_ns>,
            "Head": {"pose": "x,y,z,qx,qy,qz,qw", "status": 3},
            "Motion": {
                "joints": [
                    {"p": "x,y,z,qx,qy,qz,qw", "sn": "PC2310MLKC190058G", ...},
                    ...
                ]
            }
        }

    Args:
        file_path: Path to recorded data file
        playback_speed: Playback speed multiplier (1.0 = real-time)
        loop: Whether to loop playback when reaching end of file

    Example:
        source = RecordedDataSource("trackingData.txt", playback_speed=2.0)
        if source.initialize():
            headset = source.get_headset_pose()
            trackers = source.get_tracker_data()
    """

    def __init__(
        self,
        file_path: str,
        playback_speed: float = 1.0,
        loop: bool = True
    ):
        self.file_path = file_path
        self.playback_speed = playback_speed
        self.loop = loop

        # Internal state
        self.data_frames = []
        self.current_index = 0
        self.start_time = None
        self.first_timestamp = None
        self._initialized = False
        self._available = False

    def initialize(self) -> bool:
        """
        Load recorded data from file

        Returns:
            True if file loaded successfully, False otherwise
        """
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        frame = json.loads(line)
                        # Only include frames with tracking data (has predictTime and Head/Motion)
                        if 'predictTime' in frame and ('Head' in frame or 'Motion' in frame):
                            self.data_frames.append(frame)
                    except json.JSONDecodeError as e:
                        logger.warning("Skipping invalid JSON line: %s", e)
                        continue

            if len(self.data_frames) > 0:
                self.first_timestamp = self.data_frames[0].get('predictTime', 0)
                self._initialized = True
                self._available = True
                return True
            else:
                logger.error("No valid frames found in %s", self.file_path)
                return False

        except FileNotFoundError:
            logger.error("File not found: %s", self.file_path)
            return False
        except Exception as e:
            logger.error("Error loading data: %s", e)
            return False

    def get_headset_pose(self) -> Optional[HeadsetData]:
        """
        Get headset pose from current frame

        Returns:
            HeadsetData if available and valid, None otherwise
        """
        frame = self._get_current_frame()
        if frame is None:
            return None

        head = frame.get('Head', {})
        if 'pose' not in head:
            return None

        pose = self._parse_pose_string(head['pose'])
        if pose is None or len(pose) < 7:
            return None

        return HeadsetData(
            position=np.array(pose[:3], dtype=np.float64),
            orientation=np.array(pose[3:7], dtype=np.float64),
            is_valid=(head.get('status', 0) == 3),
            timestamp=frame.get('predictTime', 0)
        )

    def get_tracker_data(self) -> List[TrackerData]:
        """
        Get all motion tracker data from current frame

        Returns:
            List of TrackerData objects, empty list if unavailable
        """
        frame = self._get_current_frame()
        if frame is None:
            return []

        motion = frame.get('Motion', {})
        joints = motion.get('joints', [])

        result = []
        for joint in joints:
            pose_str = joint.get('p', '')
            pose = self._parse_pose_string(pose_str)
            if pose is None or len(pose) < 7:
                continue

            orientation = np.array(pose[3:7], dtype=np.float64)

            result.append(TrackerData(
                position=np.array(pose[:3], dtype=np.float64),
                orientation=orientation,
                serial_number=joint.get('sn', ''),
                is_valid=self.validate_quaternion(orientation),
                timestamp=frame.get('predictTime', 0)
            ))

        return result

    def is_available(self) -> bool:
        """
        Check if data source is available

        Returns:
            True if initialized and ready to provide data
        """
        return self._available

    def close(self):
        """
        Close the data source and release resources
        """
        self._available = False
        self._initialized = False
        self.data_frames = []
        self.current_index = 0
        self.start_time = None

    # Private helper methods

    def _get_current_frame(self):
        """
        Get current frame based on frame index and playback speed

        Uses index-based playback (assuming original ~90fps capture rate)
        instead of predictTime-based playback, because predictTime units
        vary between PICO SDK versions and are not reliably in milliseconds.

        Returns:
            Current frame dict, or None if unavailable
        """
        if not self._initialized or len(self.data_frames) == 0:
            return None

        # First call - start playback
        if self.start_time is None:
            self.start_time = time.time()
            self.current_index = 0
            return self.data_frames[0]

        # Calculate target frame index based on elapsed time and playback speed
        # Original data assumed to be captured at ~90fps
        elapsed_real = time.time() - self.start_time
        original_fps = 90.0
        target_index = int(elapsed_real * self.playback_speed * original_fps)

        if target_index >= len(self.data_frames):
            if self.loop:
                # Loop: reset timer when wrapping
                target_index = target_index % len(self.data_frames)
                if target_index == 0:
                    self.start_time = time.time()
            else:
                # Stay at last frame
                target_index = len(self.data_frames) - 1

        self.current_index = target_index
        return self.data_frames[target_index]

    def _parse_pose_string(self, pose_str: str) -> Optional[np.ndarray]:
        """
        Parse comma-separated pose string

        Args:
            pose_str: String in format "x,y,z,qx,qy,qz,qw"

        Returns:
            NumPy array [x, y, z, qx, qy, qz, qw], or None if invalid
        """
        if not pose_str:
            return None

        try:
            values = [float(x.strip()) for x in pose_str.split(',')]
            if len(values) == 7:
                return np.array(values, dtype=np.float64)
            else:
                return None
        except (ValueError, AttributeError):
            return None

    def __repr__(self):
        """String representation for debugging"""
        status = "initialized" if self._initialized else "not initialized"
        frames = len(self.data_frames)
        return (f"RecordedDataSource(file='{self.file_path}', "
                f"frames={frames}, speed={self.playback_speed}x, "
                f"status={status})")
