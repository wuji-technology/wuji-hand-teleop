"""ROS2 node that streams Apple Vision Pro hand data and publishes ROS topics."""

from __future__ import annotations

import argparse
import copy
import sys
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

# 确保可以导入同一包内的模块（当直接运行脚本时）
_script_dir = Path(__file__).resolve().parent
_package_root = _script_dir.parent  # avp_input 包的父目录
if str(_package_root) not in sys.path:
    sys.path.insert(0, str(_package_root))

import numpy as np

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:  # pragma: no cover - configuration dependency
    raise ImportError("PyYAML is required to load AVP input configuration files.") from exc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from std_msgs.msg import Float32MultiArray
    from rclpy.utilities import remove_ros_args
except ImportError as exc:  # pragma: no cover - ROS dependency
    raise ImportError("This module requires ROS 2 Python packages (rclpy, std_msgs).") from exc

from avp_stream import VisionProStreamer as _VisionProStreamer
_USING_PSEUDO_STREAMER = False


# VisionPro 25 joints to MediaPipe 21 landmarks mapping
VP_TO_MEDIAPIPE = (
    0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24
)


def convert_vp_to_mediapipe(fingers_mat: np.ndarray) -> np.ndarray:
    """Convert VisionPro (25, 4, 4) to MediaPipe (21, 3) format.

    Extracts position (translation) component from each 4x4 transformation matrix
    and maps 25 VisionPro joints to 21 MediaPipe landmarks.
    """
    mediapipe_pose = np.zeros((21, 3), dtype=np.float32)
    for mp_idx, vp_idx in enumerate(VP_TO_MEDIAPIPE):
        mediapipe_pose[mp_idx] = fingers_mat[vp_idx][:3, 3]
    return mediapipe_pose



@dataclass
class AvpInputConfig:
    """Configuration for the AVP input streaming node."""

    config_path: Optional[str] = None
    input_source: str = "visionpro"  # Placeholder for future sources (e.g., manus)

    # VisionPro-specific settings
    avp_ip: str = "192.168.50.127"

    # Publishing behaviour
    publish_rate_hz: float = 120.0
    publish_hand_topic: str = "/hand_input"
    publish_wrist_topics: bool = True
    right_wrist_topic: str = "/right_wrist"
    left_wrist_topic: str = "/left_wrist"
    publish_head_topic: bool = True
    head_topic: str = "/head_pose"
    include_right_hand: bool = True
    include_left_hand: bool = True

    @classmethod
    def from_file(cls, path: str | Path) -> "AvpInputConfig":
        """Load configuration from a YAML file."""
        cfg_path = Path(path).expanduser().resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with cfg_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        if not isinstance(raw, dict):
            raise ValueError("AvpInputConfig expects a mapping at the root of the YAML file.")

        valid_fields = {field.name for field in fields(cls)}
        data: Dict[str, Any] = {"config_path": str(cfg_path)}
        ignored_keys: list[str] = []

        for key, value in raw.items():
            if key in valid_fields:
                data[key] = value
            else:
                ignored_keys.append(key)

        if ignored_keys:
            print(f"[AvpInputConfig] Ignoring unknown keys: {', '.join(sorted(ignored_keys))}")

        return cls(**data)


class AvpInputNode(Node):
    """ROS2 node that reads Apple Vision Pro hand data and publishes ROS topics."""

    def __init__(self, config: AvpInputConfig):
        super().__init__("avp_input")
        self.config = config

        if self.config.input_source.lower() != "visionpro":
            raise ValueError("Currently only 'visionpro' input_source is supported.")

        self.get_logger().info(
            f"[AVP Input] Initializing VisionPro streamer (IP={self.config.avp_ip})."
        )

        self.streamer = _VisionProStreamer(self.config.avp_ip, False)

        # Configure QoS for real-time performance
        # BEST_EFFORT: prioritize low latency over reliability
        # KEEP_LAST with depth=1: always publish newest data immediately
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.hand_publisher = self.create_publisher(Float32MultiArray, self.config.publish_hand_topic, qos_profile)
        self.right_wrist_publisher = None
        self.left_wrist_publisher = None
        self.head_publisher = None

        if self.config.publish_wrist_topics:
            self.right_wrist_publisher = self.create_publisher(Float32MultiArray, self.config.right_wrist_topic, qos_profile)
            self.left_wrist_publisher = self.create_publisher(Float32MultiArray, self.config.left_wrist_topic, qos_profile)
        if self.config.publish_head_topic:
            self.head_publisher = self.create_publisher(Float32MultiArray, self.config.head_topic, qos_profile)

        self.timer = self.create_timer(1.0 / max(self.config.publish_rate_hz, 1.0), self._publish_latest_frame)
        self.get_logger().info(
            f"[AVP Input] Publishing hand data on '{self.config.publish_hand_topic}' "
            f"at {self.config.publish_rate_hz:.1f} Hz."
        )

    # ------------------------------------------------------------------ #
    # Data acquisition & publishing
    # ------------------------------------------------------------------ #
    def _fetch_latest(self) -> Optional[Dict[str, np.ndarray]]:
        """Retrieve the latest frame from the VisionPro streamer."""
        latest = getattr(self.streamer, "latest", None)
        if latest is None:
            return None
        return latest

    def _publish_latest_frame(self) -> None:
        frame = self._fetch_latest()
        if frame is None:
            self.get_logger().warning("No data received from VisionPro streamer yet.")
            return

        try:
            # Extract raw data directly from VisionPro (no transformation)
            right_fingers = self._extract_finger_array(frame.get("right_fingers"))
            left_fingers = self._extract_finger_array(frame.get("left_fingers"))
            right_wrist = self._extract_wrist_array(frame.get("right_wrist"))
            left_wrist = self._extract_wrist_array(frame.get("left_wrist"))
            head_pose = self._extract_wrist_array(frame.get("head"))
        except ValueError as exc:
            self.get_logger().error(f"Failed to parse VisionPro frame: {exc}")
            return

        # Convert to MediaPipe (21, 3) format and publish
        payloads = []
        if self.config.include_right_hand and right_fingers is not None:
            right_mediapipe = convert_vp_to_mediapipe(right_fingers)
            payloads.append(right_mediapipe.flatten())
        if self.config.include_left_hand and left_fingers is not None:
            left_mediapipe = convert_vp_to_mediapipe(left_fingers)
            payloads.append(left_mediapipe.flatten())

        if payloads:
            hand_msg = Float32MultiArray()
            hand_msg.data = np.concatenate(payloads).astype(np.float32).tolist()
            self.hand_publisher.publish(hand_msg)

        # Publish wrist data if requested
        if self.config.publish_wrist_topics:
            if self.right_wrist_publisher and right_wrist is not None:
                msg = Float32MultiArray()
                msg.data = right_wrist.flatten().astype(np.float32).tolist()
                self.right_wrist_publisher.publish(msg)

            if self.left_wrist_publisher and left_wrist is not None:
                msg = Float32MultiArray()
                msg.data = left_wrist.flatten().astype(np.float32).tolist()
                self.left_wrist_publisher.publish(msg)
        if self.config.publish_head_topic and self.head_publisher and head_pose is not None:
            msg = Float32MultiArray()
            msg.data = head_pose.flatten().astype(np.float32).tolist()
            self.head_publisher.publish(msg)

    @staticmethod
    def _extract_finger_array(data: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Validate and reshape finger matrices to (25, 4, 4)."""
        if data is None:
            return None
        arr = np.asarray(data, dtype=np.float32)
        arr = arr.reshape(-1, 4, 4)
        if arr.shape[0] == 25:
            return arr
        if arr.shape[0] == 26:
            return arr[1:, ...]  # drop wrist entry if present
        raise ValueError(f"Unexpected finger array shape: {arr.shape}")

    @staticmethod
    def _extract_wrist_array(data: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Validate and reshape wrist matrices to (4, 4)."""
        if data is None:
            return None
        arr = np.asarray(data, dtype=np.float32)
        arr = np.squeeze(arr)
        if arr.shape == (4, 4):
            return arr
        raise ValueError(f"Unexpected wrist array shape: {arr.shape}")

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #
    def shutdown(self) -> None:
        if hasattr(self.streamer, "close"):
            try:
                self.streamer.close()
            except Exception as exc:  # pragma: no cover - hardware side effect
                self.get_logger().error(f"Error while closing VisionPro streamer: {exc}")


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish VisionPro hand data to ROS2 topics.")
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to an AVP input YAML configuration file.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "avp_input"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)
    if args.config:
        config_path = Path(args.config)
    else:
        share_dir = Path(get_package_share_directory("avp_input"))
        config_path = share_dir / "config" / "avp_input.yaml"

    config = AvpInputConfig.from_file(str(config_path))

    rclpy.init(args=raw_argv)
    node = AvpInputNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
