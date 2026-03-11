"""
ROS2 node for teleop monitoring.
"""

from __future__ import annotations

import subprocess
import time
from typing import Dict, List, Optional, Set

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from manus_ros2_msgs.msg import ManusGlove

from .scanner import WujiHandScanner, ManusGloveScanner, TianjiArmScanner, ViveTrackerScanner, StereoHeadScanner


# Key nodes to monitor
MONITORED_NODES = [
    "manus_data_publisher",
    "manus_input",
    "wujihand_retargeting",
    "tianji_output_node",
    "openvr_input",
]

# Camera nodes to monitor (RealSense node names)
CAMERA_NODES = [
    "cam_head",
    "cam_left_wrist",
    "cam_right_wrist",
]

# Camera topic names
# D405 only publishes image_rect_raw (no dedicated RGB module); D435 publishes both; use rect_raw for compatibility
CAMERA_TOPICS = {
    "head": "/cam_head/color/image_raw",
    "left_wrist": "/cam_left_wrist/color/image_rect_raw",
    "right_wrist": "/cam_right_wrist/color/image_rect_raw",
}

# Stereo head camera topic names (CompressedImage, BEST_EFFORT QoS)
STEREO_HEAD_TOPICS = {
    "left": "/stereo/left/compressed",
    "right": "/stereo/right/compressed",
}


class TeleopMonitorNode(Node):
    """ROS2 node that monitors Manus glove and Wuji hand connection status."""

    CONNECTION_TIMEOUT = 0.5

    def __init__(self):
        super().__init__("teleop_monitor")

        # Manus glove state (via USB scan)
        self.manus_glove_serials: Set[str] = set()

        # Manus glove topic state
        self.glove_0_connected = False
        self.glove_0_last_time: Optional[float] = None
        self.glove_0_msg_count = 0
        self.glove_1_connected = False
        self.glove_1_last_time: Optional[float] = None
        self.glove_1_msg_count = 0

        # /hand_input state
        self.hand_input_connected = False
        self.hand_input_last_time: Optional[float] = None
        self.hand_input_msg_count = 0
        self.hand_input_data_size = 0  # 63=single hand, 126=dual hands

        # Wuji hand state
        self.connected_hand_serials: Set[str] = set()

        # Tianji arm state
        self.tianji_arm_reachable = False

        # Vive Tracker state
        self.vive_tracker_serials: Set[str] = set()
        self.base_stations: Dict[str, dict] = {}  # {serial: {'mode': 'A/B', 'connected': bool}}

        # Camera state
        self.camera_status: Dict[str, dict] = {
            "head": {"connected": False, "last_time": None, "msg_count": 0},
            "left_wrist": {"connected": False, "last_time": None, "msg_count": 0},
            "right_wrist": {"connected": False, "last_time": None, "msg_count": 0},
        }

        # Stereo head camera state
        self.stereo_head_status: Dict[str, dict] = {
            "left": {"connected": False, "last_time": None, "msg_count": 0},
            "right": {"connected": False, "last_time": None, "msg_count": 0},
        }
        self.stereo_head_camera_available = False  # Whether stereo camera hardware is available
        self.stereo_head_loopback_available = False  # Whether v4l2loopback is available

        # Node status
        self.running_nodes: Set[str] = set()

        # QoS for real-time data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS for camera (best effort for high bandwidth)
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to manus glove topics
        self.sub_glove_0 = self.create_subscription(
            ManusGlove, "/manus_glove_0", self._glove_0_callback, qos_profile)
        self.sub_glove_1 = self.create_subscription(
            ManusGlove, "/manus_glove_1", self._glove_1_callback, qos_profile)

        # Subscribe to /hand_input
        self.sub_hand_input = self.create_subscription(
            Float32MultiArray, "/hand_input", self._hand_input_callback, qos_profile)

        # Subscribe to camera topics
        self.sub_cam_head = self.create_subscription(
            Image, CAMERA_TOPICS["head"], self._cam_head_callback, camera_qos)
        self.sub_cam_left = self.create_subscription(
            Image, CAMERA_TOPICS["left_wrist"], self._cam_left_callback, camera_qos)
        self.sub_cam_right = self.create_subscription(
            Image, CAMERA_TOPICS["right_wrist"], self._cam_right_callback, camera_qos)

        # Subscribe to stereo head camera topics (CompressedImage, BEST_EFFORT QoS)
        self.sub_stereo_left = self.create_subscription(
            CompressedImage, STEREO_HEAD_TOPICS["left"], self._stereo_left_callback, camera_qos)
        self.sub_stereo_right = self.create_subscription(
            CompressedImage, STEREO_HEAD_TOPICS["right"], self._stereo_right_callback, camera_qos)

        self.get_logger().info("Teleop Monitor started")

    def _glove_0_callback(self, msg: ManusGlove) -> None:
        self.glove_0_last_time = time.time()
        self.glove_0_msg_count += 1

    def _glove_1_callback(self, msg: ManusGlove) -> None:
        self.glove_1_last_time = time.time()
        self.glove_1_msg_count += 1

    def _hand_input_callback(self, msg: Float32MultiArray) -> None:
        self.hand_input_last_time = time.time()
        self.hand_input_msg_count += 1
        self.hand_input_data_size = len(msg.data)

    def _cam_head_callback(self, msg: Image) -> None:
        self.camera_status["head"]["last_time"] = time.time()
        self.camera_status["head"]["msg_count"] += 1

    def _cam_left_callback(self, msg: Image) -> None:
        self.camera_status["left_wrist"]["last_time"] = time.time()
        self.camera_status["left_wrist"]["msg_count"] += 1

    def _cam_right_callback(self, msg: Image) -> None:
        self.camera_status["right_wrist"]["last_time"] = time.time()
        self.camera_status["right_wrist"]["msg_count"] += 1

    def _stereo_left_callback(self, msg: CompressedImage) -> None:
        self.stereo_head_status["left"]["last_time"] = time.time()
        self.stereo_head_status["left"]["msg_count"] += 1

    def _stereo_right_callback(self, msg: CompressedImage) -> None:
        self.stereo_head_status["right"]["last_time"] = time.time()
        self.stereo_head_status["right"]["msg_count"] += 1

    def update_glove_status(self) -> None:
        """Update Manus glove connection status via USB scan."""
        self.manus_glove_serials = ManusGloveScanner.scan_devices()

    def update_glove_topic_status(self) -> None:
        """Update Manus glove topic connection status."""
        now = time.time()
        if self.glove_0_last_time is not None:
            self.glove_0_connected = (now - self.glove_0_last_time) < self.CONNECTION_TIMEOUT
        else:
            self.glove_0_connected = False

        if self.glove_1_last_time is not None:
            self.glove_1_connected = (now - self.glove_1_last_time) < self.CONNECTION_TIMEOUT
        else:
            self.glove_1_connected = False

    def update_hand_input_status(self) -> None:
        """Update /hand_input topic connection status."""
        now = time.time()
        if self.hand_input_last_time is not None:
            self.hand_input_connected = (now - self.hand_input_last_time) < self.CONNECTION_TIMEOUT
        else:
            self.hand_input_connected = False

    def update_hand_status(self) -> None:
        """Update Wuji hand connection status via USB scan."""
        self.connected_hand_serials = WujiHandScanner.scan_devices()

    def update_tianji_status(self) -> None:
        """Update Tianji arm connection status via ping."""
        self.tianji_arm_reachable = TianjiArmScanner.check_reachable()

    def get_tianji_ip(self) -> str:
        """Get Tianji arm IP address."""
        return TianjiArmScanner.get_robot_ip()

    def update_vive_status(self) -> None:
        """Update Vive Tracker and base station status."""
        self.vive_tracker_serials = ViveTrackerScanner.scan_trackers()
        self.base_stations = ViveTrackerScanner.scan_base_stations()

    def update_node_status(self) -> None:
        """Update running node status via ros2 node list."""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                node_list = result.stdout.strip().split('\n')
                # Extract node names (remove namespace prefix like "/")
                self.running_nodes = set()
                for node in node_list:
                    node_name = node.strip().lstrip('/')
                    if node_name:
                        self.running_nodes.add(node_name)
            else:
                self.running_nodes = set()
        except Exception:
            self.running_nodes = set()

    def is_node_running(self, node_name: str) -> bool:
        """Check if a specific node is running."""
        return node_name in self.running_nodes

    def get_monitored_nodes_status(self) -> Dict[str, bool]:
        """Get status of all monitored nodes."""
        return {node: self.is_node_running(node) for node in MONITORED_NODES}

    def get_connected_hands(self) -> Set[str]:
        """Return all connected hand serial numbers."""
        return self.connected_hand_serials

    def get_hand_input_info(self) -> str:
        """Get hand input data description."""
        if self.hand_input_data_size == 126:
            return "Dual"
        elif self.hand_input_data_size == 63:
            return "Single"
        elif self.hand_input_data_size > 0:
            return f"{self.hand_input_data_size} vals"
        return "--"

    def update_camera_status(self) -> None:
        """Update camera topic connection status."""
        now = time.time()
        for cam_name, status in self.camera_status.items():
            if status["last_time"] is not None:
                status["connected"] = (now - status["last_time"]) < self.CONNECTION_TIMEOUT
            else:
                status["connected"] = False

    def get_camera_status(self) -> Dict[str, dict]:
        """Get camera connection status."""
        return self.camera_status

    def get_camera_nodes_status(self) -> Dict[str, bool]:
        """Get status of camera nodes."""
        return {node: self.is_node_running(node) for node in CAMERA_NODES}

    def update_stereo_head_status(self) -> None:
        """Update stereo head camera topic connection status."""
        now = time.time()
        for cam_name, status in self.stereo_head_status.items():
            if status["last_time"] is not None:
                status["connected"] = (now - status["last_time"]) < self.CONNECTION_TIMEOUT
            else:
                status["connected"] = False

    def update_stereo_head_hardware_status(self) -> None:
        """Update stereo head camera hardware status (camera + loopback)."""
        camera_info = StereoHeadScanner.check_stereo_camera()
        self.stereo_head_camera_available = camera_info['available']

        loopback_info = StereoHeadScanner.check_loopback_device()
        self.stereo_head_loopback_available = loopback_info['available']

    def get_stereo_head_status(self) -> Dict[str, dict]:
        """Get stereo head camera connection status."""
        return self.stereo_head_status

    def get_stereo_head_hardware_info(self) -> Dict[str, any]:
        """Get stereo head camera hardware info."""
        return {
            "camera_available": self.stereo_head_camera_available,
            "loopback_available": self.stereo_head_loopback_available,
            "camera_info": StereoHeadScanner.check_stereo_camera(),
            "loopback_info": StereoHeadScanner.check_loopback_device(),
        }
