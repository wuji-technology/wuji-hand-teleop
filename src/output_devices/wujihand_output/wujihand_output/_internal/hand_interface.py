#!/usr/bin/env python3
"""
Wuji Hand hardware interface wrapper.

Talks to the wujihandros2 driver (C++ wujihandcpp SDK) over ROS2 topics.
"""
import threading
import time
from typing import Optional
import numpy as np

# ROS2 deps
try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = None


def get_sensor_data_qos() -> 'QoSProfile':
    """Sensor-data QoS profile (matches the wujihandros2 driver)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )


class WujiHand:
    """
    Wuji-hand ROS2 interface (driven by wujihandros2).

    Talks to the wujihandros2 C++ driver over ROS2 topics:
    - publishes: /{hand_name}/joint_commands
    - subscribes: /{hand_name}/joint_states

    wujihandros2 wraps the C++ wujihandcpp SDK and supports 1000Hz control.
    """

    NUM_JOINTS = 20  # 5 fingers x 4 joints

    def __init__(
        self,
        hand_name: str,
        side: str,
        node: 'Node',
        logger=None
    ):
        """
        Initialize the ROS2 interface.

        Args:
            hand_name: wujihandros2 driver namespace (e.g. "left_hand", "right_hand").
            side: hand side, "left" or "right" (for logging).
            node: ROS2 node instance.
            logger: external logger (optional).
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 dependencies are not available; WujiHand cannot be used")

        self.hand_name = hand_name
        self.side = side
        self.node = node
        self.logger = logger or node.get_logger()

        # State cache; _last_msg_time drives is_connected freshness check.
        self._latest_positions: Optional[np.ndarray] = None
        self._last_msg_time: float = 0.0
        self._lock = threading.Lock()

        # QoS
        qos = get_sensor_data_qos()

        # Publisher: send joint commands to the wujihandros2 driver.
        self._cmd_pub = node.create_publisher(
            JointState,
            f"/{hand_name}/joint_commands",
            qos
        )

        # Subscriber: receive joint states from the wujihandros2 driver.
        self._state_sub = node.create_subscription(
            JointState,
            f"/{hand_name}/joint_states",
            self._state_callback,
            qos
        )

        self.logger.info(
            f"WujiHand initialized: {side} hand -> /{hand_name}"
        )

    def _state_callback(self, msg: 'JointState') -> None:
        """Handle a state message from the wujihandros2 driver."""
        if not (msg.position and len(msg.position) == self.NUM_JOINTS):
            return
        with self._lock:
            first_msg = self._last_msg_time == 0.0
            self._latest_positions = np.array(msg.position, dtype=np.float32)
            self._last_msg_time = time.monotonic()
        if first_msg:
            self.logger.info(f"{self.side.title()} hand connected (via wujihandros2)")

    def connect(self) -> bool:
        """
        Connection check. Under ROS2 always returns True — actual link
        management is the driver node's responsibility.

        Returns:
            bool: True.
        """
        self.logger.info(f"{self.side.title()} hand awaiting wujihandros2 driver connection...")
        return True

    CONNECTION_FRESHNESS_SEC = 1.0

    def is_connected(self) -> bool:
        """True if a state message arrived within CONNECTION_FRESHNESS_SEC."""
        with self._lock:
            last = self._last_msg_time
        return last > 0.0 and (time.monotonic() - last) < self.CONNECTION_FRESHNESS_SEC

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """
        Set joint angles (publish to the wujihandros2 driver).

        Args:
            positions: joint-angle array of shape (20,) or (5, 4).

        Returns:
            bool: publish success.
        """
        try:
            positions = np.asarray(positions, dtype=np.float32)
            if positions.shape == (5, 4):
                positions = positions.flatten()
            elif positions.shape != (self.NUM_JOINTS,):
                self.logger.error(
                    f"Invalid joint-position shape: {positions.shape}, expected (20,) or (5, 4)"
                )
                return False

            # Build a JointState message (position-only; do not set name).
            # wujihandros2 supports position-only mode and parses by index.
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.position = positions.tolist()

            self._cmd_pub.publish(msg)
            return True

        except Exception as e:
            self.logger.error(f"Failed to publish joint command: {e}")
            return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        Read the current joint angles from the subscriber cache.

        Returns:
            np.ndarray: (20,) joint angles, or None when not connected.
        """
        with self._lock:
            if self._latest_positions is not None:
                return self._latest_positions.copy()
            return None

    def disable(self) -> None:
        """
        Disable the hand.

        Note: wujihandros2 disables automatically when the driver node shuts down.
        """
        self.logger.info(
            f"{self.side.title()} hand: enable/disable is managed by the wujihandros2 driver")

    def release(self) -> None:
        """Release resources."""
        self.disable()
        with self._lock:
            self._last_msg_time = 0.0
            self._latest_positions = None
        self.logger.info(f"{self.side.title()} hand ROS2 interface released")
