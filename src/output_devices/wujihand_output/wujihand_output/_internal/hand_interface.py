#!/usr/bin/env python3
"""
Wuji Dexterous Hand Hardware Interface Wrapper

Communicates via wujihandros2 driver (C++ wujihandcpp SDK) over ROS2
"""
import threading
from typing import Optional
import numpy as np

# ROS2 dependencies
try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = None


def get_sensor_data_qos() -> 'QoSProfile':
    """Get sensor data QoS config (consistent with wujihandros2 driver)"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )


class WujiHandROS2:
    """
    Wuji Dexterous Hand ROS2 Interface (via wujihandros2 driver)

    Communicates with wujihandros2 C++ driver via ROS2 Topics:
    - Publishes: /{hand_name}/joint_commands
    - Subscribes: /{hand_name}/joint_states

    wujihandros2 uses C++ wujihandcpp SDK, supports 1000Hz control frequency
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
        Initialize ROS2 interface

        Args:
            hand_name: wujihandros2 driver namespace (e.g. "left_hand", "right_hand")
            side: Hand side "left" or "right" (for logging)
            node: ROS2 node instance
            logger: External logger (optional)
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 dependencies not installed, cannot use WujiHandROS2")

        self.hand_name = hand_name
        self.side = side
        self.node = node
        self.logger = logger or node.get_logger()

        # State cache
        self._latest_positions: Optional[np.ndarray] = None
        self._connected = False
        self._lock = threading.Lock()

        # QoS config
        qos = get_sensor_data_qos()

        # Create publisher: send joint commands to wujihandros2 driver
        self._cmd_pub = node.create_publisher(
            JointState,
            f"/{hand_name}/joint_commands",
            qos
        )

        # Create subscriber: receive joint states from wujihandros2 driver
        self._state_sub = node.create_subscription(
            JointState,
            f"/{hand_name}/joint_states",
            self._state_callback,
            qos
        )

        self.logger.info(
            f"WujiHandROS2 initialized: {side} hand -> /{hand_name}"
        )

    def _state_callback(self, msg: 'JointState') -> None:
        """Process state messages from wujihandros2 driver"""
        with self._lock:
            if msg.position and len(msg.position) == self.NUM_JOINTS:
                self._latest_positions = np.array(msg.position, dtype=np.float32)
                if not self._connected:
                    self._connected = True
                    self.logger.info(f"{self.side.title()} hand connected (via wujihandros2)")

    def connect(self) -> bool:
        """
        Connection check (in ROS2 mode always returns True, actual connection managed by driver node)

        Returns:
            bool: True
        """
        self.logger.info(f"{self.side.title()} hand waiting for wujihandros2 driver connection...")
        return True

    def is_connected(self) -> bool:
        """
        Check whether driver state has been received (indicates driver has connected to hardware)

        Returns:
            bool: Whether a state message has been received
        """
        with self._lock:
            return self._connected

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """
        Set joint angles (publish to wujihandros2 driver)

        Args:
            positions: Joint angle array, shape (20,) or (5, 4)

        Returns:
            bool: Whether publish succeeded
        """
        try:
            positions = np.asarray(positions, dtype=np.float32)
            if positions.shape == (5, 4):
                positions = positions.flatten()
            elif positions.shape != (self.NUM_JOINTS,):
                self.logger.error(
                    f"Invalid joint angle shape: {positions.shape}, expected (20,) or (5, 4)"
                )
                return False

            # Create JointState message (position only, no name set)
            # wujihandros2 driver supports position-only mode, parsed by index order
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
        Get current joint angles (read from subscription cache)

        Returns:
            np.ndarray: Joint angle array (20,), returns None if not connected
        """
        with self._lock:
            if self._latest_positions is not None:
                return self._latest_positions.copy()
            return None

    def disable(self) -> None:
        """
        Disable the dexterous hand

        Note: wujihandros2 driver automatically disables on node shutdown
        """
        self.logger.info(f"{self.side.title()} hand enable state managed by wujihandros2 driver")

    def release(self) -> None:
        """Release resources"""
        self.disable()
        with self._lock:
            self._connected = False
            self._latest_positions = None
        self.logger.info(f"{self.side.title()} hand ROS2 interface released")
