"""ROS2 node that retargets VisionPro-style hand keypoints to dual Wuji hands.

Simplified to match teleop_real.py structure: direct synchronous processing in callback.
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
import wujihandpy
import numpy as np

from ament_index_python.packages import get_package_share_directory

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rclpy.utilities import remove_ros_args
from wuji_retargeting import Retargeter


def load_hand_config(config_path: str | Path) -> Dict[str, Any]:
    """

    Args:
        config_path: Path to the YAML configuration file

    Returns:
        Dictionary with 'right_hand_serial', 'left_hand_serial', 'use_joint_input',
        and 'input_source' keys.
        Values are None if hand is disabled, or serial number string if enabled.
        'use_joint_input' is a boolean indicating whether to use joint_input mode.
        'input_source' is a string ("avp" or "manus") for selecting retarget config.

    """
    cfg_path = Path(config_path).expanduser().resolve()
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config file not found: {cfg_path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    return {
        'right_hand_serial': config.get('right_hand_serial'),
        'left_hand_serial': config.get('left_hand_serial'),
        'use_joint_input': config.get('use_joint_input', False),
        'input_source': config.get('input_source', 'manus'),
    }


class DualWujiHandRetargeting(Node):
    """
    ROS2 node consuming MediaPipe hand keypoints and commanding two Wuji hands.

    Simplified structure matching teleop_real.py: direct synchronous processing.

    Input Topics:
        - /hand_input (Float32MultiArray): MediaPipe (21, 3) keypoints
          - Single hand: 63 values (21 * 3)
          - Dual hands: 126 values (right hand first, then left hand)
        - /joint_input (Float32MultiArray): Direct joint angles (when use_joint_input=true)
          - Single hand: 20 values (5 fingers × 4 joints)
          - Dual hands: 40 values (right hand first, then left hand)

    Output: Direct hardware control via wujihandpy
    """
    
    INPUT_TOPIC = "/hand_input"
    JOINT_INPUT_TOPIC = "/joint_input"

    def __init__(
        self,
        right_hand_serial: Optional[str],
        left_hand_serial: Optional[str],
        use_joint_input: bool = False,
        input_source: str = "manus",
        retarget_config_path: Optional[str] = None
    ):
        """
        Initialize the dual Wuji hand retargeting node.

        Args:
            right_hand_serial: Serial number of right hand, or None to disable
            left_hand_serial: Serial number of left hand, or None to disable
            use_joint_input: If True, subscribe to /joint_input instead of /hand_input
            input_source: Input source type ("avp" or "manus") for selecting retarget config
            retarget_config_path: Path to retargeter config file (default: auto-select based on input_source)
        """
        super().__init__("wujihand_retargeting")

        self.right_hand_serial = right_hand_serial
        self.left_hand_serial = left_hand_serial
        self.use_joint_input = use_joint_input
        self.input_source = input_source

        # Load retargeter config based on input_source
        if retarget_config_path is None:
            share_dir = Path(get_package_share_directory("wujihand_ik"))
            if input_source == "manus":
                # Manus requires separate configs for left/right due to different z-rotation
                right_retarget_config = str(share_dir / "config" / "retarget_manus_right.yaml")
                left_retarget_config = str(share_dir / "config" / "retarget_manus_left.yaml")
            else:
                # Other input sources use a single config file
                right_retarget_config = str(share_dir / "config" / f"retarget_{input_source}.yaml")
                left_retarget_config = right_retarget_config
        else:
            right_retarget_config = left_retarget_config = retarget_config_path

        # Initialize retargeters from config
        self.right_retargeter = Retargeter.from_yaml(right_retarget_config, "right")
        self.left_retargeter = Retargeter.from_yaml(left_retarget_config, "left")
        self.get_logger().info(f"Loaded retarget config: right={Path(right_retarget_config).name}, left={Path(left_retarget_config).name}")
        
        # Initialize hardware interfaces
        self.right_hand = None
        self.left_hand = None
        self.right_handcontroller = None
        self.left_handcontroller = None
        
        self._init_hands()

        # Configure QoS for real-time performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to input topic based on mode
        if self.use_joint_input:
            # Subscribe to joint_input topic for direct joint angle control
            self.subscription = self.create_subscription(
                JointState,
                self.JOINT_INPUT_TOPIC,
                self._joint_input_callback_joint,
                qos_profile,
            )
            input_topic = self.JOINT_INPUT_TOPIC
            self.get_logger().info("Expecting JointState format from C++ manus_input_node")
        else:
            # Subscribe to hand_input topic for retargeting
            self.subscription = self.create_subscription(
                Float32MultiArray,
                self.INPUT_TOPIC,
                self._hand_input_callback,
                qos_profile,
            )
            input_topic = self.INPUT_TOPIC
        
        # Log initialization
        enabled_hands = []
        if self.right_hand_serial:
            enabled_hands.append(f"right ({self.right_hand_serial})")
        if self.left_hand_serial:
            enabled_hands.append(f"left ({self.left_hand_serial})")
        
        hands_str = ", ".join(enabled_hands) if enabled_hands else "none (simulation mode)"
        mode_str = "joint_input (direct angles)" if self.use_joint_input else "hand_input (retargeting)"
        self.get_logger().info(
            f"Subscribed to '{input_topic}' ({mode_str}) | Enabled hands: {hands_str}"
        )


    def _init_hands(self) -> None:
        """Initialize Wuji hand SDK objects if serial numbers are provided."""
        if self.right_hand_serial:
            self.right_hand, self.right_handcontroller = self._build_hand_interface(
                "right", 
                self.right_hand_serial
            )
        
        if self.left_hand_serial:
            self.left_hand, self.left_handcontroller = self._build_hand_interface(
                "left", 
                self.left_hand_serial
            )
        
        # Wait for hardware to be ready (like teleop_real.py)
        if self.right_hand or self.left_hand:
            time.sleep(0.5)
            self.get_logger().info("Hardware initialization complete.")

    def _build_hand_interface(
        self, 
        side: str, 
        serial: str
    ) -> Tuple[Any, Any]:
        """
        Build hand interface with specified serial number.
        Matches teleop_real.py initialization.
        
        Args:
            side: "right" or "left"
            serial: Serial number of the Wuji hand
            
        Returns:
            Tuple of (hand, controller)
        """
        hand = wujihandpy.Hand(serial_number=serial)
        hand.write_joint_enabled(True)
        controller = hand.realtime_controller(
            enable_upstream=False,
            filter=wujihandpy.filter.LowPass(cutoff_freq=10.0),
        )
        self.get_logger().info(f"{side.title()} hand initialized (serial={serial})")
        return hand, controller


    def _hand_input_callback(self, msg: Float32MultiArray) -> None:
        """
        Handle incoming MediaPipe (21, 3) keypoints for both hands.

        Expected input format:
            - Single hand: 63 values (21 keypoints × 3 coordinates)
            - Dual hands: 126 values (right hand first, then left hand)
        """
        raw_data = np.array(msg.data, dtype=np.float32)
        if raw_data.size == 0:
            return

        try:
            right_pose, left_pose = self._split_finger_data(raw_data)
        except ValueError as exc:
            self.get_logger().error(f"Invalid hand_input payload: {exc}")
            return

        # Process right hand
        if right_pose is not None and self.right_handcontroller is not None:
            right_angles = self.right_retargeter.retarget(right_pose)  # (20,)
            self.right_handcontroller.set_joint_target_position(right_angles.reshape(5, 4))

        # Process left hand
        if left_pose is not None and self.left_handcontroller is not None:
            left_angles = self.left_retargeter.retarget(left_pose)  # (20,)
            self.left_handcontroller.set_joint_target_position(left_angles.reshape(5, 4))


    def _joint_input_callback_joint(self, msg: JointState) -> None:
        """
        Handle incoming joint angles directly from JointState message.

        Expected input format:
            - Single hand: 20 values (5 fingers × 4 joints)
            - Dual hands: 40 values (right hand first, then left hand)
        """
        raw_data = np.array(msg.position, dtype=np.float32)
        if raw_data.size == 0:
            return

        try:
            right_angles, left_angles = self._split_joint_data(raw_data)
        except ValueError as exc:
            self.get_logger().error(f"Invalid joint_input payload: {exc}")
            return

        # Process right hand
        if right_angles is not None and self.right_handcontroller is not None:
            self.right_handcontroller.set_joint_target_position(right_angles.reshape(5, 4))

        # Process left hand
        if left_angles is not None and self.left_handcontroller is not None:
            self.left_handcontroller.set_joint_target_position(left_angles.reshape(5, 4))


    def _split_finger_data(self, raw: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Split flattened array into right/left MediaPipe keypoints.

        Args:
            raw: Flattened numpy array from ROS message

        Returns:
            Tuple of (right_hand_pose, left_hand_pose)
            Each is (21, 3) or None if not provided

        Raises:
            ValueError: If input size doesn't match expected format
        """
        single_len = 21 * 3  # 21 keypoints × 3 coordinates = 63 values
        double_len = single_len * 2  # 126 values for both hands

        if raw.size == double_len:
            if self.input_source == "avp":
                # avp_input publishes: right (63) + left (63)
                right = raw[:single_len].reshape(21, 3)
                left = raw[single_len:].reshape(21, 3)
            else:
                # manus_input_node publishes: left (63) + right (63)
                left = raw[:single_len].reshape(21, 3)
                right = raw[single_len:].reshape(21, 3)
            return right, left

        if raw.size == single_len:
            pose = raw.reshape(21, 3)
            # Route to appropriate hand based on enabled controllers
            if self.left_handcontroller is not None and self.right_handcontroller is None:
                return None, pose  # Left hand only
            else:
                return pose, None  # Right hand (default)

        raise ValueError(
            f"Invalid input size: expected {single_len} (single hand) "
            f"or {double_len} (dual hands), got {raw.size}"
        )

    def _split_joint_data(self, raw: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Split flattened array into right/left joint angles.
        
        Args:
            raw: Flattened numpy array from ROS message
            
        Returns:
            Tuple of (right_hand_angles, left_hand_angles)
            Each is (20,) or None if not provided
            
        Raises:
            ValueError: If input size doesn't match expected format
        """
        single_len = 20  # 5 fingers × 4 joints = 20 values
        double_len = single_len * 2  # 40 values for both hands

        if raw.size == double_len:
            right = raw[:single_len]
            left = raw[single_len:]
            return right, left

        if raw.size == single_len:
            # If only one hand is enabled, send data to the enabled hand
            # Check which hand controller is available
            if self.left_handcontroller is not None and self.right_handcontroller is None:
                return None, raw  # Left hand only
            else:
                return raw, None  # Right hand (default)

        raise ValueError(
            f"Invalid joint_input size: expected {single_len} (single hand) "
            f"or {double_len} (dual hands), got {raw.size}"
        )


    def shutdown(self) -> None:
        """Cleanly stop hand controllers (matches teleop_real.py finally block)."""
        self.get_logger().info("Shutting down...")
        
        if self.right_hand is not None:
            self.right_hand.write_joint_enabled(False)
            self.get_logger().info("Right hand disabled")
        
        if self.left_hand is not None:
            self.left_hand.write_joint_enabled(False)
            self.get_logger().info("Left hand disabled")

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Retarget hand keypoints to Wuji hands via /hand_input topic."
    )
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to YAML config file with serial numbers (default: use package config).",
    )
    parser.add_argument(
        "-i",
        "--input-source",
        default=None,
        choices=["avp", "manus"],
        help="Input source type: 'avp' or 'manus'. Overrides config file setting.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    """Main entry point for the wujihand retargeting node."""
    program_name = sys.argv[0] if sys.argv else "wujihand_retargeting"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    args = _parse_args(cli_argv)
    if args.config:
        config_path = Path(args.config)
    else:
        share_dir = Path(get_package_share_directory("wujihand_ik"))
        config_path = share_dir / "config" / "wujihand_ik.yaml"

    # Load serial numbers and joint_input mode from config
    config = load_hand_config(config_path)
    right_serial = config['right_hand_serial']
    left_serial = config['left_hand_serial']
    use_joint_input = config.get('use_joint_input', False)
    # Command line --input-source overrides config file
    input_source = args.input_source if args.input_source else config.get('input_source', 'manus')

    # Initialize ROS and node (simplified, like teleop_real.py)
    rclpy.init(args=raw_argv)
    node = DualWujiHandRetargeting(right_serial, left_serial, use_joint_input, input_source)
    
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
