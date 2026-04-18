"""Dexterous hand controller node

Controls Wuji dexterous hand via wujihandros2 driver (C++ wujihandcpp SDK)

Two control modes:
1. TELEOP mode: hand_input → IK retargeting → dexterous hand
2. INFERENCE mode: joint_command → dexterous hand

Control architecture (timer-driven, fixed frequency):
  Subscription callbacks only cache data → 100Hz timer consume-once → retarget → hardware
  Ensures uniform output timing, avoids timing jitter under CPU load

  Note: driver (C++) forwards the latest commands to firmware at 1000Hz,
  so repeated sending from the controller does not increase the actual hardware update rate.
  The correct way to improve smoothness is to increase retarget frequency (requires C++ rewrite).

Mode switching service: /wuji_hand/switch_mode
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float32MultiArray, Header

from wujihand_output import WujiHandController
from .common import (
    ControlMode,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# Topic names
LEFT_HAND_CMD_TOPIC = "/wuji_hand/left/joint_command"
RIGHT_HAND_CMD_TOPIC = "/wuji_hand/right/joint_command"
HAND_INPUT_TOPIC = "/hand_input"

# Control frequency (Hz)
# 100Hz: retarget ~6.6ms/frame → CPU ~66%, leaves margin for GC/scheduling
# lp_alpha=0.3 @ 100Hz → cutoff 5.7Hz, covers human hand dynamics
# driver (C++ 1000Hz) automatically repeats forwarding latest commands to firmware at 1000Hz
CONTROL_RATE_HZ = 100.0

# Joint names
JOINT_NAMES = [
    "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
    "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
    "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
    "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
    "pinky_joint_0", "pinky_joint_1", "pinky_joint_2", "pinky_joint_3",
]


class WujiHandControllerNode(Node):
    """Dexterous hand controller node (via wujihandros2 driver)

    Control architecture (timer-driven):
      Subscription callbacks only cache latest data (zero computation) → 100Hz timer consume-once → retarget → hardware
      - Fixed 10ms interval, uniform output timing
      - CPU ~66%, leaves margin for Python GIL/GC
      - Duplicate frames from 120Hz input are automatically skipped
      - driver (C++ 1000Hz) automatically repeats forwarding, no need for high-frequency sending from controller

    Joint states are published directly by wujihandros2 driver (1000Hz):
      /{hand_name}/joint_states — subscribe to this topic for data recording/monitoring
    """

    def __init__(
        self,
        input_source: str = "manus",
        left_hand_name: Optional[str] = None,
        right_hand_name: Optional[str] = None,
    ):
        super().__init__("wujihand_controller")

        self._mode = ControlMode.TELEOP
        self._input_source = input_source
        self._left_hand_name = left_hand_name
        self._right_hand_name = right_hand_name
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())

        # Initialize controller (via wujihandros2)
        self.get_logger().info("Initializing dexterous hand controller (wujihandros2)...")
        self.controller = WujiHandController(
            input_source=input_source,
            logger=self._logger_adapter,
            node=self,
            left_hand_name=left_hand_name,
            right_hand_name=right_hand_name,
        )
        self.get_logger().info("Controller initialization complete")

        # TELEOP: cache latest hand input (written by callback, consumed by timer)
        self._latest_raw: Optional[np.ndarray] = None

        # INFERENCE: joint command cache
        self._left_inference_angles: Optional[np.ndarray] = None
        self._right_inference_angles: Optional[np.ndarray] = None

        qos = get_default_qos()

        # Publishers (command topics, states published by wujihandros2 driver at 1000Hz)
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_HAND_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_HAND_CMD_TOPIC, qos)

        # Subscribers (cache data only, no retarget)
        self.hand_input_sub = self.create_subscription(
            Float32MultiArray, HAND_INPUT_TOPIC, self._hand_input_callback, qos)
        self.left_cmd_sub = self.create_subscription(
            JointState, LEFT_HAND_CMD_TOPIC, self._left_cmd_callback, qos)
        self.right_cmd_sub = self.create_subscription(
            JointState, RIGHT_HAND_CMD_TOPIC, self._right_cmd_callback, qos)

        # Services
        self.create_service(SetBool, '/wuji_hand/switch_mode', self._switch_mode_callback)
        self.create_service(Trigger, '/wuji_hand/get_mode', self._get_mode_callback)

        # Timer (fixed frequency, ensures uniform output timing)
        control_period = 1.0 / CONTROL_RATE_HZ
        self.create_timer(control_period, self._teleop_loop)
        self.create_timer(control_period, self._inference_loop)

        self.get_logger().info(
            f"Initialization complete, mode: {self._mode.value.upper()}, "
            f"Control frequency: {CONTROL_RATE_HZ}Hz"
        )

    @property
    def mode(self) -> ControlMode:
        return self._mode

    # -------------------- Service Callbacks --------------------

    def _switch_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        new_mode = ControlMode.INFERENCE if request.data else ControlMode.TELEOP
        if self._mode != new_mode:
            self._mode = new_mode
            self._latest_raw = None
            self.get_logger().info(f"Switched to {new_mode.value} mode")
        response.success = True
        response.message = f"Current mode: {new_mode.value}"
        return response

    def _get_mode_callback(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self._mode.value
        return response

    # -------------------- Subscription Callbacks (cache only, zero computation) --------------------

    def _left_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self._left_inference_angles = np.array(msg.position, dtype=np.float32)

    def _right_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self._right_inference_angles = np.array(msg.position, dtype=np.float32)

    def _hand_input_callback(self, msg: Float32MultiArray):
        """Cache latest hand input (zero computation), consumed by _teleop_loop timer"""
        if self._mode != ControlMode.TELEOP:
            return
        raw = np.array(msg.data, dtype=np.float32)
        if raw.size > 0:
            self._latest_raw = raw

    # -------------------- Control Loop (100Hz timer-driven) --------------------

    def _teleop_loop(self):
        """TELEOP control loop: consume-once → retarget → hardware

        Fixed 100Hz (10ms) interval, ensures:
        - Uniform output timing (no CPU-load jitter)
        - Each frame retargeted only once (no redundant computation)
        - CPU ~66% (leaves margin for Python GIL/GC)
        - driver automatically repeats forwarding latest commands to firmware at 1000Hz
        """
        if self._mode != ControlMode.TELEOP:
            return

        # Consume-once: retrieve and clear cache
        raw = self._latest_raw
        if raw is None:
            return
        self._latest_raw = None

        try:
            right_kp, left_kp = self._split_keypoints(raw)
        except ValueError as e:
            self.get_logger().error(f"Invalid data: {e}")
            return

        _, _, left_angles, right_angles = self.controller.set_keypoints(
            left_keypoints=left_kp, right_keypoints=right_kp)
        self._publish_command(left_angles, right_angles)

    def _inference_loop(self):
        """INFERENCE control loop: directly forward joint commands"""
        if self._mode != ControlMode.INFERENCE:
            return
        left = self._left_inference_angles
        right = self._right_inference_angles
        if left is not None and self.controller.left_hand:
            self.controller.left_hand.set_joint_positions(left)
        if right is not None and self.controller.right_hand:
            self.controller.right_hand.set_joint_positions(right)

    # -------------------- Publishing --------------------

    def _publish_command(self, left: Optional[np.ndarray], right: Optional[np.ndarray]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header = Header(stamp=stamp, frame_id="left_hand")
            msg.name = JOINT_NAMES
            msg.position = left.tolist()
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header = Header(stamp=stamp, frame_id="right_hand")
            msg.name = JOINT_NAMES
            msg.position = right.tolist()
            self.right_cmd_pub.publish(msg)

    # -------------------- Utility Methods --------------------

    def _split_keypoints(self, raw: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Split keypoint data into left and right hands"""
        single, double = 63, 126
        if raw.size == double:
            return raw[:single].reshape(21, 3), raw[single:].reshape(21, 3)
        if raw.size == single:
            kp = raw.reshape(21, 3)
            if self.controller.is_left_connected() and not self.controller.is_right_connected():
                return None, kp
            return kp, None
        raise ValueError(f"Expected {single} or {double}, got {raw.size}")

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        self.controller.disable_and_release()
        self.get_logger().info("Safely exited")


# -------------------- Entry Function --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dexterous hand controller")
    parser.add_argument("-c", "--config", help="Configuration file path")
    parser.add_argument("-i", "--input-source", choices=["manus"], help="Input source")
    parser.add_argument("--left-hand", help="Left hand wujihandros2 driver namespace")
    parser.add_argument("--right-hand", help="Right hand wujihandros2 driver namespace")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "wujihand_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    # Load configuration (CLI args take precedence over config file)
    config_path = args.config or get_package_config_path("wujihand_output", "wujihand_ik.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    left_hand_cfg = config.get('left_hand', {})
    right_hand_cfg = config.get('right_hand', {})
    node = WujiHandControllerNode(
        input_source=args.input_source or config.get('input_source', 'manus'),
        left_hand_name=args.left_hand or left_hand_cfg.get('name'),
        right_hand_name=args.right_hand or right_hand_cfg.get('name'),
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
