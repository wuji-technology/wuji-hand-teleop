"""Dexterous hand controller node (per-hand process).

One process per hand: each subscribes directly to the C++
manus_data_publisher topics, skipping the Python manus_input
intermediate layer and removing the dual-hand sync wait bottleneck.

Architecture:
  manus_data_publisher (C++) -> /manus_glove_0 (120Hz)
                             -> /manus_glove_1 (120Hz)
       v                              v
  left controller (this process)  right controller (other process)
       v                              v
  /left_hand/joint_commands       /right_hand/joint_commands

Multi-core parallel: each hand runs on its own process, GIL, and core.
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool, Trigger

from wujihand_output import WujiHandController
from .common import (
    ControlMode,
    ROS2LoggerAdapter,
    get_default_qos,
)

# Manus -> MediaPipe mapping (moved here from manus_input_node.py).
# MediaPipe: 0=WRIST, 1-4=THUMB, 5-8=INDEX, 9-12=MIDDLE, 13-16=RING, 17-20=PINKY
_MEDIAPIPE_TO_MANUS = (
    1, 22, 23, 24, 25,
    3, 4, 5, 6,
    8, 9, 10, 11,
    13, 14, 15, 16,
    18, 19, 20, 21,
)


def _convert_to_mediapipe(msg) -> Optional[np.ndarray]:
    """Convert Manus ManusGlove message to MediaPipe (21, 3) format.

    Returns None if any required node_id is missing — callers should
    drop the frame rather than substitute origin (0,0,0), which would
    yank the hand toward an invalid pose.
    """
    positions = {}
    for node in msg.raw_nodes:
        pose = node.pose
        positions[node.node_id] = np.array(
            [pose.position.x, -pose.position.y, pose.position.z],
            dtype=np.float32,
        )

    if not set(_MEDIAPIPE_TO_MANUS).issubset(positions):
        return None

    result = np.empty((21, 3), dtype=np.float32)
    for mp_idx, manus_id in enumerate(_MEDIAPIPE_TO_MANUS):
        result[mp_idx] = positions[manus_id]
    return result


class WujiHandControllerNode(Node):
    """Single-hand dexterous hand controller node.

    Subscribes directly to the C++ manus_data_publisher topics, runs the
    MediaPipe conversion + retarget + publish loop in-process. One node
    instance per hand.
    """

    def __init__(self, side: str, hand_name: str):
        super().__init__(f"wujihand_controller_{side}")

        self._side = side
        self._mode = ControlMode.TELEOP
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self.controller = None  # set below; checked by shutdown() on early failure

        self.declare_parameter('control_rate', 120.0)
        self.declare_parameter('nlopt_max_eval', 25)
        control_rate = float(self.get_parameter('control_rate').value)
        nlopt_max_eval = int(self.get_parameter('nlopt_max_eval').value)
        if control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if nlopt_max_eval < 0:
            raise ValueError("nlopt_max_eval must be >= 0 (0 = keep library default)")

        # Latest manus data cache (written by callback, consumed by timer)
        self._latest_keypoints: Optional[np.ndarray] = None

        # Verify dependencies BEFORE acquiring hardware so an ImportError
        # doesn't leave a half-initialized hand controller behind.
        try:
            from manus_ros2_msgs.msg import ManusGlove
        except ImportError:
            self.get_logger().error(
                "manus_ros2_msgs not installed, cannot subscribe to Manus topics"
            )
            raise

        # Acquire hardware. If anything below this line raises, the
        # except clause releases the controller before re-raising.
        self.get_logger().info(f"Initializing {side} hand controller (wujihandros2)...")
        self.controller = WujiHandController(
            side=side,
            hand_name=hand_name,
            input_source="manus",
            node=self,
            logger=self._logger_adapter,
            nlopt_max_eval=nlopt_max_eval,
        )
        self.get_logger().info("Controller initialized")

        try:
            qos = get_default_qos()

            # Subscribe to all manus topics, filter to this side via msg.side.
            # (Manus SDK assigns glove index by connect order; 0=left/1=right
            # is not guaranteed.)
            self.create_subscription(ManusGlove, "/manus_glove_0", self._glove_callback, qos)
            self.create_subscription(ManusGlove, "/manus_glove_1", self._glove_callback, qos)
            self.get_logger().info(
                f"Subscribed to Manus topics: /manus_glove_0, /manus_glove_1 (filter side={side})"
            )

            # Services
            self.create_service(
                SetBool, f'/wuji_hand/{side}/switch_mode', self._switch_mode_callback)
            self.create_service(
                Trigger, f'/wuji_hand/{side}/get_mode', self._get_mode_callback)

            self.create_timer(1.0 / control_rate, self._teleop_loop)
        except Exception:
            # Release the hardware controller before propagating, so we
            # don't leak a connected wujihandros2 driver session.
            try:
                self.controller.disable_and_release()
            except Exception:
                self.get_logger().exception("Failed to release controller during init rollback")
            self.controller = None
            raise

        self.get_logger().info(
            f"Ready: side={side}, rate={control_rate}Hz, "
            f"filter=msg.side={side} -> /{hand_name}/joint_commands"
        )

    # -------------------- Subscription callback --------------------

    def _glove_callback(self, msg) -> None:
        """Manus data arrives -> filter by msg.side -> MediaPipe convert -> cache."""
        if self._mode != ControlMode.TELEOP:
            return
        if msg.side.lower() != self._side:
            return  # not this side, skip
        keypoints = _convert_to_mediapipe(msg)
        if keypoints is not None:
            self._latest_keypoints = keypoints
        # else: drop frame (incomplete sensor data); reuse last valid keypoints

    # -------------------- Service callbacks --------------------

    def _switch_mode_callback(self, request, response):
        new_mode = ControlMode.INFERENCE if request.data else ControlMode.TELEOP
        # This per-hand Manus controller has no inference data path, so
        # accepting INFERENCE would silently halt joint command output.
        # Reject explicitly instead of pretending to switch.
        if new_mode == ControlMode.INFERENCE:
            response.success = False
            response.message = (
                "INFERENCE mode is not supported by this per-hand Manus controller; "
                "this node only forwards Manus retargeting (TELEOP)."
            )
            return response
        if self._mode != new_mode:
            self._mode = new_mode
            self._latest_keypoints = None
            self.get_logger().info(f"Switched to {new_mode.value} mode")
        response.success = True
        response.message = f"Current mode: {new_mode.value}"
        return response

    def _get_mode_callback(self, request, response):
        response.success = True
        response.message = self._mode.value
        return response

    # -------------------- Control loop --------------------

    def _teleop_loop(self):
        """Timer tick: consume-once -> retarget -> publish joint_commands."""
        if self._mode != ControlMode.TELEOP:
            return

        kp = self._latest_keypoints
        if kp is None:
            return
        self._latest_keypoints = None

        self.controller.set_keypoints(kp)

    # -------------------- Lifecycle --------------------

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        if self.controller is not None:
            self.controller.disable_and_release()
        self.get_logger().info("Shutdown complete")


# -------------------- Entry point --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dexterous hand controller (per-hand)")
    parser.add_argument("--side", required=True, choices=["left", "right"],
                        help="Which hand this process controls")
    parser.add_argument("--hand-name", help="wujihandros2 driver namespace")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "wujihand_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    side = args.side
    default_hand_name = "left_hand" if side == "left" else "right_hand"
    hand_name = args.hand_name or default_hand_name

    rclpy.init(args=raw_argv)
    node = WujiHandControllerNode(side=side, hand_name=hand_name)

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
