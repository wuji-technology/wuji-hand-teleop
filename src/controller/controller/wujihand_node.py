"""Wuji-hand controller node (one process per hand, multi-core parallelism).

input_source is selected by wujihand_ik.yaml: 'wuji_glove' (UDP, in-process)
or 'manus' (subscribes /manus_glove_*); publishes /{side}_hand/joint_commands.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from wujihand_output import WujiHandController
from .common import (
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# Manus -> MediaPipe mapping (lifted from the original manus_input_node.py).
# MediaPipe: 0=WRIST, 1-4=THUMB, 5-8=INDEX, 9-12=MIDDLE, 13-16=RING, 17-20=PINKY
_MEDIAPIPE_TO_MANUS = (
    1, 22, 23, 24, 25,
    3, 4, 5, 6,
    8, 9, 10, 11,
    13, 14, 15, 16,
    18, 19, 20, 21,
)

# Default control-loop rate (Hz). Override with the `control_rate` ROS2 param.
# 120Hz matches the upper bound of Manus / wuji_glove skeleton frames; higher
# adds no new input. The wujihand C++ driver on the host runs 1000Hz down to
# the firmware, so publishing faster from the controller is pointless.
DEFAULT_CONTROL_RATE_HZ = 120.0

# wuji_glove reconnect behavior: if the main loop receives no skeleton frame
# for _RECV_TIMEOUT_SEC seconds in a row, treat the underlying connection as
# lost (unplugged glove, network drop, power loss, etc.) and call
# manager.connect() to reconnect.
# Default ConnectOptions: timeout_ms=1000, retry_count=3 — when offline, a
# single reconnect blocks ~3s worst-case (i.e. the main loop misses ~3s of
# joint_commands; output resumes naturally once the link is back).
_RECV_TIMEOUT_SEC = 2.0


def _convert_to_mediapipe(msg) -> np.ndarray:
    """Convert Manus ManusGlove message to MediaPipe (21, 3) format."""
    positions = {}
    for node in msg.raw_nodes:
        pose = node.pose
        positions[node.node_id] = np.array(
            [pose.position.x, -pose.position.y, pose.position.z],
            dtype=np.float32,
        )

    result = np.zeros((21, 3), dtype=np.float32)
    for mp_idx, manus_id in enumerate(_MEDIAPIPE_TO_MANUS):
        if manus_id in positions:
            result[mp_idx] = positions[manus_id]
    return result


def _extract_wuji_glove_keypoints(skeleton) -> Optional[np.ndarray]:
    """Convert a wuji_sdk HandSkeleton to MediaPipe-style (21, 3) float32.

    Returns None if the skeleton does not have exactly 21 joints
    (caller should skip the frame).
    """
    joints = skeleton.joints
    if len(joints) != 21:
        return None
    kp = np.array(
        [j.pose.position for j in joints],
        dtype=np.float32,
    )
    if kp.shape != (21, 3):
        return None
    return kp


class WujiHandControllerNode(Node):
    """Per-hand wujihand controller node.

    Dispatches at __init__ on cfg['input_source']:
      - 'wuji_glove': connect via wuji_sdk (UDP), subscribe hand_skeleton
      - 'manus':      subscribe /manus_glove_0,1 (existing behavior)
    """

    def __init__(self, side: str, hand_name: str, cfg: dict,
                 glove_config_path: Optional[str] = None,
                 retarget_config_dir: Optional[str] = None):
        super().__init__(f"wujihand_controller_{side}")

        self._side = side
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._input_source = cfg.get("input_source", "wuji_glove")
        # _latest_keypoints is written by ROS subscription callbacks and
        # consumed by the timer-driven control loop on a different thread;
        # the lock keeps writes/reads atomic and prevents losing a frame
        # mid-swap.
        self._latest_keypoints: Optional[np.ndarray] = None
        self._keypoints_lock = threading.Lock()

        # Wuji-glove-only attributes (None for manus path)
        self._sdk_device = None
        self._sdk_sub = None
        # Stash connect params for reconnects (populated by _setup_wuji_glove).
        self._glove_sn: Optional[str] = None
        self._glove_device_name: Optional[str] = None
        self._glove_config_path: Optional[str] = None
        # recv watchdog: main loop treats (now - _last_recv_time) > _RECV_TIMEOUT_SEC as a disconnect.
        self._last_recv_time: float = 0.0
        self._reconnect_log_counter: int = 0

        # Controller (drives retargeter + wujihand driver)
        self.get_logger().info(
            f"Initializing {side}-hand controller (input_source={self._input_source})..."
        )
        self.controller = WujiHandController(
            side=side,
            hand_name=hand_name,
            input_source=self._input_source,
            node=self,
            logger=self._logger_adapter,
            retarget_config_dir=retarget_config_dir,
        )
        self.get_logger().info("Controller initialized")

        # Dispatch on input_source
        if self._input_source == "wuji_glove":
            self._setup_wuji_glove(glove_config_path)
        elif self._input_source == "manus":
            self._setup_manus()
        else:
            raise ValueError(
                f"unknown input_source: {self._input_source!r} "
                f"(expected 'wuji_glove' or 'manus')"
            )

        # control_rate comes from a ROS2 param, default 120Hz (see the
        # DEFAULT_CONTROL_RATE_HZ comment at the top of this module). No
        # rebuild needed to change it: launch -p control_rate:=... works.
        self.declare_parameter('control_rate', DEFAULT_CONTROL_RATE_HZ)
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        if self._control_rate_hz <= 0.0:
            raise ValueError(
                f"control_rate must be > 0, got {self._control_rate_hz}")
        self.create_timer(1.0 / self._control_rate_hz, self._teleop_loop)

        self.get_logger().info(
            f"Ready: side={side}, source={self._input_source}, "
            f"rate={self._control_rate_hz:.1f}Hz -> /{hand_name}/joint_commands"
        )

    # ==================== input_source=wuji_glove ====================

    def _setup_wuji_glove(self, glove_config_path: Optional[str]) -> None:
        """Load the glove config and try the first connection. A first-connect
        failure does NOT raise (the main loop keeps retrying); configuration
        errors such as a hand_side mismatch still fail-fast.
        """
        # Resolve glove config path: prefer caller-supplied (from launch), fall
        # back to ament index.
        if glove_config_path is None:
            glove_config_path = get_package_config_path(
                "wuji_glove", "wuji_glove.yaml"
            )
        glove_cfg = load_yaml_config(glove_config_path)[f"{self._side}_glove"]
        self._glove_sn = glove_cfg["serial_number"]
        self._glove_device_name = glove_cfg.get(
            "device_name", f"{self._side}_glove"
        )
        self._glove_config_path = glove_config_path

        # First connect: do not raise on failure (e.g. glove not powered at
        # startup); the main loop keeps retrying.
        self._connect_glove()

    def _connect_glove(self) -> bool:
        """Connect or reconnect a Wuji Glove.

        Returns True on success. A transient failure (device offline, network
        drop) returns False; the main loop tries again next tick.
        An SN / hand_side mismatch still raises RuntimeError — that is a
        configuration error and should not be papered over by retries.
        """
        from wuji_sdk import SdkManager, ConnectOptions  # lazy: only loaded on wuji_glove path

        # Release any prior handles (harmless if already None).
        self._sdk_sub = None
        self._sdk_device = None

        try:
            manager = SdkManager.instance()
            # enable_bridge=False keeps the glove off the zenoh device-bridge.
            # The default (True) would call declare_node_token + start_bridge_for
            # after a successful direct connect, advertising this glove on the
            # LAN so any peer could discover and take it over. Direct-connect
            # failure also falls back to zenoh discovery — both paths leak.
            opts = ConnectOptions(enable_bridge=False)
            device = manager.connect(
                sn=self._glove_sn,
                device_name=self._glove_device_name,
                options=opts,
            )
        except Exception as e:
            # Transient failure: throttle logs (one per 10 attempts) so we
            # do not flood the console.
            self._reconnect_log_counter += 1
            if self._reconnect_log_counter == 1 or self._reconnect_log_counter % 10 == 0:
                self.get_logger().warn(
                    f"wuji_sdk connect attempt #{self._reconnect_log_counter} failed: {e}"
                )
            return False

        # SN / side mismatch is a config error — do not paper over it with retries.
        actual_side = device.hand_side().get().lower()
        if actual_side != self._side:
            raise RuntimeError(
                f"{self._side}_glove SN={self._glove_sn} reports hand_side={actual_side}; "
                f"swap left_glove/right_glove SNs in wuji_glove.yaml."
            )

        self._sdk_device = device
        self._sdk_sub = device.hand_skeleton().subscribe()
        self._last_recv_time = time.monotonic()
        was_retry = self._reconnect_log_counter > 0
        self._reconnect_log_counter = 0
        self.get_logger().info(
            f"wuji_sdk {'re' if was_retry else ''}connected: SN={self._glove_sn} "
            f"side={actual_side} device_name={self._glove_device_name} "
            f"(config={self._glove_config_path})"
        )
        return True

    def _teleop_loop_wuji_glove(self) -> None:
        now = time.monotonic()

        # No active connection -> (re)connect.
        if self._sdk_sub is None:
            self._connect_glove()
            return

        skeleton = self._sdk_sub.recv()
        if skeleton is None:
            # No frame this tick — has the link been quiet too long?
            if now - self._last_recv_time > _RECV_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"wuji_sdk: no skeleton frame for {now - self._last_recv_time:.1f}s, "
                    f"reconnecting..."
                )
                self._connect_glove()  # release old sub + reconnect
            return

        # Frame received — refresh the watchdog timestamp.
        self._last_recv_time = now

        # Drain queue: keep only the latest frame to prevent lag buildup
        # when the SDK pushes faster than 120Hz.
        while True:
            newer = self._sdk_sub.recv()
            if newer is None:
                break
            skeleton = newer

        kp = _extract_wuji_glove_keypoints(skeleton)
        if kp is None:
            return
        self.controller.set_keypoints(kp)

    # ==================== input_source=manus ====================

    def _setup_manus(self) -> None:
        from manus_ros2_msgs.msg import ManusGlove  # lazy: only on manus path
        qos = get_default_qos()
        self.create_subscription(ManusGlove, "/manus_glove_0", self._manus_callback, qos)
        self.create_subscription(ManusGlove, "/manus_glove_1", self._manus_callback, qos)
        self.get_logger().info(
            f"Subscribed to Manus topics: /manus_glove_0, /manus_glove_1 "
            f"(filtering side={self._side})"
        )

    def _manus_callback(self, msg) -> None:
        if msg.side.lower() != self._side:
            return
        kp = _convert_to_mediapipe(msg)
        with self._keypoints_lock:
            self._latest_keypoints = kp

    def _teleop_loop_manus(self) -> None:
        with self._keypoints_lock:
            kp = self._latest_keypoints
            self._latest_keypoints = None  # consume-once
        if kp is None:
            return
        self.controller.set_keypoints(kp)

    # ==================== shared ====================

    def _teleop_loop(self) -> None:
        if self._input_source == "wuji_glove":
            self._teleop_loop_wuji_glove()
        else:
            self._teleop_loop_manus()

    # ==================== lifecycle ====================

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        if self._sdk_sub is not None:
            self._sdk_sub = None  # SDK has no explicit unsubscribe; release ref
        if self._sdk_device is not None:
            self._sdk_device = None
        self.controller.disable_and_release()
        self.get_logger().info("Exited cleanly")


# -------------------- Entry point --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wuji-hand controller (per-hand)")
    parser.add_argument("--side", required=True, choices=["left", "right"],
                        help="which hand to drive")
    parser.add_argument("--hand-name", help="wujihandros2 driver namespace")
    parser.add_argument("-c", "--config", help="wujihand_ik.yaml path")
    parser.add_argument(
        "--glove-config",
        help="wuji_glove.yaml path (used when input_source=wuji_glove; "
             "falls back to the wuji_glove package default via ament_index)",
    )
    parser.add_argument(
        "--retarget-config-dir",
        help="Directory containing retarget yaml (overrides the wujihand_output "
             "package's default config/). Lookup order: "
             "retarget_{input_source}_{side}.yaml -> retarget_{input_source}.yaml. "
             "Use for cross-host deployments where launch passes an explicit "
             "override directory so retarget params follow the deploy host "
             "rather than the in-package default config/.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "wujihand_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    side = args.side
    default_hand_name = "left_hand" if side == "left" else "right_hand"
    hand_name = args.hand_name or default_hand_name

    config_path = args.config or get_package_config_path(
        "wujihand_output", "wujihand_ik.yaml"
    )
    cfg = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    node = WujiHandControllerNode(
        side=side, hand_name=hand_name, cfg=cfg,
        glove_config_path=args.glove_config,
        retarget_config_dir=args.retarget_config_dir,
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
