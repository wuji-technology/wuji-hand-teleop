"""Tianji arm controller node: TF -> IK -> hardware."""
from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from rclpy.utilities import remove_ros_args
from rclpy._rclpy_pybind11 import RCLError
from tf2_ros import Buffer, TransformListener
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8
from std_srvs.srv import SetBool, Trigger

from tianji_output import TianjiChestDriver
from .common import (
    ArmState,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)


# Topic names
LEFT_ARM_CMD_TOPIC = "/left_arm/joint_commands"
RIGHT_ARM_CMD_TOPIC = "/right_arm/joint_commands"
LEFT_ARM_STATE_TOPIC = "/left_arm/joint_states"
RIGHT_ARM_STATE_TOPIC = "/right_arm/joint_states"

# Latched lifecycle state on /tianji_arm/lifecycle_state (Int8). Replaces the
# old Bool /tianji_arm/teleop_ready — that signal was edge-only ("ready or
# not"), so a Monitor waiting on it couldn't tell the difference between
# "still initializing" and "_do_enable failed with move_to_init residual".
# The Monitor's Stop button got stuck disabled in the second case. Lifecycle
# is the state machine the operator actually needs to see:
#
#   INITIALIZING  - node __init__ running; controller not yet armed
#   ENABLING      - _do_enable in progress (set_position_mode + move_to_init
#                   + set_impedance_mode); Marvin holds servos
#   READY         - _do_enable completed; teleop loop running
#   DISABLED      - _do_disable called (set_standby); waiting for re-enable
#   ENABLE_FAILED - _do_enable raised; operator should Stop or call retry
#   SDK_ERROR     - _on_sdk_error called; controller wedged
LIFECYCLE_INITIALIZING = 0
LIFECYCLE_ENABLING = 1
LIFECYCLE_READY = 2
LIFECYCLE_DISABLED = 3
LIFECYCLE_ENABLE_FAILED = 4
LIFECYCLE_SDK_ERROR = 5


class TianjiArmControllerNode(Node):
    """Tianji arm controller node."""

    def __init__(self, robot_ip: str = '192.168.1.190'):
        super().__init__("tianji_arm_controller")

        # auto_enable=True (default): node powers up the arm asynchronously
        # via a one-shot timer after __init__ returns. Setting it False keeps
        # the arm in standby on startup; operators then call ~/set_enabled to
        # bring it up. Decoupling enable from __init__ keeps SIGINT during the
        # ~10s move_to_init from leaving the node half-constructed (so
        # shutdown() never ran and the arm stayed live).
        self.declare_parameter('auto_enable', True)
        # Control loop / state-publish rates (Hz, two timers decoupled):
        # - control_rate: TF -> IK control path. 120Hz matches the upper bound
        #   of HTC tracker / MANUS / wuji_glove input streams; higher adds no
        #   new command information.
        # - state_publish_rate: /{side}_arm/joint_states publish rate.
        #   Downstream inference + data capture want finer state feedback;
        #   500Hz is stable under Python + DDS.
        # No rebuild required to change rates — launch / param yaml overrides work.
        self.declare_parameter('control_rate', 120.0)
        self.declare_parameter('state_publish_rate', 500.0)
        # Handoff after move_to_init: hold the init pose for `handoff_hold_sec`,
        # then run a `handoff_ramp_sec` smoothstep ramp from the snapshot of
        # current hardware joints to the live IK target. Avoids the snap from
        # init pose to the operator's tracker pose on the first IK frame.
        # Set either to 0.0 to disable that phase. Default 0.5s hold + 1.0s
        # ramp is the value that field-tested cleanly on Wuji Hand + Tianji.
        self.declare_parameter('handoff_hold_sec', 0.5)
        self.declare_parameter('handoff_ramp_sec', 1.0)
        auto_enable = bool(self.get_parameter('auto_enable').value)
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        self._state_publish_rate_hz = float(self.get_parameter('state_publish_rate').value)
        self._handoff_hold_sec = max(0.0, float(self.get_parameter('handoff_hold_sec').value))
        self._handoff_ramp_sec = max(0.0, float(self.get_parameter('handoff_ramp_sec').value))
        if self._control_rate_hz <= 0.0:
            raise ValueError(
                f"control_rate must be > 0, got {self._control_rate_hz}")
        if self._state_publish_rate_hz <= 0.0:
            raise ValueError(
                f"state_publish_rate must be > 0, got {self._state_publish_rate_hz}")

        self._arm_state = ArmState.IMPEDANCE
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._log_counter = 0
        self._state_error_count = 0  # consecutive get_current_joints failures

        # Initialize the controller.
        self.get_logger().info(f"Connecting to robot {robot_ip}...")
        self.controller = TianjiChestDriver(robot_ip=robot_ip, logger=self._logger_adapter)

        # STANDBY / ACTIVE state management.
        self._arm_enabled = False
        self._enable_lock = threading.Lock()

        # Pose and joint caches.
        self.left_pose = None
        self.right_pose = None
        self.left_y_axis = None
        self.right_y_axis = None
        self._last_left_state_joints = None
        self._last_right_state_joints = None

        # Handoff ramp state — populated at the end of _do_enable, consumed by
        # _teleop_control. `_handoff_start_at is None` means no ramp pending.
        self._handoff_start_at: Optional[float] = None
        self._handoff_start_left: Optional[list] = None
        self._handoff_start_right: Optional[list] = None

        # Always enter standby first. set_standby failure (e.g. state=100
        # under e-stop) must not block node startup, otherwise launch
        # respawn=True crash-loops until the fault is cleared externally.
        try:
            self.controller.set_standby()
            self.get_logger().info("Arm connected; servos in standby")
        except Exception as exc:
            self.get_logger().error(
                f"set_standby failed at startup (node will continue, "
                f"awaiting operator intervention): {exc}"
            )

        # If auto_enable, schedule async enable via a one-shot timer. Keeping
        # the ~10s move_to_init out of __init__ means SIGINT during startup
        # arrives at a fully-constructed node, so shutdown() runs and the arm
        # is brought back to standby.
        self._auto_enable_timer = None
        if auto_enable:
            self._auto_enable_timer = self.create_timer(0.1, self._auto_enable_once)

        # TF listener.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = get_default_qos()

        # Publishers.
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_ARM_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_ARM_CMD_TOPIC, qos)
        self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_TOPIC, qos)
        self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_TOPIC, qos)

        # zsp_para and pose publishers.
        self.left_zsp_para_pub = self.create_publisher(Float64MultiArray, '/left_arm/zsp_para', qos)
        self.right_zsp_para_pub = self.create_publisher(Float64MultiArray, '/right_arm/zsp_para', qos)
        self.left_ee_pose_pub = self.create_publisher(Float64MultiArray, '/left_arm/ee_pose', qos)
        self.right_ee_pose_pub = self.create_publisher(Float64MultiArray, '/right_arm/ee_pose', qos)

        # Latched lifecycle state. TRANSIENT_LOCAL so late-joining
        # subscribers (e.g. the Monitor UI opened after the controller is up)
        # immediately receive the last value, rather than racing the next
        # state transition. Monitor uses this to gate the Stop Teleop button
        # on real controller state (see the LIFECYCLE_* constants at top).
        lifecycle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lifecycle_pub = self.create_publisher(
            Int8, '/tianji_arm/lifecycle_state', lifecycle_qos)
        self._publish_lifecycle(LIFECYCLE_INITIALIZING)

        # Services.
        self.create_service(SetBool, '~/set_enabled', self._set_enabled_callback)
        self.create_service(SetBool, '/tianji_arm/set_arm_state', self._set_arm_state_callback)
        self.create_service(SetBool, '~/left/brake', self._left_brake_cb)
        self.create_service(SetBool, '~/right/brake', self._right_brake_cb)
        self.create_service(Trigger, '~/arm_status', self._arm_status_cb)
        self.create_service(SetBool, '~/left/clear_error', self._left_clear_error_cb)
        self.create_service(SetBool, '~/right/clear_error', self._right_clear_error_cb)

        # Control loop + state-publish: two timers (rates read from ROS2 params; see top of __init__).
        self.create_timer(1.0 / self._control_rate_hz, self._control_loop)
        self.create_timer(1.0 / self._state_publish_rate_hz, self._state_publish_loop)

        self.get_logger().info(
            f"Init complete: control_rate={self._control_rate_hz:.1f}Hz, "
            f"state_publish_rate={self._state_publish_rate_hz:.1f}Hz"
        )

    # -------------------- Latched lifecycle state --------------------

    def _publish_lifecycle(self, state: int):
        """Push the latched lifecycle state. Safe to call before the
        publisher exists; silently no-ops in that case."""
        pub = getattr(self, '_lifecycle_pub', None)
        if pub is None:
            return
        msg = Int8()
        msg.data = int(state)
        try:
            pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f"lifecycle_state publish skipped: {exc}")

    # -------------------- SDK exception handling --------------------

    def _on_sdk_error(self, context: str, error: Exception):
        """On SDK exception, auto-disable the arm to avoid a deadlocked state.
        Respawn recovers automatically."""
        self.get_logger().error(f"[{context}] SDK exception: {error}; auto-disabling arm")
        self._arm_enabled = False
        self._arm_state = ArmState.IMPEDANCE
        self.left_pose = None
        self.right_pose = None
        self._publish_lifecycle(LIFECYCLE_SDK_ERROR)
        try:
            self.controller.set_standby()
        except Exception as e2:
            # Error-recovery path; do not re-raise (would clobber the outer
            # timer callback). Must still log: SDK is already faulted and
            # set_standby also failing means a real hardware fault.
            self.get_logger().error(
                f"[{context}] set_standby also failed during recovery: {e2} — "
                f"SDK may be wedged, awaiting respawn"
            )

    # -------------------- Enable / disable --------------------

    def _do_enable(self):
        """Bring the arms from STANDBY to teleop-ready.

        Sequence:
            1. set_position_mode (rigid joint control) — required so
               move_to_init can actually drive the joints to the target;
               impedance K=2 is too soft for a 100°+ traverse in 3 s and
               leaves residuals of 120° (real failure observed 2026-06-11
               on tianji_arm_controller @ 192.168.1.190; Marvin replied
               cur_state=3 but the joints sat where they were).
            2. move_to_init quintic blend — accurate now that the joints
               are rigid.
            3. set_impedance_mode (the joint K/D from the README) — this
               is the mode the teleop loop runs in.

        Idempotent under _enable_lock: if already enabled, returns
        immediately without re-publishing lifecycle.
        """
        with self._enable_lock:
            if self._arm_enabled:
                return
            self._publish_lifecycle(LIFECYCLE_ENABLING)
            # Position mode with FULL velocity/acceleration ratio so the joints
            # actually reach init pose in the allotted window. Earlier defaults
            # (velRatio=30 / duration=3 / timeout=10) only moved ~40° per joint;
            # init pose differs by 120° on J1 / J4 — far past 40° — so the
            # residual check failed and the operator's Stop got gated forever.
            # quintic-blend duration 8s + 15s settle covers the worst case
            # (~170° single-joint traverse at this rate).
            self.controller.set_position_mode(velRatio=100, AccRatio=100)
            result = self.controller.move_to_init(wait=True, timeout=15, duration=8.0)
            if result is False:
                self.get_logger().error("move_to_init failed or timed out")
                raise RuntimeError("move_to_init failed")
            self.controller.set_impedance_mode(mode='joint')
            self._arm_enabled = True
            self._arm_state = ArmState.IMPEDANCE
            self._arm_handoff_snapshot()
            self._publish_lifecycle(LIFECYCLE_READY)

    def _do_disable(self):
        """Standby (servo-off, keep connection). Idempotent under _enable_lock."""
        with self._enable_lock:
            self._arm_enabled = False
            self._arm_state = ArmState.IMPEDANCE
            self.left_pose = None
            self.right_pose = None
            self._publish_lifecycle(LIFECYCLE_DISABLED)
            self.controller.set_standby()

    def _auto_enable_once(self):
        """One-shot timer callback: run _do_enable asynchronously. A failure
        here leaves the node up (operator can retry via ~/set_enabled) but
        publishes LIFECYCLE_ENABLE_FAILED so the Monitor can re-enable Stop
        Teleop instead of waiting forever for a READY signal that never
        comes."""
        # Cancel + destroy the timer first so a slow _do_enable doesn't get
        # re-entered by the next tick.
        if self._auto_enable_timer is not None:
            try:
                self._auto_enable_timer.cancel()
                self.destroy_timer(self._auto_enable_timer)
            except Exception:
                pass
            self._auto_enable_timer = None
        try:
            self._do_enable()
            self.get_logger().info("Arm auto-enabled; servos on")
        except Exception as exc:
            self.get_logger().error(
                f"Auto-enable failed: {exc}. Node stays up; call ~/set_enabled to retry."
            )
            self._publish_lifecycle(LIFECYCLE_ENABLE_FAILED)

    def _arm_handoff_snapshot(self):
        """Snapshot current hardware joints to seed the handoff ramp.

        After move_to_init the arm is at init pose; the operator's tracker is
        in some arbitrary position. Without a ramp the first IK frame would
        snap the arm directly to the operator's pose. Snapshot here, then
        _teleop_control blends from the snapshot to live IK over
        handoff_hold_sec + handoff_ramp_sec.
        """
        if self._handoff_hold_sec <= 0.0 and self._handoff_ramp_sec <= 0.0:
            self._handoff_start_at = None
            return
        try:
            left, right = self.controller.get_current_joints()
            self._handoff_start_left = list(left) if left is not None else None
            self._handoff_start_right = list(right) if right is not None else None
            self._handoff_start_at = time.monotonic()
            self.get_logger().info(
                f"Handoff ramp armed: hold={self._handoff_hold_sec:.2f}s, "
                f"ramp={self._handoff_ramp_sec:.2f}s"
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Handoff snapshot failed ({exc}); first IK frame will go straight to hardware"
            )
            self._handoff_start_at = None

    def _set_enabled_callback(self, request: SetBool.Request, response: SetBool.Response):
        """ROS2 service: remote enable/disable. Fast-path based on SDK
        cur_state; do NOT trust the _arm_enabled flag (it can drift from
        hardware). _do_enable / _do_disable own the _enable_lock, so this
        callback doesn't need to grab it itself."""
        if request.data:
            try:
                left_s, right_s = self.controller.get_arm_states_only()
            except Exception as e:
                self.get_logger().error(f"Failed to read hardware state (full enable path): {e}")
                left_s, right_s = -1, -1
            if left_s == 3 and right_s == 3:
                self._arm_enabled = True
                response.success = True
                response.message = "Already enabled (hardware in state=3)"
                return response
            try:
                self.get_logger().info("Enabling arms: set_active + move_to_init")
                self._do_enable()
                self.get_logger().info("Arms enabled, move_to_init complete")
                response.success = True
                response.message = "Arms enabled, move_to_init complete"
            except Exception as e:
                self.get_logger().error(f"Enable failed: {e}")
                self._arm_enabled = False
                self._publish_lifecycle(LIFECYCLE_ENABLE_FAILED)
                response.success = False
                response.message = f"Arm enable failed: {e}"
        else:
            try:
                self._do_disable()
            except Exception as e:
                self.get_logger().error(f"set_standby error: {e}")
                response.success = False
                response.message = f"set_standby failed: {e}"
                return response
            self.get_logger().info("Arms stopped")
            response.success = True
            response.message = "Arms stopped"
        return response

    # -------------------- Brake (release / hold) --------------------

    def _left_brake_cb(self, request, response):
        return self._brake_cb('A', request, response)

    def _right_brake_cb(self, request, response):
        return self._brake_cb('B', request, response)

    def _brake_cb(self, arm, request, response):
        side = 'left' if arm == 'A' else 'right'
        try:
            if request.data:
                self.controller.release_brake(arm)
                response.message = f"{side} arm: brake released"
            else:
                self.controller.hold_brake(arm)
                response.message = f"{side} arm: brake held"
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"{side} arm brake op failed: {e}"
        return response

    # -------------------- Clear error --------------------

    def _left_clear_error_cb(self, request, response):
        return self._clear_error_cb('A', request, response)

    def _right_clear_error_cb(self, request, response):
        return self._clear_error_cb('B', request, response)

    def _clear_error_cb(self, arm, request, response):
        side = 'left' if arm == 'A' else 'right'
        try:
            self.controller.clear_arm_error(arm)
            response.success = True
            response.message = f"{side} arm errors cleared"
        except Exception as e:
            response.success = False
            response.message = f"{side} arm clear-error failed: {e}"
        return response

    def _arm_status_cb(self, request, response):
        """ROS2 service: read both arms' state code / error code / servo error code."""
        try:
            import json
            status = self.controller.get_arm_status()
            response.success = True
            response.message = json.dumps(status)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # -------------------- Service callbacks --------------------

    def _set_arm_state_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Switch arm control state:
            data=true  -> POSITION   (state=1)
            data=false -> IMPEDANCE  (state=3)

        Fast-path based on SDK cur_state; do NOT trust _arm_state / _arm_enabled
        flags (they can drift from hardware).
        """
        target_state = 1 if request.data else 3
        target_enum = ArmState.POSITION if request.data else ArmState.IMPEDANCE
        _NAMES = {1: "rigid (position)", 3: "compliant (impedance)"}

        try:
            left_s, right_s = self.controller.get_arm_states_only()
        except Exception as e:
            response.success = False
            response.message = f"Failed to read hardware state, cannot determine mode: {e}"
            return response

        if left_s == target_state and right_s == target_state:
            self._arm_state = target_enum
            response.success = True
            response.message = f"Already in {_NAMES[target_state]} mode (cur_state={target_state})"
            return response

        if left_s == 0 or right_s == 0:
            response.success = False
            response.message = (
                f"Arms not enabled (left state={left_s}, right state={right_s}); "
                f"call enable_all first"
            )
            return response

        try:
            # Clear any accumulated errors/alarms before switching (mirrors set_active).
            robot = self.controller.robot
            robot.clear_set()
            robot.clear_error('A')
            robot.clear_error('B')
            robot.send_cmd()
            time.sleep(0.5)

            if target_state == 1:
                self.controller.set_position_mode(velRatio=30, AccRatio=30)
            else:
                self.controller.set_impedance_mode(mode='joint')
        except Exception as e:
            self._on_sdk_error("set_arm_state", e)
            response.success = False
            response.message = f"Arm communication error: {e}"
            return response

        self._arm_state = target_enum
        self.get_logger().info(f"Arms switched to {_NAMES[target_state]} mode")
        response.success = True
        response.message = _NAMES[target_state]
        return response

    # -------------------- Control loop --------------------

    def _state_publish_loop(self):
        """500Hz: publish joint_states only."""
        self._publish_state()

    def _control_loop(self):
        """120Hz: TF -> IK -> hardware. State publish is handled by a separate timer."""
        if not self._arm_enabled:
            return
        self._teleop_control()

    def _teleop_control(self):
        """Look up TF -> local IK -> hardware."""
        # Query TF
        left_tf = self._lookup_transform("left_chest", "tianji_left")
        if left_tf is not None:
            self.left_pose = self._matrix_to_pose(left_tf)

        right_tf = self._lookup_transform("right_chest", "tianji_right")
        if right_tf is not None:
            self.right_pose = self._matrix_to_pose(right_tf)

        # Upper-arm direction
        left_arm_tf = self._lookup_transform("left_chest", "left_arm")
        if left_arm_tf is not None:
            self.left_y_axis = left_arm_tf[:3, 1]

        right_arm_tf = self._lookup_transform("right_chest", "right_arm")
        if right_arm_tf is not None:
            self.right_y_axis = right_arm_tf[:3, 1]

        # Log every 3 seconds.
        self._log_counter += 1
        if self._log_counter >= 300:
            self._log_counter = 0
            self.get_logger().info(
                f"TF: left={'OK' if self.left_pose is not None else 'None'}, "
                f"right={'OK' if self.right_pose is not None else 'None'}")

        # Run IK.
        if self.left_pose is not None or self.right_pose is not None:
            # Update null-space parameters.
            if self.left_y_axis is not None:
                self.controller.left_zsp_para = [*self.left_y_axis, 0, 0, 0]
            if self.right_y_axis is not None:
                self.controller.right_zsp_para = [*self.right_y_axis, 0, 0, 0]

            blend_s = self._handoff_blend_factor()  # None = ramp finished, otherwise 0..1
            try:
                if blend_s is None:
                    # Steady-state teleop: single SDK call, IK + dispatch fused.
                    _, _, l_joints, r_joints = self.controller.move_to_pose_direct(
                        left_pose=self.left_pose, right_pose=self.right_pose, unit='m')
                else:
                    # Handoff window: dry-run IK, blend with snapshot, then dispatch.
                    _, _, l_target, r_target = self.controller.move_to_pose_direct(
                        left_pose=self.left_pose, right_pose=self.right_pose,
                        unit='m', send=False)
                    l_joints = self._blend_joints(self._handoff_start_left, l_target, blend_s)
                    r_joints = self._blend_joints(self._handoff_start_right, r_target, blend_s)
                    if l_joints is not None or r_joints is not None:
                        self.controller.move_to_joints_direct(
                            left_joints=l_joints, right_joints=r_joints)
                self._publish_command(l_joints, r_joints)
            except Exception as e:
                self._on_sdk_error("teleop_control", e)
                return

        # Publish null-space params and end-effector poses (per-arm; no need for both online).
        self._publish_zsp_para_and_pose()

    def _handoff_blend_factor(self) -> Optional[float]:
        """Return the IK-weight for the handoff ramp, or None if the ramp is done.

        0.0 = full hold at snapshot; 1.0 = full live IK target. During the hold
        phase we return 0.0 (hardware tracks the snapshot); during the ramp we
        smoothstep from 0 to 1; after the ramp we clear state and return None
        so the steady-state fused path takes over.
        """
        if self._handoff_start_at is None:
            return None
        elapsed = time.monotonic() - self._handoff_start_at
        ramp_end = self._handoff_hold_sec + self._handoff_ramp_sec
        if elapsed >= ramp_end:
            self._handoff_start_at = None
            self._handoff_start_left = None
            self._handoff_start_right = None
            self.get_logger().info("Handoff ramp complete; full-rate teleop engaged")
            return None
        if elapsed < self._handoff_hold_sec or self._handoff_ramp_sec <= 0.0:
            return 0.0
        t = (elapsed - self._handoff_hold_sec) / self._handoff_ramp_sec
        # Quintic smoothstep (same shape as move_to_joints_smooth in the driver).
        return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5

    @staticmethod
    def _blend_joints(start: Optional[list], target: Optional[list],
                      s: float) -> Optional[list]:
        """Return start*(1-s) + target*s; preserve target if start is missing,
        and preserve start if target is missing (lets a per-side TF dropout
        still hold its snapshot instead of going None)."""
        if target is None and start is None:
            return None
        if start is None:
            return list(target)
        if target is None:
            return list(start)
        return [start[i] + s * (target[i] - start[i]) for i in range(len(start))]

    # -------------------- Publish --------------------

    def _publish_command(self, left: Optional[list], right: Optional[list]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "left_base_cmd"
            msg.name = [f'left_joint_{i+1}' for i in range(7)]
            msg.position = list(left)
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "right_base_cmd"
            msg.name = [f'right_joint_{i+1}' for i in range(7)]
            msg.position = list(right)
            self.right_cmd_pub.publish(msg)

    def _publish_state(self):
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            self._state_error_count = 0  # reset on success
            self._last_left_state_joints = list(left_joints) if left_joints is not None else None
            self._last_right_state_joints = list(right_joints) if right_joints is not None else None
            stamp = self.get_clock().now().to_msg()

            if left_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "left_base_state"
                msg.name = [f'left_joint_{i+1}' for i in range(7)]
                msg.position = list(left_joints)
                self.left_state_pub.publish(msg)

            if right_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "right_base_state"
                msg.name = [f'right_joint_{i+1}' for i in range(7)]
                msg.position = list(right_joints)
                self.right_state_pub.publish(msg)
        except Exception as e:
            self._state_error_count += 1
            # Only escalate to SDK fault while the arm is still enabled (avoid infinite loop after disable).
            if self._arm_enabled and self._state_error_count >= 50:
                self._on_sdk_error("publish_state", e)
                self._state_error_count = 0

    def _publish_zsp_para_and_pose(self):
        """Publish null-space parameters and end-effector poses."""
        try:
            # --- left_zsp_para ---
            # try/except guards against the controller being torn down mid-read.
            if hasattr(self.controller, 'left_zsp_para') and self.controller.left_zsp_para is not None:
                raw_data = self.controller.left_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    # Coerce to list[float] for DDS compatibility.
                    msg.data = [float(x) for x in raw_data]
                    self.left_zsp_para_pub.publish(msg)

            # --- right_zsp_para ---
            if hasattr(self.controller, 'right_zsp_para') and self.controller.right_zsp_para is not None:
                raw_data = self.controller.right_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_data]
                    self.right_zsp_para_pub.publish(msg)

            # --- left_pose ---
            if self.left_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.left_pose]
                self.left_ee_pose_pub.publish(msg)

            # --- right_pose ---
            if self.right_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.right_pose]
                self.right_ee_pose_pub.publish(msg)

        except Exception as e:
            # During shutdown, a freed C++ object can still appear briefly.
            # Log only; do not crash the node.
            self.get_logger().warn(f"Failed to publish debug info during shutdown: {e}")

    # -------------------- TF utilities --------------------

    def _lookup_transform(self, from_frame: str, to_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            return self._transform_to_matrix(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> np.ndarray:
        """4x4 matrix -> [x, y, z, RX, RY, RZ] (degrees)."""
        xyz = matrix[:3, 3]
        rpy = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        if hasattr(self, 'controller'):
            self.controller.disable_and_release()
        self.get_logger().info("Exited cleanly")


# -------------------- Entry point --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tianji arm controller")
    parser.add_argument("-c", "--config", help="Config file path")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "tianji_arm_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    # Load config.
    config_path = args.config or get_package_config_path("tianji_output", "tianji_chest.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    node = None
    try:
        node = TianjiArmControllerNode(robot_ip=config.get("robot_ip", "192.168.1.190"))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # KeyboardInterrupt: direct SIGINT to a non-rclpy code path.
        # ExternalShutdownException: rclpy's documented signal-shutdown signal.
        # RCLError: same signal, race window — rclpy's SIGINT handler
        # invalidated the context mid-iteration and spin's next WaitSet
        # tripped on "context is not valid". All three mean "exit cleanly".
        pass
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception as e:
                print(f"shutdown() raised: {e}", file=sys.stderr)
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
