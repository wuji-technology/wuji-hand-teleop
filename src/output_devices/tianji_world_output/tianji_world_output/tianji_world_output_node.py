"""
ROS2 node that subscribes to pose topics and controls Tianji arms (Simplified).

Data Flow (Simplified):
  pico_input -> /left_arm_target_pose -> tianji_world_output -> IK -> Robot
  pico_input -> /right_arm_target_pose -> tianji_world_output -> IK -> Robot

This node:
  1. Subscribes to target pose topics (geometry_msgs/PoseStamped)
  2. Sends IK commands to Tianji arms
  3. No TF tree dependency - direct topic communication

Advantages:
  - Simple and direct data flow
  - No TF latency or complexity
  - Easy to debug and maintain
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

from tianji_world_output.cartesian_controller import CartesianController
from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge

# Log directory
LOG_DIR = Path.home() / ".wuji_teleop_logs"

# Canonical QoS for the arm joint streams (joint_states + joint_commands).
# BEST_EFFORT / KEEP_LAST depth=1 is the system-wide convention for high-rate
# joint topics — it mirrors the HTC side's controller.common.get_default_qos()
# and what wuji_teleop_monitor/ui/qos_utils.py documents subscribers expect.
# Why it matters here:
#   - These topics are an *echo* for the Monitor and the episode recorder; the
#     real arm command goes straight through the SDK, so dropping the odd echo
#     frame is harmless.
#   - RELIABLE would let a lagging subscriber stall the publish stream
#     (head-of-line blocking) and inject timestamp jitter into recorded
#     episodes — exactly the data-quality failure we want to avoid.
#   - depth>1 just buffers stale frames nobody wants (latest-wins stream).
# Subscribers auto-adapt via match_publisher_qos, so this one constant is the
# single knob that pins the contract for both PICO arm streams, identical to
# HTC's. Keep PICO and HTC on the same policy so recorded data is poolable.
ARM_JOINT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class TianjiWorldOutputNode(Node):
    """ROS2 node that subscribes to pose topics and controls Tianji arms (Simplified)."""

    def __init__(self, robot_ip: str = '192.168.1.190'):
        super().__init__("tianji_world_output")

        # Install stdlib->ROS2 logging bridge (routes non-Node class logs to /rosout)
        setup_ros2_logging_bridge(self.get_logger())

        # Parameters
        self.declare_parameter('control_rate', 90.0)
        self.declare_parameter('vel_ratio', 60)
        self.declare_parameter('acc_ratio', 60)

        control_rate = self.get_parameter('control_rate').value
        vel_ratio = int(self.get_parameter('vel_ratio').value)
        acc_ratio = int(self.get_parameter('acc_ratio').value)

        # Dedicated detailed log file (always enabled, line-buffered, prevents Ctrl+C truncation)
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._detail_log_path = LOG_DIR / f'tianji_output_{ts}.log'
        self._detail_log = None
        try:
            self._detail_log = open(self._detail_log_path, 'w', buffering=1)  # line buffering
            self._detail_log.write(f"# Tianji World Output detailed log - {ts}\n")
            self._detail_log.write(f"# IK status + dual-arm pose + joint angles + zsp_para\n")
            self.get_logger().info(f'Detailed log: {self._detail_log_path}')
        except OSError as e:
            self.get_logger().error(f'Cannot create detailed log file: {e}')

        # Create controller
        logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self.get_logger().info(f"Connecting to robot at {robot_ip}...")
        self.controller = CartesianController(robot_ip=robot_ip, logger=logger_adapter)
        self.controller.set_impedance_mode(mode='joint')

        # Set velocity/acceleration
        if vel_ratio != 60 or acc_ratio != 60:
            self.get_logger().info(f"Setting velocity ratio: vel={vel_ratio}%, acc={acc_ratio}%")
            self.controller.robot.clear_set()
            self.controller.robot.set_vel_acc(arm='A', velRatio=vel_ratio, AccRatio=acc_ratio)
            self.controller.robot.set_vel_acc(arm='B', velRatio=vel_ratio, AccRatio=acc_ratio)
            self.controller.robot.send_cmd()
            time.sleep(0.3)

        self.controller.move_to_init(wait=True, timeout=3)

        # Store pose and direction (input is already in chest coordinate frame, no conversion needed)
        self.left_arm_pose = None
        self.right_arm_pose = None
        # Initialize default elbow direction — loaded uniformly from tianji_robot.yaml (Single Source of Truth)
        config = TianjiConfig.load()
        self.left_arm_direction = config.get_default_zsp_direction('left')
        self.right_arm_direction = config.get_default_zsp_direction('right')
        self.get_logger().info(
            f"Loaded default arm angles from config: left={self.left_arm_direction}, right={self.right_arm_direction}"
        )

        # Subscribe to target pose topics
        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            '/left_arm_target_pose',
            self.left_pose_callback,
            10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            '/right_arm_target_pose',
            self.right_pose_callback,
            10
        )

        # Subscribe to elbow direction topics (optional)
        self.left_elbow_sub = self.create_subscription(
            Vector3Stamped,
            '/left_arm_elbow_direction',
            self.left_elbow_callback,
            10
        )
        self.right_elbow_sub = self.create_subscription(
            Vector3Stamped,
            '/right_arm_elbow_direction',
            self.right_elbow_callback,
            10
        )

        # Joint-state publishers — match the HTC tianji_arm_node's contract
        # (`/{side}_arm/joint_states`) so the Monitor's joint panel can read
        # real-time arm joint angles regardless of which arm controller is
        # running. QoS pinned by ARM_JOINT_QOS to the same policy as HTC.
        self.left_state_pub = self.create_publisher(
            JointState, '/left_arm/joint_states', ARM_JOINT_QOS)
        self.right_state_pub = self.create_publisher(
            JointState, '/right_arm/joint_states', ARM_JOINT_QOS)

        # Joint-command publishers — mirror the HTC tianji_arm_node contract
        # (`/{side}_arm/joint_commands`, frame_id `{side}_base_cmd`). PICO
        # input only carries a Cartesian wrist pose; the IK that turns it
        # into joint angles happens here, so this is the only place on the
        # PICO path that knows the commanded joint angles. Publishing them
        # makes the PICO path expose the same joint_commands topic as the
        # HTC path, so the Monitor's arm Cmd Hz can count real commands
        # instead of proxying off /{side}_arm_target_pose.
        self.left_cmd_pub = self.create_publisher(
            JointState, '/left_arm/joint_commands', ARM_JOINT_QOS)
        self.right_cmd_pub = self.create_publisher(
            JointState, '/right_arm/joint_commands', ARM_JOINT_QOS)

        # Null-space (elbow / arm-angle) echo publishers — mirror the HTC
        # tianji_arm_node `/{side}_arm/zsp_para` contract (Float64MultiArray).
        # PICO already does live elbow control (the direction is computed from
        # the arm-tracker geometry by pico_input and arrives on
        # /{side}_arm_elbow_direction, then feeds IK every control cycle); this
        # only echoes the constraint that was actually applied, so the Monitor
        # and the episode recorder see the same zsp_para field the HTC path
        # exposes. Closes the last cheap field-set gap between the two paths.
        self.left_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/zsp_para', ARM_JOINT_QOS)
        self.right_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/zsp_para', ARM_JOINT_QOS)

        # Control loop
        control_period = 1.0 / control_rate
        self.timer = self.create_timer(control_period, self.control_loop)

        # Joint-state publish loop — 100 Hz (matches Marvin's published rate
        # tier and is plenty for a Monitor UI preview without saturating
        # DDS). HTC side runs 500 Hz on a separate timer for inference
        # consumption; PICO doesn't have that downstream user so 100 Hz is
        # fine.
        self._joint_publish_period = 0.01
        self.joint_publish_timer = self.create_timer(
            self._joint_publish_period, self._publish_joint_state)

        self.get_logger().info("Tianji World Output node initialized (Topic-based, no TF).")
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /left_arm_target_pose")
        self.get_logger().info("  - /right_arm_target_pose")
        self.get_logger().info("  - /left_arm_elbow_direction (optional)")
        self.get_logger().info("  - /right_arm_elbow_direction (optional)")

        # First message flag (for debugging)
        self._first_pose_received = False

        # Diagnostic log counter (once per second)
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    def left_pose_callback(self, msg: PoseStamped):
        """Left arm target pose callback (input is already in chest coordinate frame)"""
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.left_arm_pose = self._pose_to_matrix(msg.pose)

    def right_pose_callback(self, msg: PoseStamped):
        """Right arm target pose callback (input is already in chest coordinate frame)"""
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("First pose data received, starting control...")
        self.right_arm_pose = self._pose_to_matrix(msg.pose)

    def left_elbow_callback(self, msg: Vector3Stamped):
        """Left arm elbow direction callback"""
        new_dir = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        # Log significant changes (>5 deg) to detailed log
        dot = np.clip(np.dot(new_dir, self.left_arm_direction), -1.0, 1.0)
        angle_change = np.degrees(np.arccos(dot))
        if angle_change > 5.0:
            self._write_detail_log(
                f"[ZSP change] Arm A: [{self.left_arm_direction[0]:.3f},{self.left_arm_direction[1]:.3f},{self.left_arm_direction[2]:.3f}]"
                f" -> [{new_dir[0]:.3f},{new_dir[1]:.3f},{new_dir[2]:.3f}] delta={angle_change:.1f} deg"
            )
        self.left_arm_direction = new_dir

    def right_elbow_callback(self, msg: Vector3Stamped):
        """Right arm elbow direction callback"""
        new_dir = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        # Log significant changes (>5 deg) to detailed log
        dot = np.clip(np.dot(new_dir, self.right_arm_direction), -1.0, 1.0)
        angle_change = np.degrees(np.arccos(dot))
        if angle_change > 5.0:
            self._write_detail_log(
                f"[ZSP change] Arm B: [{self.right_arm_direction[0]:.3f},{self.right_arm_direction[1]:.3f},{self.right_arm_direction[2]:.3f}]"
                f" -> [{new_dir[0]:.3f},{new_dir[1]:.3f},{new_dir[2]:.3f}] delta={angle_change:.1f} deg"
            )
        self.right_arm_direction = new_dir

    @staticmethod
    def _make_arm_joint_state(stamp, side: str, joints, frame_id: str) -> JointState:
        """Build one 7-DoF arm JointState message.

        Single constructor shared by both arm streams — measured
        (joint_states) and IK-commanded (joint_commands). They differ only
        in frame_id and which joint source feeds them; the naming
        (`{side}_joint_1..7`) and Marvin native unit (degrees) are
        identical by construction, which is exactly what lets the Monitor
        compare command vs. state. Keep the two streams defined here so
        they can never silently drift apart.
        """
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = [f'{side}_joint_{i + 1}' for i in range(7)]
        msg.position = list(joints)
        return msg

    def _publish_joint_state(self) -> None:
        """Publish measured arm joint angles on /{left,right}_arm/joint_states.

        Mirrors the HTC tianji_arm_node contract so the Monitor's joint
        panel (which subscribes to those two topics regardless of which
        controller is running) shows live values under the PICO path too.
        Source is the SDK readback (get_current_joints); frame_id marks
        these as measured state. Falls through silently on SDK errors —
        joint preview is a UX nicety, not a hard dependency.
        """
        try:
            left_joints, right_joints = self.controller.get_current_joints()
        except Exception:
            return
        stamp = self.get_clock().now().to_msg()
        if left_joints is not None:
            self.left_state_pub.publish(
                self._make_arm_joint_state(stamp, 'left', left_joints, 'left_base_state'))
        if right_joints is not None:
            self.right_state_pub.publish(
                self._make_arm_joint_state(stamp, 'right', right_joints, 'right_base_state'))

    def _publish_joint_command(self, left_joints, right_joints) -> None:
        """Publish IK-solved commanded joint angles on /{left,right}_arm/joint_commands.

        Same angles just sent to set_joint_cmd_pose, so this stream is the
        PICO-path equivalent of HTC's joint_commands. A None side means IK
        produced no command this tick (no pose yet, or solve failed), so
        that side is skipped — matching the HTC None semantics. frame_id
        marks these as command, distinguishing them from joint_states.
        """
        stamp = self.get_clock().now().to_msg()
        if left_joints is not None:
            self.left_cmd_pub.publish(
                self._make_arm_joint_state(stamp, 'left', left_joints, 'left_base_cmd'))
        if right_joints is not None:
            self.right_cmd_pub.publish(
                self._make_arm_joint_state(stamp, 'right', right_joints, 'right_base_cmd'))

    def _publish_zsp_para(self) -> None:
        """Echo the applied null-space (elbow) parameters on /{side}_arm/zsp_para.

        Mirrors HTC tianji_arm_node._publish_zsp_para_and_pose: publishes the
        same 6-element [dir_x, dir_y, dir_z, 0, 0, 0] vector that was just fed
        to IK this tick. Source is controller.{left,right}_zsp_para, set in
        control_loop right before the IK call. A falsy/empty side is skipped.
        """
        for zsp, pub in (
            (getattr(self.controller, 'left_zsp_para', None), self.left_zsp_para_pub),
            (getattr(self.controller, 'right_zsp_para', None), self.right_zsp_para_pub),
        ):
            if zsp:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in zsp]
                pub.publish(msg)

    def control_loop(self) -> None:
        """Main control loop: send control commands"""

        # Send control commands
        if self.left_arm_pose is not None or self.right_arm_pose is not None:
            # Update zsp_para (elbow arm angle control) - always use current elbow direction
            self.controller.left_zsp_para = [
                self.left_arm_direction[0],
                self.left_arm_direction[1],
                self.left_arm_direction[2],
                0, 0, 0
            ]

            self.controller.right_zsp_para = [
                self.right_arm_direction[0],
                self.right_arm_direction[1],
                self.right_arm_direction[2],
                0, 0, 0
            ]

            # Execute IK and send commands
            l_success, r_success, l_joints, r_joints = self.controller.move_to_pose_direct(
                left_pose=self.left_arm_pose,
                right_pose=self.right_arm_pose,
                unit='matrix'  # Use 4x4 matrix
            )

            # Publish the commanded joint angles the IK just produced. This
            # is the PICO-path equivalent of HTC's joint_commands stream:
            # same angles that were sent to set_joint_cmd_pose, so the
            # Monitor (and any future inference consumer) sees identical
            # command semantics regardless of which arm controller is live.
            self._publish_joint_command(l_joints, r_joints)

            # Echo the null-space constraint applied this tick (set above),
            # matching HTC's /{side}_arm/zsp_para so both paths expose the
            # same recorded field set.
            self._publish_zsp_para()

            # Print combined diagnostic info once per second (dual-arm pose + joints + zsp_para)
            self._debug_counter += 1
            if self._debug_counter >= self._debug_interval:
                self._debug_counter = 0
                self._log_control_status(l_success, r_success, l_joints, r_joints)

    @staticmethod
    def _pose_to_matrix(pose) -> np.ndarray:
        """Convert geometry_msgs/Pose to 4x4 transform matrix"""
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return T

    def _write_detail_log(self, msg: str) -> None:
        """Write to detailed log file (line-buffered, auto-flushed to OS buffer)"""
        if self._detail_log:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self._detail_log.write(f"{ts} {msg}\n")

    def _log_control_status(self, l_success, r_success, l_joints, r_joints) -> None:
        """Output combined diagnostic log once per second (dual-arm pose + joint angles + zsp_para)

        Output format:
          IK: A=True B=True | zsp A=[0.00,-0.89,-0.45] B=[0.00,0.89,-0.45]
            A: pos=[0.582,0.226,0.270] euler=[99.1,84.0,97.5] deg j=[55.0,-65.0,-70.0,-60.0,60.0,0.0,0.0]
            B: pos=[0.573,-0.224,0.276] euler=[...] deg j=[-55.0,-65.0,70.0,-60.0,-60.0,0.0,0.0]
        """
        ld = self.left_arm_direction
        rd = self.right_arm_direction

        # Line 1: IK status + zsp_para
        line1 = (
            f"IK: A={l_success} B={r_success} | "
            f"zsp A=[{ld[0]:.2f},{ld[1]:.2f},{ld[2]:.2f}] "
            f"B=[{rd[0]:.2f},{rd[1]:.2f},{rd[2]:.2f}]"
        )
        self.get_logger().info(line1)

        # Line 2: Arm A (left) pose + joint angles
        a_line = "  A:"
        if self.left_arm_pose is not None:
            lp = self.left_arm_pose[:3, 3]
            le = R.from_matrix(self.left_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            a_line += f" pos=[{lp[0]:.3f},{lp[1]:.3f},{lp[2]:.3f}] euler=[{le[0]:.1f},{le[1]:.1f},{le[2]:.1f}] deg"
        else:
            a_line += " no_pose"
        if l_joints:
            a_line += f" j=[{','.join(f'{j:.1f}' for j in l_joints)}]"
        elif not l_success and self.left_arm_pose is not None:
            a_line += " j=FAIL"
        self.get_logger().info(a_line)

        # Line 3: Arm B (right) pose + joint angles
        b_line = "  B:"
        if self.right_arm_pose is not None:
            rp = self.right_arm_pose[:3, 3]
            re_ = R.from_matrix(self.right_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            b_line += f" pos=[{rp[0]:.3f},{rp[1]:.3f},{rp[2]:.3f}] euler=[{re_[0]:.1f},{re_[1]:.1f},{re_[2]:.1f}] deg"
        else:
            b_line += " no_pose"
        if r_joints:
            b_line += f" j=[{','.join(f'{j:.1f}' for j in r_joints)}]"
        elif not r_success and self.right_arm_pose is not None:
            b_line += " j=FAIL"
        self.get_logger().info(b_line)

        # Write to detailed log file
        self._write_detail_log(line1)
        self._write_detail_log(a_line)
        self._write_detail_log(b_line)


def main(argv: list[str] | None = None) -> None:
    """Main entry point"""
    program_name = sys.argv[0] if sys.argv else "tianji_world_output_node"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(
        description="Tianji arm control node (Topic-based, no TF)"
    )
    parser.add_argument(
        "--robot-ip", default=None,
        help="Robot IP (default: from tianji_robot.yaml)",
    )
    args = parser.parse_args(cli_argv)

    # robot_ip: CLI > tianji_robot.yaml > default
    config = TianjiConfig.load()
    robot_ip = args.robot_ip or config.robot_ip

    rclpy.init(args=raw_argv)
    node = TianjiWorldOutputNode(robot_ip=robot_ip)

    # Shutdown flag to prevent duplicate disable_and_release calls
    _shutdown_done = False

    def _cleanup():
        """Clean up resources (close log + robot power off).

        A second SIGINT during cleanup used to abort disable_and_release
        mid-poll and leak the Marvin TCP session. Mask SIGINT for the
        duration so an impatient operator can't double-tap Ctrl-C past
        the safety power-off. SIGTERM stays unmasked because the new
        disable_and_release uses short 50ms time.sleep() chunks and has
        a 3s wall-clock cap; a SIGTERM that arrives during cleanup will
        be picked up on the next chunk boundary, which is fine.
        """
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True

        prev_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            # Close detailed log file
            if hasattr(node, '_detail_log') and node._detail_log:
                node._detail_log.write(f"# === Log ended ({datetime.now().strftime('%H:%M:%S')}) ===\n")
                node._detail_log.flush()
                node._detail_log.close()
                try:
                    node.get_logger().info(f'Detailed log saved: {node._detail_log_path}')
                except Exception:
                    print('Detailed log saved: %s' % node._detail_log_path)

            # Force flush all output
            sys.stdout.flush()
            sys.stderr.flush()

            # Robot power off (most critical step)
            try:
                node.controller.disable_and_release()
            except Exception as e:
                print(f'[WARNING] Robot power-off error: {e}', file=sys.stderr)
        finally:
            signal.signal(signal.SIGINT, prev_sigint)

    def _signal_handler(signum, frame):
        """SIGTERM/SIGINT signal handler: ensure safe robot power-off"""
        sig_name = signal.Signals(signum).name
        print(f'\n[{sig_name}] Received exit signal, shutting down safely...', file=sys.stderr)
        _cleanup()
        sys.exit(0)

    # SIGTERM: sent when ros2 launch tears down (e.g. Monitor's killpg
    # escalation after SIGINT timed out, or `kill <pid>` on the bringup
    # process). SIGINT during normal spin is handled by KeyboardInterrupt
    # in the try/except below — but if SIGTERM arrives we need this
    # explicit handler to enter the same cleanup path.
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
