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
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R

from tianji_world_output.cartesian_controller import CartesianController
from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge

# Log directory
LOG_DIR = Path.home() / ".wuji_teleop_logs"


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

        # Control loop
        control_period = 1.0 / control_rate
        self.timer = self.create_timer(control_period, self.control_loop)

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
    config = TianjiConfig.load(use_ros=False)
    robot_ip = args.robot_ip or config.robot_ip

    rclpy.init(args=raw_argv)
    node = TianjiWorldOutputNode(robot_ip=robot_ip)

    # Shutdown flag to prevent duplicate disable_and_release calls
    _shutdown_done = False

    def _cleanup():
        """Clean up resources (close log + robot power off)"""
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True

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

    def _signal_handler(signum, frame):
        """SIGTERM/SIGINT signal handler: ensure safe robot power-off"""
        sig_name = signal.Signals(signum).name
        print(f'\n[{sig_name}] Received exit signal, shutting down safely...', file=sys.stderr)
        _cleanup()
        sys.exit(0)

    # Register signal handler (SIGTERM: sent when launch shuts down)
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
