#!/usr/bin/env python3
"""
=============================================================================
Step 2: ROS2 Topic Pose Control (Simplified Architecture)
=============================================================================

Test Purpose:
  Verify ROS2 topic communication and robot arm control

Input/Output:
  Input: Target pose (PoseStamped topic)
  Processing: tianji_world_output_node subscribes -> IK calculation -> joint angles
  Output: Send joint angle commands to the real robot

Architecture:
  step2_pose_topic_control.py (publishes topic)
      |
  /left_arm_target_pose, /right_arm_target_pose (PoseStamped)
      |
  tianji_world_output_node (subscribes topic -> IK -> real robot)

Usage:
  # Terminal 1: Start the arm control node
  cd ~/ros2_ws/src/wuji-hand-teleop
  source install/setup.bash
  ros2 launch wuji_teleop_bringup test_arm_control.launch.py

  # Terminal 2: Publish test poses
  cd src/input_devices/pico_input/test

  # Static (maintain initial pose)
  python3 step2_pose_topic_control.py --mode static --duration 5.0

  # Forward (world +X) - recommended parameters
  python3 step2_pose_topic_control.py --mode forward --speed 0.03 --duration 3.0

  # Backward (world -X)
  python3 step2_pose_topic_control.py --mode back --speed 0.03 --duration 3.0

  # Move left (world +Y)
  python3 step2_pose_topic_control.py --mode left --speed 0.02 --duration 3.0

  # Move right (world -Y)
  python3 step2_pose_topic_control.py --mode right --speed 0.02 --duration 3.0

  # Move up (world +Z)
  python3 step2_pose_topic_control.py --mode up --speed 0.02 --duration 3.0

  # Move down (world -Z)
  python3 step2_pose_topic_control.py --mode down --speed 0.02 --duration 3.0

  # Rotate around world X axis (Roll)
  python3 step2_pose_topic_control.py --mode rotate_x --speed 0.5 --duration 3.0

  # Rotate around world Y axis (Pitch)
  python3 step2_pose_topic_control.py --mode rotate_y --speed 0.5 --duration 3.0

  # Rotate around world Z axis (Yaw)
  python3 step2_pose_topic_control.py --mode rotate_z --speed 0.5 --duration 3.0

  # Auto-exits after motion completes

Requirements:
  ROS2 required
  TF not required (no coordinate transforms needed)
  Launch file required

Verification Results:
  ROS2 node starts successfully
  Topic communication works
  Robot arm follows topic motion
  Test script auto-exits

=============================================================================
"""

import argparse
import time
import sys
import signal
import atexit
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped

# Global variables for signal handling cleanup
_node = None
_shutdown_requested = False


def _signal_handler(signum, frame):
    """Signal handler to ensure proper cleanup on Ctrl+C and kill"""
    global _shutdown_requested
    if not _shutdown_requested:
        _shutdown_requested = True
        print("\nReceived exit signal, cleaning up...")
        _cleanup()
        sys.exit(0)


def _cleanup():
    """Cleanup function to ensure node is properly destroyed"""
    global _node
    if _node is not None:
        try:
            _node.destroy_node()
            _node = None
        except Exception:
            pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# Register cleanup on exit
atexit.register(_cleanup)

# Register signal handlers
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

# Import shared modules (using relative path)
_test_dir = str(Path(__file__).resolve().parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
from common.robot_config import (
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    WORLD_TO_LEFT_QUAT,
    WORLD_TO_RIGHT_QUAT,
    DEFAULT_ZSP_DIRECTION,
)
from common.transform_utils import (
    transform_world_to_chest,
    get_direction_vector_world,
    get_rotation_axis_world,
    apply_world_rotation_to_chest_pose,
)


class PosePublisher(Node):
    """Pose publishing node"""

    def __init__(self, mode: str, speed: float, duration: float):
        super().__init__('pose_publisher')

        self.mode = mode
        self.speed = speed
        self.duration = duration

        # Create publishers
        self.left_pose_pub = self.create_publisher(PoseStamped, '/left_arm_target_pose', 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/right_arm_target_pose', 10)
        self.left_elbow_pub = self.create_publisher(Vector3Stamped, '/left_arm_elbow_direction', 10)
        self.right_elbow_pub = self.create_publisher(Vector3Stamped, '/right_arm_elbow_direction', 10)

        # Rotation state (for accumulated rotation)
        self.current_rot = {
            'left': TIANJI_INIT_ROT['left'].copy(),
            'right': TIANJI_INIT_ROT['right'].copy()
        }

        # Wait for subscribers to be ready
        self.get_logger().info("Waiting for tianji_world_output_node subscribers...")
        wait_count = 0
        while self.left_pose_pub.get_subscription_count() == 0:
            time.sleep(0.5)
            wait_count += 1
            if wait_count % 2 == 0:
                self.get_logger().info(f"Waiting... ({wait_count * 0.5:.1f}s)")

        self.get_logger().info(f"Subscribers ready! Starting pose publishing - mode: {mode}, speed: {speed}, duration: {duration}s")

        # Record start time (after subscribers are ready)
        self.start_time = time.time()

        # Create timer (after subscribers are ready)
        self.timer = self.create_timer(0.01, self.publish_poses)  # 100Hz

    def publish_poses(self):
        """Publish poses"""
        elapsed = time.time() - self.start_time

        # Check if finished
        if elapsed > self.duration:
            self.get_logger().info("Motion complete!")
            self.timer.cancel()
            raise SystemExit(0)

        # Compute current position
        if self.mode == 'static':
            left_pos = TIANJI_INIT_POS['left'].copy()
            right_pos = TIANJI_INIT_POS['right'].copy()

        elif self.mode in ['forward', 'back', 'left', 'right', 'up', 'down']:
            direction_world = get_direction_vector_world(self.mode)
            displacement_world = direction_world * self.speed * elapsed

            left_disp = transform_world_to_chest(displacement_world, 'left')
            right_disp = transform_world_to_chest(displacement_world, 'right')

            left_pos = TIANJI_INIT_POS['left'] + left_disp
            right_pos = TIANJI_INIT_POS['right'] + right_disp

        elif self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
            # Rotation mode: use shared library apply_world_rotation_to_chest_pose
            left_pos = TIANJI_INIT_POS['left'].copy()
            right_pos = TIANJI_INIT_POS['right'].copy()

            # Compute rotation increment
            dt = 0.01  # Timer period
            rotation_angle = self.speed * dt  # speed as angular velocity (rad/s)

            # Get rotation axis (in world coordinate system)
            axis_world = get_rotation_axis_world(self.mode)

            # Use shared library to execute 4-step algorithm: target_chest = R_w2c @ R_delta @ R_c2w @ base_chest
            R_delta = R.from_rotvec(axis_world * rotation_angle)

            self.current_rot = {
                side: apply_world_rotation_to_chest_pose(
                    self.current_rot[side], R_delta, side
                )
                for side in ['left', 'right']
            }

        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        # Publish poses
        if self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
            # Rotation mode: use accumulated rotation
            self._publish_pose(self.left_pose_pub, 'world_left', left_pos, self.current_rot['left'])
            self._publish_pose(self.right_pose_pub, 'world_right', right_pos, self.current_rot['right'])
        else:
            # Position mode: use initial rotation
            self._publish_pose(self.left_pose_pub, 'world_left', left_pos, TIANJI_INIT_ROT['left'])
            self._publish_pose(self.right_pose_pub, 'world_right', right_pos, TIANJI_INIT_ROT['right'])

        # Publish arm angle reference plane parameters (loaded from unified config)
        self._publish_elbow_direction(self.left_elbow_pub, 'world_left', DEFAULT_ZSP_DIRECTION['left'])
        self._publish_elbow_direction(self.right_elbow_pub, 'world_right', DEFAULT_ZSP_DIRECTION['right'])

        # Progress display (once per second)
        if int(elapsed * 10) % 10 == 0:
            progress = int(elapsed / self.duration * 100)
            if self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
                # Rotation mode: print current orientation euler angles
                euler_left = R.from_matrix(self.current_rot['left']).as_euler('ZYX', degrees=True)
                self.get_logger().info(
                    f"Progress: {progress}% | Publishing euler=[{euler_left[0]:.1f}, {euler_left[1]:.1f}, {euler_left[2]:.1f}] deg"
                )
            else:
                self.get_logger().info(f"Progress: {progress}% - Position: [{left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}]")

    def _publish_pose(self, publisher, frame_id: str, pos: np.ndarray, rot: np.ndarray):
        """Publish pose message"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        quat = R.from_matrix(rot).as_quat()
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        publisher.publish(msg)

    def _publish_elbow_direction(self, publisher, frame_id: str, direction: np.ndarray):
        """Publish elbow direction"""
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # Normalize
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm

        msg.vector.x = float(direction[0])
        msg.vector.y = float(direction[1])
        msg.vector.z = float(direction[2])

        publisher.publish(msg)


def main():
    global _node, _shutdown_requested

    parser = argparse.ArgumentParser(description='Test pose publisher')
    parser.add_argument('--mode', type=str, default='static',
                        choices=['static', 'forward', 'back', 'left', 'right', 'up', 'down',
                                 'rotate_x', 'rotate_y', 'rotate_z'],
                        help='Test mode')
    parser.add_argument('--speed', type=float, default=0.02,
                        help='Movement speed (m/s) or rotation speed (rad/s)')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Duration (seconds)')

    args = parser.parse_args()

    rclpy.init()
    _node = PosePublisher(
        mode=args.mode,
        speed=args.speed,
        duration=args.duration
    )

    try:
        rclpy.spin(_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not _shutdown_requested:
            _cleanup()


if __name__ == '__main__':
    main()
