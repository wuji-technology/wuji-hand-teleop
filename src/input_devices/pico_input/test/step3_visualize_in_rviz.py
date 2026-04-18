#!/usr/bin/env python3
"""
=============================================================================
Step 3: RViz Coordinate System Visualization
=============================================================================

Function: Read robot joint angles -> publish TF -> RViz displays coordinate frames (no robot control)

Warning: Cannot run simultaneously with step1/step2 (port conflict)

Usage:
  # Terminal 1: Start this script
  python3 step3_visualize_in_rviz.py

  # Terminal 2: Manually start RViz
  rviz2

  # Configure in RViz:
  #   Fixed Frame: world
  #   Add -> TF -> Check Show Axes & Show Names

Test methods:
  1. Independent test: start step3 -> manually move robot arm -> observe RViz updates
  2. Alternating test: run step1 test -> stop -> start step3 -> observe pose

Coordinate frame architecture:
  world (base)
    ├─ world_left (left arm base, Y=+0.2m)
    │    └─ left_dh_ee (left arm end-effector, real-time update)
    └─ world_right (right arm base, Y=-0.2m)
         └─ right_dh_ee (right arm end-effector, real-time update)

Parameter descriptions:
  Dual-arm base orientation: rotated ±90 degrees around X axis
    LEFT_QUAT  = [0.7071, 0, 0,  0.7071]
    RIGHT_QUAT = [0.7071, 0, 0, -0.7071]

  Dual-arm base position: Y axis ±0.2m, spacing 40cm
    LEFT_POS  = [0.0, +0.2, 0.0]
    RIGHT_POS = [0.0, -0.2, 0.0]

=============================================================================
"""
import sys
import signal
import atexit
from pathlib import Path
import rclpy
from rclpy.node import Node

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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
from tianji_world_output.cartesian_controller import CartesianController

# Import shared modules (using relative path)
_test_dir = str(Path(__file__).resolve().parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
from common.robot_config import WORLD_TO_CHEST_TRANS, ROBOT_IP
from common.transform_utils import get_tf_quaternion


# ==================== World -> Arm Base coordinate transform (using unified config)====================
# Data source: world_to_chest_trans from tianji_robot.yaml
# Follows ROS REP 103 standard: +X forward, +Y left, +Z up
#
# Note: TF requires chest->world direction quaternion, use get_tf_quaternion() to obtain


class RobotAxisVisualizer(Node):
    def __init__(self):
        super().__init__('robot_axis_visualizer')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Connect to robot
        self.get_logger().info('Connecting to robot...')
        try:
            self.controller = CartesianController(robot_ip=ROBOT_IP)
            self.get_logger().info('Connected to robot')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            raise

        # Publish static robot arm base coordinate frames
        self.publish_static_frames()

        # Periodically read and publish robot arm poses
        self.timer = self.create_timer(0.1, self.update_robot_pose)  # 10Hz

        self.get_logger().info('Started publishing TF, please check in RViz')
        self.get_logger().info('  Fixed Frame: world')
        self.get_logger().info('  Frames: world -> world_left/world_right -> left_dh_ee/right_dh_ee')
        self.get_logger().info('  Add → TF → Show Axes & Names')

    def publish_static_frames(self):
        """Publish static frames: world -> world_left/world_right"""
        now = self.get_clock().now()
        transforms = []

        # Get quaternions for TF (chest->world direction)
        left_tf_quat = get_tf_quaternion('left')
        right_tf_quat = get_tf_quaternion('right')

        # world → world_left
        t_left = TransformStamped()
        t_left.header.stamp = now.to_msg()
        t_left.header.frame_id = 'world'
        t_left.child_frame_id = 'world_left'
        t_left.transform.translation.x = float(WORLD_TO_CHEST_TRANS['left'][0])
        t_left.transform.translation.y = float(WORLD_TO_CHEST_TRANS['left'][1])
        t_left.transform.translation.z = float(WORLD_TO_CHEST_TRANS['left'][2])
        t_left.transform.rotation.x = float(left_tf_quat[0])
        t_left.transform.rotation.y = float(left_tf_quat[1])
        t_left.transform.rotation.z = float(left_tf_quat[2])
        t_left.transform.rotation.w = float(left_tf_quat[3])
        transforms.append(t_left)

        # world → world_right
        t_right = TransformStamped()
        t_right.header.stamp = now.to_msg()
        t_right.header.frame_id = 'world'
        t_right.child_frame_id = 'world_right'
        t_right.transform.translation.x = float(WORLD_TO_CHEST_TRANS['right'][0])
        t_right.transform.translation.y = float(WORLD_TO_CHEST_TRANS['right'][1])
        t_right.transform.translation.z = float(WORLD_TO_CHEST_TRANS['right'][2])
        t_right.transform.rotation.x = float(right_tf_quat[0])
        t_right.transform.rotation.y = float(right_tf_quat[1])
        t_right.transform.rotation.z = float(right_tf_quat[2])
        t_right.transform.rotation.w = float(right_tf_quat[3])
        transforms.append(t_right)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Static frames published: world -> world_left, world_right')

    def update_robot_pose(self):
        """
        Read current pose from robot and publish TF

        Transform chain (ROS REP 103 compliant):
        ===================================
        world (robot base, +X forward, +Y left, +Z up)
          ↓
        world_left / world_right (arm base, static)
          ↓
        left_dh_ee / right_dh_ee (DH end-effector, dynamically computed from FK)

        Notes:
        - FK output is the DH end-effector pose in the arm base coordinate system
        - Directly published as world_left -> left_dh_ee TF
        - Clear naming: consistent with step4
        """
        try:
            # ==================== Step 1: Read joint angles ====================
            joints = self.controller.get_current_joints()
            left_joints = joints[0]
            right_joints = joints[1]

            # ==================== Step 2: Compute FK ====================
            # FK output is the DH end-effector pose in the arm base coordinate system
            left_fk_mat = self.controller.kine_left.fk(0, left_joints)
            right_fk_mat = self.controller.kine_right.fk(1, right_joints)

            # ==================== Step 3: Extract position and orientation ====================
            # Position (millimeters -> meters)
            left_pos = np.array([left_fk_mat[0][3], left_fk_mat[1][3], left_fk_mat[2][3]]) / 1000.0
            right_pos = np.array([right_fk_mat[0][3], right_fk_mat[1][3], right_fk_mat[2][3]]) / 1000.0

            # Orientation (rotation matrix -> quaternion)
            left_rot = np.array([
                [left_fk_mat[0][0], left_fk_mat[0][1], left_fk_mat[0][2]],
                [left_fk_mat[1][0], left_fk_mat[1][1], left_fk_mat[1][2]],
                [left_fk_mat[2][0], left_fk_mat[2][1], left_fk_mat[2][2]],
            ])
            left_quat = R.from_matrix(left_rot).as_quat()

            right_rot = np.array([
                [right_fk_mat[0][0], right_fk_mat[0][1], right_fk_mat[0][2]],
                [right_fk_mat[1][0], right_fk_mat[1][1], right_fk_mat[1][2]],
                [right_fk_mat[2][0], right_fk_mat[2][1], right_fk_mat[2][2]],
            ])
            right_quat = R.from_matrix(right_rot).as_quat()

            # ==================== Step 4: Publish TF ====================
            now = self.get_clock().now()
            transforms = []

            # world_left → left_dh_ee
            t_left = TransformStamped()
            t_left.header.stamp = now.to_msg()
            t_left.header.frame_id = 'world_left'
            t_left.child_frame_id = 'left_dh_ee'
            t_left.transform.translation.x = float(left_pos[0])
            t_left.transform.translation.y = float(left_pos[1])
            t_left.transform.translation.z = float(left_pos[2])
            t_left.transform.rotation.x = float(left_quat[0])
            t_left.transform.rotation.y = float(left_quat[1])
            t_left.transform.rotation.z = float(left_quat[2])
            t_left.transform.rotation.w = float(left_quat[3])
            transforms.append(t_left)

            # world_right → right_dh_ee
            t_right = TransformStamped()
            t_right.header.stamp = now.to_msg()
            t_right.header.frame_id = 'world_right'
            t_right.child_frame_id = 'right_dh_ee'
            t_right.transform.translation.x = float(right_pos[0])
            t_right.transform.translation.y = float(right_pos[1])
            t_right.transform.translation.z = float(right_pos[2])
            t_right.transform.rotation.x = float(right_quat[0])
            t_right.transform.rotation.y = float(right_quat[1])
            t_right.transform.rotation.z = float(right_quat[2])
            t_right.transform.rotation.w = float(right_quat[3])
            transforms.append(t_right)

            self.tf_broadcaster.sendTransform(transforms)

        except Exception as e:
            self.get_logger().error(f'Failed to update pose: {e}', throttle_duration_sec=5.0)

def main(args=None):
    global _node, _shutdown_requested

    rclpy.init(args=args)
    _node = RobotAxisVisualizer()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        if not _shutdown_requested:
            _cleanup()

if __name__ == '__main__':
    main()
