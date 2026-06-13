#!/usr/bin/env python3
"""
=============================================================================
Step 4: Visualize Recorded PICO Data (Offline Test, No Real Hardware Required)
=============================================================================

Function: Read PICO recorded data -> coordinate transform -> publish TF/Topics -> RViz visualization

✅ Advantages:
  - No PICO hardware required
  - No robot hardware required
  - Repeatable coordinate transform logic testing
  - Verify incremental control algorithm
  - Complete ROS2 node with static TF publishing

⚠️ Differences from other steps:
  - step3: Read joint angles from real robot -> FK -> visualization
  - step4: Read PICO data from recorded file -> coordinate transform -> visualization (this script)
  - step5: Read PICO data from recorded file -> coordinate transform -> control real robot


Usage
========

  Terminal 1: Start this script
  ------------------
  cd ~/ros2_ws/src/wuji-hand-teleop/src/input_devices/pico_input/test
  python3 step4_visualize_recorded_data.py

  # Use other recorded files (large datasets in record/ directory)
  python3 step4_visualize_recorded_data.py --file ../record/trackingData_head_tracker_static.txt

  # Adjust playback speed (0.5=slow, 1.0=real-time, 2.0=fast)
  python3 step4_visualize_recorded_data.py --speed 0.5

  # Loop playback
  python3 step4_visualize_recorded_data.py --loop


  Terminal 2: Start RViz visualization
  ------------------------
  rviz2

  Manually configure RViz:
    1. Fixed Frame: world
    2. Add -> TF -> Check Show Axes & Show Names
    3. Add -> Axes (optional, shows world coordinate system)


Test data files (all in record/ directory)
============
  High-quality static data (recommended):
    ../record/trackingData_sample_static.txt (150 frames, avg displacement <0.1mm)

  Large datasets:
    ../record/trackingData_head_tracker_static.txt (8384 frames)
    ../record/trackingData_head_tracker.txt (5703 frames)
    ../record/trackingData_whole_data.txt (5780 frames)


Coordinate Transform Description
============
PICO coordinate system -> Robot coordinate system:
  PICO: +XRight, +YUp, +ZForward(toward user)
  Robot: +XForward, +YLeft, +ZUp

  Transform matrix pico_to_robot (loaded from tianji_robot.yaml, see transform_utils.py):
    Robot X = -PICO Z  (User moves forward -> robot moves backward, toward user)
    Robot Y = -PICO X  (User moves right -> robot moves right)
    Robot Z = +PICO Y  (User moves up -> robot moves up)


Tracker SN Mapping
===============
  190058 → pico_left_wrist  (Left wrist, controls left arm end-effector)
  190600 → pico_right_wrist (Right wrist, controls right arm end-effector)
  190046 → pico_left_arm    (Left forearm, arm angle constraint)
  190023 → pico_right_arm   (Right forearm, arm angle constraint)


TF Tree Structure
=========
  world
  ├── head (HMD headset)
  ├── world_left (left arm chest coordinate system, Y=+0.2)
  │   ├── left_dh_ee (robotLeft arm end-effector reference position, static)
  │   ├── pico_left_wrist (PICO left wrist, dynamic)
  │   └── pico_left_arm (PICO left forearm, dynamic)
  │
  └── world_right (right arm chest coordinate system, Y=-0.2)
      ├── right_dh_ee (robotRight arm end-effector reference position, static)
      ├── pico_right_wrist (PICO right wrist, dynamic)
      └── pico_right_arm (PICO right forearm, dynamic)


Published Topics
=============
  /pico_hmd              - HMD pose (PoseStamped)
  /pico_left_wrist       - Left wrist pose (PoseStamped)
  /pico_right_wrist      - Right wrist pose (PoseStamped)
  /pico_left_arm         - Left forearm pose (PoseStamped)
  /pico_right_arm        - Right forearm pose (PoseStamped)
  /left_arm_target_pose  - Left arm target pose (PoseStamped)
  /right_arm_target_pose - Right arm target pose (PoseStamped)
  /left_arm_elbow_direction  - Left arm elbow direction (Vector3Stamped, arm angle constraint)
  /right_arm_elbow_direction - Right arm elbow direction (Vector3Stamped, arm angle constraint)


Debug Commands
========
  # View all PICO topics
  ros2 topic list | grep pico

  # View tracker data
  ros2 topic echo /pico_left_wrist --once
  ros2 topic echo /left_arm_elbow_direction --once

  # View TF tree
  ros2 run tf2_tools view_frames && evince frames.pdf


Verification Criteria
========
  ✓ 4 trackers correctly initialized (pico_left_wrist, pico_right_wrist, pico_left_arm, pico_right_arm)
  ✓ Topics publishing normally, data valid
  ✓ TF frames displaying correctly in RViz
  ✓ Axis directions correct (red-X-forward, green-Y-left, blue-Z-up)
  ✓ Arm angle direction /left_arm_elbow_direction publishing normally


Next Step
======
After passing tests, proceed to step5:
  python3 step5_incremental_control_with_robot.py --dry-run

=============================================================================
"""

import sys
import time
import argparse
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
            print("ROS2 node destroyed")
        except Exception:
            pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
            print("ROS2 shutdown")
    except Exception:
        pass


# Register cleanup on exit
atexit.register(_cleanup)

# Register signal handlers
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import shared modules (using relative path)
_test_dir = str(Path(__file__).resolve().parent)
_pico_input_dir = str(Path(__file__).resolve().parent.parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
if _pico_input_dir not in sys.path:
    sys.path.insert(0, _pico_input_dir)

from common.robot_config import (
    TIANJI_INIT_POS, TIANJI_INIT_ROT, WORLD_TO_CHEST_TRANS,
    DEFAULT_ZSP_DIRECTION, ARM_INIT_POS,
)
from common.transform_utils import (
    get_tf_quaternion,
    get_pico_to_robot,
    transform_pico_rotation_to_world,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
)

# Import data source
from pico_input.data_source import RecordedDataSource


# PICO -> Robot coordinate transform (loaded from shared config)
# See tianji_world_output/transform_utils.py and pico_input/ARCHITECTURE.md §3.1

# World -> Chest conversion: using shared functions from transform_utils
# (transform_world_to_chest, apply_world_rotation_to_chest_pose etc.)


class RecordedDataVisualizer(Node):
    """
    Recorded data visualization node (dual skeleton hierarchical visualization)

    Functions:
    - Publish static TF: world -> world_left, world_right (chest coordinate system)
    - Publish static TF: world_left/right -> robot DH joints (left_dh_ee, left_dh_elbow) - fixed parameters
    - Read recorded data (RecordedDataSource)
    - Compute PICO tracker incremental poses
    - Publish dynamic TF: world_left/right -> PICO trackers (pico_left_wrist, left_arm)
    - Publish topics: /pico_*, /left_arm_target_pose, /right_arm_target_pose

    Key design (two skeletons moving together):
    - Robot DH skeleton: world_left -> left_dh_ee, left_dh_elbow (static, TIANJI_INIT_POS/ELBOW_POS)
    - PICO skeleton: world_left -> pico_left_wrist, left_arm (dynamic, incremental control)
    - Shared parent frame (chest), axes naturally aligned for easy comparison and verification
    """

    def __init__(self, data_file: str, playback_speed: float = 1.0, loop_playback: bool = False):
        super().__init__('recorded_data_visualizer')

        # Parameters
        self.playback_speed = playback_speed
        self.loop_playback = loop_playback

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Topic publishers
        self.left_arm_pose_pub = self.create_publisher(PoseStamped, '/left_arm_target_pose', 10)
        self.right_arm_pose_pub = self.create_publisher(PoseStamped, '/right_arm_target_pose', 10)

        # PICO tracker topics (all topics use unified pico_ prefix)
        self.pico_hmd_pub = self.create_publisher(PoseStamped, '/pico_hmd', 10)
        self.pico_left_wrist_pub = self.create_publisher(PoseStamped, '/pico_left_wrist', 10)
        self.pico_right_wrist_pub = self.create_publisher(PoseStamped, '/pico_right_wrist', 10)
        self.pico_left_arm_pub = self.create_publisher(PoseStamped, '/pico_left_arm', 10)
        self.pico_right_arm_pub = self.create_publisher(PoseStamped, '/pico_right_arm', 10)

        # Arm angle control topics (for tianji_world_output nullspace IK constraints)
        # Publish elbow offset direction vector: normal vector of elbow relative to shoulder-wrist plane
        self.left_elbow_dir_pub = self.create_publisher(Vector3Stamped, '/left_arm_elbow_direction', 10)
        self.right_elbow_dir_pub = self.create_publisher(Vector3Stamped, '/right_arm_elbow_direction', 10)

        # RViz Marker visualization (for understanding arm angle calculation principles)
        self.marker_pub = self.create_publisher(MarkerArray, '/elbow_angle_visualization', 10)

        # Cache wrist positions for computing shoulder-wrist line
        self.wrist_positions = {'left': None, 'right': None}

        # Initialization state
        self.initialized = False
        self.init_tracker_poses = {}
        self.init_hmd_pose = None

        # Cache: for static data optimization (reduce RViz jitter)
        self.cached_poses = {}  # {role: (pos, quat)}
        self.debug_counter = 0  # For limiting log output

        # Data source
        self.get_logger().info(f'Loading recorded data: {data_file}')
        self.data_source = RecordedDataSource(data_file, playback_speed, loop_playback)

        if not self.data_source.initialize():
            self.get_logger().error('Data source initialization failed')
            raise RuntimeError('Failed to initialize data source')

        total_frames = len(self.data_source.data_frames) if hasattr(self.data_source, 'data_frames') else 0
        self.get_logger().info(f'Data loaded successfully, total {total_frames} frames')

        # Publish static TF
        self.publish_static_frames()

        # Auto-initialize (using first frame data)
        self.create_timer(0.1, self.auto_init_once)

        # Timer: publish TF and Topics
        self.timer = self.create_timer(1.0 / 90.0, self.publish_callback)  # 90 Hz

        self.get_logger().info('=' * 70)
        self.get_logger().info('Step 4: Recorded data visualization node started')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  Data file: {data_file}')
        self.get_logger().info(f'  Playback speed: {playback_speed}x')
        self.get_logger().info(f'  Loop playback: {loop_playback}')
        self.get_logger().info(f'  Total frames: {total_frames}')
        self.get_logger().info('-' * 70)
        self.get_logger().info('In another terminal, start RViz:')
        self.get_logger().info('  rviz2 -d $(ros2 pkg prefix pico_input)/share/pico_input/rviz/pico_simulation.rviz')
        self.get_logger().info('-' * 70)
        self.get_logger().info('Verification points:')
        self.get_logger().info('  ✓ Robot DH and PICO tracker both connected to world_left/right (chest)')
        self.get_logger().info('  ✓ Axes naturally aligned (shared parent frame)')
        self.get_logger().info('  ✓ Like two skeletons moving together: robot fixed + PICO following')
        self.get_logger().info('=' * 70)

    def publish_static_frames(self):
        """Publish static TF: world -> world_left, world_right, and robot DH joint positions"""
        now = self.get_clock().now()
        transforms = []

        # Get quaternions for TF (chest->world direction)
        left_tf_quat = get_tf_quaternion('left')
        right_tf_quat = get_tf_quaternion('right')

        # world → world_left (Y=+0.2)
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

        # world → world_right (Y=-0.2)
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

        # ========== Robot DH end-effector positions (only for RViz visualization reference)==========
        # Note: dh_elbow has been removed, because nullspace IK constraints use PICO arm tracker direction vectors
        # instead of fixed elbow positions. Elbow constraints are provided by /left_arm_elbow_direction topic.

        # Left arm rotation matrix
        left_ee_quat = R.from_matrix(TIANJI_INIT_ROT['left']).as_quat()

        # Right arm rotation matrix
        right_ee_quat = R.from_matrix(TIANJI_INIT_ROT['right']).as_quat()

        # world_left → left_dh_ee (Left arm end-effector reference position)
        t_left_ee = TransformStamped()
        t_left_ee.header.stamp = now.to_msg()
        t_left_ee.header.frame_id = 'world_left'
        t_left_ee.child_frame_id = 'left_dh_ee'
        t_left_ee.transform.translation.x = float(TIANJI_INIT_POS['left'][0])
        t_left_ee.transform.translation.y = float(TIANJI_INIT_POS['left'][1])
        t_left_ee.transform.translation.z = float(TIANJI_INIT_POS['left'][2])
        t_left_ee.transform.rotation.x = float(left_ee_quat[0])
        t_left_ee.transform.rotation.y = float(left_ee_quat[1])
        t_left_ee.transform.rotation.z = float(left_ee_quat[2])
        t_left_ee.transform.rotation.w = float(left_ee_quat[3])
        transforms.append(t_left_ee)

        # world_right → right_dh_ee (Right arm end-effector reference position)
        t_right_ee = TransformStamped()
        t_right_ee.header.stamp = now.to_msg()
        t_right_ee.header.frame_id = 'world_right'
        t_right_ee.child_frame_id = 'right_dh_ee'
        t_right_ee.transform.translation.x = float(TIANJI_INIT_POS['right'][0])
        t_right_ee.transform.translation.y = float(TIANJI_INIT_POS['right'][1])
        t_right_ee.transform.translation.z = float(TIANJI_INIT_POS['right'][2])
        t_right_ee.transform.rotation.x = float(right_ee_quat[0])
        t_right_ee.transform.rotation.y = float(right_ee_quat[1])
        t_right_ee.transform.rotation.z = float(right_ee_quat[2])
        t_right_ee.transform.rotation.w = float(right_ee_quat[3])
        transforms.append(t_right_ee)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Static TF published: world -> world_left, world_right')
        self.get_logger().info('Robot end-effector references published: left_dh_ee, right_dh_ee')

    def auto_init_once(self):
        """Auto-initialize (executed only once)"""
        if self.initialized:
            return

        # Get first frame data
        headset_data = self.data_source.get_headset_pose()
        tracker_data_list = self.data_source.get_tracker_data()

        if headset_data and headset_data.is_valid and tracker_data_list:
            self.do_init(headset_data, tracker_data_list)

    def do_init(self, headset_data, tracker_data_list):
        """Execute initialization: record HMD and tracker initial poses"""
        # Record HMD initial pose
        hmd_pose = headset_data.to_pose_array()
        self.init_hmd_pose = self._pose_to_matrix(hmd_pose)

        # Record tracker initial poses.
        # NOTE: example SNs only — replace with your own when replaying your
        # own recording. Production pico_input_node reads `tracker_serial_<sn>`
        # keys dynamically from pico_input.yaml; this dev tool inlines the map
        # to stay self-contained / runnable without a ROS param yaml.
        self.init_tracker_poses = {}
        tracker_map = {
            '190058': 'pico_left_wrist',
            '190600': 'pico_right_wrist',
            '190046': 'pico_left_arm',
            '190023': 'pico_right_arm',
        }

        self.get_logger().info('Initializing...')
        for tracker in tracker_data_list:
            # Extract 6-digit SN before the trailing 'G' — anchored so prefixes
            # ending in a digit don't fool the match (see fix 3b5fa8d).
            import re
            match = re.search(r'(\d{6})G?$', tracker.serial_number)
            if not match:
                continue

            short_sn = match.group(1)
            role = tracker_map.get(short_sn)

            if role and tracker.is_valid:
                pose = tracker.to_pose_array()
                self.init_tracker_poses[role] = self._pose_to_matrix(pose)
                self.get_logger().info(f'  {role}: pos={tracker.position}')

        self.initialized = True
        self.get_logger().info(f'Initialization complete! Recorded {len(self.init_tracker_poses)} trackers')

    def publish_callback(self):
        """Main loop: read data and publish TF + Topics"""
        if not self.data_source.is_available():
            return

        if not self.initialized:
            return

        now = self.get_clock().now()

        # Get data
        headset_data = self.data_source.get_headset_pose()
        tracker_data_list = self.data_source.get_tracker_data()

        if not headset_data or not headset_data.is_valid:
            return

        # Publish HMD TF
        self.publish_hmd_tf(now, headset_data)

        # Publish Tracker TF
        if tracker_data_list:
            self.publish_tracker_tfs(now, tracker_data_list)

    def publish_hmd_tf(self, now, headset_data):
        """Publish HMD TF"""
        hmd_pose = headset_data.to_pose_array()
        head_T = self._transform_hmd_to_world(hmd_pose)

        if head_T is not None:
            pos = head_T[:3, 3]
            quat = R.from_matrix(head_T[:3, :3]).as_quat()

            # Publish TF
            self._broadcast_tf(now, 'world', 'head', pos, quat)

            # Publish topic
            self._publish_pose(self.pico_hmd_pub, now, 'world', pos, quat)

    def publish_tracker_tfs(self, now, tracker_data_list):
        """
        Publish Tracker TF and Topics, including elbow direction for arm angle control

        ============================================================================
        Arm Angle Calculation Principle
        ============================================================================

        What is arm angle?
        -----------
        Arm angle describes the angle of elbow offset relative to the "shoulder-wrist plane".
        Imagine: when you extend your arm forward, the elbow can bend in different directions:
          - Elbow pointing down = relaxed elbow (natural relaxed posture)
          - Elbow pointing outward = raised elbow (like chicken wings)
          - Elbow pointing up = inverted elbow (acrobatic posture)

        Why is arm angle control needed?
        -------------------
        A 7-DOF robot arm has "redundant degrees of freedom", meaning there are infinitely many joint solutions for the same end-effector pose.
        Arm angle constraints can specify the desired elbow direction, letting the robot choose a specific posture.

        What is the nullspace?
        --------------------------
        For a 7-DOF robot arm, to reach a 6-DOF end-effector pose, 1 DOF is "redundant".
        This redundant DOF forms a "nullspace" -- motion within this space does not change the end-effector position.

        Imagine this scenario:
        - You grip a door handle with your hand (end-effector pose is fixed)
        - You can adjust your arm posture by moving your elbow, but your hand stays on the door handle
        - The trajectory the elbow can follow is the "nullspace"

        Arm angle control is selecting a specific elbow position within the nullspace.

        Computation Method
        --------
        1. Shoulder position: shoulder = [0, 0, 0] (chest coordinate system origin)
        2. Wrist position: wrist = position of pico_left_wrist
        3. Elbow position: elbow = position of pico_left_arm (approximate)

        4. Shoulder-wrist vector: shoulder_to_wrist = wrist - shoulder
        5. Shoulder-elbow vector: shoulder_to_elbow = elbow - shoulder

        6. Projection: elbow's projection point on shoulder-wrist line
           proj = (shoulder_to_elbow · shoulder_to_wrist_unit) * shoulder_to_wrist_unit

        7. Elbow offset vector: elbow_offset = shoulder_to_elbow - proj
           This is the direction of elbow deviation from the shoulder-wrist plane!

        8. Normalize: elbow_direction = elbow_offset / |elbow_offset|

        Visualization Notes (in RViz)
        ----------------------
        - Red arrow: shoulder-wrist line
        - Green arrow: shoulder-elbow line
        - Blue arrow: elbow offset direction (elbow_direction) -- this is sent to IK
        - Yellow dot: elbow projection point on shoulder-wrist line

        ============================================================================
        """
        # Replace SNs with your own if replaying a different recording — see
        # the note on the first tracker_map above for why this dev tool inlines.
        tracker_map = {
            '190058': ('pico_left_wrist', self.pico_left_wrist_pub, self.left_arm_pose_pub, 'left'),
            '190600': ('pico_right_wrist', self.pico_right_wrist_pub, self.right_arm_pose_pub, 'right'),
            '190046': ('pico_left_arm', self.pico_left_arm_pub, None, 'left'),
            '190023': ('pico_right_arm', self.pico_right_arm_pub, None, 'right'),
        }

        # Collect wrist and forearm positions for computing elbow direction
        wrist_positions = {}
        arm_positions = {}

        for tracker in tracker_data_list:
            if not tracker.is_valid:
                continue

            # Extract 6-digit SN before the trailing 'G' (see fix 3b5fa8d).
            import re
            match = re.search(r'(\d{6})G?$', tracker.serial_number)
            if not match:
                continue

            short_sn = match.group(1)
            if short_sn not in tracker_map:
                continue

            pico_frame_name, legacy_pub, target_pub, side = tracker_map[short_sn]

            if pico_frame_name not in self.init_tracker_poses:
                continue

            # Compute incremental pose
            pose = tracker.to_pose_array()
            pos, quat = self._compute_incremental_pose(pose, pico_frame_name, side)

            if pos is None:
                continue

            # Determine parent frame (chest coordinate system)
            parent_frame = 'world_left' if side == 'left' else 'world_right'

            # Publish TF (PICO tracker position, parent frame is chest)
            self._broadcast_tf(now, parent_frame, pico_frame_name, pos, quat)

            # Publish PICO topic
            self._publish_pose(legacy_pub, now, parent_frame, pos, quat)

            # Publish target pose topic (wrist only)
            if target_pub:
                self._publish_pose(target_pub, now, parent_frame, pos, quat)

            # Collect wrist and forearm positions for elbow direction computation
            if 'wrist' in pico_frame_name:
                wrist_positions[side] = pos.copy()
            elif 'arm' in pico_frame_name:
                arm_positions[side] = pos.copy()

        # ============== Compute and publish elbow direction ==============
        # Compute geometric direction from arm tracker position (pointing toward gravity/down), negate before publishing to IK (anti-gravity direction)
        # arm_init_pos Physical value [0.2, ±0.3, +0.1] places arm below shoulder, geometric direction points downward
        # Use DEFAULT_ZSP_DIRECTION as fallback when no tracker data available (already follows IK convention)

        markers = MarkerArray()
        marker_id = 0

        for side in ['left', 'right']:
            parent_frame = 'world_left' if side == 'left' else 'world_right'
            pub = self.left_elbow_dir_pub if side == 'left' else self.right_elbow_dir_pub

            wrist_pos = wrist_positions.get(side)
            elbow_pos = arm_positions.get(side)

            if wrist_pos is not None and elbow_pos is not None:
                shoulder_pos = np.array([0.0, 0.0, 0.0])  # Chest coordinate system origin

                direction, proj_point = self._compute_elbow_direction(
                    shoulder_pos, wrist_pos, elbow_pos, side
                )

                # Geometric direction (toward gravity) -> IK direction (anti-gravity): negate
                ik_direction = -direction
                self._publish_vector3(pub, now, parent_frame, ik_direction)

                # Create visualization markers (using physical direction, consistent with green shoulder-elbow arrow)
                side_markers = self._create_elbow_visualization_markers(
                    now, parent_frame, shoulder_pos, wrist_pos, elbow_pos,
                    proj_point, direction, side, marker_id
                )
                markers.markers.extend(side_markers)
                marker_id += len(side_markers)
            else:
                # Do not publish when no tracker data, output_node keeps using last received direction
                pass

        # Publish visualization markers
        if markers.markers:
            self.marker_pub.publish(markers)

    def _compute_elbow_direction(self, shoulder: np.ndarray, wrist: np.ndarray,
                                  elbow: np.ndarray, side: str) -> tuple:
        """
        Compute elbow offset direction relative to shoulder-wrist plane

        This is the core algorithm for arm angle control:
        1. Compute shoulder-wrist vector (arm main axis)
        2. Compute shoulder-elbow vector
        3. Project elbow onto shoulder-wrist line
        4. Elbow offset = actual elbow position - projection point
        5. Normalize to get direction vector

        ============================================================================
        Geometric Principle Diagram
        ============================================================================

                    Shoulder
                      ●
                     /|\\
                    / | \\
                   /  |  \\
                  /   |   \\
           shoulder-elbow /    |    \\ shoulder-wrist
                /     |     \\
               /      |      \\
              ●-------●-------●
           Elbow   Projection  Wrist
          (elbow) (proj)   (wrist)

              ←─────→
            Elbow offset vector
          (elbow_direction)

        Elbow offset vector = elbow position - projection point
        This vector is the physical direction of elbow deviation from shoulder-wrist line (pointing toward elbow/gravity direction).
        Negate before publishing to IK: ik_direction = -direction (IK requires anti-gravity direction).

        ============================================================================
        Left/Right Arm Handling
        ============================================================================

        The initial reference positions of arm trackers (arm_init_pos, from tianji_robot.yaml) are physically correct values:
          - left arm: [0.2, +0.3, +0.1]  (Left Chest: Y+=Down, Z+=Left/outer side)
          - right arm: [0.2, -0.3, +0.1]  (Right Chest: Y-=Down, Z+=Right/outer side)

        The computed direction is the physical direction (pointing to elbow), negated to IK direction before publishing.
        Differences between left and right arms have been handled in the position transform stage.

        Returns:
            direction: elbow offset direction (unit vector, physical direction, pointing to elbow)
            proj_point: elbow projection point on shoulder-wrist line (for visualization)
        """
        # Default arm angle reference plane parameters (loaded from unified config)
        default_direction = DEFAULT_ZSP_DIRECTION[side]

        # Shoulder-wrist vector
        shoulder_to_wrist = wrist - shoulder
        sw_length = np.linalg.norm(shoulder_to_wrist)

        if sw_length < 1e-6:
            # Wrist too close to shoulder, using default relaxed elbow direction
            return default_direction, shoulder

        sw_unit = shoulder_to_wrist / sw_length

        # Shoulder-elbow vector
        shoulder_to_elbow = elbow - shoulder

        # Projection length of elbow on shoulder-wrist line
        proj_length = np.dot(shoulder_to_elbow, sw_unit)

        # Projection point
        proj_point = shoulder + proj_length * sw_unit

        # Elbow offset vector (direction of elbow deviation from shoulder-wrist line)
        elbow_offset = elbow - proj_point
        offset_length = np.linalg.norm(elbow_offset)

        if offset_length < 1e-6:
            # Elbow is exactly on shoulder-wrist line, using default relaxed elbow direction
            # This case almost never occurs in human postures
            return default_direction, proj_point

        # Normalize to get direction vector
        direction = elbow_offset / offset_length

        return direction, proj_point

    def _create_elbow_visualization_markers(self, now, frame_id: str,
                                             shoulder: np.ndarray, wrist: np.ndarray,
                                             elbow: np.ndarray, proj_point: np.ndarray,
                                             direction: np.ndarray, side: str,
                                             start_id: int) -> list:
        """
        Create RViz Markers for arm angle visualization

        Display contents:
        - Red arrow: shoulder-wrist line (arm main axis)
        - Green arrow: shoulder-elbow line (actual elbow position)
        - Blue arrow: elbow offset direction (elbow_direction sent to IK)
        - Yellow sphere: projection point (elbow projection on shoulder-wrist line)
        - Purple dashed line: connection from elbow to projection point (offset distance)
        """
        markers = []
        timestamp = now.to_msg()

        # 1. Red arrow: shoulder -> wrist (arm main axis)
        m_sw = Marker()
        m_sw.header.stamp = timestamp
        m_sw.header.frame_id = frame_id
        m_sw.ns = f'{side}_arm_axis'
        m_sw.id = start_id
        m_sw.type = Marker.ARROW
        m_sw.action = Marker.ADD
        m_sw.points = [
            self._to_point(shoulder),
            self._to_point(wrist)
        ]
        m_sw.scale.x = 0.01  # Shaft diameter
        m_sw.scale.y = 0.02  # Arrow head diameter
        m_sw.scale.z = 0.0
        m_sw.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # red
        m_sw.lifetime.sec = 0
        m_sw.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_sw)

        # 2. Green arrow: shoulder -> elbow (actual elbow)
        m_se = Marker()
        m_se.header.stamp = timestamp
        m_se.header.frame_id = frame_id
        m_se.ns = f'{side}_shoulder_to_elbow'
        m_se.id = start_id + 1
        m_se.type = Marker.ARROW
        m_se.action = Marker.ADD
        m_se.points = [
            self._to_point(shoulder),
            self._to_point(elbow)
        ]
        m_se.scale.x = 0.01
        m_se.scale.y = 0.02
        m_se.scale.z = 0.0
        m_se.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # green
        m_se.lifetime.sec = 0
        m_se.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_se)

        # 3. Blue arrow: from projection point to elbow (elbow_direction)
        # This vector is the arm angle constraint direction sent to IK
        arrow_length = 0.15  # Fixed length for easy observation
        arrow_end = proj_point + direction * arrow_length
        m_dir = Marker()
        m_dir.header.stamp = timestamp
        m_dir.header.frame_id = frame_id
        m_dir.ns = f'{side}_elbow_direction'
        m_dir.id = start_id + 2
        m_dir.type = Marker.ARROW
        m_dir.action = Marker.ADD
        m_dir.points = [
            self._to_point(proj_point),
            self._to_point(arrow_end)
        ]
        m_dir.scale.x = 0.015  # Slightly thicker for emphasis
        m_dir.scale.y = 0.025
        m_dir.scale.z = 0.0
        m_dir.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # bright blue
        m_dir.lifetime.sec = 0
        m_dir.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_dir)

        # 4. Yellow sphere: projection point
        m_proj = Marker()
        m_proj.header.stamp = timestamp
        m_proj.header.frame_id = frame_id
        m_proj.ns = f'{side}_projection_point'
        m_proj.id = start_id + 3
        m_proj.type = Marker.SPHERE
        m_proj.action = Marker.ADD
        m_proj.pose.position = self._to_point(proj_point)
        m_proj.pose.orientation.w = 1.0
        m_proj.scale.x = 0.02
        m_proj.scale.y = 0.02
        m_proj.scale.z = 0.02
        m_proj.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow
        m_proj.lifetime.sec = 0
        m_proj.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_proj)

        # 5. Purple line: projection point -> elbow (shows offset distance)
        m_offset = Marker()
        m_offset.header.stamp = timestamp
        m_offset.header.frame_id = frame_id
        m_offset.ns = f'{side}_elbow_offset'
        m_offset.id = start_id + 4
        m_offset.type = Marker.LINE_STRIP
        m_offset.action = Marker.ADD
        m_offset.points = [
            self._to_point(proj_point),
            self._to_point(elbow)
        ]
        m_offset.scale.x = 0.008  # Line width
        m_offset.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # purple
        m_offset.lifetime.sec = 0
        m_offset.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_offset)

        # 6. Text label: shows side name
        m_text = Marker()
        m_text.header.stamp = timestamp
        m_text.header.frame_id = frame_id
        m_text.ns = f'{side}_label'
        m_text.id = start_id + 5
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position = self._to_point(elbow + np.array([0, 0, 0.05]))
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.03  # Text size
        m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # white
        m_text.text = f'{side.upper()} elbow'
        m_text.lifetime.sec = 0
        m_text.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_text)

        return markers

    def _to_point(self, arr: np.ndarray):
        """numpy array → geometry_msgs Point"""
        from geometry_msgs.msg import Point
        return Point(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    def _compute_incremental_pose(self, current_pose, pico_frame_name: str, side: str):
        """Compute PICO tracker incremental pose (for comparison and verification)"""
        if pico_frame_name not in self.init_tracker_poses:
            return None, None

        current_T = self._pose_to_matrix(current_pose)
        init_T = self.init_tracker_poses[pico_frame_name]

        # Position increment
        delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]
        delta_pos_norm = np.linalg.norm(delta_pos_pico)

        # Orientation increment
        init_rot = R.from_matrix(init_T[:3, :3])
        current_rot = R.from_matrix(current_T[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()
        delta_angle = np.linalg.norm(delta_rot_pico.as_rotvec())

        # Debug output: show increment magnitude for first 10 calls
        if self.debug_counter < 10:
            self.get_logger().info(
                f'[DEBUG] {pico_frame_name}: delta_pos={delta_pos_norm:.10f}m, delta_angle={delta_angle:.10f}rad'
            )
            self.debug_counter += 1

        # Stationary detection: if increment is very small (< 1e-6), use cached pose
        # This avoids RViz jitter caused by floating point errors
        POSITION_THRESHOLD = 1e-6  # 0.001mm
        ROTATION_THRESHOLD = 1e-6  # ~0.00006 degrees

        is_static = (delta_pos_norm < POSITION_THRESHOLD and delta_angle < ROTATION_THRESHOLD)

        if is_static and pico_frame_name in self.cached_poses:
            # Use cached pose (completely stationary)
            return self.cached_poses[pico_frame_name]

        # Compute new pose
        delta_pos_world = get_pico_to_robot() @ delta_pos_pico

        # Convert to chest coordinate system (using shared library)
        delta_pos_chest = transform_world_to_chest(delta_pos_world, side)

        # Get robot initial position
        # For wrist tracker: mapped to robot end-effector initial position
        # For arm tracker: kept near origin, for computing arm angle direction vector
        if 'arm' in pico_frame_name and 'wrist' not in pico_frame_name:
            # Forearm tracker: position used for arm angle direction computation (loaded from config)
            robot_init_pos = ARM_INIT_POS[side]
        else:
            # Wrist (pico_left_wrist, pico_right_wrist) uses TIANJI_INIT_POS
            robot_init_pos = TIANJI_INIT_POS.get(side, np.zeros(3))

        target_pos = robot_init_pos + delta_pos_chest

        # Convert to world (using shared library, axis-angle method)
        delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())

        # Using shared library 4-step algorithm: target_chest = R_w2c @ delta_world @ R_c2w @ init_chest
        robot_init_rot_mat = TIANJI_INIT_ROT.get(side, np.eye(3))
        target_rot_in_chest = apply_world_rotation_to_chest_pose(
            robot_init_rot_mat, delta_rot_world, side
        )

        target_rot = R.from_matrix(target_rot_in_chest)
        target_quat = target_rot.as_quat()

        # Cache pose
        self.cached_poses[pico_frame_name] = (target_pos, target_quat)

        return target_pos, target_quat

    def _transform_hmd_to_world(self, pico_pose) -> np.ndarray:
        """Convert HMD pose to world coordinate system"""
        if self.init_hmd_pose is None:
            return None

        current_T = self._pose_to_matrix(pico_pose)

        # Position increment
        delta_pos_pico = current_T[:3, 3] - self.init_hmd_pose[:3, 3]
        delta_pos_robot = get_pico_to_robot() @ delta_pos_pico

        # HMD initial position set above origin
        target_pos = np.array([0.0, 0.0, 1.6]) + delta_pos_robot

        # Orientation increment
        current_rot = R.from_matrix(current_T[:3, :3])
        init_rot = R.from_matrix(self.init_hmd_pose[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()

        delta_rot_robot = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())
        target_rot = delta_rot_robot

        result_T = np.eye(4)
        result_T[:3, 3] = target_pos
        result_T[:3, :3] = target_rot.as_matrix()
        return result_T

    def _pose_to_matrix(self, pose) -> np.ndarray:
        """[x,y,z,qx,qy,qz,qw] -> 4x4 transformation matrix"""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
        return T

    def _broadcast_tf(self, now, parent: str, child: str, pos, quat):
        """Broadcast TF"""
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_pose(self, publisher, now, frame_id: str, pos, quat):
        """Publish PoseStamped"""
        if publisher is None:
            return

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        publisher.publish(msg)

    def _publish_vector3(self, publisher, now, frame_id: str, vector):
        """Publish Vector3Stamped (for elbow direction)"""
        if publisher is None:
            return

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'data_source'):
            self.data_source.close()
            self.get_logger().info('Data source closed')
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Step 4: Visualize recorded PICO data')
    parser.add_argument('--file', type=str,
                        default='../record/trackingData_sample_static.txt',
                        help='Recorded data file path (relative to test/ directory)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (0.5=slow, 1.0=real-time, 2.0=fast)')
    parser.add_argument('--loop', action='store_true',
                        help='Loop playback')
    args = parser.parse_args()

    # Determine data file path
    test_dir = Path(__file__).parent
    data_file = Path(args.file)
    if not data_file.is_absolute():
        # Relative path, search from test/ directory
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f"Error: data file not found {data_file}")
        print(f"   Please check if the file exists at: {test_dir}")
        return

    print("=" * 70)
    print("Step 4: Visualize recorded PICO data")
    print("=" * 70)
    print(f"  Data file: {data_file}")
    print(f"  Playback speed: {args.speed}x")
    print(f"  Loop playback: {args.loop}")
    print("=" * 70)
    print("")
    print("In another terminal, start RViz:")
    print("  rviz2 -d $(ros2 pkg prefix pico_input)/share/pico_input/rviz/pico_simulation.rviz")
    print("")
    print("Or manually configure RViz:")
    print("  1. Fixed Frame: world")
    print("  2. Add -> TF -> Check Show Axes & Show Names")
    print("=" * 70)
    print("")

    global _node, _shutdown_requested

    rclpy.init()
    _node = RecordedDataVisualizer(str(data_file), args.speed, args.loop)

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        if not _shutdown_requested:
            _cleanup()


if __name__ == '__main__':
    main()
