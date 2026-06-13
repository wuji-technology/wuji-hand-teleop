#!/usr/bin/env python3
"""
PICO Input Node - Reads VR tracking data from XRoboToolkit PC-Service

==================== Design Philosophy (Incremental Control Mode) ====================

Core idea:
  - Record initial poses of all trackers during initialization
  - Subsequently only publish the "delta" of tracker relative to initial pose
  - User can initialize in any pose, robot will not jump

Advantages:
  - Safest approach, initialization does not cause sudden robot movement
  - User can be sitting, standing, or in any pose
  - Users are advised to mimic the robot's pose for better control feel

Initialization process:
  1. User assumes a pose (recommended to mimic robot's current pose)
  2. Press initialization button (or wait for auto-initialization)
  3. System records HMD initial pose (visualization only)
  4. System records initial poses of all trackers
  5. Subsequently only publishes increments relative to initial poses

==================== Incremental Control Principle ====================

    During initialization:
      - Record user's hand pose in PICO coordinate system: init_tracker_poses[role]
      - Robot initial pose comes from tianji_robot.yaml (init_pos/init_rot)

    During runtime:
      - User's current hand pose: current_pose
      - Position increment: delta_pos = current_pos - init_pos
      - Orientation increment: delta_rot = current_rot * init_rot.inv()
      - Publish: robot target = robot initial + increment (with coordinate transform)

==================== TF Tree Structure (Unified Naming Convention) ====================

    world (robot base, fixed)
    +-- head (HMD, visualization only)
    +-- world_left (left arm chest coordinate system, static, Y=+0.2)
    |   +-- pico_left_wrist (PICO left wrist tracker, dynamic)
    |   +-- pico_left_arm (PICO left forearm tracker, dynamic)
    +-- world_right (right arm chest coordinate system, static, Y=-0.2)
        +-- pico_right_wrist (PICO right wrist tracker, dynamic)
        +-- pico_right_arm (PICO right forearm tracker, dynamic)

==================== Topics (Unified Naming Convention) ====================

    # PICO data (debug/visualization)
    /pico_hmd, /pico_left_wrist, /pico_right_wrist
    /pico_left_arm, /pico_right_arm

    # Robot control (subscribed by tianji_world_output)
    /left_arm_target_pose, /right_arm_target_pose
    /left_arm_elbow_direction, /right_arm_elbow_direction

==================== Usage ====================
    # Launch
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # Manual initialization (after user aligns with robot)
    ros2 service call /pico_input/init std_srvs/srv/Trigger

    # Reset initialization
    ros2 service call /pico_input/reset std_srvs/srv/Trigger
"""

import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

# Data source abstraction (TDD refactoring)
from pico_input.data_source import DataSource, LiveDataSource, RecordedDataSource, TrackerData, HeadsetData
from pico_input.incremental_controller import IncrementalController

# Unified config loader (via ROS2 package dependency: package.xml exec_depend)
from tianji_world_output.config_loader import get_config as get_tianji_config
from tianji_world_output.transform_utils import (
    get_tf_quaternion,
)
_tianji_config = get_tianji_config()

# The four physical tracker slots wired through the downstream pipeline.
# Each entry: yaml key -> role string the rest of the code uses.
# The yaml schema is fixed at exactly these four keys; the SN values are
# user-specific. See pico_input.yaml.
_TRACKER_SLOTS = {
    'tracker_sn_left_wrist':  'pico_left_wrist',
    'tracker_sn_right_wrist': 'pico_right_wrist',
    'tracker_sn_left_arm':    'pico_left_arm',
    'tracker_sn_right_arm':   'pico_right_arm',
}


class PicoInputNode(Node):
    """
    PICO Input Node (Incremental Control Mode + TDD Refactoring)

    Architecture:
    - IncrementalController: Pure computation (incremental pose, elbow direction, One-Euro adaptive smoothing)
    - PicoInputNode: ROS2 thin shell (data source, TF, Topics, services, visualization)

    Output:
    - TF: world -> head, pico_*_wrist, pico_*_arm
    - Topics: /left_arm_target_pose, /right_arm_target_pose
    - Topics: /pico/* (optional, debug only)
    """

    def __init__(self, data_source: Optional[DataSource] = None):
        super().__init__('pico_input_node')

        # Install stdlib->ROS2 logging bridge (routes non-Node class logs to /rosout)
        from pico_input.ros2_logging import setup_ros2_logging_bridge
        setup_ros2_logging_bridge(self.get_logger())

        # ==================== Parameter declarations ====================
        self.declare_parameter('publish_rate', 90.0)

        # Data source configuration (TDD refactoring addition)
        self.declare_parameter('data_source_type', 'live')  # 'live' or 'recorded'
        # Recorded mode is a developer-only replay tool; the open-source build
        # ships no sample tracking data, so users wanting to test 'recorded'
        # must point this parameter at their own captured file.
        self.declare_parameter('recorded_file_path', '')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop_playback', True)

        # PC-Service connection (used by LiveDataSource)
        self.declare_parameter('pc_service_host', '127.0.0.1')
        self.declare_parameter('pc_service_port', 60061)

        # Topic publishing configuration
        self.declare_parameter('enable_topic_publishing', True)
        self.declare_parameter('enable_legacy_topics', False)
        self.declare_parameter('topic_prefix', 'pico')

        # One-Euro Filter adaptive filtering parameters (loaded from pico_input.yaml)
        self.declare_parameter('one_euro_min_cutoff', 1.0)
        self.declare_parameter('one_euro_beta', 0.7)
        self.declare_parameter('elbow_min_cutoff', 0.3)

        # Initialization related parameters
        self.declare_parameter('auto_init_delay', 5.0)

        # Tracker SN slots (fixed schema, four entries — see _TRACKER_SLOTS).
        for slot_key in _TRACKER_SLOTS:
            self.declare_parameter(slot_key, '')

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.data_source_type = self.get_parameter('data_source_type').value
        self.pc_service_host = self.get_parameter('pc_service_host').value
        self.pc_service_port = self.get_parameter('pc_service_port').value
        self.enable_topic_publishing = self.get_parameter('enable_topic_publishing').value
        self.enable_legacy_topics = self.get_parameter('enable_legacy_topics').value
        self.topic_prefix = self.get_parameter('topic_prefix').value
        self.auto_init_delay = self.get_parameter('auto_init_delay').value

        # Build SN -> role lookup. yaml schema (_TRACKER_SLOTS) guarantees
        # the right four keys exist; we still have to catch empty strings
        # ("user didn't fill the placeholder") and duplicate SNs ("user
        # copy-pasted the same tracker into two slots").
        self.sn_to_role = {}
        for slot_key, role in _TRACKER_SLOTS.items():
            sn = self.get_parameter(slot_key).value
            if not sn or sn.startswith('YOUR_'):
                raise RuntimeError(
                    f"pico_input.yaml: {slot_key!r} is empty or a placeholder "
                    f"({sn!r}). Fill it with the full SN of the {role} tracker."
                )
            if sn in self.sn_to_role:
                raise RuntimeError(
                    f"pico_input.yaml: {slot_key!r} and the slot for "
                    f"{self.sn_to_role[sn]!r} both have SN {sn!r}. Each "
                    f"tracker SN must appear exactly once."
                )
            self.sn_to_role[sn] = role

        # ==================== Incremental controller (core computation logic) ====================
        # Pure computation class, no ROS2 dependency, can be unit tested independently
        min_cutoff = self.get_parameter('one_euro_min_cutoff').value
        beta = self.get_parameter('one_euro_beta').value
        elbow_min_cutoff = self.get_parameter('elbow_min_cutoff').value

        self.controller = IncrementalController(
            config=_tianji_config,
            rate=self.publish_rate,
            min_cutoff=min_cutoff,
            beta=beta,
            elbow_min_cutoff=elbow_min_cutoff,
        )
        self.get_logger().info(
            f"One-Euro Filter: rate={self.publish_rate}Hz, "
            f"min_cutoff={min_cutoff}, beta={beta}, elbow_min_cutoff={elbow_min_cutoff}"
        )
        self.init_start_time = None

        # Arm angle log counter
        self._elbow_log_counter = 0

        # ==================== Arm angle diagnostic log ====================
        self.get_logger().info('=' * 70)
        self.get_logger().info('[ELBOW DIAG] Arm angle initialization parameters:')
        for side in ['left', 'right']:
            wrist_pos = self.controller.robot_init_positions[f'pico_{side}_wrist']
            arm_pos = self.controller.robot_init_positions[f'pico_{side}_arm']
            default_dir = self.controller.default_zsp_direction[side]
            sw = wrist_pos
            sw_len = float(np.linalg.norm(sw))
            if sw_len > 1e-6:
                sw_unit = sw / sw_len
                proj = np.dot(arm_pos, sw_unit) * sw_unit
                offset = arm_pos - proj
                offset_len = float(np.linalg.norm(offset))
                dot_val = float(np.dot(offset / np.linalg.norm(offset), default_dir)) if offset_len > 1e-6 else 0.0
            else:
                offset_len = 0.0
                dot_val = 0.0
            self.get_logger().info(f'  [{side}] wrist_init_pos(chest): [{wrist_pos[0]:.4f}, {wrist_pos[1]:.4f}, {wrist_pos[2]:.4f}]')
            self.get_logger().info(f'  [{side}] arm_init_pos(chest):   [{arm_pos[0]:.4f}, {arm_pos[1]:.4f}, {arm_pos[2]:.4f}]')
            self.get_logger().info(f'  [{side}] sw_length: {sw_len:.4f}m, arm_offset: {offset_len:.4f}m')
            self.get_logger().info(f'  [{side}] default_zsp_dir: [{default_dir[0]:.4f}, {default_dir[1]:.4f}, {default_dir[2]:.4f}]')
            self.get_logger().info(f'  [{side}] dot(geom, default): {dot_val:.4f} {"(ALIGNED)" if dot_val > 0 else "(OPPOSITE!)"}')
        self.get_logger().info('=' * 70)

        # ==================== World -> Chest transform parameters ====================
        # Load world_to_chest transform from unified config (for static TF publishing)
        self.world_to_chest_trans = {
            'left': _tianji_config.world_to_chest_trans.get('left', np.array([0.0, 0.2, 0.0])),
            'right': _tianji_config.world_to_chest_trans.get('right', np.array([0.0, -0.2, 0.0])),
        }

        # ==================== TF broadcasters ====================
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static TF: world -> world_left, world_right
        self._publish_static_chest_frames()

        # ==================== Services ====================
        self.init_service = self.create_service(
            Trigger, 'pico_input/init', self._init_callback
        )
        self.reset_service = self.create_service(
            Trigger, 'pico_input/reset', self._reset_callback
        )

        # ==================== Topic publishers ====================
        # 1. Target pose publishing (tianji_world_output_node subscribes to these)
        self.left_arm_pose_pub = None
        self.right_arm_pose_pub = None
        self.left_elbow_dir_pub = None
        self.right_elbow_dir_pub = None
        if self.enable_topic_publishing:
            self.left_arm_pose_pub = self.create_publisher(
                PoseStamped, '/left_arm_target_pose', 10
            )
            self.right_arm_pose_pub = self.create_publisher(
                PoseStamped, '/right_arm_target_pose', 10
            )
            # Elbow direction (arm angle constraint)
            self.left_elbow_dir_pub = self.create_publisher(
                Vector3Stamped, '/left_arm_elbow_direction', 10
            )
            self.right_elbow_dir_pub = self.create_publisher(
                Vector3Stamped, '/right_arm_elbow_direction', 10
            )
            # Arm angle visualization Markers (display elbow direction arrows in RViz)
            self.marker_pub = self.create_publisher(
                MarkerArray, '/elbow_angle_visualization', 10
            )
            self.get_logger().info('Topic publishing enabled:')
            self.get_logger().info('  - /left_arm_target_pose, /right_arm_target_pose')
            self.get_logger().info('  - /left_arm_elbow_direction, /right_arm_elbow_direction')
            self.get_logger().info('  - /elbow_angle_visualization (RViz Markers)')

        # 2. Legacy topic publishing (debug only, /pico_*)
        self.hmd_pub = None
        self.tracker_pubs = {}
        if self.enable_legacy_topics:
            self.hmd_pub = self.create_publisher(PoseStamped, f'/{self.topic_prefix}_hmd', 10)
            # Use unified naming convention: pico_left_wrist, pico_right_wrist, pico_left_arm, pico_right_arm
            self.tracker_pubs['pico_left_wrist'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_left_wrist', 10)
            self.tracker_pubs['pico_right_wrist'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_right_wrist', 10)
            self.tracker_pubs['pico_left_arm'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_left_arm', 10)
            self.tracker_pubs['pico_right_arm'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_right_arm', 10)
            self.get_logger().info(f'Legacy topic publishing enabled: /{self.topic_prefix}_*')

        # ==================== Data source initialization (TDD refactoring) ====================
        # Use dependency injection or factory pattern to create data source
        if data_source is None:
            data_source = self._create_data_source()

        self.data_source = data_source

        # Initialize data source
        if self.data_source.initialize():
            self.get_logger().info(f'Data source initialized: {self.data_source}')
        else:
            self.get_logger().error(f'Data source initialization failed: {self.data_source}')

        # ==================== Warning throttle ====================
        self._last_warn_time = None
        self._warn_interval = 5.0  # Output warning at most once every 5 seconds
        self._no_data_count = 0
        self._waiting_for_device = True  # Waiting for PICO device connection

        # ==================== Timer ====================
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)

        self._log_startup()

    def _create_data_source(self) -> DataSource:
        """
        Factory method: Create data source based on parameters

        Returns:
            DataSource instance (LiveDataSource or RecordedDataSource)

        Raises:
            ValueError: If data_source_type is invalid
        """
        if self.data_source_type == 'live':
            self.get_logger().info('Creating LiveDataSource (real PICO SDK)')
            return LiveDataSource(self.pc_service_host, self.pc_service_port)

        elif self.data_source_type == 'recorded':
            file_path = self.get_parameter('recorded_file_path').value
            playback_speed = self.get_parameter('playback_speed').value
            loop_playback = self.get_parameter('loop_playback').value

            self.get_logger().info(f'Creating RecordedDataSource: {file_path} (speed {playback_speed}x)')
            return RecordedDataSource(file_path, playback_speed, loop_playback)

        else:
            raise ValueError(f'Unknown data_source_type: {self.data_source_type}')

    def _init_callback(self, request, response):
        """Service callback: initialization (incremental mode, using abstract data source)"""
        if not self.data_source.is_available():
            response.success = False
            response.message = 'Data source not available'
            return response

        # Get headset data from data source
        headset_data = self.data_source.get_headset_pose()
        if headset_data is None or not headset_data.is_valid:
            response.success = False
            response.message = 'HMD data not available'
            return response

        # Get tracker data from data source
        tracker_data_list = self.data_source.get_tracker_data()

        self._do_init(headset_data, tracker_data_list)
        response.success = True
        response.message = f'Initialization complete! Incremental control mode, recorded {len(self.controller.init_tracker_poses)} trackers'
        return response

    def _reset_callback(self, request, response):
        """Service callback: reset initialization"""
        self.controller.reset()
        self.init_start_time = None
        response.success = True
        response.message = 'Reset done, waiting for re-initialization'
        self.get_logger().warning('Initialization has been reset!')
        return response

    def _publish_static_chest_frames(self):
        """
        Publish static TF: world -> world_left, world_right

        These are the chest coordinate systems for the robot's left and right arms,
        serving as parent frames for PICO trackers.
        Maintains consistent TF tree structure with step3/step4.
        """
        now = self.get_clock().now()
        transforms = []

        # world -> world_left (left arm chest, Y=+0.2m)
        t_left = TransformStamped()
        t_left.header.stamp = now.to_msg()
        t_left.header.frame_id = 'world'
        t_left.child_frame_id = 'world_left'
        t_left.transform.translation.x = float(self.world_to_chest_trans['left'][0])
        t_left.transform.translation.y = float(self.world_to_chest_trans['left'][1])
        t_left.transform.translation.z = float(self.world_to_chest_trans['left'][2])
        # TF requires chest->world direction quaternion (conjugate of world_to_chest)
        tf_quat_left = get_tf_quaternion('left')
        t_left.transform.rotation.x = float(tf_quat_left[0])
        t_left.transform.rotation.y = float(tf_quat_left[1])
        t_left.transform.rotation.z = float(tf_quat_left[2])
        t_left.transform.rotation.w = float(tf_quat_left[3])
        transforms.append(t_left)

        # world -> world_right (right arm chest, Y=-0.2m)
        t_right = TransformStamped()
        t_right.header.stamp = now.to_msg()
        t_right.header.frame_id = 'world'
        t_right.child_frame_id = 'world_right'
        t_right.transform.translation.x = float(self.world_to_chest_trans['right'][0])
        t_right.transform.translation.y = float(self.world_to_chest_trans['right'][1])
        t_right.transform.translation.z = float(self.world_to_chest_trans['right'][2])
        tf_quat_right = get_tf_quaternion('right')
        t_right.transform.rotation.x = float(tf_quat_right[0])
        t_right.transform.rotation.y = float(tf_quat_right[1])
        t_right.transform.rotation.z = float(tf_quat_right[2])
        t_right.transform.rotation.w = float(tf_quat_right[3])
        transforms.append(t_right)

        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('Static TF published: world -> world_left, world_right')

    def _do_init(self, headset_data: HeadsetData, tracker_data_list: list):
        """Execute initialization: collect tracker poses, delegate to incremental controller"""
        hmd_pose_array = headset_data.to_pose_array()

        num_trackers = len(tracker_data_list)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Received {num_trackers} tracker data')

        # Log serial numbers (for debugging)
        serial_numbers = [t.serial_number for t in tracker_data_list]
        if serial_numbers:
            self.get_logger().info(f'  Serial numbers: {serial_numbers}')

        # Collect valid tracker poses
        tracker_poses = {}
        self.get_logger().info('  Processing tracker data:')
        for i, tracker_data in enumerate(tracker_data_list):
            role = self._get_role_for_tracker(tracker_data)

            sn_display = tracker_data.serial_number
            pos = tracker_data.position
            is_valid = tracker_data.is_valid

            self.get_logger().info(
                f'    [{i}] {sn_display} -> {role}: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], valid={is_valid}'
            )

            if is_valid and role:
                tracker_poses[role] = tracker_data.to_pose_array()

        # Delegate to incremental controller
        recorded_roles = self.controller.initialize(tracker_poses, hmd_pose_array)

        self.get_logger().info('-' * 60)
        self.get_logger().info('Initialization complete! (Incremental control mode)')
        self.get_logger().info('  Coordinate transform: PICO (+X right,+Y up,+Z forward) -> Robot (+X forward,+Y left,+Z up)')
        self.get_logger().info(f'  Recorded {len(recorded_roles)} tracker initial positions:')
        for role in recorded_roles:
            T = self.controller.init_tracker_poses[role]
            pos = T[:3, 3]
            self.get_logger().info(f'    {role}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

        # Check for missing trackers
        expected_roles = set(_TRACKER_SLOTS.values())
        missing_roles = expected_roles - recorded_roles
        if missing_roles:
            self.get_logger().warning(f'  Missing trackers: {missing_roles}')
            self.get_logger().warning('  Please check:')
            self.get_logger().warning('    1. All trackers are powered on and paired')
            self.get_logger().warning('    2. Serial number mapping in pico_input.yaml is correct')
            self.get_logger().warning('    3. PICO XRoboToolkit App detects all trackers')

        self.get_logger().info('-' * 60)
        self.get_logger().info('Robot initial positions (from tianji_robot.yaml):')
        for role, pos in self.controller.robot_init_positions.items():
            self.get_logger().info(f'    {role}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Incremental control mode:')
        self.get_logger().info('  User hand moves delta -> Robot hand moves delta')
        self.get_logger().info('  No jumping due to different initialization poses')
        self.get_logger().info('=' * 60)

    def _get_role_for_tracker(self, tracker_data: TrackerData) -> str:
        """Map a tracker's full SN string to its role (pico_left_wrist etc.).

        Exact-string match against the four SNs configured in pico_input.yaml.
        Returns None for any tracker the operator hasn't configured —
        callers warn and skip that frame.
        """
        role = self.sn_to_role.get(tracker_data.serial_number)
        if role is None:
            self.get_logger().warning(
                f'Cannot find role mapping for tracker '
                f'{tracker_data.serial_number}; not configured in pico_input.yaml.'
            )
        return role

    def _publish_callback(self):
        """Main loop: read data from data source and publish TF + Topics"""
        if not self.data_source.is_available():
            return

        now = self.get_clock().now()

        try:
            # Get headset data from data source
            headset_data = self.data_source.get_headset_pose()

            if headset_data is None or not headset_data.is_valid:
                self._handle_no_data(now)
                return

            # Device connected, clear waiting state
            if self._waiting_for_device:
                self._waiting_for_device = False
                self._no_data_count = 0
                self.get_logger().info('Data source ready, receiving data...')

            # Get tracker data from data source
            tracker_data_list = self.data_source.get_tracker_data()

            # Auto-initialization logic
            if not self.controller.initialized and self.auto_init_delay > 0:
                if self.init_start_time is None:
                    self.init_start_time = now
                    self.get_logger().info(f'Waiting {self.auto_init_delay:.1f} seconds before auto-initialization...')
                    self.get_logger().info('  Recommended to mimic the robot\'s current pose for better control feel')
                    self.get_logger().info('  Or call: ros2 service call /pico_input/init std_srvs/srv/Trigger')
                else:
                    elapsed = (now - self.init_start_time).nanoseconds / 1e9
                    if elapsed >= self.auto_init_delay:
                        self._do_init(headset_data, tracker_data_list)

            if not self.controller.initialized:
                return

            # Convert HMD to world coordinate system and publish TF (visualization only)
            hmd_pose_array = headset_data.to_pose_array()
            head_T = self.controller.compute_hmd_world_pose(hmd_pose_array)
            if head_T is not None:
                pos = head_T[:3, 3]
                quat = R.from_matrix(head_T[:3, :3]).as_quat()

                # Check converted quaternion validity
                quat_norm = np.linalg.norm(quat)
                if quat_norm > 0.9 and quat_norm < 1.1:
                    self._broadcast_tf(now, 'world', 'head', pos, quat)

                    # Legacy topic (optional, debug only)
                    if self.enable_legacy_topics and self.hmd_pub:
                        self._publish_pose(self.hmd_pub, now, 'world', pos, quat)

            # Process Trackers
            self._process_trackers(now, tracker_data_list)

        except Exception as e:
            self._handle_error(now, str(e))

    def _handle_no_data(self, now):
        """Handle no-data situation (PICO not connected)"""
        self._no_data_count += 1

        # Only output hints periodically while waiting for device
        if self._waiting_for_device:
            current_time = now.nanoseconds / 1e9
            if self._last_warn_time is None or (current_time - self._last_warn_time) >= self._warn_interval:
                self._last_warn_time = current_time
                self.get_logger().warning(
                    f'Waiting for PICO device connection... (waited {self._no_data_count} frames)\n'
                    '  Please ensure:\n'
                    '  1. PICO headset is powered on and running XRoboToolkit App\n'
                    '  2. PICO and PC are on the same WiFi network\n'
                    '  3. App is connected to PC IP address'
                )

    def _handle_error(self, now, error_msg: str):
        """Handle errors (with throttling)"""
        current_time = now.nanoseconds / 1e9
        if self._last_warn_time is None or (current_time - self._last_warn_time) >= self._warn_interval:
            self._last_warn_time = current_time
            # Simplify error message
            if 'zero norm quaternions' in error_msg:
                self.get_logger().warning('PICO data invalid (quaternion is zero) - please check device connection')
            else:
                self.get_logger().warning(f'Data processing error: {error_msg}')

    def _process_trackers(self, now, tracker_data_list: list):
        """
        Process Motion Tracker data, compute incremental poses

        Args:
            now: Current timestamp
            tracker_data_list: List[TrackerData] from data source
        """
        if not tracker_data_list:
            return

        # Temporary storage for elbow direction computation
        tf_data = {}      # role -> (pos, quat) (robot coordinate system)

        for i, tracker_data in enumerate(tracker_data_list):
            # Get role
            role = self._get_role_for_tracker(tracker_data)
            if not role:
                continue

            # Check data validity
            if not tracker_data.is_valid:
                continue

            # Convert to pose array
            pose = tracker_data.to_pose_array()

            # Compute incremental pose (delegate to controller)
            pos, quat = self.controller.compute_target_pose(pose, role)
            if pos is None:
                continue

            tf_data[role] = (pos, quat)

            # Determine parent frame (chest coordinate system)
            if 'left' in role:
                parent_frame = 'world_left'
            else:
                parent_frame = 'world_right'

            # 1. TF broadcast: world_left/world_right -> pico_left_wrist/pico_left_arm etc.
            self._broadcast_tf(now, parent_frame, role, pos, quat)

            # 2. Topic publishing (subscribed by tianji_world_output)
            if self.enable_topic_publishing:
                if role == 'pico_left_wrist':
                    self._publish_pose(self.left_arm_pose_pub, now, parent_frame, pos, quat)
                elif role == 'pico_right_wrist':
                    self._publish_pose(self.right_arm_pose_pub, now, parent_frame, pos, quat)

            # 3. Legacy topic publishing: /pico_left_wrist, /pico_right_wrist, etc. (debug only)
            if self.enable_legacy_topics and role in self.tracker_pubs:
                self._publish_pose(self.tracker_pubs[role], now, parent_frame, pos, quat)

        # ==================== Compute and publish elbow direction ====================
        if self.enable_topic_publishing:
            self._publish_elbow_directions(now, tf_data)

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
        """Publish PoseStamped (for RViz/simulation)"""
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

    def _publish_elbow_directions(self, now, tf_data: dict):
        """
        Publish elbow direction vectors (arm angle constraint) + RViz visualization Markers

        Computes geometric direction from arm tracker position (pointing toward gravity/downward),
        negates it before publishing to IK (anti-gravity direction).
        RViz Markers use physical direction (not negated), consistent with green shoulder-elbow arrow.
        """
        # Collect wrist and forearm positions
        wrist_positions = {}
        arm_positions = {}
        for role, (pos, quat) in tf_data.items():
            side = 'left' if 'left' in role else 'right'
            if 'wrist' in role:
                wrist_positions[side] = pos.copy()
            elif 'arm' in role:
                arm_positions[side] = pos.copy()

        markers = MarkerArray()
        marker_id = 0

        for side in ['left', 'right']:
            parent_frame = f'world_{side}'
            pub = self.left_elbow_dir_pub if side == 'left' else self.right_elbow_dir_pub
            default_dir = self.controller.default_zsp_direction[side]

            wrist_pos = wrist_positions.get(side)
            elbow_pos = arm_positions.get(side)

            if wrist_pos is not None and elbow_pos is not None:
                shoulder_pos = np.array([0.0, 0.0, 0.0])
                direction, proj_point = self.controller.compute_elbow_direction(
                    shoulder_pos, wrist_pos, elbow_pos, side
                )

                # Geometric direction (toward gravity) -> IK direction (anti-gravity): negate
                ik_direction = -direction
                self._publish_vector3(pub, now, parent_frame, ik_direction)

                # Diagnostic log (every 90 frames ~ once per second)
                if self._elbow_log_counter % 90 == 0:
                    sw_len = float(np.linalg.norm(wrist_pos))
                    offset_len = float(np.linalg.norm(elbow_pos - proj_point))
                    dot_default = float(np.dot(ik_direction, default_dir))
                    self.get_logger().info(
                        f'[ELBOW {side}] wrist=[{wrist_pos[0]:.3f},{wrist_pos[1]:.3f},{wrist_pos[2]:.3f}] '
                        f'arm=[{elbow_pos[0]:.3f},{elbow_pos[1]:.3f},{elbow_pos[2]:.3f}] '
                        f'sw={sw_len:.3f} offset={offset_len:.4f}'
                    )
                    self.get_logger().info(
                        f'[ELBOW {side}] phys=[{direction[0]:.3f},{direction[1]:.3f},{direction[2]:.3f}] '
                        f'IK=[{ik_direction[0]:.3f},{ik_direction[1]:.3f},{ik_direction[2]:.3f}] '
                        f'dot_default={dot_default:.3f}'
                    )

                # Visualization Markers (use physical direction, consistent with green shoulder-elbow arrow)
                side_markers = self._create_elbow_markers(
                    now, parent_frame, shoulder_pos, wrist_pos, elbow_pos,
                    proj_point, direction, side, marker_id
                )
                markers.markers.extend(side_markers)
                marker_id += len(side_markers)

        self._elbow_log_counter += 1

        # Publish visualization markers
        if markers.markers and self.marker_pub:
            self.marker_pub.publish(markers)

    def _create_elbow_markers(self, now, frame_id: str,
                               shoulder: np.ndarray, wrist: np.ndarray,
                               elbow: np.ndarray, proj_point: np.ndarray,
                               direction: np.ndarray, side: str,
                               start_id: int) -> list:
        """
        Create arm angle visualization RViz Markers (reference step4)

        - Red arrow: shoulder -> wrist
        - Green arrow: shoulder -> elbow
        - Blue arrow: projection point -> elbow offset direction (elbow_direction)
        - Yellow sphere: projection point
        """
        markers = []
        timestamp = now.to_msg()
        lifetime_ns = int(0.15 * 1e9)

        def _pt(arr):
            p = Point()
            p.x, p.y, p.z = float(arr[0]), float(arr[1]), float(arr[2])
            return p

        # 1. Red arrow: shoulder -> wrist
        m_sw = Marker()
        m_sw.header.stamp = timestamp
        m_sw.header.frame_id = frame_id
        m_sw.ns = f'{side}_arm_axis'
        m_sw.id = start_id
        m_sw.type = Marker.ARROW
        m_sw.action = Marker.ADD
        m_sw.points = [_pt(shoulder), _pt(wrist)]
        m_sw.scale.x = 0.01
        m_sw.scale.y = 0.02
        m_sw.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        m_sw.lifetime.nanosec = lifetime_ns
        markers.append(m_sw)

        # 2. Green arrow: shoulder -> elbow
        m_se = Marker()
        m_se.header.stamp = timestamp
        m_se.header.frame_id = frame_id
        m_se.ns = f'{side}_shoulder_to_elbow'
        m_se.id = start_id + 1
        m_se.type = Marker.ARROW
        m_se.action = Marker.ADD
        m_se.points = [_pt(shoulder), _pt(elbow)]
        m_se.scale.x = 0.01
        m_se.scale.y = 0.02
        m_se.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        m_se.lifetime.nanosec = lifetime_ns
        markers.append(m_se)

        # 3. Blue arrow: projection point -> elbow offset direction (elbow_direction)
        arrow_length = 0.15
        arrow_end = proj_point + direction * arrow_length
        m_dir = Marker()
        m_dir.header.stamp = timestamp
        m_dir.header.frame_id = frame_id
        m_dir.ns = f'{side}_elbow_direction'
        m_dir.id = start_id + 2
        m_dir.type = Marker.ARROW
        m_dir.action = Marker.ADD
        m_dir.points = [_pt(proj_point), _pt(arrow_end)]
        m_dir.scale.x = 0.015
        m_dir.scale.y = 0.025
        m_dir.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        m_dir.lifetime.nanosec = lifetime_ns
        markers.append(m_dir)

        # 4. Yellow sphere: projection point
        m_proj = Marker()
        m_proj.header.stamp = timestamp
        m_proj.header.frame_id = frame_id
        m_proj.ns = f'{side}_projection_point'
        m_proj.id = start_id + 3
        m_proj.type = Marker.SPHERE
        m_proj.action = Marker.ADD
        m_proj.pose.position = _pt(proj_point)
        m_proj.pose.orientation.w = 1.0
        m_proj.scale.x = m_proj.scale.y = m_proj.scale.z = 0.02
        m_proj.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        m_proj.lifetime.nanosec = lifetime_ns
        markers.append(m_proj)

        return markers

    def _publish_vector3(self, publisher, now, frame_id: str, vector: np.ndarray):
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

    def _log_startup(self):
        """Print startup information"""
        # Get local IP address
        local_ip = "127.0.0.1"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass

        mode_parts = []
        if self.enable_topic_publishing:
            mode_parts.append('Topics (/left_arm_target_pose, /right_arm_target_pose)')
        if self.enable_legacy_topics:
            mode_parts.append(f'Legacy Topics (/{self.topic_prefix}/*)')
        mode_parts.append('TF')
        mode = ' + '.join(mode_parts)

        self.get_logger().info('=' * 70)
        self.get_logger().info('PICO Input Node (TDD Refactoring) - Incremental Control Mode')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  Data source: {self.data_source}')
        self.get_logger().info(f'  Status: {"Ready" if self.data_source.is_available() else "Not ready"}')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Output mode: {mode}')
        self.get_logger().info('  Arm angle control: always enabled (dynamic elbow direction, One-Euro adaptive smoothing)')
        self.get_logger().info(f'  Filtering: One-Euro Filter (position+orientation+elbow direction unified adaptive smoothing)')
        self.get_logger().info(f'  Tracker SN -> role: {self.sn_to_role}')
        self.get_logger().info('-' * 70)

        if self.data_source_type == 'live':
            self.get_logger().info('Live mode - PICO SDK connection config:')
            self.get_logger().info(f'  PC IP address: {local_ip}')
            self.get_logger().info(f'  Port: {self.pc_service_port}')
            self.get_logger().info('')
            self.get_logger().info('  In PICO headset, open XRoboToolkit App and enter:')
            self.get_logger().info(f'    Server address: {local_ip}')
            self.get_logger().info(f'    Port: {self.pc_service_port}')
            self.get_logger().info('')
            self.get_logger().info('  Make sure PICO and PC are on the same WiFi network!')
        elif self.data_source_type == 'recorded':
            file_path = self.get_parameter('recorded_file_path').value
            speed = self.get_parameter('playback_speed').value
            self.get_logger().info('Recorded mode - Data playback:')
            self.get_logger().info(f'  File: {file_path}')
            self.get_logger().info(f'  Speed: {speed}x')

        self.get_logger().info('-' * 70)
        self.get_logger().info('Incremental control mode:')
        self.get_logger().info('  - Record positions of four trackers during initialization')
        self.get_logger().info('  - User hand moves delta -> Robot hand moves delta')
        self.get_logger().info('  - Coordinate transform: PICO(+X right,+Y up,+Z forward) -> Robot(+X forward,+Y left,+Z up)')
        self.get_logger().info('-' * 65)
        self.get_logger().info('Robot initial positions (from tianji_robot.yaml):')
        for role, pos in self.controller.robot_init_positions.items():
            self.get_logger().info(f'  {role}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
        self.get_logger().info('-' * 65)
        self.get_logger().info('Services:')
        self.get_logger().info('  ros2 service call /pico_input/init std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /pico_input/reset std_srvs/srv/Trigger')
        if self.auto_init_delay > 0:
            self.get_logger().info(f'  Auto-initialization: after {self.auto_init_delay:.1f} seconds')
        self.get_logger().info('=' * 65)

    def destroy_node(self):
        """Clean up resources"""
        # Close data source (replaces original SDK close)
        if hasattr(self, 'data_source') and self.data_source:
            self.data_source.close()
            self.get_logger().info(f'Data source closed: {self.data_source}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Check if rclpy has already been shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()
