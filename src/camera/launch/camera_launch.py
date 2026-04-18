#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified camera launch file

Supports three camera positions: head, left_wrist, right_wrist
Supports three camera types: d435i, d405, usb
Supports head stereo camera ROS2 publishing and PICO H.264 streaming (unified_stereo, no v4l2loopback needed)

Usage:
    # Default: wrist RealSense + head stereo camera
    ros2 launch camera camera_launch.py

    # Wrist RealSense only (disable head)
    ros2 launch camera camera_launch.py enable_head:=false

    # D435 testing + head
    ros2 launch camera camera_launch.py wrist_type:=d435i enable_head:=true
"""

import os
from typing import List, Dict, Any

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# Configuration loading
# =============================================================================

def load_config(config_path: str) -> Dict[str, Any]:
    """Load YAML configuration file"""
    if not os.path.exists(config_path):
        print(f"[WARN] Config file not found: {config_path}")
        return {}
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load config: {e}")
        return {}


def get_default_config_path() -> str:
    """Get default configuration file path"""
    try:
        pkg_share = get_package_share_directory('camera')
        return os.path.join(pkg_share, 'config', 'camera_config.yaml')
    except Exception:
        return ""


# =============================================================================
# RealSense camera creation (D435i / D405)
# =============================================================================

def create_realsense_camera(
    cam_config: Dict[str, Any],
    is_master: bool,
    enable_sync: bool,
) -> List:
    """Create RealSense camera launch configuration (D435i or D405)"""
    actions = []

    cam_type = cam_config.get('type', 'd405')
    cam_name = cam_config.get('camera_name', 'camera')
    serial_no = cam_config.get('serial_number', '')

    # Resolution
    resolution = cam_config.get('resolution', {})
    width = str(resolution.get('width', 640))
    height = str(resolution.get('height', 480))
    fps = str(resolution.get('fps', 30))

    # Stream settings
    streams = cam_config.get('streams', {})
    enable_color = str(streams.get('enable_color', True)).lower()
    enable_depth = str(streams.get('enable_depth', False)).lower()
    align_depth = str(streams.get('align_depth', False)).lower()
    enable_pointcloud = str(streams.get('enable_pointcloud', False)).lower()
    enable_gyro = str(streams.get('enable_gyro', False)).lower()
    enable_accel = str(streams.get('enable_accel', False)).lower()

    # D405 has no IMU, force disable
    if cam_type == 'd405':
        enable_gyro = 'false'
        enable_accel = 'false'

    # Sync mode
    if enable_sync and not is_master:
        sync_mode = '2'  # Slave
    elif enable_sync and is_master:
        sync_mode = '1'  # Master
    else:
        sync_mode = '0'  # No sync

    # TF configuration
    tf_config = cam_config.get('tf', {})
    tf_parent = tf_config.get('parent_frame', 'base_link')
    tf_child = tf_config.get('child_frame', f'{cam_name}_link')
    tf_xyz = tf_config.get('translation', [0.0, 0.0, 0.0])
    tf_rpy = tf_config.get('rotation', [0.0, 0.0, 0.0])

    print(f"[INFO] RealSense {cam_type.upper()}: {cam_name}")
    print(f"       Serial: {serial_no if serial_no else 'auto-detect'}")
    print(f"       Resolution: {width}x{height}@{fps}fps")
    print(f"       Sync mode: {'Master' if sync_mode == '1' else 'Slave' if sync_mode == '2' else 'None'}")

    # Start camera
    cam_launch = GroupAction(
        actions=[
            SetEnvironmentVariable(
                'ROS_IMAGE_TRANSPORT_PLUGINS',
                'image_transport/raw_pub;image_transport/compressed_pub'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ]),
                launch_arguments={
                    'config_file': "''",
                    'camera_name': cam_name,
                    'camera_namespace': '',
                    'initial_reset': 'false',
                    'device_type': cam_type,
                    'serial_no': f"'{serial_no}'" if serial_no else "''",
                    'enable_depth': enable_depth,
                    'enable_color': enable_color,
                    'enable_pointcloud': enable_pointcloud,
                    'align_depth.enable': align_depth,
                    'enable_gyro': enable_gyro,
                    'enable_accel': enable_accel,
                    'depth_module.depth_profile': f'{width}x{height}x{fps}',
                    'rgb_camera.color_profile': f'{width}x{height}x{fps}',
                    'depth_module.inter_cam_sync_mode': sync_mode,
                    'rgb_camera.color_image_transport': 'raw',
                    'depth_module.depth_image_transport': 'raw',
                    'wait_for_device_timeout': '60.0',
                    'reconnect_timeout': '6.0',
                }.items()
            )
        ]
    )
    actions.append(cam_launch)

    # TF publisher
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{cam_name}_tf',
        arguments=[
            '--x', str(tf_xyz[0]),
            '--y', str(tf_xyz[1]),
            '--z', str(tf_xyz[2]),
            '--roll', str(tf_rpy[0]),
            '--pitch', str(tf_rpy[1]),
            '--yaw', str(tf_rpy[2]),
            '--frame-id', tf_parent,
            '--child-frame-id', tf_child,
        ]
    )
    actions.append(tf_node)

    return actions


# =============================================================================
# USB camera creation
# =============================================================================

def create_usb_camera(cam_config: Dict[str, Any]) -> List:
    """Create USB camera launch configuration"""
    actions = []

    cam_name = cam_config.get('camera_name', 'usb_camera')
    video_device = cam_config.get('video_device', '/dev/video0')
    pixel_format = cam_config.get('pixel_format', 'mjpeg2rgb')
    io_method = cam_config.get('io_method', 'mmap')

    # Resolution
    resolution = cam_config.get('resolution', {})
    width = resolution.get('width', 640)
    height = resolution.get('height', 480)
    fps = float(resolution.get('fps', 30))

    # TF configuration
    tf_config = cam_config.get('tf', {})
    tf_parent = tf_config.get('parent_frame', 'base_link')
    tf_child = tf_config.get('child_frame', f'{cam_name}_link')
    tf_xyz = tf_config.get('translation', [0.0, 0.0, 0.0])
    tf_rpy = tf_config.get('rotation', [0.0, 0.0, 0.0])

    print(f"[INFO] USB camera: {cam_name}")
    print(f"       Device: {video_device}")
    print(f"       Resolution: {width}x{height}@{fps}fps")

    # Camera node
    cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'{cam_name}_node',
        namespace=cam_name,
        parameters=[{
            'video_device': video_device,
            'camera_name': cam_name,
            'frame_id': tf_child,
            'image_width': width,
            'image_height': height,
            'framerate': fps,
            'pixel_format': pixel_format,
            'io_method': io_method,
        }],
        remappings=[
            ('image_raw', 'color/image_raw'),
            ('camera_info', 'color/camera_info'),
        ],
        output='screen',
    )
    actions.append(cam_node)

    # TF publisher
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{cam_name}_tf',
        arguments=[
            '--x', str(tf_xyz[0]),
            '--y', str(tf_xyz[1]),
            '--z', str(tf_xyz[2]),
            '--roll', str(tf_rpy[0]),
            '--pitch', str(tf_rpy[1]),
            '--yaw', str(tf_rpy[2]),
            '--frame-id', tf_parent,
            '--child-frame-id', tf_child,
        ]
    )
    actions.append(tf_node)

    return actions


# =============================================================================
# Main launch function
# =============================================================================

def launch_cameras(context):
    """Launch all cameras based on configuration"""
    from launch.substitutions import LaunchConfiguration

    actions = []

    # Load configuration
    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = get_default_config_path()

    config = load_config(config_file)
    if not config:
        print("[ERROR] Unable to load config!")
        return []

    # Launch argument override (empty string = use YAML default)
    wrist_type_override = LaunchConfiguration('wrist_type').perform(context)

    # Head camera parameters
    enable_head = LaunchConfiguration('enable_head').perform(context).lower() == 'true'
    enable_pico = LaunchConfiguration('enable_pico').perform(context).lower() == 'true'
    head_device = LaunchConfiguration('head_device').perform(context)
    head_fps = LaunchConfiguration('head_fps').perform(context)
    head_quality = LaunchConfiguration('head_quality').perform(context)
    head_width = LaunchConfiguration('head_width').perform(context)
    head_height = LaunchConfiguration('head_height').perform(context)

    # Global settings
    global_config = config.get('global', {})
    startup_delay = global_config.get('startup_delay', 2.0)
    enable_sync = global_config.get('enable_sync', True)

    # Camera configuration
    cameras_config = config.get('cameras', {})

    # Apply command-line overrides
    if wrist_type_override:
        for key in ['left_wrist', 'right_wrist']:
            if key in cameras_config:
                cameras_config[key]['type'] = wrist_type_override
                print(f"[Override] {key}.type = {wrist_type_override}")

    print("=" * 60)
    print("Camera Launch Configuration")
    print("=" * 60)
    print(f"Config file: {config_file}")
    print(f"Startup delay: {startup_delay}s")
    print(f"RealSense sync: {enable_sync}")
    print(f"Head stereo → ROS2: {'Enabled' if enable_head else 'Disabled'}")
    print(f"Head stereo → PICO: {'Enabled' if enable_pico else 'Disabled'}")
    print("-" * 60)

    # Count enabled RealSense cameras (for sync)
    realsense_count = sum(
        1 for cam in cameras_config.values()
        if cam.get('enabled', False) and cam.get('type', '') in ['d435i', 'd405']
    )

    current_delay = 0.0
    is_first_realsense = True

    # Launch in fixed order: head -> left_wrist -> right_wrist
    for cam_key in ['head', 'left_wrist', 'right_wrist']:
        if cam_key not in cameras_config:
            continue

        cam_config = cameras_config[cam_key]
        if not cam_config.get('enabled', False):
            print(f"[Skip] {cam_key}: not enabled")
            continue

        # Head camera: handled by unified_stereo below (outputs both ROS2 + PICO H.264)
        # Not started separately in the loop to avoid contention with unified_stereo for the same device
        if cam_key == 'head':
            if enable_head or enable_pico:
                print(f"[Skip] {cam_key}: handled by unified_stereo")
            else:
                print(f"[Skip] {cam_key}: head camera not enabled (enable_head/enable_pico=false)")
            continue

        cam_type = cam_config.get('type', 'd435i')

        # Create camera by type
        if cam_type in ['d435i', 'd405']:
            cam_actions = create_realsense_camera(
                cam_config=cam_config,
                is_master=is_first_realsense,
                enable_sync=enable_sync and realsense_count > 1,
            )
            is_first_realsense = False
        elif cam_type == 'usb':
            cam_actions = create_usb_camera(cam_config)
        else:
            print(f"[WARN] Unknown camera type: {cam_type}")
            continue

        # Add delayed start
        if current_delay > 0:
            actions.append(TimerAction(
                period=current_delay,
                actions=cam_actions
            ))
        else:
            actions.extend(cam_actions)

        current_delay += startup_delay

    # ==================== Head Stereo Camera ====================
    if enable_head or enable_pico:
        # Unified mode: unified_stereo single process handles ROS2 + PICO H.264 (no v4l2loopback needed)
        print(f"[INFO] Head stereo (unified): {head_device} @ {head_fps}fps, JPEG Q{head_quality}")
        if enable_pico:
            print(f"[INFO]   PICO H.264: on-demand via XRobo TCP 13579")
        unified_stereo = ExecuteProcess(
            cmd=[
                'python3', '-m', 'stereocamera.unified_stereo',
                '--device', head_device,
                '--fps', head_fps,
                '--quality', head_quality,
                '--width', head_width,
                '--height', head_height,
            ],
            output='screen',
            name='unified_stereo',
        )
        if current_delay > 0:
            actions.append(TimerAction(
                period=current_delay,
                actions=[unified_stereo]
            ))
        else:
            actions.append(unified_stereo)
        current_delay += startup_delay

    print("=" * 60)
    return actions


def generate_launch_description():
    """Generate launch description"""

    # Disable compressed_depth plugin
    disable_compressed_depth = SetEnvironmentVariable(
        'ROS_IMAGE_TRANSPORT_PLUGINS',
        'image_transport/raw_pub;image_transport/compressed_pub'
    )

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Camera configuration file path'
    )

    wrist_type_arg = DeclareLaunchArgument(
        'wrist_type',
        default_value='',
        description='Override wrist camera type (d435i/d405), empty=use YAML default'
    )

    
    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='Enable head stereo camera → ROS2 publishing'
    )

    enable_pico_arg = DeclareLaunchArgument(
        'enable_pico',
        default_value='false',
        description='Enable head stereo PICO H.264 (unified_stereo handles automatically, no v4l2loopback needed)'
    )

    head_device_arg = DeclareLaunchArgument(
        'head_device',
        default_value='/dev/stereo_camera',
        description='Head stereo camera device path'
    )

    head_fps_arg = DeclareLaunchArgument(
        'head_fps',
        default_value='30',
        description='Head stereo camera ROS2 publishing frame rate'
    )

    head_quality_arg = DeclareLaunchArgument(
        'head_quality',
        default_value='70',
        description='Head stereo camera JPEG compression quality (1-100)'
    )

    head_width_arg = DeclareLaunchArgument(
        'head_width',
        default_value='2560',
        description='Head stereo camera frame width (left+right combined)'
    )

    head_height_arg = DeclareLaunchArgument(
        'head_height',
        default_value='720',
        description='Head stereo camera frame height'
    )

    return LaunchDescription([
        disable_compressed_depth,
        config_file_arg,
        wrist_type_arg,
        enable_head_arg,
        enable_pico_arg,
        head_device_arg,
        head_fps_arg,
        head_quality_arg,
        head_width_arg,
        head_height_arg,
        OpaqueFunction(function=launch_cameras),
    ])
