#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一相机启动文件

支持三个相机位置: head, left_wrist, right_wrist
支持三种相机类型: d435i, d405, usb
支持头部双目相机 ROS2 发布和 PICO H.264 串流 (unified_stereo, 无需 v4l2loopback)

Usage:
    # 仅腕部 RealSense (默认)
    ros2 launch camera camera_launch.py

    # 腕部 + 头部双目 → ROS2 (+ PICO H.264 on-demand)
    ros2 launch camera camera_launch.py enable_head:=true

    # D435 测试 + 头部
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
# 配置加载
# =============================================================================

def load_config(config_path: str) -> Dict[str, Any]:
    """加载YAML配置文件"""
    if not os.path.exists(config_path):
        print(f"[WARN] 配置文件不存在: {config_path}")
        return {}
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] 加载配置失败: {e}")
        return {}


def get_default_config_path() -> str:
    """获取默认配置文件路径"""
    try:
        pkg_share = get_package_share_directory('camera')
        return os.path.join(pkg_share, 'config', 'camera_config.yaml')
    except Exception:
        return ""


# =============================================================================
# RealSense 相机创建 (D435i / D405)
# =============================================================================

def create_realsense_camera(
    cam_config: Dict[str, Any],
    is_master: bool,
    enable_sync: bool,
) -> List:
    """创建RealSense相机启动配置 (D435i或D405)"""
    actions = []

    cam_type = cam_config.get('type', 'd405')
    cam_name = cam_config.get('camera_name', 'camera')
    serial_no = cam_config.get('serial_number', '')

    # 分辨率
    resolution = cam_config.get('resolution', {})
    width = str(resolution.get('width', 640))
    height = str(resolution.get('height', 480))
    fps = str(resolution.get('fps', 30))

    # 流设置
    streams = cam_config.get('streams', {})
    enable_color = str(streams.get('enable_color', True)).lower()
    enable_depth = str(streams.get('enable_depth', False)).lower()
    align_depth = str(streams.get('align_depth', False)).lower()
    enable_pointcloud = str(streams.get('enable_pointcloud', False)).lower()
    enable_gyro = str(streams.get('enable_gyro', False)).lower()
    enable_accel = str(streams.get('enable_accel', False)).lower()

    # D405没有IMU，强制禁用
    if cam_type == 'd405':
        enable_gyro = 'false'
        enable_accel = 'false'

    # 同步模式
    if enable_sync and not is_master:
        sync_mode = '2'  # Slave
    elif enable_sync and is_master:
        sync_mode = '1'  # Master
    else:
        sync_mode = '0'  # No sync

    # TF配置
    tf_config = cam_config.get('tf', {})
    tf_parent = tf_config.get('parent_frame', 'base_link')
    tf_child = tf_config.get('child_frame', f'{cam_name}_link')
    tf_xyz = tf_config.get('translation', [0.0, 0.0, 0.0])
    tf_rpy = tf_config.get('rotation', [0.0, 0.0, 0.0])

    print(f"[INFO] RealSense {cam_type.upper()}: {cam_name}")
    print(f"       序列号: {serial_no if serial_no else '自动检测'}")
    print(f"       分辨率: {width}x{height}@{fps}fps")
    print(f"       同步模式: {'Master' if sync_mode == '1' else 'Slave' if sync_mode == '2' else '无'}")

    # 启动相机
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

    # TF发布
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
# USB 相机创建
# =============================================================================

def create_usb_camera(cam_config: Dict[str, Any]) -> List:
    """创建USB相机启动配置"""
    actions = []

    cam_name = cam_config.get('camera_name', 'usb_camera')
    video_device = cam_config.get('video_device', '/dev/video0')
    pixel_format = cam_config.get('pixel_format', 'mjpeg2rgb')
    io_method = cam_config.get('io_method', 'mmap')

    # 分辨率
    resolution = cam_config.get('resolution', {})
    width = resolution.get('width', 640)
    height = resolution.get('height', 480)
    fps = float(resolution.get('fps', 30))

    # TF配置
    tf_config = cam_config.get('tf', {})
    tf_parent = tf_config.get('parent_frame', 'base_link')
    tf_child = tf_config.get('child_frame', f'{cam_name}_link')
    tf_xyz = tf_config.get('translation', [0.0, 0.0, 0.0])
    tf_rpy = tf_config.get('rotation', [0.0, 0.0, 0.0])

    print(f"[INFO] USB相机: {cam_name}")
    print(f"       设备: {video_device}")
    print(f"       分辨率: {width}x{height}@{fps}fps")

    # 相机节点
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

    # TF发布
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
# 主启动函数
# =============================================================================

def launch_cameras(context):
    """根据配置启动所有相机"""
    from launch.substitutions import LaunchConfiguration

    actions = []

    # 加载配置
    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = get_default_config_path()

    config = load_config(config_file)
    if not config:
        print("[ERROR] 无法加载配置!")
        return []

    # Launch argument 覆盖 (空字符串=使用YAML默认值)
    wrist_type_override = LaunchConfiguration('wrist_type').perform(context)

    # 头部相机参数
    enable_head = LaunchConfiguration('enable_head').perform(context).lower() == 'true'
    enable_pico = LaunchConfiguration('enable_pico').perform(context).lower() == 'true'
    head_device = LaunchConfiguration('head_device').perform(context)
    head_fps = LaunchConfiguration('head_fps').perform(context)
    head_quality = LaunchConfiguration('head_quality').perform(context)
    head_width = LaunchConfiguration('head_width').perform(context)
    head_height = LaunchConfiguration('head_height').perform(context)

    # 全局设置
    global_config = config.get('global', {})
    startup_delay = global_config.get('startup_delay', 2.0)
    enable_sync = global_config.get('enable_sync', True)

    # 相机配置
    cameras_config = config.get('cameras', {})

    # 应用命令行覆盖
    if wrist_type_override:
        for key in ['left_wrist', 'right_wrist']:
            if key in cameras_config:
                cameras_config[key]['type'] = wrist_type_override
                print(f"[覆盖] {key}.type = {wrist_type_override}")

    print("=" * 60)
    print("相机启动配置")
    print("=" * 60)
    print(f"配置文件: {config_file}")
    print(f"启动延迟: {startup_delay}s")
    print(f"RealSense同步: {enable_sync}")
    print(f"头部双目 → ROS2: {'启用' if enable_head else '禁用'}")
    print(f"头部双目 → PICO: {'启用' if enable_pico else '禁用'}")
    print("-" * 60)

    # 统计启用的RealSense相机数量（用于同步）
    realsense_count = sum(
        1 for cam in cameras_config.values()
        if cam.get('enabled', False) and cam.get('type', '') in ['d435i', 'd405']
    )

    current_delay = 0.0
    is_first_realsense = True

    # 按固定顺序启动: head -> left_wrist -> right_wrist
    for cam_key in ['head', 'left_wrist', 'right_wrist']:
        if cam_key not in cameras_config:
            continue

        cam_config = cameras_config[cam_key]
        if not cam_config.get('enabled', False):
            print(f"[跳过] {cam_key}: 未启用")
            continue

        # 头部相机: 由下方 unified_stereo 统一处理 (同时输出 ROS2 + PICO H.264)
        # 循环中不单独启动, 避免与 unified_stereo 抢占同一设备
        if cam_key == 'head':
            if enable_head or enable_pico:
                print(f"[跳过] {cam_key}: 由 unified_stereo 统一处理")
            else:
                print(f"[跳过] {cam_key}: 头部相机未启用 (enable_head/enable_pico=false)")
            continue

        cam_type = cam_config.get('type', 'd435i')

        # 根据类型创建相机
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
            print(f"[WARN] 未知相机类型: {cam_type}")
            continue

        # 添加延迟启动
        if current_delay > 0:
            actions.append(TimerAction(
                period=current_delay,
                actions=cam_actions
            ))
        else:
            actions.extend(cam_actions)

        current_delay += startup_delay

    # ==================== 头部双目相机 ====================
    if enable_head or enable_pico:
        # 统一模式: unified_stereo 单进程处理 ROS2 + PICO H.264 (无需 v4l2loopback)
        print(f"[INFO] 头部双目 (unified): {head_device} @ {head_fps}fps, JPEG Q{head_quality}")
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
    """生成启动描述"""

    # 禁用compressed_depth插件
    disable_compressed_depth = SetEnvironmentVariable(
        'ROS_IMAGE_TRANSPORT_PLUGINS',
        'image_transport/raw_pub;image_transport/compressed_pub'
    )

    # 启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='相机配置文件路径'
    )

    wrist_type_arg = DeclareLaunchArgument(
        'wrist_type',
        default_value='',
        description='覆盖腕部相机类型 (d435i/d405), 空=使用YAML默认值'
    )

    # 头部双目相机参数
    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='启用头部双目相机 → ROS2 发布'
    )

    enable_pico_arg = DeclareLaunchArgument(
        'enable_pico',
        default_value='false',
        description='启用头部双目 PICO H.264 (unified_stereo 自动处理, 无需 v4l2loopback)'
    )

    head_device_arg = DeclareLaunchArgument(
        'head_device',
        default_value='/dev/stereo_camera',
        description='头部双目相机设备路径'
    )

    head_fps_arg = DeclareLaunchArgument(
        'head_fps',
        default_value='30',
        description='头部双目相机 ROS2 发布帧率'
    )

    head_quality_arg = DeclareLaunchArgument(
        'head_quality',
        default_value='70',
        description='头部双目相机 JPEG 压缩质量 (1-100)'
    )

    head_width_arg = DeclareLaunchArgument(
        'head_width',
        default_value='2560',
        description='头部双目相机帧宽度 (左+右拼接)'
    )

    head_height_arg = DeclareLaunchArgument(
        'head_height',
        default_value='720',
        description='头部双目相机帧高度'
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
