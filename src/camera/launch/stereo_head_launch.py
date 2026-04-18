#!/usr/bin/env python3
"""
Head Stereo Camera Independent Launch File (backward-compatible wrapper)

Equivalent to: ros2 launch camera camera_launch.py enable_head:=true

Usage:
    ros2 launch camera stereo_head_launch.py
    ros2 launch camera stereo_head_launch.py camera_device:=/dev/video0
    ros2 launch camera stereo_head_launch.py enable_head:=false
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('camera'))
    camera_launch_path = str(pkg_share / 'launch' / 'camera_launch.py')

    # Parameters that can be overridden
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/stereo_camera',
        description='Stereo head camera device path'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frame rate (fps)'
    )

    quality_arg = DeclareLaunchArgument(
        'quality',
        default_value='70',
        description='JPEG quality (1-100, lower=faster)'
    )

    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='Enable stereo head camera ROS2 publisher (default: true)'
    )

    # Include camera_launch.py, head camera enabled by default
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'enable_head': LaunchConfiguration('enable_head'),
            'head_device': LaunchConfiguration('camera_device'),
            'head_fps': LaunchConfiguration('fps'),
            'head_quality': LaunchConfiguration('quality'),
        }.items()
    )

    return LaunchDescription([
        camera_device_arg,
        fps_arg,
        quality_arg,
        enable_head_arg,
        camera_launch,
    ])
