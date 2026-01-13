"""
Wuji Hand-Only Teleoperation Launch File / Wuji 仅手部遥操作启动文件

Launches hand teleoperation components with configurable input device.
启动手部遥操作组件，支持配置不同的输入设备。

Supported input devices / 支持的输入设备:
  - avp: Apple Vision Pro
  - manus: Manus Gloves

Usage / 使用方式:
    # Using Apple Vision Pro
    ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=avp

    # Using Manus Gloves
    ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_config_path(package: str, config_file: str) -> str:
    """Get the path to a config file in a package's share directory."""
    share_dir = Path(get_package_share_directory(package))
    return str(share_dir / "config" / config_file)


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    hand_input_arg = DeclareLaunchArgument(
        "hand_input",
        default_value="avp",
        description="Hand input device: 'avp' (Apple Vision Pro) or 'manus' (Manus Gloves)",
    )
    hand_config_arg = DeclareLaunchArgument(
        "hand_config",
        default_value=_get_config_path("wujihand_ik", "wujihand_ik.yaml"),
        description="Path to wujihand_ik config file",
    )

    hand_config = LaunchConfiguration("hand_config")
    hand_input = LaunchConfiguration("hand_input")

    return LaunchDescription([
        # Arguments
        hand_input_arg,
        hand_config_arg,

        # ==================== HAND INPUT: AVP ====================
        Node(
            package="avp_input",
            executable="avp_input",
            name="avp_input",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "avp"),
        ),

        # ==================== HAND INPUT: Manus ====================
        # Manus ROS2 Driver
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        # Manus Input Node (convert to MediaPipe format)
        Node(
            package="manus_input_py",
            executable="manus_input",
            name="manus_input",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),

        # ==================== HAND OUTPUT: Wuji Hand ====================
        Node(
            package="wujihand_ik",
            executable="wujihand_retargeting",
            name="wujihand_retargeting",
            output="screen",
            emulate_tty=True,
            arguments=["-c", hand_config, "-i", hand_input],
        ),
    ])
