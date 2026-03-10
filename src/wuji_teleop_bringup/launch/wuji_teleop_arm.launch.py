"""
Wuji Arm-Only Teleoperation Launch File / Wuji 仅机械臂遥操作启动文件

Launches arm teleoperation components with configurable input device.
启动机械臂遥操作组件，支持配置不同的输入设备。

Supported input devices / 支持的输入设备:
  - tracker: HTC Vive Trackers

Usage / 使用方式:
    # Using HTC Vive Trackers
    ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker

    # With RViz visualization
    ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker enable_rviz:=true
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _get_config_path(package: str, config_file: str) -> str:
    """Get the path to a config file in a package's share directory."""
    share_dir = Path(get_package_share_directory(package))
    return str(share_dir / "config" / config_file)


def _get_rviz_path() -> str:
    """Get the path to the RViz config file."""
    share_dir = Path(get_package_share_directory("openvr_input"))
    return str(share_dir / "rviz" / "openvr_visualization.rviz")


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    arm_input_arg = DeclareLaunchArgument(
        "arm_input",
        default_value="tracker",
        description="Arm input device: 'tracker' (HTC Vive Trackers)",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="false",
        description="Enable RViz visualization",
    )

    enable_rviz = LaunchConfiguration("enable_rviz")

    return LaunchDescription([
        # Arguments
        arm_input_arg,
        enable_rviz_arg,

        # ==================== ARM INPUT: Tracker ====================
        Node(
            package="openvr_input",
            executable="openvr_input",
            name="openvr_input",
            output="screen",
            arguments=["-c", _get_config_path("openvr_input", "openvr_input.yaml")],
            condition=LaunchConfigurationEquals("arm_input", "tracker"),
        ),

        # ==================== STATIC TF: From Config ====================
        OpaqueFunction(function=lambda ctx: create_chest_tf_nodes() + create_tianji_tf_nodes()),

        # ==================== ARM OUTPUT: Tianji ====================
        # Tianji Arm Controller (TELEOP mode by default)
        # Also publishes state: /tianji_arm/left/joint_state, /tianji_arm/right/joint_state
        Node(
            package="controller",
            executable="tianji_arm_controller",
            name="tianji_arm_controller",
            output="screen",
            emulate_tty=True,
        ),

        # ==================== VISUALIZATION ====================
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", _get_rviz_path()],
            condition=IfCondition(enable_rviz),
        ),
    ])
