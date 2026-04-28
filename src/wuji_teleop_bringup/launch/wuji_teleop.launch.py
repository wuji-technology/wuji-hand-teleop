"""
Wuji Full Teleoperation Launch File

Launches all components for hand and arm teleoperation with configurable input devices.

Supported combinations:
  - hand_input: manus (Manus Gloves)
  - arm_input: tracker (HTC Vive Trackers)

Usage:
    # Manus gloves + Vive Trackers
    ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker

    # With RViz visualization
    ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker enable_rviz:=true
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)
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
    hand_input_arg = DeclareLaunchArgument(
        "hand_input",
        default_value="manus",
        description="Hand input device: 'manus' (Manus Gloves)",
    )
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
    # ===== wujihandros2 driver parameters =====
    left_serial_arg = DeclareLaunchArgument(
        "left_serial",
        default_value=LEFT_HAND_SERIAL,
        description="Left hand serial number",
    )
    right_serial_arg = DeclareLaunchArgument(
        "right_serial",
        default_value=RIGHT_HAND_SERIAL,
        description="Right hand serial number",
    )
    left_hand_name_arg = DeclareLaunchArgument(
        "left_hand_name",
        default_value=LEFT_HAND_NAME,
        description="Left hand wujihandros2 namespace",
    )
    right_hand_name_arg = DeclareLaunchArgument(
        "right_hand_name",
        default_value=RIGHT_HAND_NAME,
        description="Right hand wujihandros2 namespace",
    )

    # Get configurations
    arm_input = LaunchConfiguration("arm_input")
    enable_rviz = LaunchConfiguration("enable_rviz")

    # Force serial_number to string type (workaround for ROS2 type inference)
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str
    )
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str
    )

    return LaunchDescription([
        # Arguments
        hand_input_arg,
        arm_input_arg,
        enable_rviz_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,

        # ==================== WUJIHANDROS2 DRIVERS ====================
        # Left hand driver (wujihandros2)
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=LaunchConfiguration("left_hand_name"),
            parameters=[{
                "serial_number": left_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
        ),
        # Right hand driver (wujihandros2)
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=LaunchConfiguration("right_hand_name"),
            parameters=[{
                "serial_number": right_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
        ),

        # ==================== ARM INPUT: Tracker ====================
        # OpenVR Input Node (for tracker arm tracking)
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

        # ==================== HAND INPUT: Manus ====================
        # Manus ROS2 Driver (USB access via udev rule, no sudo needed)
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        # ==================== HAND OUTPUT: Wuji Hand (per-hand process, multi-core parallel) ====================
        # One controller process per hand, subscribing directly to /manus_glove_* (no Python wrapper).
        # Each runs on its own GIL: measured 58Hz → 120Hz, end-to-end latency 28ms → 8ms.
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_left",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--side", "left",
                "--hand-name", LaunchConfiguration("left_hand_name"),
            ],
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_right",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--side", "right",
                "--hand-name", LaunchConfiguration("right_hand_name"),
            ],
            condition=LaunchConfigurationEquals("hand_input", "manus"),
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
