"""
Wuji Hand-Only Teleoperation Launch File

Launches hand teleoperation components with configurable input device.

Supported input devices:
  - manus: Manus Gloves

Usage:
    # Using Manus Gloves
    ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    hand_input_arg = DeclareLaunchArgument(
        "hand_input",
        default_value="manus",
        description="Hand input device: 'manus' (Manus Gloves)",
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
    ])
