"""Hand-only launch: wuji_glove (default) or manus, both ends as per-side processes.

Source selected via wujihand_ik.yaml::input_source.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)
from wuji_teleop_bringup.launch_utils import read_input_source as _read_input_source


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    left_serial_arg = DeclareLaunchArgument(
        "left_serial", default_value=LEFT_HAND_SERIAL,
        description="Left hand serial number",
    )
    right_serial_arg = DeclareLaunchArgument(
        "right_serial", default_value=RIGHT_HAND_SERIAL,
        description="Right hand serial number",
    )
    left_hand_name_arg = DeclareLaunchArgument(
        "left_hand_name", default_value=LEFT_HAND_NAME,
        description="Left hand wujihandros2 namespace",
    )
    right_hand_name_arg = DeclareLaunchArgument(
        "right_hand_name", default_value=RIGHT_HAND_NAME,
        description="Right hand wujihandros2 namespace",
    )

    left_hand_name = LaunchConfiguration("left_hand_name")
    right_hand_name = LaunchConfiguration("right_hand_name")
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str)
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str)

    # Read once at launch evaluation; decides whether to spawn the manus driver.
    input_source = _read_input_source()
    spawn_manus = IfCondition("true") if input_source == "manus" else IfCondition("false")

    return LaunchDescription([
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,

        # ==================== WUJIHANDROS2 DRIVERS ====================
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=left_hand_name,
            parameters=[{
                "serial_number": left_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=right_hand_name,
            parameters=[{
                "serial_number": right_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
        ),

        # ==================== MANUS C++ DRIVER (only if input_source = manus) ====================
        # The wuji_glove path runs in-process inside wujihand_controller via wuji_sdk;
        # no separate input node is needed for it.
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            condition=spawn_manus,
        ),

        # ==================== HAND CONTROLLERS (per-side, parallel processes) ====================
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_left",
            output="screen",
            emulate_tty=True,
            arguments=["--side", "left", "--hand-name", left_hand_name],
        ),
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_right",
            output="screen",
            emulate_tty=True,
            arguments=["--side", "right", "--hand-name", right_hand_name],
        ),
    ])
