"""
Tianji World Output Launch File (ROS REP 103 Compliant)
Tianji arm world coordinate system output launch file (ROS REP 103 compliant)

Usage:
    ros2 launch tianji_world_output tianji_world_output.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    tianji_world_output_node = Node(
        package="tianji_world_output",
        executable="tianji_world_output_node",
        name="tianji_world_output_node",
        output="screen",
        parameters=[{
            "control_rate": 90.0,  # Control frequency (Hz)
            "vel_ratio": 60,        # Velocity ratio (%)
            "acc_ratio": 60,        # Acceleration ratio (%)
            "enable_debug_log": False,
        }],
    )

    return LaunchDescription([
        tianji_world_output_node,
    ])
