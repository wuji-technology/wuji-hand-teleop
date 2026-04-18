"""
PICO Input Launch - Standalone launch for PICO data acquisition

By default only publishes TF, for testing PICO connection.
To control the robot use: pico_teleop.launch.py

Usage:
    ros2 launch pico_input pico_input.launch.py
    ros2 launch pico_input pico_input.launch.py enable_topic_publishing:=true  # Publish target_pose and other topics
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = str(Path(get_package_share_directory("pico_input")) / "config" / "pico_input.yaml")

    enable_topic_publishing_arg = DeclareLaunchArgument(
        "enable_topic_publishing",
        default_value="false",
        description="Enable topic publishing (/left_arm_target_pose etc., for RViz/simulation)"
    )

    return LaunchDescription([
        enable_topic_publishing_arg,
        Node(
            package="pico_input",
            executable="pico_input_node",
            name="pico_input_node",
            output="screen",
            parameters=[
                config,
                {"enable_topic_publishing": LaunchConfiguration("enable_topic_publishing")},
            ],
        ),
    ])
