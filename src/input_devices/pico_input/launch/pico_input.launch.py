"""
PICO Input Launch - 独立启动 PICO 数据采集

默认仅发布 TF，用于测试 PICO 连接。
要控制机器人需使用: pico_teleop.launch.py

Usage:
    ros2 launch pico_input pico_input.launch.py
    ros2 launch pico_input pico_input.launch.py enable_topic_publishing:=true  # 发布 target_pose 等 topic
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
        description="启用 Topic 发布 (/left_arm_target_pose 等，用于 RViz/仿真)"
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
