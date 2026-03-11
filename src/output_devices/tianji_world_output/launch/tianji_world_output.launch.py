"""
Tianji World Output Launch File (ROS REP 103 Compliant)
天机臂世界坐标系输出启动文件（符合 ROS REP 103 标准）

使用方法：
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
            "control_rate": 90.0,  # 控制频率 (Hz)
            "vel_ratio": 60,        # 速度比例 (%)
            "acc_ratio": 60,        # 加速度比例 (%)
            "enable_debug_log": False,
        }],
    )

    return LaunchDescription([
        tianji_world_output_node,
    ])
