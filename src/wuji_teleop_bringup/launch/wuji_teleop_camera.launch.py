"""
Wuji Teleoperation with Camera Launch File / Wuji 遥操作+相机启动文件

Launches all teleoperation components plus camera system.
启动所有遥操作组件以及相机系统。

Usage / 使用方式:
    # Basic usage with default camera config
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py

    # With custom camera config
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py camera_config:=/path/to/config.yaml

    # Disable camera
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py enable_camera:=false
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

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


# 头部双目相机现在通过 camera_launch.py 的 enable_head 参数集成
# Stereo head camera is now integrated via camera_launch.py enable_head parameter
# 通过 IncludeLaunchDescription 集成
# Integrated via IncludeLaunchDescription



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
    hand_config_arg = DeclareLaunchArgument(
        "hand_config",
        default_value=_get_config_path("wujihand_output", "wujihand_ik.yaml"),
        description="Path to wujihand_ik config file",
    )

    # ===== wujihandros2 驱动参数 =====
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

    # Camera arguments
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable camera system",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_config",
        default_value=_get_config_path("camera", "camera_config.yaml"),
        description="Path to camera config file",
    )

    # Stereo head camera arguments
    enable_head_arg = DeclareLaunchArgument(
        "enable_head",
        default_value="true",
        description="Enable stereo head camera ROS2 publisher",
    )
    head_device_arg = DeclareLaunchArgument(
        "head_device",
        default_value="/dev/stereo_camera",
        description="Stereo head camera device path",
    )
    head_fps_arg = DeclareLaunchArgument(
        "head_fps",
        default_value="30",
        description="Stereo head camera frame rate",
    )
    head_quality_arg = DeclareLaunchArgument(
        "head_quality",
        default_value="85",
        description="Stereo head camera JPEG quality (1-100)",
    )

    # Get configurations
    hand_input = LaunchConfiguration("hand_input")
    arm_input = LaunchConfiguration("arm_input")
    enable_rviz = LaunchConfiguration("enable_rviz")
    hand_config = LaunchConfiguration("hand_config")
    enable_camera = LaunchConfiguration("enable_camera")
    camera_config = LaunchConfiguration("camera_config")
    enable_head = LaunchConfiguration("enable_head")
    head_device = LaunchConfiguration("head_device")
    head_fps = LaunchConfiguration("head_fps")
    head_quality = LaunchConfiguration("head_quality")

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
        hand_config_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,
        enable_camera_arg,
        camera_config_arg,
        enable_head_arg,
        head_device_arg,
        head_fps_arg,
        head_quality_arg,

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

        # ==================== CAMERAS (统一入口: camera_launch.py) ====================
        # 头部双目 + 腕部 D405, enable_pico 不传 (SteamVR 方案无 PICO H.264)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("camera"),
                    "launch",
                    "camera_launch.py"
                ])
            ]),
            launch_arguments={
                "config_file": camera_config,
                "enable_head": enable_head,
                "head_device": head_device,
                "head_fps": head_fps,
                "head_quality": head_quality,
                # enable_pico 不传, 默认 false (SteamVR 方案无 PICO H.264 推流)
            }.items(),
            condition=IfCondition(enable_camera),
        ),

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
        Node(
            package="controller",
            executable="tianji_arm_controller",
            name="tianji_arm_controller",
            output="screen",
            emulate_tty=True,
        ),

        # ==================== HAND INPUT: Manus ====================
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            prefix="sudo -E",
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
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
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller",
            output="screen",
            emulate_tty=True,
            arguments=[
                "-c", hand_config,
                "-i", hand_input,
                "--left-hand", LaunchConfiguration("left_hand_name"),
                "--right-hand", LaunchConfiguration("right_hand_name"),
            ],
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
