"""
PICO Teleoperation Unified Launch

  - PICO scheme uses tianji_world_output (world coordinate IK), SteamVR scheme uses tianji_output (chest coordinate IK)
  - Hand output migrated from wujihand_ik to controller/wujihand_controller (2026-02-28)

Merges the original pico_teleop.launch.py and pico_preview.launch.py.
Controls which modules to launch via enable_robot / enable_camera / enable_hand parameters.

==================== Architecture: Fixed World Coordinate Frame ====================

Core design:
  - world = robot base (fixed)
  - User stands in front of robot, coordinate frames aligned at initialization
  - All trackers publish directly in world coordinate frame

Coordinate transforms (unified shared library):
  - Authoritative implementation: tianji_world_output.transform_utils
  - Config source: tianji_robot.yaml (Single Source of Truth)

==================== Data Flow Architecture ====================

    PICO SDK --> pico_input_node (coordinate transform) --> /left_arm_target_pose
                                                        --> /right_arm_target_pose
                                                        --> /left_arm_elbow_direction
                                                        --> /right_arm_elbow_direction
                                                        --> TF (in world frame)
                                                               |
    tianji_world_output_node: subscribe target_pose --> IK --> Tianji arms

    MANUS --> /hand_input --> wujihand_retargeting --> Wuji hands

    unified_stereo: /dev/stereo_camera -> OpenCV (MJPEG)
      +-- ROS2: /stereo/{left,right}/compressed (30fps JPEG)
      +-- PICO: H.264 60fps via XRobo TCP (on-demand)

    RealSense D405 (wrist) -> ROS2 compressed topics (30fps)

==================== Usage ====================

    # Real robot mode (default: launch all modules)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # Preview mode (input + visualization only, no robot control)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py \\
      enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

    # Manual re-initialization
    ros2 service call /pico_input/init std_srvs/srv/Trigger
    ros2 service call /pico_input/reset std_srvs/srv/Trigger """

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
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


def _get_config(package: str, config_file: str) -> str:
    return str(Path(get_package_share_directory(package)) / "config" / config_file)


def generate_launch_description() -> LaunchDescription:
    # ==================== Module Switches ====================
    enable_robot_arg = DeclareLaunchArgument(
        "enable_robot", default_value="true",
        description="Enable tianji arm output. Set false for preview mode."
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera", default_value="true",
        description="Enable stereo camera capture and PICO video streaming."
    )
    enable_hand_arg = DeclareLaunchArgument(
        "enable_hand", default_value="true",
        description="Enable MANUS glove input and wuji hand output."
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz", default_value="false",
        description="Enable RViz visualization."
    )
    hand_config_arg = DeclareLaunchArgument(
        "hand_config", default_value=_get_config("wujihand_output", "wujihand_ik.yaml")
    )

    # ==================== Dexterous Hand Driver Parameters ====================
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

    # ==================== Read Parameters ====================
    enable_robot = LaunchConfiguration("enable_robot")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_hand = LaunchConfiguration("enable_hand")
    enable_rviz = LaunchConfiguration("enable_rviz")
    hand_config = LaunchConfiguration("hand_config")

    # Force serial_number to string type (workaround for ROS2 type inference)
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str
    )
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str
    )

    # ==================== Startup Banner ====================
    startup_banner = LogInfo(
        msg="""
========================================================================
  PICO Teleoperation Launch
========================================================================
  Parameters:
    enable_robot  - Tianji arm output
    enable_camera - Stereo camera + PICO video (unified, no v4l2loopback)
    enable_hand   - MANUS glove input + wuji hand output
    enable_rviz   - RViz visualization

  Preview mode (no robot control):
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py \\
      enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
========================================================================
"""
    )

    # ==================== CAMERAS (unified entry: camera_launch.py) ====================
    # camera_launch.py manages all cameras:
    #   - Head stereo: unified_stereo (ROS2 30fps + PICO H.264 60fps on-demand)
    #   - Left/right wrist: RealSense D405 (ROS2 30fps)
    # Config: camera_config.yaml (device paths, serial numbers, resolution, etc.)
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera'), 'launch', 'camera_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_head': 'true',
            'enable_pico': 'true',
        }.items(),
        condition=IfCondition(enable_camera),
    )

    # ==================== PICO INPUT (always on) ====================
    pico_input_node = Node(
        package="pico_input",
        executable="pico_input_node",
        name="pico_input_node",
        output="screen",
        parameters=[_get_config("pico_input", "pico_input.yaml")],
    )

    # ==================== ARM OUTPUT (enable_robot) ====================
    tianji_world_output_node = Node(
        package="tianji_world_output",
        executable="tianji_world_output_node",
        name="tianji_world_output_node",
        output="screen",
        condition=IfCondition(enable_robot),
    )

    # ==================== HAND INPUT: MANUS (enable_hand) ====================
    manus_data_publisher = Node(
        package="manus_ros2",
        executable="manus_data_publisher",
        name="manus_data_publisher",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_hand),
    )
    # ==================== HAND OUTPUT (enable_hand) ====================
    # Per-hand controller processes (multi-core parallel, no shared GIL)
    wujihand_retargeting_left = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_left",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--side", "left",
            "--hand-name", LaunchConfiguration("left_hand_name"),
        ],
        condition=IfCondition(enable_hand),
    )
    wujihand_retargeting_right = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_right",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--side", "right",
            "--hand-name", LaunchConfiguration("right_hand_name"),
        ],
        condition=IfCondition(enable_hand),
    )

    # ==================== VISUALIZATION (enable_rviz) ====================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(Path(get_package_share_directory("pico_input")) / "rviz" / "pico_visualization.rviz")],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        # Parameters
        enable_robot_arg,
        enable_camera_arg,
        enable_hand_arg,
        enable_rviz_arg,
        hand_config_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,

        # Banner
        startup_banner,

        # All cameras (head + wrist, via camera_launch.py)
        cameras,

        # PICO input (always on)
        pico_input_node,

        # Arm output (conditional)
        tianji_world_output_node,

        # Hand driver (conditional)
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
            condition=IfCondition(enable_hand),
        ),
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
            condition=IfCondition(enable_hand),
        ),

        # Hand input + output (conditional)
        manus_data_publisher,
        wujihand_retargeting_left,
        wujihand_retargeting_right,

        # RViz (conditional)
        rviz_node,
    ])
