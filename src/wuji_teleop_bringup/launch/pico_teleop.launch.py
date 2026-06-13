"""
PICO Teleoperation Unified Launch

  - PICO uses tianji_world_output (world-frame IK).
  - Hand output migrated from wujihand_ik to controller/wujihand_controller (2026-02-28).

Merges the previous pico_teleop.launch.py and pico_preview.launch.py.
Use enable_robot / enable_camera / enable_hand to gate which modules start.

==================== Architecture: fixed world frame ====================

Core design:
  - world = robot base (does not move)
  - The operator stands in front of the robot; coordinates are aligned at init.
  - All trackers publish directly in the world frame.

Coordinate transforms (shared library):
  - Authoritative implementation: tianji_world_output.transform_utils
  - Configuration source: tianji_world.yaml (single source of truth)

==================== Data flow ====================

    PICO SDK --> pico_input_node (frame transforms) --> /left_arm_target_pose
                                                    --> /right_arm_target_pose
                                                    --> /left_arm_elbow_direction
                                                    --> /right_arm_elbow_direction
                                                    --> TF (in world frame)
                                                           |
    tianji_world_output_node: subscribes target_pose --> IK --> Tianji arm

    Wuji Glove (default, UDP) / MANUS (optional, /manus_glove_{0,1}) --> wujihand_controller --> Wuji hand

    HBVCAM stereo (head, USB UVC) -> unified_stereo
      |- ROS2 image topics
      `- FFmpeg H.264 -> TCP -> PICO headset (single process, no v4l2loopback)

    RealSense D405 (wrists) -> ROS2 compressed topics (30fps)

==================== Usage ====================

    # Real-hardware mode (default: launch every module)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # Preview mode (input + visualization only; no robot control)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py \\
      enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

    # Manual re-initialization
    ros2 service call /pico_input/init std_srvs/srv/Trigger
    ros2 service call /pico_input/reset std_srvs/srv/Trigger """

from pathlib import Path
import yaml
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
from wuji_teleop_bringup.launch_utils import resolve_config_path as _get_config


def _read_input_source() -> str:
    """Read input_source from wujihand_ik.yaml at launch evaluation time."""
    yaml_path = Path(_get_config("wujihand_output", "wujihand_ik.yaml"))
    try:
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return "wuji_glove"
    src = cfg.get("input_source", "wuji_glove")
    if src not in ("wuji_glove", "manus"):
        raise ValueError(f"wujihand_ik.yaml::input_source = {src!r} unsupported")
    return src


def generate_launch_description() -> LaunchDescription:
    # ==================== Module switches ====================
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

    # ==================== Dexterous-hand driver params ====================
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

    # ==================== Read parameters ====================
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

    # ==================== Startup banner ====================
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

    # ==================== CAMERAS (single entry point: camera_launch.py) ====================
    # camera_launch.py manages every camera:
    #   - Head: HBVCAM stereo (USB UVC) -> unified_stereo, one process emits both
    #           ROS2 image topics AND FFmpeg H.264 -> TCP for the PICO headset.
    #   - Wrists: RealSense D405 (ROS2, 30fps).
    # Config: camera_config.yaml (device paths, serials, resolution, etc.).
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

    # ==================== HAND INPUT: MANUS C++ driver (input_source==manus) ====================
    input_source = _read_input_source()
    manus_data_publisher = Node(
        package="manus_ros2",
        executable="manus_data_publisher",
        name="manus_data_publisher",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_hand) if input_source == "manus" else IfCondition("false"),
    )

    # ==================== HAND CONTROLLERS (per-side, parallel processes) ====================
    wujihand_controller_left = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_left",
        output="screen",
        emulate_tty=True,
        arguments=["--side", "left", "--hand-name", LaunchConfiguration("left_hand_name"),
                   "--config", hand_config],
        condition=IfCondition(enable_hand),
    )
    wujihand_controller_right = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_right",
        output="screen",
        emulate_tty=True,
        arguments=["--side", "right", "--hand-name", LaunchConfiguration("right_hand_name"),
                   "--config", hand_config],
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
        wujihand_controller_left,
        wujihand_controller_right,

        # RViz (conditional)
        rviz_node,
    ])
