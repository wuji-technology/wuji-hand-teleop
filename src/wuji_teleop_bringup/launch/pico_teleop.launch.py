"""
PICO 遥操作统一 Launch / PICO Teleoperation Unified Launch

  - PICO 方案使用 tianji_world_output (世界坐标系 IK), SteamVR 方案使用 tianji_output (胸部坐标系 IK)
  - 手部输出从 wujihand_ik 迁移到 controller/wujihand_controller (2026-02-28)

合并了原来的 pico_teleop.launch.py 和 pico_preview.launch.py。
通过 enable_robot / enable_camera / enable_hand 参数控制启动哪些模块。

==================== 架构: 固定世界坐标系 ====================

核心设计:
  - world = 机器人基座 (固定不动)
  - 用户站在机器人前面，初始化时对齐坐标系
  - 所有 tracker 直接发布在 world 坐标系下

坐标变换 (统一共享库):
  - 权威实现: tianji_world_output.transform_utils
  - 配置来源: tianji_robot.yaml (Single Source of Truth)

==================== 数据流架构 ====================

    PICO SDK --> pico_input_node (坐标变换) --> /left_arm_target_pose
                                            --> /right_arm_target_pose
                                            --> /left_arm_elbow_direction
                                            --> /right_arm_elbow_direction
                                            --> TF (world 下)
                                                   |
    tianji_world_output_node: 订阅 target_pose --> IK --> 天机臂

    MANUS --> /hand_input --> wujihand_retargeting --> 舞肌手

    unified_stereo: /dev/stereo_camera → OpenCV (MJPEG)
      ├── ROS2: /stereo/{left,right}/compressed (30fps JPEG)
      └── PICO: H.264 60fps via XRobo TCP (on-demand)

    RealSense D405 (腕部) → ROS2 compressed topics (30fps)

==================== 使用方法 ====================

    # 真机模式 (默认: 启动所有模块)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # 预检模式 (仅输入 + 可视化，不控制机器人)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py \\
      enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

    # 手动重新初始化
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
    # ==================== 模块开关 ====================
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

    # ==================== 灵巧手驱动参数 ====================
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

    # ==================== 读取参数 ====================
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

    # ==================== 启动提示 ====================
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

    # ==================== CAMERAS (统一入口: camera_launch.py) ====================
    # camera_launch.py 统一管理所有相机:
    #   - 头部双目: unified_stereo (ROS2 30fps + PICO H.264 60fps on-demand)
    #   - 左右腕部: RealSense D405 (ROS2 30fps)
    # 配置: camera_config.yaml (设备路径、序列号、分辨率等)
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

    # ==================== PICO INPUT (始终启动) ====================
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
        prefix="sudo -E",
        condition=IfCondition(enable_hand),
    )
    manus_input = Node(
        package="manus_input_py",
        executable="manus_input",
        name="manus_input",
        output="screen",
        condition=IfCondition(enable_hand),
    )

    # ==================== HAND OUTPUT (enable_hand) ====================
    wujihand_retargeting = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller",
        output="screen",
        arguments=["-c", hand_config, "-i", "manus"],
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
        manus_input,
        wujihand_retargeting,

        # RViz (conditional)
        rviz_node,
    ])
