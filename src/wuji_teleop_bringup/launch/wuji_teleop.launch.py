"""Full teleop: cameras + hands + arms (HTC Vive Tracker path).

Hand input source is read from wujihand_ik.yaml::input_source (no launch arg).
Args: arm_input={tracker,pico}, enable_hand, enable_arm, enable_camera, enable_rviz.

PICO is its own launch file (`pico_teleop.launch.py`) — `arm_input:=pico` here
fails fast at evaluation time rather than silently spawning tianji_arm_controller
with no TF source.

enable_arm defaults to "true" here so CLI `ros2 launch ... wuji_teleop.launch.py`
keeps the hand+arm contract documented in the README. The Monitor GUI passes
`enable_arm:=false` by default (hand-first) and lets the operator opt in.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)
from wuji_teleop_bringup.launch_utils import (
    read_input_source as _read_input_source,
    resolve_config_path as _get_config_path,
)
from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _get_rviz_path() -> str:
    """Get the path to the RViz config file."""
    share_dir = Path(get_package_share_directory("openvr_input"))
    return str(share_dir / "rviz" / "openvr_visualization.rviz")


def _reject_pico_arm_input(context, *args, **kwargs):
    """Fail fast if the operator asked for the PICO arm here.

    wuji_teleop.launch.py only knows about the HTC Vive Tracker path
    (openvr_input -> tianji_arm_controller, TF-based). The PICO path
    (pico_input -> tianji_world_output_node, topic-based) lives in
    pico_teleop.launch.py. With `arm_input:=pico` the arm-controller
    spawns with no TF source and the arm sits dead — the QA report's
    silent-failure mode. Raise instead, with a pointer to the right
    launch file.
    """
    enable_arm = context.launch_configurations.get("enable_arm", "true")
    arm_input = context.launch_configurations.get("arm_input", "tracker")
    if enable_arm.lower() == "true" and arm_input == "pico":
        raise RuntimeError(
            "wuji_teleop.launch.py does not support `arm_input:=pico` — "
            "the PICO arm pipeline (pico_input + tianji_world_output_node) "
            "is wired up in pico_teleop.launch.py instead. Run:\n"
            "    ros2 launch wuji_teleop_bringup pico_teleop.launch.py\n"
            "or pass `enable_arm:=false` here and bring up the PICO arm "
            "controllers separately."
        )
    return []


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    arm_input_arg = DeclareLaunchArgument(
        "arm_input",
        default_value="tracker",
        description="Arm input device: 'tracker' (HTC Vive Trackers) or 'pico'",
    )
    enable_hand_arg = DeclareLaunchArgument(
        "enable_hand",
        default_value="true",
        description="Enable hand input + Wuji Hand retargeting (input source via wujihand_ik.yaml)",
    )
    enable_arm_arg = DeclareLaunchArgument(
        "enable_arm",
        default_value="true",
        description="Enable arm input (openvr_input when arm_input==tracker) + tianji_arm_controller",
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable HBVCAM stereo head + D405 wrists + PICO H.264 stream",
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

    # wujihandros2 driver parameters
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

    enable_hand = LaunchConfiguration("enable_hand")
    enable_arm = LaunchConfiguration("enable_arm")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_rviz = LaunchConfiguration("enable_rviz")

    # openvr_input only when arm AND tracker arm_input: PythonExpression
    # is the canonical launch primitive for boolean AND across multiple
    # LaunchConfiguration values.
    arm_tracker_condition = IfCondition(PythonExpression([
        "'", enable_arm, "' == 'true' and '",
        LaunchConfiguration("arm_input"), "' == 'tracker'",
    ]))
    hand_config = LaunchConfiguration("hand_config")
    left_hand_name = LaunchConfiguration("left_hand_name")
    right_hand_name = LaunchConfiguration("right_hand_name")

    # input_source is read once at launch evaluation; it decides whether to
    # spawn the manus driver. wuji_glove path runs in-process inside
    # wujihand_controller via wuji_sdk — no extra input node needed.
    input_source = _read_input_source()

    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str
    )
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str
    )

    return LaunchDescription([
        # Arguments
        arm_input_arg,
        enable_hand_arg,
        enable_arm_arg,
        enable_camera_arg,
        enable_rviz_arg,
        hand_config_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,

        # Reject `arm_input:=pico` before any node spawns.
        OpaqueFunction(function=_reject_pico_arm_input),

        # ==================== CAMERAS ====================
        # Head HBVCAM stereo (RGB) + D405 wrists. The unified_stereo process
        # inside camera_launch.py publishes both ROS2 image topics AND the
        # PICO H.264 video stream from the head camera in one process.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('camera'), 'launch', 'camera_launch.py'
                ])
            ]),
            condition=IfCondition(enable_camera),
        ),

        # ==================== WUJIHANDROS2 DRIVERS ====================
        # Per-side wujihand SDK driver; wujihand_controller talks to it.
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
            condition=IfCondition(enable_hand),
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
            condition=IfCondition(enable_hand),
        ),

        # ==================== ARM INPUT ====================
        Node(
            package="openvr_input",
            executable="openvr_input",
            name="openvr_input",
            output="screen",
            arguments=["-c", _get_config_path("openvr_input", "openvr_input.yaml")],
            condition=arm_tracker_condition,
        ),
        # ==================== STATIC TF ====================
        # Chest TF is needed for hand IK regardless of arm state; tianji TF is
        # only needed when the arm pipeline is active.
        OpaqueFunction(function=lambda ctx: create_chest_tf_nodes()),
        OpaqueFunction(function=lambda ctx: (
            create_tianji_tf_nodes()
            if ctx.launch_configurations.get("enable_arm", "true").lower() == "true"
            else []
        )),

        # ==================== ARM OUTPUT: Tianji ====================
        Node(
            package="controller",
            executable="tianji_arm_controller",
            name="tianji_arm_controller",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(enable_arm),
        ),

        # ==================== HAND INPUT: MANUS C++ driver ====================
        # Only spawned when wujihand_ik.yaml::input_source == "manus".
        # The wuji_glove path runs in-process inside wujihand_controller via wuji_sdk.
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(enable_hand) if input_source == "manus" else IfCondition("false"),
        ),

        # ==================== HAND CONTROLLERS (per-side, parallel) ====================
        # Two independent processes for multi-core IK + retargeting; each
        # reads input_source from wujihand_ik.yaml and dispatches internally.
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_left",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--side", "left",
                "--hand-name", left_hand_name,
                "--config", hand_config,
            ],
            condition=IfCondition(enable_hand),
        ),
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_right",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--side", "right",
                "--hand-name", right_hand_name,
                "--config", hand_config,
            ],
            condition=IfCondition(enable_hand),
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
