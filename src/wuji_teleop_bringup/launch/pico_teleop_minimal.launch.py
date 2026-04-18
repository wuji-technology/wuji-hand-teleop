"""
PICO Teleoperation Minimal Mode - End-Effector Control Only

==================== Startup Process ====================
1. Auto-start PC-Service (if not running)
2. Display IP info -> user presses Enter to confirm
3. Launch pico_input + tianji_world_output
4. Auto-wait for nodes ready -> init -> incremental control starts
5. Ctrl+C -> safe power-off

==================== Usage ====================
1. Recorded data playback test (safe, slow, uses static data by default):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
       data_source_type:=recorded playback_speed:=0.3

2. Use large dataset playback (with noticeable motion):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
       data_source_type:=recorded playback_speed:=0.3 \
       recorded_file_path:=~/Desktop/wuji-hand-teleop/src/input_devices/pico_input/record/trackingData_whole_data.txt

3. Launch with real robot (PICO live data):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

==================== Differences from step5 ====================
This file (launch) and step5_incremental_control_with_robot.py use the same
incremental control algorithm (position + rotation transform), but differ in:

1. Arm angle control:
   - launch: Real-time dynamic arm angle. pico_input_node computes elbow
     offset direction from arm tracker position, with dead-zone debounce
     + EMA smoothing, then publishes to tianji_world_output.
     Robot elbow follows the human's actual elbow position.
   - step5:  Static default arm angle. Uses fixed DEFAULT_ZSP_DIRECTION values,
     does not use arm tracker data. Elbow always maintains default pose.

2. Position smoothing:
   - launch: pico_input_node applies EMA smoothing to position (alpha=0.6),
     reduces tracker jitter, smoother motion but slightly slower response.
   - step5:  No smoothing, directly uses raw computed values.

3. Architecture:
   - launch: pico_input -> ROS2 topic -> tianji_world_output -> robot
   - step5:  Script directly calls CartesianController, no intermediate nodes
"""

import os
import socket
import subprocess
import sys

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_config(package: str, config_file: str) -> str:
    """Get config file path"""
    return str(Path(get_package_share_directory(package)) / "config" / config_file)


def _get_default_recorded_file() -> str:
    """Get default recorded data file path (consistent with step5, uses static data for safety)"""
    src_dir = Path(__file__).resolve().parents[2]
    record_file = src_dir / "input_devices" / "pico_input" / "record" / "trackingData_sample_static.txt"
    return str(record_file)


# ==================== Pre-launch checks (executed in generate_launch_description) ====================

def _ensure_pc_service() -> bool:
    """Check and start PC-Service"""
    import time
    r = subprocess.run(["pgrep", "-f", "RoboticsServiceProcess"],
                       capture_output=True, text=True)
    if r.returncode == 0:
        print("  [OK] PC-Service is already running")
        return True

    service_dir = "/opt/apps/roboticsservice"
    if not os.path.exists(f"{service_dir}/runService.sh"):
        print("  [ERROR] PC-Service not installed!")
        return False

    print("  PC-Service not running, starting...")
    subprocess.Popen(["bash", "runService.sh"], cwd=service_dir,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(10):
        time.sleep(1)
        r = subprocess.run(["pgrep", "-f", "RoboticsServiceProcess"],
                           capture_output=True, text=True)
        if r.returncode == 0:
            print("  [OK] PC-Service started")
            return True
    print("  [WARNING] PC-Service startup timed out")
    return False


def _get_local_ip() -> str:
    """Get local IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ==================== Post-launch auto-initialization script ====================
# All output written directly to /dev/tty, bypassing ROS2 launch stdout line buffering
_POST_LAUNCH_SCRIPT = r'''
import subprocess, sys, time

_tty = None
try:
    _tty = open("/dev/tty", "w")
except Exception:
    pass

def tty_print(text=""):
    if _tty:
        _tty.write(text + "\n")
        _tty.flush()
    else:
        print(text, flush=True)

# 1. Wait for both nodes to be ready
tty_print()
tty_print("  Waiting for nodes to be ready...")
for i in range(30):
    try:
        r = subprocess.run(["ros2", "service", "list"],
                           capture_output=True, text=True, timeout=5)
        if "/pico_input/init" in r.stdout:
            tty_print("  [OK] pico_input node is ready")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(1)
else:
    tty_print("  [ERROR] pico_input node not ready!")
    sys.exit(1)

for i in range(30):
    try:
        r = subprocess.run(["ros2", "node", "list"],
                           capture_output=True, text=True, timeout=5)
        if "/tianji_world_output_node" in r.stdout:
            tty_print("  [OK] tianji_world_output node is ready")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(1)

# 2. Initialize (retry until PICO data is ready)
tty_print("  Waiting for PICO data and initializing...")
for attempt in range(60):
    try:
        r = subprocess.run(
            ["ros2", "service", "call", "/pico_input/init", "std_srvs/srv/Trigger"],
            capture_output=True, text=True, timeout=10
        )
        out = r.stdout.lower()
        if "success=true" in out or "success: true" in out:
            tty_print("  [OK] Initialization successful!")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(2)
else:
    tty_print("  [ERROR] Initialization failed (PICO data not ready, 120s timeout)!")
    sys.exit(1)

tty_print()
tty_print("  ========================================")
tty_print("  ===  Incremental control started    ===")
tty_print("  ===  Ctrl+C to stop safely          ===")
tty_print("  ========================================")
tty_print()

if _tty:
    _tty.close()
'''


def generate_launch_description() -> LaunchDescription:
    # ==================== Pre-launch interaction (main process, stdin available) ====================
    print()
    print("=" * 60)
    print("  PICO Teleoperation - Minimal Mode")
    print("=" * 60)

    _ensure_pc_service()
    print()

    ip = _get_local_ip()
    print(f"  Robot IP: 192.168.1.190")
    print(f"  Local IP: {ip}")
    print()
    print("  Please enter in the PICO headset XRoboToolkit App:")
    print(f"    Server address: {ip}")
    print("-" * 60)

    try:
        input("  Confirm PICO is configured, press Enter to launch nodes...")
    except EOFError:
        pass
    print()

    # ==================== Launch Arguments ====================
    data_source_type_arg = DeclareLaunchArgument(
        "data_source_type",
        default_value="live",
        description="Data source type: live (real PICO) or recorded (recorded playback)",
    )
    recorded_file_path_arg = DeclareLaunchArgument(
        "recorded_file_path",
        default_value=_get_default_recorded_file(),
        description="Recorded data file path (used only in recorded mode)",
    )
    playback_speed_arg = DeclareLaunchArgument(
        "playback_speed",
        default_value="0.3",
        description="Playback speed multiplier (0.3 = slow safe, 1.0 = original speed)",
    )
    loop_playback_arg = DeclareLaunchArgument(
        "loop_playback",
        default_value="true",
        description="Whether to loop playback (true/false)",
    )

    data_source_type = LaunchConfiguration("data_source_type")
    recorded_file_path = LaunchConfiguration("recorded_file_path")
    playback_speed = LaunchConfiguration("playback_speed")
    loop_playback = LaunchConfiguration("loop_playback")

    # ==================== PICO INPUT ====================
    # auto_init_delay=0: Disable auto-init, controlled by interactive script Enter (safe)
    pico_input_node = Node(
        package="pico_input",
        executable="pico_input_node",
        name="pico_input_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            _get_config("pico_input", "pico_input.yaml"),
            {
                "data_source_type": data_source_type,
                "recorded_file_path": recorded_file_path,
                "playback_speed": playback_speed,
                "loop_playback": loop_playback,
                "auto_init_delay": 0.0,
            },
        ],
    )

    # ==================== ARM OUTPUT ====================
    tianji_world_output_node = Node(
        package="tianji_world_output",
        executable="tianji_world_output_node",
        name="tianji_world_output_node",
        output="screen",
        emulate_tty=True,
        sigterm_timeout='10',
    )

    # ==================== Robot node crash -> shut down everything ====================
    shutdown_on_robot_fail = RegisterEventHandler(
        OnProcessExit(
            target_action=tianji_world_output_node,
            on_exit=[Shutdown(reason="tianji_world_output exited, terminating all nodes")],
        )
    )

    # ==================== Post-launch interactive flow ====================
    # Wait for PICO data -> Enter -> initialize
    post_launch_interactive = ExecuteProcess(
        cmd=['python3', '-c', _POST_LAUNCH_SCRIPT],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        data_source_type_arg,
        recorded_file_path_arg,
        playback_speed_arg,
        loop_playback_arg,
        pico_input_node,
        tianji_world_output_node,
        shutdown_on_robot_fail,
        post_launch_interactive,
    ])
