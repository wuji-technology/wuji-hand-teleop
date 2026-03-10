"""
PICO 遥操作精简模式 - 仅末端控制测试
Minimal PICO Teleoperation - End-Effector Control Only

==================== 启动流程 ====================
1. 自动启动 PC-Service (如果未运行)
2. 显示 IP 信息 → 用户按 Enter 确认
3. 启动 pico_input + tianji_world_output
4. 自动等待节点就绪 → init → 增量控制开始
5. Ctrl+C → 安全下电

==================== 使用方法 ====================
1. 录制数据回放测试 (安全，慢速，默认使用静态数据):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
       data_source_type:=recorded playback_speed:=0.3

2. 使用大数据集回放 (包含明显运动):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
       data_source_type:=recorded playback_speed:=0.3 \
       recorded_file_path:=/home/wuji-08/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/record/trackingData_whole_data.txt

3. 启动真机 (PICO 实时数据):
   ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

==================== 与 step5 的差异 ====================
本文件 (launch) 和 step5_incremental_control_with_robot.py 使用相同的
增量控制算法 (位置+旋转变换)，但在以下方面有区别:

1. 臂角控制:
   - launch: 实时动态臂角。pico_input_node 从 arm tracker 位置计算
     肘部偏移方向，经灰色区间防抖 + EMA 平滑后发布给 tianji_world_output。
     机器人肘部跟随人的实际肘部位置。
   - step5:  静态默认臂角。使用 DEFAULT_ZSP_DIRECTION 固定值，
     不使用 arm tracker 数据。肘部始终维持默认沉肘姿态。

2. 位置平滑:
   - launch: pico_input_node 对位置做 EMA 平滑 (alpha=0.6)，
     减少 tracker 抖动，动作更柔和但响应略慢。
   - step5:  无平滑，直接使用原始计算值。

3. 架构:
   - launch: pico_input → ROS2 topic → tianji_world_output → 机器人
   - step5:  脚本直接调用 CartesianController，无中间节点
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
    """获取配置文件路径"""
    return str(Path(get_package_share_directory(package)) / "config" / config_file)


def _get_default_recorded_file() -> str:
    """获取默认录制数据文件路径 (与 step5 一致，使用静态数据确保安全)"""
    src_dir = Path(__file__).resolve().parents[2]
    record_file = src_dir / "input_devices" / "pico_input" / "record" / "trackingData_sample_static.txt"
    return str(record_file)


# ==================== 预启动检查 (在 generate_launch_description 中执行) ====================

def _ensure_pc_service() -> bool:
    """检查并启动 PC-Service"""
    import time
    r = subprocess.run(["pgrep", "-f", "RoboticsServiceProcess"],
                       capture_output=True, text=True)
    if r.returncode == 0:
        print("  [OK] PC-Service 已在运行")
        return True

    service_dir = "/opt/apps/roboticsservice"
    if not os.path.exists(f"{service_dir}/runService.sh"):
        print("  [ERROR] PC-Service 未安装!")
        return False

    print("  PC-Service 未运行，正在启动...")
    subprocess.Popen(["bash", "runService.sh"], cwd=service_dir,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(10):
        time.sleep(1)
        r = subprocess.run(["pgrep", "-f", "RoboticsServiceProcess"],
                           capture_output=True, text=True)
        if r.returncode == 0:
            print("  [OK] PC-Service 已启动")
            return True
    print("  [WARNING] PC-Service 启动超时")
    return False


def _get_local_ip() -> str:
    """获取本机 IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ==================== 节点启动后的自动初始化脚本 ====================
# 所有输出直接写 /dev/tty，绕过 ROS2 launch 的 stdout 行缓冲
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

# 1. 等待两个节点就绪
tty_print()
tty_print("  等待节点就绪...")
for i in range(30):
    try:
        r = subprocess.run(["ros2", "service", "list"],
                           capture_output=True, text=True, timeout=5)
        if "/pico_input/init" in r.stdout:
            tty_print("  [OK] pico_input 节点已就绪")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(1)
else:
    tty_print("  [ERROR] pico_input 节点未就绪!")
    sys.exit(1)

for i in range(30):
    try:
        r = subprocess.run(["ros2", "node", "list"],
                           capture_output=True, text=True, timeout=5)
        if "/tianji_world_output_node" in r.stdout:
            tty_print("  [OK] tianji_world_output 节点已就绪")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(1)

# 2. 初始化 (重试直到 PICO 数据就绪)
tty_print("  等待 PICO 数据就绪并初始化...")
for attempt in range(60):
    try:
        r = subprocess.run(
            ["ros2", "service", "call", "/pico_input/init", "std_srvs/srv/Trigger"],
            capture_output=True, text=True, timeout=10
        )
        out = r.stdout.lower()
        if "success=true" in out or "success: true" in out:
            tty_print("  [OK] 初始化成功!")
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(2)
else:
    tty_print("  [ERROR] 初始化失败 (PICO 数据未就绪，120秒超时)!")
    sys.exit(1)

tty_print()
tty_print("  ========================================")
tty_print("  ===      增量控制已开启            ===")
tty_print("  ===  Ctrl+C 安全停止               ===")
tty_print("  ========================================")
tty_print()

if _tty:
    _tty.close()
'''


def generate_launch_description() -> LaunchDescription:
    # ==================== 预启动交互 (主进程，stdin 可用) ====================
    print()
    print("=" * 60)
    print("  PICO 遥操作 - 精简模式")
    print("=" * 60)

    _ensure_pc_service()
    print()

    ip = _get_local_ip()
    print(f"  机器人 IP: 192.168.1.190")
    print(f"  本机 IP:   {ip}")
    print()
    print("  请在 PICO 头显 XRoboToolkit App 中填写:")
    print(f"    服务器地址: {ip}")
    print("-" * 60)

    try:
        input("  确认 PICO 已配置好，按 Enter 启动节点...")
    except EOFError:
        pass
    print()

    # ==================== Launch 参数 ====================
    data_source_type_arg = DeclareLaunchArgument(
        "data_source_type",
        default_value="live",
        description="数据源类型: live (真实 PICO) 或 recorded (录制回放)",
    )
    recorded_file_path_arg = DeclareLaunchArgument(
        "recorded_file_path",
        default_value=_get_default_recorded_file(),
        description="录制数据文件路径 (仅 recorded 模式使用)",
    )
    playback_speed_arg = DeclareLaunchArgument(
        "playback_speed",
        default_value="0.3",
        description="回放速度倍率 (0.3 = 慢速安全, 1.0 = 原速)",
    )
    loop_playback_arg = DeclareLaunchArgument(
        "loop_playback",
        default_value="true",
        description="是否循环回放 (true/false)",
    )

    data_source_type = LaunchConfiguration("data_source_type")
    recorded_file_path = LaunchConfiguration("recorded_file_path")
    playback_speed = LaunchConfiguration("playback_speed")
    loop_playback = LaunchConfiguration("loop_playback")

    # ==================== PICO INPUT ====================
    # auto_init_delay=0: 禁用自动初始化，由交互脚本 Enter 控制 (安全)
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

    # ==================== 机器人节点崩溃 → 关闭一切 ====================
    shutdown_on_robot_fail = RegisterEventHandler(
        OnProcessExit(
            target_action=tianji_world_output_node,
            on_exit=[Shutdown(reason="tianji_world_output 退出，终止所有节点")],
        )
    )

    # ==================== 节点启动后的交互流程 ====================
    # 等待 PICO 数据 → Enter → 初始化
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
