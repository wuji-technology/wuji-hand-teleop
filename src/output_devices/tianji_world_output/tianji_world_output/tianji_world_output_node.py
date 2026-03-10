"""
ROS2 node that subscribes to pose topics and controls Tianji arms (Simplified).
订阅位姿 topic 并控制天机臂的 ROS2 节点（简化版，无 TF 依赖）。

Data Flow (Simplified):
  pico_input → /left_arm_target_pose → tianji_world_output → IK → Robot
  pico_input → /right_arm_target_pose → tianji_world_output → IK → Robot

This node:
  1. Subscribes to target pose topics (geometry_msgs/PoseStamped)
  2. Sends IK commands to Tianji arms
  3. No TF tree dependency - direct topic communication

Advantages:
  - Simple and direct data flow
  - No TF latency or complexity
  - Easy to debug and maintain
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R

from tianji_world_output.cartesian_controller import CartesianController
from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.ros2_logging import ROS2LoggerAdapter, setup_ros2_logging_bridge

# 日志目录
LOG_DIR = Path.home() / ".wuji_teleop_logs"


class TianjiWorldOutputNode(Node):
    """ROS2 node that subscribes to pose topics and controls Tianji arms (Simplified)."""

    def __init__(self, robot_ip: str = '192.168.1.190'):
        super().__init__("tianji_world_output")

        # 安装 stdlib→ROS2 日志桥接 (使非 Node 类日志进入 /rosout)
        setup_ros2_logging_bridge(self.get_logger())

        # 参数
        self.declare_parameter('control_rate', 90.0)
        self.declare_parameter('vel_ratio', 60)
        self.declare_parameter('acc_ratio', 60)

        control_rate = self.get_parameter('control_rate').value
        vel_ratio = int(self.get_parameter('vel_ratio').value)
        acc_ratio = int(self.get_parameter('acc_ratio').value)

        # 专用详细日志文件 (始终开启, line-buffered, 防止 Ctrl+C 截断)
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._detail_log_path = LOG_DIR / f'tianji_output_{ts}.log'
        self._detail_log = None
        try:
            self._detail_log = open(self._detail_log_path, 'w', buffering=1)  # line buffering
            self._detail_log.write(f"# Tianji World Output detailed log - {ts}\n")
            self._detail_log.write(f"# IK状态 + 双臂位姿 + 关节角 + zsp_para\n")
            self.get_logger().info(f'详细日志: {self._detail_log_path}')
        except OSError as e:
            self.get_logger().error(f'无法创建详细日志文件: {e}')

        # 创建控制器
        logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self.get_logger().info(f"Connecting to robot at {robot_ip}...")
        self.controller = CartesianController(robot_ip=robot_ip, logger=logger_adapter)
        self.controller.set_impedance_mode(mode='joint')

        # 设置速度/加速度
        if vel_ratio != 60 or acc_ratio != 60:
            self.get_logger().info(f"设置速度比例: vel={vel_ratio}%, acc={acc_ratio}%")
            self.controller.robot.clear_set()
            self.controller.robot.set_vel_acc(arm='A', velRatio=vel_ratio, AccRatio=acc_ratio)
            self.controller.robot.set_vel_acc(arm='B', velRatio=vel_ratio, AccRatio=acc_ratio)
            self.controller.robot.send_cmd()
            time.sleep(0.3)

        self.controller.move_to_init(wait=True, timeout=3)

        # 存储位姿和方向（输入已经是 chest 坐标系，无需转换）
        self.left_arm_pose = None
        self.right_arm_pose = None
        # 初始化默认沉肘方向 — 从 tianji_robot.yaml 统一加载 (Single Source of Truth)
        config = TianjiConfig.load()
        self.left_arm_direction = config.get_default_zsp_direction('left')
        self.right_arm_direction = config.get_default_zsp_direction('right')
        self.get_logger().info(
            f"从配置加载默认臂角: left={self.left_arm_direction}, right={self.right_arm_direction}"
        )

        # 订阅目标位姿 topics
        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            '/left_arm_target_pose',
            self.left_pose_callback,
            10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            '/right_arm_target_pose',
            self.right_pose_callback,
            10
        )

        # 订阅肘部方向 topics (可选)
        self.left_elbow_sub = self.create_subscription(
            Vector3Stamped,
            '/left_arm_elbow_direction',
            self.left_elbow_callback,
            10
        )
        self.right_elbow_sub = self.create_subscription(
            Vector3Stamped,
            '/right_arm_elbow_direction',
            self.right_elbow_callback,
            10
        )

        # 控制循环
        control_period = 1.0 / control_rate
        self.timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info("Tianji World Output node initialized (Topic-based, no TF).")
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /left_arm_target_pose")
        self.get_logger().info("  - /right_arm_target_pose")
        self.get_logger().info("  - /left_arm_elbow_direction (optional)")
        self.get_logger().info("  - /right_arm_elbow_direction (optional)")

        # 首次消息标志 (用于调试)
        self._first_pose_received = False

        # 诊断日志计数器 (每秒一次)
        self._debug_counter = 0
        self._debug_interval = int(control_rate)

    def left_pose_callback(self, msg: PoseStamped):
        """左臂目标位姿回调（输入已是 chest 坐标系）"""
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("✓ 首次收到位姿数据，开始控制...")
        self.left_arm_pose = self._pose_to_matrix(msg.pose)

    def right_pose_callback(self, msg: PoseStamped):
        """右臂目标位姿回调（输入已是 chest 坐标系）"""
        if not self._first_pose_received:
            self._first_pose_received = True
            self.get_logger().info("✓ 首次收到位姿数据，开始控制...")
        self.right_arm_pose = self._pose_to_matrix(msg.pose)

    def left_elbow_callback(self, msg: Vector3Stamped):
        """左臂肘部方向回调"""
        new_dir = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        # 记录显著变化 (>5°) 到详细日志
        dot = np.clip(np.dot(new_dir, self.left_arm_direction), -1.0, 1.0)
        angle_change = np.degrees(np.arccos(dot))
        if angle_change > 5.0:
            self._write_detail_log(
                f"[ZSP变化] A臂: [{self.left_arm_direction[0]:.3f},{self.left_arm_direction[1]:.3f},{self.left_arm_direction[2]:.3f}]"
                f" → [{new_dir[0]:.3f},{new_dir[1]:.3f},{new_dir[2]:.3f}] Δ={angle_change:.1f}°"
            )
        self.left_arm_direction = new_dir

    def right_elbow_callback(self, msg: Vector3Stamped):
        """右臂肘部方向回调"""
        new_dir = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        # 记录显著变化 (>5°) 到详细日志
        dot = np.clip(np.dot(new_dir, self.right_arm_direction), -1.0, 1.0)
        angle_change = np.degrees(np.arccos(dot))
        if angle_change > 5.0:
            self._write_detail_log(
                f"[ZSP变化] B臂: [{self.right_arm_direction[0]:.3f},{self.right_arm_direction[1]:.3f},{self.right_arm_direction[2]:.3f}]"
                f" → [{new_dir[0]:.3f},{new_dir[1]:.3f},{new_dir[2]:.3f}] Δ={angle_change:.1f}°"
            )
        self.right_arm_direction = new_dir

    def control_loop(self) -> None:
        """主控制循环：发送控制指令"""

        # 发送控制指令
        if self.left_arm_pose is not None or self.right_arm_pose is not None:
            # 更新 zsp_para (肘部臂角控制) - 始终使用当前 elbow direction
            self.controller.left_zsp_para = [
                self.left_arm_direction[0],
                self.left_arm_direction[1],
                self.left_arm_direction[2],
                0, 0, 0
            ]

            self.controller.right_zsp_para = [
                self.right_arm_direction[0],
                self.right_arm_direction[1],
                self.right_arm_direction[2],
                0, 0, 0
            ]

            # 执行 IK 并发送指令
            l_success, r_success, l_joints, r_joints = self.controller.move_to_pose_direct(
                left_pose=self.left_arm_pose,
                right_pose=self.right_arm_pose,
                unit='matrix'  # 使用 4x4 矩阵
            )

            # 每秒打印一次综合诊断信息 (双臂位姿 + 关节 + zsp_para)
            self._debug_counter += 1
            if self._debug_counter >= self._debug_interval:
                self._debug_counter = 0
                self._log_control_status(l_success, r_success, l_joints, r_joints)

    @staticmethod
    def _pose_to_matrix(pose) -> np.ndarray:
        """将 geometry_msgs/Pose 转换为 4x4 变换矩阵"""
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return T

    def _write_detail_log(self, msg: str) -> None:
        """写入详细日志文件 (line-buffered, 自动 flush 到 OS 缓冲区)"""
        if self._detail_log:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self._detail_log.write(f"{ts} {msg}\n")

    def _log_control_status(self, l_success, r_success, l_joints, r_joints) -> None:
        """每秒输出一次综合诊断日志 (双臂位姿 + 关节角 + zsp_para)

        输出格式:
          IK: A=True B=True | zsp A=[0.00,-0.89,-0.45] B=[0.00,0.89,-0.45]
            A: pos=[0.582,0.226,0.270] euler=[99.1,84.0,97.5]° j=[55.0,-65.0,-70.0,-60.0,60.0,0.0,0.0]
            B: pos=[0.573,-0.224,0.276] euler=[...] j=[-55.0,-65.0,70.0,-60.0,-60.0,0.0,0.0]
        """
        ld = self.left_arm_direction
        rd = self.right_arm_direction

        # Line 1: IK 状态 + zsp_para
        line1 = (
            f"IK: A={l_success} B={r_success} | "
            f"zsp A=[{ld[0]:.2f},{ld[1]:.2f},{ld[2]:.2f}] "
            f"B=[{rd[0]:.2f},{rd[1]:.2f},{rd[2]:.2f}]"
        )
        self.get_logger().info(line1)

        # Line 2: A 臂 (左) 位姿 + 关节角
        a_line = "  A:"
        if self.left_arm_pose is not None:
            lp = self.left_arm_pose[:3, 3]
            le = R.from_matrix(self.left_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            a_line += f" pos=[{lp[0]:.3f},{lp[1]:.3f},{lp[2]:.3f}] euler=[{le[0]:.1f},{le[1]:.1f},{le[2]:.1f}]°"
        else:
            a_line += " no_pose"
        if l_joints:
            a_line += f" j=[{','.join(f'{j:.1f}' for j in l_joints)}]"
        elif not l_success and self.left_arm_pose is not None:
            a_line += " j=FAIL"
        self.get_logger().info(a_line)

        # Line 3: B 臂 (右) 位姿 + 关节角
        b_line = "  B:"
        if self.right_arm_pose is not None:
            rp = self.right_arm_pose[:3, 3]
            re_ = R.from_matrix(self.right_arm_pose[:3, :3]).as_euler('ZYX', degrees=True)
            b_line += f" pos=[{rp[0]:.3f},{rp[1]:.3f},{rp[2]:.3f}] euler=[{re_[0]:.1f},{re_[1]:.1f},{re_[2]:.1f}]°"
        else:
            b_line += " no_pose"
        if r_joints:
            b_line += f" j=[{','.join(f'{j:.1f}' for j in r_joints)}]"
        elif not r_success and self.right_arm_pose is not None:
            b_line += " j=FAIL"
        self.get_logger().info(b_line)

        # 写入详细日志文件
        self._write_detail_log(line1)
        self._write_detail_log(a_line)
        self._write_detail_log(b_line)


def main(argv: list[str] | None = None) -> None:
    """主入口"""
    program_name = sys.argv[0] if sys.argv else "tianji_world_output_node"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]

    parser = argparse.ArgumentParser(
        description="Tianji arm control node (Topic-based, no TF) / 天机臂控制节点（基于 topic，无 TF）"
    )
    parser.add_argument(
        "--robot-ip", default=None,
        help="Robot IP (default: from tianji_robot.yaml)",
    )
    args = parser.parse_args(cli_argv)

    # robot_ip: CLI > tianji_robot.yaml > 默认值
    config = TianjiConfig.load(use_ros=False)
    robot_ip = args.robot_ip or config.robot_ip

    rclpy.init(args=raw_argv)
    node = TianjiWorldOutputNode(robot_ip=robot_ip)

    # 关闭标志，防止重复调用 disable_and_release
    _shutdown_done = False

    def _cleanup():
        """清理资源 (关闭日志 + 机器人下电)"""
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True

        # 关闭详细日志文件
        if hasattr(node, '_detail_log') and node._detail_log:
            node._detail_log.write(f"# === 日志结束 ({datetime.now().strftime('%H:%M:%S')}) ===\n")
            node._detail_log.flush()
            node._detail_log.close()
            try:
                node.get_logger().info(f'详细日志已保存: {node._detail_log_path}')
            except Exception:
                print('详细日志已保存: %s' % node._detail_log_path)

        # 强制刷新所有输出
        sys.stdout.flush()
        sys.stderr.flush()

        # 机器人下电 (最关键步骤)
        try:
            node.controller.disable_and_release()
        except Exception as e:
            print(f'[WARNING] 机器人下电异常: {e}', file=sys.stderr)

    def _signal_handler(signum, frame):
        """SIGTERM/SIGINT 信号处理: 确保机器人安全下电"""
        sig_name = signal.Signals(signum).name
        print(f'\n[{sig_name}] 收到退出信号, 正在安全关闭...', file=sys.stderr)
        _cleanup()
        sys.exit(0)

    # 注册信号处理器 (SIGTERM: launch 关闭时发送)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
