#!/usr/bin/env python3
"""
=============================================================================
Step 2: ROS2 Topic 位姿控制（你的简化架构）
=============================================================================

测试目的：
  验证 ROS2 topic 通信和机械臂控制

输入输出：
  输入：目标位姿（PoseStamped topic）
  处理：tianji_world_output_node 订阅 → IK 计算 → 关节角
  输出：发送关节角指令到真机

架构：
  step2_pose_topic_control.py (发布 topic)
      ↓
  /left_arm_target_pose, /right_arm_target_pose (PoseStamped)
      ↓
  tianji_world_output_node (订阅 topic → IK → 真机)

使用方式：
  # 终端 1：启动机械臂控制节点
  cd ~/Desktop/wuji-teleop-ros2-private
  source install/setup.bash
  ros2 launch wuji_teleop_bringup test_arm_control.launch.py

  # 终端 2：发布测试位姿
  cd src/input_devices/pico_input/test

  # 静止（保持初始位姿）
  python3 step2_pose_topic_control.py --mode static --duration 5.0

  # 前进（world +X）- 推荐参数
  python3 step2_pose_topic_control.py --mode forward --speed 0.03 --duration 3.0

  # 后退（world -X）
  python3 step2_pose_topic_control.py --mode back --speed 0.03 --duration 3.0

  # 左移（world +Y）
  python3 step2_pose_topic_control.py --mode left --speed 0.02 --duration 3.0

  # 右移（world -Y）
  python3 step2_pose_topic_control.py --mode right --speed 0.02 --duration 3.0

  # 上升（world +Z）
  python3 step2_pose_topic_control.py --mode up --speed 0.02 --duration 3.0

  # 下降（world -Z）
  python3 step2_pose_topic_control.py --mode down --speed 0.02 --duration 3.0

  # 绕 world X 轴旋转（Roll）
  python3 step2_pose_topic_control.py --mode rotate_x --speed 0.5 --duration 3.0

  # 绕 world Y 轴旋转（Pitch）
  python3 step2_pose_topic_control.py --mode rotate_y --speed 0.5 --duration 3.0

  # 绕 world Z 轴旋转（Yaw）
  python3 step2_pose_topic_control.py --mode rotate_z --speed 0.5 --duration 3.0

  # 运动完成后自动退出 ✅

需要：
  ✅ ROS2
  ❌ TF（不需要坐标转换）
  ✅ Launch 文件

验证结果：
  ✅ ROS2 节点启动成功
  ✅ Topic 通信正常
  ✅ 机械臂跟随 topic 运动
  ✅ 测试脚本自动退出

=============================================================================
"""

import argparse
import time
import sys
import signal
import atexit
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped

# 全局变量用于信号处理清理
_node = None
_shutdown_requested = False


def _signal_handler(signum, frame):
    """信号处理器，确保 Ctrl+C 和 kill 时能正确清理"""
    global _shutdown_requested
    if not _shutdown_requested:
        _shutdown_requested = True
        print("\n收到退出信号，正在清理...")
        _cleanup()
        sys.exit(0)


def _cleanup():
    """清理函数，确保节点正确销毁"""
    global _node
    if _node is not None:
        try:
            _node.destroy_node()
            _node = None
        except Exception:
            pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# 注册退出时清理
atexit.register(_cleanup)

# 注册信号处理
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

# 导入共享模块（使用相对路径）
_test_dir = str(Path(__file__).resolve().parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
from common.robot_config import (
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    WORLD_TO_LEFT_QUAT,
    WORLD_TO_RIGHT_QUAT,
    DEFAULT_ZSP_DIRECTION,
)
from common.transform_utils import (
    transform_world_to_chest,
    get_direction_vector_world,
    get_rotation_axis_world,
    apply_world_rotation_to_chest_pose,
)


class PosePublisher(Node):
    """位姿发布节点"""

    def __init__(self, mode: str, speed: float, duration: float):
        super().__init__('pose_publisher')

        self.mode = mode
        self.speed = speed
        self.duration = duration

        # 创建发布者
        self.left_pose_pub = self.create_publisher(PoseStamped, '/left_arm_target_pose', 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/right_arm_target_pose', 10)
        self.left_elbow_pub = self.create_publisher(Vector3Stamped, '/left_arm_elbow_direction', 10)
        self.right_elbow_pub = self.create_publisher(Vector3Stamped, '/right_arm_elbow_direction', 10)

        # 旋转状态（用于累积旋转）
        self.current_rot = {
            'left': TIANJI_INIT_ROT['left'].copy(),
            'right': TIANJI_INIT_ROT['right'].copy()
        }

        # 等待订阅者就绪
        self.get_logger().info("等待 tianji_world_output_node 订阅者...")
        wait_count = 0
        while self.left_pose_pub.get_subscription_count() == 0:
            time.sleep(0.5)
            wait_count += 1
            if wait_count % 2 == 0:
                self.get_logger().info(f"等待中... ({wait_count * 0.5:.1f}s)")

        self.get_logger().info(f"✓ 订阅者就绪! 开始发布位姿 - 模式: {mode}, 速度: {speed}, 持续: {duration}s")

        # 记录开始时间 (在订阅者就绪后)
        self.start_time = time.time()

        # 创建定时器 (在订阅者就绪后)
        self.timer = self.create_timer(0.01, self.publish_poses)  # 100Hz

    def publish_poses(self):
        """发布位姿"""
        elapsed = time.time() - self.start_time

        # 检查是否结束
        if elapsed > self.duration:
            self.get_logger().info("运动完成!")
            self.timer.cancel()
            raise SystemExit(0)

        # 计算当前位置
        if self.mode == 'static':
            left_pos = TIANJI_INIT_POS['left'].copy()
            right_pos = TIANJI_INIT_POS['right'].copy()

        elif self.mode in ['forward', 'back', 'left', 'right', 'up', 'down']:
            direction_world = get_direction_vector_world(self.mode)
            displacement_world = direction_world * self.speed * elapsed

            left_disp = transform_world_to_chest(displacement_world, 'left')
            right_disp = transform_world_to_chest(displacement_world, 'right')

            left_pos = TIANJI_INIT_POS['left'] + left_disp
            right_pos = TIANJI_INIT_POS['right'] + right_disp

        elif self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
            # 旋转模式：使用共享库 apply_world_rotation_to_chest_pose
            left_pos = TIANJI_INIT_POS['left'].copy()
            right_pos = TIANJI_INIT_POS['right'].copy()

            # 计算旋转增量
            dt = 0.01  # 定时器周期
            rotation_angle = self.speed * dt  # speed 作为角速度 (rad/s)

            # 获取旋转轴（在 world 坐标系中）
            axis_world = get_rotation_axis_world(self.mode)

            # 使用共享库执行 4 步算法: target_chest = R_w2c @ R_delta @ R_c2w @ base_chest
            R_delta = R.from_rotvec(axis_world * rotation_angle)

            self.current_rot = {
                side: apply_world_rotation_to_chest_pose(
                    self.current_rot[side], R_delta, side
                )
                for side in ['left', 'right']
            }

        else:
            self.get_logger().error(f"未知模式: {self.mode}")
            return

        # 发布位姿
        if self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
            # 旋转模式：使用累积的旋转
            self._publish_pose(self.left_pose_pub, 'world_left', left_pos, self.current_rot['left'])
            self._publish_pose(self.right_pose_pub, 'world_right', right_pos, self.current_rot['right'])
        else:
            # 位置模式：使用初始旋转
            self._publish_pose(self.left_pose_pub, 'world_left', left_pos, TIANJI_INIT_ROT['left'])
            self._publish_pose(self.right_pose_pub, 'world_right', right_pos, TIANJI_INIT_ROT['right'])

        # 发布臂角参考平面参数 (从统一配置加载)
        self._publish_elbow_direction(self.left_elbow_pub, 'world_left', DEFAULT_ZSP_DIRECTION['left'])
        self._publish_elbow_direction(self.right_elbow_pub, 'world_right', DEFAULT_ZSP_DIRECTION['right'])

        # 进度显示（每秒一次）
        if int(elapsed * 10) % 10 == 0:
            progress = int(elapsed / self.duration * 100)
            if self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
                # 旋转模式：打印当前姿态的 euler 角
                euler_left = R.from_matrix(self.current_rot['left']).as_euler('ZYX', degrees=True)
                self.get_logger().info(
                    f"进度: {progress}% | 发布euler=[{euler_left[0]:.1f}, {euler_left[1]:.1f}, {euler_left[2]:.1f}]°"
                )
            else:
                self.get_logger().info(f"进度: {progress}% - 位置: [{left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}]")

    def _publish_pose(self, publisher, frame_id: str, pos: np.ndarray, rot: np.ndarray):
        """发布位姿消息"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        quat = R.from_matrix(rot).as_quat()
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        publisher.publish(msg)

    def _publish_elbow_direction(self, publisher, frame_id: str, direction: np.ndarray):
        """发布肘部方向"""
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # 归一化
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm

        msg.vector.x = float(direction[0])
        msg.vector.y = float(direction[1])
        msg.vector.z = float(direction[2])

        publisher.publish(msg)


def main():
    global _node, _shutdown_requested

    parser = argparse.ArgumentParser(description='测试位姿发布器')
    parser.add_argument('--mode', type=str, default='static',
                        choices=['static', 'forward', 'back', 'left', 'right', 'up', 'down',
                                 'rotate_x', 'rotate_y', 'rotate_z'],
                        help='测试模式')
    parser.add_argument('--speed', type=float, default=0.02,
                        help='移动速度 (m/s) 或旋转速度 (rad/s)')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='持续时间 (秒)')

    args = parser.parse_args()

    rclpy.init()
    _node = PosePublisher(
        mode=args.mode,
        speed=args.speed,
        duration=args.duration
    )

    try:
        rclpy.spin(_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not _shutdown_requested:
            _cleanup()


if __name__ == '__main__':
    main()
