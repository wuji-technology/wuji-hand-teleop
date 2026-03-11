#!/usr/bin/env python3
"""
=============================================================================
Step 3: RViz 坐标系可视化
=============================================================================

功能：读取机器人关节角 → 发布 TF → RViz 显示坐标系（不控制机器人）

⚠️ 限制：不能与 step1/step2 同时运行（端口冲突）

使用方法：
  # 终端 1: 启动本脚本
  python3 step3_visualize_in_rviz.py

  # 终端 2: 手动启动 RViz
  rviz2

  # 在 RViz 中配置:
  #   Fixed Frame: world
  #   Add → TF → 勾选 Show Axes & Show Names

测试方式：
  1. 独立测试: 启动 step3 → 手动移动机械臂 → 观察 RViz 更新
  2. 交替测试: 运行 step1 测试 → 停止 → 启动 step3 → 观察位姿

坐标系架构：
  world (基座)
    ├─ world_left (左臂基座, Y=+0.2m)
    │    └─ left_dh_ee (左臂末端, 实时更新)
    └─ world_right (右臂基座, Y=-0.2m)
         └─ right_dh_ee (右臂末端, 实时更新)

参数说明：
  双臂基座姿态: 绕 X 轴旋转 ±90°
    LEFT_QUAT  = [0.7071, 0, 0,  0.7071]
    RIGHT_QUAT = [0.7071, 0, 0, -0.7071]

  双臂基座位置: Y 轴 ±0.2m，间距 40cm
    LEFT_POS  = [0.0, +0.2, 0.0]
    RIGHT_POS = [0.0, -0.2, 0.0]

=============================================================================
"""
import sys
import signal
import atexit
from pathlib import Path
import rclpy
from rclpy.node import Node

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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
from tianji_world_output.cartesian_controller import CartesianController

# 导入共享模块（使用相对路径）
_test_dir = str(Path(__file__).resolve().parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
from common.robot_config import WORLD_TO_CHEST_TRANS, ROBOT_IP
from common.transform_utils import get_tf_quaternion


# ==================== World → Arm Base 坐标变换（使用统一配置）====================
# 数据来源: tianji_robot.yaml 中的 world_to_chest_trans
# 遵循 ROS REP 103 标准：+X前 +Y左 +Z上
#
# 注意：TF 需要 chest→world 方向的四元数，使用 get_tf_quaternion() 获取


class RobotAxisVisualizer(Node):
    def __init__(self):
        super().__init__('robot_axis_visualizer')

        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 连接机器人
        self.get_logger().info('正在连接机器人...')
        try:
            self.controller = CartesianController(robot_ip=ROBOT_IP)
            self.get_logger().info('✓ 已连接到机器人')
        except Exception as e:
            self.get_logger().error(f'连接失败: {e}')
            raise

        # 发布静态机械臂基座坐标系
        self.publish_static_frames()

        # 定时读取并发布机械臂位姿
        self.timer = self.create_timer(0.1, self.update_robot_pose)  # 10Hz

        self.get_logger().info('开始发布 TF，请在 RViz 中查看')
        self.get_logger().info('  Fixed Frame: world')
        self.get_logger().info('  坐标系: world → world_left/world_right → left_dh_ee/right_dh_ee')
        self.get_logger().info('  Add → TF → Show Axes & Names')

    def publish_static_frames(self):
        """发布静态坐标系：world → world_left/world_right"""
        now = self.get_clock().now()
        transforms = []

        # 获取 TF 用四元数 (chest→world 方向)
        left_tf_quat = get_tf_quaternion('left')
        right_tf_quat = get_tf_quaternion('right')

        # world → world_left
        t_left = TransformStamped()
        t_left.header.stamp = now.to_msg()
        t_left.header.frame_id = 'world'
        t_left.child_frame_id = 'world_left'
        t_left.transform.translation.x = float(WORLD_TO_CHEST_TRANS['left'][0])
        t_left.transform.translation.y = float(WORLD_TO_CHEST_TRANS['left'][1])
        t_left.transform.translation.z = float(WORLD_TO_CHEST_TRANS['left'][2])
        t_left.transform.rotation.x = float(left_tf_quat[0])
        t_left.transform.rotation.y = float(left_tf_quat[1])
        t_left.transform.rotation.z = float(left_tf_quat[2])
        t_left.transform.rotation.w = float(left_tf_quat[3])
        transforms.append(t_left)

        # world → world_right
        t_right = TransformStamped()
        t_right.header.stamp = now.to_msg()
        t_right.header.frame_id = 'world'
        t_right.child_frame_id = 'world_right'
        t_right.transform.translation.x = float(WORLD_TO_CHEST_TRANS['right'][0])
        t_right.transform.translation.y = float(WORLD_TO_CHEST_TRANS['right'][1])
        t_right.transform.translation.z = float(WORLD_TO_CHEST_TRANS['right'][2])
        t_right.transform.rotation.x = float(right_tf_quat[0])
        t_right.transform.rotation.y = float(right_tf_quat[1])
        t_right.transform.rotation.z = float(right_tf_quat[2])
        t_right.transform.rotation.w = float(right_tf_quat[3])
        transforms.append(t_right)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('✓ 已发布静态坐标系: world → world_left, world_right')

    def update_robot_pose(self):
        """
        从机器人读取当前位姿并发布 TF

        变换链（符合 ROS REP 103 标准）：
        ===================================
        world (机器人基座, +X前 +Y左 +Z上)
          ↓
        world_left / world_right (机械臂基座，静态)
          ↓
        left_dh_ee / right_dh_ee (DH 末端，动态从 FK 计算)

        说明：
        - FK 输出是 DH 末端坐标系在 arm base 坐标系中的位姿
        - 直接发布为 world_left → left_dh_ee 的 TF
        - 命名清晰：与 step4 保持一致
        """
        try:
            # ==================== 步骤 1: 读取关节角 ====================
            joints = self.controller.get_current_joints()
            left_joints = joints[0]
            right_joints = joints[1]

            # ==================== 步骤 2: 计算 FK ====================
            # FK 输出是 DH 末端坐标系在 arm base 坐标系中的位姿
            left_fk_mat = self.controller.kine_left.fk(0, left_joints)
            right_fk_mat = self.controller.kine_right.fk(1, right_joints)

            # ==================== 步骤 3: 提取位置和姿态 ====================
            # 位置（毫米 → 米）
            left_pos = np.array([left_fk_mat[0][3], left_fk_mat[1][3], left_fk_mat[2][3]]) / 1000.0
            right_pos = np.array([right_fk_mat[0][3], right_fk_mat[1][3], right_fk_mat[2][3]]) / 1000.0

            # 姿态（旋转矩阵 → 四元数）
            left_rot = np.array([
                [left_fk_mat[0][0], left_fk_mat[0][1], left_fk_mat[0][2]],
                [left_fk_mat[1][0], left_fk_mat[1][1], left_fk_mat[1][2]],
                [left_fk_mat[2][0], left_fk_mat[2][1], left_fk_mat[2][2]],
            ])
            left_quat = R.from_matrix(left_rot).as_quat()

            right_rot = np.array([
                [right_fk_mat[0][0], right_fk_mat[0][1], right_fk_mat[0][2]],
                [right_fk_mat[1][0], right_fk_mat[1][1], right_fk_mat[1][2]],
                [right_fk_mat[2][0], right_fk_mat[2][1], right_fk_mat[2][2]],
            ])
            right_quat = R.from_matrix(right_rot).as_quat()

            # ==================== 步骤 4: 发布 TF ====================
            now = self.get_clock().now()
            transforms = []

            # world_left → left_dh_ee
            t_left = TransformStamped()
            t_left.header.stamp = now.to_msg()
            t_left.header.frame_id = 'world_left'
            t_left.child_frame_id = 'left_dh_ee'
            t_left.transform.translation.x = float(left_pos[0])
            t_left.transform.translation.y = float(left_pos[1])
            t_left.transform.translation.z = float(left_pos[2])
            t_left.transform.rotation.x = float(left_quat[0])
            t_left.transform.rotation.y = float(left_quat[1])
            t_left.transform.rotation.z = float(left_quat[2])
            t_left.transform.rotation.w = float(left_quat[3])
            transforms.append(t_left)

            # world_right → right_dh_ee
            t_right = TransformStamped()
            t_right.header.stamp = now.to_msg()
            t_right.header.frame_id = 'world_right'
            t_right.child_frame_id = 'right_dh_ee'
            t_right.transform.translation.x = float(right_pos[0])
            t_right.transform.translation.y = float(right_pos[1])
            t_right.transform.translation.z = float(right_pos[2])
            t_right.transform.rotation.x = float(right_quat[0])
            t_right.transform.rotation.y = float(right_quat[1])
            t_right.transform.rotation.z = float(right_quat[2])
            t_right.transform.rotation.w = float(right_quat[3])
            transforms.append(t_right)

            self.tf_broadcaster.sendTransform(transforms)

        except Exception as e:
            self.get_logger().error(f'更新位姿失败: {e}', throttle_duration_sec=5.0)

def main(args=None):
    global _node, _shutdown_requested

    rclpy.init(args=args)
    _node = RobotAxisVisualizer()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        if not _shutdown_requested:
            _cleanup()

if __name__ == '__main__':
    main()
