#!/usr/bin/env python3
"""
=============================================================================
Step 4: 可视化录制的 PICO 数据（离线测试，不需要真实硬件）
=============================================================================

功能：读取 PICO 录制数据 → 坐标变换 → 发布 TF/Topics → RViz 可视化

✅ 优点：
  - 不需要 PICO 硬件
  - 不需要机器人硬件
  - 可重复测试坐标变换逻辑
  - 验证增量控制算法
  - 完整的 ROS2 节点，包含静态 TF 发布

⚠️ 与其他 step 的区别：
  - step3: 从真机器人读取关节角 → FK → 可视化
  - step4: 从录制文件读取 PICO 数据 → 坐标变换 → 可视化 (本脚本)
  - step5: 从录制文件读取 PICO 数据 → 坐标变换 → 控制真机器人


使用方法
========

  终端 1: 启动本脚本
  ------------------
  cd /home/wuji-08/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/test
  python3 step4_visualize_recorded_data.py

  # 使用其他录制文件 (record/ 目录下的大数据集)
  python3 step4_visualize_recorded_data.py --file ../record/trackingData_head_tracker_static.txt

  # 调整回放速度 (0.5=慢速, 1.0=实时, 2.0=快速)
  python3 step4_visualize_recorded_data.py --speed 0.5

  # 循环回放
  python3 step4_visualize_recorded_data.py --loop


  终端 2: 启动 RViz 可视化
  ------------------------
  rviz2

  手动配置 RViz:
    1. Fixed Frame: world
    2. Add → TF → 勾选 Show Axes & Show Names
    3. Add → Axes (可选，显示 world 坐标系)


测试数据文件（均在 record/ 目录下）
============
  高质量静态数据 (推荐):
    ../record/trackingData_sample_static.txt (150 帧，平均位移 <0.1mm)

  大数据集:
    ../record/trackingData_head_tracker_static.txt (8384 帧)
    ../record/trackingData_head_tracker.txt (5703 帧)
    ../record/trackingData_whole_data.txt (5780 帧)


坐标变换说明
============
PICO 坐标系 → Robot 坐标系:
  PICO: +X右, +Y上, +Z前(朝向用户)
  Robot: +X前, +Y左, +Z上

  变换矩阵 pico_to_robot (从 tianji_robot.yaml 加载，详见 transform_utils.py):
    Robot X = -PICO Z  (用户前移 → 机器人后移，靠近用户)
    Robot Y = -PICO X  (用户右移 → 机器人右移)
    Robot Z = +PICO Y  (用户上移 → 机器人上移)


Tracker SN 映射
===============
  190058 → pico_left_wrist  (左手腕，控制左臂末端)
  190600 → pico_right_wrist (右手腕，控制右臂末端)
  190046 → pico_left_arm    (左前臂，臂角约束)
  190023 → pico_right_arm   (右前臂，臂角约束)


TF 树结构
=========
  world
  ├── head (HMD 头显)
  ├── world_left (左臂 chest 坐标系, Y=+0.2)
  │   ├── left_dh_ee (机器人左臂末端参考位置，静态)
  │   ├── pico_left_wrist (PICO 左手腕，动态)
  │   └── pico_left_arm (PICO 左前臂，动态)
  │
  └── world_right (右臂 chest 坐标系, Y=-0.2)
      ├── right_dh_ee (机器人右臂末端参考位置，静态)
      ├── pico_right_wrist (PICO 右手腕，动态)
      └── pico_right_arm (PICO 右前臂，动态)


发布的 Topics
=============
  /pico_hmd              - HMD 位姿 (PoseStamped)
  /pico_left_wrist       - 左手腕位姿 (PoseStamped)
  /pico_right_wrist      - 右手腕位姿 (PoseStamped)
  /pico_left_arm         - 左前臂位姿 (PoseStamped)
  /pico_right_arm        - 右前臂位姿 (PoseStamped)
  /left_arm_target_pose  - 左臂目标位姿 (PoseStamped)
  /right_arm_target_pose - 右臂目标位姿 (PoseStamped)
  /left_arm_elbow_direction  - 左臂肘部方向 (Vector3Stamped, 臂角约束)
  /right_arm_elbow_direction - 右臂肘部方向 (Vector3Stamped, 臂角约束)


调试命令
========
  # 查看所有 PICO topics
  ros2 topic list | grep pico

  # 查看 tracker 数据
  ros2 topic echo /pico_left_wrist --once
  ros2 topic echo /left_arm_elbow_direction --once

  # 查看 TF 树
  ros2 run tf2_tools view_frames && evince frames.pdf


验证标准
========
  ✓ 4 个 tracker 正确初始化 (pico_left_wrist, pico_right_wrist, pico_left_arm, pico_right_arm)
  ✓ Topics 正常发布，数据有效
  ✓ TF frames 在 RViz 中正确显示
  ✓ 坐标轴方向正确 (红X前, 绿Y左, 蓝Z上)
  ✓ 臂角方向 /left_arm_elbow_direction 正常发布


下一步
======
测试通过后，进入 step5:
  python3 step5_incremental_control_with_robot.py --dry-run

=============================================================================
"""

import sys
import time
import argparse
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
            print("✓ ROS2 节点已销毁")
        except Exception:
            pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
            print("✓ ROS2 已关闭")
    except Exception:
        pass


# 注册退出时清理
atexit.register(_cleanup)

# 注册信号处理
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

# 导入共享模块（使用相对路径）
_test_dir = str(Path(__file__).resolve().parent)
_pico_input_dir = str(Path(__file__).resolve().parent.parent)
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)
if _pico_input_dir not in sys.path:
    sys.path.insert(0, _pico_input_dir)

from common.robot_config import (
    TIANJI_INIT_POS, TIANJI_INIT_ROT, WORLD_TO_CHEST_TRANS,
    DEFAULT_ZSP_DIRECTION, ARM_INIT_POS,
)
from common.transform_utils import (
    get_tf_quaternion,
    get_pico_to_robot,
    transform_pico_rotation_to_world,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
)

# 导入数据源
from pico_input.data_source import RecordedDataSource


# PICO → 机器人坐标变换 (从共享配置加载)
# 详见 tianji_world_output/transform_utils.py 和 PICO_TELEOP_GUIDE.md §3.1

# World → Chest 转换: 使用 transform_utils 中的共享函数
# (transform_world_to_chest, apply_world_rotation_to_chest_pose 等)


class RecordedDataVisualizer(Node):
    """
    录制数据可视化节点（双骨骼层级可视化）

    功能：
    - 发布静态 TF: world → world_left, world_right (chest 坐标系)
    - 发布静态 TF: world_left/right → 机器人 DH 关节 (left_dh_ee, left_dh_elbow) - 固定参数
    - 读取录制数据 (RecordedDataSource)
    - 计算 PICO tracker 增量位姿
    - 发布动态 TF: world_left/right → PICO trackers (pico_left_wrist, left_arm)
    - 发布 Topics: /pico_*, /left_arm_target_pose, /right_arm_target_pose

    关键设计（两个骨骼一起运动）：
    - 机器人 DH 骨骼：world_left → left_dh_ee, left_dh_elbow（静态，TIANJI_INIT_POS/ELBOW_POS）
    - PICO 骨骼：world_left → pico_left_wrist, left_arm（动态，增量控制）
    - 共享父坐标系（chest），轴向自然对齐，便于对比验证
    """

    def __init__(self, data_file: str, playback_speed: float = 1.0, loop_playback: bool = False):
        super().__init__('recorded_data_visualizer')

        # 参数
        self.playback_speed = playback_speed
        self.loop_playback = loop_playback

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Topic 发布器
        self.left_arm_pose_pub = self.create_publisher(PoseStamped, '/left_arm_target_pose', 10)
        self.right_arm_pose_pub = self.create_publisher(PoseStamped, '/right_arm_target_pose', 10)

        # PICO tracker topics (所有 topic 统一使用 pico_ 前缀)
        self.pico_hmd_pub = self.create_publisher(PoseStamped, '/pico_hmd', 10)
        self.pico_left_wrist_pub = self.create_publisher(PoseStamped, '/pico_left_wrist', 10)
        self.pico_right_wrist_pub = self.create_publisher(PoseStamped, '/pico_right_wrist', 10)
        self.pico_left_arm_pub = self.create_publisher(PoseStamped, '/pico_left_arm', 10)
        self.pico_right_arm_pub = self.create_publisher(PoseStamped, '/pico_right_arm', 10)

        # 臂角控制 topics (用于 tianji_world_output 的零空间 IK 约束)
        # 发布肘部偏移方向向量: 肘部相对于肩-腕平面的法向量
        self.left_elbow_dir_pub = self.create_publisher(Vector3Stamped, '/left_arm_elbow_direction', 10)
        self.right_elbow_dir_pub = self.create_publisher(Vector3Stamped, '/right_arm_elbow_direction', 10)

        # RViz Marker 可视化 (用于理解臂角计算原理)
        self.marker_pub = self.create_publisher(MarkerArray, '/elbow_angle_visualization', 10)

        # 缓存手腕位置用于计算肩-腕连线
        self.wrist_positions = {'left': None, 'right': None}

        # 初始化状态
        self.initialized = False
        self.init_tracker_poses = {}
        self.init_hmd_pose = None

        # 缓存：用于静止数据优化（减少 RViz 抖动）
        self.cached_poses = {}  # {role: (pos, quat)}
        self.debug_counter = 0  # 用于限制日志输出

        # 数据源
        self.get_logger().info(f'加载录制数据: {data_file}')
        self.data_source = RecordedDataSource(data_file, playback_speed, loop_playback)

        if not self.data_source.initialize():
            self.get_logger().error('数据源初始化失败')
            raise RuntimeError('Failed to initialize data source')

        total_frames = len(self.data_source.data_frames) if hasattr(self.data_source, 'data_frames') else 0
        self.get_logger().info(f'✓ 数据加载成功，共 {total_frames} 帧')

        # 发布静态 TF
        self.publish_static_frames()

        # 自动初始化（使用第一帧数据）
        self.create_timer(0.1, self.auto_init_once)

        # 定时器：发布 TF 和 Topics
        self.timer = self.create_timer(1.0 / 90.0, self.publish_callback)  # 90 Hz

        self.get_logger().info('=' * 70)
        self.get_logger().info('Step 4: 录制数据可视化节点已启动')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  数据文件: {data_file}')
        self.get_logger().info(f'  回放速度: {playback_speed}x')
        self.get_logger().info(f'  循环回放: {loop_playback}')
        self.get_logger().info(f'  总帧数: {total_frames}')
        self.get_logger().info('-' * 70)
        self.get_logger().info('在另一个终端启动 RViz:')
        self.get_logger().info('  rviz2 -d $(ros2 pkg prefix pico_input)/share/pico_input/rviz/pico_simulation.rviz')
        self.get_logger().info('-' * 70)
        self.get_logger().info('验证要点:')
        self.get_logger().info('  ✓ 机器人 DH 和 PICO tracker 都连接到 world_left/right (chest)')
        self.get_logger().info('  ✓ 轴向自然对齐（共享父坐标系）')
        self.get_logger().info('  ✓ 像两个骨骼一起运动：机器人固定 + PICO 跟随')
        self.get_logger().info('=' * 70)

    def publish_static_frames(self):
        """发布静态 TF: world → world_left, world_right, 以及机器人 DH 关节位置"""
        now = self.get_clock().now()
        transforms = []

        # 获取 TF 用四元数 (chest→world 方向)
        left_tf_quat = get_tf_quaternion('left')
        right_tf_quat = get_tf_quaternion('right')

        # world → world_left (Y=+0.2)
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

        # world → world_right (Y=-0.2)
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

        # ========== 机器人 DH 末端位置（仅用于 RViz 可视化参考）==========
        # 注意: dh_elbow 已移除，因为零空间 IK 约束使用 PICO arm tracker 的方向向量
        # 而不是固定的肘部位置。肘部约束由 /left_arm_elbow_direction topic 提供。

        # 左臂旋转矩阵
        left_ee_quat = R.from_matrix(TIANJI_INIT_ROT['left']).as_quat()

        # 右臂旋转矩阵
        right_ee_quat = R.from_matrix(TIANJI_INIT_ROT['right']).as_quat()

        # world_left → left_dh_ee (左臂末端参考位置)
        t_left_ee = TransformStamped()
        t_left_ee.header.stamp = now.to_msg()
        t_left_ee.header.frame_id = 'world_left'
        t_left_ee.child_frame_id = 'left_dh_ee'
        t_left_ee.transform.translation.x = float(TIANJI_INIT_POS['left'][0])
        t_left_ee.transform.translation.y = float(TIANJI_INIT_POS['left'][1])
        t_left_ee.transform.translation.z = float(TIANJI_INIT_POS['left'][2])
        t_left_ee.transform.rotation.x = float(left_ee_quat[0])
        t_left_ee.transform.rotation.y = float(left_ee_quat[1])
        t_left_ee.transform.rotation.z = float(left_ee_quat[2])
        t_left_ee.transform.rotation.w = float(left_ee_quat[3])
        transforms.append(t_left_ee)

        # world_right → right_dh_ee (右臂末端参考位置)
        t_right_ee = TransformStamped()
        t_right_ee.header.stamp = now.to_msg()
        t_right_ee.header.frame_id = 'world_right'
        t_right_ee.child_frame_id = 'right_dh_ee'
        t_right_ee.transform.translation.x = float(TIANJI_INIT_POS['right'][0])
        t_right_ee.transform.translation.y = float(TIANJI_INIT_POS['right'][1])
        t_right_ee.transform.translation.z = float(TIANJI_INIT_POS['right'][2])
        t_right_ee.transform.rotation.x = float(right_ee_quat[0])
        t_right_ee.transform.rotation.y = float(right_ee_quat[1])
        t_right_ee.transform.rotation.z = float(right_ee_quat[2])
        t_right_ee.transform.rotation.w = float(right_ee_quat[3])
        transforms.append(t_right_ee)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('✓ 静态 TF 已发布: world → world_left, world_right')
        self.get_logger().info('✓ 机器人末端参考已发布: left_dh_ee, right_dh_ee')

    def auto_init_once(self):
        """自动初始化（仅执行一次）"""
        if self.initialized:
            return

        # 获取第一帧数据
        headset_data = self.data_source.get_headset_pose()
        tracker_data_list = self.data_source.get_tracker_data()

        if headset_data and headset_data.is_valid and tracker_data_list:
            self.do_init(headset_data, tracker_data_list)

    def do_init(self, headset_data, tracker_data_list):
        """执行初始化：记录 HMD 和 tracker 初始位姿"""
        # 记录 HMD 初始位姿
        hmd_pose = headset_data.to_pose_array()
        self.init_hmd_pose = self._pose_to_matrix(hmd_pose)

        # 记录 tracker 初始位姿
        self.init_tracker_poses = {}
        tracker_map = {
            '190058': 'pico_left_wrist',   # PICO 左手腕 tracker
            '190600': 'pico_right_wrist',  # PICO 右手腕 tracker
            '190046': 'pico_left_arm',     # PICO 左前臂 tracker (统一 pico_ 前缀)
            '190023': 'pico_right_arm',    # PICO 右前臂 tracker (统一 pico_ 前缀)
        }

        self.get_logger().info('初始化中...')
        for tracker in tracker_data_list:
            # 提取序列号中的 6 位数字
            import re
            match = re.search(r'(\d{6})', tracker.serial_number)
            if not match:
                continue

            short_sn = match.group(1)
            role = tracker_map.get(short_sn)

            if role and tracker.is_valid:
                pose = tracker.to_pose_array()
                self.init_tracker_poses[role] = self._pose_to_matrix(pose)
                self.get_logger().info(f'  {role}: pos={tracker.position}')

        self.initialized = True
        self.get_logger().info(f'✓ 初始化完成！记录了 {len(self.init_tracker_poses)} 个 tracker')

    def publish_callback(self):
        """主循环：读取数据并发布 TF + Topics"""
        if not self.data_source.is_available():
            return

        if not self.initialized:
            return

        now = self.get_clock().now()

        # 获取数据
        headset_data = self.data_source.get_headset_pose()
        tracker_data_list = self.data_source.get_tracker_data()

        if not headset_data or not headset_data.is_valid:
            return

        # 发布 HMD TF
        self.publish_hmd_tf(now, headset_data)

        # 发布 Tracker TF
        if tracker_data_list:
            self.publish_tracker_tfs(now, tracker_data_list)

    def publish_hmd_tf(self, now, headset_data):
        """发布 HMD 的 TF"""
        hmd_pose = headset_data.to_pose_array()
        head_T = self._transform_hmd_to_world(hmd_pose)

        if head_T is not None:
            pos = head_T[:3, 3]
            quat = R.from_matrix(head_T[:3, :3]).as_quat()

            # 发布 TF
            self._broadcast_tf(now, 'world', 'head', pos, quat)

            # 发布 Topic
            self._publish_pose(self.pico_hmd_pub, now, 'world', pos, quat)

    def publish_tracker_tfs(self, now, tracker_data_list):
        """
        发布 Tracker 的 TF 和 Topics，包括臂角控制的 elbow direction

        ============================================================================
        臂角 (Arm Angle) 计算原理说明
        ============================================================================

        什么是臂角？
        -----------
        臂角是描述肘部相对于"肩-腕平面"偏移方向的角度。
        想象一下：当你伸直手臂指向前方，肘部可以朝不同方向弯曲：
          - 肘部朝下 = 沉肘（自然放松姿态）
          - 肘部朝外 = 抬肘（像鸡翅膀一样）
          - 肘部朝上 = 反肘（杂技动作）

        为什么需要臂角控制？
        -------------------
        7自由度机械臂有"冗余自由度"，即末端位姿相同时，关节角有无穷多解。
        臂角约束可以指定肘部的期望方向，让机器人选择特定的姿态。

        零空间 (Nullspace) 是什么？
        --------------------------
        对于7自由度机械臂，要到达一个6自由度末端位姿，有1个自由度是"多余"的。
        这个多余的自由度形成一个"零空间"——在这个空间内运动不改变末端位置。

        想象这样一个场景：
        - 你用手握住门把手（末端位姿固定）
        - 你可以通过移动肘部来调整手臂姿态，但手始终握在门把手上
        - 肘部能移动的轨迹就是"零空间"

        臂角控制就是在零空间内选择一个特定的肘部位置。

        计算方法
        --------
        1. 肩膀位置: shoulder = [0, 0, 0] (chest 坐标系原点)
        2. 手腕位置: wrist = pico_left_wrist 的位置
        3. 肘部位置: elbow = pico_left_arm 的位置 (近似)

        4. 肩-腕向量: shoulder_to_wrist = wrist - shoulder
        5. 肩-肘向量: shoulder_to_elbow = elbow - shoulder

        6. 投影: 肘部在肩-腕连线上的投影点
           proj = (shoulder_to_elbow · shoulder_to_wrist_unit) * shoulder_to_wrist_unit

        7. 肘部偏移向量: elbow_offset = shoulder_to_elbow - proj
           这就是肘部偏离肩-腕平面的方向！

        8. 归一化: elbow_direction = elbow_offset / |elbow_offset|

        可视化说明 (在 RViz 中)
        ----------------------
        - 红色箭头: 肩-腕连线
        - 绿色箭头: 肩-肘连线
        - 蓝色箭头: 肘部偏移方向 (elbow_direction) ← 这是发送给 IK 的
        - 黄色点: 肘部在肩-腕连线上的投影点

        ============================================================================
        """
        tracker_map = {
            '190058': ('pico_left_wrist', self.pico_left_wrist_pub, self.left_arm_pose_pub, 'left'),     # PICO 左手腕
            '190600': ('pico_right_wrist', self.pico_right_wrist_pub, self.right_arm_pose_pub, 'right'), # PICO 右手腕
            '190046': ('pico_left_arm', self.pico_left_arm_pub, None, 'left'),                           # PICO 左前臂
            '190023': ('pico_right_arm', self.pico_right_arm_pub, None, 'right'),                        # PICO 右前臂
        }

        # 收集手腕和前臂位置用于计算 elbow direction
        wrist_positions = {}
        arm_positions = {}

        for tracker in tracker_data_list:
            if not tracker.is_valid:
                continue

            # 提取序列号
            import re
            match = re.search(r'(\d{6})', tracker.serial_number)
            if not match:
                continue

            short_sn = match.group(1)
            if short_sn not in tracker_map:
                continue

            pico_frame_name, legacy_pub, target_pub, side = tracker_map[short_sn]

            if pico_frame_name not in self.init_tracker_poses:
                continue

            # 计算增量位姿
            pose = tracker.to_pose_array()
            pos, quat = self._compute_incremental_pose(pose, pico_frame_name, side)

            if pos is None:
                continue

            # 确定父 frame（chest 坐标系）
            parent_frame = 'world_left' if side == 'left' else 'world_right'

            # 发布 TF (PICO tracker 位置，父 frame 为 chest)
            self._broadcast_tf(now, parent_frame, pico_frame_name, pos, quat)

            # 发布 PICO Topic
            self._publish_pose(legacy_pub, now, parent_frame, pos, quat)

            # 发布目标位姿 Topic (仅手腕)
            if target_pub:
                self._publish_pose(target_pub, now, parent_frame, pos, quat)

            # 收集手腕和前臂位置用于 elbow direction 计算
            if 'wrist' in pico_frame_name:
                wrist_positions[side] = pos.copy()
            elif 'arm' in pico_frame_name:
                arm_positions[side] = pos.copy()

        # ============== 计算并发布 elbow direction ==============
        # 从 arm tracker 位置计算几何方向(指向重力/下方)，取反后发布给 IK (反重力方向)
        # arm_init_pos 物理值 [0.2, ±0.3, +0.1] 使 arm 在肩下方，几何方向指向下方
        # 无 tracker 数据时使用 DEFAULT_ZSP_DIRECTION 作为 fallback (已是 IK 约定)

        markers = MarkerArray()
        marker_id = 0

        for side in ['left', 'right']:
            parent_frame = 'world_left' if side == 'left' else 'world_right'
            pub = self.left_elbow_dir_pub if side == 'left' else self.right_elbow_dir_pub

            wrist_pos = wrist_positions.get(side)
            elbow_pos = arm_positions.get(side)

            if wrist_pos is not None and elbow_pos is not None:
                shoulder_pos = np.array([0.0, 0.0, 0.0])  # chest 坐标系原点

                direction, proj_point = self._compute_elbow_direction(
                    shoulder_pos, wrist_pos, elbow_pos, side
                )

                # 几何方向(指向重力) → IK方向(反重力): 取反
                ik_direction = -direction
                self._publish_vector3(pub, now, parent_frame, ik_direction)

                # 创建可视化 Markers (使用物理方向 direction，与绿色肩-肘箭头一致)
                side_markers = self._create_elbow_visualization_markers(
                    now, parent_frame, shoulder_pos, wrist_pos, elbow_pos,
                    proj_point, direction, side, marker_id
                )
                markers.markers.extend(side_markers)
                marker_id += len(side_markers)
            else:
                # 无 tracker 数据时不发布，output_node 保持使用上一次收到的方向
                pass

        # 发布可视化 markers
        if markers.markers:
            self.marker_pub.publish(markers)

    def _compute_elbow_direction(self, shoulder: np.ndarray, wrist: np.ndarray,
                                  elbow: np.ndarray, side: str) -> tuple:
        """
        计算肘部相对于肩-腕平面的偏移方向

        这是臂角控制的核心算法：
        1. 计算肩-腕向量 (手臂主轴)
        2. 计算肩-肘向量
        3. 将肘部投影到肩-腕连线上
        4. 肘部偏移 = 实际肘部位置 - 投影点
        5. 归一化得到方向向量

        ============================================================================
        几何原理图解
        ============================================================================

                    肩膀 (shoulder)
                      ●
                     /|\\
                    / | \\
                   /  |  \\
                  /   |   \\
           肩-肘 /    |    \\ 肩-腕
                /     |     \\
               /      |      \\
              ●-------●-------●
           肘部    投影点    手腕
          (elbow) (proj)   (wrist)

              ←─────→
            肘部偏移向量
          (elbow_direction)

        肘部偏移向量 = 肘部位置 - 投影点
        这个向量是肘部偏离肩-腕连线的物理方向(指向肘部/重力方向)。
        发布给 IK 前需取反: ik_direction = -direction (IK 要求反重力方向)。

        ============================================================================
        关于左右臂的处理
        ============================================================================

        arm tracker 的初始参考位置 (arm_init_pos, 来自 tianji_robot.yaml) 为物理正确值:
          - 左臂: [0.2, +0.3, +0.1]  (Left Chest: Y+=下, Z+=左/外侧)
          - 右臂: [0.2, -0.3, +0.1]  (Right Chest: Y-=下, Z+=右/外侧)

        计算出的 direction 是物理方向(指向肘部)，发布前取反为 IK 方向。
        左右臂的差异已经在位置变换阶段处理好了。

        返回:
            direction: 肘部偏移方向 (单位向量，物理方向，指向肘部)
            proj_point: 肘部在肩-腕连线上的投影点 (用于可视化)
        """
        # 默认臂角参考平面参数 (从统一配置加载)
        default_direction = DEFAULT_ZSP_DIRECTION[side]

        # 肩-腕向量
        shoulder_to_wrist = wrist - shoulder
        sw_length = np.linalg.norm(shoulder_to_wrist)

        if sw_length < 1e-6:
            # 手腕太靠近肩膀，使用默认沉肘方向
            return default_direction, shoulder

        sw_unit = shoulder_to_wrist / sw_length

        # 肩-肘向量
        shoulder_to_elbow = elbow - shoulder

        # 肘部在肩-腕连线上的投影长度
        proj_length = np.dot(shoulder_to_elbow, sw_unit)

        # 投影点
        proj_point = shoulder + proj_length * sw_unit

        # 肘部偏移向量 (肘部偏离肩-腕连线的方向)
        elbow_offset = elbow - proj_point
        offset_length = np.linalg.norm(elbow_offset)

        if offset_length < 1e-6:
            # 肘部正好在肩-腕连线上，使用默认沉肘方向
            # 这种情况在人体姿态中几乎不会发生
            return default_direction, proj_point

        # 归一化得到方向向量
        direction = elbow_offset / offset_length

        return direction, proj_point

    def _create_elbow_visualization_markers(self, now, frame_id: str,
                                             shoulder: np.ndarray, wrist: np.ndarray,
                                             elbow: np.ndarray, proj_point: np.ndarray,
                                             direction: np.ndarray, side: str,
                                             start_id: int) -> list:
        """
        创建臂角可视化的 RViz Markers

        显示内容:
        - 红色箭头: 肩-腕连线 (手臂主轴)
        - 绿色箭头: 肩-肘连线 (实际肘部位置)
        - 蓝色箭头: 肘部偏移方向 (发送给 IK 的 elbow_direction)
        - 黄色小球: 投影点 (肘部在肩-腕线上的投影)
        - 紫色虚线: 肘部到投影点的连线 (偏移距离)
        """
        markers = []
        timestamp = now.to_msg()

        # 1. 红色箭头: 肩 → 腕 (手臂主轴)
        m_sw = Marker()
        m_sw.header.stamp = timestamp
        m_sw.header.frame_id = frame_id
        m_sw.ns = f'{side}_arm_axis'
        m_sw.id = start_id
        m_sw.type = Marker.ARROW
        m_sw.action = Marker.ADD
        m_sw.points = [
            self._to_point(shoulder),
            self._to_point(wrist)
        ]
        m_sw.scale.x = 0.01  # 箭杆直径
        m_sw.scale.y = 0.02  # 箭头直径
        m_sw.scale.z = 0.0
        m_sw.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # 红色
        m_sw.lifetime.sec = 0
        m_sw.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_sw)

        # 2. 绿色箭头: 肩 → 肘 (实际肘部)
        m_se = Marker()
        m_se.header.stamp = timestamp
        m_se.header.frame_id = frame_id
        m_se.ns = f'{side}_shoulder_to_elbow'
        m_se.id = start_id + 1
        m_se.type = Marker.ARROW
        m_se.action = Marker.ADD
        m_se.points = [
            self._to_point(shoulder),
            self._to_point(elbow)
        ]
        m_se.scale.x = 0.01
        m_se.scale.y = 0.02
        m_se.scale.z = 0.0
        m_se.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色
        m_se.lifetime.sec = 0
        m_se.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_se)

        # 3. 蓝色箭头: 从投影点指向肘部 (elbow_direction)
        # 这个向量就是发送给 IK 的臂角约束方向
        arrow_length = 0.15  # 固定长度便于观察
        arrow_end = proj_point + direction * arrow_length
        m_dir = Marker()
        m_dir.header.stamp = timestamp
        m_dir.header.frame_id = frame_id
        m_dir.ns = f'{side}_elbow_direction'
        m_dir.id = start_id + 2
        m_dir.type = Marker.ARROW
        m_dir.action = Marker.ADD
        m_dir.points = [
            self._to_point(proj_point),
            self._to_point(arrow_end)
        ]
        m_dir.scale.x = 0.015  # 稍粗一些突出显示
        m_dir.scale.y = 0.025
        m_dir.scale.z = 0.0
        m_dir.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # 亮蓝色
        m_dir.lifetime.sec = 0
        m_dir.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_dir)

        # 4. 黄色小球: 投影点
        m_proj = Marker()
        m_proj.header.stamp = timestamp
        m_proj.header.frame_id = frame_id
        m_proj.ns = f'{side}_projection_point'
        m_proj.id = start_id + 3
        m_proj.type = Marker.SPHERE
        m_proj.action = Marker.ADD
        m_proj.pose.position = self._to_point(proj_point)
        m_proj.pose.orientation.w = 1.0
        m_proj.scale.x = 0.02
        m_proj.scale.y = 0.02
        m_proj.scale.z = 0.02
        m_proj.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 黄色
        m_proj.lifetime.sec = 0
        m_proj.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_proj)

        # 5. 紫色线: 投影点 → 肘部 (显示偏移距离)
        m_offset = Marker()
        m_offset.header.stamp = timestamp
        m_offset.header.frame_id = frame_id
        m_offset.ns = f'{side}_elbow_offset'
        m_offset.id = start_id + 4
        m_offset.type = Marker.LINE_STRIP
        m_offset.action = Marker.ADD
        m_offset.points = [
            self._to_point(proj_point),
            self._to_point(elbow)
        ]
        m_offset.scale.x = 0.008  # 线宽
        m_offset.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # 紫色
        m_offset.lifetime.sec = 0
        m_offset.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_offset)

        # 6. 文字标签: 显示 side 名称
        m_text = Marker()
        m_text.header.stamp = timestamp
        m_text.header.frame_id = frame_id
        m_text.ns = f'{side}_label'
        m_text.id = start_id + 5
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position = self._to_point(elbow + np.array([0, 0, 0.05]))
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.03  # 文字大小
        m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 白色
        m_text.text = f'{side.upper()} elbow'
        m_text.lifetime.sec = 0
        m_text.lifetime.nanosec = int(0.1 * 1e9)
        markers.append(m_text)

        return markers

    def _to_point(self, arr: np.ndarray):
        """numpy array → geometry_msgs Point"""
        from geometry_msgs.msg import Point
        return Point(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    def _compute_incremental_pose(self, current_pose, pico_frame_name: str, side: str):
        """计算 PICO tracker 的增量位姿（用于对比验证）"""
        if pico_frame_name not in self.init_tracker_poses:
            return None, None

        current_T = self._pose_to_matrix(current_pose)
        init_T = self.init_tracker_poses[pico_frame_name]

        # 位置增量
        delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]
        delta_pos_norm = np.linalg.norm(delta_pos_pico)

        # 姿态增量
        init_rot = R.from_matrix(init_T[:3, :3])
        current_rot = R.from_matrix(current_T[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()
        delta_angle = np.linalg.norm(delta_rot_pico.as_rotvec())

        # 调试输出：前 10 次调用显示增量大小
        if self.debug_counter < 10:
            self.get_logger().info(
                f'[DEBUG] {pico_frame_name}: delta_pos={delta_pos_norm:.10f}m, delta_angle={delta_angle:.10f}rad'
            )
            self.debug_counter += 1

        # 静止检测：如果增量极小（< 1e-6），使用缓存位姿
        # 这可以避免浮点误差导致的 RViz 抖动
        POSITION_THRESHOLD = 1e-6  # 0.001mm
        ROTATION_THRESHOLD = 1e-6  # ~0.00006 度

        is_static = (delta_pos_norm < POSITION_THRESHOLD and delta_angle < ROTATION_THRESHOLD)

        if is_static and pico_frame_name in self.cached_poses:
            # 使用缓存的位姿（完全静止）
            return self.cached_poses[pico_frame_name]

        # 计算新位姿
        delta_pos_world = get_pico_to_robot() @ delta_pos_pico

        # 转换到 chest 坐标系（使用共享库）
        delta_pos_chest = transform_world_to_chest(delta_pos_world, side)

        # 获取机器人初始位置
        # 对于 wrist tracker: 映射到机器人末端初始位置
        # 对于 arm tracker: 保持在原点附近，用于计算臂角方向向量
        if 'arm' in pico_frame_name and 'wrist' not in pico_frame_name:
            # 前臂 tracker: 位置用于计算臂角方向 (从配置加载)
            robot_init_pos = ARM_INIT_POS[side]
        else:
            # 手腕（pico_left_wrist, pico_right_wrist）使用 TIANJI_INIT_POS
            robot_init_pos = TIANJI_INIT_POS.get(side, np.zeros(3))

        target_pos = robot_init_pos + delta_pos_chest

        # 转换到 world (使用共享库，轴角方法)
        delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())

        # 使用共享库 4 步算法: target_chest = R_w2c @ delta_world @ R_c2w @ init_chest
        robot_init_rot_mat = TIANJI_INIT_ROT.get(side, np.eye(3))
        target_rot_in_chest = apply_world_rotation_to_chest_pose(
            robot_init_rot_mat, delta_rot_world, side
        )

        target_rot = R.from_matrix(target_rot_in_chest)
        target_quat = target_rot.as_quat()

        # 缓存位姿
        self.cached_poses[pico_frame_name] = (target_pos, target_quat)

        return target_pos, target_quat

    def _transform_hmd_to_world(self, pico_pose) -> np.ndarray:
        """将 HMD 位姿转换到世界坐标系"""
        if self.init_hmd_pose is None:
            return None

        current_T = self._pose_to_matrix(pico_pose)

        # 位置增量
        delta_pos_pico = current_T[:3, 3] - self.init_hmd_pose[:3, 3]
        delta_pos_robot = get_pico_to_robot() @ delta_pos_pico

        # HMD 初始位置设为原点上方
        target_pos = np.array([0.0, 0.0, 1.6]) + delta_pos_robot

        # 姿态增量
        current_rot = R.from_matrix(current_T[:3, :3])
        init_rot = R.from_matrix(self.init_hmd_pose[:3, :3])
        delta_rot_pico = current_rot * init_rot.inv()

        delta_rot_robot = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())
        target_rot = delta_rot_robot

        result_T = np.eye(4)
        result_T[:3, 3] = target_pos
        result_T[:3, :3] = target_rot.as_matrix()
        return result_T

    def _pose_to_matrix(self, pose) -> np.ndarray:
        """[x,y,z,qx,qy,qz,qw] → 4x4 变换矩阵"""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
        return T

    def _broadcast_tf(self, now, parent: str, child: str, pos, quat):
        """广播 TF"""
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_pose(self, publisher, now, frame_id: str, pos, quat):
        """发布 PoseStamped"""
        if publisher is None:
            return

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        publisher.publish(msg)

    def _publish_vector3(self, publisher, now, frame_id: str, vector):
        """发布 Vector3Stamped (用于 elbow direction)"""
        if publisher is None:
            return

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'data_source'):
            self.data_source.close()
            self.get_logger().info('数据源已关闭')
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Step 4: 可视化录制的 PICO 数据')
    parser.add_argument('--file', type=str,
                        default='../record/trackingData_sample_static.txt',
                        help='录制数据文件路径（相对于 test/ 目录）')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度倍率 (0.5=慢速, 1.0=实时, 2.0=快速)')
    parser.add_argument('--loop', action='store_true',
                        help='循环回放')
    args = parser.parse_args()

    # 确定数据文件路径
    data_file = Path(args.file)
    if not data_file.is_absolute():
        # 相对路径，从 test/ 目录查找
        test_dir = Path(__file__).parent
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f"❌ 错误: 找不到数据文件 {data_file}")
        print(f"   请检查文件是否存在于: {test_dir}")
        return

    print("=" * 70)
    print("Step 4: 可视化录制的 PICO 数据")
    print("=" * 70)
    print(f"  数据文件: {data_file}")
    print(f"  回放速度: {args.speed}x")
    print(f"  循环回放: {args.loop}")
    print("=" * 70)
    print("")
    print("在另一个终端启动 RViz:")
    print("  rviz2 -d $(ros2 pkg prefix pico_input)/share/pico_input/rviz/pico_simulation.rviz")
    print("")
    print("或手动配置 RViz:")
    print("  1. Fixed Frame: world")
    print("  2. Add → TF → 勾选 Show Axes & Show Names")
    print("=" * 70)
    print("")

    global _node, _shutdown_requested

    rclpy.init()
    _node = RecordedDataVisualizer(str(data_file), args.speed, args.loop)

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        print("\n✓ 用户中断")
    finally:
        if not _shutdown_requested:
            _cleanup()


if __name__ == '__main__':
    main()
