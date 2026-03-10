#!/usr/bin/env python3
"""
=============================================================================
PICO 原始数据可视化 - 直接显示 PICO 坐标系中的 tracker 位姿
=============================================================================

功能：
  - 直接可视化 PICO 原始数据（不做坐标变换）
  - 查看 PICO 坐标系方向: +X右, +Y上, +Z前(toward user)
  - 分析 tracker 数据质量（噪声、抖动、丢帧）
  - 对比不同 tracker 的稳定性

使用方法：
==========
  终端 1: 启动本脚本
  cd /home/wuji-08/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/record
  python3 visualize_pico_raw.py

  # 使用指定数据文件
  python3 visualize_pico_raw.py --file trackingData_head_tracker_static.txt

  终端 2: 启动 RViz
  rviz2
  - Fixed Frame: pico_origin
  - Add → TF → Show Axes, Show Names

PICO 坐标系说明:
================
  pico_origin (原点)
  ├── head (HMD 头显)
  ├── left_wrist_raw (左手腕 tracker)
  ├── right_wrist_raw (右手腕 tracker)
  ├── left_arm_raw (左前臂 tracker)
  └── right_arm_raw (右前臂 tracker)

  坐标轴方向（在 PICO 中）:
    红色 = X 轴 (向右)
    绿色 = Y 轴 (向上)
    蓝色 = Z 轴 (向前，朝向用户)

=============================================================================
"""

import sys
import json
import time
import argparse
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class PicoRawVisualizer(Node):
    """PICO 原始数据可视化节点"""

    def __init__(self, data_file: str, playback_speed: float = 1.0, loop: bool = True):
        super().__init__('pico_raw_visualizer')

        self.playback_speed = playback_speed
        self.loop = loop

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 发布器 - 原始 PICO 数据
        self.head_pub = self.create_publisher(PoseStamped, '/pico_head_raw', 10)
        self.left_wrist_pub = self.create_publisher(PoseStamped, '/pico_left_wrist_raw', 10)
        self.right_wrist_pub = self.create_publisher(PoseStamped, '/pico_right_wrist_raw', 10)
        self.left_arm_pub = self.create_publisher(PoseStamped, '/pico_left_arm_raw', 10)
        self.right_arm_pub = self.create_publisher(PoseStamped, '/pico_right_arm_raw', 10)

        # 加载数据
        self.data_file = data_file
        self.frames = self._load_data(data_file)
        self.current_frame = 0
        self.last_time = time.time()

        # 统计信息 - 包含位置和姿态
        self.stats = {
            'head': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'left_wrist': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'right_wrist': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'left_arm': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'right_arm': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
        }
        self.prev_positions = {}
        self.prev_rotations = {}

        # Tracker SN 映射
        self.tracker_map = {
            '190058': 'left_wrist',
            '190600': 'right_wrist',
            '190046': 'left_arm',
            '190023': 'right_arm',
        }

        # 发布静态 TF: pico_origin
        self._publish_origin()

        # 统计打印控制
        self.stats_printed = False

        # 定时器
        self.timer = self.create_timer(1.0 / 90.0, self._timer_callback)

        self.get_logger().info('=' * 70)
        self.get_logger().info('PICO 原始数据可视化')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  数据文件: {data_file}')
        self.get_logger().info(f'  总帧数: {len(self.frames)}')
        self.get_logger().info(f'  回放速度: {playback_speed}x')
        self.get_logger().info(f'  循环: {loop}')
        self.get_logger().info('-' * 70)
        self.get_logger().info('PICO 坐标系:')
        self.get_logger().info('  红色 X 轴 = 向右')
        self.get_logger().info('  绿色 Y 轴 = 向上')
        self.get_logger().info('  蓝色 Z 轴 = 向前(朝向用户)')
        self.get_logger().info('-' * 70)
        self.get_logger().info('RViz 配置:')
        self.get_logger().info('  Fixed Frame: pico_origin')
        self.get_logger().info('  Add → TF → Show Axes, Show Names')
        self.get_logger().info('=' * 70)

    def _load_data(self, filepath: str) -> list:
        """加载录制数据"""
        frames = []
        with open(filepath, 'r') as f:
            for i, line in enumerate(f):
                if i == 0 and 'notice' in line:
                    continue
                try:
                    data = json.loads(line.strip())
                    frames.append(data)
                except:
                    pass
        self.get_logger().info(f'加载了 {len(frames)} 帧数据')
        return frames

    def _publish_origin(self):
        """发布 pico_origin 静态 TF"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'pico_origin'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def _timer_callback(self):
        """定时回调：发布当前帧数据"""
        if not self.frames:
            return

        # 获取当前帧
        frame = self.frames[self.current_frame]
        now = self.get_clock().now()

        # 解析并发布 Head
        if 'Head' in frame and 'pose' in frame['Head']:
            self._publish_head(now, frame['Head'])

        # 解析并发布 Trackers
        if 'Motion' in frame and 'joints' in frame['Motion']:
            self._publish_trackers(now, frame['Motion']['joints'])

        # 更新帧索引
        self.current_frame += 1
        if self.current_frame >= len(self.frames):
            if self.loop:
                # 循环播放时只在第一次打印统计
                if not self.stats_printed:
                    self._print_stats()
                    self.stats_printed = True
                self.current_frame = 0
            else:
                self._print_stats()
                self.get_logger().info('回放结束')
                self.timer.cancel()

    def _parse_pose(self, pose_str: str):
        """解析位姿字符串 'x,y,z,qx,qy,qz,qw'"""
        parts = [float(x) for x in pose_str.split(',')]
        pos = np.array(parts[:3])
        quat = np.array(parts[3:7])
        # 归一化四元数
        quat = quat / np.linalg.norm(quat)
        return pos, quat

    def _publish_head(self, now, head_data):
        """发布 Head 数据"""
        pose_str = head_data.get('pose', '')
        if not pose_str:
            return

        pos, quat = self._parse_pose(pose_str)

        # 更新统计
        self._update_stats('head', pos, quat)

        # 发布 TF
        self._broadcast_tf(now, 'pico_origin', 'head_raw', pos, quat)

        # 发布 Topic
        self._publish_pose(self.head_pub, now, 'pico_origin', pos, quat)

    def _publish_trackers(self, now, joints):
        """发布 Tracker 数据"""
        import re

        pub_map = {
            'left_wrist': self.left_wrist_pub,
            'right_wrist': self.right_wrist_pub,
            'left_arm': self.left_arm_pub,
            'right_arm': self.right_arm_pub,
        }

        for joint in joints:
            sn = joint.get('sn', '')
            match = re.search(r'(\d{6})', sn)
            if not match:
                continue

            short_sn = match.group(1)
            if short_sn not in self.tracker_map:
                continue

            role = self.tracker_map[short_sn]
            pose_str = joint.get('p', '')
            if not pose_str:
                continue

            pos, quat = self._parse_pose(pose_str)

            # 更新统计
            self._update_stats(role, pos, quat)

            # 发布 TF
            frame_name = f'{role}_raw'
            self._broadcast_tf(now, 'pico_origin', frame_name, pos, quat)

            # 发布 Topic
            pub = pub_map.get(role)
            if pub:
                self._publish_pose(pub, now, 'pico_origin', pos, quat)

    def _update_stats(self, role: str, pos: np.ndarray, quat: np.ndarray):
        """更新统计信息（位置和姿态）"""
        self.stats[role]['positions'].append(pos.copy())
        self.stats[role]['rotations'].append(quat.copy())

        # 位置增量
        if role in self.prev_positions:
            pos_delta = np.linalg.norm(pos - self.prev_positions[role])
            self.stats[role]['pos_deltas'].append(pos_delta)

        # 姿态增量（使用角度差）
        if role in self.prev_rotations:
            r1 = R.from_quat(self.prev_rotations[role])
            r2 = R.from_quat(quat)
            rot_delta = (r2 * r1.inv()).magnitude()  # 角度差（弧度）
            self.stats[role]['rot_deltas'].append(rot_delta)

        self.prev_positions[role] = pos.copy()
        self.prev_rotations[role] = quat.copy()

    def _print_stats(self):
        """打印统计信息"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('传感器质量分析 (PICO 原始坐标系)')
        self.get_logger().info('=' * 70)

        for role, data in self.stats.items():
            if not data['positions']:
                continue

            positions = np.array(data['positions'])
            rotations = np.array(data['rotations']) if data['rotations'] else None
            pos_deltas = np.array(data['pos_deltas']) if data['pos_deltas'] else np.array([0])
            rot_deltas = np.array(data['rot_deltas']) if data['rot_deltas'] else np.array([0])

            # 位置统计
            pos_mean = positions.mean(axis=0)
            pos_range = positions.max(axis=0) - positions.min(axis=0)

            # 帧间位移统计
            pos_delta_mean = pos_deltas.mean() * 1000  # mm
            pos_delta_max = pos_deltas.max() * 1000    # mm
            pos_delta_std = pos_deltas.std() * 1000    # mm

            # 帧间旋转统计
            rot_delta_mean = np.degrees(rot_deltas.mean())  # degrees
            rot_delta_max = np.degrees(rot_deltas.max())
            rot_delta_std = np.degrees(rot_deltas.std())

            self.get_logger().info(f'\n{role}:')
            self.get_logger().info(f'  帧数: {len(data["positions"])}')
            self.get_logger().info(f'  位置均值: X={pos_mean[0]:.4f}, Y={pos_mean[1]:.4f}, Z={pos_mean[2]:.4f} m')
            self.get_logger().info(f'  位置范围: X={pos_range[0]*1000:.2f}mm, Y={pos_range[1]*1000:.2f}mm, Z={pos_range[2]*1000:.2f}mm')
            self.get_logger().info(f'  帧间位移: 均值={pos_delta_mean:.4f}mm, 最大={pos_delta_max:.4f}mm, 标准差={pos_delta_std:.4f}mm')
            self.get_logger().info(f'  帧间旋转: 均值={rot_delta_mean:.4f}°, 最大={rot_delta_max:.4f}°, 标准差={rot_delta_std:.4f}°')

            # 姿态分析（欧拉角）
            if rotations is not None and len(rotations) > 0:
                mean_quat = rotations.mean(axis=0)
                mean_quat = mean_quat / np.linalg.norm(mean_quat)  # 归一化
                euler = R.from_quat(mean_quat).as_euler('xyz', degrees=True)
                self.get_logger().info(f'  姿态均值 (欧拉角): Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°')

            # 质量评估
            if pos_delta_mean < 0.1:
                quality = '⭐ 极佳'
            elif pos_delta_mean < 0.5:
                quality = '✓ 良好'
            elif pos_delta_mean < 2.0:
                quality = '⚠ 一般'
            else:
                quality = '✗ 较差'

            self.get_logger().info(f'  质量评估: {quality}')

        # 坐标轴朝向分析
        self.get_logger().info('')
        self.get_logger().info('-' * 70)
        self.get_logger().info('坐标轴朝向分析:')
        self.get_logger().info('-' * 70)

        # 分析 head 位置来推断朝向
        if self.stats['head']['positions']:
            head_mean = np.array(self.stats['head']['positions']).mean(axis=0)
            self.get_logger().info(f'  Head 平均位置: X={head_mean[0]:.3f}, Y={head_mean[1]:.3f}, Z={head_mean[2]:.3f}')

        # 分析左右手位置差异
        if self.stats['left_wrist']['positions'] and self.stats['right_wrist']['positions']:
            left_mean = np.array(self.stats['left_wrist']['positions']).mean(axis=0)
            right_mean = np.array(self.stats['right_wrist']['positions']).mean(axis=0)

            diff = right_mean - left_mean
            self.get_logger().info(f'  左手平均位置: X={left_mean[0]:.3f}, Y={left_mean[1]:.3f}, Z={left_mean[2]:.3f}')
            self.get_logger().info(f'  右手平均位置: X={right_mean[0]:.3f}, Y={right_mean[1]:.3f}, Z={right_mean[2]:.3f}')
            self.get_logger().info(f'  左右差异: ΔX={diff[0]:.3f}, ΔY={diff[1]:.3f}, ΔZ={diff[2]:.3f}')

            # 推断坐标轴
            if abs(diff[0]) > abs(diff[1]) and abs(diff[0]) > abs(diff[2]):
                if diff[0] > 0:
                    self.get_logger().info('  → PICO X 轴: 右手在左手的 +X 方向 (X = 向右)')
                else:
                    self.get_logger().info('  → PICO X 轴: 左手在右手的 +X 方向 (X = 向左)')

        self.get_logger().info('')
        self.get_logger().info('=' * 70)

        # 重置统计（使用正确的结构）
        for role in self.stats:
            self.stats[role] = {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []}
        self.prev_positions = {}
        self.prev_rotations = {}

    def _broadcast_tf(self, now, parent: str, child: str, pos: np.ndarray, quat: np.ndarray):
        """发布 TF"""
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

    def _publish_pose(self, publisher, now, frame_id: str, pos: np.ndarray, quat: np.ndarray):
        """发布 PoseStamped"""
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


def main():
    parser = argparse.ArgumentParser(description='PICO 原始数据可视化')
    parser.add_argument('--file', type=str,
                        default='trackingData_sample.txt',
                        help='数据文件路径')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度')
    parser.add_argument('--no-loop', action='store_true',
                        help='不循环回放')
    args = parser.parse_args()

    # 确定文件路径
    data_file = Path(args.file)
    if not data_file.is_absolute():
        test_dir = Path(__file__).parent
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f'❌ 找不到文件: {data_file}')
        return

    rclpy.init()
    node = PicoRawVisualizer(str(data_file), args.speed, not args.no_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n✓ 用户中断')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
