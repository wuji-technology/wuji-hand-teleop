#!/usr/bin/env python3
"""
=============================================================================
PICO Raw Data Visualization - Directly display tracker poses in PICO coordinate system
=============================================================================

Features:
  - Directly visualize PICO raw data (no coordinate transformation)
  - View PICO coordinate system orientation: +X right, +Y up, +Z forward (toward user)
  - Analyze tracker data quality (noise, jitter, dropped frames)
  - Compare stability across different trackers

Usage:
==========
  Terminal 1: Start this script
  cd ~/Desktop/wuji-hand-teleop/src/input_devices/pico_input/record
  python3 visualize_pico_raw.py

  # Use a specific data file
  python3 visualize_pico_raw.py --file trackingData_head_tracker_static.txt

  Terminal 2: Start RViz
  rviz2
  - Fixed Frame: pico_origin
  - Add -> TF -> Show Axes, Show Names

PICO Coordinate System Description:
================
  pico_origin (origin)
  +-- head (HMD headset)
  +-- left_wrist_raw (left wrist tracker)
  +-- right_wrist_raw (right wrist tracker)
  +-- left_arm_raw (left forearm tracker)
  +-- right_arm_raw (right forearm tracker)

  Axis directions (in PICO):
    Red = X axis (rightward)
    Green = Y axis (upward)
    Blue = Z axis (forward, toward user)

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
    """PICO raw data visualization node"""

    def __init__(self, data_file: str, playback_speed: float = 1.0, loop: bool = True):
        super().__init__('pico_raw_visualizer')

        self.playback_speed = playback_speed
        self.loop = loop

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publishers - raw PICO data
        self.head_pub = self.create_publisher(PoseStamped, '/pico_head_raw', 10)
        self.left_wrist_pub = self.create_publisher(PoseStamped, '/pico_left_wrist_raw', 10)
        self.right_wrist_pub = self.create_publisher(PoseStamped, '/pico_right_wrist_raw', 10)
        self.left_arm_pub = self.create_publisher(PoseStamped, '/pico_left_arm_raw', 10)
        self.right_arm_pub = self.create_publisher(PoseStamped, '/pico_right_arm_raw', 10)

        # Load data
        self.data_file = data_file
        self.frames = self._load_data(data_file)
        self.current_frame = 0
        self.last_time = time.time()

        # Statistics - includes position and orientation
        self.stats = {
            'head': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'left_wrist': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'right_wrist': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'left_arm': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
            'right_arm': {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []},
        }
        self.prev_positions = {}
        self.prev_rotations = {}

        # Tracker SN mapping
        self.tracker_map = {
            '190058': 'left_wrist',
            '190600': 'right_wrist',
            '190046': 'left_arm',
            '190023': 'right_arm',
        }

        # Publish static TF: pico_origin
        self._publish_origin()

        # Statistics print control
        self.stats_printed = False

        # Timer
        self.timer = self.create_timer(1.0 / 90.0, self._timer_callback)

        self.get_logger().info('=' * 70)
        self.get_logger().info('PICO Raw Data Visualization')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  Data file: {data_file}')
        self.get_logger().info(f'  Total frames: {len(self.frames)}')
        self.get_logger().info(f'  Playback speed: {playback_speed}x')
        self.get_logger().info(f'  Loop: {loop}')
        self.get_logger().info('-' * 70)
        self.get_logger().info('PICO Coordinate System:')
        self.get_logger().info('  Red X axis = rightward')
        self.get_logger().info('  Green Y axis = upward')
        self.get_logger().info('  Blue Z axis = forward (toward user)')
        self.get_logger().info('-' * 70)
        self.get_logger().info('RViz Configuration:')
        self.get_logger().info('  Fixed Frame: pico_origin')
        self.get_logger().info('  Add -> TF -> Show Axes, Show Names')
        self.get_logger().info('=' * 70)

    def _load_data(self, filepath: str) -> list:
        """Load recorded data"""
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
        self.get_logger().info(f'Loaded {len(frames)} frames of data')
        return frames

    def _publish_origin(self):
        """Publish pico_origin static TF"""
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
        """Timer callback: publish current frame data"""
        if not self.frames:
            return

        # Get current frame
        frame = self.frames[self.current_frame]
        now = self.get_clock().now()

        # Parse and publish Head
        if 'Head' in frame and 'pose' in frame['Head']:
            self._publish_head(now, frame['Head'])

        # Parse and publish Trackers
        if 'Motion' in frame and 'joints' in frame['Motion']:
            self._publish_trackers(now, frame['Motion']['joints'])

        # Update frame index
        self.current_frame += 1
        if self.current_frame >= len(self.frames):
            if self.loop:
                # Only print statistics on first loop iteration
                if not self.stats_printed:
                    self._print_stats()
                    self.stats_printed = True
                self.current_frame = 0
            else:
                self._print_stats()
                self.get_logger().info('Playback ended')
                self.timer.cancel()

    def _parse_pose(self, pose_str: str):
        """Parse pose string 'x,y,z,qx,qy,qz,qw'"""
        parts = [float(x) for x in pose_str.split(',')]
        pos = np.array(parts[:3])
        quat = np.array(parts[3:7])
        # Normalize quaternion
        quat = quat / np.linalg.norm(quat)
        return pos, quat

    def _publish_head(self, now, head_data):
        """Publish Head data"""
        pose_str = head_data.get('pose', '')
        if not pose_str:
            return

        pos, quat = self._parse_pose(pose_str)

        # Update statistics
        self._update_stats('head', pos, quat)

        # Publish TF
        self._broadcast_tf(now, 'pico_origin', 'head_raw', pos, quat)

        # Publish Topic
        self._publish_pose(self.head_pub, now, 'pico_origin', pos, quat)

    def _publish_trackers(self, now, joints):
        """Publish Tracker data"""
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

            # Update statistics
            self._update_stats(role, pos, quat)

            # Publish TF
            frame_name = f'{role}_raw'
            self._broadcast_tf(now, 'pico_origin', frame_name, pos, quat)

            # Publish Topic
            pub = pub_map.get(role)
            if pub:
                self._publish_pose(pub, now, 'pico_origin', pos, quat)

    def _update_stats(self, role: str, pos: np.ndarray, quat: np.ndarray):
        """Update statistics (position and orientation)"""
        self.stats[role]['positions'].append(pos.copy())
        self.stats[role]['rotations'].append(quat.copy())

        # Position delta
        if role in self.prev_positions:
            pos_delta = np.linalg.norm(pos - self.prev_positions[role])
            self.stats[role]['pos_deltas'].append(pos_delta)

        # Orientation delta (using angular difference)
        if role in self.prev_rotations:
            r1 = R.from_quat(self.prev_rotations[role])
            r2 = R.from_quat(quat)
            rot_delta = (r2 * r1.inv()).magnitude()  # Angular difference (radians)
            self.stats[role]['rot_deltas'].append(rot_delta)

        self.prev_positions[role] = pos.copy()
        self.prev_rotations[role] = quat.copy()

    def _print_stats(self):
        """Print statistics"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Sensor Quality Analysis (PICO Raw Coordinate System)')
        self.get_logger().info('=' * 70)

        for role, data in self.stats.items():
            if not data['positions']:
                continue

            positions = np.array(data['positions'])
            rotations = np.array(data['rotations']) if data['rotations'] else None
            pos_deltas = np.array(data['pos_deltas']) if data['pos_deltas'] else np.array([0])
            rot_deltas = np.array(data['rot_deltas']) if data['rot_deltas'] else np.array([0])

            # Position statistics
            pos_mean = positions.mean(axis=0)
            pos_range = positions.max(axis=0) - positions.min(axis=0)

            # Inter-frame displacement statistics
            pos_delta_mean = pos_deltas.mean() * 1000  # mm
            pos_delta_max = pos_deltas.max() * 1000    # mm
            pos_delta_std = pos_deltas.std() * 1000    # mm

            # Inter-frame rotation statistics
            rot_delta_mean = np.degrees(rot_deltas.mean())  # degrees
            rot_delta_max = np.degrees(rot_deltas.max())
            rot_delta_std = np.degrees(rot_deltas.std())

            self.get_logger().info(f'\n{role}:')
            self.get_logger().info(f'  Frames: {len(data["positions"])}')
            self.get_logger().info(f'  Position mean: X={pos_mean[0]:.4f}, Y={pos_mean[1]:.4f}, Z={pos_mean[2]:.4f} m')
            self.get_logger().info(f'  Position range: X={pos_range[0]*1000:.2f}mm, Y={pos_range[1]*1000:.2f}mm, Z={pos_range[2]*1000:.2f}mm')
            self.get_logger().info(f'  Inter-frame displacement: mean={pos_delta_mean:.4f}mm, max={pos_delta_max:.4f}mm, std={pos_delta_std:.4f}mm')
            self.get_logger().info(f'  Inter-frame rotation: mean={rot_delta_mean:.4f}deg, max={rot_delta_max:.4f}deg, std={rot_delta_std:.4f}deg')

            # Orientation analysis (Euler angles)
            if rotations is not None and len(rotations) > 0:
                mean_quat = rotations.mean(axis=0)
                mean_quat = mean_quat / np.linalg.norm(mean_quat)  # Normalize
                euler = R.from_quat(mean_quat).as_euler('xyz', degrees=True)
                self.get_logger().info(f'  Orientation mean (Euler): Roll={euler[0]:.1f}deg, Pitch={euler[1]:.1f}deg, Yaw={euler[2]:.1f}deg')

            # Quality assessment
            if pos_delta_mean < 0.1:
                quality = 'Excellent'
            elif pos_delta_mean < 0.5:
                quality = 'Good'
            elif pos_delta_mean < 2.0:
                quality = 'Fair'
            else:
                quality = 'Poor'

            self.get_logger().info(f'  Quality assessment: {quality}')

        # Axis orientation analysis
        self.get_logger().info('')
        self.get_logger().info('-' * 70)
        self.get_logger().info('Axis Orientation Analysis:')
        self.get_logger().info('-' * 70)

        # Analyze head position to infer orientation
        if self.stats['head']['positions']:
            head_mean = np.array(self.stats['head']['positions']).mean(axis=0)
            self.get_logger().info(f'  Head mean position: X={head_mean[0]:.3f}, Y={head_mean[1]:.3f}, Z={head_mean[2]:.3f}')

        # Analyze left-right hand position differences
        if self.stats['left_wrist']['positions'] and self.stats['right_wrist']['positions']:
            left_mean = np.array(self.stats['left_wrist']['positions']).mean(axis=0)
            right_mean = np.array(self.stats['right_wrist']['positions']).mean(axis=0)

            diff = right_mean - left_mean
            self.get_logger().info(f'  Left hand mean position: X={left_mean[0]:.3f}, Y={left_mean[1]:.3f}, Z={left_mean[2]:.3f}')
            self.get_logger().info(f'  Right hand mean position: X={right_mean[0]:.3f}, Y={right_mean[1]:.3f}, Z={right_mean[2]:.3f}')
            self.get_logger().info(f'  Left-right difference: dX={diff[0]:.3f}, dY={diff[1]:.3f}, dZ={diff[2]:.3f}')

            # Infer axis direction
            if abs(diff[0]) > abs(diff[1]) and abs(diff[0]) > abs(diff[2]):
                if diff[0] > 0:
                    self.get_logger().info('  -> PICO X axis: right hand is in +X direction of left hand (X = rightward)')
                else:
                    self.get_logger().info('  -> PICO X axis: left hand is in +X direction of right hand (X = leftward)')

        self.get_logger().info('')
        self.get_logger().info('=' * 70)

        # Reset statistics (using correct structure)
        for role in self.stats:
            self.stats[role] = {'positions': [], 'rotations': [], 'pos_deltas': [], 'rot_deltas': []}
        self.prev_positions = {}
        self.prev_rotations = {}

    def _broadcast_tf(self, now, parent: str, child: str, pos: np.ndarray, quat: np.ndarray):
        """Broadcast TF"""
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
        """Publish PoseStamped"""
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
    parser = argparse.ArgumentParser(description='PICO Raw Data Visualization')
    parser.add_argument('--file', type=str,
                        default='trackingData_sample.txt',
                        help='Data file path')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed')
    parser.add_argument('--no-loop', action='store_true',
                        help='Disable loop playback')
    args = parser.parse_args()

    # Determine file path
    data_file = Path(args.file)
    if not data_file.is_absolute():
        test_dir = Path(__file__).parent
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f'File not found: {data_file}')
        return

    rclpy.init()
    node = PicoRawVisualizer(str(data_file), args.speed, not args.no_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nUser interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
