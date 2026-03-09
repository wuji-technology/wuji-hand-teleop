#!/usr/bin/env python3
"""
分析 PICO 录制数据中的运动模式

功能:
  1. 分析各 tracker 在 X/Y/Z 方向的运动范围
  2. 找出包含明显运动的时间段
  3. 提取关键帧用于测试
  4. 分析旋转变化，找出包含明显旋转的时间段

使用方法:
  # 分析位置运动
  python3 analyze_motion_data.py --file trackingData_whole_data.txt

  # 分析旋转运动
  python3 analyze_motion_data.py --file trackingData_whole_data.txt --rotation

  # 同时分析位置和旋转
  python3 analyze_motion_data.py --file trackingData_whole_data.txt --all
"""

import json
import argparse
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


def load_data(filepath: str) -> list:
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
    return frames


def parse_pose(pose_str: str):
    """解析位姿字符串 'x,y,z,qx,qy,qz,qw'"""
    parts = [float(x) for x in pose_str.split(',')]
    pos = np.array(parts[:3])
    quat = np.array(parts[3:7])
    return pos, quat


def extract_tracker_positions(frames: list, tracker_map: dict) -> dict:
    """提取各 tracker 的位置序列"""
    import re

    positions = {
        'head': [],
        'left_wrist': [],
        'right_wrist': [],
        'left_arm': [],
        'right_arm': [],
    }

    for frame in frames:
        # Head
        if 'Head' in frame and 'pose' in frame['Head']:
            pos, _ = parse_pose(frame['Head']['pose'])
            positions['head'].append(pos)

        # Trackers
        if 'Motion' in frame and 'joints' in frame['Motion']:
            frame_trackers = {}
            for joint in frame['Motion']['joints']:
                sn = joint.get('sn', '')
                match = re.search(r'(\d{6})', sn)
                if match:
                    short_sn = match.group(1)
                    if short_sn in tracker_map:
                        role = tracker_map[short_sn]
                        if 'p' in joint:
                            pos, _ = parse_pose(joint['p'])
                            frame_trackers[role] = pos

            for role in ['left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
                if role in frame_trackers:
                    positions[role].append(frame_trackers[role])

    # 转换为 numpy 数组
    for role in positions:
        if positions[role]:
            positions[role] = np.array(positions[role])
        else:
            positions[role] = np.array([]).reshape(0, 3)

    return positions


def extract_tracker_poses(frames: list, tracker_map: dict) -> dict:
    """提取各 tracker 的位置和旋转序列"""
    import re

    poses = {
        'head': {'pos': [], 'quat': []},
        'left_wrist': {'pos': [], 'quat': []},
        'right_wrist': {'pos': [], 'quat': []},
        'left_arm': {'pos': [], 'quat': []},
        'right_arm': {'pos': [], 'quat': []},
    }

    for frame in frames:
        # Head
        if 'Head' in frame and 'pose' in frame['Head']:
            pos, quat = parse_pose(frame['Head']['pose'])
            poses['head']['pos'].append(pos)
            poses['head']['quat'].append(quat)

        # Trackers
        if 'Motion' in frame and 'joints' in frame['Motion']:
            frame_trackers = {}
            for joint in frame['Motion']['joints']:
                sn = joint.get('sn', '')
                match = re.search(r'(\d{6})', sn)
                if match:
                    short_sn = match.group(1)
                    if short_sn in tracker_map:
                        role = tracker_map[short_sn]
                        if 'p' in joint:
                            pos, quat = parse_pose(joint['p'])
                            frame_trackers[role] = (pos, quat)

            for role in ['left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
                if role in frame_trackers:
                    poses[role]['pos'].append(frame_trackers[role][0])
                    poses[role]['quat'].append(frame_trackers[role][1])

    # 转换为 numpy 数组
    for role in poses:
        if poses[role]['pos']:
            poses[role]['pos'] = np.array(poses[role]['pos'])
            poses[role]['quat'] = np.array(poses[role]['quat'])
        else:
            poses[role]['pos'] = np.array([]).reshape(0, 3)
            poses[role]['quat'] = np.array([]).reshape(0, 4)

    return poses


def quaternion_angle_diff(q1: np.ndarray, q2: np.ndarray) -> float:
    """计算两个四元数之间的角度差（弧度）"""
    # 确保四元数符号一致（取较短路径）
    if np.dot(q1, q2) < 0:
        q2 = -q2

    # 计算相对旋转
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    r_diff = r2 * r1.inv()

    # 返回旋转角度
    return r_diff.magnitude()


def analyze_rotation(quats: np.ndarray, role: str, window_size: int = 30):
    """分析旋转模式"""
    if len(quats) < window_size:
        print(f"  {role}: 数据不足")
        return None

    # 计算帧间旋转角度
    frame_angles = []
    for i in range(1, len(quats)):
        angle = quaternion_angle_diff(quats[i-1], quats[i])
        frame_angles.append(np.degrees(angle))
    frame_angles = np.array(frame_angles)

    # 整体统计
    total_rotation = np.sum(frame_angles)
    max_frame_rotation = np.max(frame_angles)
    mean_frame_rotation = np.mean(frame_angles)

    print(f"\n{role}:")
    print(f"  帧数: {len(quats)}")
    print(f"  累计旋转: {total_rotation:.2f}°")
    print(f"  帧间最大旋转: {max_frame_rotation:.2f}°")
    print(f"  帧间平均旋转: {mean_frame_rotation:.4f}°")

    # 找出窗口内累计旋转最大的片段
    max_window_rotation = 0
    max_start = 0
    for i in range(len(frame_angles) - window_size + 1):
        window_rotation = np.sum(frame_angles[i:i+window_size])
        if window_rotation > max_window_rotation:
            max_window_rotation = window_rotation
            max_start = i

    # 计算窗口起止的相对旋转
    if max_start + window_size < len(quats):
        q_start = quats[max_start]
        q_end = quats[max_start + window_size]
        relative_angle = np.degrees(quaternion_angle_diff(q_start, q_end))

        # 分析主要旋转轴
        r_start = R.from_quat(q_start)
        r_end = R.from_quat(q_end)
        r_diff = r_end * r_start.inv()
        rotvec = r_diff.as_rotvec()
        axis = rotvec / (np.linalg.norm(rotvec) + 1e-10)

        print(f"  最大旋转片段: 帧 {max_start}-{max_start+window_size}")
        print(f"    累计旋转: {max_window_rotation:.2f}°")
        print(f"    起止相对旋转: {relative_angle:.2f}°")
        print(f"    主旋转轴 (PICO): [{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}]")

    return {
        'total_rotation': total_rotation,
        'max_frame_rotation': max_frame_rotation,
        'max_window_start': max_start,
        'max_window_end': max_start + window_size,
        'max_window_rotation': max_window_rotation,
    }


def find_rotation_segments(poses: dict, min_rotation_deg: float = 10.0, window_size: int = 30):
    """找出包含明显旋转的片段"""
    print("\n" + "=" * 70)
    print("Wrist Tracker 旋转片段分析")
    print("=" * 70)
    print(f"最小旋转阈值: {min_rotation_deg}°")

    segments = {'left_wrist': [], 'right_wrist': []}

    for role in ['left_wrist', 'right_wrist']:
        quats = poses[role]['quat']
        if len(quats) < window_size:
            continue

        print(f"\n{role}:")

        # 计算帧间旋转角度
        frame_angles = []
        for i in range(1, len(quats)):
            angle = quaternion_angle_diff(quats[i-1], quats[i])
            frame_angles.append(np.degrees(angle))
        frame_angles = np.array(frame_angles)

        # 滑动窗口找旋转片段
        for i in range(0, len(frame_angles) - window_size + 1, window_size // 2):
            window_rotation = np.sum(frame_angles[i:i+window_size])

            if window_rotation >= min_rotation_deg:
                # 计算起止相对旋转和主轴
                q_start = quats[i]
                q_end = quats[i + window_size]
                relative_angle = np.degrees(quaternion_angle_diff(q_start, q_end))

                r_start = R.from_quat(q_start)
                r_end = R.from_quat(q_end)
                r_diff = r_end * r_start.inv()
                rotvec = r_diff.as_rotvec()
                angle_mag = np.linalg.norm(rotvec)
                axis = rotvec / (angle_mag + 1e-10) if angle_mag > 1e-6 else np.array([0, 0, 1])

                # 判断主旋转方向
                abs_axis = np.abs(axis)
                main_axis_idx = np.argmax(abs_axis)
                axis_names = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)']
                main_axis = axis_names[main_axis_idx]
                axis_sign = '+' if axis[main_axis_idx] > 0 else '-'

                segments[role].append({
                    'start': i,
                    'end': i + window_size,
                    'cumulative_rotation': window_rotation,
                    'relative_rotation': relative_angle,
                    'main_axis': main_axis,
                    'axis_sign': axis_sign,
                    'axis_vector': axis.tolist(),
                })

                print(f"  帧 {i:5d}-{i+window_size:5d}: "
                      f"累计 {window_rotation:6.2f}°, "
                      f"相对 {relative_angle:6.2f}°, "
                      f"主轴: {axis_sign}{main_axis}")

    return segments


def analyze_motion(positions: np.ndarray, role: str, window_size: int = 30):
    """分析运动模式"""
    if len(positions) < window_size:
        print(f"  {role}: 数据不足")
        return None

    # 整体统计
    pos_range = positions.max(axis=0) - positions.min(axis=0)
    pos_mean = positions.mean(axis=0)

    print(f"\n{role}:")
    print(f"  帧数: {len(positions)}")
    print(f"  位置均值 (PICO): X={pos_mean[0]:.4f}, Y={pos_mean[1]:.4f}, Z={pos_mean[2]:.4f}")
    print(f"  位置范围 (PICO): X={pos_range[0]*100:.2f}cm, Y={pos_range[1]*100:.2f}cm, Z={pos_range[2]*100:.2f}cm")

    # 找出各方向运动最明显的片段
    results = {}
    for axis, axis_name, pico_dir in [(0, 'X', '左右'), (1, 'Y', '上下'), (2, 'Z', '前后')]:
        # 滑动窗口找最大运动范围
        max_range = 0
        max_start = 0
        for i in range(len(positions) - window_size):
            segment = positions[i:i+window_size, axis]
            seg_range = segment.max() - segment.min()
            if seg_range > max_range:
                max_range = seg_range
                max_start = i

        # 方向判断
        segment = positions[max_start:max_start+window_size, axis]
        direction = "+" if segment[-1] > segment[0] else "-"

        results[axis_name] = {
            'start': max_start,
            'end': max_start + window_size,
            'range_cm': max_range * 100,
            'direction': direction
        }

        print(f"  {axis_name} ({pico_dir}): 最大运动 {max_range*100:.2f}cm @ 帧 {max_start}-{max_start+window_size} (方向: {direction})")

    return results


def find_arm_movement_segments(positions: dict, min_movement_cm: float = 3.0, window_size: int = 30):
    """找出 arm tracker 有明显运动的片段"""
    print("\n" + "=" * 70)
    print("Arm Tracker 运动片段分析")
    print("=" * 70)

    segments = {'left_arm': [], 'right_arm': []}

    for role in ['left_arm', 'right_arm']:
        pos = positions[role]
        if len(pos) < window_size:
            continue

        print(f"\n{role}:")

        for axis, axis_name, pico_dir in [(0, 'X', '左右'), (1, 'Y', '上下'), (2, 'Z', '前后')]:
            # 找出所有超过阈值的运动片段
            for i in range(0, len(pos) - window_size, window_size // 2):
                segment = pos[i:i+window_size, axis]
                seg_range = segment.max() - segment.min()

                if seg_range * 100 >= min_movement_cm:
                    direction = "+" if segment[-1] > segment[0] else "-"
                    segments[role].append({
                        'axis': axis_name,
                        'pico_dir': pico_dir,
                        'start': i,
                        'end': i + window_size,
                        'range_cm': seg_range * 100,
                        'direction': direction,
                        'start_pos': pos[i].tolist(),
                        'end_pos': pos[i + window_size - 1].tolist()
                    })
                    print(f"  帧 {i:5d}-{i+window_size:5d}: {axis_name} ({pico_dir}) 移动 {seg_range*100:5.2f}cm ({direction})")

    return segments


def main():
    parser = argparse.ArgumentParser(description='分析 PICO 录制数据中的运动模式')
    parser.add_argument('--file', type=str, default='trackingData_whole_data.txt',
                        help='数据文件路径')
    parser.add_argument('--window', type=int, default=30,
                        help='滑动窗口大小（帧数）')
    parser.add_argument('--min-movement', type=float, default=3.0,
                        help='最小位置运动阈值 (cm)')
    parser.add_argument('--min-rotation', type=float, default=10.0,
                        help='最小旋转阈值 (度)')
    parser.add_argument('--rotation', action='store_true',
                        help='分析旋转运动')
    parser.add_argument('--all', action='store_true',
                        help='同时分析位置和旋转')
    args = parser.parse_args()

    # 确定文件路径
    data_file = Path(args.file)
    if not data_file.is_absolute():
        data_file = Path(__file__).parent / args.file

    if not data_file.exists():
        print(f'❌ 找不到文件: {data_file}')
        return

    # Tracker SN 映射
    tracker_map = {
        '190058': 'left_wrist',
        '190600': 'right_wrist',
        '190046': 'left_arm',
        '190023': 'right_arm',
    }

    print("=" * 70)
    print("PICO 录制数据运动分析")
    print("=" * 70)
    print(f"数据文件: {data_file}")
    print(f"窗口大小: {args.window} 帧")
    print(f"最小位置运动阈值: {args.min_movement} cm")
    print(f"最小旋转阈值: {args.min_rotation}°")
    print(f"分析模式: {'全部' if args.all else ('旋转' if args.rotation else '位置')}")

    # 加载数据
    frames = load_data(str(data_file))
    print(f"加载了 {len(frames)} 帧数据")

    # 提取位置和旋转
    poses = extract_tracker_poses(frames, tracker_map)

    # 位置分析
    do_position = not args.rotation or args.all
    do_rotation = args.rotation or args.all

    if do_position:
        # 提取位置（兼容旧接口）
        positions = {role: poses[role]['pos'] for role in poses}

        # 分析各 tracker 运动
        print("\n" + "=" * 70)
        print("各 Tracker 整体运动分析 (PICO 坐标系)")
        print("=" * 70)
        print("PICO 坐标系: X=左右, Y=上下, Z=前后(朝向用户)")

        for role in ['head', 'left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
            if len(positions[role]) > 0:
                analyze_motion(positions[role], role, args.window)

        # 找出 arm tracker 运动片段
        segments = find_arm_movement_segments(positions, args.min_movement, args.window)

        # 位置测试建议
        print("\n" + "=" * 70)
        print("位置测试的数据片段建议")
        print("=" * 70)
        print("\n可以使用以下帧范围测试机械臂位置运动:")

        for role in ['left_wrist', 'right_wrist']:
            pos = positions[role]
            if len(pos) > args.window:
                print(f"\n{role}:")
                for axis, axis_name, pico_dir in [(0, 'X', '左右'), (1, 'Y', '上下'), (2, 'Z', '前后')]:
                    max_range = 0
                    max_start = 0
                    for i in range(len(pos) - args.window):
                        segment = pos[i:i+args.window, axis]
                        seg_range = segment.max() - segment.min()
                        if seg_range > max_range:
                            max_range = seg_range
                            max_start = i
                    if max_range * 100 >= args.min_movement:
                        print(f"  {axis_name} ({pico_dir}): 帧 {max_start}-{max_start+args.window} (移动 {max_range*100:.2f}cm)")

    if do_rotation:
        # 分析旋转
        print("\n" + "=" * 70)
        print("各 Tracker 旋转分析 (PICO 坐标系)")
        print("=" * 70)
        print("PICO 坐标系: X=Roll(左右倾斜), Y=Pitch(俯仰), Z=Yaw(水平旋转)")

        rotation_results = {}
        for role in ['left_wrist', 'right_wrist']:
            if len(poses[role]['quat']) > 0:
                rotation_results[role] = analyze_rotation(poses[role]['quat'], role, args.window)

        # 找出旋转片段
        rotation_segments = find_rotation_segments(poses, args.min_rotation, args.window)

        # 旋转测试建议
        print("\n" + "=" * 70)
        print("旋转测试的数据片段建议")
        print("=" * 70)
        print("\n可以使用以下帧范围测试机械臂旋转控制:")

        for role in ['left_wrist', 'right_wrist']:
            if rotation_segments[role]:
                print(f"\n{role}:")
                # 按主旋转轴分组
                by_axis = {}
                for seg in rotation_segments[role]:
                    key = seg['main_axis']
                    if key not in by_axis:
                        by_axis[key] = []
                    by_axis[key].append(seg)

                for axis_key, segs in by_axis.items():
                    best = max(segs, key=lambda x: x['relative_rotation'])
                    sign = best['axis_sign']
                    print(f"  {sign}{axis_key}: 帧 {best['start']}-{best['end']} "
                          f"(旋转 {best['relative_rotation']:.2f}°)")

    print("\n" + "=" * 70)
    print("提示: 使用 step5 测试时添加 --start-frame 和 --end-frame 参数")
    print("=" * 70)


if __name__ == '__main__':
    main()
