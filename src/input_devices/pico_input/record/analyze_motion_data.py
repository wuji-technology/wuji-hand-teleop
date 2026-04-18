#!/usr/bin/env python3
"""
Analyze motion patterns in PICO recorded data

Features:
  1. Analyze motion range of each tracker along X/Y/Z axes
  2. Find time segments containing significant motion
  3. Extract key frames for testing
  4. Analyze rotation changes, find time segments with significant rotation

Usage:
  # Analyze position motion
  python3 analyze_motion_data.py --file trackingData_whole_data.txt

  # Analyze rotation motion
  python3 analyze_motion_data.py --file trackingData_whole_data.txt --rotation

  # Analyze both position and rotation
  python3 analyze_motion_data.py --file trackingData_whole_data.txt --all
"""

import json
import argparse
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


def load_data(filepath: str) -> list:
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
    return frames


def parse_pose(pose_str: str):
    """Parse pose string 'x,y,z,qx,qy,qz,qw'"""
    parts = [float(x) for x in pose_str.split(',')]
    pos = np.array(parts[:3])
    quat = np.array(parts[3:7])
    return pos, quat


def extract_tracker_positions(frames: list, tracker_map: dict) -> dict:
    """Extract position sequences for each tracker"""
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

    # Convert to numpy arrays
    for role in positions:
        if positions[role]:
            positions[role] = np.array(positions[role])
        else:
            positions[role] = np.array([]).reshape(0, 3)

    return positions


def extract_tracker_poses(frames: list, tracker_map: dict) -> dict:
    """Extract position and rotation sequences for each tracker"""
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

    # Convert to numpy arrays
    for role in poses:
        if poses[role]['pos']:
            poses[role]['pos'] = np.array(poses[role]['pos'])
            poses[role]['quat'] = np.array(poses[role]['quat'])
        else:
            poses[role]['pos'] = np.array([]).reshape(0, 3)
            poses[role]['quat'] = np.array([]).reshape(0, 4)

    return poses


def quaternion_angle_diff(q1: np.ndarray, q2: np.ndarray) -> float:
    """Compute angular difference between two quaternions (radians)"""
    # Ensure consistent quaternion signs (take shorter path)
    if np.dot(q1, q2) < 0:
        q2 = -q2

    # Compute relative rotation
    r1 = R.from_quat(q1)
    r2 = R.from_quat(q2)
    r_diff = r2 * r1.inv()

    # Return rotation angle
    return r_diff.magnitude()


def analyze_rotation(quats: np.ndarray, role: str, window_size: int = 30):
    """Analyze rotation patterns"""
    if len(quats) < window_size:
        print(f"  {role}: Insufficient data")
        return None

    # Compute inter-frame rotation angles
    frame_angles = []
    for i in range(1, len(quats)):
        angle = quaternion_angle_diff(quats[i-1], quats[i])
        frame_angles.append(np.degrees(angle))
    frame_angles = np.array(frame_angles)

    # Overall statistics
    total_rotation = np.sum(frame_angles)
    max_frame_rotation = np.max(frame_angles)
    mean_frame_rotation = np.mean(frame_angles)

    print(f"\n{role}:")
    print(f"  Frames: {len(quats)}")
    print(f"  Cumulative rotation: {total_rotation:.2f} deg")
    print(f"  Max inter-frame rotation: {max_frame_rotation:.2f} deg")
    print(f"  Mean inter-frame rotation: {mean_frame_rotation:.4f} deg")

    # Find segment with maximum cumulative rotation within window
    max_window_rotation = 0
    max_start = 0
    for i in range(len(frame_angles) - window_size + 1):
        window_rotation = np.sum(frame_angles[i:i+window_size])
        if window_rotation > max_window_rotation:
            max_window_rotation = window_rotation
            max_start = i

    # Compute relative rotation between window start and end
    if max_start + window_size < len(quats):
        q_start = quats[max_start]
        q_end = quats[max_start + window_size]
        relative_angle = np.degrees(quaternion_angle_diff(q_start, q_end))

        # Analyze primary rotation axis
        r_start = R.from_quat(q_start)
        r_end = R.from_quat(q_end)
        r_diff = r_end * r_start.inv()
        rotvec = r_diff.as_rotvec()
        axis = rotvec / (np.linalg.norm(rotvec) + 1e-10)

        print(f"  Max rotation segment: frames {max_start}-{max_start+window_size}")
        print(f"    Cumulative rotation: {max_window_rotation:.2f} deg")
        print(f"    Start-end relative rotation: {relative_angle:.2f} deg")
        print(f"    Primary rotation axis (PICO): [{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}]")

    return {
        'total_rotation': total_rotation,
        'max_frame_rotation': max_frame_rotation,
        'max_window_start': max_start,
        'max_window_end': max_start + window_size,
        'max_window_rotation': max_window_rotation,
    }


def find_rotation_segments(poses: dict, min_rotation_deg: float = 10.0, window_size: int = 30):
    """Find segments containing significant rotation"""
    print("\n" + "=" * 70)
    print("Wrist Tracker Rotation Segment Analysis")
    print("=" * 70)
    print(f"Minimum rotation threshold: {min_rotation_deg} deg")

    segments = {'left_wrist': [], 'right_wrist': []}

    for role in ['left_wrist', 'right_wrist']:
        quats = poses[role]['quat']
        if len(quats) < window_size:
            continue

        print(f"\n{role}:")

        # Compute inter-frame rotation angles
        frame_angles = []
        for i in range(1, len(quats)):
            angle = quaternion_angle_diff(quats[i-1], quats[i])
            frame_angles.append(np.degrees(angle))
        frame_angles = np.array(frame_angles)

        # Sliding window to find rotation segments
        for i in range(0, len(frame_angles) - window_size + 1, window_size // 2):
            window_rotation = np.sum(frame_angles[i:i+window_size])

            if window_rotation >= min_rotation_deg:
                # Compute start-end relative rotation and primary axis
                q_start = quats[i]
                q_end = quats[i + window_size]
                relative_angle = np.degrees(quaternion_angle_diff(q_start, q_end))

                r_start = R.from_quat(q_start)
                r_end = R.from_quat(q_end)
                r_diff = r_end * r_start.inv()
                rotvec = r_diff.as_rotvec()
                angle_mag = np.linalg.norm(rotvec)
                axis = rotvec / (angle_mag + 1e-10) if angle_mag > 1e-6 else np.array([0, 0, 1])

                # Determine primary rotation direction
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

                print(f"  Frame {i:5d}-{i+window_size:5d}: "
                      f"cumulative {window_rotation:6.2f} deg, "
                      f"relative {relative_angle:6.2f} deg, "
                      f"primary axis: {axis_sign}{main_axis}")

    return segments


def analyze_motion(positions: np.ndarray, role: str, window_size: int = 30):
    """Analyze motion patterns"""
    if len(positions) < window_size:
        print(f"  {role}: Insufficient data")
        return None

    # Overall statistics
    pos_range = positions.max(axis=0) - positions.min(axis=0)
    pos_mean = positions.mean(axis=0)

    print(f"\n{role}:")
    print(f"  Frames: {len(positions)}")
    print(f"  Position mean (PICO): X={pos_mean[0]:.4f}, Y={pos_mean[1]:.4f}, Z={pos_mean[2]:.4f}")
    print(f"  Position range (PICO): X={pos_range[0]*100:.2f}cm, Y={pos_range[1]*100:.2f}cm, Z={pos_range[2]*100:.2f}cm")

    # Find segment with maximum motion in each direction
    results = {}
    for axis, axis_name, pico_dir in [(0, 'X', 'left-right'), (1, 'Y', 'up-down'), (2, 'Z', 'front-back')]:
        # Sliding window to find maximum motion range
        max_range = 0
        max_start = 0
        for i in range(len(positions) - window_size):
            segment = positions[i:i+window_size, axis]
            seg_range = segment.max() - segment.min()
            if seg_range > max_range:
                max_range = seg_range
                max_start = i

        # Direction determination
        segment = positions[max_start:max_start+window_size, axis]
        direction = "+" if segment[-1] > segment[0] else "-"

        results[axis_name] = {
            'start': max_start,
            'end': max_start + window_size,
            'range_cm': max_range * 100,
            'direction': direction
        }

        print(f"  {axis_name} ({pico_dir}): max motion {max_range*100:.2f}cm @ frames {max_start}-{max_start+window_size} (direction: {direction})")

    return results


def find_arm_movement_segments(positions: dict, min_movement_cm: float = 3.0, window_size: int = 30):
    """Find segments where arm tracker has significant motion"""
    print("\n" + "=" * 70)
    print("Arm Tracker Motion Segment Analysis")
    print("=" * 70)

    segments = {'left_arm': [], 'right_arm': []}

    for role in ['left_arm', 'right_arm']:
        pos = positions[role]
        if len(pos) < window_size:
            continue

        print(f"\n{role}:")

        for axis, axis_name, pico_dir in [(0, 'X', 'left-right'), (1, 'Y', 'up-down'), (2, 'Z', 'front-back')]:
            # Find all motion segments exceeding threshold
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
                    print(f"  Frame {i:5d}-{i+window_size:5d}: {axis_name} ({pico_dir}) moved {seg_range*100:5.2f}cm ({direction})")

    return segments


def main():
    parser = argparse.ArgumentParser(description='Analyze motion patterns in PICO recorded data')
    parser.add_argument('--file', type=str, default='trackingData_whole_data.txt',
                        help='Data file path')
    parser.add_argument('--window', type=int, default=30,
                        help='Sliding window size (frames)')
    parser.add_argument('--min-movement', type=float, default=3.0,
                        help='Minimum position motion threshold (cm)')
    parser.add_argument('--min-rotation', type=float, default=10.0,
                        help='Minimum rotation threshold (degrees)')
    parser.add_argument('--rotation', action='store_true',
                        help='Analyze rotation motion')
    parser.add_argument('--all', action='store_true',
                        help='Analyze both position and rotation')
    args = parser.parse_args()

    # Determine file path
    data_file = Path(args.file)
    if not data_file.is_absolute():
        data_file = Path(__file__).parent / args.file

    if not data_file.exists():
        print(f'File not found: {data_file}')
        return

    # Tracker SN mapping
    tracker_map = {
        '190058': 'left_wrist',
        '190600': 'right_wrist',
        '190046': 'left_arm',
        '190023': 'right_arm',
    }

    print("=" * 70)
    print("PICO Recorded Data Motion Analysis")
    print("=" * 70)
    print(f"Data file: {data_file}")
    print(f"Window size: {args.window} frames")
    print(f"Min position motion threshold: {args.min_movement} cm")
    print(f"Min rotation threshold: {args.min_rotation} deg")
    print(f"Analysis mode: {'all' if args.all else ('rotation' if args.rotation else 'position')}")

    # Load data
    frames = load_data(str(data_file))
    print(f"Loaded {len(frames)} frames of data")

    # Extract positions and rotations
    poses = extract_tracker_poses(frames, tracker_map)

    # Position analysis
    do_position = not args.rotation or args.all
    do_rotation = args.rotation or args.all

    if do_position:
        # Extract positions (compatible with old interface)
        positions = {role: poses[role]['pos'] for role in poses}

        # Analyze each tracker's motion
        print("\n" + "=" * 70)
        print("Overall Motion Analysis per Tracker (PICO Coordinate System)")
        print("=" * 70)
        print("PICO coordinate system: X=left-right, Y=up-down, Z=front-back (toward user)")

        for role in ['head', 'left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
            if len(positions[role]) > 0:
                analyze_motion(positions[role], role, args.window)

        # Find arm tracker motion segments
        segments = find_arm_movement_segments(positions, args.min_movement, args.window)

        # Position test suggestions
        print("\n" + "=" * 70)
        print("Suggested Data Segments for Position Testing")
        print("=" * 70)
        print("\nThe following frame ranges can be used to test arm position motion:")

        for role in ['left_wrist', 'right_wrist']:
            pos = positions[role]
            if len(pos) > args.window:
                print(f"\n{role}:")
                for axis, axis_name, pico_dir in [(0, 'X', 'left-right'), (1, 'Y', 'up-down'), (2, 'Z', 'front-back')]:
                    max_range = 0
                    max_start = 0
                    for i in range(len(pos) - args.window):
                        segment = pos[i:i+args.window, axis]
                        seg_range = segment.max() - segment.min()
                        if seg_range > max_range:
                            max_range = seg_range
                            max_start = i
                    if max_range * 100 >= args.min_movement:
                        print(f"  {axis_name} ({pico_dir}): frames {max_start}-{max_start+args.window} (moved {max_range*100:.2f}cm)")

    if do_rotation:
        # Analyze rotation
        print("\n" + "=" * 70)
        print("Rotation Analysis per Tracker (PICO Coordinate System)")
        print("=" * 70)
        print("PICO coordinate system: X=Roll (left-right tilt), Y=Pitch (up-down tilt), Z=Yaw (horizontal rotation)")

        rotation_results = {}
        for role in ['left_wrist', 'right_wrist']:
            if len(poses[role]['quat']) > 0:
                rotation_results[role] = analyze_rotation(poses[role]['quat'], role, args.window)

        # Find rotation segments
        rotation_segments = find_rotation_segments(poses, args.min_rotation, args.window)

        # Rotation test suggestions
        print("\n" + "=" * 70)
        print("Suggested Data Segments for Rotation Testing")
        print("=" * 70)
        print("\nThe following frame ranges can be used to test arm rotation control:")

        for role in ['left_wrist', 'right_wrist']:
            if rotation_segments[role]:
                print(f"\n{role}:")
                # Group by primary rotation axis
                by_axis = {}
                for seg in rotation_segments[role]:
                    key = seg['main_axis']
                    if key not in by_axis:
                        by_axis[key] = []
                    by_axis[key].append(seg)

                for axis_key, segs in by_axis.items():
                    best = max(segs, key=lambda x: x['relative_rotation'])
                    sign = best['axis_sign']
                    print(f"  {sign}{axis_key}: frames {best['start']}-{best['end']} "
                          f"(rotation {best['relative_rotation']:.2f} deg)")

    print("\n" + "=" * 70)
    print("Tip: When testing with step5, add --start-frame and --end-frame arguments")
    print("=" * 70)


if __name__ == '__main__':
    main()
