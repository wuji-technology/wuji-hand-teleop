#!/usr/bin/env python3
"""
=============================================================================
Step 5: Incremental Control of Real Robot Using Recorded Data (Verify Complete Teleoperation Flow)
=============================================================================

Function: Read PICO recorded data -> coordinate transform -> compute incremental pose -> control real robot

Purpose:
  - Verify the complete PICO -> Robot teleoperation flow is correct
  - Use recorded data for repeatable testing
  - No PICO hardware required, only robot needed

⚠️ Prerequisites:
  - ✓ step4 test passed (coordinate transforms correct)
  - ✓ Robot is powered on and connected
  - ✓ Robot is in a safe position

💡 For individual rotation/position/arm angle testing, use step6_arm_angle_stability_test.py


=============================================================================
Coordinate Transform Quick Reference
=============================================================================

PICO -> Robot position mapping:
  Robot X = -PICO Z  (PICO reaches forward -> Robot forward)
  Robot Y = -PICO X  (PICO moves right -> Robot right)
  Robot Z = +PICO Y  (PICO raises hand up -> Robot up)

PICO -> Robot rotation axis mapping (ROS REP 103 right-hand rule, counterclockwise positive when looking from axis positive end toward origin):
  Around PICO +X -> around Robot -Y -> fingers press down (Pitch down)
  Around PICO -X -> around Robot +Y -> fingers tilt up (Pitch up)
  Around PICO Y -> around Robot +Z (fingers yaw right)
  Around PICO Z -> around Robot -X (wrist rolls right)


=============================================================================
Usage
=============================================================================

  Step 1: Dry run test (recommended first)
  --------------------------------
  cd ~/ros2_ws/src/wuji-hand-teleop/src/input_devices/pico_input/test

  # Dry run: only print poses, do not control robot
  python3 step5_incremental_control_with_robot.py --dry-run

  # Verify output poses are reasonable (should be near TIANJI_INIT_POS)


  Step 2: Move robot to initial position
  ------------------------------
  python3 tool/move_to_init_pose.py


  Step 3: Start incremental control
  --------------------
  # Use default static data (trackingData_sample_static.txt, 150 frames of static data)
  python3 step5_incremental_control_with_robot.py

  # Slower playback is safer
  python3 step5_incremental_control_with_robot.py --speed 0.3

  # Control left arm only
  python3 step5_incremental_control_with_robot.py --left-only

  # Control right arm only
  python3 step5_incremental_control_with_robot.py --right-only

  # Use large dataset
  python3 step5_incremental_control_with_robot.py --file ../record/trackingData_whole_data.txt

  # Specify frame range for testing
  python3 step5_incremental_control_with_robot.py --file ../record/trackingData_whole_data.txt --start-frame 100 --end-frame 200 --left-only --speed 0.5 -v


=============================================================================
Related Tools
=============================================================================

  Analyze motion segments in recorded data:
    cd ../record && python3 analyze_motion_data.py --file trackingData_whole_data.txt --all

  Test rotation/position/arm angle individually (recommended for debugging):
    python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only
    python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01


=============================================================================
Test data files (all in record/ directory)
=============================================================================

  High-quality static data (recommended for first test):
    ../record/trackingData_sample_static.txt (150 frames, avg displacement <0.1mm)

  Large datasets (contain noticeable motion):
    ../record/trackingData_whole_data.txt (5780 frames) - Recommended for verification testing
    ../record/trackingData_head_tracker_static.txt (8384 frames)
    ../record/trackingData_head_tracker.txt (5703 frames)

  Motion analysis tools:
    # Analyze position motion
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt

    # Analyze rotation motion
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt --rotation

    # Analyze both position and rotation
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt --all


=============================================================================
Tracker SN Mapping
=============================================================================

  190058 → pico_left_wrist  (Left wrist, controls left arm)
  190600 → pico_right_wrist (Right wrist, controls right arm)
  190046 → pico_left_arm    (Left forearm, arm angle constraint)
  190023 → pico_right_arm   (Right forearm, arm angle constraint)


=============================================================================
Incremental Control Principle
=============================================================================

  1. Initialization: record first frame PICO tracker pose
  2. Each frame:
     a. Compute PICO pose increment: Δ_pico = current - init
     b. Coordinate transform: Δ_world = PICO_TO_ROBOT @ Δ_pico
     c. Convert to chest coordinate system: Δ_chest = WORLD_TO_CHEST @ Δ_world
     d. Compute target pose: target = TIANJI_INIT_POS + Δ_chest
     e. Send to robot


=============================================================================
Differences from pico_teleop.launch.py
=============================================================================

This script and launch use the same incremental control algorithm, but with the following differences:

1. Arm angle control:
   - step5 (this script): Static default arm angle. Uses DEFAULT_ZSP_DIRECTION fixed values,
     does not use arm tracker data. Elbow always maintains default relaxed posture.
     Suitable for verifying basic pose transforms are correct.
   - launch: Real-time dynamic arm angle. pico_input_node computes from arm tracker position
     elbow offset direction, published to tianji_world_output after deadzone debouncing + EMA smoothing.
     Robot elbow follows the human's actual elbow position.

2. Position smoothing:
   - step5 (this script): No smoothing, directly uses raw computed values.
   - launch: pico_input_node applies EMA smoothing to position (alpha=0.6),
     reduces tracker jitter, smoother motion but slightly slower response.

3. Architecture:
   - step5 (this script): Directly calls CartesianController, no intermediate nodes.
   - launch: pico_input → ROS2 topic → tianji_world_output → robot.


=============================================================================
Safety Notes
=============================================================================

⚠️ For first run:
  1. First use --dry-run to verify output poses
  2. Ensure no people or obstacles around the robot
  3. Have the emergency stop button ready
  4. Use slow playback (--speed 0.3) to observe robot motion

⚠️ If robot motion is abnormal:
  - Immediately press Ctrl+C to stop
  - Check if coordinate transforms are correct
  - Go back to step4 to verify visualization


=============================================================================
Command Line Parameters
=============================================================================

  --file FILE         Data file path (default: ../record/trackingData_sample_static.txt)
  --speed SPEED       Playback speed multiplier (default: 1.0)
  --dry-run           Dry run mode, print only without control
  --left-only         Control left arm only
  --right-only        Control right arm only
  --start-frame N     Start frame index (from 0)
  --end-frame N       End frame index
  -v, --verbose       Show detailed transform log (PICO increment -> Chest increment)


=============================================================================
Next Step
=============================================================================

After passing tests, you can:
  1. Use real PICO device for real-time control
  2. Record more test data to build test datasets
  3. Proceed to step6 arm angle stability test

=============================================================================
"""

import sys
import time
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import shared modules (using relative path)
_test_dir = str(Path(__file__).resolve().parent)
_pico_input_dir = str(Path(__file__).resolve().parent.parent)
_src_dir = str(Path(__file__).resolve().parent.parent.parent.parent)
_tianji_wo_dir = str(Path(_src_dir) / 'output_devices' / 'tianji_world_output')
for _p in [_test_dir, _pico_input_dir, _tianji_wo_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS, TIANJI_INIT_ROT,
    WORLD_TO_LEFT_QUAT, WORLD_TO_RIGHT_QUAT,
    ROBOT_IP,
    DEFAULT_ZSP_DIRECTION, ARM_INIT_POS,
)
from common.transform_utils import (
    get_pico_to_robot,
    transform_pico_rotation_to_world,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
)

# Import data source and robot control
from pico_input.data_source import RecordedDataSource
from tianji_world_output.cartesian_controller import CartesianController


# PICO -> Robot coordinate transform (loaded from shared config)
# See tianji_world_output/transform_utils.py and pico_input/ARCHITECTURE.md §3.1

# World -> Chest conversion: using shared functions from transform_utils
# (transform_world_to_chest, apply_world_rotation_to_chest_pose etc.)

# Tracker SN -> role mapping. Replace with your own SNs.
# Production pico_input_node reads `tracker_serial_<sn>` keys from
# pico_input.yaml dynamically (see fix 3b5fa8d); this dev tool inlines
# the map to stay runnable as a plain script.
TRACKER_SN_MAP = {
    '190058': 'pico_left_wrist',
    '190600': 'pico_right_wrist',
    '190046': 'pico_left_arm',
    '190023': 'pico_right_arm',
}


def compute_elbow_direction(shoulder: np.ndarray, wrist: np.ndarray,
                            elbow: np.ndarray, side: str) -> np.ndarray:
    """
    Compute elbow offset direction relative to shoulder-wrist plane

    ============================================================================
    Arm Angle Calculation Principle
    ============================================================================

    This is the core of nullspace IK constraint: specifying the desired elbow offset direction in space.

    Geometric principle:
                Shoulder
                  ●
                 /|\\
                / | \\
               /  |  \\
         shoulder-elbow /   |   \\ shoulder-wrist
              /    |    \\
             ●─────●─────●
          Elbow   Projection   Wrist

            ←────→
          Elbow offset vector (direction)

    Computation steps:
    1. Shoulder-wrist vector: straight line from shoulder to wrist (arm main axis)
    2. Shoulder-elbow vector: from shoulder to elbow (forearm tracker position)
    3. Projection: project elbow onto shoulder-wrist line
    4. Offset vector: actual elbow position - projection point = direction of elbow deviation from main axis
    5. Normalize: get unit direction vector

    ============================================================================
    Relaxed elbow direction description (Chest coordinate system)
    ============================================================================

    Chest coordinate system definition:
      Left Chest:  +Y = -Z_world (downward), +Z = +Y_world (toward left)
      Right Chest: +Y = +Z_world (upward), +Z = -Y_world (toward right)

    Default arm angle reference plane parameters (consistent with cartesian_controller):
    - left arm: [0, -1, -0.5]  → Y-(anti-gravity) + Z-(toward outer side/right)
    - right arm: [0, +1, -0.5]  → Y+(anti-gravity) + Z-(toward outer side/left)

    Note: differences between left and right arms are handled in the arm tracker initial position (robot_init_pos),
    so the direction computed here can be used directly.

    Args:
        shoulder: Shoulder position (chest coordinate system origin, [0,0,0])
        wrist: Wrist position (converted from PICO wrist tracker)
        elbow: Elbow position (converted from PICO arm tracker)
        side: 'left' or 'right'

    Returns:
        direction: Unit direction vector [x, y, z]
    """
    # Default arm angle reference plane parameters (loaded from unified config)
    default_direction = DEFAULT_ZSP_DIRECTION[side]

    # Shoulder-wrist vector
    shoulder_to_wrist = wrist - shoulder
    sw_length = np.linalg.norm(shoulder_to_wrist)

    if sw_length < 1e-6:
        # Wrist too close to shoulder, using default relaxed elbow direction
        return default_direction

    sw_unit = shoulder_to_wrist / sw_length

    # Shoulder-elbow vector
    shoulder_to_elbow = elbow - shoulder

    # Projection length of elbow on shoulder-wrist line
    proj_length = np.dot(shoulder_to_elbow, sw_unit)

    # Projection point
    proj_point = shoulder + proj_length * sw_unit

    # Elbow offset vector
    elbow_offset = elbow - proj_point
    offset_length = np.linalg.norm(elbow_offset)

    if offset_length < 1e-6:
        # Elbow is exactly on shoulder-wrist line, using default relaxed elbow direction
        return default_direction

    # Normalize to get direction vector
    direction = elbow_offset / offset_length

    return direction


def compute_incremental_pose(current_pose, init_pose, robot_init_pos, robot_init_rot, is_left: bool):
    """
    Compute incremental pose (same algorithm as IncrementalController.compute_target_pose)

    Uses the same 4-step algorithm as step1/step2, ensuring rotation transforms are correct.

    Args:
        current_pose: [x, y, z, qx, qy, qz, qw] Current pose (PICO coordinate system)
        init_pose: [x, y, z, qx, qy, qz, qw] Initial pose (PICO coordinate system)
        robot_init_pos: Robot initial position (chest coordinate system)
        robot_init_rot: Robot initial orientation (Rotation object, chest coordinate system)
        is_left: Whether it is the left arm

    Returns:
        (target_pos, target_quat) Target pose (chest coordinate system)
    """
    side = 'left' if is_left else 'right'

    # Position increment
    delta_pos_pico = np.array(current_pose[:3]) - np.array(init_pose[:3])
    delta_pos_world = get_pico_to_robot() @ delta_pos_pico

    # Convert to chest coordinate system (using shared library)
    delta_pos_chest = transform_world_to_chest(delta_pos_world, side)

    target_pos = robot_init_pos + delta_pos_chest

    # Orientation increment
    init_rot = R.from_quat(init_pose[3:7])
    current_rot = R.from_quat(current_pose[3:7])
    delta_rot_pico = current_rot * init_rot.inv()

    # Using shared library: PICO->World rotation transform + 4-step algorithm
    delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())
    robot_init_rot_mat = robot_init_rot.as_matrix()
    target_rot_in_chest = apply_world_rotation_to_chest_pose(
        robot_init_rot_mat, delta_rot_world, side
    )

    target_rot = R.from_matrix(target_rot_in_chest)
    target_quat = target_rot.as_quat()

    return target_pos, target_quat


def main():
    parser = argparse.ArgumentParser(description='Step 5: Incremental control of real robot using recorded data')
    parser.add_argument('--file', type=str,
                        default='../record/trackingData_sample_static.txt',
                        help='Recorded data file path (relative to test/ directory)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (0.5=slow, 1.0=real-time, 2.0=fast)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Dry run mode (print only, do not control robot)')
    parser.add_argument('--left-only', action='store_true',
                        help='Control left arm only')
    parser.add_argument('--right-only', action='store_true',
                        help='Control right arm only')
    parser.add_argument('--start-frame', type=int, default=0,
                        help='Start frame index (from 0)')
    parser.add_argument('--end-frame', type=int, default=None,
                        help='End frame index (plays to end if not specified)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed transform log (PICO increment -> robot increment)')
    args = parser.parse_args()

    if args.speed <= 0:
        parser.error("--speed must be greater than 0")

    # Determine data file path
    data_file = Path(args.file)
    if not data_file.is_absolute():
        # Relative path, search from pico_input/test directory
        test_dir = Path(__file__).parent
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f"Error: data file not found {data_file}")
        return

    print("=" * 70)
    print("Step 5: Incremental control of real robot using recorded data")
    print("=" * 70)
    print(f"  Data file: {data_file}")
    print(f"  Playback speed: {args.speed}x")
    print(f"  Dry run mode: {args.dry_run}")
    print(f"  Frame range: {args.start_frame} - {args.end_frame if args.end_frame else 'end'}")
    print(f"  Control mode: ", end="")
    if args.left_only:
        print("Left arm only")
    elif args.right_only:
        print("Right arm only")
    else:
        print("Dual arms")
    print("=" * 70)

    # Create data source
    print("\n[1/4] Loading recorded data...")
    data_source = RecordedDataSource(
        str(data_file),
        playback_speed=args.speed,
        loop=False  # step5 does not loop, plays once
    )

    if not data_source.initialize():
        print("Error: Data source initialization failed")
        return

    print(f"Data loaded successfully, total {len(data_source.data_frames)} frames")

    # Connect to robot(if not dry run)
    controller = None
    if not args.dry_run:
        print("\n[2/4] Connect to robot...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"Connected to robot {ROBOT_IP}")

            # Enable robot (critical step!)
            print("  Setting impedance mode...")
            controller.set_impedance_mode(mode='joint')
            print("  Robot enabled")

            # Move to initial pose
            print("  Moving to initial pose...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ Reached initial pose")

        except Exception as e:
            print(f"Connection failed: {e}")
            if controller is not None:
                controller.disable_and_release()
            return
    else:
        print("\n[2/4] Skipping robot connection (dry run mode)")

    # Initialize: skip to start frame and record initial poses
    print("\n[3/4] Initializing (recording initial poses)...")
    time.sleep(0.1)

    # Jump directly to start frame (directly set data source internal state, bypassing time sync)
    if args.start_frame > 0:
        if args.start_frame >= len(data_source.data_frames):
            print(f"Error: start frame {args.start_frame} exceeds data range (total {len(data_source.data_frames)} frames)")
            return
        print(f"  Jumping to frame {args.start_frame}...")
        # Directly set start index and timestamp baseline
        data_source.current_index = args.start_frame
        data_source.first_timestamp = data_source.data_frames[args.start_frame].get('predictTime', 0)
        data_source.start_time = time.time()

    tracker_data_list = data_source.get_tracker_data()
    if not tracker_data_list:
        print("Error: unable to get tracker data")
        return

    # Record initial poses
    init_poses = {}
    import re

    for tracker in tracker_data_list:
        # Extract 6-digit SN before the trailing 'G' (see fix 3b5fa8d).
        match = re.search(r'(\d{6})G?$', tracker.serial_number)
        if not match:
            continue

        short_sn = match.group(1)
        role = TRACKER_SN_MAP.get(short_sn)

        if role and tracker.is_valid:
            init_poses[role] = tracker.to_pose_array()
            print(f"  {role}: pos={tracker.position}, valid={tracker.is_valid}")

    if not init_poses:
        print("Error: no valid tracker data found")
        return

    print(f"Recorded {len(init_poses)} trackers initialpose")

    # Main loop: playback and control
    print("\n[4/4] Starting playback and robot control...")
    print("  Press Ctrl+C to stop\n")

    frame_count = 0
    start_time = time.time()

    # Compute end frame index
    end_index = args.end_frame if args.end_frame is not None else len(data_source.data_frames) - 1
    end_index = min(end_index, len(data_source.data_frames) - 1)

    # Helper function: parse pose string
    def parse_pose_string(pose_str):
        values = [float(x.strip()) for x in pose_str.split(',')]
        return np.array(values[:7], dtype=np.float64)

    try:
        # Iterate frames by index, not relying on time synchronization
        for frame_idx in range(args.start_frame + 1, end_index + 1):
            frame = data_source.data_frames[frame_idx]
            frame_count += 1
            actual_frame = frame_idx

            # Parse current frame tracker data
            motion = frame.get('Motion', {})
            joints = motion.get('joints', [])

            # First pass: collect all tracker poses
            current_poses = {}  # {role: (pos, quat)}
            for joint in joints:
                sn = joint.get('sn', '')
                pose_str = joint.get('p', '')
                if not pose_str:
                    continue

                match = re.search(r'(\d{6})G?$', sn)
                if not match:
                    continue

                short_sn = match.group(1)
                role = TRACKER_SN_MAP.get(short_sn)

                if role and role in init_poses:
                    is_left = 'left' in role
                    side = 'left' if is_left else 'right'

                    # Parse current pose
                    current_pose = parse_pose_string(pose_str)

                    # Compute PICO increment (for verbose logging)
                    pico_delta = current_pose[:3] - init_poses[role][:3]

                    # Key: arm tracker and wrist tracker use different initial positions
                    if 'arm' in role and 'wrist' not in role:
                        # Forearm tracker: load initial position from config
                        robot_init_pos = ARM_INIT_POS[side]
                    else:
                        robot_init_pos = TIANJI_INIT_POS[side]

                    robot_init_rot = R.from_matrix(TIANJI_INIT_ROT[side])

                    target_pos, target_quat = compute_incremental_pose(
                        current_pose,
                        init_poses[role],
                        robot_init_pos,
                        robot_init_rot,
                        is_left
                    )
                    # Compute robot increment (relative to initial position)
                    robot_delta = target_pos - robot_init_pos
                    current_poses[role] = (target_pos, target_quat, pico_delta, robot_delta)

            # Second pass: compute elbow direction and control robot
            for side in ['left', 'right']:
                wrist_role = f'pico_{side}_wrist'
                arm_role = f'pico_{side}_arm'

                if wrist_role not in current_poses:
                    continue

                # Check if this arm needs to be controlled
                is_left = (side == 'left')
                if args.left_only and not is_left:
                    continue
                if args.right_only and is_left:
                    continue

                wrist_pos, wrist_quat, pico_delta, robot_delta = current_poses[wrist_role]
                shoulder_pos = np.array([0.0, 0.0, 0.0])

                # Using configuration default zsp_para direction (verified through FK->IK closed-loop)
                # Note: geometric direction (compute_elbow_direction) and IK's zsp_para convention differ
                # Geometric direction Y component points toward gravity, zsp_para Y component needs anti-gravity direction
                elbow_direction = DEFAULT_ZSP_DIRECTION[side]

                # Print information
                if args.verbose:
                    # Verbose mode: show PICO increment -> robot increment
                    # PICO coordinate system: +X=right, +Y=up, +Z=toward user (back)
                    # Note: PICO +Z is "toward user" direction, i.e. the direction of user's hand moving backward!
                    #       When user reaches forward, PICO Z is negative!
                    # Chest coordinate system:
                    #   Left Chest: X=Forward, Y=Down, Z=Left
                    #   Right Chest: X=Forward, Y=Up, Z=Right
                    if side == 'left':
                        y_label = "Down+"
                        z_label = "Left+"
                    else:
                        y_label = "Up+"
                        z_label = "Right+"
                    print(f"[frames {actual_frame:5d}] {wrist_role:15s}:")
                    print(f"    PICO Δ: X={pico_delta[0]*100:+6.2f}cm(Right+), Y={pico_delta[1]*100:+6.2f}cm(Up+), Z={pico_delta[2]*100:+6.2f}cm(Backward+/toward user)")
                    print(f"    Chest Δ: X={robot_delta[0]*100:+6.2f}cm(Forward+), Y={robot_delta[1]*100:+6.2f}cm({y_label}), Z={robot_delta[2]*100:+6.2f}cm({z_label})")
                else:
                    print(f"[frames {actual_frame:5d}] {wrist_role:15s}: "
                          f"pos=[{wrist_pos[0]:.3f}, {wrist_pos[1]:.3f}, {wrist_pos[2]:.3f}] "
                          f"elbow_dir=[{elbow_direction[0]:.2f}, {elbow_direction[1]:.2f}, {elbow_direction[2]:.2f}]")

                # Send to robot
                if controller:
                    try:
                        # Update elbow direction (zsp_para)
                        zsp_para = [
                            elbow_direction[0],
                            elbow_direction[1],
                            elbow_direction[2],
                            0, 0, 0
                        ]
                        if side == 'left':
                            controller.left_zsp_para = zsp_para
                        else:
                            controller.right_zsp_para = zsp_para

                        # Build pose matrix (position in meters, move_to_pose_direct will convert internally)
                        rot_mat = R.from_quat(wrist_quat).as_matrix()
                        pose_mat = np.eye(4)
                        pose_mat[:3, :3] = rot_mat
                        pose_mat[:3, 3] = wrist_pos  # meters, not millimeters

                        if side == 'left':
                            controller.move_to_pose_direct(
                                left_pose=pose_mat,
                                right_pose=None,
                                unit='matrix'
                            )
                        else:
                            controller.move_to_pose_direct(
                                left_pose=None,
                                right_pose=pose_mat,
                                unit='matrix'
                            )
                    except Exception as e:
                        print(f"  ⚠️ Control failed: {e}")

            # Control playback speed
            time.sleep(0.01 / args.speed)

    except KeyboardInterrupt:
        print("\n\nUser interrupted")

    # Statistics
    elapsed = time.time() - start_time
    print("\n" + "=" * 70)
    print(f"Playback complete")
    print(f"  Total frames: {frame_count}")
    print(f"  Elapsed: {elapsed:.1f} seconds")
    print(f"  Average frame rate: {frame_count/elapsed:.1f} fps")
    print("=" * 70)

    # Cleanup
    data_source.close()
    if controller:
        print("Disabling and releasing robot...")
        controller.disable_and_release()
        print("Robot safely exited")


if __name__ == '__main__':
    main()
