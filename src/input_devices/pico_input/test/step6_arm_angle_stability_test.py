#!/usr/bin/env python3
"""
Step 6: Robot Stability Test (Simulated Tracking Data)

================================================================================
Feature Overview
================================================================================

Three test modes:
  1. arm_angle: Arm angle test - fixed end-effector position, test different elbow directions
  2. position:  Position test - fixed arm angle, test end-effector forward/backward/left/right/up/down movement
  3. rotation:  Rotation test - fixed position, test end-effector rotation around each axis

================================================================================
Test Mode Descriptions
================================================================================

[Mode 1: Arm angle test (arm_angle)]
  - End-effector pose fixed at initial position
  - Change zsp_para (elbow direction)
  - Test IK stability under different arm angles

  Parameters:
    --pitch-range MIN MAX STEP  Pitch angle range (elbow forward/backward offset)
    --yaw-range MIN MAX STEP    Abduction angle range (elbow down/outward ratio)

  Angle definitions:
    Pitch: Positive=elbow forward, negative=elbow backward, 0=no front/back offset
    Yaw:   0 deg=purely downward, 45°=Default relaxed elbow, 90°=purely outward

[Mode 2: Position test (position)]
  - Arm angle fixed to default relaxed elbow (yaw=45°)
  - End-effector position offset from initial position
  - Test IK stability at different end-effector positions
  - Position offset uses Robot World coordinate system (unified interface for both arms, internally auto-converted to Chest)

  Parameters:
    --pos-x-range MIN MAX STEP  X-direction (front/back) offset range (meters)
    --pos-y-range MIN MAX STEP  Y-direction (left/right) offset range (meters)
    --pos-z-range MIN MAX STEP  Z-direction (up/down) offset range (meters)

[Mode 3: Rotation test (rotation)]
  - Position fixed at initial position
  - End-effector orientation rotated from initial orientation
  - For verifying correctness of rotation transforms

  Parameters:
    --rot-axis X/Y/Z/-X/-Y/-Z  Rotation axis (Robot World coordinate system)
    --rot-angle DEGREES        Rotation angle (degrees)

  Robot World coordinate system rotation effects (ROS REP 103 right-hand rule, counterclockwise positive when looking from axis positive end toward origin):
    Around +X axis: Wrist rolls right (Roll right: top tilts right)
    Around -X axis: Wrist rolls left (Roll left: top tilts left)
    Around +Y axis: Fingers press down (Pitch down: look down)
    Around -Y axis: Fingers tilt up (Pitch up: look up)
    Around +Z axis: Fingers yaw left (Yaw left: counterclockwise from above)
    Around -Z axis: Fingers yaw right (Yaw right: clockwise from above)

================================================================================
Usage Examples
================================================================================

[Arm Angle Test]
  # Dry run (print only)
  python3 step6_arm_angle_stability_test.py --dry-run

  # Test left arm, default range
  python3 step6_arm_angle_stability_test.py --left-only

  # Large range arm angle test
  python3 step6_arm_angle_stability_test.py --pitch-range -30 30 10 --yaw-range 10 80 10

[Position Test]
  # Forward/backward movement test (+-5cm, step 1cm)
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01

  # Left/right movement test
  python3 step6_arm_angle_stability_test.py --mode position --pos-y-range -0.05 0.05 0.01

  # Up/down movement test
  python3 step6_arm_angle_stability_test.py --mode position --pos-z-range -0.05 0.05 0.01

  # Combined position test (forward/backward + up/down)
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.03 0.03 0.01 --pos-z-range -0.03 0.03 0.01

[Rotation Test]
  # Around +X axis rotation 30 deg (Wrist rolls right)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only

  # Around -Y axis rotation 20 deg (Fingers tilt up) - Note: use = syntax for negative axis
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Y --rot-angle 20 --left-only

  # Around +Z axis rotation 30 deg (Fingers yaw left)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis Z --rot-angle 30 --left-only

  # Around -Z axis rotation 30 deg (Fingers yaw right) - Note: use = syntax for negative axis
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Z --rot-angle 30 --left-only

[Common Parameters]
  --dry-run       Print only, do not control robot
  --left-only     Test left arm only
  --right-only    Test right arm only
  --hold-time N   Hold time per test point (seconds)
"""

import argparse
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

# Add common directory to path
sys.path.insert(0, str(Path(__file__).parent))

from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    ROBOT_IP,
)

# Import coordinate transform functions (unified implementation, consistent with step2/4/5)
from common.transform_utils import (
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
    elbow_direction_from_angles,
)

# import CartesianController
try:
    from tianji_world_output.cartesian_controller import CartesianController
except ImportError:
    print("Warning: Cannot import CartesianController, only dry-run mode supported")
    CartesianController = None


def generate_test_cases(pitch_range: tuple, yaw_range: tuple) -> list:
    """
    Generate test cases

    Args:
        pitch_range: (min, max, step) pitch anglerange
        yaw_range: (min, max, step) Abduction angle range

    Returns:
        test_cases: [(name, pitch, yaw), ...]
    """
    test_cases = []

    pitch_min, pitch_max, pitch_step = pitch_range
    yaw_min, yaw_max, yaw_step = yaw_range

    # First test default arm angle
    test_cases.append(("Default relaxed elbow (0 deg, 45 deg)", 0, 45))

    # Test different abduction angles (fixed pitch=0)
    for yaw in range(yaw_min, yaw_max + 1, yaw_step):
        if yaw != 45:  # Skip already added default value
            test_cases.append((f"Abduction{yaw}°", 0, yaw))

    # Test different pitch angles (fixed yaw=45)
    for pitch in range(pitch_min, pitch_max + 1, pitch_step):
        if pitch != 0:  # Skip already added default value
            name = f"Forward tilt{pitch}°" if pitch > 0 else f"Backward tilt{-pitch}°"
            test_cases.append((name, pitch, 45))

    # Combined test (optional)
    # for pitch in range(pitch_min, pitch_max + 1, pitch_step):
    #     for yaw in range(yaw_min, yaw_max + 1, yaw_step):
    #         if pitch != 0 and yaw != 45:
    #             test_cases.append((f"pitch={pitch}°,yaw={yaw}°", pitch, yaw))

    return test_cases


def generate_position_test_cases(x_range: tuple, y_range: tuple, z_range: tuple) -> list:
    """
    Generate position test cases (World coordinate system)

    Position increment defined in Robot World coordinate system (same for both arms):
      X = Forward(+) / Backward(-)
      Y = Left(+) / Right(-)
      Z = Up(+) / Down(-)

    Internally converted to Chest coordinate system via transform_world_to_chest() before applying.

    Args:
        x_range: (min, max, step) X-direction (front/back) offset range (meters), None means do not test
        y_range: (min, max, step) Y-direction (left/right) offset range (meters), None means do not test
        z_range: (min, max, step) Z-direction (up/down) offset range (meters), None means do not test

    Returns:
        test_cases: [(name, dx, dy, dz), ...]  dx/dy/dz is World coordinate systemincrement
    """
    test_cases = []

    # First add origin (initial position)
    test_cases.append(("Initial position", 0.0, 0.0, 0.0))

    # X-direction test (forward/backward)
    if x_range is not None:
        x_min, x_max, x_step = x_range
        x_values = np.arange(x_min, x_max + x_step * 0.5, x_step)
        for dx in x_values:
            if abs(dx) > 1e-6:  # Skip zero point
                name = f"Forward{dx*100:.1f}cm" if dx > 0 else f"Backward{-dx*100:.1f}cm"
                test_cases.append((name, float(dx), 0.0, 0.0))

    # Y-direction test (left/right)- World Y+ = Left, Y- = Right
    if y_range is not None:
        y_min, y_max, y_step = y_range
        y_values = np.arange(y_min, y_max + y_step * 0.5, y_step)
        for dy in y_values:
            if abs(dy) > 1e-6:  # Skip zero point
                name = f"Left{dy*100:.1f}cm" if dy > 0 else f"Right{-dy*100:.1f}cm"
                test_cases.append((name, 0.0, float(dy), 0.0))

    # Z-direction test (up/down)- World Z+ = Up, Z- = Down
    if z_range is not None:
        z_min, z_max, z_step = z_range
        z_values = np.arange(z_min, z_max + z_step * 0.5, z_step)
        for dz in z_values:
            if abs(dz) > 1e-6:  # Skip zero point
                name = f"Up{dz*100:.1f}cm" if dz > 0 else f"Down{-dz*100:.1f}cm"
                test_cases.append((name, 0.0, 0.0, float(dz)))

    return test_cases


def print_test_header():
    """Print test header"""
    print("\n" + "=" * 90)
    print(f"{'Test Name':<20} {'Pitch':>8} {'Yaw':>8} {'Direction':<25} {'IK Result':<10}")
    print("=" * 90)


def print_test_result(name: str, pitch: float, yaw: float, direction: np.ndarray,
                      ik_success: bool, side: str):
    """Print single test result"""
    dir_str = f"[{direction[0]:6.3f}, {direction[1]:6.3f}, {direction[2]:6.3f}]"
    status = "✓ Success" if ik_success else "✗ Failed"
    print(f"{name:<20} {pitch:>8.1f} {yaw:>8.1f} {dir_str:<25} {status:<10} ({side})")


def print_position_test_header():
    """Print position test header"""
    print("\n" + "=" * 90)
    print(f"{'Test Name':<20} {'ΔX(cm)':>10} {'ΔY(cm)':>10} {'ΔZ(cm)':>10} {'IK Result':<10}")
    print("=" * 90)


def print_position_test_result(name: str, dx: float, dy: float, dz: float,
                                ik_success: bool, side: str):
    """Print single position test result"""
    status = "✓ Success" if ik_success else "✗ Failed"
    print(f"{name:<20} {dx*100:>10.1f} {dy*100:>10.1f} {dz*100:>10.1f} {status:<10} ({side})")


def parse_rotation_axis(axis_str: str) -> tuple:
    """
    Parse rotation axis parameter

    Args:
        axis_str: Axis name, e.g. 'X', '-X', 'Y', '-Y', 'Z', '-Z'

    Returns:
        (axis_vector, axis_name, expected_effect): Axis vector, axis name, expected effect description

    Note:
        Rotation effects are based on ROS REP 103 right-hand rule:
        - Thumb points in axis positive direction
        - Fingers curl in positive rotation direction (counterclockwise positive when looking from axis positive end toward origin)

        In robot coordinate system (verified on real robot):
        - +X rotation: Wrist rolls right (Roll right)   - Rx(theta): top tilts right
        - +Y rotation: Fingers press down (Pitch down)      - Ry(theta): front end tilts down
        - +Z rotation: Fingers yaw left (Yaw left)        - Rz(theta): front end yaws left
    """
    axis_map = {
        'X':  (np.array([1, 0, 0]), "+X", "Wrist rolls right (Roll right)"),
        '-X': (np.array([-1, 0, 0]), "-X", "Wrist rolls left (Roll left)"),
        'Y':  (np.array([0, 1, 0]), "+Y", "Fingers press down (Pitch down)"),
        '-Y': (np.array([0, -1, 0]), "-Y", "Fingers tilt up (Pitch up)"),
        'Z':  (np.array([0, 0, 1]), "+Z", "Fingers yaw left (Yaw left)"),
        '-Z': (np.array([0, 0, -1]), "-Z", "Fingers yaw right (Yaw right)"),
    }

    # Normalize input
    axis_upper = axis_str.upper().strip()
    if axis_upper.startswith('+'):
        axis_upper = axis_upper[1:]

    if axis_upper not in axis_map:
        raise ValueError(f"Invalid rotation axis: {axis_str}. Supported: X, -X, Y, -Y, Z, -Z")

    return axis_map[axis_upper]


def run_rotation_test(args):
    """Run rotation test"""

    print("\n" + "=" * 70)
    print("Step 6: Rotation Test")
    print("=" * 70)

    # Parse rotation parameters
    try:
        axis_vec, axis_name, expected_effect = parse_rotation_axis(args.rot_axis)
    except ValueError as e:
        print(f"\nError: {e}")
        return

    angle_deg = args.rot_angle

    print(f"\nParameters:")
    print(f"  Rotation axis (Robot World): {axis_name}")
    print(f"  Rotation angle: {angle_deg}°")
    print(f"  Expected effect: {expected_effect}")
    print(f"  Hold time: {args.hold_time}s")
    print(f"  Mode: {'Dry run' if args.dry_run else 'Real robot control'}")

    print("\nRotation effect reference table (Robot World coordinate system - ROS REP 103 right-hand rule, CCW positive from axis positive end):")
    print("  +----------------+-------------------------------------------------+")
    print("  | Rotation axis  | Real robot effect (thumb on axis, fingers curl for positive rotation)        |")
    print("  +----------------+-------------------------------------------------+")
    print("  | Around +X axis | Wrist rolls right (Roll right: top tilts right)        |")
    print("  | Around -X axis | Wrist rolls left (Roll left: top tilts left)         |")
    print("  | Around +Y axis | Fingers press down (Pitch down: look down)                |")
    print("  | Around -Y axis | Fingers tilt up (Pitch up: look up)                  |")
    print("  | Around +Z axis | Fingers yaw left (Yaw left: CCW from above)          |")
    print("  | Around -Z axis | Fingers yaw right (Yaw right: CW from above)         |")
    print("  +----------------+-------------------------------------------------+")

    # Determine which arms to test
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"\n  Test arms: {', '.join(sides)}")

    # Connect to robot
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\nError: CartesianController not available, please use --dry-run mode")
            return

        print("\n[1/4] Connect to robot...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  Connected to {ROBOT_IP}")

            print("  Setting impedance mode...")
            controller.set_impedance_mode(mode='joint')
            print("  Robot enabled")

            print("  Moving to initial pose...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ Reached initial pose")

        except Exception as e:
            print(f"  ❌ Connection failed: {e}")
            return
    else:
        print("\n[1/4] Skipping robot connection (dry run mode)")

    # Compute rotation
    print("\n[2/4] Computing rotation...")

    # Rotate in Robot World coordinate system
    rotvec_world = axis_vec * np.radians(angle_deg)
    R_delta_world = R.from_rotvec(rotvec_world)

    print(f"  Robot World rotation vector: [{rotvec_world[0]:.4f}, {rotvec_world[1]:.4f}, {rotvec_world[2]:.4f}] rad")

    # Using fixed default relaxed elbow arm angle
    default_pitch, default_yaw = 0, 45

    # Run test
    print("\n[3/4] Executing rotation...")

    for side in sides:
        print(f"\n  [{side.upper()} ARM]")

        # Base pose (initial pose, Chest coordinate system)
        base_pos = TIANJI_INIT_POS[side].copy()
        base_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        # Set arm angle
        direction = elbow_direction_from_angles(default_pitch, default_yaw, side)
        zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

        # Using correct rotation transform (4-step algorithm consistent with step2)
        target_rot_matrix = apply_world_rotation_to_chest_pose(
            base_rot.as_matrix(), R_delta_world, side
        )
        target_rot = R.from_matrix(target_rot_matrix)

        # Build pose matrix
        pose_mat = np.eye(4)
        pose_mat[:3, :3] = target_rot.as_matrix()
        pose_mat[:3, 3] = base_pos  # Position unchanged

        # Print debug information
        print(f"    Base orientation (Chest):")
        print(f"      Position: [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}] m")
        euler_base = base_rot.as_euler('xyz', degrees=True)
        print(f"      Orientation: [{euler_base[0]:.2f}, {euler_base[1]:.2f}, {euler_base[2]:.2f}]° (XYZ euler)")

        euler_target = target_rot.as_euler('xyz', degrees=True)
        print(f"    Target orientation (Chest):")
        print(f"      Position: [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}] m (unchanged)")
        print(f"      Orientation: [{euler_target[0]:.2f}, {euler_target[1]:.2f}, {euler_target[2]:.2f}]° (XYZ euler)")

        if controller:
            try:
                # Set arm angle
                if side == 'left':
                    controller.left_zsp_para = zsp_para
                else:
                    controller.right_zsp_para = zsp_para

                # Send control command
                if side == 'left':
                    left_ok, _, left_joints, _ = controller.move_to_pose_direct(
                        left_pose=pose_mat,
                        right_pose=None,
                        unit='matrix'
                    )
                    ik_success = left_ok
                else:
                    _, right_ok, _, right_joints = controller.move_to_pose_direct(
                        left_pose=None,
                        right_pose=pose_mat,
                        unit='matrix'
                    )
                    ik_success = right_ok

                if ik_success:
                    print(f"    ✓ IK succeeded, rotating...")
                else:
                    print(f"    ✗ IK failed!")

                # Hold for a period
                print(f"    Holding for {args.hold_time}s...")
                time.sleep(args.hold_time)

            except Exception as e:
                print(f"    ⚠️ Control exception: {e}")
        else:
            print(f"    (Dry run mode, skipping actual control)")

    # Return to initial pose
    if controller:
        print("\n[4/4] Returning to initial pose...")
        controller.move_to_init(
            wait=True, timeout=2,
            init_joints_left=INIT_JOINTS['left'],
            init_joints_right=INIT_JOINTS['right']
        )
        print("  Returned to initial pose")

        print("\nDisabling and releasing robot...")
        controller.disable_and_release()
        print("Robot safely exited")
    else:
        print("\n[4/4] Skipping return to initial pose (dry run mode)")

    print("\n" + "=" * 70)
    print(f"Rotation test complete: around {axis_name} axis rotation {angle_deg}°")
    print(f"Expected effect: {expected_effect}")
    print("Please observe if real robot effect matches expectations")
    print("=" * 70)


def run_stability_test(args):
    """Run arm angle stability test"""

    print("\n" + "=" * 70)
    print("Step 6: Arm Angle Stability Test")
    print("=" * 70)

    # Parse angle range
    pitch_range = tuple(args.pitch_range)
    yaw_range = tuple(args.yaw_range)

    print(f"\nParameters:")
    print(f"  Pitch range: {pitch_range[0]}° ~ {pitch_range[1]}°, step {pitch_range[2]}°")
    print(f"  Yaw range:   {yaw_range[0]}° ~ {yaw_range[1]}°, step {yaw_range[2]}°")
    print(f"  Hold time:   {args.hold_time}s")
    print(f"  Mode:       {'Dry run' if args.dry_run else 'Real robot control'}")

    # Generate test cases
    test_cases = generate_test_cases(pitch_range, yaw_range)
    print(f"  Test cases:  {len(test_cases)}")

    # Determine which arms to test
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"  Test arms:   {', '.join(sides)}")

    # Connect to robot
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\nError: CartesianController not available, please use --dry-run mode")
            return

        print("\n[1/3] Connect to robot...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  Connected to {ROBOT_IP}")

            print("  Setting impedance mode...")
            controller.set_impedance_mode(mode='joint')
            print("  Robot enabled")

            print("  Moving to initial pose...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ Reached initial pose")

        except Exception as e:
            print(f"  ❌ Connection failed: {e}")
            return
    else:
        print("\n[1/3] Skipping robot connection (dry run mode)")

    # Prepare result statistics
    results = {
        'left': {'success': 0, 'fail': 0, 'cases': []},
        'right': {'success': 0, 'fail': 0, 'cases': []}
    }

    # Run test
    print("\n[2/3] Starting tests...")
    print_test_header()

    for side in sides:
        # Fixed wrist pose (using initial pose)
        wrist_pos = TIANJI_INIT_POS[side].copy()
        wrist_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        for name, pitch, yaw in test_cases:
            # Compute elbow direction
            direction = elbow_direction_from_angles(pitch, yaw, side)

            # Build zsp_para
            zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

            ik_success = True  # Default success

            if controller:
                try:
                    # Set arm angle
                    if side == 'left':
                        controller.left_zsp_para = zsp_para
                    else:
                        controller.right_zsp_para = zsp_para

                    # Build pose matrix
                    pose_mat = np.eye(4)
                    pose_mat[:3, :3] = wrist_rot.as_matrix()
                    pose_mat[:3, 3] = wrist_pos

                    # Send control command
                    if side == 'left':
                        left_ok, _, left_joints, _ = controller.move_to_pose_direct(
                            left_pose=pose_mat,
                            right_pose=None,
                            unit='matrix'
                        )
                        ik_success = left_ok
                    else:
                        _, right_ok, _, right_joints = controller.move_to_pose_direct(
                            left_pose=None,
                            right_pose=pose_mat,
                            unit='matrix'
                        )
                        ik_success = right_ok

                    # Hold for a period
                    time.sleep(args.hold_time)

                except Exception as e:
                    print(f"  ⚠️ Control exception: {e}")
                    ik_success = False

            # Record result
            print_test_result(name, pitch, yaw, direction, ik_success, side)

            if ik_success:
                results[side]['success'] += 1
            else:
                results[side]['fail'] += 1

            results[side]['cases'].append({
                'name': name,
                'pitch': pitch,
                'yaw': yaw,
                'direction': direction.tolist(),
                'success': ik_success
            })

    # Print statistics
    print("\n" + "=" * 90)
    print("[3/3] Test Statistics")
    print("=" * 90)

    for side in sides:
        total = results[side]['success'] + results[side]['fail']
        success_rate = results[side]['success'] / total * 100 if total > 0 else 0
        print(f"\n{side.upper()} ARM:")
        print(f"  Success: {results[side]['success']}/{total} ({success_rate:.1f}%)")
        print(f"  Failed: {results[side]['fail']}/{total}")

        if results[side]['fail'] > 0:
            print(f"  Failed cases:")
            for case in results[side]['cases']:
                if not case['success']:
                    print(f"    - {case['name']}: pitch={case['pitch']}°, yaw={case['yaw']}°")

    # Cleanup
    if controller:
        print("\nDisabling and releasing robot...")
        controller.disable_and_release()
        print("Robot safely exited")

    print("\n" + "=" * 70)
    print("Test complete")
    print("=" * 70)


def run_position_test(args):
    """Run position stability test"""

    print("\n" + "=" * 70)
    print("Step 6: Position Stability Test")
    print("=" * 70)

    # Parse position range
    x_range = tuple(args.pos_x_range) if args.pos_x_range else None
    y_range = tuple(args.pos_y_range) if args.pos_y_range else None
    z_range = tuple(args.pos_z_range) if args.pos_z_range else None

    print(f"\nParameters:")
    print(f"  Coordinate system: Robot World (ROS REP 103)")
    print(f"    X = Forward(+) / Backward(-)")
    print(f"    Y = Left(+) / Right(-)")
    print(f"    Z = Up(+) / Down(-)")
    print(f"  Note: Internally auto-converted to Chest coordinate system (unified interface for both arms)")
    if x_range:
        print(f"  X range (front/back): {x_range[0]*100:.1f}cm ~ {x_range[1]*100:.1f}cm, step {x_range[2]*100:.1f}cm")
    if y_range:
        print(f"  Y range (left/right): {y_range[0]*100:.1f}cm ~ {y_range[1]*100:.1f}cm, step {y_range[2]*100:.1f}cm")
    if z_range:
        print(f"  Z range (up/down): {z_range[0]*100:.1f}cm ~ {z_range[1]*100:.1f}cm, step {z_range[2]*100:.1f}cm")
    print(f"  Hold time:   {args.hold_time}s")
    print(f"  Mode:       {'Dry run' if args.dry_run else 'Real robot control'}")

    # Check whether any range parameter is provided
    if x_range is None and y_range is None and z_range is None:
        print("\nError: Position mode requires at least one range parameter (--pos-x-range, --pos-y-range, --pos-z-range)")
        return

    # Determine which arms to test
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"  Test arms:   {', '.join(sides)}")

    # Connect to robot
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\nError: CartesianController not available, please use --dry-run mode")
            return

        print("\n[1/3] Connect to robot...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  Connected to {ROBOT_IP}")

            print("  Setting impedance mode...")
            controller.set_impedance_mode(mode='joint')
            print("  Robot enabled")

            print("  Moving to initial pose...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ Reached initial pose")

        except Exception as e:
            print(f"  ❌ Connection failed: {e}")
            return
    else:
        print("\n[1/3] Skipping robot connection (dry run mode)")

    # Prepare result statistics
    results = {
        'left': {'success': 0, 'fail': 0, 'cases': []},
        'right': {'success': 0, 'fail': 0, 'cases': []}
    }

    # Run test
    print("\n[2/3] Starting tests...")
    print_position_test_header()

    # Using fixed default relaxed elbow arm angle
    default_pitch, default_yaw = 0, 45

    for side in sides:
        # Test cases use World coordinate system (same for both arms)
        test_cases = generate_position_test_cases(x_range, y_range, z_range)
        print(f"\n  [{side.upper()} ARM] test cases: {len(test_cases)}")

        # Base pose (initial pose, Chest coordinate system)
        base_pos = TIANJI_INIT_POS[side].copy()
        base_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        # Set default arm angle
        direction = elbow_direction_from_angles(default_pitch, default_yaw, side)
        zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

        for name, dx, dy, dz in test_cases:
            # dx, dy, dz are World coordinate system increments, need to convert to Chest coordinate system
            displacement_world = np.array([dx, dy, dz])
            displacement_chest = transform_world_to_chest(displacement_world, side)
            target_pos = base_pos + displacement_chest

            ik_success = True  # Default success

            if controller:
                try:
                    # Set arm angle
                    if side == 'left':
                        controller.left_zsp_para = zsp_para
                    else:
                        controller.right_zsp_para = zsp_para

                    # Build pose matrix
                    pose_mat = np.eye(4)
                    pose_mat[:3, :3] = base_rot.as_matrix()
                    pose_mat[:3, 3] = target_pos

                    # Send control command
                    if side == 'left':
                        left_ok, _, left_joints, _ = controller.move_to_pose_direct(
                            left_pose=pose_mat,
                            right_pose=None,
                            unit='matrix'
                        )
                        ik_success = left_ok
                    else:
                        _, right_ok, _, right_joints = controller.move_to_pose_direct(
                            left_pose=None,
                            right_pose=pose_mat,
                            unit='matrix'
                        )
                        ik_success = right_ok

                    # Hold for a period
                    time.sleep(args.hold_time)

                except Exception as e:
                    print(f"  ⚠️ Control exception: {e}")
                    ik_success = False

            # Record result
            print_position_test_result(name, dx, dy, dz, ik_success, side)

            if ik_success:
                results[side]['success'] += 1
            else:
                results[side]['fail'] += 1

            results[side]['cases'].append({
                'name': name,
                'dx': dx,
                'dy': dy,
                'dz': dz,
                'success': ik_success
            })

    # Print statistics
    print("\n" + "=" * 90)
    print("[3/3] Test Statistics")
    print("=" * 90)

    for side in sides:
        total = results[side]['success'] + results[side]['fail']
        success_rate = results[side]['success'] / total * 100 if total > 0 else 0
        print(f"\n{side.upper()} ARM:")
        print(f"  Success: {results[side]['success']}/{total} ({success_rate:.1f}%)")
        print(f"  Failed: {results[side]['fail']}/{total}")

        if results[side]['fail'] > 0:
            print(f"  Failed cases:")
            for case in results[side]['cases']:
                if not case['success']:
                    print(f"    - {case['name']}: Δx={case['dx']*100:.1f}cm, Δy={case['dy']*100:.1f}cm, Δz={case['dz']*100:.1f}cm")

    # Cleanup
    if controller:
        print("\nDisabling and releasing robot...")
        controller.disable_and_release()
        print("Robot safely exited")

    print("\n" + "=" * 70)
    print("Test complete")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="Step 6: Robot Stability Test (Simulated Tracking Data)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  [Arm Angle Test]
  # Dry run (print only)
  python3 step6_arm_angle_stability_test.py --dry-run

  # test left arm, pitch +/-10 deg, yaw 30 deg~60 deg
  python3 step6_arm_angle_stability_test.py --left-only --pitch-range -10 10 5 --yaw-range 30 60 10

  [Position Test]
  # Forward/backward movement test (+-5cm, step 1cm)
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01

  # Combined position test (forward/backward + up/down)
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.03 0.03 0.01 --pos-z-range -0.03 0.03 0.01

  [Rotation Test]
  # Around +X axis rotation 30 deg (Wrist rolls right)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only

  # Around -Y axis rotation 20 deg (Fingers tilt up) - Note: use = syntax for negative axis
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Y --rot-angle 20 --left-only

  # Around +Z axis rotation 30 deg (Fingers yaw left)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis Z --rot-angle 30 --left-only

  # Around -Z axis rotation 30 deg (Fingers yaw right) - Note: use = syntax for negative axis
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Z --rot-angle 30 --left-only

Angle descriptions (arm angle mode):
  Pitch: elbow forward/backward offset
    - Positive = elbow forward
    - Negative = elbow backward
    - 0° = No front/back offset

  Yaw (abduction angle): elbow anti-gravity/outward ratio (Chest coordinate system)
    - 0° = Elbow purely anti-gravity [left arm: 0,-1,0] [right arm: 0,+1,0]
    - 45° = Default relaxed elbow [left arm: 0,-0.707,-0.707] [right arm: 0,+0.707,-0.707]
    - 90° = Elbow purely outward [0, 0, -1]

Position description (position mode) - Robot World coordinate system (same for both arms):
  X direction: Forward(+) / Backward(-)
  Y direction: Left(+) / Right(-)
  Z direction: Up(+) / Down(-)

  Note: Internally auto-converted to Chest coordinate system (users need not worry about Chest axis differences)

Rotation description (rotation mode) - Robot World coordinate system (ROS REP 103 right-hand rule, CCW positive from axis positive end):
  Around +X axis: Wrist rolls right (Roll right: top tilts right)
  Around -X axis: Wrist rolls left (Roll left: top tilts left)
  Around +Y axis: Fingers press down (Pitch down: look down)
  Around -Y axis: Fingers tilt up (Pitch up: look up)
  Around +Z axis: Fingers yaw left (Yaw left: counterclockwise from above)
  Around -Z axis: Fingers yaw right (Yaw right: clockwise from above)
"""
    )

    # Common parameters
    parser.add_argument('--mode', type=str, choices=['arm_angle', 'position', 'rotation'], default='arm_angle',
                        help='Test mode: arm_angle=arm angle test, position=position test, rotation=rotation test (default: arm_angle)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Dry run mode, print only without controlling robot')
    parser.add_argument('--left-only', action='store_true',
                        help='Test left arm only')
    parser.add_argument('--right-only', action='store_true',
                        help='Test right arm only')
    parser.add_argument('--hold-time', type=float, default=1.0,
                        help='Hold time per test point (seconds) (default: 1.0)')

    # Arm angle test parameters
    parser.add_argument('--pitch-range', type=int, nargs=3, default=[-10, 10, 5],
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Pitch angle range (default: -10 10 5)')
    parser.add_argument('--yaw-range', type=int, nargs=3, default=[30, 60, 10],
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Abduction angle range (default: 30 60 10)')

    # Position test parameters
    parser.add_argument('--pos-x-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='X direction (forward/backward) offset range, unit: meters (e.g.: -0.05 0.05 0.01)')
    parser.add_argument('--pos-y-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Y direction (left/right) offset range, unit: meters (e.g.: -0.05 0.05 0.01)')
    parser.add_argument('--pos-z-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Z direction (up/down) offset range, unit: meters (e.g.: -0.05 0.05 0.01)')

    # Rotation test parameters
    parser.add_argument('--rot-axis', type=str, default='X',
                        help='Rotation axis (Robot World coordinate system): X, -X, Y, -Y, Z, -Z (default: X)')
    parser.add_argument('--rot-angle', type=float, default=30.0,
                        help='Rotation angle in degrees (default: 30.0)')

    args = parser.parse_args()

    # Check parameter conflicts
    if args.left_only and args.right_only:
        print("Error: --left-only and --right-only cannot be used simultaneously")
        sys.exit(1)

    # Run corresponding test based on mode
    if args.mode == 'arm_angle':
        run_stability_test(args)
    elif args.mode == 'position':
        run_position_test(args)
    else:  # rotation
        run_rotation_test(args)


if __name__ == '__main__':
    main()
