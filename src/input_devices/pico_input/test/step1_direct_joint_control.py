#!/usr/bin/env python3
"""
=============================================================================
Step 1: Direct Joint Space Control (Most Basic Test)
=============================================================================

Test Purpose:
  Verify robot SDK connection and basic motion functionality

Input/Output:
  Input: Target pose (Cartesian space coordinates)
  Processing: IK calculation -> joint angles -> quintic polynomial interpolation
  Output: Send joint angle commands to the real robot

Usage:
  cd ~/Desktop/wuji-hand-teleop/src/input_devices/pico_input/test

  # === Translation Tests ===
  # Basic test (maintain initial pose)
  python3 step1_direct_joint_control.py --mode static --duration 5.0

  # Forward (world +X) - recommended parameters
  python3 step1_direct_joint_control.py --mode forward --speed 0.03 --duration 2.0

  # Backward (world -X)
  python3 step1_direct_joint_control.py --mode back --speed 0.03 --duration 2.0

  # Move left (world +Y)
  python3 step1_direct_joint_control.py --mode left --speed 0.02 --duration 2.0

  # Move right (world -Y)
  python3 step1_direct_joint_control.py --mode right --speed 0.02 --duration 2.0

  # Move up (world +Z)
  python3 step1_direct_joint_control.py --mode up --speed 0.02 --duration 2.0

  # Move down (world -Z)
  python3 step1_direct_joint_control.py --mode down --speed 0.02 --duration 2.0

  # === Rotation Tests (extrinsic rotation, around world coordinate axes) ===
  # Alignment test (verify end-effector axes align with world coordinate system)
  python3 step1_direct_joint_control.py --mode align_test --duration 5.0
  # -> Robot arm moves to "standard horizontal pose"
  # -> In RViz check: world_left_dh_ee and world_right_dh_ee axes
  # -> Should be perfectly parallel to world axes (X-red, Y-green, Z-blue)

  # Rotate around X axis (roll, front-back axis) - speed unit is rad/s
  python3 step1_direct_joint_control.py --mode rotate_x --speed 0.3 --duration 3.0
  # Rotation angle = 0.3 rad/s * 3.0 s = 0.9 rad ~ 51.6 degrees

  # Rotate around Y axis (pitch, left-right axis)
  python3 step1_direct_joint_control.py --mode rotate_y --speed 0.3 --duration 3.0

  # Rotate around Z axis (yaw, up-down axis)
  python3 step1_direct_joint_control.py --mode rotate_z --speed 0.3 --duration 3.0

  # === Other Tests ===
  # Single arm test (left arm only)
  python3 step1_direct_joint_control.py --mode left_only --speed 0.03 --duration 3.0

  # Single arm test (right arm only)
  python3 step1_direct_joint_control.py --mode right_only --speed 0.03 --duration 3.0

Not required:
  No ROS2
  No TF
  No Launch file

Features:
  - Uses joint space interpolation (quintic polynomial) for smooth motion
  - Avoids discontinuities caused by continuous IK computation
  - Directly calls robot SDK, bypasses ROS2
  - References cartesian_controller.move_to_init implementation

Verification Results:
  Robot arm can connect
  Robot arm can move to initial pose
  Control is smooth without jitter
  Auto power-off after motion completes

=============================================================================
"""

import argparse
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import robot control library (using relative path)
from pathlib import Path
_test_dir = str(Path(__file__).resolve().parent)
_src_dir = str(Path(__file__).resolve().parents[3])  # test -> pico_input -> input_devices -> src
_tianji_wo_dir = str(Path(_src_dir) / 'output_devices' / 'tianji_world_output')
for _p in [_tianji_wo_dir, _test_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    WORLD_REFERENCE_ROT,
    ZSP_PARA,
    DGR,
    ZSP_TYPE,
    ZSP_ANGLE,
    ROBOT_IP,
    KINE_CONFIG_PATH,
)
from common.transform_utils import (
    transform_world_to_chest,
    get_direction_vector_world,
    get_world_to_chest_rotation,
    apply_world_rotation_to_chest_pose,
)
from common.robot_lifecycle import enable_robot, disable_robot, send_joint_command
from tianji_world_output.fx_kine import Marvin_Kine
from tianji_world_output.fx_robot import Marvin_Robot


class JointSpaceController:
    """Joint space trajectory controller (using quintic polynomial interpolation)"""

    def __init__(self, mode: str, speed: float, duration: float,
                 interpolation_dt: float = 0.02, dry_run: bool = False):
        self.mode = mode
        self.speed = speed
        self.duration = duration
        self.interpolation_dt = interpolation_dt  # Interpolation time step
        self.dry_run = dry_run

        # Initialize kinematics library
        print("Initializing kinematics library...")
        config_path = KINE_CONFIG_PATH

        self.kine_left = Marvin_Kine()
        self.kine_right = Marvin_Kine()

        config_result_left = self.kine_left.load_config(config_path)
        if not config_result_left:
            raise RuntimeError("Failed to load left arm configuration")

        self.kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result_left['TYPE'][0],
            dh=config_result_left['DH'][0],
            pnva=config_result_left['PNVA'][0],
            j67=config_result_left['BD'][0]
        )

        config_result_right = self.kine_right.load_config(config_path)
        if not config_result_right:
            raise RuntimeError("Failed to load right arm configuration")

        self.kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result_right['TYPE'][1],
            dh=config_result_right['DH'][1],
            pnva=config_result_right['PNVA'][1],
            j67=config_result_right['BD'][1]
        )

        print("Kinematics library initialized successfully")

        # Initialize robot connection
        if not self.dry_run:
            print("Connecting to robot...")
            self.robot = Marvin_Robot()
            robot_ip = ROBOT_IP
            connection_status = self.robot.connect(robot_ip)
            if not connection_status:
                raise RuntimeError(f"Failed to connect to robot (IP: {robot_ip})")
            print(f"Robot connected successfully (IP: {robot_ip})")

            # Enable robot (power on)
            enable_robot(self.robot, state=3, vel_ratio=60, acc_ratio=60)
        else:
            self.robot = None
            print("Dry-run mode: not actually controlling the robot")

        # zsp_para (elbow direction hint)
        self.zsp_para_left = ZSP_PARA['left']
        self.zsp_para_right = ZSP_PARA['right']

        print("="*60)
        print(f"Joint Space Trajectory Control - Quintic Polynomial Interpolation")
        print(f"  Mode: {mode}")
        print(f"  Speed: {speed} m/s")
        print(f"  Duration: {duration} seconds")
        print(f"  Interpolation step: {interpolation_dt * 1000:.0f} ms")
        print("="*60)

    def run(self):
        """Run trajectory control"""
        print("\nStarting motion...")

        if self.mode == 'static':
            self._run_static()
        elif self.mode == 'align_test':
            self._run_align_test()
        elif self.mode in ['forward', 'back', 'left', 'right', 'up', 'down']:
            self._run_linear()
        elif self.mode in ['rotate_x', 'rotate_y', 'rotate_z']:
            self._run_rotation()
        elif self.mode in ['left_only', 'right_only']:
            self._run_single_arm()
        else:
            raise ValueError(f"Unknown mode: {self.mode}")

        print(f"\nTest completed!")

    def disable_robot_safe(self):
        """Safely disable robot"""
        if not self.dry_run and self.robot is not None:
            disable_robot(self.robot)

    def _run_static(self):
        """Static test: maintain initial pose"""
        print("Maintaining initial pose...")
        start_joints = {
            'left': INIT_JOINTS['left'],
            'right': INIT_JOINTS['right']
        }
        end_joints = start_joints

        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _run_align_test(self):
        """Alignment test: move to world-coordinate-aligned pose

        Purpose: Verify whether end-effector axes align with world coordinate system
        - Position: Keep initial position unchanged
        - Orientation: Set to world reference orientation (identity matrix)
        - In RViz check: world_left_dh_ee and world_right_dh_ee axes
          should be perfectly parallel to world axes (X-red, Y-green, Z-blue)
        """
        print("="*60)
        print("Alignment test: move to world-coordinate-aligned pose")
        print("="*60)
        print("Goal: End-effector axes fully aligned with world coordinate system")
        print("  X-axis (red) -> forward")
        print("  Y-axis (green) -> left")
        print("  Z-axis (blue) -> up")
        print()
        print("Verification method:")
        print("  1. Run step3_visualize_in_rviz.py to start visualization")
        print("  2. Observe TF axes in RViz")
        print("  3. world_left_dh_ee and world_right_dh_ee should be parallel to world")
        print("="*60)

        # Position remains unchanged
        target_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }

        # Set orientation to world reference orientation
        # world -> world_left/world_right rotation (using shared utilities)
        R_world_to_left = get_world_to_chest_rotation('left')
        R_world_to_right = get_world_to_chest_rotation('right')

        # Convert world reference orientation to each arm's coordinate system
        target_rot = {
            'left': R_world_to_left @ WORLD_REFERENCE_ROT,
            'right': R_world_to_right @ WORLD_REFERENCE_ROT
        }

        print("\nComputing IK...")
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints_with_rotation(target_pos, target_rot)

        if end_joints is None:
            print("IK failed, cannot generate trajectory")
            return

        print("\nStarting motion to aligned pose...")
        self._execute_trajectory(start_joints, end_joints, self.duration)

        print("\n" + "="*60)
        print("Reached aligned pose")
        print("Please verify in RViz that end-effector axes are parallel to world coordinate system")
        print("If parallel, the coordinate system setup is correct")
        print("="*60)

    def _run_linear(self):
        """Linear motion (unified world coordinate system architecture)

        Architecture:
          1. Compute direction and distance uniformly in world coordinate system
          2. Convert to chest coordinate system in _compute_target_joints
          3. Avoid duplicated coordinate conversion code
        """
        # Define direction in world coordinate system (unified)
        direction_world = get_direction_vector_world(self.mode)
        distance = self.speed * self.duration

        print(f"Motion direction: {self.mode}")
        print(f"Motion distance: {distance:.4f} m")
        print(f"World direction vector: {direction_world}")

        # Compute displacement in world coordinate system (unified)
        displacement_world = direction_world * distance
        print(f"World displacement: {displacement_world}")

        # Compute target position (in chest coordinate system)
        # Note: TIANJI_INIT_POS is already in each arm's chest coordinate system
        # Need to convert world displacement to chest coordinate system
        start_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }

        # Convert world displacement to chest coordinate system
        displacement_left_chest = transform_world_to_chest(displacement_world, 'left')
        displacement_right_chest = transform_world_to_chest(displacement_world, 'right')

        print(f"left_chest displacement: {displacement_left_chest}")
        print(f"right_chest displacement: {displacement_right_chest}")

        end_pos = {
            'left': start_pos['left'] + displacement_left_chest,
            'right': start_pos['right'] + displacement_right_chest
        }

        # Call IK to compute start and end joint angles
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints(end_pos)

        if end_joints is None:
            print("IK failed, cannot generate trajectory")
            return

        # Execute joint space trajectory
        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _run_rotation(self):
        """Rotation motion (change orientation only, position unchanged)

        Purpose: Test wrist rotation (RPY)
        - Position remains unchanged (controlled by translation)
        - Only change orientation (controlled by rotation)
        - VR teleoperation: controller position -> position, controller orientation -> orientation

        Core principle:
        - Left and right arms should have consistent end-effector orientations in world coordinate system
        - This way when both wrists "turn left together", they look synchronized in world frame
        """
        # Compute rotation angle (radians)
        rotation_angle = self.speed * self.duration  # speed here means angular velocity (rad/s)

        print(f"Rotation direction: {self.mode}")
        print(f"Rotation angle: {np.degrees(rotation_angle):.2f} deg ({rotation_angle:.4f} rad)")

        # Define rotation axis in world coordinate system (standard definition)
        if self.mode == 'rotate_x':
            axis_world = np.array([1.0, 0.0, 0.0])  # Around world X axis
            axis_name = "X-axis (front-back)"
        elif self.mode == 'rotate_y':
            axis_world = np.array([0.0, 1.0, 0.0])  # Around world Y axis
            axis_name = "Y-axis (left-right)"
        elif self.mode == 'rotate_z':
            axis_world = np.array([0.0, 0.0, 1.0])  # Around world Z axis
            axis_name = "Z-axis (up-down)"

        print(f"World rotation axis: {axis_name} {axis_world}")
        print("Key: Left and right arms have consistent end-effector orientations in world frame")

        # Position remains unchanged
        start_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }
        end_pos = start_pos  # Position unchanged

        # === Core algorithm (using shared library apply_world_rotation_to_chest_pose) ===
        print("\nAlgorithm steps:")

        # Step 1: Get current end-effector orientation (in each arm's coordinate system)
        current_rot_in_arm = {
            'left': TIANJI_INIT_ROT['left'],
            'right': TIANJI_INIT_ROT['right']
        }
        print("  1. Current end-effector orientations (in each arm's coordinate system) are known")

        # Steps 2-4: Use shared library to execute 4-step algorithm
        # target_chest = R_w2c @ R_delta @ R_c2w @ base_chest
        R_delta = R.from_rotvec(axis_world * rotation_angle)
        end_rot = {
            side: apply_world_rotation_to_chest_pose(
                current_rot_in_arm[side], R_delta, side
            )
            for side in ['left', 'right']
        }
        print(f"  2-4. Used shared library apply_world_rotation_to_chest_pose() to complete 4-step algorithm")
        print(f"       Rotated {np.degrees(rotation_angle):.2f} deg in world (around {axis_name})")
        print("       Applied same rotation increment to both arms")

        print("\nAlgorithm guarantees:")
        print("  - Both arms had the same rotation applied in world frame")
        print("  - Rotation axis is in world coordinate system")
        print("  - Position unchanged, only orientation changed")

        # Call IK to compute start and end joint angles
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints_with_rotation(end_pos, end_rot)

        if end_joints is None:
            print("IK failed, cannot generate trajectory")
            return

        # Execute joint space trajectory
        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _compute_target_joints_with_rotation(self, target_pos, target_rot):
        """Compute joint angles for target position and orientation (supports rotation)"""
        result = {}

        for side in ['left', 'right']:
            # Construct pose matrix
            pose_mat = np.eye(4)
            pose_mat[:3, :3] = target_rot[side]
            pose_mat[:3, 3] = target_pos[side] * 1000  # Convert to millimeters

            # Call IK
            kine = self.kine_left if side == 'left' else self.kine_right
            serial = 0 if side == 'left' else 1
            ref_joints = INIT_JOINTS[side]
            zsp_para = self.zsp_para_left if side == 'left' else self.zsp_para_right

            ik_result = kine.ik(
                robot_serial=serial,
                pose_mat=pose_mat.tolist(),
                ref_joints=ref_joints,
                zsp_type=ZSP_TYPE,
                zsp_para=zsp_para,
                zsp_angle=ZSP_ANGLE,
                dgr=DGR
            )

            if ik_result is False:
                print(f"  {side} arm IK failed: cannot compute")
                return None

            if ik_result.m_Output_IsOutRange:
                print(f"  {side} arm IK failed: out of workspace")
                return None

            # Convert to list
            joints = ik_result.m_Output_RetJoint
            if hasattr(joints, 'to_list'):
                joints = joints.to_list()
            elif hasattr(joints, 'data'):
                joints = list(joints.data)
            else:
                joints = list(joints)

            result[side] = joints

        return result

    def _run_single_arm(self):
        """Single arm motion"""
        active_side = 'left' if self.mode == 'left_only' else 'right'
        print(f"Only {active_side} arm moving")

        direction = np.array([1.0, 0.0, 0.0])  # Forward
        distance = self.speed * self.duration

        start_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }
        end_pos = start_pos.copy()
        end_pos[active_side] = start_pos[active_side] + direction * distance

        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints(end_pos)

        if end_joints is None:
            print("IK failed")
            return

        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _compute_target_joints(self, target_pos):
        """Compute joint angles for target position (calls IK)"""
        result = {}

        for side in ['left', 'right']:
            # Construct pose matrix
            pose_mat = np.eye(4)
            pose_mat[:3, :3] = TIANJI_INIT_ROT[side]
            pose_mat[:3, 3] = target_pos[side] * 1000  # Convert to millimeters

            # Call IK
            kine = self.kine_left if side == 'left' else self.kine_right
            serial = 0 if side == 'left' else 1
            ref_joints = INIT_JOINTS[side]
            zsp_para = self.zsp_para_left if side == 'left' else self.zsp_para_right

            ik_result = kine.ik(
                robot_serial=serial,
                pose_mat=pose_mat.tolist(),
                ref_joints=ref_joints,
                zsp_type=ZSP_TYPE,
                zsp_para=zsp_para,
                zsp_angle=ZSP_ANGLE,
                dgr=DGR
            )

            if ik_result is False:
                print(f"  {side} arm IK failed: cannot compute")
                return None

            if ik_result.m_Output_IsOutRange:
                print(f"  {side} arm IK failed: out of workspace")
                return None

            # Convert to list
            joints = ik_result.m_Output_RetJoint
            if hasattr(joints, 'to_list'):
                joints = joints.to_list()
            elif hasattr(joints, 'data'):
                joints = list(joints.data)
            else:
                joints = list(joints)

            result[side] = joints

        return result

    def _execute_trajectory(self, start_joints, end_joints, duration, verbose=True):
        """
        Execute trajectory in joint space using quintic polynomial interpolation

        Quintic polynomial: s(t) = 10t^3 - 15t^4 + 6t^5
        Properties: s(0)=0, s(1)=1, s'(0)=0, s'(1)=0, s''(0)=0, s''(1)=0
        Ensures zero velocity and acceleration at start and end
        """
        if duration <= 0 or self.interpolation_dt <= 0:
            raise ValueError("duration and interpolation_dt must be greater than 0")
        num_points = max(1, int(duration / self.interpolation_dt))

        if verbose:
            print(f"Generating trajectory: {num_points} interpolation points")

        for i in range(num_points + 1):
            t = i / num_points  # Normalized time [0, 1]

            # Quintic polynomial interpolation
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            # Compute current joint angles
            target_left = [
                start_joints['left'][j] + s * (end_joints['left'][j] - start_joints['left'][j])
                for j in range(7)
            ]
            target_right = [
                start_joints['right'][j] + s * (end_joints['right'][j] - start_joints['right'][j])
                for j in range(7)
            ]

            # Send command
            if not self.dry_run:
                send_joint_command(self.robot, target_left, target_right)

            # Progress display
            if verbose and i % max(1, num_points // 10) == 0:
                progress = int(t * 100)
                print(f"  [{progress:3d}%] t={i*self.interpolation_dt:.2f}s", flush=True)

            time.sleep(self.interpolation_dt)


def main():
    parser = argparse.ArgumentParser(description='Tianji robot joint space trajectory control test')
    parser.add_argument('--mode', type=str, default='static',
                        choices=['static', 'align_test',
                                'forward', 'back', 'left', 'right', 'up', 'down',
                                'rotate_x', 'rotate_y', 'rotate_z',
                                'left_only', 'right_only'],
                        help='Test mode')
    parser.add_argument('--speed', type=float, default=0.02,
                        help='Movement speed (m/s) or angular velocity (rad/s, for rotation)')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Duration (seconds)')
    parser.add_argument('--dt', type=float, default=0.02,
                        help='Interpolation time step (seconds)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Dry-run mode (do not actually control the robot)')

    args = parser.parse_args()

    controller = None
    try:
        controller = JointSpaceController(
            mode=args.mode,
            speed=args.speed,
            duration=args.duration,
            interpolation_dt=args.dt,
            dry_run=args.dry_run
        )
        controller.run()
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure robot is powered off
        if controller is not None:
            controller.disable_robot_safe()


if __name__ == '__main__':
    main()
