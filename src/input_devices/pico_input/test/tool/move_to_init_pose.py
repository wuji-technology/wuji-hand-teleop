#!/usr/bin/env python3
"""
Move robot to initial pose (using shared modules and lifecycle management)
Corresponds to robot_*_init configuration in pico_input.yaml
"""

import sys
import numpy as np
import time
from pathlib import Path

# Add paths (using relative paths)
_tool_dir = Path(__file__).resolve().parent
_test_dir = str(_tool_dir.parent)
_src_dir = _tool_dir.parents[3]  # tool → test → pico_input → input_devices → src
_tianji_wo_dir = str(_src_dir / 'output_devices' / 'tianji_world_output')
for _p in [_tianji_wo_dir, _test_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
from common.robot_config import INIT_JOINTS, ROBOT_IP
from common.robot_lifecycle import enable_robot, disable_robot, send_joint_command
from tianji_world_output.cartesian_controller import CartesianController


def main():
    controller = None

    try:
        print("=" * 70)
        print("Move robot to initial pose (PICO teleoperation zero position)")
        print("=" * 70)

        print("\nInitial joint angles:")
        print(f"  left arm: {INIT_JOINTS['left']}")
        print(f"  right arm: {INIT_JOINTS['right']}")

        # Connect to robot
        print("\nConnecting to robot...")
        controller = CartesianController(robot_ip=ROBOT_IP)
        print("Connected to robot")

        # Enable robot (using unified lifecycle management)
        enable_robot(controller.robot, state=3, vel_ratio=60, acc_ratio=60)

        # Compute FK to verify target pose
        print("\nComputing FK to verify target pose...")
        left_fk = controller.kine_left.fk(0, INIT_JOINTS['left'])
        right_fk = controller.kine_right.fk(1, INIT_JOINTS['right'])

        left_pos = np.array([left_fk[0][3], left_fk[1][3], left_fk[2][3]]) / 1000.0
        right_pos = np.array([right_fk[0][3], right_fk[1][3], right_fk[2][3]]) / 1000.0

        print("\nTarget pose (FK computed):")
        print(f"  Left wrist position: [{left_pos[0]:.4f}, {left_pos[1]:.4f}, {left_pos[2]:.4f}]")
        print(f"  Right wrist position: [{right_pos[0]:.4f}, {right_pos[1]:.4f}, {right_pos[2]:.4f}]")

        # Get current joint angles
        print("\nReading current joint angles...")
        current_joints = controller.get_current_joints()
        current_left = np.array(current_joints[0])
        current_right = np.array(current_joints[1])

        print(f"  Current left arm: {current_left}")
        print(f"  Current right arm: {current_right}")

        # Move to initial pose
        print("\nStarting motion...")
        print("  ⚠️  Please ensure no obstacles around the robot!")
        print("  ⚠️  Robot will smoothly move to initial pose within 5 seconds")
        input("  Press Enter to continue...")

        # Smooth interpolation movement (5 seconds, 100Hz)
        duration = 5.0
        rate = 100  # Hz
        num_steps = int(duration * rate)
        dt = 1.0 / rate

        target_left_final = np.array(INIT_JOINTS['left'])
        target_right_final = np.array(INIT_JOINTS['right'])

        print(f"\nMoving... ({duration} seconds)")
        for i in range(num_steps + 1):
            alpha = i / num_steps  # 0 → 1

            # Linear interpolation
            target_left = current_left + alpha * (target_left_final - current_left)
            target_right = current_right + alpha * (target_right_final - current_right)

            # Send joint commands (using unified interface)
            send_joint_command(controller.robot, target_left.tolist(), target_right.tolist())

            # Progress display
            if i % 50 == 0:
                progress = int(100 * alpha)
                print(f"  Progress: {progress}%")

            time.sleep(dt)

        print("\n✓ Reached initial pose!")
        print("\nRobot is now at PICO teleoperation zero pose (end-effector axis directions in World):")
        print("  - Leftwrist: X→Right(-Y), Y→Down(-Z), Z→Forward(+X)")
        print("  - Rightwrist: X→Left(+Y), Y→Up(+Z), Z→Forward(+X)")
        print("\nReady for teleoperation testing!")

    except KeyboardInterrupt:
        print("\n\n⚠️  User interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure power-off (important!)
        if controller is not None:
            disable_robot(controller.robot)


if __name__ == '__main__':
    main()
