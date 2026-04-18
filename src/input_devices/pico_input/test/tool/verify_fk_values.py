#!/usr/bin/env python3
"""
Verify TIANJI_INIT_POS/ROT values in tianji_robot.yaml

Functions:
  1. Compute poses from INIT_JOINTS using FK
  2. Compare with tianji_robot.yaml configuration values
  3. Display coordinate system description

Usage:
  cd /path/to/test && python3 tool/verify_fk_values.py
"""

import sys
import numpy as np
from pathlib import Path

# Import shared modules (using relative path)
_tool_dir = Path(__file__).resolve().parent
_test_dir = str(_tool_dir.parent)  # tool → test
_src_dir = _tool_dir.parents[3]    # tool → test → pico_input → input_devices → src
_tianji_wo_dir = str(_src_dir / 'output_devices' / 'tianji_world_output')
for _p in [_tianji_wo_dir, _test_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    ROBOT_IP,
)

# Import robot controller
try:
    from tianji_world_output.cartesian_controller import CartesianController
    HAS_CONTROLLER = True
except ImportError:
    print("⚠️  Cannot import CartesianController, will only display configuration values")
    HAS_CONTROLLER = False


def main():
    print("=" * 70)
    print("Verify FK computed values vs tianji_robot.yaml configuration values")
    print("=" * 70)

    print("\n[1]INIT_JOINTS (initial joint angles, degrees)")
    print(f"  left arm: {INIT_JOINTS['left']}")
    print(f"  right arm: {INIT_JOINTS['right']}")

    print("\n[2]tianji_robot.yaml configuration values (Chest coordinate system)")
    print("\n  TIANJI_INIT_POS (end-effector position, meters):")
    print(f"    left arm: {TIANJI_INIT_POS['left']}")
    print(f"    right arm: {TIANJI_INIT_POS['right']}")

    print("\n  TIANJI_INIT_ROT (end-effector orientation, rotation matrix):")
    print(f"    left arm:\n{TIANJI_INIT_ROT['left']}")
    print(f"    right arm:\n{TIANJI_INIT_ROT['right']}")

    if not HAS_CONTROLLER:
        print("\n" + "=" * 70)
        print("Warning: Cannot connect to robot, skipping FK verification")
        print("=" * 70)
        return

    print("\n[3]Recomputing using FK...")

    try:
        controller = CartesianController(robot_ip=ROBOT_IP)
        print(f"  Connected to robot {ROBOT_IP}")

        # compute FK
        left_fk = controller.kine_left.fk(0, INIT_JOINTS['left'])
        right_fk = controller.kine_right.fk(1, INIT_JOINTS['right'])

        # Extract position (millimeters -> meters)
        left_pos_fk = np.array([left_fk[0][3], left_fk[1][3], left_fk[2][3]]) / 1000.0
        right_pos_fk = np.array([right_fk[0][3], right_fk[1][3], right_fk[2][3]]) / 1000.0

        # Extract rotation matrix
        left_rot_fk = np.array([
            [left_fk[0][0], left_fk[0][1], left_fk[0][2]],
            [left_fk[1][0], left_fk[1][1], left_fk[1][2]],
            [left_fk[2][0], left_fk[2][1], left_fk[2][2]],
        ])
        right_rot_fk = np.array([
            [right_fk[0][0], right_fk[0][1], right_fk[0][2]],
            [right_fk[1][0], right_fk[1][1], right_fk[1][2]],
            [right_fk[2][0], right_fk[2][1], right_fk[2][2]],
        ])

        print("\n[4]FK computed results vs configuration values comparison")

        print("\n  Left arm position:")
        print(f"    FK computed:  [{left_pos_fk[0]:.4f}, {left_pos_fk[1]:.4f}, {left_pos_fk[2]:.4f}]")
        print(f"    Config value:  [{TIANJI_INIT_POS['left'][0]:.4f}, {TIANJI_INIT_POS['left'][1]:.4f}, {TIANJI_INIT_POS['left'][2]:.4f}]")
        pos_diff_left = np.linalg.norm(left_pos_fk - TIANJI_INIT_POS['left']) * 1000
        print(f"    Difference:    {pos_diff_left:.2f} mm")

        print("\n  Right arm position:")
        print(f"    FK computed:  [{right_pos_fk[0]:.4f}, {right_pos_fk[1]:.4f}, {right_pos_fk[2]:.4f}]")
        print(f"    Config value:  [{TIANJI_INIT_POS['right'][0]:.4f}, {TIANJI_INIT_POS['right'][1]:.4f}, {TIANJI_INIT_POS['right'][2]:.4f}]")
        pos_diff_right = np.linalg.norm(right_pos_fk - TIANJI_INIT_POS['right']) * 1000
        print(f"    Difference:    {pos_diff_right:.2f} mm")

        print("\n  Left arm orientation:")
        rot_diff_left = np.linalg.norm(left_rot_fk - TIANJI_INIT_ROT['left'])
        print(f"    Difference norm: {rot_diff_left:.6f}")

        print("\n  Right arm orientation:")
        rot_diff_right = np.linalg.norm(right_rot_fk - TIANJI_INIT_ROT['right'])
        print(f"    Difference norm: {rot_diff_right:.6f}")

        # Verification Results
        print("\n" + "=" * 70)
        if pos_diff_left < 1 and pos_diff_right < 1 and rot_diff_left < 0.01 and rot_diff_right < 0.01:
            print("Verification passed: FK computed values match configuration values")
        else:
            print("⚠️  Verification failed: FK computed values differ from configuration values")
            print("   Possible cause: different kinematics configuration file version")

        # If configuration needs updating, print new values
        print("\n[5]If updating tianji_robot.yaml, use the following values:")
        print(f"""
TIANJI_INIT_POS = {{
    'left': np.array([{left_pos_fk[0]:.4f}, {left_pos_fk[1]:.4f}, {left_pos_fk[2]:.4f}]),
    'right': np.array([{right_pos_fk[0]:.4f}, {right_pos_fk[1]:.4f}, {right_pos_fk[2]:.4f}]),
}}

TIANJI_INIT_ROT = {{
    'left': np.array([
        [{left_rot_fk[0,0]:.4f}, {left_rot_fk[0,1]:.4f}, {left_rot_fk[0,2]:.4f}],
        [{left_rot_fk[1,0]:.4f}, {left_rot_fk[1,1]:.4f}, {left_rot_fk[1,2]:.4f}],
        [{left_rot_fk[2,0]:.4f}, {left_rot_fk[2,1]:.4f}, {left_rot_fk[2,2]:.4f}],
    ]),
    'right': np.array([
        [{right_rot_fk[0,0]:.4f}, {right_rot_fk[0,1]:.4f}, {right_rot_fk[0,2]:.4f}],
        [{right_rot_fk[1,0]:.4f}, {right_rot_fk[1,1]:.4f}, {right_rot_fk[1,2]:.4f}],
        [{right_rot_fk[2,0]:.4f}, {right_rot_fk[2,1]:.4f}, {right_rot_fk[2,2]:.4f}],
    ]),
}}
""")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

    print("\n[Coordinate System Description]")
    print("""
  TIANJI_INIT_POS/ROT are values in Chest coordinate system, not World coordinate system!

  Coordinate system relationships:
    World (ROS REP 103)           Chest (robot arm base)
    +-------------------+           +-------------------+
    | X = Forward       |           | Left:  X=Forward Y=Down Z=Left
    | Y = Left          |  ----->>  | Right: X=Forward Y=Up Z=Right
    | Z = Up            |  Rotate   |
    +-------------------+  X +/-90  +-------------------+

  Conversion formulas:
    Left Chest  = World rotated around X axis by +90 degrees
    Right Chest = World rotated around X axis by -90 degrees
""")


if __name__ == '__main__':
    main()
