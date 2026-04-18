#!/usr/bin/env python3
"""
=============================================================================
Diagnostic Tool: Determine Correct zsp_para Values
=============================================================================

Functions:
  1. Initialize kinematics solver (no robot connection needed)
  2. FK(init_joints) -> compute initial pose matrix
  3. Iterate through multiple zsp_para candidate values (each tested in independent subprocess to prevent C library segfault)
  4. IK(pose_mat, zsp_para) -> compute joint angles
  5. Compare IK result with init_joints difference
  6. Find zsp_para that best reproduces init_joints

Usage:
  cd ~/Desktop/wuji-hand-teleop/src/input_devices/pico_input/test
  python3 tool/diagnose_zsp_para.py

Notes:
  This tool only uses the kinematics library (libKine.so), no real robot connection needed.
  It empirically determines the correct zsp_para through FK->IK closed-loop testing.

  Each candidate is tested in an independent subprocess to avoid C library internal state being
  corrupted by bad IK results causing subsequent calls to segfault.

  IPC uses temporary JSON files (not Queue), ensuring data is not lost after os._exit(0).

=============================================================================
"""

import sys
import os
import json
import time
import tempfile
import multiprocessing
import numpy as np
from pathlib import Path

# Add paths (using relative paths)
_tool_dir = Path(__file__).resolve().parent
_test_dir = str(_tool_dir.parent)  # tool → test
_src_dir = _tool_dir.parents[3]    # tool → test → pico_input → input_devices → src
_tianji_wo_dir = str(_src_dir / 'output_devices' / 'tianji_world_output')
for _p in [_tianji_wo_dir, _test_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _test_single_candidate(config_path, init_joints_left, init_joints_right,
                            fk_left, fk_right, left_zsp, right_zsp,
                            zsp_type, zsp_angle, dgr, result_file_path):
    """
    Test a single zsp_para candidate value in an independent subprocess.

    Even if C library segfaults, it only kills the subprocess without affecting the main process.
    Results are passed back via JSON temporary files (file I/O is synchronous, not dependent on background threads).

    Key: Uses os._exit(0) to force exit, skipping Python cleanup phase.
    C library (libKine.so) segfaults when Python GC collects ctypes objects,
    causing subprocess exitcode != 0, even if IK computation itself succeeded.
    """
    # Suppress C library stdout noise (LOADMvCfg, FX_Robot_Init etc.)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    os.dup2(devnull_fd, 1)
    os.dup2(devnull_fd, 2)

    try:
        # Each subprocess re-initializes kinematics solver (avoid sharing C library global state)
        from tianji_world_output.fx_kine import Marvin_Kine

        kine_left = Marvin_Kine()
        config_result = kine_left.load_config(config_path=config_path)
        time.sleep(0.1)
        kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result['TYPE'][0],
            dh=config_result['DH'][0],
            pnva=config_result['PNVA'][0],
            j67=config_result['BD'][0]
        )

        kine_right = Marvin_Kine()
        config_result2 = kine_right.load_config(config_path=config_path)
        time.sleep(0.1)
        kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result2['TYPE'][1],
            dh=config_result2['DH'][1],
            pnva=config_result2['PNVA'][1],
            j67=config_result2['BD'][1]
        )

        # Left arm IK
        left_ik = _run_ik(kine_left, 0, fk_left, init_joints_left,
                          left_zsp, zsp_type, zsp_angle, dgr)
        # Right arm IK
        right_ik = _run_ik(kine_right, 1, fk_right, init_joints_right,
                           right_zsp, zsp_type, zsp_angle, dgr)

        result = {
            'left_joints': left_ik,
            'right_joints': right_ik,
            'error': None,
        }
    except Exception as e:
        result = {
            'left_joints': None,
            'right_joints': None,
            'error': str(e),
        }

    # Write results to JSON file (synchronous I/O, persisted upon completion)
    try:
        with open(result_file_path, 'w') as f:
            json.dump(result, f)
            f.flush()
            os.fsync(f.fileno())
    except Exception:
        pass

    # Restore stdout/stderr
    os.dup2(old_stdout, 1)
    os.dup2(old_stderr, 2)
    os.close(devnull_fd)
    os.close(old_stdout)
    os.close(old_stderr)

    # Force exit, skip Python cleanup (avoid ctypes GC triggering C library segfault)
    os._exit(0)


def _run_ik(kine, serial, pose_mat, ref_joints, zsp_para,
            zsp_type=1, zsp_angle=0.0, dgr=None):
    """Run inverse kinematics"""
    if dgr is None:
        dgr = [5.0, 5.0, 5.0]

    ik_result = kine.ik(
        robot_serial=serial,
        pose_mat=pose_mat,
        ref_joints=ref_joints,
        zsp_type=zsp_type,
        zsp_para=zsp_para,
        zsp_angle=zsp_angle,
        dgr=dgr
    )

    if ik_result is False:
        return None
    if ik_result.m_Output_IsOutRange or ik_result.m_Output_IsJntExd:
        return None

    joints = ik_result.m_Output_RetJoint
    if hasattr(joints, 'to_list'):
        return joints.to_list()
    return list(joints)


def test_candidate_safe(config_path, init_joints, fk_mats,
                        left_zsp, right_zsp, zsp_type=1,
                        zsp_angle=0.0, dgr=None, timeout=10):
    """
    Safely test a candidate value in subprocess (prevents segfault)

    Uses temporary JSON files to pass results, avoiding Queue feeder thread being killed by os._exit.

    Returns:
        dict with 'left_joints', 'right_joints', 'status'
        status: 'ok', 'ik_fail', 'crash', 'timeout', 'error'
    """
    if dgr is None:
        dgr = [5.0, 5.0, 5.0]

    # Create temporary result file
    fd, result_file_path = tempfile.mkstemp(suffix='.json', prefix='zsp_diag_')
    os.close(fd)  # Close fd, let subprocess open and write itself

    proc = multiprocessing.Process(
        target=_test_single_candidate,
        args=(config_path,
              init_joints['left'], init_joints['right'],
              fk_mats['left'], fk_mats['right'],
              left_zsp, right_zsp,
              zsp_type, zsp_angle, dgr,
              result_file_path),
        daemon=True,
    )
    proc.start()
    proc.join(timeout=timeout)

    if proc.is_alive():
        proc.terminate()
        proc.join(timeout=2)
        _cleanup_file(result_file_path)
        return {'left_joints': None, 'right_joints': None, 'status': 'timeout'}

    # Read result file (even if exitcode != 0, file may have been written)
    result = None
    try:
        with open(result_file_path, 'r') as f:
            content = f.read().strip()
            if content:
                result = json.loads(content)
    except Exception:
        pass
    finally:
        _cleanup_file(result_file_path)

    if result is None:
        # File empty or not found: real crash (segfault during IK computation)
        return {'left_joints': None, 'right_joints': None, 'status': 'crash'}

    if result.get('error'):
        return {'left_joints': None, 'right_joints': None, 'status': 'error'}

    if result['left_joints'] is None or result['right_joints'] is None:
        return {**result, 'status': 'ik_fail'}

    return {**result, 'status': 'ok'}


def _cleanup_file(path):
    """Safely delete temporary file"""
    try:
        os.unlink(path)
    except OSError:
        pass


def joints_error(j1, j2):
    """Compute difference between two sets of joint angles"""
    diff = [abs(j1[i] - j2[i]) for i in range(7)]
    return max(diff), sum(diff) / 7, diff


def init_kine_and_fk(config_path, init_joints):
    """Initialize solver in main process and compute FK (done once only)"""
    from tianji_world_output.fx_kine import Marvin_Kine

    kine_left = Marvin_Kine()
    config_result = kine_left.load_config(config_path=config_path)
    time.sleep(0.3)
    kine_left.initial_kine(
        robot_serial=0,
        robot_type=config_result['TYPE'][0],
        dh=config_result['DH'][0],
        pnva=config_result['PNVA'][0],
        j67=config_result['BD'][0]
    )

    kine_right = Marvin_Kine()
    config_result2 = kine_right.load_config(config_path=config_path)
    time.sleep(0.3)
    kine_right.initial_kine(
        robot_serial=1,
        robot_type=config_result2['TYPE'][1],
        dh=config_result2['DH'][1],
        pnva=config_result2['PNVA'][1],
        j67=config_result2['BD'][1]
    )

    # Print gravity direction
    grv_left = config_result['GRV'][0]
    grv_right = config_result['GRV'][1]
    print(f"\n  left arm GRV: {grv_left}")
    print(f"  right arm GRV: {grv_right}")
    print("  → Left arm gravity=+Y, right arm gravity=-Y (DH base coordinate system)")

    # FK
    fk_left = kine_left.fk(robot_serial=0, joints=init_joints['left'])
    fk_right = kine_right.fk(robot_serial=1, joints=init_joints['right'])
    if fk_left is False or fk_right is False:
        raise RuntimeError("FK failed! Please check init_joints")

    return {'left': fk_left, 'right': fk_right}, config_result


def main():
    from tianji_world_output.config_loader import TianjiConfig

    print("=" * 70)
    print("  zsp_para Diagnostic Tool - FK->IK Closed-Loop Test (subprocess isolation version)")
    print("=" * 70)

    # Load configuration
    config = TianjiConfig.load(use_ros=False)
    config_path = config.get_kine_config_path()
    print(f"\nKinematics configuration: {config_path}")

    init_joints = {
        'left': config.init_joints['left'].tolist(),
        'right': config.init_joints['right'].tolist(),
    }
    zsp_angle = float(config.zsp_angle)
    dgr = list(config.dgr)

    print(f"\n  left arm init_joints: {init_joints['left']}")
    print(f"  right arm init_joints: {init_joints['right']}")
    print(f"  zsp_angle: {zsp_angle}, dgr: {dgr}")

    # Step 1: FK
    print("\n" + "=" * 70)
    print("  Step 1: FK(init_joints) -> Initial Pose Matrix")
    print("=" * 70)

    print("\nInitializing kinematics solver...")
    fk_mats, raw_config = init_kine_and_fk(config_path, init_joints)
    print("\n  Left arm FK position (mm): [{:.2f}, {:.2f}, {:.2f}]".format(
        fk_mats['left'][0][3], fk_mats['left'][1][3], fk_mats['left'][2][3]))
    print("  Right arm FK position (mm): [{:.2f}, {:.2f}, {:.2f}]".format(
        fk_mats['right'][0][3], fk_mats['right'][1][3], fk_mats['right'][2][3]))

    # Step 2: Coarse search - classic candidates
    print("\n" + "=" * 70)
    print("  Step 2: Coarse Search - Classic zsp_para Candidates")
    print("=" * 70)

    coarse_candidates = [
        # (name, left_zsp, right_zsp)
        # Current configuration
        ("Current: [0,-1,-0.5]/[0,+1,-0.5]", [0,-1,-0.5,0,0,0], [0,1,-0.5,0,0,0]),
        # Pure anti-gravity
        ("Pure Y: [0,-1,0]/[0,+1,0]",        [0,-1,0,0,0,0],    [0,1,0,0,0,0]),
        # Anti-gravity + Z variants
        ("Y-Z large: [0,-1,-1]/[0,+1,-1]",    [0,-1,-1,0,0,0],   [0,1,-1,0,0,0]),
        ("Y-Z small: [0,-1,-0.3]/[0,+1,-0.3]",[0,-1,-0.3,0,0,0], [0,1,-0.3,0,0,0]),
        ("Y-Z positive: [0,-1,+0.5]/[0,+1,+0.5]",[0,-1,0.5,0,0,0],  [0,1,0.5,0,0,0]),
        # Pure Z
        ("Pure Z-: [0,0,-1]/[0,0,-1]",       [0,0,-1,0,0,0],    [0,0,-1,0,0,0]),
        # Add X component
        ("X+Y: [0.3,-1,-0.5]/[0.3,+1,-0.5]", [0.3,-1,-0.5,0,0,0], [0.3,1,-0.5,0,0,0]),
        ("X-Y: [-0.3,-1,-0.5]/[-0.3,+1,-0.5]", [-0.3,-1,-0.5,0,0,0], [-0.3,1,-0.5,0,0,0]),
        # Y inverted (control group - should be poor)
        ("Inverted Y: [0,+1,-0.5]/[0,-1,-0.5]", [0,1,-0.5,0,0,0],  [0,-1,-0.5,0,0,0]),
    ]

    print(f"\n  {'Candidate':<42} | {'L_max':>6} | {'R_max':>6} | {'Avg':>6} | Status")
    print("  " + "-" * 80)

    coarse_results = []
    for name, left_zsp, right_zsp in coarse_candidates:
        result = test_candidate_safe(
            config_path, init_joints, fk_mats,
            left_zsp, right_zsp,
            zsp_type=1, zsp_angle=zsp_angle, dgr=dgr
        )

        if result['status'] != 'ok':
            print(f"  {name:<42} | {'--':>6} | {'--':>6} | {'--':>6} | {result['status']}")
            coarse_results.append((name, left_zsp, right_zsp, None, None, float('inf')))
            continue

        l_max, l_avg, _ = joints_error(init_joints['left'], result['left_joints'])
        r_max, r_avg, _ = joints_error(init_joints['right'], result['right_joints'])
        total_avg = (l_avg + r_avg) / 2

        marker = " <<< BEST" if l_max < 1 and r_max < 1 else ""
        print(f"  {name:<42} | {l_max:>5.1f}° | {r_max:>5.1f}° | {total_avg:>5.2f}° | ok{marker}")
        coarse_results.append((name, left_zsp, right_zsp, l_max, r_max, total_avg))

    # zsp_type=0 baseline
    print()
    result_t0 = test_candidate_safe(
        config_path, init_joints, fk_mats,
        [0,0,0,0,0,0], [0,0,0,0,0,0],
        zsp_type=0, zsp_angle=zsp_angle, dgr=dgr
    )
    if result_t0['status'] == 'ok':
        l_max_t0, l_avg_t0, _ = joints_error(init_joints['left'], result_t0['left_joints'])
        r_max_t0, r_avg_t0, _ = joints_error(init_joints['right'], result_t0['right_joints'])
        total_t0 = (l_avg_t0 + r_avg_t0) / 2
        print(f"  {'zsp_type=0 (Baseline reference)':<42} | {l_max_t0:>5.1f}° | {r_max_t0:>5.1f}° | {total_t0:>5.2f}° | ok")
    else:
        print(f"  {'zsp_type=0 (Baseline reference)':<42} | {'--':>6} | {'--':>6} | {'--':>6} | {result_t0['status']}")

    # Step 3: Fine search - grid search near best coarse candidate
    print("\n" + "=" * 70)
    print("  Step 3: Fine Search - Y/Z Parameter Grid (left arm zsp_para)")
    print("=" * 70)

    y_values = [-1.5, -1.2, -1.0, -0.8, -0.5, -0.3]
    z_values = [-1.0, -0.8, -0.5, -0.3, 0.0, 0.3, 0.5]

    print(f"\n  Searching {len(y_values)}x{len(z_values)} = {len(y_values)*len(z_values)} combinations...")
    print("  (Left arm=[0,Y,Z], right arm=[0,-Y,Z], Y negated following GRV symmetry)")
    print()

    best_fine = None
    best_fine_err = float('inf')
    fine_results = []

    # Table header
    yz_label = "Y\\Z"
    header = f"  {yz_label:>5}"
    for z in z_values:
        header += f" | {z:>+5.1f}"
    print(header)
    print("  " + "-" * (7 + 8 * len(z_values)))

    for y in y_values:
        row = f"  {y:>+5.1f}"
        for z in z_values:
            left_zsp = [0, y, z, 0, 0, 0]
            right_zsp = [0, -y, z, 0, 0, 0]

            result = test_candidate_safe(
                config_path, init_joints, fk_mats,
                left_zsp, right_zsp,
                zsp_type=1, zsp_angle=zsp_angle, dgr=dgr
            )

            if result['status'] != 'ok':
                row += f" | {'--':>5}"
                continue

            l_max, l_avg, _ = joints_error(init_joints['left'], result['left_joints'])
            r_max, r_avg, _ = joints_error(init_joints['right'], result['right_joints'])
            total_max = max(l_max, r_max)
            total_avg = (l_avg + r_avg) / 2

            fine_results.append((y, z, l_max, r_max, total_avg, result))

            if total_avg < best_fine_err:
                best_fine_err = total_avg
                best_fine = (y, z, l_max, r_max, total_avg, result)

            # Display max error, mark best
            marker = "*" if total_max < 1 else ""
            row += f" | {total_max:>4.1f}°{marker}"

        print(row)

    # Step 4: Best result details
    print("\n" + "=" * 70)
    print("  Step 4: Best Candidate Details")
    print("=" * 70)

    if best_fine:
        y, z, l_max, r_max, avg, result = best_fine
        print(f"\n  Best: left=[0, {y}, {z}] / Right=[0, {-y}, {z}]")
        print(f"  left arm max_err={l_max:.3f}°, right arm max_err={r_max:.3f}°, Avg={avg:.3f}°")

        left_ik = result['left_joints']
        right_ik = result['right_joints']

        print("\n  Left arm joint angle comparison (degrees):")
        print(f"    {'Joint':<6} {'init':>10} {'IK Result':>10} {'Diff':>8}")
        for i in range(7):
            d = left_ik[i] - init_joints['left'][i]
            print(f"    J{i+1:<5} {init_joints['left'][i]:>10.3f} {left_ik[i]:>10.3f} {d:>+8.3f}")

        print("\n  Right arm joint angle comparison (degrees):")
        print(f"    {'Joint':<6} {'init':>10} {'IK Result':>10} {'Diff':>8}")
        for i in range(7):
            d = right_ik[i] - init_joints['right'][i]
            print(f"    J{i+1:<5} {init_joints['right'][i]:>10.3f} {right_ik[i]:>10.3f} {d:>+8.3f}")

        print("\n  Recommended to write in tianji_robot.yaml:")
        print("    default_zsp_para:")
        print(f"      left:  [0, {y}, {z}, 0, 0, 0]")
        print(f"      right: [0, {-y}, {z}, 0, 0, 0]")
    else:
        print("\n  No usable zsp_para combination found!")
        print("  Please check if init_joints is valid")

    # Top 5
    fine_results.sort(key=lambda x: x[4])  # sort by avg error
    print("\n  Top 5 Best combinations:")
    print(f"    {'Rank':>4} {'Y':>5} {'Z':>5} | {'L_max':>6} {'R_max':>6} {'Avg':>6}")
    for i, (y, z, lm, rm, avg, _) in enumerate(fine_results[:5]):
        print(f"    #{i+1:<3} {y:>+5.1f} {z:>+5.1f} | {lm:>5.2f}° {rm:>5.2f}° {avg:>5.3f}°")

    # Step 5: Left/Right Arm Independent Search (Verify GRV Symmetry)
    print("\n" + "=" * 70)
    print("  Step 5: Left/Right Arm Independent Search (Verify GRV Symmetry)")
    print("=" * 70)
    print("\n  Both arms use the *same* zsp_para=[0,Y,Z], observe respective errors.")
    print("  Expected: left arm optimal Y<0 (anti GRV +Y), right arm optimal Y>0 (anti GRV -Y)")
    print("  If optimal Y values are negatives of each other, the symmetry assumption is confirmed.\n")

    indep_y = [-1.5, -1.0, -0.8, -0.5, -0.3, 0.0, 0.3, 0.5, 0.8, 1.0, 1.5]
    indep_z = [-0.5, -0.3, 0.0]

    # Collect data
    indep_data = {}  # (y, z) -> (l_max, r_max) or None
    best_left = (None, None, float('inf'))   # (y, z, err)
    best_right = (None, None, float('inf'))

    for z in indep_z:
        for y in indep_y:
            zsp = [0, y, z, 0, 0, 0]
            result = test_candidate_safe(
                config_path, init_joints, fk_mats,
                zsp, zsp,  # Both arms use same zsp_para
                zsp_type=1, zsp_angle=zsp_angle, dgr=dgr
            )

            l_max = r_max = None
            if result.get('left_joints'):
                l_max, _, _ = joints_error(init_joints['left'], result['left_joints'])
                if l_max < best_left[2]:
                    best_left = (y, z, l_max)
            if result.get('right_joints'):
                r_max, _, _ = joints_error(init_joints['right'], result['right_joints'])
                if r_max < best_right[2]:
                    best_right = (y, z, r_max)

            indep_data[(y, z)] = (l_max, r_max)

    # Left arm table
    yz_label = "Y\\Z"
    print("  Left arm (A) max_err (degrees):")
    header = f"    {yz_label:>5}"
    for z in indep_z:
        header += f" | {z:>+5.1f}"
    print(header)
    print("    " + "-" * (7 + 8 * len(indep_z)))
    for y in indep_y:
        row = f"    {y:>+5.1f}"
        for z in indep_z:
            l_max, _ = indep_data.get((y, z), (None, None))
            if l_max is not None:
                marker = "*" if l_max < 1 else ""
                row += f" | {l_max:>4.1f}°{marker}"
            else:
                row += f" | {'--':>5}"
        print(row)
    if best_left[0] is not None:
        print(f"    → Left arm independent optimal: Y={best_left[0]:+.1f}, Z={best_left[1]:+.1f}, max_err={best_left[2]:.2f}°")

    # Right arm table
    print("\n  Right arm (B) max_err (degrees):")
    print(header)
    print("    " + "-" * (7 + 8 * len(indep_z)))
    for y in indep_y:
        row = f"    {y:>+5.1f}"
        for z in indep_z:
            _, r_max = indep_data.get((y, z), (None, None))
            if r_max is not None:
                marker = "*" if r_max < 1 else ""
                row += f" | {r_max:>4.1f}°{marker}"
            else:
                row += f" | {'--':>5}"
        print(row)
    if best_right[0] is not None:
        print(f"    → Right arm independent optimal: Y={best_right[0]:+.1f}, Z={best_right[1]:+.1f}, max_err={best_right[2]:.2f}°")

    # Symmetry verification
    print("\n  Symmetry verification:")
    if best_left[0] is not None and best_right[0] is not None:
        y_sum = best_left[0] + best_right[0]
        z_diff = abs(best_left[1] - best_right[1])
        sym_ok = abs(y_sum) < 0.3 and z_diff < 0.3
        print(f"    Left arm optimal Y={best_left[0]:+.1f}, Right arm optimal Y={best_right[0]:+.1f}, Ysum={y_sum:+.2f}")
        print(f"    Left arm optimal Z={best_left[1]:+.1f}, Right arm optimal Z={best_right[1]:+.1f}, Zdifference={z_diff:.2f}")
        print(f"    {'✓ GRV symmetry confirmed!' if sym_ok else '✗ Symmetry deviation, please check'}")
    else:
        print("    Cannot verify (one arm has no valid results)")

    print("\n" + "=" * 70)
    print("  Diagnosis complete!")
    print("=" * 70)
    print("\nDescription:")
    print("  * Marker: that arm max_err < 1 deg (perfect match)")
    print("  Step 3 table: max(L_max, R_max) (symmetric search)")
    print("  Step 5 table: individual arm errors (same zsp_para)")
    print("  Y component: left arm optimal should be negative (anti GRV +Y), right arm optimal should be positive (anti GRV -Y)")
    print("  Segfault/crash candidates shown as '--' (isolated by subprocess)")


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn', force=True)
    main()
