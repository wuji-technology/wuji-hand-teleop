#!/usr/bin/env python3
"""
=============================================================================
诊断工具: 确定正确的 zsp_para 值
=============================================================================

功能：
  1. 初始化运动学求解器（不需要机器人连接）
  2. FK(init_joints) → 计算初始位姿矩阵
  3. 遍历多种 zsp_para 候选值（每个在独立子进程中测试，防止 C 库 segfault）
  4. IK(pose_mat, zsp_para) → 计算关节角
  5. 对比 IK 结果与 init_joints 的差异
  6. 找出能最好重现 init_joints 的 zsp_para

使用：
  cd ~/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/test
  python3 tool/diagnose_zsp_para.py

说明：
  本工具只使用运动学库（libKine.so），不需要连接真实机器人。
  它通过 FK→IK 闭环测试，经验性地确定正确的 zsp_para。

  每个候选值在独立子进程中测试，避免 C 库内部状态被坏的 IK 结果
  破坏后导致后续调用 segfault。

  IPC 使用临时 JSON 文件（非 Queue），确保 os._exit(0) 后数据不丢失。

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

# 添加路径（使用相对路径）
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
    在独立子进程中测试单个 zsp_para 候选值。

    即使 C 库 segfault，也只会杀死子进程，不影响主进程。
    结果通过 JSON 临时文件传回（文件 I/O 是同步的，不依赖后台线程）。

    关键: 使用 os._exit(0) 强制退出，跳过 Python 清理阶段。
    C 库 (libKine.so) 在 Python GC 回收 ctypes 对象时会 segfault，
    导致子进程 exitcode != 0，即使 IK 计算本身成功。
    """
    # 抑制 C 库 stdout 噪音 (LOADMvCfg, FX_Robot_Init 等)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    os.dup2(devnull_fd, 1)
    os.dup2(devnull_fd, 2)

    try:
        # 每个子进程重新初始化运动学求解器 (避免共享 C 库全局状态)
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

        # 左臂 IK
        left_ik = _run_ik(kine_left, 0, fk_left, init_joints_left,
                          left_zsp, zsp_type, zsp_angle, dgr)
        # 右臂 IK
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

    # 将结果写入 JSON 文件 (同步 I/O，写完即持久化)
    try:
        with open(result_file_path, 'w') as f:
            json.dump(result, f)
            f.flush()
            os.fsync(f.fileno())
    except Exception:
        pass

    # 恢复 stdout/stderr
    os.dup2(old_stdout, 1)
    os.dup2(old_stderr, 2)
    os.close(devnull_fd)
    os.close(old_stdout)
    os.close(old_stderr)

    # 强制退出，跳过 Python 清理 (避免 ctypes GC 触发 C 库 segfault)
    os._exit(0)


def _run_ik(kine, serial, pose_mat, ref_joints, zsp_para,
            zsp_type=1, zsp_angle=0.0, dgr=None):
    """运行逆运动学"""
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
    在子进程中安全测试一个候选值 (防 segfault)

    使用临时 JSON 文件传递结果，避免 Queue feeder thread 被 os._exit 杀死的问题。

    Returns:
        dict with 'left_joints', 'right_joints', 'status'
        status: 'ok', 'ik_fail', 'crash', 'timeout', 'error'
    """
    if dgr is None:
        dgr = [5.0, 5.0, 5.0]

    # 创建临时结果文件
    fd, result_file_path = tempfile.mkstemp(suffix='.json', prefix='zsp_diag_')
    os.close(fd)  # 关闭 fd，让子进程自己打开写入

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

    # 读取结果文件 (即使 exitcode != 0，文件可能已写入)
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
        # 文件为空或不存在: 真正的崩溃 (IK 计算期间 segfault)
        return {'left_joints': None, 'right_joints': None, 'status': 'crash'}

    if result.get('error'):
        return {'left_joints': None, 'right_joints': None, 'status': 'error'}

    if result['left_joints'] is None or result['right_joints'] is None:
        return {**result, 'status': 'ik_fail'}

    return {**result, 'status': 'ok'}


def _cleanup_file(path):
    """安全删除临时文件"""
    try:
        os.unlink(path)
    except OSError:
        pass


def joints_error(j1, j2):
    """计算两组关节角的差异"""
    diff = [abs(j1[i] - j2[i]) for i in range(7)]
    return max(diff), sum(diff) / 7, diff


def init_kine_and_fk(config_path, init_joints):
    """在主进程中初始化求解器并计算 FK (只做一次)"""
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

    # 打印重力方向
    grv_left = config_result['GRV'][0]
    grv_right = config_result['GRV'][1]
    print(f"\n  左臂 GRV: {grv_left}")
    print(f"  右臂 GRV: {grv_right}")
    print(f"  → 左臂重力=+Y, 右臂重力=-Y (DH 基坐标系)")

    # FK
    fk_left = kine_left.fk(robot_serial=0, joints=init_joints['left'])
    fk_right = kine_right.fk(robot_serial=1, joints=init_joints['right'])
    if fk_left is False or fk_right is False:
        raise RuntimeError("FK 失败! 请检查 init_joints")

    return {'left': fk_left, 'right': fk_right}, config_result


def main():
    from tianji_world_output.config_loader import TianjiConfig

    print("=" * 70)
    print("  zsp_para 诊断工具 - FK→IK 闭环测试 (子进程隔离版)")
    print("=" * 70)

    # 加载配置
    config = TianjiConfig.load(use_ros=False)
    config_path = config.get_kine_config_path()
    print(f"\n运动学配置: {config_path}")

    init_joints = {
        'left': config.init_joints['left'].tolist(),
        'right': config.init_joints['right'].tolist(),
    }
    zsp_angle = float(config.zsp_angle)
    dgr = list(config.dgr)

    print(f"\n  左臂 init_joints: {init_joints['left']}")
    print(f"  右臂 init_joints: {init_joints['right']}")
    print(f"  zsp_angle: {zsp_angle}, dgr: {dgr}")

    # Step 1: FK
    print("\n" + "=" * 70)
    print("  Step 1: FK(init_joints) → 初始位姿矩阵")
    print("=" * 70)

    print("\n初始化运动学求解器...")
    fk_mats, raw_config = init_kine_and_fk(config_path, init_joints)
    print("\n  左臂 FK 位置 (mm): [{:.2f}, {:.2f}, {:.2f}]".format(
        fk_mats['left'][0][3], fk_mats['left'][1][3], fk_mats['left'][2][3]))
    print("  右臂 FK 位置 (mm): [{:.2f}, {:.2f}, {:.2f}]".format(
        fk_mats['right'][0][3], fk_mats['right'][1][3], fk_mats['right'][2][3]))

    # Step 2: 粗搜索 — 经典候选值
    print("\n" + "=" * 70)
    print("  Step 2: 粗搜索 — 经典 zsp_para 候选值")
    print("=" * 70)

    coarse_candidates = [
        # (名称, left_zsp, right_zsp)
        # 当前配置
        ("当前: [0,-1,-0.5]/[0,+1,-0.5]", [0,-1,-0.5,0,0,0], [0,1,-0.5,0,0,0]),
        # 纯反重力
        ("纯Y: [0,-1,0]/[0,+1,0]",        [0,-1,0,0,0,0],    [0,1,0,0,0,0]),
        # 反重力 + Z变体
        ("Y-Z大: [0,-1,-1]/[0,+1,-1]",    [0,-1,-1,0,0,0],   [0,1,-1,0,0,0]),
        ("Y-Z小: [0,-1,-0.3]/[0,+1,-0.3]",[0,-1,-0.3,0,0,0], [0,1,-0.3,0,0,0]),
        ("Y-Z正: [0,-1,+0.5]/[0,+1,+0.5]",[0,-1,0.5,0,0,0],  [0,1,0.5,0,0,0]),
        # 纯 Z
        ("纯Z-: [0,0,-1]/[0,0,-1]",       [0,0,-1,0,0,0],    [0,0,-1,0,0,0]),
        # 加 X 分量
        ("X+Y: [0.3,-1,-0.5]/[0.3,+1,-0.5]", [0.3,-1,-0.5,0,0,0], [0.3,1,-0.5,0,0,0]),
        ("X-Y: [-0.3,-1,-0.5]/[-0.3,+1,-0.5]", [-0.3,-1,-0.5,0,0,0], [-0.3,1,-0.5,0,0,0]),
        # Y 反向 (对照组 — 应该很差)
        ("反向Y: [0,+1,-0.5]/[0,-1,-0.5]", [0,1,-0.5,0,0,0],  [0,-1,-0.5,0,0,0]),
    ]

    print(f"\n  {'候选值':<42} | {'左max':>6} | {'右max':>6} | {'平均':>6} | 状态")
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

    # zsp_type=0 基线
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
        print(f"  {'zsp_type=0 (基线参考)':<42} | {l_max_t0:>5.1f}° | {r_max_t0:>5.1f}° | {total_t0:>5.2f}° | ok")
    else:
        print(f"  {'zsp_type=0 (基线参考)':<42} | {'--':>6} | {'--':>6} | {'--':>6} | {result_t0['status']}")

    # Step 3: 细搜索 — 在最佳粗候选附近网格搜索
    print("\n" + "=" * 70)
    print("  Step 3: 细搜索 — Y/Z 参数网格 (左臂 zsp_para)")
    print("=" * 70)

    y_values = [-1.5, -1.2, -1.0, -0.8, -0.5, -0.3]
    z_values = [-1.0, -0.8, -0.5, -0.3, 0.0, 0.3, 0.5]

    print(f"\n  搜索 {len(y_values)}x{len(z_values)} = {len(y_values)*len(z_values)} 组合...")
    print(f"  (左臂=[0,Y,Z], 右臂=[0,-Y,Z]，Y 取反遵循 GRV 对称)")
    print()

    best_fine = None
    best_fine_err = float('inf')
    fine_results = []

    # 表头
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

            # 用 max error 显示，标记最佳
            marker = "*" if total_max < 1 else ""
            row += f" | {total_max:>4.1f}°{marker}"

        print(row)

    # Step 4: 最佳结果详情
    print("\n" + "=" * 70)
    print("  Step 4: 最佳候选详情")
    print("=" * 70)

    if best_fine:
        y, z, l_max, r_max, avg, result = best_fine
        print(f"\n  最佳: 左=[0, {y}, {z}] / 右=[0, {-y}, {z}]")
        print(f"  左臂 max_err={l_max:.3f}°, 右臂 max_err={r_max:.3f}°, 平均={avg:.3f}°")

        left_ik = result['left_joints']
        right_ik = result['right_joints']

        print(f"\n  左臂关节角对比 (度):")
        print(f"    {'关节':<6} {'init':>10} {'IK结果':>10} {'差异':>8}")
        for i in range(7):
            d = left_ik[i] - init_joints['left'][i]
            print(f"    J{i+1:<5} {init_joints['left'][i]:>10.3f} {left_ik[i]:>10.3f} {d:>+8.3f}")

        print(f"\n  右臂关节角对比 (度):")
        print(f"    {'关节':<6} {'init':>10} {'IK结果':>10} {'差异':>8}")
        for i in range(7):
            d = right_ik[i] - init_joints['right'][i]
            print(f"    J{i+1:<5} {init_joints['right'][i]:>10.3f} {right_ik[i]:>10.3f} {d:>+8.3f}")

        print(f"\n  推荐写入 tianji_robot.yaml:")
        print(f"    default_zsp_para:")
        print(f"      left:  [0, {y}, {z}, 0, 0, 0]")
        print(f"      right: [0, {-y}, {z}, 0, 0, 0]")
    else:
        print("\n  未找到可用的 zsp_para 组合!")
        print("  请检查 init_joints 是否有效")

    # Top 5
    fine_results.sort(key=lambda x: x[4])  # sort by avg error
    print(f"\n  Top 5 最佳组合:")
    print(f"    {'排名':>4} {'Y':>5} {'Z':>5} | {'左max':>6} {'右max':>6} {'平均':>6}")
    for i, (y, z, lm, rm, avg, _) in enumerate(fine_results[:5]):
        print(f"    #{i+1:<3} {y:>+5.1f} {z:>+5.1f} | {lm:>5.2f}° {rm:>5.2f}° {avg:>5.3f}°")

    # Step 5: 左右臂独立搜索 (验证 GRV 对称性)
    print("\n" + "=" * 70)
    print("  Step 5: 左右臂独立搜索 (验证 GRV 对称性)")
    print("=" * 70)
    print("\n  两臂使用 *相同* zsp_para=[0,Y,Z]，分别观察各自误差。")
    print("  预期: 左臂最优 Y<0 (反 GRV +Y)，右臂最优 Y>0 (反 GRV -Y)")
    print("  如果最优 Y 值互为相反数，则证明对称假设正确。\n")

    indep_y = [-1.5, -1.0, -0.8, -0.5, -0.3, 0.0, 0.3, 0.5, 0.8, 1.0, 1.5]
    indep_z = [-0.5, -0.3, 0.0]

    # 收集数据
    indep_data = {}  # (y, z) -> (l_max, r_max) or None
    best_left = (None, None, float('inf'))   # (y, z, err)
    best_right = (None, None, float('inf'))

    for z in indep_z:
        for y in indep_y:
            zsp = [0, y, z, 0, 0, 0]
            result = test_candidate_safe(
                config_path, init_joints, fk_mats,
                zsp, zsp,  # 两臂用相同 zsp_para
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

    # 左臂表
    yz_label = "Y\\Z"
    print(f"  左臂 (A) max_err (度):")
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
        print(f"    → 左臂独立最优: Y={best_left[0]:+.1f}, Z={best_left[1]:+.1f}, max_err={best_left[2]:.2f}°")

    # 右臂表
    print(f"\n  右臂 (B) max_err (度):")
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
        print(f"    → 右臂独立最优: Y={best_right[0]:+.1f}, Z={best_right[1]:+.1f}, max_err={best_right[2]:.2f}°")

    # 对称性验证
    print(f"\n  对称性验证:")
    if best_left[0] is not None and best_right[0] is not None:
        y_sum = best_left[0] + best_right[0]
        z_diff = abs(best_left[1] - best_right[1])
        sym_ok = abs(y_sum) < 0.3 and z_diff < 0.3
        print(f"    左臂最优 Y={best_left[0]:+.1f}, 右臂最优 Y={best_right[0]:+.1f}, Y之和={y_sum:+.2f}")
        print(f"    左臂最优 Z={best_left[1]:+.1f}, 右臂最优 Z={best_right[1]:+.1f}, Z之差={z_diff:.2f}")
        print(f"    {'✓ GRV 对称性成立!' if sym_ok else '✗ 对称性偏离，请检查'}")
    else:
        print("    无法验证 (某臂无有效结果)")

    print("\n" + "=" * 70)
    print("  诊断完成!")
    print("=" * 70)
    print("\n说明:")
    print("  * 标记: 该臂 max_err < 1° (完美匹配)")
    print("  Step 3 表格: max(左max, 右max) (对称搜索)")
    print("  Step 5 表格: 各臂独立误差 (相同 zsp_para)")
    print("  Y 分量: 左臂最优应为负 (反 GRV +Y)，右臂最优应为正 (反 GRV -Y)")
    print("  segfault/crash 的候选值显示为 '--' (已被子进程隔离)")


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn', force=True)
    main()
