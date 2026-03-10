#!/usr/bin/env python3
"""
Step 6: 机器人稳定性测试（模拟 Tracking 数据）

================================================================================
功能概述
================================================================================

三种测试模式：
  1. arm_angle: 臂角测试 - 固定末端位置，测试不同肘部方向
  2. position:  位置测试 - 固定臂角，测试末端前后/左右/上下移动
  3. rotation:  旋转测试 - 固定位置，测试末端绕各轴旋转

================================================================================
测试模式说明
================================================================================

【模式1: 臂角测试 (arm_angle)】
  - 末端位姿固定在初始位置
  - 改变 zsp_para (elbow direction)
  - 测试 IK 在不同臂角下的稳定性

  参数:
    --pitch-range MIN MAX STEP  俯仰角范围（肘部前后偏移）
    --yaw-range MIN MAX STEP    外展角范围（肘部向下/向外比例）

  角度定义:
    Pitch: 正=肘部向前, 负=肘部向后, 0=不前后偏
    Yaw:   0°=纯向下, 45°=默认沉肘, 90°=纯向外

【模式2: 位置测试 (position)】
  - 臂角固定为默认沉肘 (yaw=45°)
  - 末端位置在初始位置基础上偏移
  - 测试 IK 在不同末端位置下的稳定性
  - 位置偏移使用 Robot World 坐标系（两臂接口统一，内部自动转换到 Chest）

  参数:
    --pos-x-range MIN MAX STEP  X方向(前后)偏移范围 (米)
    --pos-y-range MIN MAX STEP  Y方向(左右)偏移范围 (米)
    --pos-z-range MIN MAX STEP  Z方向(上下)偏移范围 (米)

【模式3: 旋转测试 (rotation)】
  - 位置固定在初始位置
  - 末端姿态在初始姿态基础上旋转
  - 用于验证旋转变换的正确性

  参数:
    --rot-axis X/Y/Z/-X/-Y/-Z  旋转轴（Robot World 坐标系）
    --rot-angle DEGREES        旋转角度（度）

  Robot World 坐标系旋转效果（ROS REP 103 右手定则，从轴正端向原点看逆时针为正）:
    绕 +X 轴: 手腕向右翻滚 (Roll right: 顶部向右倾)
    绕 -X 轴: 手腕向左翻滚 (Roll left: 顶部向左倾)
    绕 +Y 轴: 手指向下压 (Pitch down: 低头)
    绕 -Y 轴: 手指向上翘 (Pitch up: 抬头)
    绕 +Z 轴: 手指向左偏 (Yaw left: 从上看逆时针)
    绕 -Z 轴: 手指向右偏 (Yaw right: 从上看顺时针)

================================================================================
使用示例
================================================================================

【臂角测试】
  # 干运行（仅打印）
  python3 step6_arm_angle_stability_test.py --dry-run

  # 测试左臂，默认范围
  python3 step6_arm_angle_stability_test.py --left-only

  # 大范围臂角测试
  python3 step6_arm_angle_stability_test.py --pitch-range -30 30 10 --yaw-range 10 80 10

【位置测试】
  # 前后移动测试（±5cm，步长1cm）
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01

  # 左右移动测试
  python3 step6_arm_angle_stability_test.py --mode position --pos-y-range -0.05 0.05 0.01

  # 上下移动测试
  python3 step6_arm_angle_stability_test.py --mode position --pos-z-range -0.05 0.05 0.01

  # 综合位置测试（前后+上下）
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.03 0.03 0.01 --pos-z-range -0.03 0.03 0.01

【旋转测试】
  # 绕 +X 轴旋转 30° (手腕向右翻滚)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only

  # 绕 -Y 轴旋转 20° (手指向上翘) - 注意负轴用 = 语法
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Y --rot-angle 20 --left-only

  # 绕 +Z 轴旋转 30° (手指向左偏)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis Z --rot-angle 30 --left-only

  # 绕 -Z 轴旋转 30° (手指向右偏) - 注意负轴用 = 语法
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Z --rot-angle 30 --left-only

【通用参数】
  --dry-run       仅打印，不控制机器人
  --left-only     仅测试左臂
  --right-only    仅测试右臂
  --hold-time N   每个测试点保持时间（秒）
"""

import argparse
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

# 添加 common 目录到路径
sys.path.insert(0, str(Path(__file__).parent))

from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS,
    TIANJI_INIT_ROT,
    ROBOT_IP,
)

# 导入坐标变换函数（统一实现，确保与 step2/4/5 一致）
from common.transform_utils import (
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
    elbow_direction_from_angles,
)

# 导入 CartesianController
try:
    from tianji_world_output.cartesian_controller import CartesianController
except ImportError:
    print("警告: 无法导入 CartesianController，仅支持 dry-run 模式")
    CartesianController = None


def generate_test_cases(pitch_range: tuple, yaw_range: tuple) -> list:
    """
    生成测试用例

    Args:
        pitch_range: (min, max, step) 俯仰角范围
        yaw_range: (min, max, step) 外展角范围

    Returns:
        test_cases: [(name, pitch, yaw), ...]
    """
    test_cases = []

    pitch_min, pitch_max, pitch_step = pitch_range
    yaw_min, yaw_max, yaw_step = yaw_range

    # 先测试默认臂角
    test_cases.append(("默认沉肘(0°,45°)", 0, 45))

    # 测试不同外展角（固定 pitch=0）
    for yaw in range(yaw_min, yaw_max + 1, yaw_step):
        if yaw != 45:  # 跳过已添加的默认值
            test_cases.append((f"外展{yaw}°", 0, yaw))

    # 测试不同俯仰角（固定 yaw=45）
    for pitch in range(pitch_min, pitch_max + 1, pitch_step):
        if pitch != 0:  # 跳过已添加的默认值
            name = f"前倾{pitch}°" if pitch > 0 else f"后倾{-pitch}°"
            test_cases.append((name, pitch, 45))

    # 组合测试（可选）
    # for pitch in range(pitch_min, pitch_max + 1, pitch_step):
    #     for yaw in range(yaw_min, yaw_max + 1, yaw_step):
    #         if pitch != 0 and yaw != 45:
    #             test_cases.append((f"pitch={pitch}°,yaw={yaw}°", pitch, yaw))

    return test_cases


def generate_position_test_cases(x_range: tuple, y_range: tuple, z_range: tuple) -> list:
    """
    生成位置测试用例（World 坐标系）

    位置增量在 Robot World 坐标系中定义（两臂相同）：
      X = 前(+) / 后(-)
      Y = 左(+) / 右(-)
      Z = 上(+) / 下(-)

    内部会通过 transform_world_to_chest() 转换到 Chest 坐标系后应用。

    Args:
        x_range: (min, max, step) X方向(前后)偏移范围 (米)，None 表示不测试
        y_range: (min, max, step) Y方向(左右)偏移范围 (米)，None 表示不测试
        z_range: (min, max, step) Z方向(上下)偏移范围 (米)，None 表示不测试

    Returns:
        test_cases: [(name, dx, dy, dz), ...]  dx/dy/dz 为 World 坐标系增量
    """
    test_cases = []

    # 先添加原点（初始位置）
    test_cases.append(("初始位置", 0.0, 0.0, 0.0))

    # X 方向测试（前后）
    if x_range is not None:
        x_min, x_max, x_step = x_range
        x_values = np.arange(x_min, x_max + x_step * 0.5, x_step)
        for dx in x_values:
            if abs(dx) > 1e-6:  # 跳过零点
                name = f"前{dx*100:.1f}cm" if dx > 0 else f"后{-dx*100:.1f}cm"
                test_cases.append((name, float(dx), 0.0, 0.0))

    # Y 方向测试（左右）- World Y+ = 左, Y- = 右
    if y_range is not None:
        y_min, y_max, y_step = y_range
        y_values = np.arange(y_min, y_max + y_step * 0.5, y_step)
        for dy in y_values:
            if abs(dy) > 1e-6:  # 跳过零点
                name = f"左{dy*100:.1f}cm" if dy > 0 else f"右{-dy*100:.1f}cm"
                test_cases.append((name, 0.0, float(dy), 0.0))

    # Z 方向测试（上下）- World Z+ = 上, Z- = 下
    if z_range is not None:
        z_min, z_max, z_step = z_range
        z_values = np.arange(z_min, z_max + z_step * 0.5, z_step)
        for dz in z_values:
            if abs(dz) > 1e-6:  # 跳过零点
                name = f"上{dz*100:.1f}cm" if dz > 0 else f"下{-dz*100:.1f}cm"
                test_cases.append((name, 0.0, 0.0, float(dz)))

    return test_cases


def print_test_header():
    """打印测试表头"""
    print("\n" + "=" * 90)
    print(f"{'测试名称':<20} {'Pitch':>8} {'Yaw':>8} {'Direction':<25} {'IK结果':<10}")
    print("=" * 90)


def print_test_result(name: str, pitch: float, yaw: float, direction: np.ndarray,
                      ik_success: bool, side: str):
    """打印单个测试结果"""
    dir_str = f"[{direction[0]:6.3f}, {direction[1]:6.3f}, {direction[2]:6.3f}]"
    status = "✓ 成功" if ik_success else "✗ 失败"
    print(f"{name:<20} {pitch:>8.1f} {yaw:>8.1f} {dir_str:<25} {status:<10} ({side})")


def print_position_test_header():
    """打印位置测试表头"""
    print("\n" + "=" * 90)
    print(f"{'测试名称':<20} {'ΔX(cm)':>10} {'ΔY(cm)':>10} {'ΔZ(cm)':>10} {'IK结果':<10}")
    print("=" * 90)


def print_position_test_result(name: str, dx: float, dy: float, dz: float,
                                ik_success: bool, side: str):
    """打印单个位置测试结果"""
    status = "✓ 成功" if ik_success else "✗ 失败"
    print(f"{name:<20} {dx*100:>10.1f} {dy*100:>10.1f} {dz*100:>10.1f} {status:<10} ({side})")


def parse_rotation_axis(axis_str: str) -> tuple:
    """
    解析旋转轴参数

    Args:
        axis_str: 轴名称，如 'X', '-X', 'Y', '-Y', 'Z', '-Z'

    Returns:
        (axis_vector, axis_name, expected_effect): 轴向量、轴名称、预期效果描述

    Note:
        旋转效果基于 ROS REP 103 右手定则：
        - 大拇指指向轴的正方向
        - 四指弯曲方向为正旋转方向（从轴正端向原点看逆时针为正）

        机器人坐标系中（已通过实机验证）:
        - +X 旋转：手腕向右翻滚 (Roll right)   — Rx(θ): 顶部向右倾
        - +Y 旋转：手指向下压 (Pitch down)      — Ry(θ): 前端向下倾
        - +Z 旋转：手指向左偏 (Yaw left)        — Rz(θ): 前端向左偏
    """
    axis_map = {
        'X':  (np.array([1, 0, 0]), "+X", "手腕向右翻滚 (Roll right)"),
        '-X': (np.array([-1, 0, 0]), "-X", "手腕向左翻滚 (Roll left)"),
        'Y':  (np.array([0, 1, 0]), "+Y", "手指向下压 (Pitch down)"),
        '-Y': (np.array([0, -1, 0]), "-Y", "手指向上翘 (Pitch up)"),
        'Z':  (np.array([0, 0, 1]), "+Z", "手指向左偏 (Yaw left)"),
        '-Z': (np.array([0, 0, -1]), "-Z", "手指向右偏 (Yaw right)"),
    }

    # 标准化输入
    axis_upper = axis_str.upper().strip()
    if axis_upper.startswith('+'):
        axis_upper = axis_upper[1:]

    if axis_upper not in axis_map:
        raise ValueError(f"无效的旋转轴: {axis_str}，支持: X, -X, Y, -Y, Z, -Z")

    return axis_map[axis_upper]


def run_rotation_test(args):
    """运行旋转测试"""

    print("\n" + "=" * 70)
    print("Step 6: 旋转测试")
    print("=" * 70)

    # 解析旋转参数
    try:
        axis_vec, axis_name, expected_effect = parse_rotation_axis(args.rot_axis)
    except ValueError as e:
        print(f"\n❌ 错误: {e}")
        return

    angle_deg = args.rot_angle

    print(f"\n参数:")
    print(f"  旋转轴 (Robot World): {axis_name}")
    print(f"  旋转角度: {angle_deg}°")
    print(f"  预期效果: {expected_effect}")
    print(f"  保持时间: {args.hold_time}s")
    print(f"  模式: {'干运行' if args.dry_run else '真机控制'}")

    print("\n旋转效果参考表 (Robot World 坐标系 - ROS REP 103 右手定则，从轴正端向原点看逆时针为正):")
    print("  ┌──────────────┬───────────────────────────────────────────────┐")
    print("  │ 旋转轴       │ 实机效果 (大拇指指向轴,四指弯曲为正转)        │")
    print("  ├──────────────┼───────────────────────────────────────────────┤")
    print("  │ 绕 +X 轴     │ 手腕向右翻滚 (Roll right: 顶部向右倾)        │")
    print("  │ 绕 -X 轴     │ 手腕向左翻滚 (Roll left: 顶部向左倾)         │")
    print("  │ 绕 +Y 轴     │ 手指向下压 (Pitch down: 低头)                │")
    print("  │ 绕 -Y 轴     │ 手指向上翘 (Pitch up: 抬头)                  │")
    print("  │ 绕 +Z 轴     │ 手指向左偏 (Yaw left: 从上看逆时针)          │")
    print("  │ 绕 -Z 轴     │ 手指向右偏 (Yaw right: 从上看顺时针)         │")
    print("  └──────────────┴───────────────────────────────────────────────┘")

    # 确定测试哪些臂
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"\n  测试臂: {', '.join(sides)}")

    # 连接机器人
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\n❌ 错误: CartesianController 不可用，请使用 --dry-run 模式")
            return

        print("\n[1/4] 连接机器人...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  ✓ 已连接到 {ROBOT_IP}")

            print("  设置阻抗模式...")
            controller.set_impedance_mode(mode='joint')
            print("  ✓ 机器人已使能")

            print("  移动到初始位姿...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ 已到达初始位姿")

        except Exception as e:
            print(f"  ❌ 连接失败: {e}")
            return
    else:
        print("\n[1/4] 跳过机器人连接（干运行模式）")

    # 计算旋转
    print("\n[2/4] 计算旋转...")

    # 在 Robot World 坐标系中的旋转
    rotvec_world = axis_vec * np.radians(angle_deg)
    R_delta_world = R.from_rotvec(rotvec_world)

    print(f"  Robot World 旋转向量: [{rotvec_world[0]:.4f}, {rotvec_world[1]:.4f}, {rotvec_world[2]:.4f}] rad")

    # 固定使用默认沉肘臂角
    default_pitch, default_yaw = 0, 45

    # 运行测试
    print("\n[3/4] 执行旋转...")

    for side in sides:
        print(f"\n  [{side.upper()} 臂]")

        # 基准位姿（初始位姿，Chest 坐标系）
        base_pos = TIANJI_INIT_POS[side].copy()
        base_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        # 设置臂角
        direction = elbow_direction_from_angles(default_pitch, default_yaw, side)
        zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

        # 使用正确的旋转变换（与 step2 一致的4步算法）
        target_rot_matrix = apply_world_rotation_to_chest_pose(
            base_rot.as_matrix(), R_delta_world, side
        )
        target_rot = R.from_matrix(target_rot_matrix)

        # 构建位姿矩阵
        pose_mat = np.eye(4)
        pose_mat[:3, :3] = target_rot.as_matrix()
        pose_mat[:3, 3] = base_pos  # 位置保持不变

        # 打印调试信息
        print(f"    基准姿态 (Chest):")
        print(f"      位置: [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}] m")
        euler_base = base_rot.as_euler('xyz', degrees=True)
        print(f"      姿态: [{euler_base[0]:.2f}, {euler_base[1]:.2f}, {euler_base[2]:.2f}]° (XYZ euler)")

        euler_target = target_rot.as_euler('xyz', degrees=True)
        print(f"    目标姿态 (Chest):")
        print(f"      位置: [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}] m (不变)")
        print(f"      姿态: [{euler_target[0]:.2f}, {euler_target[1]:.2f}, {euler_target[2]:.2f}]° (XYZ euler)")

        if controller:
            try:
                # 设置臂角
                if side == 'left':
                    controller.left_zsp_para = zsp_para
                else:
                    controller.right_zsp_para = zsp_para

                # 发送控制命令
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
                    print(f"    ✓ IK 成功，正在旋转...")
                else:
                    print(f"    ✗ IK 失败!")

                # 保持一段时间
                print(f"    保持 {args.hold_time}s...")
                time.sleep(args.hold_time)

            except Exception as e:
                print(f"    ⚠️ 控制异常: {e}")
        else:
            print(f"    (干运行模式，跳过实际控制)")

    # 返回初始位姿
    if controller:
        print("\n[4/4] 返回初始位姿...")
        controller.move_to_init(
            wait=True, timeout=2,
            init_joints_left=INIT_JOINTS['left'],
            init_joints_right=INIT_JOINTS['right']
        )
        print("  ✓ 已返回初始位姿")

        print("\n下电并释放机器人...")
        controller.disable_and_release()
        print("✓ 机器人已安全退出")
    else:
        print("\n[4/4] 跳过返回初始位姿（干运行模式）")

    print("\n" + "=" * 70)
    print(f"旋转测试完成: 绕 {axis_name} 轴旋转 {angle_deg}°")
    print(f"预期效果: {expected_effect}")
    print("请观察实机效果是否符合预期")
    print("=" * 70)


def run_stability_test(args):
    """运行臂角稳定性测试"""

    print("\n" + "=" * 70)
    print("Step 6: 臂角稳定性测试")
    print("=" * 70)

    # 解析角度范围
    pitch_range = tuple(args.pitch_range)
    yaw_range = tuple(args.yaw_range)

    print(f"\n参数:")
    print(f"  Pitch 范围: {pitch_range[0]}° ~ {pitch_range[1]}°, 步长 {pitch_range[2]}°")
    print(f"  Yaw 范围:   {yaw_range[0]}° ~ {yaw_range[1]}°, 步长 {yaw_range[2]}°")
    print(f"  保持时间:   {args.hold_time}s")
    print(f"  模式:       {'干运行' if args.dry_run else '真机控制'}")

    # 生成测试用例
    test_cases = generate_test_cases(pitch_range, yaw_range)
    print(f"  测试用例:   {len(test_cases)} 个")

    # 确定测试哪些臂
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"  测试臂:     {', '.join(sides)}")

    # 连接机器人
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\n❌ 错误: CartesianController 不可用，请使用 --dry-run 模式")
            return

        print("\n[1/3] 连接机器人...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  ✓ 已连接到 {ROBOT_IP}")

            print("  设置阻抗模式...")
            controller.set_impedance_mode(mode='joint')
            print("  ✓ 机器人已使能")

            print("  移动到初始位姿...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ 已到达初始位姿")

        except Exception as e:
            print(f"  ❌ 连接失败: {e}")
            return
    else:
        print("\n[1/3] 跳过机器人连接（干运行模式）")

    # 准备结果统计
    results = {
        'left': {'success': 0, 'fail': 0, 'cases': []},
        'right': {'success': 0, 'fail': 0, 'cases': []}
    }

    # 运行测试
    print("\n[2/3] 开始测试...")
    print_test_header()

    for side in sides:
        # 固定的手腕位姿（使用初始位姿）
        wrist_pos = TIANJI_INIT_POS[side].copy()
        wrist_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        for name, pitch, yaw in test_cases:
            # 计算 elbow direction
            direction = elbow_direction_from_angles(pitch, yaw, side)

            # 构建 zsp_para
            zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

            ik_success = True  # 默认成功

            if controller:
                try:
                    # 设置臂角
                    if side == 'left':
                        controller.left_zsp_para = zsp_para
                    else:
                        controller.right_zsp_para = zsp_para

                    # 构建位姿矩阵
                    pose_mat = np.eye(4)
                    pose_mat[:3, :3] = wrist_rot.as_matrix()
                    pose_mat[:3, 3] = wrist_pos

                    # 发送控制命令
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

                    # 保持一段时间
                    time.sleep(args.hold_time)

                except Exception as e:
                    print(f"  ⚠️ 控制异常: {e}")
                    ik_success = False

            # 记录结果
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

    # 打印统计
    print("\n" + "=" * 90)
    print("[3/3] 测试统计")
    print("=" * 90)

    for side in sides:
        total = results[side]['success'] + results[side]['fail']
        success_rate = results[side]['success'] / total * 100 if total > 0 else 0
        print(f"\n{side.upper()} 臂:")
        print(f"  成功: {results[side]['success']}/{total} ({success_rate:.1f}%)")
        print(f"  失败: {results[side]['fail']}/{total}")

        if results[side]['fail'] > 0:
            print(f"  失败用例:")
            for case in results[side]['cases']:
                if not case['success']:
                    print(f"    - {case['name']}: pitch={case['pitch']}°, yaw={case['yaw']}°")

    # 清理
    if controller:
        print("\n下电并释放机器人...")
        controller.disable_and_release()
        print("✓ 机器人已安全退出")

    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70)


def run_position_test(args):
    """运行位置稳定性测试"""

    print("\n" + "=" * 70)
    print("Step 6: 位置稳定性测试")
    print("=" * 70)

    # 解析位置范围
    x_range = tuple(args.pos_x_range) if args.pos_x_range else None
    y_range = tuple(args.pos_y_range) if args.pos_y_range else None
    z_range = tuple(args.pos_z_range) if args.pos_z_range else None

    print(f"\n参数:")
    print(f"  坐标系: Robot World (ROS REP 103)")
    print(f"    X = 前(+) / 后(-)")
    print(f"    Y = 左(+) / 右(-)")
    print(f"    Z = 上(+) / 下(-)")
    print(f"  注意: 内部自动转换到 Chest 坐标系（两臂接口统一）")
    if x_range:
        print(f"  X 范围 (前后): {x_range[0]*100:.1f}cm ~ {x_range[1]*100:.1f}cm, 步长 {x_range[2]*100:.1f}cm")
    if y_range:
        print(f"  Y 范围 (左右): {y_range[0]*100:.1f}cm ~ {y_range[1]*100:.1f}cm, 步长 {y_range[2]*100:.1f}cm")
    if z_range:
        print(f"  Z 范围 (上下): {z_range[0]*100:.1f}cm ~ {z_range[1]*100:.1f}cm, 步长 {z_range[2]*100:.1f}cm")
    print(f"  保持时间:   {args.hold_time}s")
    print(f"  模式:       {'干运行' if args.dry_run else '真机控制'}")

    # 检查是否有范围参数
    if x_range is None and y_range is None and z_range is None:
        print("\n❌ 错误: 位置模式需要至少指定一个范围参数 (--pos-x-range, --pos-y-range, --pos-z-range)")
        return

    # 确定测试哪些臂
    sides = []
    if args.left_only:
        sides = ['left']
    elif args.right_only:
        sides = ['right']
    else:
        sides = ['left', 'right']
    print(f"  测试臂:     {', '.join(sides)}")

    # 连接机器人
    controller = None
    if not args.dry_run:
        if CartesianController is None:
            print("\n❌ 错误: CartesianController 不可用，请使用 --dry-run 模式")
            return

        print("\n[1/3] 连接机器人...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"  ✓ 已连接到 {ROBOT_IP}")

            print("  设置阻抗模式...")
            controller.set_impedance_mode(mode='joint')
            print("  ✓ 机器人已使能")

            print("  移动到初始位姿...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ 已到达初始位姿")

        except Exception as e:
            print(f"  ❌ 连接失败: {e}")
            return
    else:
        print("\n[1/3] 跳过机器人连接（干运行模式）")

    # 准备结果统计
    results = {
        'left': {'success': 0, 'fail': 0, 'cases': []},
        'right': {'success': 0, 'fail': 0, 'cases': []}
    }

    # 运行测试
    print("\n[2/3] 开始测试...")
    print_position_test_header()

    # 固定使用默认沉肘臂角
    default_pitch, default_yaw = 0, 45

    for side in sides:
        # 测试用例使用 World 坐标系（两臂相同）
        test_cases = generate_position_test_cases(x_range, y_range, z_range)
        print(f"\n  [{side.upper()} 臂] 测试用例: {len(test_cases)} 个")

        # 基准位姿（初始位姿，Chest 坐标系）
        base_pos = TIANJI_INIT_POS[side].copy()
        base_rot = R.from_matrix(TIANJI_INIT_ROT[side])

        # 设置默认臂角
        direction = elbow_direction_from_angles(default_pitch, default_yaw, side)
        zsp_para = [direction[0], direction[1], direction[2], 0, 0, 0]

        for name, dx, dy, dz in test_cases:
            # dx, dy, dz 是 World 坐标系增量，需要转换到 Chest 坐标系
            displacement_world = np.array([dx, dy, dz])
            displacement_chest = transform_world_to_chest(displacement_world, side)
            target_pos = base_pos + displacement_chest

            ik_success = True  # 默认成功

            if controller:
                try:
                    # 设置臂角
                    if side == 'left':
                        controller.left_zsp_para = zsp_para
                    else:
                        controller.right_zsp_para = zsp_para

                    # 构建位姿矩阵
                    pose_mat = np.eye(4)
                    pose_mat[:3, :3] = base_rot.as_matrix()
                    pose_mat[:3, 3] = target_pos

                    # 发送控制命令
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

                    # 保持一段时间
                    time.sleep(args.hold_time)

                except Exception as e:
                    print(f"  ⚠️ 控制异常: {e}")
                    ik_success = False

            # 记录结果
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

    # 打印统计
    print("\n" + "=" * 90)
    print("[3/3] 测试统计")
    print("=" * 90)

    for side in sides:
        total = results[side]['success'] + results[side]['fail']
        success_rate = results[side]['success'] / total * 100 if total > 0 else 0
        print(f"\n{side.upper()} 臂:")
        print(f"  成功: {results[side]['success']}/{total} ({success_rate:.1f}%)")
        print(f"  失败: {results[side]['fail']}/{total}")

        if results[side]['fail'] > 0:
            print(f"  失败用例:")
            for case in results[side]['cases']:
                if not case['success']:
                    print(f"    - {case['name']}: Δx={case['dx']*100:.1f}cm, Δy={case['dy']*100:.1f}cm, Δz={case['dz']*100:.1f}cm")

    # 清理
    if controller:
        print("\n下电并释放机器人...")
        controller.disable_and_release()
        print("✓ 机器人已安全退出")

    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="Step 6: 机器人稳定性测试（模拟 Tracking 数据）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  【臂角测试】
  # 干运行（仅打印）
  python3 step6_arm_angle_stability_test.py --dry-run

  # 测试左臂，pitch ±10°，yaw 30°~60°
  python3 step6_arm_angle_stability_test.py --left-only --pitch-range -10 10 5 --yaw-range 30 60 10

  【位置测试】
  # 前后移动测试（±5cm，步长1cm）
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01

  # 综合位置测试（前后+上下）
  python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.03 0.03 0.01 --pos-z-range -0.03 0.03 0.01

  【旋转测试】
  # 绕 +X 轴旋转 30° (手腕向右翻滚)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only

  # 绕 -Y 轴旋转 20° (手指向上翘) - 注意负轴用 = 语法
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Y --rot-angle 20 --left-only

  # 绕 +Z 轴旋转 30° (手指向左偏)
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis Z --rot-angle 30 --left-only

  # 绕 -Z 轴旋转 30° (手指向右偏) - 注意负轴用 = 语法
  python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis=-Z --rot-angle 30 --left-only

角度说明 (臂角模式):
  Pitch (俯仰角): 肘部前后偏移
    - 正值 = 肘部向前
    - 负值 = 肘部向后
    - 0° = 不前后偏移

  Yaw (外展角): 肘部反向重力/向外的比例 (Chest坐标系)
    - 0° = 肘部纯反向重力 [左臂: 0,-1,0] [右臂: 0,+1,0]
    - 45° = 默认沉肘 [左臂: 0,-0.707,-0.707] [右臂: 0,+0.707,-0.707]
    - 90° = 肘部纯向外 [0, 0, -1]

位置说明 (位置模式) - Robot World 坐标系 (两臂相同):
  X 方向: 前(+) / 后(-)
  Y 方向: 左(+) / 右(-)
  Z 方向: 上(+) / 下(-)

  注意: 内部自动转换到 Chest 坐标系（用户无需关心 Chest 坐标轴差异）

旋转说明 (旋转模式) - Robot World 坐标系 (ROS REP 103 右手定则，从轴正端向原点看逆时针为正):
  绕 +X 轴: 手腕向右翻滚 (Roll right: 顶部向右倾)
  绕 -X 轴: 手腕向左翻滚 (Roll left: 顶部向左倾)
  绕 +Y 轴: 手指向下压 (Pitch down: 低头)
  绕 -Y 轴: 手指向上翘 (Pitch up: 抬头)
  绕 +Z 轴: 手指向左偏 (Yaw left: 从上看逆时针)
  绕 -Z 轴: 手指向右偏 (Yaw right: 从上看顺时针)
"""
    )

    # 通用参数
    parser.add_argument('--mode', type=str, choices=['arm_angle', 'position', 'rotation'], default='arm_angle',
                        help='测试模式: arm_angle=臂角测试, position=位置测试, rotation=旋转测试 (默认: arm_angle)')
    parser.add_argument('--dry-run', action='store_true',
                        help='干运行模式，仅打印不控制机器人')
    parser.add_argument('--left-only', action='store_true',
                        help='仅测试左臂')
    parser.add_argument('--right-only', action='store_true',
                        help='仅测试右臂')
    parser.add_argument('--hold-time', type=float, default=1.0,
                        help='每个测试点保持时间（秒）(默认: 1.0)')

    # 臂角测试参数
    parser.add_argument('--pitch-range', type=int, nargs=3, default=[-10, 10, 5],
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='俯仰角范围 (默认: -10 10 5)')
    parser.add_argument('--yaw-range', type=int, nargs=3, default=[30, 60, 10],
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='外展角范围 (默认: 30 60 10)')

    # 位置测试参数
    parser.add_argument('--pos-x-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='X方向(前后)偏移范围，单位米 (例: -0.05 0.05 0.01)')
    parser.add_argument('--pos-y-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Y方向(左右)偏移范围，单位米 (例: -0.05 0.05 0.01)')
    parser.add_argument('--pos-z-range', type=float, nargs=3, default=None,
                        metavar=('MIN', 'MAX', 'STEP'),
                        help='Z方向(上下)偏移范围，单位米 (例: -0.05 0.05 0.01)')

    # 旋转测试参数
    parser.add_argument('--rot-axis', type=str, default='X',
                        help='旋转轴 (Robot World 坐标系): X, -X, Y, -Y, Z, -Z (默认: X)')
    parser.add_argument('--rot-angle', type=float, default=30.0,
                        help='旋转角度（度）(默认: 30.0)')

    args = parser.parse_args()

    # 检查参数冲突
    if args.left_only and args.right_only:
        print("错误: --left-only 和 --right-only 不能同时使用")
        sys.exit(1)

    # 根据模式运行对应测试
    if args.mode == 'arm_angle':
        run_stability_test(args)
    elif args.mode == 'position':
        run_position_test(args)
    else:  # rotation
        run_rotation_test(args)


if __name__ == '__main__':
    main()
