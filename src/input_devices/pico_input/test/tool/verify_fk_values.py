#!/usr/bin/env python3
"""
验证 tianji_robot.yaml 中的 TIANJI_INIT_POS/ROT 值

功能：
  1. 从 INIT_JOINTS 使用 FK 计算位姿
  2. 与 tianji_robot.yaml 配置值对比
  3. 显示坐标系说明

使用：
  cd /path/to/test && python3 tool/verify_fk_values.py
"""

import sys
import numpy as np
from pathlib import Path

# 导入共享模块（使用相对路径）
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

# 导入机器人控制器
try:
    from tianji_world_output.cartesian_controller import CartesianController
    HAS_CONTROLLER = True
except ImportError:
    print("⚠️  无法导入 CartesianController，将只显示配置值")
    HAS_CONTROLLER = False


def main():
    print("=" * 70)
    print("验证 FK 计算值 vs tianji_robot.yaml 配置值")
    print("=" * 70)

    print("\n【1】INIT_JOINTS（初始关节角，度）")
    print(f"  左臂: {INIT_JOINTS['left']}")
    print(f"  右臂: {INIT_JOINTS['right']}")

    print("\n【2】tianji_robot.yaml 配置值（Chest 坐标系）")
    print("\n  TIANJI_INIT_POS（末端位置，米）:")
    print(f"    左臂: {TIANJI_INIT_POS['left']}")
    print(f"    右臂: {TIANJI_INIT_POS['right']}")

    print("\n  TIANJI_INIT_ROT（末端姿态，旋转矩阵）:")
    print(f"    左臂:\n{TIANJI_INIT_ROT['left']}")
    print(f"    右臂:\n{TIANJI_INIT_ROT['right']}")

    if not HAS_CONTROLLER:
        print("\n" + "=" * 70)
        print("⚠️  无法连接机器人，跳过 FK 验证")
        print("=" * 70)
        return

    print("\n【3】使用 FK 重新计算...")

    try:
        controller = CartesianController(robot_ip=ROBOT_IP)
        print(f"  ✓ 已连接到机器人 {ROBOT_IP}")

        # 计算 FK
        left_fk = controller.kine_left.fk(0, INIT_JOINTS['left'])
        right_fk = controller.kine_right.fk(1, INIT_JOINTS['right'])

        # 提取位置（毫米 → 米）
        left_pos_fk = np.array([left_fk[0][3], left_fk[1][3], left_fk[2][3]]) / 1000.0
        right_pos_fk = np.array([right_fk[0][3], right_fk[1][3], right_fk[2][3]]) / 1000.0

        # 提取旋转矩阵
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

        print("\n【4】FK 计算结果 vs 配置值对比")

        print("\n  左臂位置:")
        print(f"    FK计算:  [{left_pos_fk[0]:.4f}, {left_pos_fk[1]:.4f}, {left_pos_fk[2]:.4f}]")
        print(f"    配置值:  [{TIANJI_INIT_POS['left'][0]:.4f}, {TIANJI_INIT_POS['left'][1]:.4f}, {TIANJI_INIT_POS['left'][2]:.4f}]")
        pos_diff_left = np.linalg.norm(left_pos_fk - TIANJI_INIT_POS['left']) * 1000
        print(f"    差异:    {pos_diff_left:.2f} mm")

        print("\n  右臂位置:")
        print(f"    FK计算:  [{right_pos_fk[0]:.4f}, {right_pos_fk[1]:.4f}, {right_pos_fk[2]:.4f}]")
        print(f"    配置值:  [{TIANJI_INIT_POS['right'][0]:.4f}, {TIANJI_INIT_POS['right'][1]:.4f}, {TIANJI_INIT_POS['right'][2]:.4f}]")
        pos_diff_right = np.linalg.norm(right_pos_fk - TIANJI_INIT_POS['right']) * 1000
        print(f"    差异:    {pos_diff_right:.2f} mm")

        print("\n  左臂姿态:")
        rot_diff_left = np.linalg.norm(left_rot_fk - TIANJI_INIT_ROT['left'])
        print(f"    差异范数: {rot_diff_left:.6f}")

        print("\n  右臂姿态:")
        rot_diff_right = np.linalg.norm(right_rot_fk - TIANJI_INIT_ROT['right'])
        print(f"    差异范数: {rot_diff_right:.6f}")

        # 验证结果
        print("\n" + "=" * 70)
        if pos_diff_left < 1 and pos_diff_right < 1 and rot_diff_left < 0.01 and rot_diff_right < 0.01:
            print("✓ 验证通过：FK 计算值与配置值一致")
        else:
            print("⚠️  验证失败：FK 计算值与配置值有差异")
            print("   可能原因：运动学配置文件版本不同")

        # 如果需要更新配置，打印新值
        print("\n【5】如需更新 tianji_robot.yaml，使用以下值：")
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
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

    print("\n【坐标系说明】")
    print("""
  TIANJI_INIT_POS/ROT 是在 Chest 坐标系中的值，不是 World 坐标系！

  坐标系关系：
    World (ROS REP 103)           Chest (机械臂基座)
    ┌─────────────────┐           ┌─────────────────┐
    │ X = 前          │           │ Left:  X前 Y下 Z左
    │ Y = 左          │  ──────>  │ Right: X前 Y上 Z右
    │ Z = 上          │  绕X轴±90°│
    └─────────────────┘           └─────────────────┘

  转换公式：
    Left Chest  = World 绕 X 轴旋转 +90°
    Right Chest = World 绕 X 轴旋转 -90°
""")


if __name__ == '__main__':
    main()
