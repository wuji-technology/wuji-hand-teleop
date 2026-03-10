#!/usr/bin/env python3
"""
移动机器人到初始位姿（使用共享模块和生命周期管理）
对应 pico_input.yaml 中的 robot_*_init 配置
"""

import sys
import numpy as np
import time
from pathlib import Path

# 添加路径（使用相对路径）
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
        print("移动机器人到初始位姿（PICO遥操零位）")
        print("=" * 70)

        print("\n初始关节角:")
        print(f"  左臂: {INIT_JOINTS['left']}")
        print(f"  右臂: {INIT_JOINTS['right']}")

        # 连接机器人
        print("\n正在连接机器人...")
        controller = CartesianController(robot_ip=ROBOT_IP)
        print("✓ 已连接到机器人")

        # 使能机器人（使用统一的生命周期管理）
        enable_robot(controller.robot, state=3, vel_ratio=60, acc_ratio=60)

        # 计算FK验证目标位姿
        print("\n计算FK验证目标位姿...")
        left_fk = controller.kine_left.fk(0, INIT_JOINTS['left'])
        right_fk = controller.kine_right.fk(1, INIT_JOINTS['right'])

        left_pos = np.array([left_fk[0][3], left_fk[1][3], left_fk[2][3]]) / 1000.0
        right_pos = np.array([right_fk[0][3], right_fk[1][3], right_fk[2][3]]) / 1000.0

        print(f"\n目标位姿 (FK计算):")
        print(f"  左腕位置: [{left_pos[0]:.4f}, {left_pos[1]:.4f}, {left_pos[2]:.4f}]")
        print(f"  右腕位置: [{right_pos[0]:.4f}, {right_pos[1]:.4f}, {right_pos[2]:.4f}]")

        # 获取当前关节角
        print("\n读取当前关节角...")
        current_joints = controller.get_current_joints()
        current_left = np.array(current_joints[0])
        current_right = np.array(current_joints[1])

        print(f"  当前左臂: {current_left}")
        print(f"  当前右臂: {current_right}")

        # 移动到初始位姿
        print("\n开始移动...")
        print("  ⚠️  请确保机器人周围无障碍物!")
        print("  ⚠️  机器人将在5秒内平滑移动到初始位姿")
        input("  按 Enter 继续...")

        # 平滑插值移动 (5秒，100Hz)
        duration = 5.0
        rate = 100  # Hz
        num_steps = int(duration * rate)
        dt = 1.0 / rate

        target_left_final = np.array(INIT_JOINTS['left'])
        target_right_final = np.array(INIT_JOINTS['right'])

        print(f"\n移动中... ({duration}秒)")
        for i in range(num_steps + 1):
            alpha = i / num_steps  # 0 → 1

            # 线性插值
            target_left = current_left + alpha * (target_left_final - current_left)
            target_right = current_right + alpha * (target_right_final - current_right)

            # 发送关节指令（使用统一接口）
            send_joint_command(controller.robot, target_left.tolist(), target_right.tolist())

            # 进度显示
            if i % 50 == 0:
                progress = int(100 * alpha)
                print(f"  进度: {progress}%")

            time.sleep(dt)

        print("\n✓ 已到达初始位姿!")
        print("\n机器人现在处于PICO遥操的零位姿态 (末端坐标轴在World中的方向):")
        print("  - 左手腕: X→右(-Y), Y→下(-Z), Z→前(+X)")
        print("  - 右手腕: X→左(+Y), Y→上(+Z), Z→前(+X)")
        print("\n可以开始遥操测试了!")

    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保下电（重要！）
        if controller is not None:
            disable_robot(controller.robot)


if __name__ == '__main__':
    main()
