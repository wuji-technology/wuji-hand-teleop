#!/usr/bin/env python3
"""
机器人生命周期管理工具（统一上电/下电流程）
"""

import time


class RobotLifecycleManager:
    """机器人生命周期管理"""

    @staticmethod
    def enable_robot(robot, state=3, vel_ratio=60, acc_ratio=60, verbose=True):
        """统一的上电流程

        Args:
            robot: Marvin_Robot 实例
            state: 机器人状态 (0=下电, 1=位置跟随, 3=阻抗模式)
            vel_ratio: 速度比例 (0-100)
            acc_ratio: 加速度比例 (0-100)
            verbose: 是否打印日志

        状态说明:
            state=0: 下电/禁用
            state=1: 位置跟随模式
            state=3: 阻抗模式（推荐，允许力控）
        """
        if verbose:
            print("使能机器人...")

        # 1. 清除错误（重要！）
        robot.clear_set()
        robot.clear_error('A')
        robot.clear_error('B')
        robot.send_cmd()
        time.sleep(0.5)

        if verbose:
            print("  ✓ 已清除错误")

        # 2. 上电并设置模式
        robot.clear_set()
        robot.set_state(arm='A', state=state)
        robot.set_state(arm='B', state=state)
        robot.set_vel_acc(arm='A', velRatio=vel_ratio, AccRatio=acc_ratio)
        robot.set_vel_acc(arm='B', velRatio=vel_ratio, AccRatio=acc_ratio)
        robot.send_cmd()
        time.sleep(0.5)

        if verbose:
            state_name = {0: "下电", 1: "位置跟随", 3: "阻抗模式"}.get(state, f"状态{state}")
            print(f"  ✓ 机器人已使能（{state_name}，速度{vel_ratio}%）")

    @staticmethod
    def disable_robot(robot, verbose=True):
        """统一的下电流程

        Args:
            robot: Marvin_Robot 实例
            verbose: 是否打印日志
        """
        if verbose:
            print("下电机器人...")

        try:
            robot.clear_set()
            robot.set_state(arm='A', state=0)  # 下电左臂
            robot.set_state(arm='B', state=0)  # 下电右臂
            robot.send_cmd()
            time.sleep(0.5)

            if verbose:
                print("  ✓ 机器人已下电")
        except Exception as e:
            if verbose:
                print(f"  ⚠ 下电失败: {e}")

    @staticmethod
    def send_joint_command(robot, left_joints, right_joints):
        """统一的关节指令发送

        Args:
            robot: Marvin_Robot 实例
            left_joints: 左臂关节角 (list/array)
            right_joints: 右臂关节角 (list/array)
        """
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=left_joints)
        robot.set_joint_cmd_pose(arm='B', joints=right_joints)
        robot.send_cmd()


# 便捷函数
def enable_robot(robot, **kwargs):
    """便捷函数：上电"""
    RobotLifecycleManager.enable_robot(robot, **kwargs)


def disable_robot(robot, **kwargs):
    """便捷函数：下电"""
    RobotLifecycleManager.disable_robot(robot, **kwargs)


def send_joint_command(robot, left_joints, right_joints):
    """便捷函数：发送关节指令"""
    RobotLifecycleManager.send_joint_command(robot, left_joints, right_joints)
