#!/usr/bin/env python3
"""
Robot lifecycle management utility (unified power-on/power-off workflow)
"""

import time


class RobotLifecycleManager:
    """Robot lifecycle management"""

    @staticmethod
    def enable_robot(robot, state=3, vel_ratio=60, acc_ratio=60, verbose=True):
        """Unified power-on workflow

        Args:
            robot: Marvin_Robot instance
            state: Robot state (0=power-off, 1=position follow, 3=impedance mode)
            vel_ratio: Velocity ratio (0-100)
            acc_ratio: Acceleration ratio (0-100)
            verbose: Whether to print logs

        State descriptions:
            state=0: Power-off/disabled
            state=1: Position follow mode
            state=3: Impedance mode (recommended, allows force control)
        """
        if verbose:
            print("Enabling robot...")

        # 1. Clear errors (important!)
        robot.clear_set()
        robot.clear_error('A')
        robot.clear_error('B')
        robot.send_cmd()
        time.sleep(0.5)

        if verbose:
            print("  Errors cleared")

        # 2. Power on and set mode
        robot.clear_set()
        robot.set_state(arm='A', state=state)
        robot.set_state(arm='B', state=state)
        robot.set_vel_acc(arm='A', velRatio=vel_ratio, AccRatio=acc_ratio)
        robot.set_vel_acc(arm='B', velRatio=vel_ratio, AccRatio=acc_ratio)
        robot.send_cmd()
        time.sleep(0.5)

        if verbose:
            state_name = {0: "power-off", 1: "position follow", 3: "impedance mode"}.get(state, f"state {state}")
            print(f"  Robot enabled ({state_name}, velocity {vel_ratio}%)")

    @staticmethod
    def disable_robot(robot, verbose=True):
        """Unified power-off workflow

        Args:
            robot: Marvin_Robot instance
            verbose: Whether to print logs
        """
        if verbose:
            print("Disabling robot...")

        try:
            robot.clear_set()
            robot.set_state(arm='A', state=0)  # Power off left arm
            robot.set_state(arm='B', state=0)  # Power off right arm
            robot.send_cmd()
            time.sleep(0.5)

            if verbose:
                print("  Robot disabled")
        except Exception as e:
            if verbose:
                print(f"  Power-off failed: {e}")

    @staticmethod
    def send_joint_command(robot, left_joints, right_joints):
        """Unified joint command sending

        Args:
            robot: Marvin_Robot instance
            left_joints: Left arm joint angles (list/array)
            right_joints: Right arm joint angles (list/array)
        """
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=left_joints)
        robot.set_joint_cmd_pose(arm='B', joints=right_joints)
        robot.send_cmd()


# Convenience functions
def enable_robot(robot, **kwargs):
    """Convenience function: power on"""
    RobotLifecycleManager.enable_robot(robot, **kwargs)


def disable_robot(robot, **kwargs):
    """Convenience function: power off"""
    RobotLifecycleManager.disable_robot(robot, **kwargs)


def send_joint_command(robot, left_joints, right_joints):
    """Convenience function: send joint command"""
    RobotLifecycleManager.send_joint_command(robot, left_joints, right_joints)
