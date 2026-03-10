#!/usr/bin/env python3
"""
=============================================================================
Step 1: 直接关节空间控制（最基础测试）
=============================================================================

测试目的：
  验证机器人 SDK 连接和基础运动功能

输入输出：
  输入：目标位姿（笛卡尔空间坐标）
  处理：IK 计算 → 关节角 → 五次多项式插值
  输出：发送关节角指令到真机

使用方式：
  cd ~/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/test

  # === 平移测试 ===
  # 基础测试（保持初始位姿）
  python3 step1_direct_joint_control.py --mode static --duration 5.0

  # 前进（world +X）- 推荐参数
  python3 step1_direct_joint_control.py --mode forward --speed 0.03 --duration 2.0

  # 后退（world -X）
  python3 step1_direct_joint_control.py --mode back --speed 0.03 --duration 2.0

  # 左移（world +Y）
  python3 step1_direct_joint_control.py --mode left --speed 0.02 --duration 2.0

  # 右移（world -Y）
  python3 step1_direct_joint_control.py --mode right --speed 0.02 --duration 2.0

  # 上升（world +Z）
  python3 step1_direct_joint_control.py --mode up --speed 0.02 --duration 2.0

  # 下降（world -Z）
  python3 step1_direct_joint_control.py --mode down --speed 0.02 --duration 2.0

  # === 旋转测试（外旋，绕 world 坐标系轴）===
  # 对齐测试（验证末端坐标轴与世界坐标系是否对齐）
  python3 step1_direct_joint_control.py --mode align_test --duration 5.0
  # → 机械臂运动到"标准平举姿势"
  # → 在 RViz 中查看：world_left_dh_ee 和 world_right_dh_ee 的坐标轴
  # → 应该与 world 坐标轴完全平行（X红 Y绿 Z蓝）

  # 绕 X 轴旋转（roll，前后轴）- speed 单位是 rad/s
  python3 step1_direct_joint_control.py --mode rotate_x --speed 0.3 --duration 3.0
  # 旋转角度 = 0.3 rad/s * 3.0 s = 0.9 rad ≈ 51.6°

  # 绕 Y 轴旋转（pitch，左右轴）
  python3 step1_direct_joint_control.py --mode rotate_y --speed 0.3 --duration 3.0

  # 绕 Z 轴旋转（yaw，上下轴）
  python3 step1_direct_joint_control.py --mode rotate_z --speed 0.3 --duration 3.0

  # === 其他测试 ===
  # 单臂测试（仅左臂）
  python3 step1_direct_joint_control.py --mode left_only --speed 0.03 --duration 3.0

  # 单臂测试（仅右臂）
  python3 step1_direct_joint_control.py --mode right_only --speed 0.03 --duration 3.0

不需要：
  ❌ ROS2
  ❌ TF
  ❌ Launch 文件

特点：
  - 使用关节空间插值（五次多项式）实现平滑运动
  - 避免连续 IK 计算导致的不连续
  - 直接调用机器人 SDK，不经过 ROS2
  - 参考 cartesian_controller.move_to_init 的实现

验证结果：
  ✅ 机械臂能连接
  ✅ 机械臂能运动到初始位姿
  ✅ 控制平滑无抖动
  ✅ 运动完成后自动下电

=============================================================================
"""

import argparse
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

# 导入机器人控制库（使用相对路径）
from pathlib import Path
_test_dir = str(Path(__file__).resolve().parent)
_src_dir = str(Path(__file__).resolve().parents[3])  # test → pico_input → input_devices → src
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
    """关节空间轨迹控制器（使用五次多项式插值）"""

    def __init__(self, mode: str, speed: float, duration: float,
                 interpolation_dt: float = 0.02, dry_run: bool = False):
        self.mode = mode
        self.speed = speed
        self.duration = duration
        self.interpolation_dt = interpolation_dt  # 插值时间步长
        self.dry_run = dry_run

        # 初始化运动学库
        print("初始化运动学库...")
        config_path = KINE_CONFIG_PATH

        self.kine_left = Marvin_Kine()
        self.kine_right = Marvin_Kine()

        config_result_left = self.kine_left.load_config(config_path)
        if not config_result_left:
            raise RuntimeError("无法加载左臂配置")

        self.kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result_left['TYPE'][0],
            dh=config_result_left['DH'][0],
            pnva=config_result_left['PNVA'][0],
            j67=config_result_left['BD'][0]
        )

        config_result_right = self.kine_right.load_config(config_path)
        if not config_result_right:
            raise RuntimeError("无法加载右臂配置")

        self.kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result_right['TYPE'][1],
            dh=config_result_right['DH'][1],
            pnva=config_result_right['PNVA'][1],
            j67=config_result_right['BD'][1]
        )

        print("✓ 运动学库初始化成功")

        # 初始化机器人连接
        if not self.dry_run:
            print("连接机器人...")
            self.robot = Marvin_Robot()
            robot_ip = ROBOT_IP
            connection_status = self.robot.connect(robot_ip)
            if not connection_status:
                raise RuntimeError(f"无法连接到机器人 (IP: {robot_ip})")
            print(f"✓ 机器人连接成功 (IP: {robot_ip})")

            # 使能机器人（上电）
            enable_robot(self.robot, state=3, vel_ratio=60, acc_ratio=60)
        else:
            self.robot = None
            print("⚠ Dry-run 模式：不实际控制机器人")

        # zsp_para（肘部方向提示）
        self.zsp_para_left = ZSP_PARA['left']
        self.zsp_para_right = ZSP_PARA['right']

        print("="*60)
        print(f"关节空间轨迹控制 - 五次多项式插值")
        print(f"  模式: {mode}")
        print(f"  速度: {speed} m/s")
        print(f"  持续: {duration} 秒")
        print(f"  插值步长: {interpolation_dt * 1000:.0f} ms")
        print("="*60)

    def run(self):
        """运行轨迹控制"""
        print("\n开始运动...")

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
            raise ValueError(f"未知模式: {self.mode}")

        print(f"\n✓ 测试完成!")

    def disable_robot_safe(self):
        """下电机器人（安全）"""
        if not self.dry_run and self.robot is not None:
            disable_robot(self.robot)

    def _run_static(self):
        """静态测试：保持初始位姿"""
        print("保持初始位姿...")
        start_joints = {
            'left': INIT_JOINTS['left'],
            'right': INIT_JOINTS['right']
        }
        end_joints = start_joints

        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _run_align_test(self):
        """对齐测试：运动到世界坐标系对齐的姿态

        用途：验证末端坐标轴与世界坐标系是否对齐
        - 位置：保持初始位置不变
        - 姿态：设置为世界参考姿态（单位矩阵）
        - 在 RViz 中查看：world_left_dh_ee 和 world_right_dh_ee 的坐标轴
          应该与 world 坐标轴完全平行（X红 Y绿 Z蓝）
        """
        print("="*60)
        print("对齐测试：运动到世界坐标系对齐的姿态")
        print("="*60)
        print("目标：末端坐标轴与世界坐标系完全对齐")
        print("  X轴（红）→ 前")
        print("  Y轴（绿）→ 左")
        print("  Z轴（蓝）→ 上")
        print()
        print("验证方法：")
        print("  1. 运行 step3_visualize_in_rviz.py 启动可视化")
        print("  2. 在 RViz 中观察 TF 坐标轴")
        print("  3. world_left_dh_ee 和 world_right_dh_ee 应该与 world 平行")
        print("="*60)

        # 位置保持不变
        target_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }

        # 姿态设置为世界参考姿态
        # world → world_left/world_right 的旋转（使用共享工具）
        R_world_to_left = get_world_to_chest_rotation('left')
        R_world_to_right = get_world_to_chest_rotation('right')

        # 将世界参考姿态转换到各臂坐标系
        target_rot = {
            'left': R_world_to_left @ WORLD_REFERENCE_ROT,
            'right': R_world_to_right @ WORLD_REFERENCE_ROT
        }

        print("\n计算 IK...")
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints_with_rotation(target_pos, target_rot)

        if end_joints is None:
            print("❌ IK 失败，无法生成轨迹")
            return

        print("\n开始运动到对齐姿态...")
        self._execute_trajectory(start_joints, end_joints, self.duration)

        print("\n" + "="*60)
        print("✓ 已到达对齐姿态")
        print("请在 RViz 中验证末端坐标轴是否与世界坐标系平行")
        print("如果平行，说明坐标系设置正确")
        print("="*60)

    def _run_linear(self):
        """直线运动（统一 world 坐标系架构）

        架构：
          1. 在 world 坐标系统一计算方向和距离
          2. 在 _compute_target_joints 中转换到 chest 坐标系
          3. 避免重复的坐标转换代码
        """
        # 在 world 坐标系定义方向（统一）
        direction_world = get_direction_vector_world(self.mode)
        distance = self.speed * self.duration

        print(f"运动方向: {self.mode}")
        print(f"运动距离: {distance:.4f} m")
        print(f"world 方向向量: {direction_world}")

        # 在 world 坐标系计算位移（统一）
        displacement_world = direction_world * distance
        print(f"world 位移: {displacement_world}")

        # 计算目标位置（在 chest 坐标系中）
        # 注意：TIANJI_INIT_POS 已经在各自的 chest 坐标系中
        # 需要将 world 位移转换到 chest 坐标系
        start_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }

        # 将 world 位移转换到 chest 坐标系
        displacement_left_chest = transform_world_to_chest(displacement_world, 'left')
        displacement_right_chest = transform_world_to_chest(displacement_world, 'right')

        print(f"left_chest 位移: {displacement_left_chest}")
        print(f"right_chest 位移: {displacement_right_chest}")

        end_pos = {
            'left': start_pos['left'] + displacement_left_chest,
            'right': start_pos['right'] + displacement_right_chest
        }

        # 调用 IK 计算起终点关节角
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints(end_pos)

        if end_joints is None:
            print("❌ IK 失败，无法生成轨迹")
            return

        # 执行关节空间轨迹
        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _run_rotation(self):
        """旋转运动（只改变姿态，位置不变）

        用途：测试手腕旋转（RPY）
        - 位置保持不变（由平移控制）
        - 只改变姿态（由旋转控制）
        - VR 遥操作：手柄位置 → 位置，手柄姿态 → 姿态

        核心原理：
        - 左右臂在 world 坐标系中的末端姿态应该一致
        - 这样两个手腕"一起往左转"时，在 world 中看起来是同步的
        """
        # 计算旋转角度（弧度）
        rotation_angle = self.speed * self.duration  # speed 这里表示角速度 (rad/s)

        print(f"旋转方向: {self.mode}")
        print(f"旋转角度: {np.degrees(rotation_angle):.2f}° ({rotation_angle:.4f} rad)")

        # 在 world 坐标系定义旋转轴（标准定义）
        if self.mode == 'rotate_x':
            axis_world = np.array([1.0, 0.0, 0.0])  # 绕 world X 轴
            axis_name = "X轴（前后）"
        elif self.mode == 'rotate_y':
            axis_world = np.array([0.0, 1.0, 0.0])  # 绕 world Y 轴
            axis_name = "Y轴（左右）"
        elif self.mode == 'rotate_z':
            axis_world = np.array([0.0, 0.0, 1.0])  # 绕 world Z 轴
            axis_name = "Z轴（上下）"

        print(f"world 旋转轴: {axis_name} {axis_world}")
        print("关键：左右臂在 world 中的末端姿态一致")

        # 位置保持不变
        start_pos = {
            'left': TIANJI_INIT_POS['left'].copy(),
            'right': TIANJI_INIT_POS['right'].copy()
        }
        end_pos = start_pos  # 位置不变

        # === 核心算法（使用共享库 apply_world_rotation_to_chest_pose）===
        print("\n算法步骤：")

        # 步骤1: 获取当前末端姿态（在各臂坐标系中）
        current_rot_in_arm = {
            'left': TIANJI_INIT_ROT['left'],
            'right': TIANJI_INIT_ROT['right']
        }
        print("  1. 当前末端姿态（在各臂坐标系中）已知")

        # 步骤2-4: 使用共享库执行 4 步算法
        # target_chest = R_w2c @ R_delta @ R_c2w @ base_chest
        R_delta = R.from_rotvec(axis_world * rotation_angle)
        end_rot = {
            side: apply_world_rotation_to_chest_pose(
                current_rot_in_arm[side], R_delta, side
            )
            for side in ['left', 'right']
        }
        print(f"  2-4. 使用共享库 apply_world_rotation_to_chest_pose() 完成 4 步算法")
        print(f"       在 world 中旋转 {np.degrees(rotation_angle):.2f}° (绕 {axis_name})")
        print("       ✓ 左右臂应用相同的旋转增量")

        print("\n✓ 算法保证：")
        print("  - 左右臂在 world 中应用了相同的旋转")
        print("  - 旋转轴是 world 坐标系的轴")
        print("  - 位置保持不变，只改变姿态")

        # 调用 IK 计算起终点关节角
        start_joints = INIT_JOINTS
        end_joints = self._compute_target_joints_with_rotation(end_pos, end_rot)

        if end_joints is None:
            print("❌ IK 失败，无法生成轨迹")
            return

        # 执行关节空间轨迹
        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _compute_target_joints_with_rotation(self, target_pos, target_rot):
        """计算目标位置和姿态的关节角（支持旋转）"""
        result = {}

        for side in ['left', 'right']:
            # 构造位姿矩阵
            pose_mat = np.eye(4)
            pose_mat[:3, :3] = target_rot[side]
            pose_mat[:3, 3] = target_pos[side] * 1000  # 转换为毫米

            # 调用 IK
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
                print(f"  ❌ {side} 臂 IK 失败: 无法计算")
                return None

            if ik_result.m_Output_IsOutRange:
                print(f"  ❌ {side} 臂 IK 失败: 超出工作空间")
                return None

            # 转换为列表
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
        """单臂运动"""
        active_side = 'left' if self.mode == 'left_only' else 'right'
        print(f"仅 {active_side} 臂运动")

        direction = np.array([1.0, 0.0, 0.0])  # 向前
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
            print("❌ IK 失败")
            return

        self._execute_trajectory(start_joints, end_joints, self.duration)

    def _compute_target_joints(self, target_pos):
        """计算目标位置的关节角（调用 IK）"""
        result = {}

        for side in ['left', 'right']:
            # 构造位姿矩阵
            pose_mat = np.eye(4)
            pose_mat[:3, :3] = TIANJI_INIT_ROT[side]
            pose_mat[:3, 3] = target_pos[side] * 1000  # 转换为毫米

            # 调用 IK
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
                print(f"  ❌ {side} 臂 IK 失败: 无法计算")
                return None

            if ik_result.m_Output_IsOutRange:
                print(f"  ❌ {side} 臂 IK 失败: 超出工作空间")
                return None

            # 转换为列表
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
        使用五次多项式插值在关节空间执行轨迹

        五次多项式: s(t) = 10t³ - 15t⁴ + 6t⁵
        特性: s(0)=0, s(1)=1, s'(0)=0, s'(1)=0, s''(0)=0, s''(1)=0
        确保起止速度和加速度为 0
        """
        num_points = int(duration / self.interpolation_dt)

        if verbose:
            print(f"生成轨迹: {num_points} 个插值点")

        for i in range(num_points + 1):
            t = i / num_points  # 归一化时间 [0, 1]

            # 五次多项式插值
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            # 计算当前关节角
            target_left = [
                start_joints['left'][j] + s * (end_joints['left'][j] - start_joints['left'][j])
                for j in range(7)
            ]
            target_right = [
                start_joints['right'][j] + s * (end_joints['right'][j] - start_joints['right'][j])
                for j in range(7)
            ]

            # 发送指令
            if not self.dry_run:
                send_joint_command(self.robot, target_left, target_right)

            # 进度显示
            if verbose and i % max(1, num_points // 10) == 0:
                progress = int(t * 100)
                print(f"  [{progress:3d}%] t={i*self.interpolation_dt:.2f}s", flush=True)

            time.sleep(self.interpolation_dt)


def main():
    parser = argparse.ArgumentParser(description='天机机器人关节空间轨迹控制测试')
    parser.add_argument('--mode', type=str, default='static',
                        choices=['static', 'align_test',
                                'forward', 'back', 'left', 'right', 'up', 'down',
                                'rotate_x', 'rotate_y', 'rotate_z',
                                'left_only', 'right_only'],
                        help='测试模式')
    parser.add_argument('--speed', type=float, default=0.02,
                        help='移动速度 (m/s) 或角速度 (rad/s，用于旋转)')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='持续时间 (秒)')
    parser.add_argument('--dt', type=float, default=0.02,
                        help='插值时间步长 (秒)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Dry-run 模式（不实际控制机器人）')

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
        print("\n中断测试")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保机器人下电
        if controller is not None:
            controller.disable_robot_safe()


if __name__ == '__main__':
    main()
