#!/usr/bin/env python3
"""
笛卡尔位姿控制器 - 封装类 / Cartesian Pose Controller
简化使用末端位姿控制机器人的流程
"""
from .fx_robot import Marvin_Robot
from .fx_kine import Marvin_Kine
from .structure_data import DCSS
import time
import logging
import os
from ament_index_python.packages import get_package_share_directory


class CartesianController:
    """笛卡尔空间控制器 / Cartesian Space Controller"""

    def __init__(self, robot_ip='192.168.1.190', config_path=None, logger=None):
        """
        初始化笛卡尔控制器（同时初始化左右两臂）

        Args:
            robot_ip: 机器人IP地址
            config_path: 运动学配置文件路径
                - None: 使用默认的 'ccs_m6.MvKDCfg'（从ROS2包中自动查找）
                - 相对路径: 从ROS2包的config目录中查找
                - 绝对路径: 直接使用该路径
            logger: 外部传入的 logger（可选，用于集成 ROS2 日志系统）
        """
        # 日志：优先使用外部传入的 logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger(__name__)

        # 解析配置文件路径
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'

            package_share = get_package_share_directory('tianji_world_output')
            config_path = os.path.join(package_share, 'config', config_filename)
        # 检查文件是否存在
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"配置文件不存在: {config_path}")


        self.logger.debug("加载配置文件: %s", config_path)

        # 初始化左臂运动学
        self.logger.debug("[A臂] 初始化运动学SDK...")
        self.kine_left = Marvin_Kine()
        config_result = self.kine_left.load_config(config_path=config_path)
        time.sleep(0.3)
        self.kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result['TYPE'][0],
            dh=config_result['DH'][0],
            pnva=config_result['PNVA'][0],
            j67=config_result['BD'][0]
        )

        # 初始化右臂运动学
        self.logger.debug("[B臂] 初始化运动学SDK...")
        self.kine_right = Marvin_Kine()
        config_result = self.kine_right.load_config(config_path=config_path)
        time.sleep(0.3)
        self.kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result['TYPE'][1],
            dh=config_result['DH'][1],
            pnva=config_result['PNVA'][1],
            j67=config_result['BD'][1]
        )

        # 初始化机器人连接
        self.logger.debug("初始化机器人控制...")
        self.robot = Marvin_Robot()

        init = self.robot.connect(robot_ip)
        if init == 0:
            raise ConnectionError("连接失败：端口占用!")

        time.sleep(0.5)
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)

        if not self._verify_connection():
            raise ConnectionError("机器人连接失败!")

        # IK 参数（可实时修改）— 从统一配置加载
        from tianji_world_output.config_loader import TianjiConfig
        self._cfg = TianjiConfig.load(use_ros=False)
        self.zsp_type = self._cfg.zsp_type
        left_zsp = self._cfg.default_zsp_para.get('left', [0, -1, -0.5, 0, 0, 0])
        right_zsp = self._cfg.default_zsp_para.get('right', [0, 1, -0.5, 0, 0, 0])
        self.left_zsp_para = list(left_zsp)
        self.right_zsp_para = list(right_zsp)
        self.zsp_angle = self._cfg.zsp_angle
        self.dgr = list(self._cfg.dgr)

        # IK 种子: 上一次成功的关节角, 每帧更新, 保证构型连续
        default_left = list(self._cfg.init_joints.get('left', [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]))
        default_right = list(self._cfg.init_joints.get('right', [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]))
        self._last_valid_left_joints = default_left
        self._last_valid_right_joints = default_right

        # 设置工具参数 (wuji hand)
        self._set_tool_params()

        self.logger.info("✅ 双臂初始化完成")

    def _set_tool_params(self):
        """
        设置工具参数（运动学 + 动力学）

        运动学参数 [X, Y, Z, A, B, C]:
            X, Y, Z: 工具中心点相对于法兰的位置偏移 (mm)
            A, B, C: 工具相对于法兰的姿态偏移 (度)

        动力学参数 [M, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]:
            M: 工具质量 (kg)
            mx, my, mz: 质心相对于法兰的位置 (mm)
            Ixx~Izz: 惯性张量 (kg·mm²)
        """
        # Wuji Hand 负载参数 (与 tianji_arm_controller.py 一致)
        tool_kine = [0, 0, 120, 0, 0, 0]  # 工具中心点距法兰 120mm
        tool_dyn = [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0]  # 质量 0.95kg, 质心距法兰 90mm

        self.logger.info("设置工具参数 (Wuji Hand): kine=%s, dyn=%s", tool_kine, tool_dyn)

        self.robot.clear_set()
        self.robot.set_tool(arm='A', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.set_tool(arm='B', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.send_cmd()
        time.sleep(0.3)

    def _verify_connection(self):
        """验证机器人连接"""
        dcss = DCSS()
        motion_tag = 0
        frame_update = None
        for i in range(5):
            sub_data = self.robot.subscribe(dcss)
            # 检查左臂连接
            serial = sub_data['outputs'][0]['frame_serial']
            if serial != 0 and frame_update != serial:
                motion_tag += 1
                frame_update = serial
            time.sleep(0.1)
        return motion_tag > 0

    def get_current_joints(self):
        """获取当前双臂关节角度"""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """设置双臂阻抗模式"""
        # 设置双臂状态
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=3)
        self.robot.set_state(arm='B', state=3)
        self.robot.set_vel_acc(arm='A', velRatio=60, AccRatio=60)
        self.robot.set_vel_acc(arm='B', velRatio=60, AccRatio=60)
        self.robot.send_cmd()
        time.sleep(0.5)

        if mode == 'cart':
            K = K or [8000, 8000, 8000, 100, 100, 100, 20]
            D = D or [0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4]

            self.robot.clear_set()
            self.robot.set_cart_kd_params(arm='A', K=K, D=D, type=2)
            self.robot.set_cart_kd_params(arm='B', K=K, D=D, type=2)
            time.sleep(0.5)
            self.robot.set_impedance_type(arm='A', type=2)
            self.robot.set_impedance_type(arm='B', type=2)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info("双臂笛卡尔阻抗模式 K=%s", K)

        elif mode == 'joint':
            K = K or [2,2,2,1.6, 1, 1, 1]
            D = D or [0.3,0.3,0.3,0.2,0.2,0.2,0.2]

            self.robot.clear_set()
            self.robot.set_joint_kd_params(arm='A', K=K, D=D)
            self.robot.set_joint_kd_params(arm='B', K=K, D=D)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.robot.clear_set()
            self.robot.set_impedance_type(arm='A', type=1)
            self.robot.set_impedance_type(arm='B', type=1)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info("双臂关节阻抗模式 K=%s", K)

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01,
                     init_joints_left=None, init_joints_right=None):
        """
        双臂同时移动到初始位姿（使用关节空间轨迹插值实现柔顺运动）

        Args:
            wait: 是否等待运动完成
            timeout: 到达后额外等待时间（秒）
            duration: 轨迹总时长（秒），越大越慢越柔顺
            dt: 插值时间步长（秒）
            init_joints_left: 左臂初始关节角（度），None 使用默认值
            init_joints_right: 右臂初始关节角（度），None 使用默认值

        Returns:
            bool: 是否成功
        """
        # 默认初始关节角度 — 从 tianji_robot.yaml 统一加载
        DEFAULT_INIT_LEFT = list(self._cfg.init_joints.get('left', [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]))
        DEFAULT_INIT_RIGHT = list(self._cfg.init_joints.get('right', [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]))

        INIT_JOINTS_LEFT = init_joints_left if init_joints_left is not None else DEFAULT_INIT_LEFT
        INIT_JOINTS_RIGHT = init_joints_right if init_joints_right is not None else DEFAULT_INIT_RIGHT

        self.logger.debug("双臂移动到初始位姿（%ss 柔顺轨迹）...", duration)

        # 获取当前关节角度
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)
        num_points = int(duration / dt)

        # 使用五次多项式插值生成柔顺轨迹（起止速度和加速度为0）
        for i in range(num_points + 1):
            t = i / num_points  # 归一化时间 [0, 1]
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            target_left = [
                start_left[j] + s * (INIT_JOINTS_LEFT[j] - start_left[j])
                for j in range(7)
            ]
            target_right = [
                start_right[j] + s * (INIT_JOINTS_RIGHT[j] - start_right[j])
                for j in range(7)
            ]

            # 发送双臂关节指令
            self.robot.clear_set()
            self.robot.set_joint_cmd_pose(arm='A', joints=target_left)
            self.robot.set_joint_cmd_pose(arm='B', joints=target_right)
            self.robot.send_cmd()

            time.sleep(dt)

        if wait:
            time.sleep(timeout)

        # 验证是否到达初始位姿
        final_left, final_right = self.get_current_joints()
        left_errors = [abs(final_left[i] - INIT_JOINTS_LEFT[i]) for i in range(7)]
        right_errors = [abs(final_right[i] - INIT_JOINTS_RIGHT[i]) for i in range(7)]
        max_left_error = max(left_errors)
        max_right_error = max(right_errors)

        success = True
        if max_left_error < 5.0:
            self.logger.debug("[A臂] 已到达初始位姿")
        else:
            self.logger.warning("[A臂] 初始位姿误差较大 (%.1f°)", max_left_error)
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[B臂] 已到达初始位姿")
        else:
            self.logger.warning("[B臂] 初始位姿误差较大 (%.1f°)", max_right_error)
            success = False

        # 设置 IK 种子为初始关节角
        self._last_valid_left_joints = list(INIT_JOINTS_LEFT)
        self._last_valid_right_joints = list(INIT_JOINTS_RIGHT)
        self.logger.info("IK 种子已设置")

        return success

    def move_to_pose_direct(self, left_pose, right_pose, unit='mm'):
        """
        双臂同时 IK 解算并发送关节指令（非阻塞，用于实时追踪）

        Args:
            left_pose: 左臂目标位姿，None 表示不控制左臂
                      - 若 unit='matrix': 4x4 numpy 矩阵 (米)
                      - 若 unit='mm'/'m': [X, Y, Z, RX, RY, RZ] (毫米或米)
            right_pose: 右臂目标位姿，None 表示不控制右臂 (格式同上)
            unit: 'matrix' (4x4 矩阵), 'mm' (毫米) 或 'm' (米)

        Returns:
            tuple: (left_success, right_success, left_joints, right_joints)
        """
        left_mat = self._convert_pose_to_mat(left_pose, unit, self.kine_left) if left_pose is not None else None
        right_mat = self._convert_pose_to_mat(right_pose, unit, self.kine_right) if right_pose is not None else None

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

        if left_mat is not None:
            left_joints, left_success = self._solve_ik(
                self.kine_left, 0, left_mat,
                self._last_valid_left_joints, 'A'
            )
            if left_success:
                self._last_valid_left_joints = list(left_joints)

        if right_mat is not None:
            right_joints, right_success = self._solve_ik(
                self.kine_right, 1, right_mat,
                self._last_valid_right_joints, 'B'
            )
            if right_success:
                self._last_valid_right_joints = list(right_joints)

        self.robot.clear_set()
        if left_joints is not None:
            self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
        if right_joints is not None:
            self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
        self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    def _convert_pose_to_mat(self, pose, unit, kine):
        """将位姿转换为 IK 所需的矩阵格式 (mm)"""
        if unit == 'matrix':
            mat = pose.copy()
            mat[:3, 3] *= 1000  # m → mm
            return mat.tolist()
        else:
            pose_mm = list(pose)
            if unit == 'm':
                for i in range(3):
                    pose_mm[i] *= 1000
            return kine.xyzabc_to_mat4x4(pose_mm)

    def _solve_ik(self, kine, robot_serial, pose_mat, ref_joints, arm_name):
        """
        IK 求解 (两步回退)

        1. zsp_type=1 (含肘部约束)
        2. zsp_type=0 (最小化与种子的关节距离)
        信任 IK 求解器: 有效解直接采用, 种子每帧更新保证连续性。
        """
        zsp_para = self.left_zsp_para if arm_name == 'A' else self.right_zsp_para

        # 优先: 带肘部约束
        joints = self._call_ik(kine, robot_serial, pose_mat, ref_joints,
                                self.zsp_type, zsp_para, arm_name)
        if joints is not None:
            return joints, True

        # 回退: 最小化关节距离
        if self.zsp_type != 0:
            joints = self._call_ik(kine, robot_serial, pose_mat, ref_joints,
                                    0, zsp_para, arm_name)
            if joints is not None:
                return joints, True

        return None, False

    def _call_ik(self, kine, robot_serial, pose_mat, ref_joints,
                  zsp_type, zsp_para, arm_name):
        """单次 IK 调用, 返回关节角 list 或 None"""
        try:
            ik_result = kine.ik(
                robot_serial=robot_serial,
                pose_mat=pose_mat,
                ref_joints=ref_joints,
                zsp_type=zsp_type,
                zsp_para=zsp_para,
                zsp_angle=self.zsp_angle,
                dgr=self.dgr
            )
            if ik_result is not False:
                if not ik_result.m_Output_IsOutRange and not ik_result.m_Output_IsJntExd:
                    return ik_result.m_Output_RetJoint.to_list()
        except Exception as e:
            self.logger.warning("[%s臂 IK] Exception: %s", arm_name, e)
        return None

    def disable_and_release(self):
        """下使能并释放双臂 / Disable and release both arms"""
        self.logger.info("双臂下使能 / Disabling arms...")
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=0)
        self.robot.set_state(arm='B', state=0)
        self.robot.send_cmd()
        time.sleep(2)

        self.logger.debug("释放连接...")
        self.robot.release_robot()
        self.logger.info("已安全退出 / Safely exited")
