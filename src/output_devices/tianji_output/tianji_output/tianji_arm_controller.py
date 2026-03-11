#!/usr/bin/env python3
"""
天机臂统一控制器 - 封装类 / Tianji Arm Unified Controller

整合笛卡尔空间控制和关节空间控制：
- 笛卡尔空间控制：通过末端位姿控制（IK 解算）
- 关节空间控制：直接使用关节角度控制
"""
try:
    from tianji_output._internal.fx_robot import Marvin_Robot
    from tianji_output._internal.fx_kine import Marvin_Kine
    from tianji_output._internal.structure_data import DCSS
except ImportError:
    from ._internal.fx_robot import Marvin_Robot
    from ._internal.fx_kine import Marvin_Kine
    from ._internal.structure_data import DCSS
import time
import logging
import os
from ament_index_python.packages import get_package_share_directory


class TianjiArmController:
    """
    天机臂统一控制器 / Tianji Arm Unified Controller

    同时支持笛卡尔空间控制和关节空间控制。
    """

    def __init__(self, robot_ip='192.168.1.190', config_path=None, logger=None):
        """
        初始化天机臂控制器（同时初始化左右两臂）

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
            self.logger = logging.getLogger('TianjiArmController')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        # 解析配置文件路径
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'
            package_share = get_package_share_directory('tianji_output')
            config_path = os.path.join(package_share, 'config', config_filename)

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"配置文件不存在: {config_path}")

        self.logger.debug(f"加载配置文件: {config_path}")

        # ---------------------- 初始化运动学 ----------------------
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

        # ---------------------- 初始化机器人连接 ----------------------
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

        # 保存为共享实例
        TianjiArmController._shared_robot = self.robot
        TianjiArmController._shared_kine_left = self.kine_left
        TianjiArmController._shared_kine_right = self.kine_right
        TianjiArmController._initialized = True

        # ---------------------- IK 参数（可实时修改）----------------------
        self.zsp_type = 1                           # 零空间约束类型
        self.left_zsp_para = [0, -1, -1, 0, 0, 0]   # 左臂零空间参考平面参数
        self.right_zsp_para = [0, 1, -1, 0, 0, 0]   # 右臂零空间参考平面参数
        self.zsp_angle = 0.0                        # 零空间臂角旋转角度
        self.dgr = [5.0, 5.0, 5.0]                  # 奇异允许角度范围

        # 设置工具参数 (wuji hand)
        self._set_tool_params()

        self.logger.info("双臂控制器初始化完成")

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
        tool_kine = [0, 0, 120, 0, 0, 0]  # 工具中心点距法兰 120mm
        tool_dyn = [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0]  # 质量 0.95kg, 质心距法兰 90mm

        self.logger.debug(f"设置工具参数: kine={tool_kine}, dyn={tool_dyn}")

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
            serial = sub_data['outputs'][0]['frame_serial']
            if serial != 0 and frame_update != serial:
                motion_tag += 1
                frame_update = serial
            time.sleep(0.1)
        return motion_tag > 0

    # ==================== 状态获取方法 ====================

    def get_current_joints(self):
        """
        获取当前双臂关节角度

        Returns:
            tuple: (left_joints, right_joints) 各为7个关节角度的列表 (度)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def get_current_joint_velocities(self):
        """
        获取当前双臂关节速度

        Returns:
            tuple: (left_velocities, right_velocities) 各为7个关节速度的列表 (度/秒)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_vel = sub_data["outputs"][0]["fb_joint_vel"]
        right_vel = sub_data["outputs"][1]["fb_joint_vel"]
        return left_vel, right_vel

    def get_current_joint_torques(self):
        """
        获取当前双臂关节力矩

        Returns:
            tuple: (left_torques, right_torques) 各为7个关节力矩的列表 (Nm)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_torque = sub_data["outputs"][0]["fb_joint_tor"]
        right_torque = sub_data["outputs"][1]["fb_joint_tor"]
        return left_torque, right_torque

    def get_full_state(self):
        """
        获取完整的机器人状态

        Returns:
            dict: 包含双臂关节位置、速度、力矩等信息
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return {
            'left': {
                'joints': sub_data["outputs"][0]["fb_joint_pos"],
                'velocities': sub_data["outputs"][0]["fb_joint_vel"],
                'torques': sub_data["outputs"][0]["fb_joint_tor"],
            },
            'right': {
                'joints': sub_data["outputs"][1]["fb_joint_pos"],
                'velocities': sub_data["outputs"][1]["fb_joint_vel"],
                'torques': sub_data["outputs"][1]["fb_joint_tor"],
            }
        }

    # ==================== 阻抗模式设置 ====================

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """
        设置双臂阻抗模式

        Args:
            mode: 'joint' 关节阻抗 或 'cart' 笛卡尔阻抗
            K: 刚度参数列表 (7个元素)
            D: 阻尼参数列表 (7个元素)
        """
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

            self.logger.info(f"双臂笛卡尔阻抗模式 K={K}")

        elif mode == 'joint':
            K = K or [2, 2, 2, 1.6, 1, 1, 1]
            D = D or [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]

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

            self.logger.info(f"双臂关节阻抗模式 K={K}")

    # ==================== 笛卡尔空间控制方法 ====================

    def move_to_pose_direct(self, left_pose=None, right_pose=None, unit='mm'):
        """
        笛卡尔空间控制：双臂同时 IK 解算并发送关节指令（非阻塞，用于实时追踪）

        Args:
            left_pose: [X, Y, Z, RX, RY, RZ] 左臂目标位姿，None 表示不控制左臂
            right_pose: [X, Y, Z, RX, RY, RZ] 右臂目标位姿，None 表示不控制右臂
            unit: 'mm' (毫米) 或 'm' (米)

        Returns:
            tuple: (left_success, right_success, left_joints, right_joints)
        """
        # 转换单位为 mm
        left_mm = None
        right_mm = None
        if left_pose is not None:
            left_mm = list(left_pose)
            if unit == 'm':
                for i in range(3):
                    left_mm[i] *= 1000
        if right_pose is not None:
            right_mm = list(right_pose)
            if unit == 'm':
                for i in range(3):
                    right_mm[i] *= 1000

        # 获取当前关节作为 IK 参考
        ref_left, ref_right = self.get_current_joints()

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

        # 左臂 IK 解算
        if left_mm is not None:
            try:
                left_mat = self.kine_left.xyzabc_to_mat4x4(left_mm)
                left_ik = self.kine_left.ik(
                    robot_serial=0,
                    pose_mat=left_mat,
                    ref_joints=ref_left,
                    zsp_type=self.zsp_type,
                    zsp_para=self.left_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr
                )
                if left_ik is not False:
                    if not left_ik.m_Output_IsOutRange and not left_ik.m_Output_IsJntExd:
                        left_joints = left_ik.m_Output_RetJoint.to_list()
                        left_success = True
            except Exception as e:
                self.logger.debug(f"左臂 IK 解算异常: {e}")

        # 右臂 IK 解算
        if right_mm is not None:
            try:
                right_mat = self.kine_right.xyzabc_to_mat4x4(right_mm)
                right_ik = self.kine_right.ik(
                    robot_serial=1,
                    pose_mat=right_mat,
                    ref_joints=ref_right,
                    zsp_type=self.zsp_type,
                    zsp_para=self.right_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr
                )
                if right_ik is not False:
                    if not right_ik.m_Output_IsOutRange and not right_ik.m_Output_IsJntExd:
                        right_joints = right_ik.m_Output_RetJoint.to_list()
                        right_success = True
            except Exception as e:
                self.logger.debug(f"右臂 IK 解算异常: {e}")

        # IK 调试输出
        if left_joints is not None:
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT_IK]  joints: [{left_joints_str}]")
        else:
            self.logger.debug(f"[LEFT_IK]  FAILED!")

        if right_joints is not None:
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT_IK] joints: [{right_joints_str}]")
        else:
            self.logger.debug(f"[RIGHT_IK] FAILED!")

        # 发送双臂关节指令
        self.robot.clear_set()
        if left_joints is not None:
            self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
        if right_joints is not None:
            self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
        self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    # ==================== 关节空间控制方法 ====================

    def move_to_joints_direct(self, left_joints=None, right_joints=None):
        """
        关节空间控制：双臂同时发送关节角度指令（非阻塞，用于实时追踪）

        Args:
            left_joints: [j1, j2, j3, j4, j5, j6, j7] 左臂目标关节角度 (度)，None 表示不控制左臂
            right_joints: [j1, j2, j3, j4, j5, j6, j7] 右臂目标关节角度 (度)，None 表示不控制右臂

        Returns:
            tuple: (left_success, right_success)
        """
        left_success = left_joints is not None
        right_success = right_joints is not None

        self.robot.clear_set()
        if left_joints is not None:
            self.robot.set_joint_cmd_pose(arm='A', joints=list(left_joints))
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT]  joints: [{left_joints_str}]")
        if right_joints is not None:
            self.robot.set_joint_cmd_pose(arm='B', joints=list(right_joints))
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT] joints: [{right_joints_str}]")
        self.robot.send_cmd()

        return left_success, right_success

    def move_to_joints_smooth(self, left_target=None, right_target=None, duration=3.0, dt=0.01):
        """
        双臂平滑移动到目标关节角度（使用五次多项式插值）

        Args:
            left_target: [j1, j2, j3, j4, j5, j6, j7] 左臂目标关节角度 (度)，None 表示不控制左臂
            right_target: [j1, j2, j3, j4, j5, j6, j7] 右臂目标关节角度 (度)，None 表示不控制右臂
            duration: 轨迹总时长（秒），越大越慢越柔顺
            dt: 插值时间步长（秒）

        Returns:
            bool: 是否成功
        """
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)

        if left_target is None:
            left_target = start_left
        if right_target is None:
            right_target = start_right

        num_points = int(duration / dt)

        self.logger.debug(f"平滑移动到目标位置（{duration}s 柔顺轨迹）...")

        for i in range(num_points + 1):
            t = i / num_points
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            target_left = [
                start_left[j] + s * (left_target[j] - start_left[j])
                for j in range(7)
            ]
            target_right = [
                start_right[j] + s * (right_target[j] - start_right[j])
                for j in range(7)
            ]

            self.robot.clear_set()
            self.robot.set_joint_cmd_pose(arm='A', joints=target_left)
            self.robot.set_joint_cmd_pose(arm='B', joints=target_right)
            self.robot.send_cmd()

            time.sleep(dt)

        return True

    # ==================== 初始位姿和释放 ====================

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01):
        """
        双臂同时移动到初始位姿（使用关节空间轨迹插值实现柔顺运动）

        Args:
            wait: 是否等待运动完成
            timeout: 到达后额外等待时间（秒）
            duration: 轨迹总时长（秒），越大越慢越柔顺
            dt: 插值时间步长（秒）

        Returns:
            bool: 是否成功
        """
        INIT_JOINTS_LEFT = [56.9, -63.0, -46.8, -87.8, 143.2, -4.1, -45.6]
        INIT_JOINTS_RIGHT = [-50.9, -70.5, 42.6, -80.3, -140.1, -5.5, 38.9]

        self.logger.debug(f"双臂移动到初始位姿（{duration}s 柔顺轨迹）...")

        self.move_to_joints_smooth(
            left_target=INIT_JOINTS_LEFT,
            right_target=INIT_JOINTS_RIGHT,
            duration=duration,
            dt=dt
        )

        if wait:
            time.sleep(timeout)

        final_left, final_right = self.get_current_joints()
        left_errors = [abs(final_left[i] - INIT_JOINTS_LEFT[i]) for i in range(7)]
        right_errors = [abs(final_right[i] - INIT_JOINTS_RIGHT[i]) for i in range(7)]
        max_left_error = max(left_errors)
        max_right_error = max(right_errors)

        success = True
        if max_left_error < 5.0:
            self.logger.debug(f"[A臂] 已到达初始位姿")
        else:
            self.logger.warning(f"[A臂] 初始位姿误差较大 ({max_left_error:.1f}°)")
            success = False

        if max_right_error < 5.0:
            self.logger.debug(f"[B臂] 已到达初始位姿")
        else:
            self.logger.warning(f"[B臂] 初始位姿误差较大 ({max_right_error:.1f}°)")
            success = False

        return success

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
