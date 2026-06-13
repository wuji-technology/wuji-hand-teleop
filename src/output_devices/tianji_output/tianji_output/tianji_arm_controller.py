#!/usr/bin/env python3
"""
Tianji Arm Unified Controller

Integrates Cartesian space control and joint space control:
- Cartesian space control: via end-effector pose control (IK solving)
- Joint space control: direct joint angle control
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
    Tianji Arm Unified Controller

    Supports both Cartesian space control and joint space control.
    """

    def __init__(self, robot_ip='192.168.1.190', config_path=None, logger=None):
        """
        Initialize Tianji arm controller (initializes both left and right arms)

        Args:
            robot_ip: Robot IP address
            config_path: Kinematics configuration file path
                - None: Use default 'ccs_m6.MvKDCfg'(auto-searched from ROS2 package)
                - Relative path: search from ROS2 package config directory
                - Absolute path: use this path directly
            logger: External logger (optional, for integrating ROS2 logging system)
        """
        # Logging: prefer externally provided logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('TianjiArmController')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        # Parse configuration file path
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'
            package_share = get_package_share_directory('tianji_output')
            config_path = os.path.join(package_share, 'config', config_filename)

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file does not exist: {config_path}")

        self.logger.debug(f"Loading configuration file: {config_path}")

        # ---------------------- Initialize kinematics ----------------------
        # Initialize left arm kinematics
        self.logger.debug("[Arm A] Initializing kinematics SDK...")
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

        # Initialize right arm kinematics
        self.logger.debug("[Arm B] Initializing kinematics SDK...")
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

        # ---------------------- Initialize robot connection ----------------------
        self.logger.debug("Initializing robot control...")
        self.robot = Marvin_Robot()

        init = self.robot.connect(robot_ip)
        if init == 0:
            raise ConnectionError("Connection failed: port occupied!")

        time.sleep(0.5)
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)

        if not self._verify_connection():
            raise ConnectionError("Robot connection failed!")

        # Save as shared instance
        TianjiArmController._shared_robot = self.robot
        TianjiArmController._shared_kine_left = self.kine_left
        TianjiArmController._shared_kine_right = self.kine_right
        TianjiArmController._initialized = True

        # ---------------------- IK parameters (can be modified in real-time)----------------------
        self.zsp_type = 1                           # Nullspace constraint type
        self.left_zsp_para = [0, -1, -1, 0, 0, 0]   # Left arm nullspace reference plane parameters
        self.right_zsp_para = [0, 1, -1, 0, 0, 0]   # Right arm nullspace reference plane parameters
        self.zsp_angle = 0.0                        # Nullspace arm angle rotation angle
        self.dgr = [5.0, 5.0, 5.0]                  # Singularity tolerance angle range

        # Set tool parameters (wuji hand)
        self._set_tool_params()

        self.logger.info("Dual-arm controller initialization complete")

    def _set_tool_params(self):
        # IK solves at the flange (tool_kine zero); mounted-tool kinematics
        # belong in URDF / chest TF, not in the SDK's set_tool matrix.
        tool_kine = [0, 0, 0, 0, 0, 0]
        # Identified Wuji Hand payload (M kg, mr mm, I kg·mm²); rewrite if
        # you mount a different end-effector.
        tool_dyn = [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0]

        self.logger.debug(f"Setting tool parameters: kine={tool_kine}, dyn={tool_dyn}")

        self.robot.clear_set()
        self.robot.set_tool(arm='A', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.set_tool(arm='B', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.send_cmd()
        time.sleep(0.3)

    def _verify_connection(self):
        """Verify robot connection"""
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

    # ==================== State Retrieval Methods ====================

    def get_current_joints(self):
        """
        Get current dual-arm joint angles

        Returns:
            tuple: (left_joints, right_joints) each is a list of 7 joint angles (degrees)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def get_current_joint_velocities(self):
        """
        Get current dual-arm joint velocities

        Returns:
            tuple: (left_velocities, right_velocities) each is a list of 7 joint velocities (degrees/second)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_vel = sub_data["outputs"][0]["fb_joint_vel"]
        right_vel = sub_data["outputs"][1]["fb_joint_vel"]
        return left_vel, right_vel

    def get_current_joint_torques(self):
        """
        Get current dual-arm joint torques

        Returns:
            tuple: (left_torques, right_torques) each is a list of 7 joint torques (Nm)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_torque = sub_data["outputs"][0]["fb_joint_tor"]
        right_torque = sub_data["outputs"][1]["fb_joint_tor"]
        return left_torque, right_torque

    def get_full_state(self):
        """
        Get full robot state

        Returns:
            dict: Contains dual-arm joint position, velocity, torque and other information
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

    def get_arm_states_only(self):
        """Lightweight: (left_cur_state, right_cur_state) ints in one SDK subscribe."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return (sub_data["states"][0]["cur_state"], sub_data["states"][1]["cur_state"])

    def get_arm_status(self):
        """Dual-arm state + servo fault codes + frame realtime metrics.

        Returns dict {'left': {...}, 'right': {...}} where each side has
        state, err_code, servo_errors (7 hex), servo_descriptions (7 strings),
        realtime (frame_miss_cnt, quality, etc.).
        """
        from .fault_codes import describe_fault_codes, read_servo_fault_codes
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        result = {}
        for idx, (arm_id, side) in enumerate([('A', 'left'), ('B', 'right')]):
            servo_errors = read_servo_fault_codes(self.robot.robot, arm_id)
            inputs = sub_data.get('inputs') or []
            rt_in = inputs[idx] if idx < len(inputs) else {}
            frame_miss_cnt = int(rt_in.get('frame_miss_cnt', 0))
            result[side] = {
                'state': sub_data["states"][idx]["cur_state"],
                'err_code': sub_data["states"][idx]["err_code"],
                'servo_errors': servo_errors,
                'servo_descriptions': describe_fault_codes(servo_errors, empty=''),
                'realtime': {
                    'frame_miss_cnt': frame_miss_cnt,
                    'max_frame_miss_cnt': int(rt_in.get('max_frame_miss_cnt', 0)),
                    'in_frame_serial': int(rt_in.get('in_frame_serial', 0)),
                    'sys_cyc_miss_cnt': int(rt_in.get('sys_cyc_miss_cnt', 0)),
                    'max_sys_cyc_miss_cnt': int(rt_in.get('max_sys_cyc_miss_cnt', 0)),
                    'quality': 'good' if frame_miss_cnt < 20 else 'poor',
                },
            }
        return result

    # ==================== Impedance Mode Setup ====================

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """
        Set dual-arm impedance mode

        Args:
            mode: 'joint' joint impedance or 'cart' Cartesian impedance
            K: Stiffness parameter list (7 elements)
            D: Damping parameter list (7 elements)
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

            self.logger.info(f"Dual-arm Cartesian impedance mode K={K}")

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

            self.logger.info(f"Dual-arm joint impedance mode K={K}")

    # ==================== Cartesian Space Control Methods ====================

    def move_to_pose_direct(self, left_pose=None, right_pose=None, unit='mm'):
        """
        Cartesian space control: simultaneous dual-arm IK solving and joint command sending (non-blocking, for real-time tracking)

        Args:
            left_pose: [X, Y, Z, RX, RY, RZ] Left arm target pose, None means do not control left arm
            right_pose: [X, Y, Z, RX, RY, RZ] Right arm target pose, None means do not control right arm
            unit: 'mm' (millimeters) or 'm' (meters)

        Returns:
            tuple: (left_success, right_success, left_joints, right_joints)
        """
        # Convert units to mm
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

        # Get current joints as IK reference
        ref_left, ref_right = self.get_current_joints()

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

        # Left arm IK solving
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
                self.logger.debug(f"Left arm IK solving exception: {e}")

        # Right arm IK solving
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
                self.logger.debug(f"Right arm IK solving exception: {e}")

        # IK debug output
        if left_joints is not None:
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT_IK]  joints: [{left_joints_str}]")
        else:
            self.logger.debug("[LEFT_IK]  FAILED!")

        if right_joints is not None:
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT_IK] joints: [{right_joints_str}]")
        else:
            self.logger.debug("[RIGHT_IK] FAILED!")

        # Send dual-arm joint commands
        self.robot.clear_set()
        if left_joints is not None:
            self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
        if right_joints is not None:
            self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
        self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    # ==================== Joint Space Control Methods ====================

    def move_to_joints_direct(self, left_joints=None, right_joints=None):
        """
        Joint space control: simultaneous dual-arm joint angle command sending (non-blocking, for real-time tracking)

        Args:
            left_joints: [j1, j2, j3, j4, j5, j6, j7] left arm target joint angles (degrees), None means do not control left arm
            right_joints: [j1, j2, j3, j4, j5, j6, j7] right arm target joint angles (degrees), None means do not control right arm

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
        Smooth dual-arm movement to target joint angles (using quintic polynomial interpolation)

        Args:
            left_target: [j1, j2, j3, j4, j5, j6, j7] left arm target joint angles (degrees), None means do not control left arm
            right_target: [j1, j2, j3, j4, j5, j6, j7] right arm target joint angles (degrees), None means do not control right arm
            duration: Total trajectory duration (seconds), larger = slower and smoother
            dt: Interpolation time step (seconds)

        Returns:
            bool: Whether successful
        """
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)

        if left_target is None:
            left_target = start_left
        if right_target is None:
            right_target = start_right

        num_points = int(duration / dt)

        self.logger.debug(f"Smoothly move to target position({duration}s compliant trajectory)...")

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

    # ==================== Initial Pose and Release ====================

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01):
        """
        Move both arms to initial pose simultaneously (using joint space trajectory interpolation for compliant motion)

        Args:
            wait: Whether to wait for motion completion
            timeout: Additional wait time after reaching (seconds)
            duration: Total trajectory duration (seconds), larger = slower and smoother
            dt: Interpolation time step (seconds)

        Returns:
            bool: Whether successful
        """
        INIT_JOINTS_LEFT = [56.9, -63.0, -46.8, -87.8, 143.2, -4.1, -45.6]
        INIT_JOINTS_RIGHT = [-50.9, -70.5, 42.6, -80.3, -140.1, -5.5, 38.9]

        self.logger.debug(f"Moving both arms to initial pose({duration}s compliant trajectory)...")

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
            self.logger.debug("[Arm A] Reached initial pose")
        else:
            self.logger.warning(f"[Arm A] Large initial pose error ({max_left_error:.1f}°)")
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[Arm B] Reached initial pose")
        else:
            self.logger.warning(f"[Arm B] Large initial pose error ({max_right_error:.1f}°)")
            success = False

        return success

    def disable_and_release(self):
        """Disable and release both arms"""
        self.logger.info("Disabling both arms...")
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=0)
        self.robot.set_state(arm='B', state=0)
        self.robot.send_cmd()
        time.sleep(2)

        self.logger.debug("Releasing connection...")
        self.robot.release_robot()
        self.logger.info("Safely exited")
