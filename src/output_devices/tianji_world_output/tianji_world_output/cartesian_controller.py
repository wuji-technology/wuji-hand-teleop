#!/usr/bin/env python3
"""
Cartesian Pose Controller - Wrapper Class
Simplifies robot end-effector pose control workflow
"""
from .fx_robot import Marvin_Robot
from .fx_kine import Marvin_Kine
from .structure_data import DCSS
import time
import logging
import os
from ament_index_python.packages import get_package_share_directory


class CartesianController:
    """Cartesian Space Controller"""

    def __init__(self, robot_ip='192.168.1.190', config_path=None, logger=None):
        """
        Initialize Cartesian controller (initializes both left and right arms)

        Args:
            robot_ip: Robot IP address
            config_path: Kinematics config file path
                - None: Use default 'ccs_m6.MvKDCfg' (auto-located from ROS2 package)
                - Relative path: Located from the ROS2 package config directory
                - Absolute path: Used directly
            logger: External logger (optional, for ROS2 logging integration)
        """
        # Logger: prefer externally provided logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger(__name__)

        # Resolve config file path
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'

            package_share = get_package_share_directory('tianji_world_output')
            config_path = os.path.join(package_share, 'config', config_filename)
        # Check if file exists
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")


        self.logger.debug("Loading config file: %s", config_path)

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

        # Initialize robot connection
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

        # IK parameters (can be modified at runtime) — loaded from unified config
        from tianji_world_output.config_loader import TianjiConfig
        self._cfg = TianjiConfig.load()
        self.zsp_type = self._cfg.zsp_type
        left_zsp = self._cfg.default_zsp_para.get('left', [0, -1, -0.5, 0, 0, 0])
        right_zsp = self._cfg.default_zsp_para.get('right', [0, 1, -0.5, 0, 0, 0])
        self.left_zsp_para = list(left_zsp)
        self.right_zsp_para = list(right_zsp)
        self.zsp_angle = self._cfg.zsp_angle
        self.dgr = list(self._cfg.dgr)

        # IK seed: last successful joint angles, updated each frame to ensure configuration continuity
        default_left = list(self._cfg.init_joints.get('left', [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]))
        default_right = list(self._cfg.init_joints.get('right', [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]))
        self._last_valid_left_joints = default_left
        self._last_valid_right_joints = default_right

        # Set tool parameters (wuji hand)
        self._set_tool_params()

        self.logger.info("Dual-arm initialization complete")

    def _set_tool_params(self):
        # IK solves at the flange (tool_kine zero); mounted-tool kinematics
        # belong in URDF / chest TF, not in the SDK's set_tool matrix.
        tool_kine = [0, 0, 0, 0, 0, 0]
        # Identified Wuji Hand payload (M kg, mr mm, I kg·mm²); rewrite if
        # you mount a different end-effector.
        tool_dyn = [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0]

        self.logger.info("Setting tool parameters (Wuji Hand): kine=%s, dyn=%s", tool_kine, tool_dyn)

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
            # Check left arm connection
            serial = sub_data['outputs'][0]['frame_serial']
            if serial != 0 and frame_update != serial:
                motion_tag += 1
                frame_update = serial
            time.sleep(0.1)
        return motion_tag > 0

    def get_current_joints(self):
        """Get current dual-arm joint angles"""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def _wait_for_arm_state(self, target_state, timeout=2.0, poll_interval=0.05):
        """Poll cur_state until both arms reach target_state, or timeout.

        Marvin's set_state(N) is not synchronous — the arm goes through a
        transitional state before landing on target_state. Polling the
        DCSS readback is the only reliable way to know the transition
        actually happened (vs. a fixed time.sleep that's either too short
        or wastes time).

        Returns (ok: bool, left_state: int, right_state: int).
        """
        dcss = DCSS()
        deadline = time.monotonic() + timeout
        a_state, b_state = -1, -1
        while time.monotonic() < deadline:
            sub = self.robot.subscribe(dcss)
            a_state = sub["states"][0]["cur_state"]
            b_state = sub["states"][1]["cur_state"]
            if a_state == target_state and b_state == target_state:
                return True, a_state, b_state
            time.sleep(poll_interval)
        return False, a_state, b_state

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """Enable both arms in impedance (state=3) mode.

        Was sleep-based (five `time.sleep(0.5)` chunks); switched to a
        DCSS poll for the actual state transition. The kd_params /
        impedance_type writes that follow still need a short settle
        before the next `send_cmd`, because the Marvin protocol applies
        parameters out-of-band of cur_state and there's no readback we
        can poll. Those sleeps are intentional and small.
        """
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=3)
        self.robot.set_state(arm='B', state=3)
        self.robot.set_vel_acc(arm='A', velRatio=60, AccRatio=60)
        self.robot.set_vel_acc(arm='B', velRatio=60, AccRatio=60)
        self.robot.send_cmd()
        ok, a, b = self._wait_for_arm_state(target_state=3, timeout=2.0)
        if not ok:
            raise RuntimeError(
                f"set_impedance_mode: arms didn't reach state=3 in 2s "
                f"(left={a}, right={b}). Servos faulted or the cabinet is offline."
            )

        if mode == 'cart':
            K = K or [8000, 8000, 8000, 100, 100, 100, 20]
            D = D or [0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4]

            self.robot.clear_set()
            self.robot.set_cart_kd_params(arm='A', K=K, D=D, type=2)
            self.robot.set_cart_kd_params(arm='B', K=K, D=D, type=2)
            time.sleep(0.5)  # param-apply settle (no cur_state to poll)
            self.robot.set_impedance_type(arm='A', type=2)
            self.robot.set_impedance_type(arm='B', type=2)
            self.robot.send_cmd()
            time.sleep(0.5)  # param-apply settle

            self.logger.info("Dual-arm Cartesian impedance mode K=%s", K)

        elif mode == 'joint':
            K = K or [2,2,2,1.6, 1, 1, 1]
            D = D or [0.3,0.3,0.3,0.2,0.2,0.2,0.2]

            self.robot.clear_set()
            self.robot.set_joint_kd_params(arm='A', K=K, D=D)
            self.robot.set_joint_kd_params(arm='B', K=K, D=D)
            self.robot.send_cmd()
            time.sleep(0.5)  # param-apply settle

            self.robot.clear_set()
            self.robot.set_impedance_type(arm='A', type=1)
            self.robot.set_impedance_type(arm='B', type=1)
            self.robot.send_cmd()
            time.sleep(0.5)  # param-apply settle

            self.logger.info("Dual-arm joint impedance mode K=%s", K)

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01,
                     init_joints_left=None, init_joints_right=None):
        """
        Move both arms simultaneously to initial pose (using joint-space trajectory interpolation for compliant motion)

        Args:
            wait: Whether to wait for motion completion
            timeout: Additional wait time after reaching target (seconds)
            duration: Total trajectory duration (seconds), larger = slower and more compliant
            dt: Interpolation time step (seconds)
            init_joints_left: Left arm initial joint angles (degrees), None uses default
            init_joints_right: Right arm initial joint angles (degrees), None uses default

        Returns:
            bool: Whether successful
        """
        # Default initial joint angles — loaded uniformly from tianji_robot.yaml
        DEFAULT_INIT_LEFT = list(self._cfg.init_joints.get('left', [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]))
        DEFAULT_INIT_RIGHT = list(self._cfg.init_joints.get('right', [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]))

        INIT_JOINTS_LEFT = init_joints_left if init_joints_left is not None else DEFAULT_INIT_LEFT
        INIT_JOINTS_RIGHT = init_joints_right if init_joints_right is not None else DEFAULT_INIT_RIGHT

        self.logger.debug("Moving both arms to initial pose (%ss compliant trajectory)...", duration)

        # Get current joint angles
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)
        num_points = int(duration / dt)

        # Use quintic polynomial interpolation for compliant trajectory (zero start/end velocity and acceleration)
        for i in range(num_points + 1):
            t = i / num_points  # Normalized time [0, 1]
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            target_left = [
                start_left[j] + s * (INIT_JOINTS_LEFT[j] - start_left[j])
                for j in range(7)
            ]
            target_right = [
                start_right[j] + s * (INIT_JOINTS_RIGHT[j] - start_right[j])
                for j in range(7)
            ]

            # Send dual-arm joint commands
            self.robot.clear_set()
            self.robot.set_joint_cmd_pose(arm='A', joints=target_left)
            self.robot.set_joint_cmd_pose(arm='B', joints=target_right)
            self.robot.send_cmd()

            time.sleep(dt)

        if wait:
            time.sleep(timeout)

        # Verify whether initial pose was reached
        final_left, final_right = self.get_current_joints()
        left_errors = [abs(final_left[i] - INIT_JOINTS_LEFT[i]) for i in range(7)]
        right_errors = [abs(final_right[i] - INIT_JOINTS_RIGHT[i]) for i in range(7)]
        max_left_error = max(left_errors)
        max_right_error = max(right_errors)

        success = True
        if max_left_error < 5.0:
            self.logger.debug("[Arm A] Reached initial pose")
        else:
            self.logger.warning("[Arm A] Large initial pose error (%.1f deg)", max_left_error)
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[Arm B] Reached initial pose")
        else:
            self.logger.warning("[Arm B] Large initial pose error (%.1f deg)", max_right_error)
            success = False

        # Set IK seed to initial joint angles
        self._last_valid_left_joints = list(INIT_JOINTS_LEFT)
        self._last_valid_right_joints = list(INIT_JOINTS_RIGHT)
        self.logger.info("IK seed has been set")

        return success

    def move_to_pose_direct(self, left_pose, right_pose, unit='mm'):
        """
        Dual-arm simultaneous IK solve and send joint commands (non-blocking, for real-time tracking)

        Args:
            left_pose: Left arm target pose, None means do not control left arm
                      - If unit='matrix': 4x4 numpy matrix (meters)
                      - If unit='mm'/'m': [X, Y, Z, RX, RY, RZ] (millimeters or meters)
            right_pose: Right arm target pose, None means do not control right arm (same format)
            unit: 'matrix' (4x4 matrix), 'mm' (millimeters) or 'm' (meters)

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
        """Convert pose to matrix format required by IK (mm)"""
        if unit == 'matrix':
            mat = pose.copy()
            mat[:3, 3] *= 1000  # m -> mm
            return mat.tolist()
        else:
            pose_mm = list(pose)
            if unit == 'm':
                for i in range(3):
                    pose_mm[i] *= 1000
            return kine.xyzabc_to_mat4x4(pose_mm)

    def _solve_ik(self, kine, robot_serial, pose_mat, ref_joints, arm_name):
        """
        IK solve (two-step fallback)

        1. zsp_type=1 (with elbow constraint)
        2. zsp_type=0 (minimize joint distance from seed)
        Trust the IK solver: valid solutions are used directly, seed updated each frame to ensure continuity.
        """
        zsp_para = self.left_zsp_para if arm_name == 'A' else self.right_zsp_para

        # Primary: with elbow constraint
        joints = self._call_ik(kine, robot_serial, pose_mat, ref_joints,
                                self.zsp_type, zsp_para, arm_name)
        if joints is not None:
            return joints, True

        # Fallback: minimize joint distance
        if self.zsp_type != 0:
            joints = self._call_ik(kine, robot_serial, pose_mat, ref_joints,
                                    0, zsp_para, arm_name)
            if joints is not None:
                return joints, True

        return None, False

    def _call_ik(self, kine, robot_serial, pose_mat, ref_joints,
                  zsp_type, zsp_para, arm_name):
        """Single IK call, returns joint angle list or None"""
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
            self.logger.warning("[Arm %s IK] Exception: %s", arm_name, e)
        return None

    def disable_and_release(self, poll_timeout=3.0):
        """Shutdown path: clear errors, request set_state(0), poll until the
        firmware confirms cur_state==0 (or poll_timeout elapses), then
        release the TCP session.

        Same shape as tianji_chest_driver.disable_and_release (fixed in
        9b087b5 / f950490). The previous implementation here had a fixed
        time.sleep(2) that two things go wrong with on shutdown:

          1. SIGINT received while sleeping raises KeyboardInterrupt in
             the middle of cleanup. release_robot() never runs, the
             Marvin TCP session leaks, and the next launch fails to
             connect.
          2. 2s is not always enough for state=3 (impedance) -> state=0.
             When the firmware takes longer, the operator stops teleop
             but the arms remain at TORQ holding torque — unsafe.

        Poll cur_state instead, short interruption-safe time.sleep(0.05)
        chunks, log on timeout, and ALWAYS release_robot() so the TCP
        session is freed even if state transition failed.
        """
        self.logger.info("Disabling arms (set_state=0)...")
        try:
            # Empirically, calling clear_error() BEFORE set_state(0) prevents
            # the firmware from honoring the state transition (the arms stay
            # at cur_state=3 indefinitely; cf. the smoke test in this commit
            # message). set_standby() works because it never clear_errors —
            # mirror that. If the user really needs errors cleared on
            # shutdown, do it AFTER cur_state reaches 0.
            self.robot.clear_set()
            self.robot.set_state(arm='A', state=0)
            self.robot.set_state(arm='B', state=0)
            self.robot.send_cmd()
        except Exception as exc:
            self.logger.warning("set_state(0) on shutdown failed (ignored): %s", exc)

        try:
            ok, a, b = self._wait_for_arm_state(target_state=0,
                                                timeout=max(0.5, float(poll_timeout)))
            if ok:
                self.logger.info("Both arms at cur_state=0")
            else:
                self.logger.warning(
                    "Servos may still be on (cur_state left=%d, right=%d); "
                    "releasing TCP anyway", a, b)
        except Exception as exc:
            self.logger.warning("cur_state poll failed (ignored): %s", exc)

        self.logger.debug("Releasing connection...")
        try:
            self.robot.release_robot()
        except Exception as exc:
            self.logger.warning("release_robot failed (ignored): %s", exc)
        self.logger.info("Safely exited")
