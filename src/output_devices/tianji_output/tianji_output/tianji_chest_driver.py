#!/usr/bin/env python3
"""
Tianji Arm Chest-frame Driver.

Integrates Cartesian space control (via IK) and joint space control
(direct joint angles). HTC Tracker / PICO teleop uses the Cartesian
path; the joint path is kept for future use.

All tunable parameters load from config/tianji_chest.yaml (single
source of truth).

Notes:
- Requires controller firmware 100342 or newer (otherwise set_tool_kine
  has no effect on IK).
- IK tool offset is set on the kine side (set_tool_kine); the hardware
  side `robot.set_tool` is given kineParams=[0,0,0,0,0,0] so we don't
  double-apply the offset.
"""
try:
    from tianji_output._internal.fx_robot import Marvin_Robot
    from tianji_output._internal.fx_kine import Marvin_Kine
    from tianji_output._internal.structure_data import DCSS
    from tianji_output.fault_codes import describe_fault_codes, read_servo_fault_codes
except ImportError:
    from ._internal.fx_robot import Marvin_Robot
    from ._internal.fx_kine import Marvin_Kine
    from ._internal.structure_data import DCSS
    from .fault_codes import describe_fault_codes, read_servo_fault_codes
import time
import logging
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class TianjiChestDriver:
    """
    Tianji dual-arm Chest-frame driver.

    Supports Cartesian-space control (via IK) and joint-space control
    (direct angles). HTC/Tracker teleop uses the Cartesian pathway.
    """

    def __init__(self, robot_ip='192.168.1.190', config_path=None, logger=None):
        """
        Initialize both arms.

        Args:
            robot_ip: robot controller IP address.
            config_path: kinematics config file path.
                - None: use the default 'ccs_m6.MvKDCfg' shipped in the
                  ROS2 package share dir.
                - absolute path: use it directly.
            logger: optional external logger (e.g. ROS2 node logger).
        """
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('TianjiChestDriver')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        # Load tianji_chest.yaml
        package_share = get_package_share_directory('tianji_output')
        yaml_path = os.path.join(package_share, 'config', 'tianji_chest.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        init_joints = cfg.get('init_joints')
        if not init_joints or 'left' not in init_joints or 'right' not in init_joints:
            raise ValueError(
                f"tianji_chest.yaml missing init_joints.left/right: {yaml_path}")
        self._init_joints_left = list(init_joints['left'])
        self._init_joints_right = list(init_joints['right'])

        # Resolve kinematics config path
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'
            config_path = os.path.join(package_share, 'config', config_filename)

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        self.logger.debug(f"Loading config: {config_path}")

        # ---------------------- Kinematics init ----------------------
        self.logger.debug("[arm A] Initializing kinematics SDK...")
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
        # Tool kinematics offset: identity (no offset).
        # New SDK handles tool offset on the IK side; hardware-side
        # robot.set_tool no longer receives kineParams.
        self.kine_left.set_tool_kine(0, [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        self.logger.debug("[arm B] Initializing kinematics SDK...")
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
        self.kine_right.set_tool_kine(1, [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        # ---------------------- Robot connection ----------------------
        self.logger.debug("Initializing robot control...")
        self.robot = Marvin_Robot()

        init = self.robot.connect(robot_ip)
        if init == 0:
            raise ConnectionError("Connection failed: port in use")

        time.sleep(0.5)
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)

        if not self._verify_connection():
            raise ConnectionError("Robot connection failed")

        TianjiChestDriver._shared_robot = self.robot
        TianjiChestDriver._shared_kine_left = self.kine_left
        TianjiChestDriver._shared_kine_right = self.kine_right
        TianjiChestDriver._initialized = True

        # ---------------------- IK parameters (live-tunable) ----------------------
        self.zsp_type = 1                            # nullspace constraint type
        self.left_zsp_para = [0, -1, -0.5, 0, 0, 0]  # left-arm nullspace plane
        self.right_zsp_para = [0, 1, -0.5, 0, 0, 0]  # right-arm nullspace plane
        self.zsp_angle = 0.0                         # nullspace arm-angle rotation
        self.dgr = [5.0, 5.0, 5.0]                   # singularity tolerance (deg)

        # Tool dynamic parameters (Wuji Hand)
        self._set_tool_params()

        self.logger.info("Dual-arm controller initialized")

    def _set_tool_params(self):
        """
        Set tool dynamic params (gravity compensation, hardware side).

        kineParams is forced to [0,0,0,0,0,0]: the geometric tool offset
        is fed to IK via kine.set_tool_kine(); setting it on the hardware
        side too would double-compensate and misalign the wrist tracker
        from the flange.
        """
        # Tool dynamics: mass 0.95kg, COM at +90mm Z in tool frame, zero
        # inertia tensor. Left/right symmetric.
        tool_dyn_left = [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0]
        tool_dyn_right = tool_dyn_left

        self.logger.debug(
            f"Setting tool dynamics: A={tool_dyn_left}, B={tool_dyn_right}")

        self.robot.clear_set()
        self.robot.set_tool(arm='A', kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=tool_dyn_left)
        self.robot.set_tool(arm='B', kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=tool_dyn_right)
        self.robot.send_cmd()
        time.sleep(0.3)

    def _verify_connection(self):
        """Sanity check: subscribe a few times, look for frame_serial advancing."""
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

    # ==================== State queries ====================

    def get_current_joints(self):
        """Return (left_joints, right_joints), each 7 joint angles in degrees."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def get_current_joint_velocities(self):
        """Return (left_velocities, right_velocities), each 7 joint velocities in deg/s."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_vel = sub_data["outputs"][0]["fb_joint_vel"]
        right_vel = sub_data["outputs"][1]["fb_joint_vel"]
        return left_vel, right_vel

    def get_current_joint_torques(self):
        """Return (left_torques, right_torques), each 7 joint torques in Nm."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_torque = sub_data["outputs"][0]["fb_joint_tor"]
        right_torque = sub_data["outputs"][1]["fb_joint_tor"]
        return left_torque, right_torque

    def get_full_state(self):
        """Return dual-arm joint positions, velocities, and torques as a dict."""
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
        """Lightweight state read: return (left_cur_state, right_cur_state).

        Used by service callbacks for hardware-driven state checks
        (e.g. to confirm the arm is actually in the target state rather
        than trusting a Python flag). One SDK subscribe, no servo-error
        read — about 5 ms. Much lighter than get_arm_status().
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return (sub_data["states"][0]["cur_state"], sub_data["states"][1]["cur_state"])

    def get_arm_status(self):
        """Dual-arm state codes, servo error codes, and trajectory-realtime metrics.

        Returns:
            dict: {'left': {'state': int, 'err_code': int,
                            'servo_errors': list[str],          # 7 hex strings
                            'servo_descriptions': list[str],    # 7 EN strings (empty=no error)
                            'realtime': dict},                  # frame_miss_cnt etc.
                   'right': ...}
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        result = {}
        for idx, (arm_id, side) in enumerate([('A', 'left'), ('B', 'right')]):
            servo_errors = self._get_servo_errors_raw(arm_id)
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

    def _get_servo_errors_raw(self, arm: str):
        """Return 7 servo fault codes for the given arm (hex string list)."""
        return read_servo_fault_codes(self.robot.robot, arm)

    # ==================== Impedance mode ====================

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """
        Set dual-arm impedance mode.

        Order: set impedance params and type first, then servo-on
        (state=3). Servoing on with stale params is what causes the
        first-frame jitter we hit in production.

        Args:
            mode: 'joint' or 'cart'.
            K: stiffness list (7 elements).
            D: damping list (7 elements).
        """
        if mode == 'cart':
            K = K or [8000, 8000, 8000, 100, 100, 100, 20]
            D = D or [0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4]

            self.robot.clear_set()
            self.robot.set_cart_kd_params(arm='A', K=K, D=D, type=2)
            self.robot.set_cart_kd_params(arm='B', K=K, D=D, type=2)
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
            self.robot.set_impedance_type(arm='A', type=1)
            self.robot.set_impedance_type(arm='B', type=1)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info(f"Dual-arm joint impedance mode K={K}")

        # Servo-on after params are loaded.
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=3)
        self.robot.set_state(arm='B', state=3)
        self.robot.set_vel_acc(arm='A', velRatio=60, AccRatio=60)
        self.robot.set_vel_acc(arm='B', velRatio=60, AccRatio=60)
        self.robot.send_cmd()

        # Poll until both arms reach state=3. state=100 and 101/102/103 are all
        # TRANSIENT switching codes — the arm passes through them and lands on 3
        # on its own, so we just keep waiting (no clear_error, no fail).
        ok, diag = self._poll_arm_state_after_switch(target_state=3, timeout=5.0)
        if not ok:
            # Do NOT abort enable on a slow/sticky switch — match main, which
            # never polled and just proceeded. The joints report no servo fault
            # (state=100/err=6 is an arm-level transient code that one arm can
            # sit in briefly), and the arm settles into impedance as commands
            # flow. Raising here strands the operator in ENABLE_FAILED over a
            # benign transient.
            self.logger.warning("set_impedance_mode: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm impedance mode active (cur_state=3)")

    def _poll_arm_state_after_switch(self, target_state, timeout=5.0, poll_interval=0.05):
        """Poll until both arms reach target_state, or timeout.

        SDK state machine: set_state(N) is not synchronous. The arm passes
        through transient switching codes — 100, 101, 102, 103 — before it
        lands on target_state. state=100 is NOT a fault here: it is just one of
        those transient codes and the arm reaches the target on its own a moment
        later, so we simply wait.

        (The previous version treated 100 as ARM_STATE_ERROR and ran
        clear_error on it. That dropped the arm back to standby (state=0) and
        broke the in-progress switch, so the arm never reached impedance and
        enable failed — a regression vs main, which never polled and just let
        the transition finish.)

        Returns: (ok: bool, diagnostic: str).
        """
        dcss = DCSS()
        deadline = time.monotonic() + timeout
        a_state, b_state, a_err, b_err = -1, -1, -1, -1
        while time.monotonic() < deadline:
            sub_data = self.robot.subscribe(dcss)
            a_state = sub_data["states"][0]["cur_state"]
            b_state = sub_data["states"][1]["cur_state"]
            a_err = sub_data["states"][0]["err_code"]
            b_err = sub_data["states"][1]["err_code"]
            if a_state == target_state and b_state == target_state:
                return True, ""
            # Any other code (100/101/102/103/0) is a transient switching state;
            # keep waiting for the target rather than interfering with clear_error.
            time.sleep(poll_interval)
        diag = (
            f"state-switch timed out after {timeout}s target={target_state} "
            f"left(state={a_state}, err_code={a_err}); "
            f"right(state={b_state}, err_code={b_err})"
        )
        return False, diag

    # ==================== Cartesian-space control ====================

    def move_to_pose_direct(self, left_pose=None, right_pose=None, unit='mm', send=True):
        """
        Cartesian-space control: IK-solve both arms and dispatch joint commands.
        Non-blocking, suitable for realtime tracking.

        Args:
            left_pose: [X, Y, Z, RX, RY, RZ] left target pose; None to skip the arm.
            right_pose: [X, Y, Z, RX, RY, RZ] right target pose; None to skip.
            unit: 'mm' or 'm'.
            send: if False, only run IK and return the solved joints without
                pushing them to the controller. Used by the handoff ramp in
                tianji_arm_node, which needs to blend the IK target with a
                start snapshot before dispatching via move_to_joints_direct.

        Returns:
            tuple: (left_success, right_success, left_joints, right_joints).
        """
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

        ref_left, ref_right = self.get_current_joints()

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

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
                    dgr=self.dgr,
                )
                if left_ik is not False:
                    if not left_ik.m_Output_IsOutRange and not left_ik.m_Output_IsJntExd:
                        left_joints = left_ik.m_Output_RetJoint.to_list()
                        left_success = True
            except Exception as e:
                self.logger.debug(f"left IK exception: {e}")

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
                    dgr=self.dgr,
                )
                if right_ik is not False:
                    if not right_ik.m_Output_IsOutRange and not right_ik.m_Output_IsJntExd:
                        right_joints = right_ik.m_Output_RetJoint.to_list()
                        right_success = True
            except Exception as e:
                self.logger.debug(f"right IK exception: {e}")

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

        if send:
            self.robot.clear_set()
            if left_joints is not None:
                self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
            if right_joints is not None:
                self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
            self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    # ==================== Joint-space control ====================

    def move_to_joints_direct(self, left_joints=None, right_joints=None):
        """
        Joint-space control: dispatch joint angle commands directly.
        Non-blocking, suitable for realtime tracking.

        Args:
            left_joints: [j1..j7] left target angles (degrees); None to skip.
            right_joints: [j1..j7] right target angles (degrees); None to skip.

        Returns:
            tuple: (left_success, right_success).
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
        Smoothly move both arms to the target joint angles using a quintic blend.

        Args:
            left_target: [j1..j7] left target angles; None keeps current.
            right_target: [j1..j7] right target angles; None keeps current.
            duration: trajectory duration (s); larger = slower / smoother.
            dt: interpolation step (s).

        Returns:
            bool: success.
        """
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)

        if left_target is None:
            left_target = start_left
        if right_target is None:
            right_target = start_right

        num_points = int(duration / dt)

        self.logger.debug(f"Smooth move to target ({duration}s quintic blend)...")

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

    # ==================== Home pose / release ====================

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01):
        """
        Smoothly move both arms to the configured init pose.

        Args:
            wait: if True, sleep `timeout` seconds after the move.
            timeout: extra settle time (s).
            duration: trajectory duration (s).
            dt: interpolation step (s).

        Returns:
            bool: success.
        """
        INIT_JOINTS_LEFT = list(self._init_joints_left)
        INIT_JOINTS_RIGHT = list(self._init_joints_right)

        self.logger.debug(f"Moving to init pose ({duration}s quintic blend)...")

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
            self.logger.debug("[arm A] reached init pose")
        else:
            self.logger.warning(f"[arm A] init-pose residual {max_left_error:.1f} deg")
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[arm B] reached init pose")
        else:
            self.logger.warning(f"[arm B] init-pose residual {max_right_error:.1f} deg")
            success = False

        return success

    def set_standby(self):
        """Servo-off (state=0), keep connection, ready to resume. State-driven (waits cur_state=0)."""
        self.logger.info("Dual-arm STANDBY (state=0, connection preserved)")
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=0)
        self.robot.set_state(arm='B', state=0)
        self.robot.send_cmd()

        ok, diag = self._poll_arm_state_after_switch(target_state=0, timeout=2.0)
        if not ok:
            self.logger.warning("set_standby: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm STANDBY reached (cur_state=0)")

    def set_active(self, mode='joint', K=None, D=None):
        """Servo-on (state=3), apply impedance params. Safe to call any time."""
        self.logger.info("Dual-arm ACTIVE (impedance mode)")
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)
        self.set_impedance_mode(mode=mode, K=K, D=D)

    def set_position_mode(self, velRatio=30, AccRatio=30):
        """Switch to position-tracking mode (state=1), with state-machine poll (target=1, timeout=2s)."""
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=1)
        self.robot.set_state(arm='B', state=1)
        self.robot.set_vel_acc(arm='A', velRatio=velRatio, AccRatio=AccRatio)
        self.robot.set_vel_acc(arm='B', velRatio=velRatio, AccRatio=AccRatio)
        self.robot.send_cmd()

        ok, diag = self._poll_arm_state_after_switch(target_state=1, timeout=2.0)
        if not ok:
            self.logger.warning("set_position_mode: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm position mode active (cur_state=1)")

    def release_brake(self, arm: str = 'A'):
        """Release the brake. Direct SDK call, not via clear_set/send_cmd batching."""
        param = 'BRAK0' if arm == 'A' else 'BRAK1'
        self.robot.set_param('int', param, 2)
        self.logger.info(f"{'left' if arm == 'A' else 'right'} arm brake released")

    def hold_brake(self, arm: str = 'A'):
        """Engage the brake. Direct SDK call."""
        param = 'BRAK0' if arm == 'A' else 'BRAK1'
        self.robot.set_param('int', param, 1)
        self.logger.info(f"{'left' if arm == 'A' else 'right'} arm brake held")

    def clear_arm_error(self, arm: str = 'A'):
        """Clear error state on a single arm."""
        side = 'left' if arm == 'A' else 'right'
        self.robot.clear_set()
        self.robot.clear_error(arm)
        self.robot.send_cmd()
        time.sleep(0.5)
        self.logger.info(f"{side} arm error cleared (clear_error {arm})")

    def disable_and_release(self, poll_timeout=3.0):
        """Shutdown path: clear errors, request set_state(0), poll until the
        firmware confirms cur_state==0 (or `poll_timeout` elapses), then
        release the TCP session.

        We do NOT call self.set_standby() here because its poll-and-raise
        loop is built for the runtime enable path (where a failed state
        switch is actionable). On shutdown a raise would skip release_robot()
        and leak the Marvin TCP session — which is exactly what users hit
        as "Stop won't power down". So we poll, log on timeout, and still
        release. A fixed sleep was tried and proved unreliable: the firmware
        sometimes takes longer than 2s to acknowledge state=0, especially
        when transitioning out of impedance (state=3).
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

        a, b = -1, -1
        try:
            dcss = DCSS()
            deadline = time.monotonic() + max(0.5, float(poll_timeout))
            while time.monotonic() < deadline:
                sub = self.robot.subscribe(dcss)
                a = sub["states"][0]["cur_state"]
                b = sub["states"][1]["cur_state"]
                if a == 0 and b == 0:
                    self.logger.info("Both arms at cur_state=0")
                    break
                time.sleep(0.05)
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
