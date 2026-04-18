from ctypes import *
import ctypes
import inspect
from textwrap import dedent
import os
import math
import logging
from pathlib import Path
from .robot_structures import *

# Configure log system
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.WARNING)  # Reduce log output, only display warnings and errors

# Get the lib directory path relative to this file
_package_dir = Path(__file__).resolve().parent
_lib_dir = _package_dir / 'lib'


class Marvin_Kine:
    def __init__(self):
        """Initialize robot control class"""
        import sys
        logger.info(f'user platform: {sys.platform}')
        if sys.platform == 'win32':
            lib_path = str(_lib_dir / 'libKine.dll')
            self.kine = ctypes.WinDLL(lib_path)
        else:
            lib_path = str(_lib_dir / 'libKine.so')
            self.kine = ctypes.CDLL(lib_path)

        logger.info(f'Loaded library from: {lib_path}')

        # Create structure instances
        self.sp = FX_InvKineSolvePara()
        self.jacobi = FX_Jacobi()
        self.jacobi_dot = FX_Jacobi()

    def help(self, method_name: str = None) -> None:
        """Display help information
        Args: method_name (str): Optional method name to display specific method help
        """
        print(f"\n{' API help ':=^50}\n")

        # Get all public methods
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]

        # If no method name specified, display all methods list
        if method_name is None:
            print("Available methods:")
            for name, func in methods:
                # Get function signature
                signature = inspect.signature(func)
                # Get parameter list
                params = []
                for param in signature.parameters.values():
                    param_str = param.name
                    if param.default is not param.empty:
                        param_str += f"={param.default!r}"
                    if param.annotation is not param.empty:
                        param_str += f": {param.annotation.__name__}"
                    if param.kind == param.VAR_POSITIONAL:
                        param_str = "*" + param_str
                    elif param.kind == param.VAR_KEYWORD:
                        param_str = "**" + param_str
                    elif param.kind == param.KEYWORD_ONLY:
                        param_str = "[kw] " + param_str
                    params.append(param_str)

                param_list = ", ".join(params)
                print(f"  - {name}({param_list})")

            print("\nUse help('method_name') for detailed help information")
            print(f"{'=' * 50}")
            return

        # Display help for specific method
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "No documentation available"

            # Get function signature
            signature = inspect.signature(func)

            print(f"Method: {method_name}{signature}")
            print("\n" + dedent(doc))

            # Display parameter details
            print("\nParameter details:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"type: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"default: {param.default!r}"
                # param_info += f"type: {_param_kind_to_str(param.kind)}"
                print(param_info)
        else:
            print(f"Error: method not found '{method_name}'")

        print(f"{'=' * 50}")

    def _param_kind_to_str(kind):
        """Convert parameter kind to readable string"""
        mapping = {
            inspect.Parameter.POSITIONAL_ONLY: "Positional argument",
            inspect.Parameter.POSITIONAL_OR_KEYWORD: "Positional or keyword argument",
            inspect.Parameter.VAR_POSITIONAL: "Variable positional argument (*args)",
            inspect.Parameter.KEYWORD_ONLY: "Keyword-only argument",
            inspect.Parameter.VAR_KEYWORD: "Variable keyword argument (**kwargs)"
        }
        return mapping.get(kind, "Unknown parameter type")

    def load_config(self, config_path: str):
        ''' Before use, make sure to confirm the robot model and import the correct configuration file. Import robot arm configuration.
        :param config_path: Local robot arm configuration file a.MvKDCfg, can be relative path.
        - The a.MvKDCfg file contains dual-arm parameters related to kinematics and dynamics computation. Before computation, the robot arm configuration file must be imported.
        - TYPE=1006: DL robot model; TYPE=1007: Pilot-SRS robot model (dual arms are MARVIN); TYPE=1017: Pilot-CCS robot model (dual arms are MARVIN)!
        - GRV parameters are dual-arm gravity direction, e.g. [0.000,9.810,0.000]; DH parameters are dual-arm MDH parameters, containing each joint's MDH parameters and flange MDH parameters; PNVA parameters are the maximum positive/negative acceleration and jerk allowed for each joint of both arms; BD parameters are specific to the Pilot-CCS robot model, representing the second-order polynomial curve fit for joints 6-7 self-interference allowable range (other robot models have all zeros for this parameter); Mass parameters are each joint's mass for both arms; MCP parameters are each joint's center of mass for both arms; I parameters are each joint's inertia for both arms.
        - MDH parameter units are degrees and millimeters (mm); velocity and acceleration units are degrees/second; joint mass, joint center of mass, and joint inertia units are all in SI units.
        :return:
        '''

        if not os.path.exists(config_path):
            raise ValueError("no config file")

        # Define function prototype
        self.kine.LOADMvCfg.argtypes = [
            c_char_p,  # FX_CHAR* path
            ctypes.POINTER(c_long * 2),  # FX_INT32L TYPE[2]
            ctypes.POINTER((c_double * 3) * 2),  # FX_DOUBLE GRV[2][3]
            ctypes.POINTER(((c_double * 4) * 8) * 2),  # FX_DOUBLE DH[2][8][4]
            ctypes.POINTER(((c_double * 4) * 7) * 2),  # FX_DOUBLE PNVA[2][7][4]
            ctypes.POINTER(((c_double * 3) * 4) * 2),  # FX_DOUBLE BD[2][4][3]
            ctypes.POINTER((c_double * 7) * 2),  # FX_DOUBLE Mass[2][7]
            ctypes.POINTER(((c_double * 3) * 7) * 2),  # FX_DOUBLE MCP[2][7][3]
            ctypes.POINTER(((c_double * 6) * 7) * 2)  # FX_DOUBLE I[2][7][6]
        ]
        self.kine.LOADMvCfg.restype = c_bool  # return type FX_BOOL

        # Initialize all array parameters
        TYPE = (c_long * 2)()
        GRV = ((c_double * 3) * 2)()
        DH = (((c_double * 4) * 8) * 2)()
        PNVA = (((c_double * 4) * 7) * 2)()
        BD = (((c_double * 3) * 4) * 2)()
        Mass = ((c_double * 7) * 2)()
        MCP = (((c_double * 3) * 7) * 2)()
        I = (((c_double * 6) * 7) * 2)()

        # Call function

        success = self.kine.LOADMvCfg(
            config_path.encode('utf-8'),
            ctypes.byref(TYPE),
            ctypes.byref(GRV),
            ctypes.byref(DH),
            ctypes.byref(PNVA),
            ctypes.byref(BD),
            ctypes.byref(Mass),
            ctypes.byref(MCP),
            ctypes.byref(I)
        )

        # Process result
        if success:
            result = {
                'TYPE': [TYPE[i] for i in range(2)],
                'GRV': [[GRV[i][j] for j in range(3)] for i in range(2)],
                'DH': [[[DH[i][j][k] for k in range(4)] for j in range(8)] for i in range(2)],
                'PNVA': [[[PNVA[i][j][k] for k in range(4)] for j in range(7)] for i in range(2)],
                'BD': [[[BD[i][j][k] for k in range(3)] for j in range(4)] for i in range(2)],
                'Mass': [[Mass[i][j] for j in range(7)] for i in range(2)],
                'MCP': [[[MCP[i][j][k] for k in range(3)] for j in range(7)] for i in range(2)],
                'I': [[[I[i][j][k] for k in range(6)] for j in range(7)] for i in range(2)]
            }
            logger.info("Load config successful")
            return result
        else:
            logger.error("Load config failed")
            return None

    def initial_kine(self, robot_serial: int, robot_type: int, dh: list, pnva: list, j67: list):
        '''Initialize kinematics-related parameters
        - Before kinematics calculations, initialization functions must be called in this order to initialize imported configuration parameters.
        - RobotSerial=0, left arm; RobotSerial=1, right arm
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param type: int. Robot model identifier
        :param dh: list(8,4), Each axis DH: alpha, a, d, theta.
        :param pnva: list(7,4), Each axis: joint upper limit p, joint lower limit n, max velocity v, max acceleration a.
        :param j67: list(4,3), Only effective for CCS model, joints 6-7 interference limits.
        :return:
            bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if type(robot_type) != int:
            raise ValueError("robot_type  must be int type")


        if len(dh) != 8:
            raise ValueError("dh  must be 8 rows")
        else:
            for i in range(len(dh)):
                if len(dh[i]) != 4:
                    raise ValueError("dh  must be 4 columns")

        if len(pnva) != 7:
            raise ValueError("pnva  must be 7 rows")
        else:
            for i in range(len(pnva)):
                if len(pnva[i]) != 4:
                    raise ValueError("pnva  must be 4 columns")

        if len(j67) != 4:
            raise ValueError("j67  must be 4 rows")
        else:
            for i in range(len(j67)):
                if len(j67[i]) != 3:
                    raise ValueError("j67  must be 3 columns")

        Serial = ctypes.c_long(robot_serial)
        robot_type_ = c_long(robot_type)

        DH = ((c_double * 4) * 8)()
        for i in range(8):
            for j in range(4):
                DH[i][j] = dh[i][j]

        PNVA = ((c_double * 4) * 7)()
        for i in range(7):
            for j in range(4):
                PNVA[i][j] = pnva[i][j]

        J67 = ((c_double * 3) * 4)()
        for i in range(4):
            for j in range(3):
                J67[i][j] = j67[i][j]

        ''' ini type'''
        self.kine.FX_Robot_Init_Type.argtypes = [c_long, c_long]
        self.kine.FX_Robot_Init_Type.restype = c_bool
        success1 = self.kine.FX_Robot_Init_Type(Serial, robot_type_)

        ''' ini dh'''
        # FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4]);
        self.kine.FX_Robot_Init_Kine.argtypes = [c_long, (c_double * 4) * 8]
        self.kine.FX_Robot_Init_Kine.restype = c_bool
        success2 = self.kine.FX_Robot_Init_Kine(Serial, DH)

        ''' ini Lmt'''
        # FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3]);
        self.kine.FX_Robot_Init_Lmt.argtypes = [c_long, (c_double * 4) * 7, (c_double * 3) * 4]
        self.kine.FX_Robot_Init_Lmt.restype = c_bool
        success3 = self.kine.FX_Robot_Init_Lmt(Serial, PNVA, J67)

        # print(success1,success2,success3)
        if success1 and success2 and success3:
            logger.info('Initial kinematics successful')
            return True
        elif not success1:
            logger.error('Initial kinematics failed:FX_Robot_Init_Type')
            return False
        elif not success2:
            logger.error('Initial kinematics failed:FX_Robot_Init_Kine')
            return False
        elif not success3:
            logger.error('Initial kinematics failed:FX_Robot_Init_Lmt')
            return False

    def set_tool_kine(self, robot_serial: int, tool_mat: list):
        '''Tool kinematics setup
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param tool_mat: list(4,4) tool kinematics info, homogeneous transformation matrix, rotation and translation relative to end-effector flange. Please confirm the flange coordinate system.
        :return:bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(tool_mat) != 4:
            raise ValueError("tool_mat  must be 4 rows")
        else:
            for i in range(len(tool_mat)):
                if len(tool_mat[i]) != 4:
                    raise ValueError("tool_mat  must be 4 columns")

        Serial = ctypes.c_long(robot_serial)

        TOOL = ((c_double * 4) * 4)()
        for i in range(4):
            for j in range(4):
                TOOL[i][j] = tool_mat[i][j]

        '''set tool'''
        self.kine.FX_Robot_Tool_Set.argtypes = [c_long, (c_double * 4) * 4]
        self.kine.FX_Robot_Tool_Set.restype = c_bool
        success1 = self.kine.FX_Robot_Tool_Set(Serial, TOOL)
        if success1:
            logger.info('set tool kinematics info successful')
            return True
        else:
            logger.error('set tool kinematics info failed!')
            return False

    def remove_tool_kine(self, robot_serial: int):
        '''Remove tool kinematics setup
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :return:bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)
        '''remove tool'''
        self.kine.FX_Robot_Tool_Rmv.argtypes = [c_long]
        self.kine.FX_Robot_Tool_Rmv.restype = c_bool
        success1 = self.kine.FX_Robot_Tool_Rmv(Serial)
        if success1:
            logger.info('remove tool kinematics info successful')
            return True
        else:
            logger.error('remove tool kinematics info failed!')
            return False



    def fk(self, robot_serial: int, joints: list):
        '''Forward kinematics: joint angles to end-effector TCP position and orientation 4x4
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param joints: list(7,1). Angle values
        :return:
            4x4 pose matrix, list(4,4)
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(joints) != 7:
            raise ValueError("shape error: fk input joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_double = (ctypes.c_double * 7)(j0, j1, j2, j3, j4, j5, j6)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.FX_Robot_Kine_FK.argtypes = [c_long,
                                               ctypes.POINTER(ctypes.c_double * 7),
                                               ctypes.POINTER((ctypes.c_double * 4) * 4)]
        self.kine.FX_Robot_Kine_FK.restype = c_bool
        success1 = self.kine.FX_Robot_Kine_FK(Serial, ctypes.byref(joints_double), ctypes.byref(pg))
        if success1:
            fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
            for i in range(4):
                for j in range(4):
                    fk_mat[i][j] = pg[i][j]
            logger.info(f'fk result, matrix:{fk_mat}')
            return fk_mat
        else:
            return False

    def ik(self, robot_serial: int, pose_mat: list, ref_joints: list, zsp_type: int, zsp_para: list,
               zsp_angle: float, dgr: list):
        '''Inverse kinematics: end-effector position and orientation to joint values
        :param pose_mat: list(4,4), 4x4 position-orientation list.
        :param ref_joints: list(7,1),Reference input angles, constrains solution configuration to be close to reference, prevents solution configuration jumps.
        :param zsp_type: int, Nullspace constraint type (0: minimum Euclidean distance to reference angles; 1: closest to reference arm angle plane)
        :param zsp_para: list(6,), If Nullspace constraint type is 1, additional reference angle plane parameters are needed,[x,y,z,a,b,c]=[0,0,0,0,0,0],Can adjust in one of x, y, z directions
        :param zsp_angle: float, With end-effector pose unchanged, nullspace arm angle rotation relative to reference plane(unit: degrees)
        :param dgr: list(3,), Allowable angle range for singularity at joints 1/2/3. If no extra requirements, no input needed. Default is 0.05(unit: degrees)
        :return:
            Structure. The following fields are most relevant:
                m_Output_RetJoint      : Inverse kinematics solved joint angles (unit: degrees)
                m_Output_IsOutRange    : Whether current pose exceeds reachable workspace (False: not exceeded; True: exceeded)
                m_Output_IsDeg[7]      : Whether each joint is singular (False: not singular; True: singular):
                m_Output_IsJntExd      : Whether any joint exceeds positive/negative position limits (False: not exceeded; True: exceeded):
                m_Output_JntExdTags[7] : Whether each joint exceeds positive/negative position limits (False: not exceeded; True: exceeded):
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        if len(ref_joints) != 7:
            raise ValueError("ref_joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)
        # Copy 4x4 matrix data to sp.m_Input_IK_TargetTCP
        matrix_data = []
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])

        self.sp.m_Input_IK_TargetTCP = Matrix4(matrix_data)

        # Copy joint angle values to sp.m_Input_IK_RefJoint
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        jv = (c_double * 7)(j0_, j1_, j2_, j3_, j4_, j5_, j6_)
        self.sp.m_Input_IK_RefJoint = Vect7(jv)

        self.sp.m_Input_IK_ZSPType = zsp_type
        if zsp_type == 1:
            p0, p1, p2, p3, p4, p5 = zsp_para
            zsp_para_value = (c_double * 6)(p0, p1, p2, p3, p4, p5)
            self.sp.m_Input_IK_ZSPPara = zsp_para_value
        self.sp.m_Input_ZSP_Angle = zsp_angle

        dgr1, dgr2, dgr3 = dgr
        # dgr_value=(c_double*3)(dgr1,dgr2,dgr3)
        self.sp.m_DGR1 = dgr1
        self.sp.m_DGR2 = dgr2
        self.sp.m_DGR3 = dgr3


        # Call inverse kinematics function
        self.kine.FX_Robot_Kine_IK.argtypes = [c_long, POINTER(FX_InvKineSolvePara)]
        self.kine.FX_Robot_Kine_IK.restype = c_bool
        success = self.kine.FX_Robot_Kine_IK(Serial, byref(self.sp))
        if not success:
            logger.warning("Robot Inverse Kinematics Error")
            return False
        else:
            logger.debug(f"IK success, joints:{self.sp.m_Output_RetJoint.to_list()}")
            return self.sp

    def ik_nsp(self, robot_serial: int, pose_mat: list, ref_joints: list, zsp_type: int, zsp_para: list,
               zsp_angle: float, dgr: list):
        '''IK optimization: adjustable direction. Cannot be used alone. Use when the arm angle from IK solution does not satisfy the current solution selection requirements.
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param pose_mat: list(4,4), 4x4 position-orientation list.
        :param ref_joints: list(7,1),Reference input angles, constrains solution configuration to be close to reference, prevents solution configuration jumps.
        :param zsp_type: int, Nullspace constraint type (0: minimum Euclidean distance to reference angles; 1: closest to reference arm angle plane)
        :param zsp_para: list(6,), If Nullspace constraint type is 1, additional reference angle plane parameters are needed,[x,y,z,a,b,c]=[0,0,0,0,0,0],Can adjust in one of x, y, z directions
        :param zsp_angle: float, With end-effector pose unchanged, nullspace arm angle rotation relative to reference plane(unit: degrees)
        :param dgr: list(3,), Allowable angle range for singularity at joints 1/2/3. If no extra requirements, no input needed. Default is 0.05(unit: degrees)
        :return:
            Structure. The following fields are most relevant:
                m_Output_RetJoint      : Inverse kinematics solved joint angles (unit: degrees)
                m_Output_IsOutRange    : Whether current pose exceeds reachable workspace (False: not exceeded; True: exceeded)
                m_Output_IsDeg[7]      : Whether each joint is singular (False: not singular; True: singular):
                m_Output_IsJntExd      : Whether any joint exceeds positive/negative position limits (False: not exceeded; True: exceeded):
                m_Output_JntExdTags[7] : Whether each joint exceeds positive/negative position limits (False: not exceeded; True: exceeded):
        '''

        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        if len(ref_joints) != 7:
            raise ValueError("ref_joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        # Copy 4x4 matrix data to sp.m_Input_IK_TargetTCP
        matrix_data = []
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])

        self.sp.m_Input_IK_TargetTCP = Matrix4(matrix_data)

        # Copy joint angle values to sp.m_Input_IK_RefJoint
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        jv = (c_double * 7)(j0_, j1_, j2_, j3_, j4_, j5_, j6_)
        self.sp.m_Input_IK_RefJoint = Vect7(jv)

        self.sp.m_Input_IK_ZSPType = zsp_type
        if zsp_type == 1:
            p0, p1, p2, p3, p4, p5 = zsp_para
            zsp_para_value = (c_double * 6)(p0, p1, p2, p3, p4, p5)
            self.sp.m_Input_IK_ZSPPara = zsp_para_value
        self.sp.m_Input_ZSP_Angle = zsp_angle

        dgr1, dgr2, dgr3 = dgr
        # dgr_value=(c_double*3)(dgr1,dgr2,dgr3)
        self.sp.m_DGR1 = dgr1
        self.sp.m_DGR2 = dgr2
        self.sp.m_DGR3 = dgr3

        self.kine.FX_Robot_Kine_IK_NSP.argtypes = [c_long, POINTER(FX_InvKineSolvePara)]
        self.kine.FX_Robot_Kine_IK_NSP.restype = c_bool
        success = self.kine.FX_Robot_Kine_IK_NSP(Serial, byref(self.sp))
        if not success:
            logger.error("Robot Inverse Kinematics NSP Error")
            return False
        else:
            logger.info("Robot Inverse Kinematics NSP Success")
            logger.info(f"ik joints:{self.sp.m_Output_RetJoint.to_list()}")
            return self.sp

    def joints2JacobMatrix(self, robot_serial: int, joints: list):
        '''Convert current joint angles to Jacobian matrix
        :param robot_serial:  int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param joints: list(7,1), Current joints
        :return: Jacobian matrix 6x7
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(joints) != 7:
            raise ValueError("joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        joints_double = ctypes.c_double * 7
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)

        example_matrix = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        ]

        # Set Jacobian matrix
        self.jacobi.set_jcb(example_matrix)

        self.kine.FX_Robot_Kine_Jacb.argtypes = [c_long, c_double * 7, POINTER(FX_Jacobi)]
        self.kine.FX_Robot_Kine_Jacb.restype = c_bool
        success = self.kine.FX_Robot_Kine_Jacb(Serial, joints_value, byref(self.jacobi))

        if not success:
            logger.error("Joints2Jacobi Error")
            return False
        else:
            logger.info("Joints2Jacobi Success")
            logger.info(f"Jacobi matrix:{self.jacobi.get_jcb()}")
            return self.jacobi.get_jcb()


    def mat4x4_to_xyzabc(self,pose_mat:list):
        '''Convert end-effector position and orientation to XYZABC
        :param pose_mat: list(4,4), 4x4 position-orientation list.
        :return:
                (6,1) Pose information XYZ and Euler angles ABC (unit: mm/degrees)
        '''
        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        matrix_data =( (c_double*4)*4)()
        for i in range(4):
            for j in range(4):
                matrix_data[i][j]=pose_mat[i][j]

        xyzabc=(c_double*6)(0,0,0,0,0,0)

        self.kine.FX_Matrix42XYZABCDEG.argtypes = [(c_double*4)*4,c_double*6]
        self.kine.FX_Matrix42XYZABCDEG.restype = c_bool
        success = self.kine.FX_Matrix42XYZABCDEG(matrix_data,xyzabc)

        if not success:
            logger.error("Pose mat to xyzabc Error")
            return False
        else:
            logger.info("Pose mat to xyzabc Success")

            pose_6d=[xyzabc[i] for i in range(6)]
            logger.info(f"xyzabc:{pose_6d}")
            return pose_6d


    def xyzabc_to_mat4x4(self,xyzabc:list):
        '''Convert end-effector XYZABC to position and orientation matrix
        param xyzabc: list(6,),
        return:
            mat4x4  list(4,4)

        '''
        if len(xyzabc) != 6:
            raise ValueError("length of xyzabc must be 6")

        j0, j1, j2, j3, j4, j5 = xyzabc
        joints_double = (ctypes.c_double * 6)(j0, j1, j2, j3, j4, j5)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.FX_XYZABC2Matrix4DEG.argtypes = [ctypes.POINTER(ctypes.c_double * 6),
                                     ctypes.POINTER((ctypes.c_double * 4) * 4)]

        self.kine.FX_XYZABC2Matrix4DEG(ctypes.byref(joints_double), ctypes.byref(pg))
        fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        for i in range(4):
            for j in range(4):
                fk_mat[i][j] = pg[i][j]
        if not fk_mat:
            logger.error("xyzabc to mat4x4 Error")
            return False
        else:
            logger.info("xyzabc to mat4x4 Success")
            return fk_mat


    def movL(self,robot_serial: int,start_xyzabc:list, end_xyzabc:list,ref_joints:list,vel:float,acc:float,save_path):
        '''Linear planning (MOVL). Planning file frequency 500Hz, i.e. one row executed every 2ms.

        :param robot_serial: int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param start_xyzabc:Start point end-effector position and orientation: xyz translation unit: mm, abc rotation unit: degrees
        :param end_xyzabc:End point end-effector position and orientation: xyz translation unit: mm, abc rotation unit: degrees
        :param ref_joints: Reference joint configuration
        :param vel:Planning velocity
        :param acc:Planning acceleration
        :param save_path:Path to save the planned file
        :return: bool
        Special notes: 1. Before linear planning, the starting joint position must be passed through the forward kinematics interface to update the starting joint data.
                2. Must read the function return value. If joint exceeds limits, return is false, and the planning PVT file will not be saved.
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)

        path = save_path.encode('utf-8')
        path_char = ctypes.c_char_p(path)

        s0,s1,s2,s3,s4,s5=start_xyzabc
        start= (ctypes.c_double * 6)( s0,s1,s2,s3,s4,s5)

        e0,e1,e2,e3,e4,e5=end_xyzabc
        end= (ctypes.c_double * 6)(e0,e1,e2,e3,e4,e5)

        vel_value=c_double(vel)

        acc_value=c_double(acc)

        j0, j1, j2, j3, j4, j5, j6 = ref_joints
        joints_vel_value = (c_double * 7)(j0, j1, j2, j3, j4, j5, j6)

        self.kine.FX_Robot_PLN_MOVL.argtypes=[c_long,c_double*6,c_double*6,c_double*7,c_double,c_double,c_char_p]
        self.kine.FX_Robot_PLN_MOVL.restype=c_bool
        success1=self.kine.FX_Robot_PLN_MOVL(Serial,start,end,joints_vel_value,vel_value,acc_value,path_char)
        if success1:
            logger.info(f'Plan MOVL successful, PATH saved as :{save_path}')
            return  True

        else:
            logger.error(f'Plan MOVL failed!')
            return False


    def movL_KeepJ(self,robot_serial: int,start_joints:list, end_joints:list,vel:float,save_path):
        '''Linear planning with joint configuration preserved (MOVL KeepJ). Planning file point frequency 50Hz, i.e. one row executed every 20ms.

        :param robot_serial: int, RobotSerial=0, left arm; RobotSerial=1, right arm
        :param start_joints: Starting point joint positions (unit: degrees)
        :param end_joints: End point joint positions (unit: degrees)
        :param vel:Planning velocity, percentage, value range 0-100
        :param save_path:Save path for the planned file
        :return: bool
        Special notes: 1. Before linear planning, the starting joint position must be passed through the forward kinematics interface to update the starting joint data.
                2. Must read the function return value. If joint exceeds limits, return is false, and the planning PVT file will not be saved.
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)

        path = save_path.encode('utf-8')
        path_char = ctypes.c_char_p(path)

        s0,s1,s2,s3,s4,s5,s6=start_joints
        start= (ctypes.c_double * 7)( s0,s1,s2,s3,s4,s5,s6)

        e0,e1,e2,e3,e4,e5,e6=end_joints
        end= (ctypes.c_double * 7)(e0,e1,e2,e3,e4,e5,e6)

        vel_value=c_double(vel)

        self.kine.FX_Robot_PLN_MOVL_KeepJ.argtypes=[c_long,c_double*7,c_double*7,c_double,c_char_p]
        self.kine.FX_Robot_PLN_MOVL_KeepJ.restype=c_bool
        success1=self.kine.FX_Robot_PLN_MOVL_KeepJ(Serial,start,end,vel_value,path_char)
        if success1:
            logger.info(f'Plan MOVL KeepJ successful, PATH saved as :{save_path}')
            return  True

        else:
            logger.error(f'Plan MOVL KeepJ failed!')
            return False


    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''Tool dynamics parameter identification
        :param robot_type: int. 1: CCS robot model, 2: SRS robot model
        :param ipath: sting, Import tool identification trajectory data via relative path.
        :return:
          Identification successful, returns a list of length 10:
                    m,mcp,i
        Identification failed, returns a text string. Please resolve the error based on the text content.

        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        if robot_type==1:
            print(f'CCS tool identy')
        elif robot_type==2:
            print(f'SRS tool identy')

        robot_type_ = c_int(robot_type)
        iden_path = ipath.encode('utf-8')
        path_char = ctypes.c_char_p(iden_path)

        # Create pointer variables instead of arrays
        mm_ptr = pointer(c_double(0))
        mcp_ptr = (c_double * 3)()
        ii_ptr = (c_double * 6)()

        # Set function prototype
        self.kine.FX_Robot_Iden_LoadDyn.argtypes = [
            c_int,
            c_char_p,
            POINTER(c_double),
            POINTER(c_double*3),
            POINTER(c_double*6)
        ]
        self.kine.FX_Robot_Iden_LoadDyn.restype = c_int

        # Call function
        ret_int = self.kine.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )
        if ret_int==0:
            logger.info('Identify tool dynamics successful')

            # Extract result
            dyn_para=[]
            m_val = mm_ptr.contents.value
            mcp_list = [mcp_ptr[i] for i in range(3)]
            ii_list = [ii_ptr[i] for i in range(6)]
            'ixx iyy izz ixy ixz iyz'

            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)

            dyn_para.append(ii_list[0])
            dyn_para.append(ii_list[3])
            dyn_para.append(ii_list[4])
            dyn_para.append(ii_list[1])
            dyn_para.append(ii_list[5])
            dyn_para.append(ii_list[2])


            logger.info(f'tool dynamics[m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]: {dyn_para}')
            return dyn_para
        else:
            logger.error('Identify tool dynamics failed!')
            logger.error(f'identify_tool_dyn returnError code:{ret_int}\n ret=1, Computation error, need to re-collect data\n ret=2,Error opening data collection file, check the sampling file\n ret=3,Configuration file has been modified\n ret=4, Insufficient collection time, lacking valid data')
            if ret_int==1:
                return 'ret=1, Computation error, need to re-collect data'
            elif ret_int==2:
                return 'ret=2,Error opening data collection file, check the sampling file'
            elif ret_int==3:
                return "ret=3,Configuration file has been modified"
            elif ret_int==4:
                return 'ret=4, Insufficient collection time, lacking valid data'




if __name__ == "__main__":
    kk = Marvin_Kine()  # Instantiate
    kk.help()  # View methods
    kk.help('load_config')
    exit()
