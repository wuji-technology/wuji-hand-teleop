import ctypes
import inspect
from textwrap import dedent
import os
import re
from typing import Union
from pathlib import Path

# Get the lib directory path relative to this file
_package_dir = Path(__file__).resolve().parent
_lib_dir = _package_dir / 'lib'

def update_text_file_simple(mode, data_list, filename):
    """
    Simplified file update function
    """
    if mode not in ['A', 'B'] or len(data_list) != 16:
        return False
    try:
        # If file exists, read content; otherwise create default content
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                lines = file.readlines()
        else:
            lines = [','.join(['0'] * 16) + '\n', ','.join(['0'] * 16) + '\n']
        # Update corresponding line
        line_index = 0 if mode == 'A' else 1
        lines[line_index] = ','.join(str(x) for x in data_list) + '\n'

        # Write back to file
        with open(filename, 'w', encoding='utf-8') as file:
            file.writelines(lines)
        return True
    except Exception as e:
        print(f"Error updating file: {e}")
        return False

def read_csv_file_to_float_strict(filename, expected_columns=16):
    """
    Read CSV file content and convert to float, strictly validate column count

    Parameters:
        filename: filename
        expected_columns: expected column count (default 16)

    Returns:
        If file is empty: return 0
        If file has one line: return [float1, float2, ...]
        If file has two lines: return [[float1, float2, ...], [float1, float2, ...]]
        If file does not exist or conversion failed: return -1
    """

    if os.path.getsize(filename) == 0:
        return 0

    try:
        with open(filename, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        non_empty_lines = [line.strip() for line in lines if line.strip()]

        if len(non_empty_lines) == 0:
            return 0

        all_float_data = []
        for line_num, line in enumerate(non_empty_lines, 1):
            values = line.split(',')
            # Filter empty values and strip whitespace
            cleaned_values = [v.strip() for v in values if v.strip()]

            # Verify column count
            if len(cleaned_values) != expected_columns:
                print(f"Line {line_num}: expected {expected_columns} columns, found {len(cleaned_values)} columns")
                return -1

            float_values = []
            for value in cleaned_values:
                try:
                    float_value = float(value)
                    float_values.append(float_value)
                except ValueError:
                    print(f"Line {line_num}: cannot convert content '{value}'  to float")
                    return -1

            all_float_data.append(float_values)

        # Return different format based on line count
        if len(all_float_data) == 1:
            return all_float_data[0]
        elif len(all_float_data) == 2:
            return all_float_data
        else:
            print(f"File contains {len(all_float_data)} lines, only 1-2 lines supported")
            return -1

    except Exception as e:
        print(f"Error reading file: {e}")
        return -1
def decimal_to_hex(number, prefix=False, upper=True, float_precision=8):
    """
    Convert decimal number to hexadecimal representation

    Parameters:
        number: Decimal number to convert, can be integer or float
        prefix: Whether to add "0x" prefix, default is False
        upper: Whether to use uppercase letters, default is True
        float_precision: Float conversion precision (decimal places), default is 8

    Returns:
        str: Hexadecimal representation string

    Exceptions:
        TypeError: Raised when input is not a number
    """
    # Check whether input is a number
    if not isinstance(number, (int, float)):
        raise TypeError("Input must be integer or float")

    # Handle integer
    if isinstance(number, int):
        hex_str = hex(number)
    # Handle float
    else:
        # Use float.hex() method to get hexadecimal representation of float
        hex_str = float(number).hex()

        # If needed, can limit fractional part precision
        if float_precision is not None:
            parts = hex_str.split('.')
            if len(parts) > 1:
                exponent_part = parts[1].split('p')
                if len(exponent_part) > 1:
                    hex_str = f"{parts[0]}.{exponent_part[0][:float_precision]}p{exponent_part[1]}"

    # Remove or keep prefix
    if not prefix and hex_str.startswith('0x'):
        hex_str = hex_str[2:]
    elif prefix and not hex_str.startswith('0x'):
        hex_str = '0x' + hex_str

    # Handle case
    if upper:
        hex_str = hex_str.upper()
    else:
        hex_str = hex_str.lower()

    return hex_str

def identify_and_calculate_length(input_data: Union[str, bytes]) -> dict:
    result = {
        "input": input_data,
        "type": None,
        "length_bytes": None,
        "bytes_representation": None
    }

    # Handle bytes input
    if isinstance(input_data, bytes):
        result["type"] = "bytes"
        result["length_bytes"] = len(input_data)
        result["bytes_representation"] = input_data
        return result

    # Handle string input
    if isinstance(input_data, str):
        # Check whether it is a hexadecimal string (may contain spaces and 0x prefix)
        # Remove all spaces and 0x prefix
        clean_input = re.sub(r'\s+', '', input_data.lower())

        if clean_input.startswith('0x'):
            clean_input = clean_input[2:]

        # Check whether it is a valid hexadecimal string
        hex_pattern = re.compile(r'^[0-9a-f]+$')
        if hex_pattern.match(clean_input):
            # Ensure length is even
            if len(clean_input) % 2 != 0:
                clean_input = '0' + clean_input

            try:
                bytes_rep = bytes.fromhex(clean_input)
                result["type"] = "hex string"
                result["length_bytes"] = len(bytes_rep)
                result["bytes_representation"] = bytes_rep
                return result
            except ValueError:
                pass  # If not valid hexadecimal, continue trying other interpretations

        # Check whether it is already a bytes representation form (e.g. b"\x06\x01\xe3\x08")
        if input_data.startswith('b"') and input_data.endswith('"'):
            try:
                # Use eval to safely convert (Note: in practice a safer method may be needed)
                bytes_rep = eval(input_data)
                if isinstance(bytes_rep, bytes):
                    result["type"] = "bytes representation string"
                    result["length_bytes"] = len(bytes_rep)
                    result["bytes_representation"] = bytes_rep
                    return result
            except:
                pass

        # If not any of the above types, treat as regular string
        try:
            bytes_rep = input_data.encode('utf-8')
            result["type"] = "regular string"
            result["length_bytes"] = len(bytes_rep)
            result["bytes_representation"] = bytes_rep
            return result
        except UnicodeEncodeError:
            raise ValueError("Input is not a valid hexadecimal string and cannot be encoded as UTF-8 bytes")

    # If neither string nor bytes, raise exception
    raise TypeError("Input must be string or bytes")

def structure2dict(dcss):
    result = {
        "para_name": ['Marvin_sub_data'],
        "states": [
            {
                "cur_state": dcss.m_State[0].m_CurState,
                "cmd_state": dcss.m_State[0].m_CmdState,
                "err_code": dcss.m_State[0].m_ERRCode
            },
            {
                "cur_state": dcss.m_State[1].m_CurState,
                "cmd_state": dcss.m_State[1].m_CmdState,
                "err_code": dcss.m_State[1].m_ERRCode
            }
        ]
    }
    # 3. Process real-time output array
    result["outputs"] = [
        {
            "frame_serial": rt_out.m_OutFrameSerial,
            "tip_di": rt_out.m_TipDI,
            "low_speed_flag": rt_out.m_LowSpdFlag,
            "fb_joint_pos": [round(rt_out.m_FB_Joint_Pos[j], 4) for j in range(7)],
            "fb_joint_vel": [round(rt_out.m_FB_Joint_Vel[j], 4) for j in range(7)],
            "fb_joint_posE": [round(rt_out.m_FB_Joint_PosE[j], 4) for j in range(7)],
            "fb_joint_cmd": [round(rt_out.m_FB_Joint_Cmd[j], 4) for j in range(7)],
            "fb_joint_cToq": [round(rt_out.m_FB_Joint_CToq[j], 4) for j in range(7)],
            "fb_joint_sToq": [round(rt_out.m_FB_Joint_SToq[j], 4) for j in range(7)],
            "fb_joint_them": [round(rt_out.m_FB_Joint_Them[j], 4) for j in range(7)],
            "est_joint_firc": [round(rt_out.m_EST_Joint_Firc[j], 4) for j in range(7)],
            "est_joint_firc_dot": [round(rt_out.m_EST_Joint_Firc_Dot[j], 4) for j in range(7)],
            "est_joint_force": [round(rt_out.m_EST_Joint_Force[j], 4) for j in range(7)],
            "est_cart_fn": [round(rt_out.m_EST_Cart_FN[j], 4) for j in range(6)]
        } for rt_out in dcss.m_Out
    ]

    # 4. Process real-time input array (RT_IN)
    result["inputs"] = [
        {
            "rt_in_switch": rt_in.m_RtInSwitch,
            "imp_type": rt_in.m_ImpType,
            "in_frame_serial": rt_in.m_InFrameSerial,
            "frame_miss_cnt": rt_in.m_FrameMissCnt,
            "max_frame_miss_cnt": rt_in.m_MaxFrameMissCnt,
            "sys_cyc": rt_in.m_SysCyc,
            "sys_cyc_miss_cnt": rt_in.m_SysCycMissCnt,
            "max_sys_cyc_miss_cnt": rt_in.m_MaxSysCycMissCnt,
            "tool_kine": [round(rt_in.m_ToolKine[j], 4) for j in range(6)],
            "tool_dyn": [round(rt_in.m_ToolDyn[j], 4) for j in range(10)],
            "joint_cmd_pos": [round(rt_in.m_Joint_CMD_Pos[j], 4) for j in range(7)],
            "joint_vel_ratio": rt_in.m_Joint_Vel_Ratio,
            "joint_acc_ratio": rt_in.m_Joint_Acc_Ratio,
            "joint_k": [round(rt_in.m_Joint_K[j], 4) for j in range(7)],
            "joint_d": [round(rt_in.m_Joint_D[j], 4) for j in range(7)],
            "drag_sp_type": rt_in.m_DragSpType,
            "drag_sp_para": [round(rt_in.m_DragSpPara[j], 4) for j in range(6)],
            "cart_kd_type": rt_in.m_Cart_KD_Type,
            "cart_k": [round(rt_in.m_Cart_K[j], 4) for j in range(6)],
            "cart_d": [round(rt_in.m_Cart_D[j], 4) for j in range(6)],
            "cart_kn": round(rt_in.m_Cart_KN, 4),
            "cart_dn": round(rt_in.m_Cart_DN, 4),
            "force_fb_type": rt_in.m_Force_FB_Type,
            "force_type": rt_in.m_Force_Type,
            "force_dir": [round(rt_in.m_Force_Dir[j], 4) for j in range(6)],
            "force_pidul": [round(rt_in.m_Force_PIDUL[j], 4) for j in range(7)],
            "force_adj_lmt": round(rt_in.m_Force_AdjLmt, 4),
            "force_cmd": round(rt_in.m_Force_Cmd, 4),
            "set_tags": list(rt_in.m_SET_Tags),
            "update_tags": list(rt_in.m_Update_Tags),
            "pvt_id": rt_in.m_PvtID,
            "pvt_id_update": rt_in.m_PvtID_Update,
            "pvt_run_id": rt_in.m_Pvt_RunID,
            "pvt_run_state": rt_in.m_Pvt_RunState
        } for rt_in in dcss.m_In
    ]

    result["ParaName"]=[list(dcss.m_ParaName)]
    result["ParaType"]=[dcss.m_ParaType]
    result["ParaIns"]=[dcss.m_ParaIns]
    result["ParaValueI"]=[dcss.m_ParaValueI]
    result["ParaValueF"]=[dcss.m_ParaValueF]
    result["ParaCmdSerial"]=[dcss.m_ParaCmdSerial]
    result["ParaRetSerial"]=[dcss.m_ParaRetSerial]

    return result

class Marvin_Robot:
    def __init__(self):
        """Initialize robot control class"""
        import sys
        print(f'user platform: {sys.platform}')
        if sys.platform=='win32':
            lib_path = str(_lib_dir / 'libMarvinSDK.dll')
            self.robot = ctypes.WinDLL(lib_path)
        else:
            lib_path = str(_lib_dir / 'libMarvinSDK.so')
            self.robot = ctypes.CDLL(lib_path)
        print(f'Loaded library from: {lib_path}')
        self.ErrorCode = None
        self.a_pvt_path=None
        self.b_pvt_path = None
        self.local_file_path=None
        self.remote_file_path=None
        self.save_csv_path=None
        self.save_data_path=None

    def _convert_ip(self, ip_str):
        """Convert IP string to ctypes array"""
        ip1, ip2, ip3, ip4 = ip_str.split('.')
        ip_uchar = ctypes.c_ubyte
        return ip_uchar(int(ip1)), ip_uchar(int(ip2)), ip_uchar(int(ip3)), ip_uchar(int(ip4))

    def connect(self, robot_ip: str):
        '''Connect to robot
        :param robot_ip: Robot IP address, ensure Ethernet connection is pingable.
        :return:
            int: Connection status code 1: True; 0: False

        eg:
            connect(robot_ip='192.168.1.190')
        '''
        ip1, ip2, ip3, ip4 = self._convert_ip(robot_ip)
        return self.robot.OnLinkTo(ip1, ip2, ip3, ip4)


    def subscribe(self,dcss):
        '''Subscribe to robot status data
        :param dcss:  Structure, see structure_data.py
        :return:
            Nested dictionary
        '''
        self.robot.OnGetBuf(ctypes.byref(dcss))
        result=structure2dict(dcss)
        return result

    def release_robot(self):
        ''' Disconnect from robot
        :return:
            int: Disconnect status code 1: True; 0: False
        '''
        return self.robot.OnRelease()

    def SDK_version(self):
        '''Check SDK version
        :return:
            long: SDK version
        '''
        return self.robot.OnGetSDKVersion()

    def update_SDK(self, sdk_path: str):
        '''Update system SDK version
        :param sdk_path: Local absolute path of SDK file to upload to controller
        :return:
        '''
        sdk_char = ctypes.c_char_p(sdk_path.encode('utf-8'))
        self.robot.OnUpdateSystem(sdk_char)

    def download_sdk_log(self, log_path:str):
        '''Download SDK log to local machine
        :param log_path: Absolute path to download log to local machine
        :return:
        '''
        log_char = ctypes.c_char_p(log_path.encode('utf-8'))
        return self.robot.OnDownloadLog(log_char)


    def get_param(self,type:str,paraName:str):
        '''Get parameter information
        :param type: float or int .Parameterstype
        :param paraName:  Parameter name, see robot.ini
        :return: Parameter value
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            # Get float type parameter:
            I want to get the CartJNTDampJ1 value in the [R.A0.CTRL] parameter group:
            para=get_float_params('float','R.A0.CTRL.CartJNTDampJ1')

            # Get integer type parameter:
            I want to get the Type value in the [R.A0.BASIC] parameter group
            para=get_int_params('int','R.A0.BASIC.Type')
        '''
        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # Explicitly add null terminator
            if type=='float':
                result = ctypes.c_double(0)
                self.robot.OnGetFloatPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetFloatPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, float parameters={result.value}")
                return re_flag,result.value
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnGetIntPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetIntPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, int parameters={result.value}")
                return re_flag, result.value
        except Exception as e:
            print("ERROR:",e)

    def save_para_file(self):
        '''Save configuration file
        :return:
        '''
        self.robot.OnSavePara.restype = ctypes.c_long
        return self.robot.OnSavePara()


    def set_param(self,type:str,paraName:str,value:float):
        '''Set parameter information
        :param type: float or int .Parameterstype
        :param paraName:  Parameter name, see robot.ini
        :param value:
        :return:
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            # Set float type parameter:
            I want to set the CartJNTDampJ1 value in the [R.A0.CTRL] parameter group to 0.0
            set_params('float','R.A0.CTRL.CartJNTDampJ1,0.0)

            # Set integer type parameter:
            I want to set the Type value in the [R.A0.BASIC] parameter group to 0
            set_params('int','R.A0.BASIC.Type',0)
        '''

        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # Explicitly add null terminator
            if type=='float':
                result = ctypes.c_double(value)
                self.robot.OnSetFloatPara.restype = ctypes.c_long
                return self.robot.OnSetFloatPara(param_buf, result)
            elif type=='int':
                result = ctypes.c_int(int(value))
                self.robot.OnSetIntPara.restype = ctypes.c_long
                return self.robot.OnSetIntPara(param_buf, result)
        except Exception as e:
            print("ERROR:",e)

    def clear_set(self):
        '''Clear before sending command
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnClearSet()

    def send_cmd(self):
        '''Send command
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnSetSend()

    def collect_data(self,targetNum:int,targetID:list[int],recordNum:int):
        '''Collect data
        :param targetNum:targetNum: number of columns to collect. Maximum 35, because at most 35 features can be collected at once.
        :param targetID: list(35,1) corresponding data collection ID numbers (see below)
        :param recordNum: Number of rows to collect; less than 1000 will collect 1000 rows; setting more than 1M will collect 1M rows.
        :return:
                    Data collection ID numbers
                    Left arm
                        0-6  	Left armJointposition
                        10-16 	Left arm joint velocities
                        20-26   Left armExternal encoder position
                        30-36   Left armJointcommandposition
                        40-46	Left arm joint current (per mille ratio)
                        50-56   Left arm joint sensor torque (Nm)
                        60-66	Left armFriction estimate
                        70-76	Left armFriction velocity estimate
                        80-85   Left arm joint external force estimates
                        90-95	Left armEnd-effector external force estimate
                    Right arm corresponds to + 100

                    eg1: Collect left arm and right arm joint positions, total 14 columns, collect 1000 rows:
                        cols=14
                        idx=[0,1,2,3,4,5,6,
                             100,101,102,103,104,105,106,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=1000
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)

                    eg2: Collect left arm 2nd joint velocity and current, total 2 columns, collect 500 rows:
                        cols=2
                        idx=[11,31,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=500
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)
        '''
        targetNum_int=ctypes.c_int(targetNum)
        targetID_int=(ctypes.c_long * len(targetID))(*targetID)
        recordNum_int=ctypes.c_int(recordNum)
        return self.robot.OnStartGather(targetNum_int,targetID_int,recordNum_int)

    def stop_collect_data(self):
        '''Stop collecting data
        Note: Collection will automatically stop when the specified number of rows is reached. To stop collection midway, call this function and wait 1ms for collection to stop.
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnStopGather()

    def save_collected_data_to_path(self,path:str):
        '''Save collected data to specified absolute path
        :param path:Local absolute path
        :return:
        '''
        self.save_data_path=path.encode('utf-8')
        path_char=ctypes.c_char_p(self.save_data_path)
        return self.robot.OnSaveGatherData(path_char)

    def save_collected_data_as_csv_to_path(self,path:str):
        '''Save collected data in CSV format to specified absolute path
        :param path:Local absolute path
        :return:
        '''
        path1='tmp.txt'
        self.save_data_path = path1.encode('utf-8')
        path_char = ctypes.c_char_p(self.save_data_path)
        self.robot.OnSaveGatherData(path_char)
        import time
        time.sleep(0.2)

        import csv
        with open(path1, 'r') as file:
            lines = file.readlines()
        processed_data=[]
        lines = lines[1:]
        for i, line in enumerate(lines):
            parts = line.strip().split('$')
            numbers = []
            for part in parts:
                if part:
                    num_str = part.split()[-1]
                    numbers.append(num_str)
            if len(numbers) >= 2:
                numbers = numbers[2:]
            processed_data.append(numbers)

        try:
            with open(path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(processed_data)
            print(f"Data saved successfully to: {path}")
            if os.path.exists(path1):
                os.remove(path1)
            return True
        except Exception as e:
            print(f"Save failed: {e}")
            if os.path.exists(path1):
                os.remove(path1)
            return False


    def soft_stop(self, arm:str):
        '''Robot arm emergency stop
        :param arm: ‘A’, 'B', 'AB', Can soft-stop one arm, or soft-stop both arms.
        :return:
        '''
        try:
            if arm=='A':
                return self.robot.OnEMG_A()
            elif arm=='B':
                return self.robot.OnEMG_B()
            elif arm=='AB':
                return self.robot.OnEMG_AB()
        except Exception as e:
            print("ERROR:", e)


    def get_servo_error_code(self, arm:str):
       '''Get robot arm servo error code
       :param self:
       :param arm:
       :return: (7,1) error list, hexadecimal
       '''
       try:
           err_code_value = (ctypes.c_long * 7)()
           if arm=='A':
               self.robot.OnGetServoErr_A.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_A(ctypes.byref(err_code_value))
               # print('err_code_value',err_code_value[-1])
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code
           elif arm=='B':
               self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_B(ctypes.byref(err_code_value))
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code

       except Exception as e:
           print("ERROR:", e)


    def clear_error(self,arm:str):
        '''Clear errors
        :return:none
        '''
        try:
            if arm=='A':
                return self.robot.OnClearErr_A()
            elif arm=='B':
                return self.robot.OnClearErr_B()
        except Exception as e:
            print(f'ERROR:{e}')


    def set_state(self,arm:str,state:int):
        '''Set state
        :param state:
                   ARM_STATE_IDLE = 0,            //////// Servo off
                   ARM_STATE_POSITION = 1,		//////// Position follow
                   ARM_STATE_PVT = 2,			//////// PVT
                   ARM_STATE_TORQ = 3,			//////// Torque
                   ARM_STATE_RELEASE = 4,		//////// Collaborative release

        :return:
        '''
        try:
            state_int = ctypes.c_int(state)
            if arm=="A":
                return self.robot.OnSetTargetState_A(state_int)
            elif arm=='B':
                return self.robot.OnSetTargetState_B(state_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_impedance_type(self, arm:str,type: int):
        '''Set impedance type
        :param type:
            Type = 1 Joint impedance
            Type = 2 Cartesian impedance
            Type = 3 Force control
            Note: Must be in ARM_STATE_TORQ state: set_state(arm='A',state=3) to control in impedance mode!!!
        :return:
            int : 1: True,  2: False
        '''
        try:
            type_int = ctypes.c_int(type)
            if arm=='A':
                return self.robot.OnSetImpType_A(type_int)
            elif arm == 'B':
                return self.robot.OnSetImpType_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_vel_acc(self, arm:str, velRatio: int, AccRatio: int):
        '''Set velocity and acceleration ratio
        :param velRatio: Velocity percentage
        :param AccRatio: Acceleration percentage
        :return:
            int: 1: True; 0: False
        '''
        try:
            velRatio_int = ctypes.c_int(velRatio)
            AccRatio_int = ctypes.c_int(AccRatio)
            if arm=='A':
                return self.robot.OnSetJointLmt_A(velRatio_int, AccRatio_int)
            elif arm=='B':
                return self.robot.OnSetJointLmt_B(velRatio_int, AccRatio_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_tool(self,arm:str, kineParams: list, dynamicParams: list):
        '''Set tool information
        :param kineParams: list(6,1). kinematicsParameters XYZABC Unit: mm and degrees
        :param dynamicParams: list(10,1). dynamics parameters: mass M, center of mass [3]: mx,my,mz, inertia I[6]: XX,XY,XZ,YY,YZ,ZZ
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5 = kineParams
            d0, d1, d2, d3, d4, d5, d6, d7, d8, d9 = dynamicParams
            kp_double = ctypes.c_double * 6
            kineParams_value = kp_double(k0, k1, k2, k3, k4, k5)
            dp_double = ctypes.c_double * 10
            dynamicParams_value = dp_double(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9)
            if arm=='A':
                return self.robot.OnSetTool_A(kineParams_value, dynamicParams_value)
            if arm=='B':
                return self.robot.OnSetTool_B(kineParams_value, dynamicParams_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_kd_params(self,arm:str, K: list, D: list):
        '''Set joint impedance parameters

        # For joint impedance, lower stiffness is needed to avoid vibration, and compliance is desired, so low stiffness with low damping is used.
        Joints 1-7 stiffness should not exceed 2
        Joints 1-7 damping between 0-1
        :param K: list(7,1). Stiffness Nm/degree.Set the force for each axis as stiffness coefficient. e.g. K=[2,2,2,1,1,1,1], axes 1-3 have 2N as stiffness coefficient for control computation, axes 4-7 have 1N as stiffness coefficient for control computation.
        :param D: list(7,1). damping Nm / (degrees / second). Set the damping coefficient for each axis.
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D

            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            if arm=="A":
                return self.robot.OnSetJointKD_A(k_value, d_value)
            elif arm == "B":
                return self.robot.OnSetJointKD_B(k_value, d_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_cart_kd_params(self, arm:str, K: list, D: list, type: int):
        '''Set Cartesian impedance parameters
            # In Cartesian impedance mode:
            Stiffness coefficients: 1-3 translation stiffness coefficient should not exceed 3000, 4-6 rotation should not exceed 100. Nullspace stiffness coefficient should not exceed 20
            Damping coefficients: Translation and rotation damping coefficient between 0-1. Nullspace damping coefficient should not exceed 1
            Nullspace control keeps end-effector fixed while arm angle moves. Interface not yet available.

        :param K: list(7,1). K[0]-K[2] N*m, x,y,z control force per meter in translation direction; K[3]-k[5] N*m/rad, rx,ry,rz control force per radian of rotation; K[6] N*m/rad, nullspace total stiffness coefficient
        :param D: list(7,1). D[0]-D[5]  Damping ratio coefficient, D[6] Nullspace total damping ratio coefficient
        :param type:int. set_A_arm_impedance_typesetImpedance type
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D
            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            type_int = ctypes.c_int(type)
            if arm=="A":
                return self.robot.OnSetCartKD_A(k_value, d_value, type_int)
            if arm == "B":
                return self.robot.OnSetCartKD_B(k_value, d_value, type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_force_control_params(self,arm:str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float):
        '''Set force control parameters
        :param fcType: Force control type. 0: Cartesian space force control; 1: Tool space force control (not yet implemented)
        :param fxDirection: list(6,1). Force control direction: set controlled direction to 1. Currently only supports X,Y,Z control directions. For example, if force control direction is z, fxDirection=[0,0,1,0,0,0]
        :param fcCtrlpara: list(7,1). Control parameters, currently all 0
        :param fcAdjLmt:Millimeters, allowable adjustment range
        :return:
            int : 1: True,  2: False
        '''
        try:
            fc_int=ctypes.c_int(fcType)
            k0, k1, k2, k3, k4, k5 = fxDirection
            d0, d1, d2, d3, d4, d5, d6 = fcCtrlpara
            fxDir_arr = (ctypes.c_double * 6)( k0, k1, k2, k3, k4, k5 )
            fcCtrlPara_arr = (ctypes.c_double * 7)(d0, d1, d2, d3, d4, d5, d6 )
            adj_double=ctypes.c_double(fcAdjLmt)
            if arm=='A':
                return self.robot.OnSetForceCtrPara_A(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
            elif arm=='B':
                return self.robot.OnSetForceCtrPara_B(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_cmd_pose(self,arm:str, joints:list):
        '''Set joint tracking command values
        :param joints: list(7,1). Degrees (not radians), effective in both position follow and torque modes
        :return:
            int : 1: True,  2: False
        '''
        try:
            j0, j1, j2, j3, j4, j5, j6= joints
            joints_double = ctypes.c_double * 7
            joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)
            if arm=='A':
                return self.robot.OnSetJointCmdPos_A(joints_value )
            elif arm == 'B':
                return self.robot.OnSetJointCmdPos_B(joints_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_force_cmd(self,arm:str, f:float):
        '''Set force control parameters
        :param f: Target force, unit: N or Nm
        :return:
            int : 1: True,  2: False
        '''
        try:
            f_double=ctypes.c_double(f)
            if arm=='A':
                return self.robot.OnSetForceCmd_A(f_double)
            elif arm == 'B':
                return self.robot.OnSetForceCmd_B(f_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_pvt_id(self,arm:str,id:int):
        '''Set the specified PVT ID and run
        :param id: Range 1-99. Must be in ARM_STATE_PVT state, i.e.: set_arm_state(arm='A',state=2)
        :return:
            int : 1: True,  2: False
        '''
        try:
            if arm=='B':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_B(id_int)
            elif arm=='A':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_A(id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def send_pvt_file(self,arm:str, pvt_path: str, id: int):
        '''Upload PVT file to specified ID
        :param pvt_path: Local pvt file absolute/relative path
        :param id:
        :return:


            PVT file format: see DEMO_SRS_Left.fmv
            The first row contains row count and column count info. “PoinType=9@9341” means the PVT file contains 9 columns of data with 9341 total points.
            Why 9 columns? The first 8 columns are joint angles. Why 8? We reserve 8 joints; the humanoid arm has 7 DOF, so the first 7 values are valid and the 8th column is filled with 0.
            The 9th column is a tag column, fill all with 0.
        '''
        try :
            if arm=='A':
                self.a_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.a_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return  self.robot.OnSendPVT_A(pvt_char, id_int)
            elif arm=='B':
                self.b_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.b_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return self.robot.OnSendPVT_B(pvt_char, id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_drag_space(self,arm:str, dgType: int):
        '''Set drag space
        :param dgType:
                0 Exit drag mode
                1 Joint space drag
                2 Cartesian space x-direction drag
                3 Cartesian space y-direction drag
                4 Cartesian space z-direction drag
                5 Cartesian space rotation drag
        :return:
        '''
        try:
            type_int = ctypes.c_int(dgType)
            if arm=='A':
                return self.robot.OnSetDragSpace_A(type_int)
            elif arm=='B':
                return self.robot.OnSetDragSpace_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def receive_file(self, local_path: str, remote_path: str):
        '''Download file from robot controller to host computer
        :param local_path: Local absolute path
        :param remote_path: Robot controller absolute path
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnRecvFile(local_char, remote_char)


    def send_file(self, local_path: str, remote_path: str):
        '''Upload file from host computer to robot controller
        :param local_path: Local absolute path
        :param remote_path: Robot controller absolute path
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnSendFile(local_char, remote_char)


    def log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLogOn()
            elif flag=='0':
                return self.robot.OnLogOff()
        except Exception as e:
            print(f'ERROR:{e}')


    def local_log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLocalLogOn()
            elif flag=='0':
                return self.robot.OnLocalLogOff()
        except Exception as e:
            print(f'ERROR:{e}')

    def clear_485_cache(self,arm:str):
        '''Clear send buffer

        :param arm: Robot arm ID “A” OR “B”
        :return: bool
        '''
        try:
            if arm == 'A':
                return self.robot.OnClearChDataA()
            elif arm == 'B':
                return self.robot.OnClearChDataB()
        except Exception as e:
            print(f'ERROR:{e}')

    def set_485_data(self, arm: str, data:bytes, size_int:int,com:int):
        '''Send data to specified 485 source. Each time length not exceeding 256 bytes; if exceeded, split into multiple packets.

        :param arm: Robot arm ID “A” OR “B”
        :param data: Byte data to transmit (Length not exceeding 256)
        :param size_int: int, Send byte length, cannot exceed 256
        :param com: Data source, 1: ‘C’ end; 2: com1; 3: com2
        :return: bool
        '''

        try:
            # Define function prototype
            self.robot.OnSetChDataA.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataA.restype = ctypes.c_bool

            # Define function prototype
            self.robot.OnSetChDataB.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataB.restype = ctypes.c_long

            # Verify parameters
            if len(data) >= 257:
                raise ValueError(f"data length ({len(data)}) exceeds 256 byte limit")
            if size_int >= 257:
                print(f"size_int({size_int})exceeds 256, will be truncated")
                size_int = 256

            result = identify_and_calculate_length(data)
            if result['type'] == "hex string" or result['type'] == 'bytes' or result[
                'type'] == "bytes representation string":
                print("-" * 50)
                print(f"input: {data}")
                print(f"type: {result['type']}")
                print(f"byte length: {result['length_bytes']}")
                print(f"bytes representation: {result['bytes_representation']}")
                print("-" * 50)
            else:
                print(f"ERROR: set_485_data input must be hex string of bytes string")
                return False, False

            size_int_long = ctypes.c_long(result['length_bytes'])
            com_long = ctypes.c_long(com)

            data_buffer = (ctypes.c_ubyte * 256)()
            # Copy data to buffer
            data_length = min(len(result['bytes_representation']), size_int)
            for i in range(data_length):
                data_buffer[i] = result['bytes_representation'][i]
            if arm == 'A':
                return True, self.robot.OnSetChDataA(data_buffer, size_int_long, com_long)
            elif arm == 'B':
                return True, self.robot.OnSetChDataB(data_buffer, size_int_long, com_long)
        except Exception as e:
            print(f'ERROR:{e}')


    def get_485_data(self, arm: str,com:int):
        '''Receive 485 data from specified source
        :param arm: Robot arm ID “A” OR “B”
        :param com: Data source, 1: ‘C’ end; 2: com1; 3: com2
        :return: int, Length (size)
        '''
        try:
            # Create 256-byte buffer
            data_buffer = (ctypes.c_ubyte * 256)()
            ret_ch = ctypes.c_long(com)
            if arm == 'A':
                result = self.robot.OnGetChDataA(data_buffer, ctypes.byref(ret_ch))
                # Extract byte data
                byte_data = bytes(data_buffer)  # or bytearray(data_buffer)
                print(f'arm receive byte_data :{byte_data}')
                hex_list = []
                for byte in byte_data:
                    # Convert each byte to two-digit hexadecimal
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

            elif arm == 'B':
                result = self.robot.OnGetChDataB(data_buffer, ctypes.byref(ret_ch))
                # Extract byte data
                byte_data = bytes(data_buffer)  # or bytearray(data_buffer)
                # print(f'B arm receive byte_data :{byte_data }')
                hex_list = []
                for byte in byte_data:
                    # Convert each byte to two-digit hexadecimal
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

        except Exception as e:
            print(f'ERROR:{e}')

    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''Tool dynamics parameter identification
        FX_BOOL  FX_Robot_Iden_LoadDyn(FX_INT32L Type,FX_CHAR* path,FX_DOUBLE mass, Vect3 mr, Vect6 I);
        :param robot_type: int, Robot model, imported from CONFIG
        :param ipath: sting, Import tool identification trajectory data via relative path.
        :return:
            m,mcp,i
        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        robot_type_ = ctypes.c_int(robot_type)
        iden_path = ipath.encode('utf-8')
        path_char = ctypes.c_char_p(iden_path)

        # Create pointer variables instead of arrays
        mm_ptr = ctypes.pointer(ctypes.c_double(0))
        mcp_ptr = (ctypes.c_double * 3)()
        ii_ptr = (ctypes.c_double * 6)()

        # Set function prototype
        self.robot.FX_Robot_Iden_LoadDyn.argtypes = [
            ctypes.c_long,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double * 3),
            ctypes.POINTER(ctypes.c_double * 6)
        ]
        self.robot.FX_Robot_Iden_LoadDyn.restype = ctypes.c_bool

        # Call function
        success1 = self.robot.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )

        if success1:
            print('Identify tool dynamics successful')

            # Extract result
            dyn_para = []
            m_val = mm_ptr.contents.value
            mcp_list = [mcp_ptr[i] for i in range(3)]
            ii_list = [ii_ptr[i] for i in range(6)]

            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)
            for j in ii_list:
                dyn_para.append(j)

            print(f'tool dynamics: {dyn_para}')
            return dyn_para
        else:
            print('****error: Identify tool dynamics failed!')
            return False


    def get_tool_info(self,):
        '''Check if controller has saved tool information
        :return:
           m,mcp,i
        '''

        '''tool '''
        local_path='tool_dyn_kine.txt'
        remote_path='/home/fusion/tool_dyn_kine.txt'
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        self.robot.OnRecvFile(local_char, remote_char)
        time.sleep(1)
        tool_result = read_csv_file_to_float_strict(local_path, expected_columns=16)
        return tool_result



    def help(self, method_name: str = None) -> None:
        """
        Display help information

        Parameters:
            method_name (str): Optional method name. Display help for specific method info
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



if __name__ == "__main__":
    import time

    tj_robot = Marvin_Robot()
    tj_robot.help()
    tj_robot.help('collect_data')

