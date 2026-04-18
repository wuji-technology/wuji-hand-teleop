from ctypes import *


# Define StateCtr structure
class StateCtr(Structure):
    _fields_ = [
        ("m_CurState", c_int),  # * Current state */ ArmState
        ("m_CmdState", c_int),  # * Command state */ DCSSCmdType 0
        ("m_ERRCode", c_int)  # * Error code   */
    ]


# Define RT_IN structure
class RT_IN(Structure):
    _fields_ = [
        ("m_RtInSwitch", c_int),  # * Real-time input switch for user real-time data on/off: 0 - close rt_in; 1 - open rt_in *
        ("m_ImpType", c_int),  #Impedance type
        ("m_InFrameSerial", c_int),  # short input frame serial number 0 - 1000000 modulo
        ("m_FrameMissCnt", c_short),  # short frame miss count
        ("m_MaxFrameMissCnt", c_short),  # short max frame miss count before triggering backward

        ("m_SysCyc", c_int),  # 0 -  1000000
        ("m_SysCycMissCnt", c_short),  # short Real-time miss count
        ("m_MaxSysCycMissCnt", c_short),  # short max real-time miss count before triggering backward

        ("m_ToolKine", c_float * 6),  # Tool kinematics parameters 1
        ("m_ToolDyn", c_float * 10),  # Tool dynamics parameters 1

        ("m_Joint_CMD_Pos", c_float * 7),  # Joint position command
        ("m_Joint_Vel_Ratio", c_short),  # short Joint velocity limit percentage 2
        ("m_Joint_Acc_Ratio", c_short),  # short Joint acceleration limit percentage 2

        ("m_Joint_K", c_float * 7),  # Joint impedance stiffness K command 3
        ("m_Joint_D", c_float * 7),  # Joint impedance damping D command 4

        ("m_DragSpType", c_int),  # Nullspace type 5
        ("m_DragSpPara", c_float * 6),  # Nullspace parameter type 5

        ("m_Cart_KD_Type", c_int),  # Cartesian impedance type
        ("m_Cart_K", c_float*6),  # Cartesian impedance stiffness K command 4
        ("m_Cart_D", c_float*6),  # Cartesian impedance damping D command 4
        ("m_Cart_KN", c_float),  # 4
        ("m_Cart_DN", c_float),  # 4

        ("m_Force_FB_Type", c_int),  # Force control feedback source type
        ("m_Force_Type", c_int),  # Force control type 6
        ("m_Force_Dir", c_float * 6),  # Force control direction in 6D space 6
        ("m_Force_PIDUL", c_float * 7),  # Force control PID 6
        ("m_Force_AdjLmt", c_float),  # Maximum allowable adjustment range 6

        ("m_Force_Cmd", c_float),  # Force control command 8

        ("m_SET_Tags", c_ubyte * 16),  # Nullspace type 5
        ("m_Update_Tags", c_ubyte * 16),  # Nullspace type 5

        ("m_PvtID", c_ubyte),  #Set PVT ID
        ("m_PvtID_Update", c_ubyte),  #PVT ID update status
        ("m_Pvt_RunID", c_ubyte), #0: no pvt file; 1~99: user uploaded PVT
        ("m_Pvt_RunState", c_ubyte),  #0: idle; 1: loading ; 2: running; 3: error

    ]


# Define RT_OUT structure
class RT_OUT(Structure):
    _fields_ = [
        ("m_OutFrameSerial", c_int),  # output frame serial number 0 - 1000000 modulo
        ("m_FB_Joint_Pos", c_float * 7),  # Joint position feedback
        ("m_FB_Joint_Vel", c_float * 7),  # Joint velocity feedback
        ("m_FB_Joint_PosE", c_float * 7),  # Joint position (external encoder)
        ("m_FB_Joint_Cmd", c_float * 7),  # Joint position command
        ("m_FB_Joint_CToq", c_float * 7),  # Joint command torque
        ("m_FB_Joint_SToq", c_float * 7),  # Joint actual torque
        ("m_FB_Joint_Them", c_float * 7),  # Joint temperature
        ("m_EST_Joint_Firc", c_float * 7),  # Joint friction estimate
        ("m_EST_Joint_Firc_Dot", c_float * 7),  # Joint force disturbance estimate derivative
        ("m_EST_Joint_Force", c_float * 7),  # Joint force disturbance estimate
        ("m_EST_Cart_FN", c_float * 6),  # End-effector Cartesian space force disturbance estimate
        ("m_TipDI", c_char),  # End-effector digital input
        ("m_LowSpdFlag", c_char),  # Low speed flag
        # ("m_pad", c_char * 2)  # Padding bytes
    ]


# Define DCSS structure
class DCSS(Structure):
    _fields_ = [
        ("m_State", StateCtr * 2),  # Status controller array
        ("m_In", RT_IN * 2),  # Input data array
        ("m_Out", RT_OUT * 2),  # Output data array

        ("m_ParaName", c_char * 30),  # Parameter name, related to robot configuration parameters
        ("m_ParaType", c_ubyte),  # 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
        ("m_ParaIns", c_ubyte),  # DCSSCfgOperationType
        ("m_ParaValueI", c_int),  # FX_INT32 value
        ("m_ParaValueF", c_float),  # FX_FLOAT value
        ("m_ParaCmdSerial", c_short),  # short from PC
        ("m_ParaRetSerial", c_short),  # short working: 0; finish: cmd serial; error cmd_serial + 100
    ]





def call_on_get_buf():
    # Create DCSS structure instance
    dcss = DCSS()

    # Initialize structure data (optional)
    dcss.m_ParaName = b"DefaultParameters"  # byte string

    # Call C function
    result = lib.OnGetBuf(byref(dcss))

    if not result:
        print("Function call failed")
        return None

    # Process return data
    return process_dcss_data(dcss)


def process_dcss_data(dcss):
    """Process and extract data from DCSS structure"""
    data = {}

    # Extract parameter name
    data['para_name'] = dcss.m_ParaName.decode().strip('\x00')

    # Process two StateCtr instances
    states = []
    for i in range(2):
        state = {
            'cur_state': dcss.m_State[i].m_CurState,
            'cmd_state': dcss.m_State[i].m_CmdState,
            'err_code': dcss.m_State[i].m_ERRCode
        }
        states.append(state)
    data['states'] = states

    # Process two RT_OUT instances
    outputs = []
    for i in range(2):
        output = {
            'frame_serial': dcss.m_Out[i].m_OutFrameSerial,
            'joint_pos': list(dcss.m_Out[i].m_FB_Joint_Pos),
            'joint_vel': list(dcss.m_Out[i].m_FB_Joint_Vel),
            # Other fields can be added as needed...
            'tip_di': dcss.m_Out[i].m_TipDI,
            'low_speed_flag': dcss.m_Out[i].m_LowSpdFlag
        }
        outputs.append(output)
    data['outputs'] = outputs

    return data


# Example usage
if __name__ == "__main__":
    '''
    ctypes      type	C           equivalent type	Size (bytes)	        Value range
    c_int16	    int16_t     	    2	                    -32,768 to 32,767
    c_ubyte	    unsigned char	    1	                    0 to 255
    c_int8	    int8_t	            1	                    -128 to 127
    c_uint8	    uint8_t	            1	                    0 to 255
    c_int32	    int32_t	            4	                    -2,147,483,648 to 2,147,483,647
    c_uint32	uint32_t	        4	                    0 to 4,294,967,295
    c_float	    float	            4	                    approx. ±3.4e38 (7 digits precision)
    c_double	double	            8	                    approx. ±1.8e308 (15 digits precision)
    c_char	    char	            1	                    -128 to 127 or 0 to 255
    c_bool	    bool (C99+)	        1	                    0 or 1
    c_size_t	size_t	            4or8	                 platform dependent
    '''
    lib = CDLL("MarvinLib/libMarvinSDK.so")  # Replace with actual library path
    lib.OnGetBuf.argtypes = [POINTER(DCSS)]
    lib.OnGetBuf.restype = c_bool

    # Check whether structure sizes match the C side
    print(f"StateCtr size: {sizeof(StateCtr)} ")
    print(f"RT_IN size: {sizeof(RT_IN)} ")
    print(f"RT_OUT size: {sizeof(RT_OUT)} ")
    print(f"DCSS size: {sizeof(DCSS)} ")
    # result_data = call_on_get_buf()
    # if result_data:
    #     import pprint
    #
    #     pprint.pprint(result_data)
    # dcss = DCSS()
    # print(dcss.m_State)
