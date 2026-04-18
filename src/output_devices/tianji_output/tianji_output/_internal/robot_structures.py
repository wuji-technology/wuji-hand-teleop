from ctypes import *

# Define basic types
FX_INT32L = c_long
FX_DOUBLE = c_double
FX_BOOL = c_bool


# Define Vect7 type (array of 7 doubles)
class Vect7(Structure):
    _fields_ = [("data", FX_DOUBLE * 7)]

    def __init__(self, values=None):
        super().__init__()
        if values is not None:
            if len(values) != 7:
                raise ValueError("Vect7 requires exactly 7 values")
            for i, val in enumerate(values):
                self.data[i] = val

    def to_list(self):
        return [self.data[i] for i in range(7)]

    def __str__(self):
        return str(self.to_list())


# Define Matrix4 type (4x4 matrix, 16 doubles)
class Matrix4(Structure):
    _fields_ = [("data", FX_DOUBLE * 16)]

    def __init__(self, values=None):
        super().__init__()
        if values is not None:
            if len(values) != 16:
                raise ValueError("Matrix4 requires exactly 16 values")
            for i, val in enumerate(values):
                self.data[i] = val

    def to_list(self):
        return [self.data[i] for i in range(16)]

    def __str__(self):
        return str(self.to_list())


# Define main structure FX_InvKineSolvePara
class FX_InvKineSolvePara(Structure):
    _fields_ = [
        ("m_Input_IK_TargetTCP", Matrix4),
        ("m_Input_IK_RefJoint", Vect7),
        ("m_Input_IK_ZSPType", FX_INT32L),
        ("m_Input_IK_ZSPPara", FX_DOUBLE * 6),
        ("m_Input_ZSP_Angle", FX_DOUBLE),
        ("m_DGR1", FX_DOUBLE),
        ("m_DGR2", FX_DOUBLE),
        ("m_DGR3", FX_DOUBLE),
        ("m_Output_RetJoint", Vect7),
        ("m_Output_IsOutRange", FX_BOOL),
        ("m_Output_IsDeg", FX_BOOL * 7),
        ("m_Output_JntExdTags", FX_BOOL * 7),
        ("m_Output_IsJntExd", FX_BOOL),
        ("m_Output_RunLmtP", Vect7),
        ("m_Output_RunLmtN", Vect7)
    ]

    def __init__(self):
        super().__init__()
        # Initialize arrays
        for i in range(6):
            self.m_Input_IK_ZSPPara[i] = 0.0

        # Initialize boolean arrays
        for i in range(7):
            self.m_Output_IsDeg[i] = False
            self.m_Output_JntExdTags[i] = False

    def set_input_ik_zsp_para(self, values):
        if len(values) != 6:
            raise ValueError("m_Input_IK_ZSPPara requires exactly 6 values")
        for i, val in enumerate(values):
            self.m_Input_IK_ZSPPara[i] = val

    def get_input_ik_zsp_para(self):
        return [self.m_Input_IK_ZSPPara[i] for i in range(6)]

    def set_output_is_deg(self, values):
        if len(values) != 7:
            raise ValueError("m_Output_IsDeg requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Output_IsDeg[i] = val

    def get_output_is_deg(self):
        return [self.m_Output_IsDeg[i] for i in range(7)]

    def set_output_jnt_exd_tags(self, values):
        if len(values) != 7:
            raise ValueError("m_Output_JntExdTags requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Output_JntExdTags[i] = val

    def get_output_jnt_exd_tags(self):
        return [self.m_Output_JntExdTags[i] for i in range(7)]

# Define FX_Jacobi structure
class FX_Jacobi(Structure):
    _fields_ = [
        ("m_AxisNum", FX_INT32L),
        ("m_Jcb", (FX_DOUBLE * 7) * 6)  # 6x7 two-dimensional array
    ]

    def __init__(self):
        super().__init__()
        self.m_AxisNum = 0
        # Initialize two-dimensional array to 0
        for i in range(6):
            for j in range(7):
                self.m_Jcb[i][j] = 0.0

    def set_jcb(self, matrix):
        """
        Set Jacobian matrix values

        Parameters:
        matrix: 6x7 two-dimensional list or numpy array
        """
        if len(matrix) != 6 or any(len(row) != 7 for row in matrix):
            raise ValueError("Jacobian matrix must be a 6x7 two-dimensional array")

        for i in range(6):
            for j in range(7):
                self.m_Jcb[i][j] = matrix[i][j]

    def get_jcb(self):
        """
        Get Jacobian matrix values

        Returns:
        6x7 two-dimensional list
        """
        result = []
        for i in range(6):
            row = []
            for j in range(7):
                row.append(self.m_Jcb[i][j])
            result.append(row)
        return result

    def __str__(self):
        """
        Return string representation of Jacobian matrix
        """
        result = f"AxisNum: {self.m_AxisNum}\nJacobian Matrix:\n"
        for i in range(6):
            row = [f"{self.m_Jcb[i][j]:.6f}" for j in range(7)]
            result += "  " + "  ".join(row) + "\n"
        return result


# Example usage
if __name__ == "__main__":
    # Create structure instance
    params = FX_InvKineSolvePara()

    # Set input values
    params.m_Input_IK_TargetTCP = Matrix4([i for i in range(16)])
    params.m_Input_IK_RefJoint = Vect7([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    params.m_Input_IK_ZSPType = 1
    params.set_input_ik_zsp_para([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    params.m_Input_ZSP_Angle = 45.0
    params.m_DGR1 = 10.0
    params.m_DGR2 = 20.0
    params.m_DGR3 = 30.0

    # Set output values (assuming returned from C++ function)
    params.m_Output_RetJoint = Vect7([1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7])
    params.m_Output_IsOutRange = False
    params.set_output_is_deg([True, False, True, False, True, False, True])
    params.set_output_jnt_exd_tags([False, True, False, True, False, True, False])
    params.m_Output_IsJntExd = True
    params.m_Output_RunLmtP = Vect7([2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7])
    params.m_Output_RunLmtN = Vect7([3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7])

    # Print some values for verification
    print("Input IK ZSP Para:", params.get_input_ik_zsp_para())
    print("Output IsDeg:", params.get_output_is_deg())
    print("Output JntExdTags:", params.get_output_jnt_exd_tags())
    print("Output RetJoint:", params.m_Output_RetJoint.to_list())


    # create FX_Jacobi instance
    jacobi = FX_Jacobi()

    # Set axis count
    jacobi.m_AxisNum = 6

    # Create an example Jacobian matrix (6x7)
    example_matrix = [
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ]

    # Set Jacobian matrix
    jacobi.set_jcb(example_matrix)

    # Print structure contents
    print(jacobi)

    # Get and print Jacobian matrix
    matrix = jacobi.get_jcb()
    print("Retrieved Jacobian matrix:")
    for row in matrix:
        print(row)
