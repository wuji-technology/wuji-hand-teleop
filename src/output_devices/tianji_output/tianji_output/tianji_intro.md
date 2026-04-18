
# 7-DOF Robotic Arm Redundancy Parameter Documentation

## 1. Core Concept: Redundancy
For a 7-DOF robotic arm, given an end-effector pose (position + orientation, 6 constraints total), there exist **infinitely many** joint angle solutions. Geometrically, these solutions manifest as: while keeping the hand and shoulder positions fixed, the elbow can rotate around the **"shoulder-wrist line"**.

To select a unique configuration from the infinitely many solutions, the SDK introduces two key parameters:
1.  **Arm Vector**: Defines the reference direction for $0^\circ$.
2.  **Arm Angle**: The rotation angle relative to the reference direction.

---

## 2. Geometric Model
We simplify the robotic arm into a triangular relationship between **shoulder point ($P_A$)**, **elbow point ($P_{Elbow}$)**, and **wrist point ($P_B$)**.
* **Rotation axis**: The vector from the shoulder point to the wrist point.
* **Orbit circle**: The elbow moves on a plane perpendicular to the rotation axis.

### Parameter Details

#### 2.1 Arm Vector (ZSPPara / Reference Vector)
Defines the **"zero position plane"**. The algorithm automatically projects the user-input vector onto the plane perpendicular to the rotation axis.

* **SDK mapping**: `m_Input_IK_ZSPPara`
* **Common settings**:
    * `[0, 0, -1]`: **Elbow naturally pointing down** (recommended default).
    * `[0, 0, 1]`: Elbow pointing up (to avoid obstacles below).
    * `[0, 1, 0]`: Elbow pointing sideways.

#### 2.2 Arm Angle (ZSP_Angle / Arm Angle)
Defines the **rotation angle** of the elbow relative to the reference plane.

* **SDK mapping**: `m_Input_ZSP_Angle`
* **Unit**: Degrees.
* **Purpose**: After determining the general direction (set by the arm vector), fine-tune obstacle avoidance by adjusting this angle.

---

## 3. SDK Structure Mapping (FX_InvKineSolvePara)

When using the debugging tool, the data fields on the right side directly correspond to SDK inputs and outputs:

### Input
* `m_Input_IK_TargetTCP`: Target end-effector 4x4 matrix (containing position and rotation).
* `m_Input_IK_ZSPType`: Set to **1** (NEAR_DIR mode).
* `m_Input_IK_ZSPPara`: Input reference vector (e.g., `0,0,-1`).
* `m_Input_ZSP_Angle`: Arm angle (e.g., `30.0` degrees).

### Output
* `m_Output_RetJoint`: Computed joint angles.
* `m_Output_IsOutRange`: **[Important]** If True (red), the target point is too far and the robotic arm cannot reach it. Check the `TCP` coordinates in this case.

---

## 4. Debugging Tool User Guide

1.  **Modify position**: Drag the `TCP X/Y/Z` sliders, or type numbers directly into the text fields and press Enter.
2.  **Observe redundancy**: Keep TCP unchanged, only drag `Arm Angle`, and observe how the elbow rotates around the axis.
3.  **Understand the reference vector**: 
    * Check `Show Ref Vector`.
    * Modify `RefVec Z`.
    * Observe how the green reference arrow changes the starting position of $0^\circ$.
