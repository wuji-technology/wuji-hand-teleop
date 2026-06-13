#!/usr/bin/env python3
"""
Coordinate Transform Utility Functions (Authoritative Shared Library)

Single authoritative implementation of all coordinate/rotation transform functions.
pico_input_node, step1-6 test scripts all import from here.

Config source: tianji_world_output/config/tianji_robot.yaml (Single Source of Truth)

=============================================================================
Positive Rotation Direction Convention (ROS REP 103 Right-Hand Rule)
=============================================================================

  How to determine: Point right thumb along the positive axis direction,
  fingers curl in the positive rotation direction.
  Equivalent: Looking from the positive end of the axis toward the origin,
  counterclockwise = positive direction.

  Robot World coordinate frame (X=forward, Y=left, Z=up):

    Positive rotation around +X (Roll):  Y->Z, i.e. left->up -> top of wrist tilts right (Roll right)
    Positive rotation around +Y (Pitch): Z->X, i.e. up->forward -> fingertips press down (Pitch down)
    Positive rotation around +Z (Yaw):   X->Y, i.e. forward->left -> fingertips point left (Yaw left)

  Mnemonic: "Positive Roll tilts right, positive Pitch looks down, positive Yaw turns left"

  Mathematical verification (standard rotation matrices):
    Rx(t)@[0,0,1] = [0, -sin(t), cos(t)]  -> up direction tilts right -> Roll right
    Ry(t)@[1,0,0] = [cos(t), 0, -sin(t)]  -> forward direction presses down -> Pitch down
    Rz(t)@[1,0,0] = [cos(t), sin(t), 0]   -> forward direction points left -> Yaw left

=============================================================================
Coordinate Frame Definitions
=============================================================================

  World (ROS REP 103): X=forward, Y=left, Z=up
  Left Chest:  X=forward, Y=down, Z=left  (World rotated +90 deg around X axis)
  Right Chest: X=forward, Y=up, Z=right  (World rotated -90 deg around X axis)

  Vector transform:
    Left:  world [x, y, z] -> chest [x, -z, y]
    Right: world [x, y, z] -> chest [x, z, -y]

=============================================================================
Function Index
=============================================================================

  Position transforms:
    transform_world_to_chest()   - World->Chest vector transform
    transform_chest_to_world()   - Chest->World vector transform

  Rotation transforms:
    get_world_to_chest_rotation()  - World->Chest rotation matrix
    get_chest_to_world_rotation()  - Chest->World rotation matrix
    apply_world_rotation_to_chest_pose()  - Rotate in World, return Chest orientation
    transform_pico_rotation_to_world()    - PICO->World rotation transform (axis-angle method)

  TF publishing:
    get_tf_quaternion()  - Get quaternion for TF publishing

  Direction queries:
    get_direction_vector_world()  - Get motion direction vector
    get_rotation_axis_world()     - Get rotation axis vector

  Config queries:
    get_pico_to_robot()  - Get PICO->Robot 3x3 transform matrix

  Arm angle control:
    elbow_direction_from_angles()  - Generate zsp_para direction vector from angles

=============================================================================
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from .config_loader import get_config as _get_config


# =============================================================================
# Internal helper: lazy-load config
# =============================================================================

def _config():
    """Lazy-load config singleton"""
    return _get_config()


# =============================================================================
# Position transforms
# =============================================================================

def transform_world_to_chest(vector_world, side):
    """Transform a vector from world coordinate frame to chest (world_left/world_right) coordinate frame

    Args:
        vector_world: Vector in world coordinate frame (position/direction)
        side: 'left' or 'right'

    Returns:
        np.array: Vector in chest coordinate frame

    Note:
        Uses efficient axis mapping (equivalent to rotation matrix transform)

        Coordinate frame definitions (from tianji_robot.yaml):
          World: X=forward, Y=left, Z=up
          Left Chest:  X=forward, Y=down, Z=left (rotated +90 deg around World X axis)
          Right Chest: X=forward, Y=up, Z=right (rotated -90 deg around World X axis)

        Vector transform mapping (derived via R_world_to_chest = R_chest_to_world^T):
          Left:  world [x, y, z] -> chest [x, -z, y]
          Right: world [x, y, z] -> chest [x, z, -y]
    """
    x, y, z = vector_world[0], vector_world[1], vector_world[2]

    if side == 'left':
        return np.array([x, -z, y])
    else:
        return np.array([x, z, -y])


def transform_chest_to_world(vector_chest, side):
    """Transform a vector from chest coordinate frame to world coordinate frame (inverse transform)

    Args:
        vector_chest: Vector in chest coordinate frame
        side: 'left' or 'right'

    Returns:
        np.array: Vector in world coordinate frame
    """
    x, y, z = vector_chest[0], vector_chest[1], vector_chest[2]

    if side == 'left':
        # Inverse mapping: [x, z, -y]
        return np.array([x, z, -y])
    else:
        # Inverse mapping: [x, -z, y]
        return np.array([x, -z, y])


# =============================================================================
# Direction queries
# =============================================================================

def get_direction_vector_world(mode):
    """Get motion direction vector in world coordinate frame

    Args:
        mode: Motion mode string

    Returns:
        np.array: Unit direction vector in world coordinate frame
    """
    directions = {
        'forward': np.array([1.0, 0.0, 0.0]),   # world +X
        'back': np.array([-1.0, 0.0, 0.0]),     # world -X
        'left': np.array([0.0, 1.0, 0.0]),      # world +Y
        'right': np.array([0.0, -1.0, 0.0]),    # world -Y
        'up': np.array([0.0, 0.0, 1.0]),        # world +Z
        'down': np.array([0.0, 0.0, -1.0]),     # world -Z
    }
    return directions.get(mode, np.array([0.0, 0.0, 0.0]))


def get_rotation_axis_world(mode):
    """Get rotation axis vector in world coordinate frame

    Args:
        mode: Rotation mode string

    Returns:
        np.array: Unit rotation axis vector in world coordinate frame
    """
    axes = {
        'rotate_x': np.array([1.0, 0.0, 0.0]),  # X axis (forward/back)
        'rotate_y': np.array([0.0, 1.0, 0.0]),  # Y axis (left/right)
        'rotate_z': np.array([0.0, 0.0, 1.0]),  # Z axis (up/down)
    }
    return axes.get(mode, np.array([0.0, 0.0, 0.0]))


# =============================================================================
# Rotation matrices
# =============================================================================

def get_world_to_chest_rotation(side):
    """Get world -> chest rotation matrix

    Used to transform vectors/orientations from world coordinate frame to chest coordinate frame:
      v_chest = R_world_to_chest @ v_world
      R_chest = R_world_to_chest @ R_world @ R_world_to_chest.T

    Args:
        side: 'left' or 'right'

    Returns:
        np.ndarray: 3x3 rotation matrix

    Note:
        The quaternion in the config file directly represents the world->chest transform.
        Left: +90 deg around X, Right: -90 deg around X
    """
    return _config().get_world_to_chest_rotation(side)


def get_chest_to_world_rotation(side):
    """Get chest -> world rotation matrix

    Used to transform vectors/orientations from chest coordinate frame to world coordinate frame:
      v_world = R_chest_to_world @ v_chest
      R_world = R_chest_to_world @ R_chest @ R_chest_to_world.T

    Args:
        side: 'left' or 'right'

    Returns:
        np.ndarray: 3x3 rotation matrix (transpose/inverse of world_to_chest)

    Note:
        This is the inverse transform of get_world_to_chest_rotation.
        Left: -90 deg around X, Right: +90 deg around X
    """
    return get_world_to_chest_rotation(side).T


def get_tf_quaternion(side):
    """Get quaternion for TF publishing (chest->world direction)

    TF transforms describe a child frame's orientation in the parent frame.
    For world -> world_left/world_right, we need the chest->world rotation,
    which is the inverse of world_to_chest.

    Args:
        side: 'left' or 'right'

    Returns:
        np.ndarray: Quaternion [qx, qy, qz, qw]
    """
    quat = _config().world_to_chest_quat[side]

    # Return conjugate quaternion (i.e. inverse rotation)
    # For unit quaternions, inverse = conjugate = [-qx, -qy, -qz, qw]
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]])


# =============================================================================
# Config queries
# =============================================================================

def get_pico_to_robot():
    """Get PICO->Robot 3x3 transform matrix (loaded from config)

    Returns:
        np.ndarray: 3x3 matrix (det=+1, two axis negations cancel out)

    Default:
        [[0, 0, -1],    # Robot X = -PICO Z (forward)
         [-1, 0, 0],    # Robot Y = -PICO X (left)
         [0, 1, 0]]     # Robot Z = +PICO Y (up)

    Physical meaning:
        When the user faces the robot's forward direction:
          Reach forward -> PICO -Z -> Robot +X (forward)
          Reach right   -> PICO +X -> Robot -Y (right)
          Reach up      -> PICO +Y -> Robot +Z (up)
    """
    return _config().pico_to_robot


# =============================================================================
# Orientation rotation transforms
# =============================================================================

def apply_world_rotation_to_chest_pose(base_rot_chest, R_delta_world, side):
    """Apply rotation delta in World coordinate frame, return target orientation in Chest coordinate frame

    This is the core orientation transform algorithm in teleoperation (4-step method).
    Ensures rotation is performed in World coordinate frame (physically intuitive),
    then converted back to Chest coordinate frame (required by IK).

    Algorithm steps:
      1. Chest->World: Convert base orientation to World coordinate frame
      2. Left-multiply delta: Left-multiply rotation delta in World (extrinsic = rotate around world axes)
      3. World->Chest: Convert target orientation back to Chest coordinate frame

    Mathematical formula:
      target_chest = R_w2c @ R_delta @ R_c2w @ base_chest

    Positive rotation directions (right-hand rule):
      Around +X: Roll right (top tilts right)
      Around +Y: Pitch down (fingers press down)
      Around +Z: Yaw left (fingers point left)

    Args:
        base_rot_chest: Base orientation (Chest coordinate frame, 3x3 rotation matrix)
        R_delta_world: Delta rotation (World coordinate frame, scipy Rotation object)
        side: 'left' or 'right'

    Returns:
        np.ndarray: Target orientation (Chest coordinate frame, 3x3 rotation matrix)

    Example:
        >>> from scipy.spatial.transform import Rotation as R
        >>> # Rotate 30 deg around World +Y axis (fingers press down 30 deg)
        >>> R_delta = R.from_rotvec([0, np.radians(30), 0])
        >>> target = apply_world_rotation_to_chest_pose(init_rot, R_delta, 'left')
    """
    R_chest_to_world = get_chest_to_world_rotation(side)
    R_world_to_chest = get_world_to_chest_rotation(side)

    base_rot_in_world = R_chest_to_world @ base_rot_chest
    target_rot_in_world = R_delta_world.as_matrix() @ base_rot_in_world
    target_rot_in_chest = R_world_to_chest @ target_rot_in_world

    return target_rot_in_chest


def transform_pico_rotation_to_world(delta_rot_pico, pico_to_robot):
    """Transform rotation delta from PICO coordinate frame to Robot World coordinate frame (axis-angle method)

    pico_to_robot matrix is an orthogonal axis-mapping matrix (determinant = +1, two axis negations cancel out).
    Uses axis-angle method to transform rotation: only the rotation axis direction is transformed, angle is preserved.

    PICO->Robot rotation effects:
      PICO around +X -> Robot around -Y -> Pitch up (fingers tilt up)
      PICO around +Y -> Robot around +Z -> Yaw left (fingers point left)
      PICO around +Z -> Robot around -X -> Roll left (wrist rolls left)

    Args:
        delta_rot_pico: Rotation delta in PICO coordinate frame (scipy Rotation object)
        pico_to_robot: 3x3 transform matrix (det=+1, loaded from tianji_robot.yaml)

    Returns:
        scipy.spatial.transform.Rotation: Rotation delta in World coordinate frame
    """
    rotvec = delta_rot_pico.as_rotvec()
    angle = np.linalg.norm(rotvec)

    if angle < 1e-10:
        return R.identity()

    axis_pico = rotvec / angle
    axis_world = pico_to_robot @ axis_pico

    return R.from_rotvec(axis_world * angle)


# =============================================================================
# Arm angle control
# =============================================================================

def elbow_direction_from_angles(pitch_deg, yaw_deg, side):
    """Generate IK zsp_para direction vector from pitch and yaw angles (Chest coordinate frame)

    Used to generate zsp_para for different arm angle postures. The result follows
    IK solver conventions: Y component direction is **opposite** to the gravity
    direction in the Chest coordinate frame.

    Important: zsp_para is NOT the geometric elbow direction!
      - Geometric elbow direction Y points toward gravity (physical elbow position direction)
      - zsp_para Y points against gravity (IK reference plane normal convention)
      - See diagnose_zsp_para.py for FK->IK closed-loop verification

    Angle definitions:
      pitch: 0=no forward/backward offset, positive=elbow forward, negative=elbow backward
      yaw:   0=pure anti-gravity, 45=default elbow-down, 90=pure outward

    Default elbow-down (pitch=0, yaw=45):
      Left arm:  [0, -0.707, -0.707]  (Y-=anti-gravity direction, Z-=outward)
      Right arm: [0, +0.707, -0.707]  (Y+=anti-gravity direction, Z-=outward)

    Args:
        pitch_deg: Pitch angle (degrees)
        yaw_deg: Yaw/abduction angle (degrees)
        side: 'left' or 'right'

    Returns:
        np.ndarray: Normalized 3D direction vector [x, y, z] (Chest coordinate frame)

    Example:
        >>> # Default elbow-down
        >>> d = elbow_direction_from_angles(0, 45, 'left')
        >>> # d ~ [0, -0.707, -0.707]
        >>> zsp_para = [d[0], d[1], d[2], 0, 0, 0]
    """
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    # X component: forward/backward offset
    x = np.sin(pitch)

    # Anti-gravity component (adjusted by pitch)
    anti_gravity = np.cos(pitch) * np.cos(yaw)
    # Outward component (adjusted by pitch)
    outward = np.cos(pitch) * np.sin(yaw)

    if side == 'left':
        # Left Chest: Y-=anti-gravity direction, -Z=outward (rightward)
        y = -anti_gravity
        z = -outward
    else:
        # Right Chest: Y+=anti-gravity direction, -Z=outward (leftward)
        y = anti_gravity
        z = -outward

    direction = np.array([x, y, z])

    norm = np.linalg.norm(direction)
    if norm > 1e-6:
        direction = direction / norm

    return direction
