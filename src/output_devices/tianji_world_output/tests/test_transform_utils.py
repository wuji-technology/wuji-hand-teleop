"""Unit tests for transform_utils.py — coordinate transformations"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from tianji_world_output.transform_utils import (
    transform_world_to_chest,
    transform_chest_to_world,
    get_world_to_chest_rotation,
    get_chest_to_world_rotation,
    apply_world_rotation_to_chest_pose,
    transform_pico_rotation_to_world,
    get_tf_quaternion,
    get_pico_to_robot,
    get_direction_vector_world,
    get_rotation_axis_world,
    elbow_direction_from_angles,
)


# =============================================================================
# Position transforms: transform_world_to_chest / transform_chest_to_world
# =============================================================================

class TestWorldToChest:
    """World→Chest position transform"""

    def test_left_forward(self):
        """World +X (forward) → Left Chest +X"""
        result = transform_world_to_chest(np.array([1, 0, 0]), 'left')
        np.testing.assert_array_almost_equal(result, [1, 0, 0])

    def test_left_up(self):
        """World +Z (up) → Left Chest -Y (down in left chest = up in world)"""
        result = transform_world_to_chest(np.array([0, 0, 1]), 'left')
        np.testing.assert_array_almost_equal(result, [0, -1, 0])

    def test_left_leftward(self):
        """World +Y (left) → Left Chest +Z"""
        result = transform_world_to_chest(np.array([0, 1, 0]), 'left')
        np.testing.assert_array_almost_equal(result, [0, 0, 1])

    def test_right_forward(self):
        """World +X (forward) → Right Chest +X"""
        result = transform_world_to_chest(np.array([1, 0, 0]), 'right')
        np.testing.assert_array_almost_equal(result, [1, 0, 0])

    def test_right_up(self):
        """World +Z (up) → Right Chest +Y"""
        result = transform_world_to_chest(np.array([0, 0, 1]), 'right')
        np.testing.assert_array_almost_equal(result, [0, 1, 0])

    def test_right_leftward(self):
        """World +Y (left) → Right Chest -Z"""
        result = transform_world_to_chest(np.array([0, 1, 0]), 'right')
        np.testing.assert_array_almost_equal(result, [0, 0, -1])


class TestChestToWorld:
    """Chest→World position transform (inverse)"""

    def test_roundtrip_left(self):
        """World→Chest→World roundtrip should be identity (left)"""
        v = np.array([0.3, -0.5, 0.7])
        result = transform_chest_to_world(transform_world_to_chest(v, 'left'), 'left')
        np.testing.assert_array_almost_equal(result, v)

    def test_roundtrip_right(self):
        """World→Chest→World roundtrip should be identity (right)"""
        v = np.array([0.3, -0.5, 0.7])
        result = transform_chest_to_world(transform_world_to_chest(v, 'right'), 'right')
        np.testing.assert_array_almost_equal(result, v)

    def test_roundtrip_reverse_left(self):
        """Chest→World→Chest roundtrip should be identity (left)"""
        v = np.array([0.1, 0.2, 0.3])
        result = transform_world_to_chest(transform_chest_to_world(v, 'left'), 'left')
        np.testing.assert_array_almost_equal(result, v)

    def test_roundtrip_reverse_right(self):
        """Chest→World→Chest roundtrip should be identity (right)"""
        v = np.array([0.1, 0.2, 0.3])
        result = transform_world_to_chest(transform_chest_to_world(v, 'right'), 'right')
        np.testing.assert_array_almost_equal(result, v)


# =============================================================================
# Rotation matrices
# =============================================================================

class TestRotationMatrices:
    """World↔Chest rotation matrix tests"""

    def test_left_is_orthogonal(self):
        """Left rotation matrix should be orthogonal (det=+1)"""
        R_mat = get_world_to_chest_rotation('left')
        np.testing.assert_almost_equal(np.linalg.det(R_mat), 1.0)
        np.testing.assert_array_almost_equal(R_mat @ R_mat.T, np.eye(3))

    def test_right_is_orthogonal(self):
        """Right rotation matrix should be orthogonal (det=+1)"""
        R_mat = get_world_to_chest_rotation('right')
        np.testing.assert_almost_equal(np.linalg.det(R_mat), 1.0)
        np.testing.assert_array_almost_equal(R_mat @ R_mat.T, np.eye(3))

    def test_inverse_is_transpose(self):
        """Chest→World should be the transpose of World→Chest"""
        for side in ['left', 'right']:
            R_w2c = get_world_to_chest_rotation(side)
            R_c2w = get_chest_to_world_rotation(side)
            np.testing.assert_array_almost_equal(R_c2w, R_w2c.T)

    def test_left_matches_axis_mapping(self):
        """Left rotation matrix should match documented axis mapping"""
        R_w2c = get_world_to_chest_rotation('left')
        # World +Z → Chest -Y
        np.testing.assert_array_almost_equal(R_w2c @ [0, 0, 1], [0, -1, 0], decimal=5)
        # World +Y → Chest +Z
        np.testing.assert_array_almost_equal(R_w2c @ [0, 1, 0], [0, 0, 1], decimal=5)

    def test_right_matches_axis_mapping(self):
        """Right rotation matrix should match documented axis mapping"""
        R_w2c = get_world_to_chest_rotation('right')
        # World +Z → Chest +Y
        np.testing.assert_array_almost_equal(R_w2c @ [0, 0, 1], [0, 1, 0], decimal=5)
        # World +Y → Chest -Z
        np.testing.assert_array_almost_equal(R_w2c @ [0, 1, 0], [0, 0, -1], decimal=5)


# =============================================================================
# TF quaternion
# =============================================================================

class TestTfQuaternion:
    """TF publish quaternion (chest→world conjugate)"""

    def test_left_is_conjugate(self):
        """Left TF quaternion should be conjugate of world→chest"""
        from tianji_world_output.config_loader import get_config
        cfg = get_config()
        w2c = cfg.world_to_chest_quat['left']
        tf_q = get_tf_quaternion('left')
        # Conjugate: [-qx, -qy, -qz, qw]
        np.testing.assert_array_almost_equal(tf_q, [-w2c[0], -w2c[1], -w2c[2], w2c[3]])

    def test_right_is_conjugate(self):
        """Right TF quaternion should be conjugate of world→chest"""
        from tianji_world_output.config_loader import get_config
        cfg = get_config()
        w2c = cfg.world_to_chest_quat['right']
        tf_q = get_tf_quaternion('right')
        np.testing.assert_array_almost_equal(tf_q, [-w2c[0], -w2c[1], -w2c[2], w2c[3]])

    def test_quaternion_is_unit(self):
        """TF quaternion should be unit quaternion"""
        for side in ['left', 'right']:
            q = get_tf_quaternion(side)
            np.testing.assert_almost_equal(np.linalg.norm(q), 1.0)


# =============================================================================
# apply_world_rotation_to_chest_pose (4-step algorithm)
# =============================================================================

class TestApplyWorldRotation:
    """4-step rotation algorithm: target_chest = R_w2c @ R_delta @ R_c2w @ base_chest"""

    def test_identity_rotation(self):
        """Identity rotation should return the base pose unchanged"""
        base = np.eye(3)
        R_delta = R.identity()
        for side in ['left', 'right']:
            result = apply_world_rotation_to_chest_pose(base, R_delta, side)
            np.testing.assert_array_almost_equal(result, base)

    def test_result_is_valid_rotation(self):
        """Result should be a valid rotation matrix (orthogonal, det=+1)"""
        from tianji_world_output.config_loader import get_config
        cfg = get_config()
        base = cfg.init_rot['left']
        R_delta = R.from_rotvec([0, np.radians(30), 0])
        result = apply_world_rotation_to_chest_pose(base, R_delta, 'left')
        np.testing.assert_almost_equal(np.linalg.det(result), 1.0, decimal=3)
        np.testing.assert_array_almost_equal(result @ result.T, np.eye(3), decimal=3)


# =============================================================================
# transform_pico_rotation_to_world (axis-angle method)
# =============================================================================

class TestPicoRotationToWorld:
    """PICO→World rotation via axis-angle transform"""

    def test_identity_returns_identity(self):
        """Identity rotation should return identity"""
        pico_to_robot = get_pico_to_robot()
        result = transform_pico_rotation_to_world(R.identity(), pico_to_robot)
        np.testing.assert_array_almost_equal(result.as_rotvec(), [0, 0, 0], decimal=5)

    def test_pico_x_to_robot_neg_y(self):
        """PICO +X rotation → Robot -Y rotation (pitch up)"""
        pico_to_robot = get_pico_to_robot()
        angle = np.radians(30)
        delta_pico = R.from_rotvec([angle, 0, 0])
        result = transform_pico_rotation_to_world(delta_pico, pico_to_robot)
        rotvec = result.as_rotvec()
        # Should be rotation around Robot -Y axis
        axis = rotvec / np.linalg.norm(rotvec)
        np.testing.assert_array_almost_equal(axis, [0, -1, 0], decimal=5)
        np.testing.assert_almost_equal(np.linalg.norm(rotvec), angle, decimal=5)

    def test_pico_y_to_robot_pos_z(self):
        """PICO +Y rotation → Robot +Z rotation (yaw left)"""
        pico_to_robot = get_pico_to_robot()
        angle = np.radians(45)
        delta_pico = R.from_rotvec([0, angle, 0])
        result = transform_pico_rotation_to_world(delta_pico, pico_to_robot)
        rotvec = result.as_rotvec()
        axis = rotvec / np.linalg.norm(rotvec)
        np.testing.assert_array_almost_equal(axis, [0, 0, 1], decimal=5)
        np.testing.assert_almost_equal(np.linalg.norm(rotvec), angle, decimal=5)

    def test_preserves_angle(self):
        """Rotation angle should be preserved after axis transform"""
        pico_to_robot = get_pico_to_robot()
        angle = np.radians(60)
        delta_pico = R.from_rotvec([0, 0, angle])
        result = transform_pico_rotation_to_world(delta_pico, pico_to_robot)
        np.testing.assert_almost_equal(np.linalg.norm(result.as_rotvec()), angle, decimal=5)


# =============================================================================
# elbow_direction_from_angles
# =============================================================================

class TestElbowDirection:
    """Elbow direction vector generation for IK zsp_para"""

    def test_default_left_sinking_elbow(self):
        """Default sinking elbow for left arm: Y < 0 (anti-gravity), Z < 0 (outward)"""
        d = elbow_direction_from_angles(0, 45, 'left')
        assert d[0] == pytest.approx(0, abs=1e-6)
        assert d[1] < 0  # anti-gravity direction
        assert d[2] < 0  # outward

    def test_default_right_sinking_elbow(self):
        """Default sinking elbow for right arm: Y > 0 (anti-gravity), Z < 0 (outward)"""
        d = elbow_direction_from_angles(0, 45, 'right')
        assert d[0] == pytest.approx(0, abs=1e-6)
        assert d[1] > 0  # anti-gravity direction
        assert d[2] < 0  # outward

    def test_is_unit_vector(self):
        """Output should be a unit vector"""
        for side in ['left', 'right']:
            for pitch in [0, 15, -15]:
                for yaw in [0, 45, 90]:
                    d = elbow_direction_from_angles(pitch, yaw, side)
                    np.testing.assert_almost_equal(np.linalg.norm(d), 1.0)

    def test_left_right_y_symmetry(self):
        """Left and right should have opposite Y signs for same angles"""
        d_left = elbow_direction_from_angles(0, 45, 'left')
        d_right = elbow_direction_from_angles(0, 45, 'right')
        np.testing.assert_almost_equal(d_left[1], -d_right[1])
        np.testing.assert_almost_equal(d_left[2], d_right[2])


# =============================================================================
# Direction / axis queries
# =============================================================================

class TestDirectionQueries:
    """Direction vector and rotation axis queries"""

    def test_forward_is_x(self):
        np.testing.assert_array_equal(get_direction_vector_world('forward'), [1, 0, 0])

    def test_up_is_z(self):
        np.testing.assert_array_equal(get_direction_vector_world('up'), [0, 0, 1])

    def test_unknown_returns_zero(self):
        np.testing.assert_array_equal(get_direction_vector_world('unknown'), [0, 0, 0])

    def test_rotate_x(self):
        np.testing.assert_array_equal(get_rotation_axis_world('rotate_x'), [1, 0, 0])


# =============================================================================
# PICO→Robot matrix properties
# =============================================================================

class TestPicoToRobot:
    """PICO→Robot transform matrix properties"""

    def test_is_orthogonal(self):
        """pico_to_robot should be orthogonal"""
        M = get_pico_to_robot()
        np.testing.assert_array_almost_equal(M @ M.T, np.eye(3))

    def test_det_is_plus_one(self):
        """pico_to_robot det should be +1 (proper rotation, no mirror)"""
        M = get_pico_to_robot()
        np.testing.assert_almost_equal(np.linalg.det(M), 1.0)

    def test_pico_neg_z_to_robot_pos_x(self):
        """PICO -Z (forward) → Robot +X (forward)"""
        M = get_pico_to_robot()
        result = M @ np.array([0, 0, -1])
        np.testing.assert_array_almost_equal(result, [1, 0, 0])

    def test_pico_pos_y_to_robot_pos_z(self):
        """PICO +Y (up) → Robot +Z (up)"""
        M = get_pico_to_robot()
        result = M @ np.array([0, 1, 0])
        np.testing.assert_array_almost_equal(result, [0, 0, 1])
