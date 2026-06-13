"""Unit tests for config_loader.py — YAML loading and TianjiConfig"""
import numpy as np
import pytest

from tianji_world_output.config_loader import TianjiConfig, get_config, reload_config


@pytest.fixture
def cfg():
    """Load config without ROS (uses file-relative path)"""
    return TianjiConfig.load()


# =============================================================================
# Loading / singleton
# =============================================================================

class TestLoading:
    """Config file loading and singleton behaviour"""

    def test_load_returns_config(self, cfg):
        assert isinstance(cfg, TianjiConfig)

    def test_get_config_singleton(self):
        """get_config() should return the same instance"""
        c1 = get_config()
        c2 = get_config()
        assert c1 is c2

    def test_reload_config_returns_new(self):
        """reload_config() should return a fresh instance"""
        c1 = get_config()
        c2 = reload_config()
        assert c1 is not c2

    def test_raw_is_dict(self, cfg):
        assert isinstance(cfg.raw, dict)
        assert len(cfg.raw) > 0

    def test_config_path_exists(self, cfg):
        assert cfg.config_path.exists()


# =============================================================================
# Scalar fields
# =============================================================================

class TestScalarFields:
    """Scalar config fields"""

    def test_robot_ip(self, cfg):
        assert isinstance(cfg.robot_ip, str)
        assert cfg.robot_ip  # non-empty

    def test_zsp_type(self, cfg):
        assert cfg.zsp_type == 1

    def test_zsp_angle(self, cfg):
        assert isinstance(cfg.zsp_angle, float)

    def test_dgr(self, cfg):
        assert len(cfg.dgr) == 3
        for v in cfg.dgr:
            assert v > 0


# =============================================================================
# Dict[str, ndarray] fields — left/right present, correct shapes
# =============================================================================

class TestDictFields:
    """Dict fields should have left/right keys with correct shapes"""

    def test_init_joints(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.init_joints[side]
            assert isinstance(arr, np.ndarray)
            assert arr.shape == (7,)

    def test_init_pos(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.init_pos[side]
            assert arr.shape == (3,)

    def test_init_rot(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.init_rot[side]
            assert arr.shape == (3, 3)

    def test_init_quat(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.init_quat[side]
            assert arr.shape == (4,)

    def test_world_to_chest_quat(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.world_to_chest_quat[side]
            assert arr.shape == (4,)

    def test_world_to_chest_trans(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.world_to_chest_trans[side]
            assert arr.shape == (3,)

    def test_arm_init_pos(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.arm_init_pos[side]
            assert arr.shape == (3,)

    def test_arm_init_quat(self, cfg):
        for side in ['left', 'right']:
            arr = cfg.arm_init_quat[side]
            assert arr.shape == (4,)


# =============================================================================
# Quaternion normalization
# =============================================================================

class TestQuaternions:
    """Quaternions should be unit quaternions"""

    def test_world_to_chest_quat_unit(self, cfg):
        for side in ['left', 'right']:
            np.testing.assert_almost_equal(
                np.linalg.norm(cfg.world_to_chest_quat[side]), 1.0
            )

    def test_init_quat_unit(self, cfg):
        for side in ['left', 'right']:
            np.testing.assert_almost_equal(
                np.linalg.norm(cfg.init_quat[side]), 1.0, decimal=3
            )

    def test_arm_init_quat_unit(self, cfg):
        for side in ['left', 'right']:
            np.testing.assert_almost_equal(
                np.linalg.norm(cfg.arm_init_quat[side]), 1.0, decimal=3
            )


# =============================================================================
# Rotation matrix validity
# =============================================================================

class TestRotationMatrices:
    """Rotation matrices should be orthogonal with det=+1"""

    def test_init_rot_orthogonal(self, cfg):
        for side in ['left', 'right']:
            R_mat = cfg.init_rot[side]
            np.testing.assert_almost_equal(np.linalg.det(R_mat), 1.0, decimal=3)
            np.testing.assert_array_almost_equal(R_mat @ R_mat.T, np.eye(3), decimal=3)

    def test_pico_to_robot_shape(self, cfg):
        assert cfg.pico_to_robot.shape == (3, 3)

    def test_pico_to_robot_orthogonal(self, cfg):
        M = cfg.pico_to_robot
        np.testing.assert_almost_equal(np.linalg.det(M), 1.0)
        np.testing.assert_array_almost_equal(M @ M.T, np.eye(3))


# =============================================================================
# Left/right symmetry checks
# =============================================================================

class TestSymmetry:
    """Left and right configs should be mirror-symmetric"""

    def test_init_pos_x_equal(self, cfg):
        """Left and right should have the same X position"""
        np.testing.assert_almost_equal(
            cfg.init_pos['left'][0], cfg.init_pos['right'][0]
        )

    def test_init_pos_y_opposite(self, cfg):
        """Left and right Y positions should be opposite"""
        np.testing.assert_almost_equal(
            cfg.init_pos['left'][1], -cfg.init_pos['right'][1]
        )

    def test_init_pos_z_equal(self, cfg):
        """Left and right should have the same Z position"""
        np.testing.assert_almost_equal(
            cfg.init_pos['left'][2], cfg.init_pos['right'][2]
        )


# =============================================================================
# Helper methods
# =============================================================================

class TestHelperMethods:
    """TianjiConfig helper methods"""

    def test_get_world_to_chest_rotation(self, cfg):
        for side in ['left', 'right']:
            R_mat = cfg.get_world_to_chest_rotation(side)
            assert R_mat.shape == (3, 3)
            np.testing.assert_almost_equal(np.linalg.det(R_mat), 1.0)

    def test_get_chest_to_world_rotation_is_inverse(self, cfg):
        for side in ['left', 'right']:
            R_w2c = cfg.get_world_to_chest_rotation(side)
            R_c2w = cfg.get_chest_to_world_rotation(side)
            np.testing.assert_array_almost_equal(R_c2w, R_w2c.T)

    def test_get_default_zsp_direction_is_unit(self, cfg):
        for side in ['left', 'right']:
            d = cfg.get_default_zsp_direction(side)
            np.testing.assert_almost_equal(np.linalg.norm(d), 1.0)

    def test_get_default_zsp_direction_left_right_y_sign(self, cfg):
        """Left zsp Y < 0, Right zsp Y > 0 (anti-gravity convention)"""
        d_left = cfg.get_default_zsp_direction('left')
        d_right = cfg.get_default_zsp_direction('right')
        assert d_left[1] < 0
        assert d_right[1] > 0
