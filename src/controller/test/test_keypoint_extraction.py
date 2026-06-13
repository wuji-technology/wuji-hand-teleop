"""Unit tests for Wuji Glove skeleton → MediaPipe (21,3) extraction."""
from types import SimpleNamespace

import numpy as np

from controller.wujihand_node import _extract_wuji_glove_keypoints


def _make_skeleton(positions, frame_id="r_wrist"):
    """Build a minimal skeleton stub: header + joints[*].pose.position."""
    joints = [
        SimpleNamespace(pose=SimpleNamespace(position=list(p)))
        for p in positions
    ]
    header = SimpleNamespace(frame_id=frame_id)
    return SimpleNamespace(header=header, joints=joints)


def test_extracts_21_keypoints_with_correct_shape():
    positions = [(i * 0.01, i * 0.02, i * 0.03) for i in range(21)]
    skel = _make_skeleton(positions)
    kp = _extract_wuji_glove_keypoints(skel)
    assert kp.shape == (21, 3)
    assert kp.dtype == np.float32
    np.testing.assert_allclose(kp[0], [0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(kp[20], [0.20, 0.40, 0.60], atol=1e-6)


def test_returns_none_on_wrong_joint_count():
    positions = [(0.0, 0.0, 0.0)] * 15  # too few
    skel = _make_skeleton(positions)
    assert _extract_wuji_glove_keypoints(skel) is None


def test_returns_none_on_too_many_joints():
    positions = [(0.0, 0.0, 0.0)] * 25
    skel = _make_skeleton(positions)
    assert _extract_wuji_glove_keypoints(skel) is None
