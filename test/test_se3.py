"""Unit tests for cone_calib.se3 (SE(3) helpers + Kabsch/Umeyama)."""

from cone_calib import se3
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def test_quat_matrix_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        quat = Rotation.random(random_state=rng).as_quat()
        mat = se3.quat_to_matrix(quat)
        back = se3.matrix_to_quat(mat)
        # quaternion double cover: compare rotations, not raw components
        assert np.allclose(se3.quat_to_matrix(back), mat, atol=1e-12)


def test_invert_transform_round_trip():
    rng = np.random.default_rng(1)
    transform = se3.pose_to_matrix(rng.normal(size=3), Rotation.random(random_state=rng).as_quat())
    identity = se3.compose(transform, se3.invert_transform(transform))
    assert np.allclose(identity, np.eye(4), atol=1e-12)


def test_pose_matrix_round_trip():
    rng = np.random.default_rng(2)
    t = rng.normal(size=3)
    q = Rotation.random(random_state=rng).as_quat()
    t2, q2 = se3.matrix_to_pose(se3.pose_to_matrix(t, q))
    assert np.allclose(t2, t, atol=1e-12)
    assert np.allclose(se3.quat_to_matrix(q2), se3.quat_to_matrix(q), atol=1e-12)


def test_quaternion_mean_identical():
    quat = Rotation.from_euler('xyz', [10, 20, 30], degrees=True).as_quat()
    mean = se3.quaternion_mean(np.tile(quat, (5, 1)))
    assert np.allclose(se3.quat_to_matrix(mean), se3.quat_to_matrix(quat), atol=1e-10)


def test_quaternion_mean_small_spread():
    base = Rotation.from_euler('xyz', [5, -8, 12], degrees=True)
    rng = np.random.default_rng(3)
    perturbed = [(base * Rotation.from_rotvec(rng.normal(scale=1e-3, size=3))).as_quat()
                 for _ in range(50)]
    mean = se3.quaternion_mean(np.array(perturbed))
    angle, _ = se3.relative_rotation(base.as_matrix(), se3.quat_to_matrix(mean))
    assert angle < 1e-2


def test_kabsch_recovers_rigid_transform():
    rng = np.random.default_rng(4)
    rot = Rotation.random(random_state=rng).as_matrix()
    trans = rng.normal(size=3)
    src = rng.normal(size=(8, 3))
    dst = src @ rot.T + trans
    rec_rot, rec_t, info = se3.kabsch_umeyama(src, dst)
    assert np.allclose(rec_rot, rot, atol=1e-9)
    assert np.allclose(rec_t, trans, atol=1e-9)
    assert info['rms'] < 1e-9
    # proper rotation only (no reflection)
    assert np.isclose(np.linalg.det(rec_rot), 1.0, atol=1e-9)


def test_kabsch_handles_reflection_case():
    # Nearly-planar points plus a transform: result must still be a proper rotation.
    rng = np.random.default_rng(5)
    rot = Rotation.random(random_state=rng).as_matrix()
    src = rng.normal(size=(6, 3))
    src[:, 2] *= 1e-6
    dst = src @ rot.T + rng.normal(size=3)
    rec_rot, _, _ = se3.kabsch_umeyama(src, dst)
    assert np.isclose(np.linalg.det(rec_rot), 1.0, atol=1e-6)


def test_kabsch_raises_on_collinear():
    line = np.array([[0.0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    with pytest.raises(se3.CollinearPointsError):
        se3.kabsch_umeyama(line, line + np.array([1.0, 2.0, 3.0]))


def test_kabsch_raises_on_too_few_points():
    with pytest.raises(se3.CollinearPointsError):
        se3.kabsch_umeyama(np.zeros((2, 3)), np.ones((2, 3)))


def test_transform_diff():
    base = se3.pose_to_matrix([1.0, 2.0, 3.0],
                              Rotation.from_euler('z', 10, degrees=True).as_quat())
    rotated = se3.pose_to_matrix([1.0, 2.0, 3.003],
                                 Rotation.from_euler('z', 12, degrees=True).as_quat())
    linear, angular = se3.transform_diff(base, rotated)
    assert np.isclose(linear, 0.003, atol=1e-9)
    assert np.isclose(np.degrees(angular), 2.0, atol=1e-6)
