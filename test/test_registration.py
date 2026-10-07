"""Unit tests for cone_calib.registration (inter-robot + CAD sanity)."""

from cone_calib import registration, se3
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def _seats():
    return {'p1_s1': [0.0, 0.0, 0.0], 'p1_s2': [0.150, 0.0, 0.0],
            'p1_s3': [0.0, 0.150, 0.0], 'p2_s1': [0.08, 0.11, 0.02]}


def test_registration_recovers_transform():
    rng = np.random.default_rng(20)
    t_a_b = se3.pose_to_matrix(rng.normal(size=3), Rotation.random(random_state=rng).as_quat())
    centres_b = {s: np.asarray(c, dtype=float) for s, c in _seats().items()}
    centres_a = {s: (t_a_b @ np.append(c, 1.0))[:3] for s, c in centres_b.items()}
    result = registration.register_robots(centres_a, centres_b, 'ur3e', 'fr3')
    assert np.allclose(result.transform_a_b, t_a_b, atol=1e-9)
    assert result.rms < 1e-9
    assert set(result.shared_seats) == set(centres_b)


def test_registration_requires_three_shared_seats():
    centres_a = {'p1_s1': [0, 0, 0], 'p1_s2': [1, 0, 0]}
    centres_b = {'p1_s1': [0, 0, 0], 'p1_s2': [1, 0, 0]}
    with pytest.raises(se3.CollinearPointsError):
        registration.register_robots(centres_a, centres_b, 'ur3e', 'fr3')


def test_registration_raises_on_collinear_seats():
    collinear = {'a': [0, 0, 0], 'b': [1, 0, 0], 'c': [2, 0, 0], 'd': [3, 0, 0]}
    centres_b = {s: np.asarray(c, dtype=float) for s, c in collinear.items()}
    rot = Rotation.from_euler('z', 20, degrees=True).as_matrix()
    centres_a = {s: rot @ c + np.array([1.0, 2.0, 3.0]) for s, c in centres_b.items()}
    with pytest.raises(se3.CollinearPointsError):
        registration.register_robots(centres_a, centres_b, 'ur3e', 'fr3')


def test_inter_seat_distance_check():
    nominal = {s: np.asarray(c, dtype=float) for s, c in _seats().items()}
    # Place the robot centres as a rigid transform of the nominal: distances match.
    rot = Rotation.from_euler('xyz', [5, 10, 15], degrees=True).as_matrix()
    centres = {s: rot @ c + np.array([0.3, -0.2, 0.1]) for s, c in nominal.items()}
    check = registration.inter_seat_distance_check(centres, nominal)
    assert check['max_abs_deviation'] < 1e-9


def test_orthogonality_right_angle_is_zero_deviation():
    nominal = {s: np.asarray(c, float) for s, c in _seats().items()}
    rot = Rotation.from_euler('xyz', [10, 20, 30], degrees=True).as_matrix()
    centres = {s: rot @ c + np.array([0.3, -0.2, 0.1]) for s, c in nominal.items()}
    out = registration.orthogonality_check(centres, nominal,
                                           triple=['p1_s1', 'p1_s2', 'p1_s3'])
    assert abs(out['measured_angle_deg'] - 90.0) < 1e-6
    assert abs(out['deviation_from_90_deg']) < 1e-6


def test_orthogonality_detects_skew_and_auto_picks_triple():
    # Two vectors at 80 deg; deviation from 90 must read -10 deg.
    a = np.array([0.15, 0.0, 0.0])
    b = np.array([0.15 * np.cos(np.radians(80)), 0.15 * np.sin(np.radians(80)), 0.0])
    nominal = {'o': np.zeros(3), 'a': a, 'b': b}
    out = registration.orthogonality_check(nominal, nominal)  # auto-pick triple
    assert set(out['triple']) == {'o', 'a', 'b'}
    assert abs(out['measured_angle_deg'] - 80.0) < 1e-6
    assert abs(out['deviation_from_90_deg'] - (-10.0)) < 1e-6
    assert abs(out['deviation_from_nominal_deg']) < 1e-6


def test_two_route_discrepancy_small():
    rng = np.random.default_rng(21)
    nominal = {s: np.asarray(c, dtype=float) for s, c in _seats().items()}
    t_a_plate = se3.pose_to_matrix(rng.normal(size=3), Rotation.random(random_state=rng).as_quat())
    t_b_plate = se3.pose_to_matrix(rng.normal(size=3), Rotation.random(random_state=rng).as_quat())
    centres_a = {s: (t_a_plate @ np.append(c, 1.0))[:3] for s, c in nominal.items()}
    centres_b = {s: (t_b_plate @ np.append(c, 1.0))[:3] for s, c in nominal.items()}
    t_a_b = se3.compose(t_a_plate, se3.invert_transform(t_b_plate))
    out = registration.two_route_discrepancy(t_a_b, centres_a, centres_b, nominal)
    assert out is not None
    linear, angular, _ = out
    assert linear < 1e-9 and angular < 1e-9
