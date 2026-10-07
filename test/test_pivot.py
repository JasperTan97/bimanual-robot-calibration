"""Unit tests for cone_calib.pivot (pivot calibration + observability guards)."""

from cone_calib.pivot import Capture, solve_pivot, TipOffsetUnobservable
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def _make_captures(rng, tip_offset, centres, n_per_seat,
                   tilt_deg=30.0, pos_noise=1e-4, rot_noise_deg=0.02):
    """Synthesise captures so that R_i @ tip_offset + p_i = c_k (+ noise)."""
    tip_offset = np.asarray(tip_offset, dtype=float)
    captures = []
    index = 0
    for seat_id, centre in centres.items():
        centre = np.asarray(centre, dtype=float)
        for _ in range(n_per_seat):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = rng.uniform(0.0, np.radians(tilt_deg))
            rot = Rotation.from_rotvec(axis * angle)
            rot = rot * Rotation.from_rotvec(rng.normal(scale=np.radians(rot_noise_deg),
                                                        size=3))
            rot_mat = rot.as_matrix()
            position = centre - rot_mat @ tip_offset + rng.normal(scale=pos_noise, size=3)
            captures.append(Capture(seat_id, rot_mat, position, index))
            index += 1
    return captures


def test_synthetic_round_trip_estimate_tip():
    rng = np.random.default_rng(10)
    tip_offset = np.array([0.003, -0.002, 0.107])
    centres = {'p1_s1': [0.10, 0.20, 0.30], 'p1_s2': [0.25, 0.20, 0.30],
               'p1_s3': [0.10, 0.35, 0.31]}
    captures = _make_captures(rng, tip_offset, centres, n_per_seat=12)
    result = solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                         estimate_tip_offset=True)
    assert np.linalg.norm(result.tip_offset - tip_offset) < 1.5e-3
    for seat_id, centre in centres.items():
        assert np.linalg.norm(result.seat_centres[seat_id] - np.asarray(centre)) < 1.5e-3
    assert result.rms_overall < 1e-3
    assert result.condition_number < 1.0e4
    assert result.flagged_outliers == []
    # per-divot RMS (mean-squared-error check per seat) is reported for every seat
    assert set(result.per_seat_rms) == set(centres)
    assert all(v < 1e-3 for v in result.per_seat_rms.values())


def test_fixed_tip_matches_when_exact():
    rng = np.random.default_rng(11)
    tip_offset = np.array([0.0, 0.0, 0.1])
    centres = {'p1_s1': [0.1, 0.2, 0.3], 'p1_s2': [0.3, 0.2, 0.3], 'p1_s3': [0.1, 0.4, 0.3]}
    captures = _make_captures(rng, tip_offset, centres, n_per_seat=10, pos_noise=0.0,
                              rot_noise_deg=0.0)
    result = solve_pivot('ur3e', captures, tip_offset_param=tip_offset,
                         estimate_tip_offset=False)
    assert np.allclose(result.tip_offset, tip_offset)
    for seat_id, centre in centres.items():
        assert np.linalg.norm(result.seat_centres[seat_id] - np.asarray(centre)) < 1e-9
    assert result.rms_overall < 1e-9


def test_degenerate_rotation_about_tool_axis_refuses():
    # All rotations about +z (the tool axis) leave the z-offset unobservable.
    rng = np.random.default_rng(12)
    tip_offset = np.array([0.01, 0.0, 0.1])
    centres = {'p1_s1': [0.1, 0.2, 0.3], 'p1_s2': [0.3, 0.2, 0.3], 'p1_s3': [0.1, 0.4, 0.3]}
    captures = []
    index = 0
    for seat_id, centre in centres.items():
        centre = np.asarray(centre, dtype=float)
        for _ in range(10):
            theta = rng.uniform(0, 2 * np.pi)
            rot_mat = Rotation.from_rotvec([0, 0, theta]).as_matrix()
            position = centre - rot_mat @ tip_offset
            captures.append(Capture(seat_id, rot_mat, position, index))
            index += 1
    with pytest.raises(TipOffsetUnobservable):
        solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                    estimate_tip_offset=True)


def test_low_tilt_spread_warns():
    rng = np.random.default_rng(13)
    tip_offset = np.array([0.0, 0.0, 0.1])
    centres = {'p1_s1': [0.1, 0.2, 0.3], 'p1_s2': [0.3, 0.2, 0.3], 'p1_s3': [0.1, 0.4, 0.3]}
    captures = _make_captures(rng, tip_offset, centres, n_per_seat=10, tilt_deg=4.0)
    # Small tilt is still (barely) observable here; expect a warning, not a refusal.
    try:
        result = solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                             estimate_tip_offset=True, max_condition_number=1e9)
    except TipOffsetUnobservable:
        return  # refusing on ill-conditioning is also acceptable
    assert any('tilt spread' in w for w in result.warnings)


def test_outlier_is_flagged_and_optionally_rejected():
    rng = np.random.default_rng(14)
    tip_offset = np.array([0.002, 0.001, 0.1])
    centres = {'p1_s1': [0.1, 0.2, 0.3], 'p1_s2': [0.3, 0.2, 0.3], 'p1_s3': [0.1, 0.4, 0.3]}
    captures = _make_captures(rng, tip_offset, centres, n_per_seat=12)
    # Corrupt one capture's position by 2 cm.
    captures[5].position = captures[5].position + np.array([0.02, 0.0, 0.0])
    flagged = solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                          estimate_tip_offset=True)
    assert captures[5].index in flagged.flagged_outliers
    rejected = solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                           estimate_tip_offset=True, reject_outliers=True)
    # With the outlier removed the tip offset is recovered accurately.
    assert np.linalg.norm(rejected.tip_offset - tip_offset) < 1.5e-3


def test_reject_outliers_drops_fully_bad_seat_without_crashing():
    # One seat probed badly (all its captures scattered) must not crash the
    # re-solve with a KeyError; it should be dropped with a warning.
    rng = np.random.default_rng(15)
    tip_offset = np.array([0.0, 0.0, 0.1])
    good = {'p1_s1': [0.1, 0.2, 0.3], 'p1_s2': [0.3, 0.2, 0.3], 'p1_s3': [0.1, 0.4, 0.3]}
    captures = _make_captures(rng, tip_offset, good, n_per_seat=10)
    index = len(captures)
    bad_centre = np.array([0.2, 0.3, 0.35])
    for _ in range(4):
        rot = Rotation.from_rotvec(rng.normal(scale=0.3, size=3)).as_matrix()
        pos = bad_centre - rot @ tip_offset + rng.normal(scale=0.15, size=3)
        captures.append(Capture('p1_s4', rot, pos, index))
        index += 1
    result = solve_pivot('fr3', captures, tip_offset_param=[0, 0, 0.1],
                         estimate_tip_offset=True, reject_outliers=True)
    assert 'p1_s4' not in result.seat_centres
    assert any('dropped' in w for w in result.warnings)
    for seat_id, centre in good.items():
        assert np.linalg.norm(result.seat_centres[seat_id] - np.asarray(centre)) < 3e-3
