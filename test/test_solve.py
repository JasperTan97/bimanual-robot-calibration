"""Integration tests for cone_calib.solve (cross-check direction, multi-robot)."""

from cone_calib import io, se3, solve
import numpy as np
from scipy.spatial.transform import Rotation

NOMINAL = {'p1_s1': [0.0, 0.0, 0.0], 'p1_s2': [0.150, 0.0, 0.0],
           'p1_s3': [0.0, 0.150, 0.0], 'p2_s1': [0.08, 0.11, 0.02]}


def _make_session(tmp_path, robot_id, base_frame, t_rob_plate, tip_offset, seed):
    rng = np.random.default_rng(seed)
    cfg = {'robot_id': robot_id, 'base_frame': base_frame, 'flange_frame': base_frame + '_tool',
           'joint_states_topic': '/js', 'joint_names': ['j1'], 'tip_offset': list(tip_offset),
           'estimate_tip_offset': True}
    session = io.new_session(robot_id, cfg, 'tf')
    session['base_frame'] = base_frame
    index = 0
    for seat_id, nom in NOMINAL.items():
        centre = (t_rob_plate @ np.append(np.asarray(nom, float), 1.0))[:3]
        for _ in range(8):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            rot = Rotation.from_rotvec(axis * rng.uniform(0, np.radians(30))).as_matrix()
            pos = centre - rot @ np.asarray(tip_offset) + rng.normal(scale=5e-5, size=3)
            session['captures'].append(io.make_capture_record(
                seat_id, index, 1.0, ['j1'], [0.0], [0.0], pos, se3.matrix_to_quat(rot), 'tf'))
            index += 1
    path = str(tmp_path / ('%s_session.yaml' % robot_id))
    io.save_session(path, session)
    return path


def _write_plate(tmp_path, reference):
    plate = {'seats_nominal': NOMINAL, 'min_captures_per_seat': 6,
             'min_tilt_spread_deg': 15.0, 'max_condition_number': 1.0e4,
             'reject_outliers': False, 'outlier_mad_k': 4.0, 'reference_robot': reference}
    path = str(tmp_path / 'plate.yaml')
    io.save_results(path, plate)
    return path


def _write_static_tfs(tmp_path, transform):
    translation, quat = se3.matrix_to_pose(transform)
    data = {'transforms': [{
        'name': 'world_to_base', 'parent': 'world', 'child': 'base',
        'translation': [float(v) for v in translation],
        'quaternion': [float(v) for v in quat]}]}
    path = str(tmp_path / 'static_tfs.yaml')
    io.save_results(path, data)
    return path


def test_cross_check_is_direction_agnostic(tmp_path):
    # Ground truth T_A_B with A = ur3e. world_to_base is stored as the pose of
    # fr3 base in ur3e base (alias frames world/base). The cross-check must pass
    # whether ur3e OR fr3 is chosen as the reference.
    t_a_b = se3.pose_to_matrix([0.30, -0.45, 0.02],
                               Rotation.from_euler('xyz', [1, -2, 40], degrees=True).as_quat())
    t_a_plate = se3.pose_to_matrix([0.1, 0.2, 0.05],
                                   Rotation.from_euler('xyz', [3, 4, 25], degrees=True).as_quat())
    t_b_plate = se3.compose(se3.invert_transform(t_a_b), t_a_plate)
    ur = _make_session(tmp_path, 'ur3e', 'ur3e_base_link', t_a_plate, [0.0, 0.0, 0.1], 1)
    fr = _make_session(tmp_path, 'fr3', 'fr3_link0', t_b_plate, [0.002, -0.001, 0.107], 2)
    static_tfs = _write_static_tfs(tmp_path, t_a_b)

    for reference in ('ur3e', 'fr3'):
        plate = _write_plate(tmp_path, reference)
        results, code = solve.solve([
            '--session', ur, '--session', fr, '--plate', plate,
            '--reference-robot', reference,
            '--cross-check-static-tfs', static_tfs, '--out', str(tmp_path / 'r.yaml')])
        assert code == 0
        cc = results['sanity']['cross_check']
        assert cc['within_thresholds'], (reference, cc['linear_m'], cc['angular_deg'])


def test_three_sessions_register_all_pairs(tmp_path):
    rng_t = se3.pose_to_matrix([0.2, 0.1, 0.0],
                               Rotation.from_euler('z', 10, degrees=True).as_quat())
    ur = _make_session(tmp_path, 'ur3e', 'ur3e_base_link', rng_t, [0.0, 0.0, 0.1], 3)
    fr = _make_session(tmp_path, 'fr3', 'fr3_link0',
                       se3.pose_to_matrix([0.4, -0.3, 0.0],
                                          Rotation.from_euler('z', -20, degrees=True).as_quat()),
                       [0.0, 0.0, 0.1], 4)
    u5 = _make_session(tmp_path, 'ur5e', 'ur5e_base_link',
                       se3.pose_to_matrix([-0.1, 0.5, 0.03],
                                          Rotation.from_euler('z', 70, degrees=True).as_quat()),
                       [0.0, 0.0, 0.1], 5)
    plate = _write_plate(tmp_path, 'ur3e')
    results, code = solve.solve([
        '--session', ur, '--session', fr, '--session', u5,
        '--plate', plate, '--out', str(tmp_path / 'r.yaml')])
    assert code == 0
    # Both non-reference robots are registered (nothing silently dropped).
    assert set(results['pairs'].keys()) == {'fr3', 'ur5e'}
    assert any('registered' in w and 'pairs' in w for w in results['warnings'])
