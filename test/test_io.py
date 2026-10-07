"""Unit tests for cone_calib.io (session round-trip, emitters)."""

from cone_calib import io, se3
import numpy as np
from scipy.spatial.transform import Rotation


def test_session_round_trip(tmp_path):
    session = io.new_session('fr3', {
        'base_frame': 'fr3_link0', 'flange_frame': 'fr3_link8',
        'joint_names': ['fr3_joint1'], 'tip_offset': [0, 0, 0.107]}, 'tf')
    session['captures'].append(io.make_capture_record(
        seat_id='p1_s1', capture_index=0, timestamp=123.0,
        joint_names=['fr3_joint1'], joint_positions=[0.1],
        joint_position_std=[1e-5], translation=[0.1, 0.2, 0.3],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0], pose_source='tf'))
    path = str(tmp_path / 'sess.yaml')
    io.save_session(path, session)
    loaded = io.load_session(path)
    assert loaded['robot_id'] == 'fr3'
    captures = io.session_to_captures(loaded)
    assert len(captures) == 1
    assert captures[0].seat_id == 'p1_s1'
    assert np.allclose(captures[0].position, [0.1, 0.2, 0.3])
    assert np.allclose(captures[0].rotation, np.eye(3), atol=1e-12)


def test_config_hash_stable_and_order_insensitive():
    a = {'x': 1, 'y': [1, 2, 3]}
    b = {'y': [1, 2, 3], 'x': 1}
    assert io.config_hash(a) == io.config_hash(b)
    assert io.config_hash(a) != io.config_hash({'x': 2})


def test_xacro_snippet_contains_frames():
    transform = se3.pose_to_matrix([0.1, 0.2, 0.3],
                                   Rotation.from_euler('z', 30, degrees=True).as_quat())
    snippet = io.xacro_joint_snippet('ur_to_fr3', 'ur3e_base_link', 'fr3_link0', transform)
    assert 'ur3e_base_link' in snippet and 'fr3_link0' in snippet
    assert 'type="fixed"' in snippet
