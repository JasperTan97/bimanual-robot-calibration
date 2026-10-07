"""Convention guards: T_A_B direction and XYZW quaternion order.

These exist so a frame-inversion or quaternion-order bug cannot pass silently.
"""

from cone_calib import io, registration, se3
import numpy as np
from scipy.spatial.transform import Rotation


def test_quaternion_is_xyzw():
    # 90 deg about +z -> scipy XYZW quaternion is [0, 0, sin45, cos45].
    quat = Rotation.from_euler('z', 90, degrees=True).as_quat()
    assert np.allclose(quat, [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-12)
    # pose_to_matrix must interpret the 3rd/4th entries as z/w, not w/z.
    mat = se3.pose_to_matrix([0, 0, 0], quat)
    assert np.allclose(mat[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_t_a_b_direction_maps_b_into_a():
    # x_A = T_A_B @ x_B. Build a known T_A_B and verify the mapping direction.
    t_a_b = se3.pose_to_matrix([0.5, -0.2, 0.1],
                               Rotation.from_euler('z', 30, degrees=True).as_quat())
    x_b = np.array([1.0, 0.0, 0.0, 1.0])
    x_a = t_a_b @ x_b
    expected = t_a_b[:3, :3] @ [1.0, 0.0, 0.0] + t_a_b[:3, 3]
    assert np.allclose(x_a[:3], expected, atol=1e-12)


def test_registration_direction_ur_as_a():
    # A = ur3e (reference), B = fr3. Seat centres in B map into A via T_A_B.
    rng = np.random.default_rng(7)
    t_a_b = se3.pose_to_matrix(rng.normal(size=3), Rotation.random(random_state=rng).as_quat())
    seats = {'p1_s1': [0.0, 0.0, 0.0], 'p1_s2': [0.15, 0.0, 0.0],
             'p1_s3': [0.0, 0.15, 0.0], 'p1_s4': [0.1, 0.12, 0.03]}
    centres_b = {s: np.asarray(c, dtype=float) for s, c in seats.items()}
    centres_a = {s: (t_a_b @ np.append(c, 1.0))[:3] for s, c in centres_b.items()}
    result = registration.register_robots(
        centres_a, centres_b, reference_robot='ur3e', other_robot='fr3')
    assert result.reference_robot == 'ur3e'
    assert np.allclose(result.transform_a_b, t_a_b, atol=1e-9)
    assert result.rms < 1e-9


def test_static_tf_entry_is_xyzw_schema():
    transform = se3.pose_to_matrix([0.1, 0.2, 0.3],
                                   Rotation.from_euler('x', 15, degrees=True).as_quat())
    entry = io.static_tf_entry('ur3e_base_link_to_fr3_link0', 'ur3e_base_link',
                               'fr3_link0', transform, 'cone_calib', '2026-10-07')
    assert entry['parent'] == 'ur3e_base_link' and entry['child'] == 'fr3_link0'
    assert len(entry['translation']) == 3 and len(entry['quaternion']) == 4
    # quaternion stored XYZW: rebuilding must reproduce the rotation.
    rebuilt = se3.pose_to_matrix(entry['translation'], entry['quaternion'])
    assert np.allclose(rebuilt, transform, atol=1e-8)
