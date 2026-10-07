"""SE(3) and rigid-registration helpers for cone_calib.

ROS-free. Pure numpy/scipy so the module is unit-testable and usable offline.

Conventions:
  * Rotations go through ``scipy.spatial.transform.Rotation``.
  * Quaternions are ``(x, y, z, w)`` (scipy/XYZW order) everywhere a
    quaternion crosses a function boundary.
  * A homogeneous transform ``T`` is a 4x4 matrix with ``T[:3, :3]`` the
    rotation and ``T[:3, 3]`` the translation, acting on column vectors:
    ``x_parent = T @ [x_child; 1]``.
  * ``T_A_B`` denotes the pose of frame ``B`` expressed in frame ``A`` so
    that ``x_A = T_A_B @ x_B``.
"""

import numpy as np
from scipy.spatial.transform import Rotation


class CollinearPointsError(ValueError):
    """Raised when a point set is too degenerate for rigid registration."""


def quat_to_matrix(quat_xyzw):
    """Return the 3x3 rotation matrix for a quaternion given as (x, y, z, w)."""
    return Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()


def matrix_to_quat(rot):
    """Return the (x, y, z, w) quaternion for a 3x3 rotation matrix."""
    return Rotation.from_matrix(np.asarray(rot, dtype=float)).as_quat()


def pose_to_matrix(translation, quat_xyzw):
    """Build a 4x4 transform from a translation and an (x, y, z, w) quaternion."""
    t = np.asarray(translation, dtype=float).reshape(3)
    out = np.eye(4)
    out[:3, :3] = quat_to_matrix(quat_xyzw)
    out[:3, 3] = t
    return out


def matrix_to_pose(transform):
    """Split a 4x4 transform into (translation (3,), quaternion (x, y, z, w))."""
    transform = np.asarray(transform, dtype=float)
    translation = transform[:3, 3].copy()
    quat = matrix_to_quat(transform[:3, :3])
    return translation, quat


def invert_transform(transform):
    """Return the inverse of a 4x4 rigid transform."""
    transform = np.asarray(transform, dtype=float)
    rot = transform[:3, :3]
    t = transform[:3, 3]
    out = np.eye(4)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ t
    return out


def compose(*transforms):
    """Return the left-to-right matrix product of the given 4x4 transforms."""
    out = np.eye(4)
    for transform in transforms:
        out = out @ np.asarray(transform, dtype=float)
    return out


def matrix_to_rpy_deg(transform_or_rot):
    """Return intrinsic-xyz roll/pitch/yaw in degrees for a transform or rotation."""
    mat = np.asarray(transform_or_rot, dtype=float)
    if mat.shape == (4, 4):
        mat = mat[:3, :3]
    return Rotation.from_matrix(mat).as_euler('xyz', degrees=True)


def quaternion_mean(quats_xyzw, weights=None):
    """Average unit quaternions via the eigenvector method (Markley et al.).

    Input and output are (x, y, z, w). The result is sign-canonicalised to a
    non-negative scalar (w) component. The eigenvector method is order-agnostic
    because it operates on the outer products of the quaternion components.
    """
    quats = np.asarray(quats_xyzw, dtype=float)
    if quats.ndim != 2 or quats.shape[1] != 4:
        raise ValueError('quats_xyzw must have shape (N, 4)')
    if weights is None:
        weights = np.ones(len(quats))
    weights = np.asarray(weights, dtype=float)
    accum = np.zeros((4, 4))
    for quat, weight in zip(quats, weights):
        norm = np.linalg.norm(quat)
        if norm == 0.0:
            raise ValueError('encountered a zero quaternion')
        unit = quat / norm
        accum += weight * np.outer(unit, unit)
    accum /= np.sum(weights)
    eigvals, eigvecs = np.linalg.eigh(accum)
    mean = eigvecs[:, int(np.argmax(eigvals))]
    if mean[3] < 0.0:
        mean = -mean
    return mean


def average_pose(translations, quats_xyzw):
    """Return (mean translation, mean quaternion xyzw) for a batch of poses."""
    translations = np.asarray(translations, dtype=float)
    mean_t = translations.mean(axis=0)
    mean_q = quaternion_mean(quats_xyzw)
    return mean_t, mean_q


def relative_rotation(rot_a, rot_b):
    """Return (angle_rad, unit_axis) of the rotation taking rot_a to rot_b."""
    rel = Rotation.from_matrix(np.asarray(rot_a, dtype=float)).inv() * \
        Rotation.from_matrix(np.asarray(rot_b, dtype=float))
    rotvec = rel.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        axis = np.array([0.0, 0.0, 0.0])
    else:
        axis = rotvec / angle
    return angle, axis


def kabsch_umeyama(src, dst):
    """Least-squares rigid transform mapping ``src`` onto ``dst``.

    Solves for rotation ``R`` and translation ``t`` minimising
    ``sum_i || dst_i - (R @ src_i + t) ||^2`` with a proper rotation
    (reflections are corrected via the sign of the determinant). No scaling.

    Args:
        src: (N, 3) source points.
        dst: (N, 3) target points.

    Returns:
        (R (3x3), t (3,), info) where ``info`` carries the per-point rms, the
        singular values of the cross-covariance, and the singular values of the
        centred source point matrix (for degeneracy inspection).

    Raises:
        CollinearPointsError: fewer than 3 points, or the centred source points
            are collinear (second singular value negligible vs the first).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError('src and dst must both have shape (N, 3)')
    n = src.shape[0]
    if n < 3:
        raise CollinearPointsError('need at least 3 point pairs, got %d' % n)

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    spread_sv = np.linalg.svd(src_c, compute_uv=False)
    # A rank < 2 centred set is collinear (or coincident); a unique 3D rigid
    # pose cannot be recovered. Coplanar-but-not-collinear (rank 2) is fine.
    if spread_sv[0] <= 0.0 or spread_sv[1] <= 1e-9 * spread_sv[0]:
        raise CollinearPointsError(
            'shared points are collinear/coincident (singular values %s)'
            % np.array2string(spread_sv, precision=6))

    cov = src_c.T @ dst_c
    u_mat, cov_sv, vt_mat = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(vt_mat.T @ u_mat.T))
    correction = np.diag([1.0, 1.0, d])
    rot = vt_mat.T @ correction @ u_mat.T
    t = mu_dst - rot @ mu_src

    residuals = dst - (src @ rot.T + t)
    rms = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    info = {
        'rms': rms,
        'per_point_residual': np.linalg.norm(residuals, axis=1),
        'cov_singular_values': cov_sv,
        'spread_singular_values': spread_sv,
    }
    return rot, t, info


def rigid_transform_from_points(src, dst):
    """Return a 4x4 ``T`` with ``dst ~= T @ src`` (homogeneous), plus info dict."""
    rot, t, info = kabsch_umeyama(src, dst)
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = t
    return transform, info


def transform_diff(transform_a, transform_b):
    """Return (linear_m, angular_rad) distance between two 4x4 transforms.

    Linear is the translation difference norm; angular is the shortest-arc
    angle of the relative rotation (matches arms_data_collection
    ``print_tf_precise``: ``2*acos(|w|)``).
    """
    transform_a = np.asarray(transform_a, dtype=float)
    transform_b = np.asarray(transform_b, dtype=float)
    linear = float(np.linalg.norm(transform_a[:3, 3] - transform_b[:3, 3]))
    rel = Rotation.from_matrix(transform_a[:3, :3]).inv() * \
        Rotation.from_matrix(transform_b[:3, :3])
    angular = float(np.linalg.norm(rel.as_rotvec()))
    return linear, angular
