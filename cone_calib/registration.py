"""Inter-robot registration and CAD sanity checks for cone_calib.

ROS-free. Consumes the per-robot seat centres produced by :mod:`cone_calib.pivot`
and solves the rigid transform between the two robot bases, then runs the
nominal-distance sanity checks. See design doc section 4.2 / 4.3.
"""

from dataclasses import dataclass, field
from itertools import combinations

from cone_calib import se3
import numpy as np


@dataclass
class RegistrationResult:
    """Rigid transform between two robot bases, with residuals."""

    reference_robot: str                    # robot A
    other_robot: str                        # robot B
    transform_a_b: np.ndarray               # T_A_B: x_A = T_A_B @ x_B
    shared_seats: list
    per_point_residual: dict                # seat_id -> residual norm (m), in A
    rms: float
    spread_singular_values: np.ndarray
    warnings: list = field(default_factory=list)


def register_robots(centres_a, centres_b, reference_robot, other_robot):
    """Solve ``T_A_B`` (pose of base B in base A) from matched seat centres.

    Args:
        centres_a: dict seat_id -> (3,) centre in robot A's base frame.
        centres_b: dict seat_id -> (3,) centre in robot B's base frame.
        reference_robot: id of robot A (for labelling).
        other_robot: id of robot B (for labelling).

    Correspondence is purely by shared seat_id. ``x_A = T_A_B @ x_B``, so the
    registration maps B points onto A points.

    Raises:
        se3.CollinearPointsError: fewer than 3 shared non-collinear seats.
    """
    shared = sorted(set(centres_a) & set(centres_b))
    if len(shared) < 3:
        raise se3.CollinearPointsError(
            'need >= 3 shared seats between %s and %s, got %d: %s'
            % (reference_robot, other_robot, len(shared), shared))

    src = np.array([centres_b[s] for s in shared])   # B points
    dst = np.array([centres_a[s] for s in shared])   # A points
    transform, info = se3.rigid_transform_from_points(src, dst)

    per_point = {s: float(r) for s, r in zip(shared, info['per_point_residual'])}
    return RegistrationResult(
        reference_robot=reference_robot,
        other_robot=other_robot,
        transform_a_b=transform,
        shared_seats=shared,
        per_point_residual=per_point,
        rms=info['rms'],
        spread_singular_values=info['spread_singular_values'],
    )


def _pairwise_distances(centres, seat_ids):
    return {tuple(sorted(pair)): float(np.linalg.norm(centres[pair[0]] - centres[pair[1]]))
            for pair in combinations(seat_ids, 2)}


def inter_seat_distance_check(centres, nominal):
    """Compare a robot's measured inter-seat distances to the CAD nominal.

    Args:
        centres: dict seat_id -> (3,) in the robot base.
        nominal: dict seat_id -> (3,) in the plate frame.

    Returns a dict with per-pair deviations (m) and the max absolute deviation.
    """
    shared = sorted(set(centres) & set(nominal))
    measured = _pairwise_distances(centres, shared)
    cad = _pairwise_distances({s: np.asarray(nominal[s], dtype=float) for s in shared},
                              shared)
    deviations = {pair: measured[pair] - cad[pair] for pair in measured}
    max_abs = max((abs(v) for v in deviations.values()), default=0.0)
    return {'per_pair_deviation': deviations,
            'measured': measured,
            'nominal': cad,
            'max_abs_deviation': max_abs}


def compare_robot_distances(centres_a, centres_b):
    """Compare inter-seat distances measured by the two robots against each other."""
    shared = sorted(set(centres_a) & set(centres_b))
    dist_a = _pairwise_distances(centres_a, shared)
    dist_b = _pairwise_distances(centres_b, shared)
    deviations = {pair: dist_a[pair] - dist_b[pair] for pair in dist_a}
    max_abs = max((abs(v) for v in deviations.values()), default=0.0)
    return {'per_pair_deviation': deviations, 'max_abs_deviation': max_abs}


def _angle_between_deg(v1, v2):
    """Return the angle (deg) between two 3-vectors, or None if either is zero."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 == 0.0 or n2 == 0.0:
        return None
    cos = float(np.dot(v1, v2) / (n1 * n2))
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def _best_orthogonal_triple(nominal):
    """Pick (origin, a, b) whose nominal vectors are closest to orthogonal."""
    seats = sorted(nominal)
    best = None
    best_dev = float('inf')
    for origin in seats:
        for a, b in combinations([s for s in seats if s != origin], 2):
            ang = _angle_between_deg(
                np.asarray(nominal[a], float) - np.asarray(nominal[origin], float),
                np.asarray(nominal[b], float) - np.asarray(nominal[origin], float))
            if ang is None:
                continue
            dev = abs(ang - 90.0)
            if dev < best_dev:
                best_dev = dev
                best = (origin, a, b)
    return best


def orthogonality_check(centres, nominal, triple=None):
    """Check the angle between two plate vectors and its deviation from 90 deg.

    Picks (or takes) three seats (origin, a, b); forms the measured vectors
    (origin->a) and (origin->b) from the robot-measured seat centres, and reports
    the measured angle, the nominal (CAD) angle, and the deviation from 90 deg.
    Returns None if the triple cannot be formed from the available centres.
    """
    if triple is None:
        triple = _best_orthogonal_triple(nominal)
    if triple is None or len(triple) != 3:
        return None
    origin, a, b = triple
    if not all(k in centres for k in (origin, a, b)):
        return None
    v1 = np.asarray(centres[a], float) - np.asarray(centres[origin], float)
    v2 = np.asarray(centres[b], float) - np.asarray(centres[origin], float)
    measured = _angle_between_deg(v1, v2)
    if measured is None:
        return None
    nominal_angle = None
    if all(k in nominal for k in (origin, a, b)):
        nominal_angle = _angle_between_deg(
            np.asarray(nominal[a], float) - np.asarray(nominal[origin], float),
            np.asarray(nominal[b], float) - np.asarray(nominal[origin], float))
    out = {
        'triple': [origin, a, b],
        'measured_angle_deg': measured,
        'nominal_angle_deg': nominal_angle,
        'deviation_from_90_deg': measured - 90.0,
    }
    if nominal_angle is not None:
        out['deviation_from_nominal_deg'] = measured - nominal_angle
    return out


def register_to_nominal(centres, nominal):
    """Return ``T_robot_plate`` (pose of the plate frame in the robot base).

    ``x_robot = T_robot_plate @ x_plate``.
    """
    shared = sorted(set(centres) & set(nominal))
    if len(shared) < 3:
        raise se3.CollinearPointsError(
            'need >= 3 shared seats with the CAD nominal, got %d' % len(shared))
    src = np.array([np.asarray(nominal[s], dtype=float) for s in shared])   # plate
    dst = np.array([centres[s] for s in shared])                            # robot
    transform, info = se3.rigid_transform_from_points(src, dst)
    return transform, info


def two_route_discrepancy(transform_a_b, centres_a, centres_b, nominal):
    """Compare direct ``T_A_B`` with the CAD route ``T_A_plate @ inv(T_B_plate)``.

    Returns (linear_m, angular_rad, transform_a_b_via_cad) or None if either
    robot cannot be registered to the CAD nominal.
    """
    try:
        t_a_plate, _ = register_to_nominal(centres_a, nominal)
        t_b_plate, _ = register_to_nominal(centres_b, nominal)
    except se3.CollinearPointsError:
        return None
    via_cad = se3.compose(t_a_plate, se3.invert_transform(t_b_plate))
    linear, angular = se3.transform_diff(transform_a_b, via_cad)
    return linear, angular, via_cad
