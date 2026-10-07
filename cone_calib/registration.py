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
