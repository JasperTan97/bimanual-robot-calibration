"""Pivot (sphere-in-seat) calibration for a single robot, all seats jointly.

ROS-free. For capture ``i`` recorded in seat ``k`` with flange pose
``(R_i, p_i)`` in the robot base and probe-tip offset ``t`` (in the flange
frame), the tip centre must coincide with the fixed seat centre ``c_k``::

    R_i t + p_i = c_k   =>   [ R_i | -E_k ] [ t ; c_1 ; ... ; c_K ] = -p_i

Stacking every capture gives one linear system with ``3 + 3K`` unknowns solved
by least squares. See docs/2026-10-07_CONE_CALIB_DESIGN.md section 4.1.
"""

from dataclasses import dataclass, field

from cone_calib import se3
import numpy as np


@dataclass
class Capture:
    """One averaged capture: flange pose (R, p) in the robot base, tagged by seat."""

    seat_id: str
    rotation: np.ndarray            # (3, 3)
    position: np.ndarray            # (3,)
    index: int = -1
    meta: dict = field(default_factory=dict)


class TipOffsetUnobservable(RuntimeError):
    """Raised when the pivot system is too ill-conditioned to estimate ``t``."""

    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


@dataclass
class PivotResult:
    """Result of a pivot calibration plus all requested diagnostics."""

    robot_id: str
    tip_offset: np.ndarray
    estimated_tip_offset: bool
    seat_ids: list
    seat_centres: dict                      # seat_id -> (3,)
    per_capture_residual: np.ndarray        # (M,) in capture order
    capture_seats: list                     # (M,) seat_id per capture
    rms_overall: float
    max_overall: float
    per_seat_rms: dict
    per_seat_max: dict
    singular_values: np.ndarray
    condition_number: float
    tip_offset_param: np.ndarray
    tip_offset_delta: float
    tilt_spread_deg: dict                   # seat_id -> max pairwise tilt (deg)
    warnings: list
    flagged_outliers: list                  # list of capture indices (global)
    n_captures: int


def _group_by_seat(captures):
    seats = {}
    for cap in captures:
        seats.setdefault(cap.seat_id, []).append(cap)
    return seats


def _tilt_spread_deg(caps):
    """Return the max pairwise relative-rotation angle (deg) over a seat's captures."""
    max_angle = 0.0
    for i in range(len(caps)):
        for j in range(i + 1, len(caps)):
            angle, _ = se3.relative_rotation(caps[i].rotation, caps[j].rotation)
            max_angle = max(max_angle, angle)
    return np.degrees(max_angle)


def _axes_nearly_parallel(caps, tol_deg):
    """Return True if every pairwise relative-rotation axis aligns within ``tol_deg``.

    Rotations purely about a single (e.g. tool) axis leave ``t`` unobservable.
    """
    axes = []
    for i in range(len(caps)):
        for j in range(i + 1, len(caps)):
            angle, axis = se3.relative_rotation(caps[i].rotation, caps[j].rotation)
            if angle > np.radians(1.0):       # ignore near-identical orientations
                axes.append(axis)
    if len(axes) < 2:
        return False
    axes = np.asarray(axes)
    # Dominant direction = top eigenvector of sum(a a^T); flip signs to it.
    dominant = np.linalg.eigh(axes.T @ axes)[1][:, -1]
    cosines = np.abs(axes @ dominant)
    max_dev_deg = np.degrees(np.arccos(np.clip(cosines.min(), -1.0, 1.0)))
    return max_dev_deg < tol_deg


def _solve_once(captures, estimate_tip_offset, tip_offset_param):
    """Core least-squares solve. Returns (tip_offset, centres, A_singular_values)."""
    seat_ids = sorted({cap.seat_id for cap in captures})
    seat_index = {sid: k for k, sid in enumerate(seat_ids)}
    num_seats = len(seat_ids)
    num_caps = len(captures)

    if estimate_tip_offset:
        cols = 3 + 3 * num_seats
        a_mat = np.zeros((3 * num_caps, cols))
        b_vec = np.zeros(3 * num_caps)
        for row, cap in enumerate(captures):
            block = slice(3 * row, 3 * row + 3)
            a_mat[block, 0:3] = cap.rotation
            k = seat_index[cap.seat_id]
            a_mat[block, 3 + 3 * k:6 + 3 * k] = -np.eye(3)
            b_vec[block] = -cap.position
        solution, _, _, singular_values = np.linalg.lstsq(a_mat, b_vec, rcond=None)
        tip_offset = solution[0:3]
        centres = {sid: solution[3 + 3 * seat_index[sid]:6 + 3 * seat_index[sid]]
                   for sid in seat_ids}
    else:
        tip_offset = np.asarray(tip_offset_param, dtype=float).reshape(3)
        # c_k = mean_i(R_i t + p_i) over captures in seat k.
        centres = {}
        for sid, caps in _group_by_seat(captures).items():
            tips = np.array([cap.rotation @ tip_offset + cap.position for cap in caps])
            centres[sid] = tips.mean(axis=0)
        # Singular values of the (block-diagonal) averaging system are trivial;
        # report the spread of the stacked rotation blocks for context.
        a_mat = np.vstack([cap.rotation for cap in captures])
        singular_values = np.linalg.svd(a_mat, compute_uv=False)
    return tip_offset, centres, seat_ids, singular_values


def _residuals(captures, tip_offset, centres):
    norms = np.zeros(len(captures))
    for row, cap in enumerate(captures):
        tip = cap.rotation @ tip_offset + cap.position
        norms[row] = np.linalg.norm(tip - centres[cap.seat_id])
    return norms


def _flag_outliers(residual_norms, mad_k):
    """One-sided robust flag: residual > median + mad_k * (scaled MAD)."""
    if len(residual_norms) == 0:
        return np.zeros(0, dtype=bool)
    median = np.median(residual_norms)
    mad = 1.4826 * np.median(np.abs(residual_norms - median))
    if mad <= 0.0:
        return np.zeros(len(residual_norms), dtype=bool)
    return residual_norms > (median + mad_k * mad)


def solve_pivot(robot_id, captures, tip_offset_param,
                estimate_tip_offset=True,
                max_condition_number=1.0e4,
                min_tilt_spread_deg=15.0,
                axis_parallel_tol_deg=10.0,
                reject_outliers=False,
                outlier_mad_k=4.0):
    """Run the pivot calibration with diagnostics and observability guards.

    Raises:
        TipOffsetUnobservable: when ``estimate_tip_offset`` is requested but the
            stacked system's condition number exceeds ``max_condition_number``.
    """
    captures = list(captures)
    if len(captures) == 0:
        raise ValueError('no captures provided for robot %r' % robot_id)
    tip_offset_param = np.asarray(tip_offset_param, dtype=float).reshape(3)

    tip_offset, centres, seat_ids, singular_values = _solve_once(
        captures, estimate_tip_offset, tip_offset_param)

    positive_sv = singular_values[singular_values > 0.0]
    condition_number = (float(singular_values[0] / positive_sv[-1])
                        if len(positive_sv) else float('inf'))

    warnings = []
    seats = _group_by_seat(captures)

    # Observability: per-seat tilt spread + relative-axis parallelism.
    tilt_spread = {}
    for sid, caps in seats.items():
        spread = _tilt_spread_deg(caps)
        tilt_spread[sid] = spread
        if estimate_tip_offset and spread < min_tilt_spread_deg:
            warnings.append(
                'seat %s tilt spread %.1f deg < %.1f deg min; tip offset weakly '
                'observable' % (sid, spread, min_tilt_spread_deg))
        if estimate_tip_offset and _axes_nearly_parallel(caps, axis_parallel_tol_deg):
            warnings.append(
                'seat %s relative-rotation axes are nearly parallel; tip offset '
                'is unobservable along the shared axis' % sid)

    # Hard refuse on ill-conditioning when estimating the tip offset.
    if estimate_tip_offset and condition_number > max_condition_number:
        raise TipOffsetUnobservable(
            'condition number %.3e exceeds max %.3e; refusing to estimate the '
            'tip offset. Capture more diverse tool orientations (tilt spread '
            '>= %.1f deg about varied axes), or set estimate_tip_offset=false.'
            % (condition_number, max_condition_number, min_tilt_spread_deg),
            diagnostics={'condition_number': condition_number,
                         'singular_values': singular_values,
                         'tilt_spread_deg': tilt_spread})

    residual_norms = _residuals(captures, tip_offset, centres)
    outlier_mask = _flag_outliers(residual_norms, outlier_mad_k)
    flagged = [captures[i].index for i in np.nonzero(outlier_mask)[0]]

    # Captures whose residuals/stats are reported. After a reject re-solve this
    # is the inlier set, so every seat in `seat_ids` keeps >= 1 capture here and
    # `centres` is never indexed by a dropped seat.
    report_captures = captures
    if reject_outliers and outlier_mask.any() and (~outlier_mask).sum() >= 3:
        inliers = [cap for cap, bad in zip(captures, outlier_mask) if not bad]
        tip_offset, centres, seat_ids, singular_values = _solve_once(
            inliers, estimate_tip_offset, tip_offset_param)
        positive_sv = singular_values[singular_values > 0.0]
        condition_number = (float(singular_values[0] / positive_sv[-1])
                            if len(positive_sv) else float('inf'))
        warnings.append('rejected %d outlier capture(s): %s'
                        % (len(flagged), flagged))
        dropped_seats = sorted({cap.seat_id for cap in captures} - set(seat_ids))
        if dropped_seats:
            warnings.append(
                'seat(s) %s had ALL captures rejected as outliers and were '
                'dropped from the solution' % dropped_seats)
        report_captures = inliers
        residual_norms = _residuals(report_captures, tip_offset, centres)

    per_seat_rms = {}
    per_seat_max = {}
    capture_seats = [cap.seat_id for cap in report_captures]
    for sid in seat_ids:
        idx = [i for i, cap in enumerate(report_captures) if cap.seat_id == sid]
        seat_norms = residual_norms[idx]
        per_seat_rms[sid] = float(np.sqrt(np.mean(seat_norms ** 2)))
        per_seat_max[sid] = float(seat_norms.max())

    return PivotResult(
        robot_id=robot_id,
        tip_offset=tip_offset,
        estimated_tip_offset=estimate_tip_offset,
        seat_ids=seat_ids,
        seat_centres=centres,
        per_capture_residual=residual_norms,
        capture_seats=capture_seats,
        rms_overall=float(np.sqrt(np.mean(residual_norms ** 2))),
        max_overall=float(residual_norms.max()),
        per_seat_rms=per_seat_rms,
        per_seat_max=per_seat_max,
        singular_values=singular_values,
        condition_number=condition_number,
        tip_offset_param=tip_offset_param,
        tip_offset_delta=float(np.linalg.norm(tip_offset - tip_offset_param)),
        tilt_spread_deg=tilt_spread,
        warnings=warnings,
        flagged_outliers=flagged,
        n_captures=len(report_captures),
    )
