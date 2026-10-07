"""Offline cone_calib solver: raw capture sessions -> calibration results.

ROS-free CLI (run via ``ros2 run cone_calib solve`` or directly). Runs the
per-robot pivot calibration, the inter-robot registration, and the CAD sanity
checks, then writes a results YAML plus a pasteable static_tfs entry, a xacro
snippet, and the ready-to-run publish command. See design doc section 8.
"""

import argparse
from datetime import date
import sys

from cone_calib import io, pivot, registration, se3
import numpy as np


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='cone_calib solve',
        description='Solve base-to-base calibration from raw capture sessions.')
    parser.add_argument('--session', action='append', default=[], required=True,
                        metavar='PATH', help='raw session file (repeat, one per robot)')
    parser.add_argument('--plate', required=True, help='plate config YAML')
    parser.add_argument('--reference-robot', default=None,
                        help='robot id to use as frame A (default: plate reference_robot)')
    parser.add_argument('--out', default=None, help='results YAML path')
    parser.add_argument('--cross-check-static-tfs', default=None,
                        help='static_tfs.yaml to cross-check T_A_B against')
    parser.add_argument('--cross-check-name', default='world_to_base',
                        help='transform name in the static_tfs file (default world_to_base)')
    parser.add_argument('--recompute-fk', action='store_true',
                        help='recompute flange poses from joints (not implemented)')
    # Ignore ROS args injected by `ros2 run`.
    args, _ = parser.parse_known_args(argv)
    return args


def _pose_block(transform):
    translation, quat = se3.matrix_to_pose(transform)
    return {
        'translation': [float(v) for v in translation],
        'quaternion_xyzw': [float(v) for v in quat],
        'rpy_deg': [float(v) for v in se3.matrix_to_rpy_deg(transform)],
        'matrix': [[float(v) for v in row] for row in transform],
    }


def _run_pivot(session, plate):
    cfg = session.get('robot_config', {}) or {}
    tip_offset = cfg.get('tip_offset', session.get('tip_offset', [0.0, 0.0, 0.0]))
    estimate = bool(cfg.get('estimate_tip_offset', True))
    captures = io.session_to_captures(session)
    result = pivot.solve_pivot(
        session['robot_id'], captures, tip_offset_param=tip_offset,
        estimate_tip_offset=estimate,
        max_condition_number=float(plate.get('max_condition_number', 1.0e4)),
        min_tilt_spread_deg=float(plate.get('min_tilt_spread_deg', 15.0)),
        reject_outliers=bool(plate.get('reject_outliers', False)),
        outlier_mad_k=float(plate.get('outlier_mad_k', 4.0)))
    return result


def _pivot_diagnostics(result, min_captures):
    warnings = list(result.warnings)
    for seat, caps in _counts(result).items():
        if caps < min_captures:
            warnings.append('seat %s has %d captures < min_captures_per_seat %d'
                            % (seat, caps, min_captures))
    return {
        'estimated_tip_offset': result.estimated_tip_offset,
        'tip_offset': [float(v) for v in result.tip_offset],
        'tip_offset_param': [float(v) for v in result.tip_offset_param],
        'tip_offset_delta_m': result.tip_offset_delta,
        'rms_overall_m': result.rms_overall,
        'max_overall_m': result.max_overall,
        'per_seat_rms_m': result.per_seat_rms,
        'per_seat_max_m': result.per_seat_max,
        'condition_number': result.condition_number,
        'singular_values': [float(v) for v in result.singular_values],
        'tilt_spread_deg': result.tilt_spread_deg,
        'flagged_outliers': result.flagged_outliers,
        'n_captures': result.n_captures,
        'warnings': warnings,
    }


def _counts(result):
    counts = {}
    for seat in result.capture_seats:
        counts[seat] = counts.get(seat, 0) + 1
    return counts


def _load_cross_check(path, name):
    data = io.load_yaml(path)
    for entry in data.get('transforms', []):
        if entry.get('name') == name:
            transform = se3.pose_to_matrix(entry['translation'], entry['quaternion'])
            return transform, entry.get('parent'), entry.get('child')
    return None


def solve(argv):
    """Run the full pipeline; returns (results_dict, exit_code)."""
    args = _parse_args(argv)
    if args.recompute_fk:
        print('ERROR: --recompute-fk is not implemented in this cut '
              '(see design doc 6.4).', file=sys.stderr)
        return {}, 2

    plate = io.load_plate_config(args.plate)
    min_captures = int(plate.get('min_captures_per_seat', 6))
    sessions = {}
    for path in args.session:
        session = io.load_session(path)
        sessions[session['robot_id']] = {'session': session, 'path': path}
    if len(sessions) < 2:
        print('ERROR: need sessions for at least 2 robots, got %s'
              % list(sessions), file=sys.stderr)
        return {}, 2

    reference = args.reference_robot or plate.get('reference_robot')
    if reference not in sessions:
        print('ERROR: reference robot %r not among sessions %s'
              % (reference, list(sessions)), file=sys.stderr)
        return {}, 2
    others = [r for r in sessions if r != reference]

    results = {
        'generated_by': 'cone_calib solve',
        'calibrated_on': str(date.today()),
        'reference_robot': reference,
        'other_robots': others,
        'inputs': {r: sessions[r]['path'] for r in sessions},
        'config_hash': io.config_hash(plate, {r: sessions[r]['session'].get('robot_config')
                                              for r in sessions}),
        'warnings': [],
    }

    # --- per-robot pivot ---
    pivots = {}
    pivot_diag = {}
    for robot, bundle in sessions.items():
        try:
            res = _run_pivot(bundle['session'], plate)
        except pivot.TipOffsetUnobservable as exc:
            print('ERROR: pivot for %s refused: %s' % (robot, exc), file=sys.stderr)
            results['pivot_error'] = {'robot': robot, 'message': str(exc)}
            io_save(results, args, sessions, reference)
            return results, 1
        pivots[robot] = res
        pivot_diag[robot] = _pivot_diagnostics(res, min_captures)
    results['tip_offsets'] = {r: pivot_diag[r]['tip_offset'] for r in pivots}
    results['seat_centres'] = {
        r: {s: [float(v) for v in c] for s, c in pivots[r].seat_centres.items()}
        for r in pivots}
    results['pivot_diagnostics'] = pivot_diag
    for robot in pivots:
        results['warnings'].extend('[%s] %s' % (robot, w) for w in pivot_diag[robot]['warnings'])

    # --- registration: reference (A) vs EACH other robot (B) ---
    reg_pairs = {}
    for ob in others:
        try:
            reg_pairs[ob] = registration.register_robots(
                pivots[reference].seat_centres, pivots[ob].seat_centres, reference, ob)
        except se3.CollinearPointsError as exc:
            results['warnings'].append(
                'registration %s<-%s failed: %s' % (reference, ob, exc))
    if not reg_pairs:
        print('ERROR: no robot pair could be registered against %s' % reference,
              file=sys.stderr)
        io_save(results, args, sessions, reference)
        return results, 1
    other = others[0] if others[0] in reg_pairs else sorted(reg_pairs)[0]
    reg = reg_pairs[other]
    t_a_b = reg.transform_a_b
    parent = sessions[reference]['session'].get('base_frame', reference)
    child = sessions[other]['session'].get('base_frame', other)
    results['T_A_B'] = {
        'description': 'pose of base B (%s) expressed in base A (%s); x_A = T_A_B @ x_B'
                       % (other, reference),
        **_pose_block(t_a_b)}
    results['registration'] = {
        'shared_seats': reg.shared_seats,
        'rms_m': reg.rms,
        'per_point_residual_m': reg.per_point_residual,
        'spread_singular_values': [float(v) for v in reg.spread_singular_values],
    }
    if len(reg_pairs) > 1:
        results['warnings'].append(
            'registered %d robot pairs against reference %s; top-level T_A_B / '
            'static_tfs_entry / publish_command reflect the %s<-%s pair. See '
            'results["pairs"] for every pair.'
            % (len(reg_pairs), reference, reference, other))

    # --- sanity checks ---
    nominal = plate['seats_nominal']
    sanity = {'inter_seat_vs_cad': {}, 'robot_vs_robot_distances': None,
              'two_route_discrepancy': None, 'cross_check': None}
    for robot in pivots:
        chk = registration.inter_seat_distance_check(pivots[robot].seat_centres, nominal)
        sanity['inter_seat_vs_cad'][robot] = {
            'max_abs_deviation_m': chk['max_abs_deviation'],
            'per_pair_deviation_m': {'%s|%s' % p: v
                                     for p, v in chk['per_pair_deviation'].items()}}
    rr = registration.compare_robot_distances(
        pivots[reference].seat_centres, pivots[other].seat_centres)
    sanity['robot_vs_robot_distances'] = {
        'max_abs_deviation_m': rr['max_abs_deviation'],
        'per_pair_deviation_m': {'%s|%s' % p: v for p, v in rr['per_pair_deviation'].items()}}
    two_route = registration.two_route_discrepancy(
        t_a_b, pivots[reference].seat_centres, pivots[other].seat_centres, nominal)
    if two_route is not None:
        lin, ang, via = two_route
        sanity['two_route_discrepancy'] = {
            'linear_m': lin, 'angular_rad': ang, 'angular_deg': float(np.degrees(ang)),
            'T_A_B_via_cad': _pose_block(via)}

    if args.cross_check_static_tfs:
        loaded = _load_cross_check(args.cross_check_static_tfs, args.cross_check_name)
        if loaded is None:
            results['warnings'].append(
                'cross-check transform %r not found in %s'
                % (args.cross_check_name, args.cross_check_static_tfs))
        else:
            ref_tf, ref_parent, ref_child = loaded
            # Direction-agnostic: the stored transform may be expressed in the
            # opposite parent/child order (e.g. with --reference-robot flipped)
            # or use alias frame names (world==ur3e_base_link, base==fr3_link0),
            # so compare against both it and its inverse and take the closer one.
            lin_f, ang_f = se3.transform_diff(t_a_b, ref_tf)
            lin_i, ang_i = se3.transform_diff(t_a_b, se3.invert_transform(ref_tf))
            use_inv = lin_i < lin_f
            lin, ang = (lin_i, ang_i) if use_inv else (lin_f, ang_f)
            sanity['cross_check'] = {
                'name': args.cross_check_name, 'source': args.cross_check_static_tfs,
                'stored_frames': '%s->%s' % (ref_parent, ref_child),
                'compared_frames': '%s->%s' % (parent, child),
                'matched_direction': 'inverse' if use_inv else 'same',
                'linear_m': lin, 'angular_rad': ang,
                'angular_deg': float(np.degrees(ang)),
                'within_thresholds': bool(lin <= 2e-3 and ang <= 0.05),
                'thresholds': {'linear_m': 2e-3, 'angular_rad': 0.05}}
    results['sanity'] = sanity

    # --- emitters (top-level = the primary reference<-other pair) ---
    entry_name = '%s_to_%s' % (parent, child)
    tf_entry = io.static_tf_entry(
        entry_name, parent, child, t_a_b,
        'cone_calib solve (%s A=%s B=%s)' % (results['calibrated_on'], reference, other),
        results['calibrated_on'])
    results['static_tfs_entry'] = tf_entry
    results['xacro_joint'] = io.xacro_joint_snippet(entry_name, parent, child, t_a_b)
    tr = tf_entry['translation']
    q = tf_entry['quaternion']
    results['publish_command'] = (
        'ros2 launch cone_calib publish_calibration.launch.py '
        'parent:=%s child:=%s x:=%.9f y:=%.9f z:=%.9f qx:=%.9f qy:=%.9f qz:=%.9f qw:=%.9f'
        % (parent, child, tr[0], tr[1], tr[2], q[0], q[1], q[2], q[3]))

    # --- per-pair block: every registered reference<-other pair ---
    results['pairs'] = {}
    for ob, rpair in reg_pairs.items():
        ob_parent = sessions[reference]['session'].get('base_frame', reference)
        ob_child = sessions[ob]['session'].get('base_frame', ob)
        ob_name = '%s_to_%s' % (ob_parent, ob_child)
        results['pairs'][ob] = {
            'T_A_B': _pose_block(rpair.transform_a_b),
            'registration_rms_m': rpair.rms,
            'shared_seats': rpair.shared_seats,
            'static_tfs_entry': io.static_tf_entry(
                ob_name, ob_parent, ob_child, rpair.transform_a_b,
                'cone_calib solve (%s A=%s B=%s)' % (results['calibrated_on'], reference, ob),
                results['calibrated_on']),
        }

    io_save(results, args, sessions, reference)
    _print_summary(results)
    return results, 0


def io_save(results, args, sessions, reference):
    """Pick an output path and write the results YAML."""
    out = args.out
    if out is None:
        base = sessions[reference]['session'].get('session_dir') or '.'
        out = base.rstrip('/') + '/cone_calib_results.yaml'
    results['results_path'] = out
    io.save_results(out, results)


def _print_summary(results):
    print('\n=============== cone_calib results ===============')
    tab = results['T_A_B']
    print('T_A_B (%s)' % tab['description'])
    print('  translation: %s' % tab['translation'])
    print('  quaternion xyzw: %s' % tab['quaternion_xyzw'])
    print('  rpy deg: %s' % tab['rpy_deg'])
    print('registration rms: %.4f mm over seats %s'
          % (1e3 * results['registration']['rms_m'], results['registration']['shared_seats']))
    for robot, diag in results['pivot_diagnostics'].items():
        print('pivot[%s]: tip=%s rms=%.3f mm cond=%.1f%s'
              % (robot, ['%.4f' % v for v in diag['tip_offset']],
                 1e3 * diag['rms_overall_m'], diag['condition_number'],
                 ' est' if diag['estimated_tip_offset'] else ' fixed'))
    cc = results['sanity'].get('cross_check')
    if cc:
        print('cross-check vs %s: linear %.3f mm angular %.4f deg -> %s'
              % (cc['name'], 1e3 * cc['linear_m'], cc['angular_deg'],
                 'OK' if cc['within_thresholds'] else 'OUT OF THRESHOLD'))
    for warning in results['warnings']:
        print('WARN: %s' % warning)
    print('results -> %s' % results.get('results_path'))
    print('publish: %s' % results['publish_command'])
    print('==================================================\n')


def main():
    _, code = solve(sys.argv[1:])
    sys.exit(code)


if __name__ == '__main__':
    main()
