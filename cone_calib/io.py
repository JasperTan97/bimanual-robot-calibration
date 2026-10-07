"""Load/save raw capture sessions, configs, and results for cone_calib.

ROS-free (YAML + numpy only) so the offline solver and tests do not need a ROS
environment. Raw joint positions are always stored so flange poses can be
recomputed offline after a kinematics update (``--recompute-fk``).
"""

import hashlib
import json
import os
import tempfile

from cone_calib import se3
from cone_calib.pivot import Capture
import numpy as np
import yaml


def load_yaml(path):
    """Load a YAML file into a Python object."""
    with open(path, 'r') as handle:
        return yaml.safe_load(handle)


def load_robot_config(path):
    """Load a per-robot config YAML (config/<robot_id>.yaml)."""
    cfg = load_yaml(path)
    required = ['robot_id', 'base_frame', 'flange_frame', 'joint_states_topic',
                'joint_names', 'tip_offset']
    missing = [key for key in required if key not in cfg]
    if missing:
        raise ValueError('robot config %s missing keys: %s' % (path, missing))
    return cfg


def load_plate_config(path):
    """Load the plate config YAML (config/plate.yaml)."""
    cfg = load_yaml(path)
    if 'seats_nominal' not in cfg:
        raise ValueError('plate config %s missing seats_nominal' % path)
    return cfg


def new_session(robot_id, robot_config, pose_source):
    """Return an empty in-memory session dict for ``robot_id``."""
    return {
        'robot_id': robot_id,
        'pose_source': pose_source,
        'base_frame': robot_config.get('base_frame'),
        'flange_frame': robot_config.get('flange_frame'),
        'joint_names': list(robot_config.get('joint_names', [])),
        'tip_offset': [float(v) for v in robot_config.get('tip_offset', [0, 0, 0])],
        # Embed the robot config so the offline solver knows tip_offset and
        # estimate_tip_offset without a separate --robot-config argument.
        'robot_config': _to_builtin(robot_config),
        'captures': [],
    }


def make_capture_record(seat_id, capture_index, timestamp, joint_names,
                        joint_positions, joint_position_std,
                        translation, quat_xyzw, pose_source):
    """Build one raw per-capture record (all native Python types for YAML)."""
    return {
        'seat_id': str(seat_id),
        'capture_index': int(capture_index),
        'timestamp': float(timestamp),
        'joint_names': list(joint_names),
        'joint_positions': [float(v) for v in joint_positions],
        'joint_position_std': [float(v) for v in joint_position_std],
        'flange_pose': {
            'position': [float(v) for v in translation],
            'quaternion_xyzw': [float(v) for v in quat_xyzw],
        },
        'pose_source': str(pose_source),
    }


def save_session(path, session):
    """Write a session dict to ``path`` atomically (unique temp then rename)."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    # Unique temp in the target dir so concurrent writers never collide and
    # os.replace stays atomic (a fixed ``.tmp`` would race across threads).
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.session-', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as handle:
            yaml.safe_dump(session, handle, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_session(path):
    """Load a raw capture session written by :func:`save_session`."""
    session = load_yaml(path)
    if 'captures' not in session or 'robot_id' not in session:
        raise ValueError('%s is not a cone_calib session file' % path)
    return session


def session_to_captures(session):
    """Convert a loaded session's records into :class:`Capture` objects.

    Uses the stored flange pose (position + XYZW quaternion). Recomputing the
    pose from joint positions is a separate, explicit step (``--recompute-fk``).
    """
    captures = []
    for rec in session['captures']:
        pose = rec['flange_pose']
        rotation = se3.quat_to_matrix(pose['quaternion_xyzw'])
        position = np.asarray(pose['position'], dtype=float)
        captures.append(Capture(
            seat_id=str(rec['seat_id']),
            rotation=rotation,
            position=position,
            index=int(rec.get('capture_index', -1)),
            meta={'pose_source': rec.get('pose_source')},
        ))
    return captures


def config_hash(*configs):
    """Return a short sha256 over one or more config dicts (order-insensitive)."""
    blob = json.dumps(configs, sort_keys=True, default=str).encode('utf-8')
    return hashlib.sha256(blob).hexdigest()[:16]


def static_tf_entry(name, parent, child, transform, source, calibrated_on):
    """Build a static_tfs.yaml-schema entry (translation + XYZW quaternion).

    Matches src/arms_data_collection/config/static_tfs.yaml so the result can be
    pasted straight into that single-source-of-truth file.
    """
    translation, quat = se3.matrix_to_pose(transform)
    return {
        'name': name,
        'parent': parent,
        'child': child,
        'translation': [round(float(v), 9) for v in translation],
        'quaternion': [round(float(v), 9) for v in quat],
        'source': source,
        'calibrated_on': calibrated_on,
    }


def xacro_joint_snippet(joint_name, parent, child, transform):
    """Return a fixed-joint xacro <joint> snippet placing ``child`` in ``parent``."""
    translation, _ = se3.matrix_to_pose(transform)
    roll, pitch, yaw = np.radians(se3.matrix_to_rpy_deg(transform))
    return (
        '<joint name="%s" type="fixed">\n'
        '  <parent link="%s"/>\n'
        '  <child link="%s"/>\n'
        '  <origin xyz="%.9f %.9f %.9f" rpy="%.9f %.9f %.9f"/>\n'
        '</joint>\n'
        % (joint_name, parent, child,
           translation[0], translation[1], translation[2], roll, pitch, yaw)
    )


def _to_builtin(obj):
    """Recursively convert numpy types/arrays to built-ins for YAML dumping."""
    if isinstance(obj, dict):
        return {str(k): _to_builtin(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_builtin(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    return obj


def save_results(path, results):
    """Write the solver results dict (numpy-aware) to ``path`` as YAML."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as handle:
        yaml.safe_dump(_to_builtin(results), handle, sort_keys=False)
