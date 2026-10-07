"""Flange-pose source abstraction for cone_calib.

Two sources: a TF lookup (``base_frame`` -> ``flange_frame`` via a tf2 buffer)
and an offline FK recompute from stored joint positions. The FK path is a
guarded stub in the first cut (see design doc 6.4 / 8): the workspace has no
active FK, and recomputing UR poses correctly requires feeding the per-unit
``my_robot_calibration.yaml`` (else several-mm error).

``JointMapper`` is pure (no ROS import) so it is independently testable; the
TF source only touches the buffer handed to it.
"""

import numpy as np


class PoseSourceError(RuntimeError):
    """Raised when a flange pose cannot be obtained."""


class JointMapper:
    """Reorder an incoming JointState's values into a fixed, configured order.

    The UR ``/joint_states`` order is not pinned (exploration section 2), so we
    must map by name, never by index.
    """

    def __init__(self, joint_names):
        self.joint_names = list(joint_names)
        self._wanted = set(self.joint_names)

    def has_all(self, names):
        """Return True if every configured joint name is present in ``names``."""
        return self._wanted.issubset(set(names))

    def extract(self, names, values):
        """Return the ``values`` reordered to the configured ``joint_names``."""
        if not values or len(values) != len(names):
            raise PoseSourceError('JointState field length mismatch')
        index = {name: i for i, name in enumerate(names)}
        missing = [n for n in self.joint_names if n not in index]
        if missing:
            raise PoseSourceError('JointState missing joints: %s' % missing)
        return np.array([values[index[n]] for n in self.joint_names], dtype=float)


class TfPoseSource:
    """Flange pose from a tf2 buffer (``base_frame`` -> ``flange_frame``)."""

    kind = 'tf'

    def __init__(self, tf_buffer, base_frame, flange_frame):
        self._buffer = tf_buffer
        self.base_frame = base_frame
        self.flange_frame = flange_frame

    def lookup(self, time=None):
        """Return (translation (3,), quaternion xyzw (4,)) or raise PoseSourceError."""
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tf = self._buffer.lookup_transform(
                self.base_frame, self.flange_frame, time if time is not None else Time())
        except TransformException as exc:
            raise PoseSourceError(
                'TF %s -> %s unavailable: %s'
                % (self.base_frame, self.flange_frame, exc))
        t = tf.transform.translation
        q = tf.transform.rotation
        return (np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w]))


class FkPoseSource:
    """Placeholder for joint-position -> flange FK (not implemented in cut 1)."""

    kind = 'fk'

    def __init__(self, *args, **kwargs):
        pass

    def lookup(self, time=None):
        raise NotImplementedError(
            'pose_source: fk is not implemented yet. Use pose_source: tf. '
            'Offline FK recompute needs a KDL/urdf chain and, for the UR, the '
            'per-unit my_robot_calibration.yaml (see design doc 6.4).')


def recompute_poses_from_joints(session, urdf_path=None):
    """Offline FK recompute (``--recompute-fk``) — not implemented in cut 1."""
    raise NotImplementedError(
        '--recompute-fk is not implemented yet. The stored flange poses are '
        'used as-is. Building this requires a KDL/urdf_parser chain fed with '
        'the current URDF and the UR per-unit calibration (design doc 6.4).')
