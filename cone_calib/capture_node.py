"""cone_calib capture node: read-only sampling of hand-guided pivot captures.

Runs one instance per robot, parameterised by ``config/<robot_id>.yaml``.
Subscribes to the robot's joint-state topic and the TF tree; it **never**
publishes anything that moves the robot. The human hand-guides the probe into a
seat and triggers a capture (terminal key or ``std_srvs/Trigger`` service); the
node waits for the arm to settle, averages a window of joint states and flange
poses, and appends the raw record to a timestamped session file.

See docs/2026-10-07_CONE_CALIB_DESIGN.md section 7.
"""

from datetime import datetime
import os
import threading
import time

from cone_calib import io, se3
from cone_calib.kinematics import JointMapper, PoseSourceError, TfPoseSource
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class CaptureNode(Node):
    """Interactive, read-only capture node for one robot."""

    def __init__(self):
        super().__init__('cone_calib_capture')

        self.declare_parameter('config_file', '')
        self.declare_parameter('plate_config', '')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('interactive', True)

        config_file = self.get_parameter('config_file').value
        plate_config = self.get_parameter('plate_config').value
        if not config_file or not plate_config:
            raise RuntimeError(
                'config_file and plate_config parameters are required')

        self.robot_cfg = io.load_robot_config(config_file)
        self.plate_cfg = io.load_plate_config(plate_config)
        self.interactive = bool(self.get_parameter('interactive').value)

        self.robot_id = self.robot_cfg['robot_id']
        self.base_frame = self.robot_cfg['base_frame']
        self.flange_frame = self.robot_cfg['flange_frame']
        self.joint_names = list(self.robot_cfg['joint_names'])
        self.tip_offset = np.asarray(self.robot_cfg['tip_offset'], dtype=float)
        self.pose_source_kind = self.robot_cfg.get('pose_source', 'tf')

        settle = self.robot_cfg.get('settle', {})
        self.vel_threshold = float(settle.get('joint_vel_threshold', 0.002))
        self.settle_time = float(settle.get('settle_time', 0.5))
        self.average_window = float(settle.get('average_window', 1.0))
        self.settle_timeout = float(settle.get('timeout', 10.0))

        self.seat_ids = sorted(self.plate_cfg['seats_nominal'].keys())
        if not self.seat_ids:
            raise RuntimeError('plate config has no seats_nominal entries')
        self.seat_index = 0

        # Session + output path.
        out_dir = self.get_parameter('output_dir').value or \
            os.path.join(os.getcwd(), 'data', 'cone_calib')
        stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.session_dir = os.path.join(out_dir, '%s_%s' % (self.robot_id, stamp))
        self.session_path = os.path.join(self.session_dir, 'session.yaml')
        self.session = io.new_session(self.robot_id, self.robot_cfg, self.pose_source_kind)
        self.session['session_dir'] = self.session_dir
        self.session['base_frame'] = self.base_frame
        self.session['flange_frame'] = self.flange_frame
        self.session['ur_calibration_note'] = (
            'UR factory calibration only applies if the robot was brought up via '
            'the repo launches; confirm no kinematics-hash warning at bringup.')

        # Shared joint-state buffers (written by subscription, read by capture).
        self._lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._mapper = JointMapper(self.joint_names)
        self._latest = None            # (monotonic_t, positions, velocities)
        self._last_pos = None
        self._last_t = None
        self._vel_hist = []            # list of (monotonic_t, velocities)
        self._missing_warned = False
        self._counter = 0
        self._quit = False

        # TF + pose source.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.pose_source_kind != 'tf':
            raise RuntimeError(
                "pose_source '%s' not supported in this node; use 'tf'"
                % self.pose_source_kind)
        self.pose_source = TfPoseSource(self.tf_buffer, self.base_frame, self.flange_frame)

        # The joint_states subscription gets its own reentrant group so it (and
        # the TF listener, which already uses its own group) keeps being
        # serviced while a blocking ~/capture service callback waits for settle
        # and averages. Otherwise the shared default MutuallyExclusiveCallbackGroup
        # would freeze incoming data for the whole capture and settle could never
        # complete in service mode.
        self._data_cbg = ReentrantCallbackGroup()
        self.create_subscription(
            JointState, self.robot_cfg['joint_states_topic'], self._js_cb, 50,
            callback_group=self._data_cbg)

        # Trigger services (read-only; none command motion).
        self.create_service(Trigger, '~/capture', self._srv_capture)
        self.create_service(Trigger, '~/next_seat', self._srv_next_seat)
        self.create_service(Trigger, '~/undo', self._srv_undo)
        self.create_service(Trigger, '~/save', self._srv_save)

        io.save_session(self.session_path, self.session)
        self.get_logger().info(
            'cone_calib capture [%s]: base=%s flange=%s topic=%s' %
            (self.robot_id, self.base_frame, self.flange_frame,
             self.robot_cfg['joint_states_topic']))
        self.get_logger().info('session -> %s' % self.session_path)

    # ------------------------------------------------------------------ data
    def _js_cb(self, msg):
        if not self._mapper.has_all(msg.name):
            if not self._missing_warned:
                self.get_logger().warn(
                    'joint_states missing configured joints; waiting. got=%s'
                    % list(msg.name))
                self._missing_warned = True
            return
        try:
            pos = self._mapper.extract(msg.name, msg.position)
        except PoseSourceError:
            return
        vel = None
        if msg.velocity and len(msg.velocity) == len(msg.name):
            try:
                vel = self._mapper.extract(msg.name, msg.velocity)
            except PoseSourceError:
                vel = None
        now = time.monotonic()
        with self._lock:
            if vel is None:
                if self._last_pos is not None and self._last_t is not None \
                        and now > self._last_t:
                    vel = (pos - self._last_pos) / (now - self._last_t)
                else:
                    vel = np.zeros_like(pos)
            self._last_pos = pos
            self._last_t = now
            self._latest = (now, pos, vel)
            self._vel_hist.append((now, vel))
            cutoff = now - max(2.0 * self.settle_time, 2.0)
            self._vel_hist = [(t, v) for (t, v) in self._vel_hist if t >= cutoff]

    def _is_settled(self):
        now = time.monotonic()
        with self._lock:
            if self._latest is None:
                return False, 'no joint_states received yet'
            older = [v for (t, v) in self._vel_hist if t < now - self.settle_time]
            window = [v for (t, v) in self._vel_hist if t >= now - self.settle_time]
        if not window:
            return False, 'no recent joint_states'
        if not older:
            return False, 'stream younger than settle_time'
        worst = max(float(np.max(np.abs(v))) for v in window)
        return worst < self.vel_threshold, 'max|vel|=%.4f rad/s' % worst

    # --------------------------------------------------------------- capture
    def do_capture(self, seat_id):
        """Settle, average a window, append the record. Returns (ok, message)."""
        with self._capture_lock:
            start = time.monotonic()
            while True:
                settled, info = self._is_settled()
                if settled:
                    break
                if time.monotonic() - start > self.settle_timeout:
                    return False, 'settle timeout after %.1fs (%s)' % (
                        self.settle_timeout, info)
                time.sleep(0.02)

            joints = []
            translations = []
            quats = []
            window_start = time.monotonic()
            while time.monotonic() - window_start < self.average_window:
                with self._lock:
                    latest = self._latest
                if latest is not None:
                    try:
                        trans, quat = self.pose_source.lookup()
                    except PoseSourceError:
                        time.sleep(0.02)
                        continue
                    joints.append(latest[1])
                    translations.append(trans)
                    quats.append(quat)
                time.sleep(0.02)

            if len(joints) < 2 or len(translations) < 2:
                return False, 'too few samples in averaging window (joints=%d poses=%d)' \
                    % (len(joints), len(translations))

            joints = np.asarray(joints)
            mean_j = joints.mean(axis=0)
            std_j = joints.std(axis=0)
            mean_t, mean_q = se3.average_pose(np.asarray(translations), np.asarray(quats))

            index = self._counter
            self._counter += 1
            record = io.make_capture_record(
                seat_id=seat_id, capture_index=index,
                timestamp=self.get_clock().now().nanoseconds * 1e-9,
                joint_names=self.joint_names, joint_positions=mean_j,
                joint_position_std=std_j, translation=mean_t, quat_xyzw=mean_q,
                pose_source=self.pose_source.kind)
            self.session['captures'].append(record)
            io.save_session(self.session_path, self.session)
            return True, self._feedback(seat_id)

    def _feedback(self, seat_id):
        caps = [c for c in self.session['captures'] if c['seat_id'] == seat_id]
        total = len(self.session['captures'])
        rotations = [se3.quat_to_matrix(c['flange_pose']['quaternion_xyzw']) for c in caps]
        spread = 0.0
        for i in range(len(rotations)):
            for j in range(i + 1, len(rotations)):
                ang, _ = se3.relative_rotation(rotations[i], rotations[j])
                spread = max(spread, np.degrees(ang))
        msg = 'captured seat %s: %d here, %d total; tilt spread %.1f deg' % (
            seat_id, len(caps), total, spread)
        if len(caps) >= 3:
            msg += '; pivot(fixed-t) rms %.2f mm' % (1e3 * self._quick_residual(caps))
        return msg

    def _quick_residual(self, caps):
        tips = np.array([
            se3.quat_to_matrix(c['flange_pose']['quaternion_xyzw']) @ self.tip_offset
            + np.asarray(c['flange_pose']['position']) for c in caps])
        centre = tips.mean(axis=0)
        return float(np.sqrt(np.mean(np.sum((tips - centre) ** 2, axis=1))))

    def _save(self):
        """Persist the session under the capture lock (thread-safe, crash-safe)."""
        with self._capture_lock:
            io.save_session(self.session_path, self.session)

    def undo(self):
        """Remove the last capture. Returns (ok, message)."""
        with self._capture_lock:
            if not self.session['captures']:
                return False, 'nothing to undo'
            removed = self.session['captures'].pop()
            io.save_session(self.session_path, self.session)
            return True, 'undid capture %d (seat %s); %d remain' % (
                removed['capture_index'], removed['seat_id'],
                len(self.session['captures']))

    def current_seat(self):
        return self.seat_ids[self.seat_index]

    def next_seat(self):
        self.seat_index = (self.seat_index + 1) % len(self.seat_ids)
        return self.current_seat()

    def set_seat(self, seat_id):
        if seat_id in self.seat_ids:
            self.seat_index = self.seat_ids.index(seat_id)
            return True
        return False

    # -------------------------------------------------------------- services
    def _srv_capture(self, request, response):
        ok, message = self.do_capture(self.current_seat())
        response.success = ok
        response.message = message
        self.get_logger().info(message)
        return response

    def _srv_next_seat(self, request, response):
        response.success = True
        response.message = 'current seat is now %s' % self.next_seat()
        return response

    def _srv_undo(self, request, response):
        response.success, response.message = self.undo()
        return response

    def _srv_save(self, request, response):
        self._save()
        response.success = True
        response.message = 'saved %d captures to %s' % (
            len(self.session['captures']), self.session_path)
        return response

    # -------------------------------------------------------------- terminal
    def run_terminal(self):
        """Blocking stdin loop. Enter=capture, n=next seat, u=undo, s=save, q=quit."""
        help_text = ('\n[cone_calib %s] commands: <Enter>=capture current seat, '
                     'n=next seat, u=undo last, s=save, q=quit+save, '
                     'seat <id>=select seat\nseats: %s\n'
                     % (self.robot_id, ', '.join(self.seat_ids)))
        print(help_text, flush=True)
        while rclpy.ok() and not self._quit:
            try:
                line = input('[%s] seat %s > ' % (self.robot_id, self.current_seat()))
            except (EOFError, KeyboardInterrupt):
                break
            cmd = line.strip()
            lower = cmd.lower()
            if lower == '':
                ok, message = self.do_capture(self.current_seat())
                print(('  OK  ' if ok else '  !!  ') + message, flush=True)
            elif lower == 'n':
                print('  -> seat %s' % self.next_seat(), flush=True)
            elif lower == 'u':
                ok, message = self.undo()
                print('  ' + message, flush=True)
            elif lower == 's':
                self._save()
                print('  saved -> %s' % self.session_path, flush=True)
            elif lower == 'q':
                break
            elif lower.startswith('seat '):
                sid = cmd.split(None, 1)[1].strip()
                print('  -> seat %s' % sid if self.set_seat(sid)
                      else '  unknown seat %s' % sid, flush=True)
            else:
                print(help_text, flush=True)
        self._save()
        print('\nsaved %d captures -> %s' % (
            len(self.session['captures']), self.session_path), flush=True)
        self._quit = True


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        if node.interactive:
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            node.run_terminal()
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._save()
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
