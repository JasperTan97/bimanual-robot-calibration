"""Echo the probe sphere-centre position in the world frame (read-only).

Drive each robot so its probe tip sits at the SAME physical point; both robots
should then report the same world-frame sphere centre if the base-to-base
calibration (and TF tree) is correct. The discrepancy between the two robots is
the base-to-base error, directly in metres.

Reads TF only; never commands motion. The world->flange chain must be published
(each robot's robot_state_publisher, plus the base-to-base transform: either the
arms_data_collection static_tfs publisher, or cone_calib's
publish_calibration.launch.py).
"""

from cone_calib import io, se3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SphereEcho(Node):
    """Periodically print the probe sphere centre expressed in the world frame."""

    def __init__(self):
        super().__init__('cone_calib_echo_sphere')
        self.declare_parameter('config_file', '')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('flange_frame', '')
        self.declare_parameter('tip_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('rate', 2.0)

        config_file = self.get_parameter('config_file').value
        self.world_frame = self.get_parameter('world_frame').value
        flange = self.get_parameter('flange_frame').value
        tip = [float(v) for v in self.get_parameter('tip_offset').value]
        self.robot_id = 'robot'
        if config_file:
            cfg = io.load_robot_config(config_file)
            self.robot_id = cfg['robot_id']
            flange = flange or cfg['flange_frame']
            if tip == [0.0, 0.0, 0.0]:
                tip = [float(v) for v in cfg['tip_offset']]
        if not flange:
            raise RuntimeError('provide either config_file or flange_frame')
        self.flange_frame = flange
        self.tip_offset = np.asarray(tip, dtype=float)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        rate = max(float(self.get_parameter('rate').value), 0.1)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'echo sphere centre [%s]: %s -> %s, tip_offset=%s m'
            % (self.robot_id, self.world_frame, self.flange_frame,
               self.tip_offset.tolist()))

    def _tick(self):
        try:
            tf = self.buffer.lookup_transform(self.world_frame, self.flange_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(
                'TF %s -> %s unavailable: %s'
                % (self.world_frame, self.flange_frame, exc),
                throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        transform = se3.pose_to_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        sphere = (transform @ np.append(self.tip_offset, 1.0))[:3]
        print('[%s] sphere@%s = [% .6f, % .6f, % .6f] m   (flange origin = '
              '[% .4f, % .4f, % .4f])'
              % (self.robot_id, self.world_frame, sphere[0], sphere[1], sphere[2],
                 t.x, t.y, t.z), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = SphereEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
