"""Predictor node (ARCHITECTURE.md 4.2).

Runs in its own process on a slower clock than gimbal_control, so that
inference never blocks the control loop.
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time


class PredictorNode(Node):

    def __init__(self):
        super().__init__('predictor')
        # Values are TBD (ARCHITECTURE.md 5.3). Set them in gimbal_bringup/config/gimbal.yaml.
        self.declare_parameter('predictor_period', Parameter.Type.DOUBLE)
        # u_k older than this is not used (base_command relay publishes at a fixed rate).
        self.declare_parameter('command_timeout', 0.2)  # s
        period = self.get_parameter('predictor_period').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # Latest executed base command u_k = (v_cmd, w_cmd), echoed by the robot Pi.
        self.u = None          # (v [m/s], w [rad/s])
        self.u_stamp = None    # rclpy Time, stamped on the robot Pi when sent to the driver
        self.create_subscription(TwistStamped, '/base_command', self.on_base_command, 10)

        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(f'predictor started, period {period} s')

    def on_base_command(self, msg):
        self.u = (msg.twist.linear.x, msg.twist.angular.z)
        self.u_stamp = Time.from_msg(msg.header.stamp)

    def current_command(self):
        """Return (v, w, stamp) of the latest base command, or None if missing or stale."""
        if self.u is None:
            return None
        age = (self.get_clock().now() - self.u_stamp).nanoseconds * 1e-9
        if age > self.command_timeout:
            return None
        return (*self.u, self.u_stamp)

    def step(self):
        command = self.current_command()
        if command is None:
            self.get_logger().warn('no current /base_command, skipping prediction',
                                   throttle_duration_sec=5.0)
            return
        # TODO: build context C_k = (u_k, last H samples of IMU A)
        # TODO: physical rollout, eqs. (4)-(6)
        # TODO: flow-matching residuals, eq. (7), all M samples in one batch
        # TODO: publish M candidate trajectories with the timestamp of step k
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
