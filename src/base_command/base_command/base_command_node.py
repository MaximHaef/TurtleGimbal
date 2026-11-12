"""base_command node: runs on the robot Pi (ARCHITECTURE.md 4.1).

Single gateway between command sources and the Kobuki driver.

    /cmd_vel (Twist, any source) ──► hold + timeout ──► saturate ──┬─► /commands/velocity (Twist, kobuki_node)
                                                                    └─► /base_command (TwistStamped, u_k)

/base_command carries exactly what was sent to the driver, stamped on the robot
Pi at the moment it was sent. The gimbal Pi uses it as the input u_k of the
predictor, regardless of which machine produced the request.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from base_command.command_hold import CommandHold
from base_command.saturation import saturate


class BaseCommandNode(Node):

    def __init__(self):
        super().__init__('base_command')
        self.declare_parameter('publish_rate', 50.0)        # Hz
        self.declare_parameter('request_timeout', 0.5)      # s, below kobuki_node's cmd_vel_timeout_sec (0.6)
        self.declare_parameter('track_width', 0.230)        # m, b (to verify, ARCHITECTURE.md 5.3)
        self.declare_parameter('wheel_speed_limit', 0.7)    # m/s, v_max (to verify, ARCHITECTURE.md 5.3)

        rate = self.get_parameter('publish_rate').value
        self.hold = CommandHold(self.get_parameter('request_timeout').value)
        self.track_width = self.get_parameter('track_width').value
        self.wheel_speed_limit = self.get_parameter('wheel_speed_limit').value

        self.driver_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.echo_pub = self.create_publisher(TwistStamped, '/base_command', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_request, 10)

        self._was_fresh = False
        self._was_saturated = False
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f'base_command started: {rate} Hz, timeout {self.hold.timeout} s, '
            f'b {self.track_width} m, v_max {self.wheel_speed_limit} m/s')

    def _now(self):
        return self.get_clock().now()

    def on_request(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('ignoring non-finite command')
            return
        self.hold.update(v, w, self._now().nanoseconds * 1e-9)

    def step(self):
        now = self._now()
        v, w, fresh = self.hold.get(now.nanoseconds * 1e-9)
        if self._was_fresh and not fresh:
            self.get_logger().warn('no command request within timeout -> sending zero')
        self._was_fresh = fresh

        v, w, s = saturate(v, w, self.track_width, self.wheel_speed_limit)
        saturated = s < 1.0
        if saturated and not self._was_saturated:
            self.get_logger().warn(f'command above wheel-speed limit, scaled by {s:.2f}')
        self._was_saturated = saturated

        self.publish(v, w, now)

    def publish(self, v, w, stamp):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.driver_pub.publish(cmd)

        echo = TwistStamped()
        echo.header.stamp = stamp.to_msg()
        echo.header.frame_id = 'base_footprint'
        echo.twist = cmd
        self.echo_pub.publish(echo)

    def stop(self):
        """Send zero once on shutdown, so the base does not coast until the driver timeout."""
        self.publish(0.0, 0.0, self._now())


def main(args=None):
    # Keep the context alive on Ctrl-C so the final zero command can still be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
