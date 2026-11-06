"""gimbal_control node (ARCHITECTURE.md 4.3).

Fixed-rate control loop with period dt. Starts with a ROS timer; moves to a
real-time thread pinned to a reserved core (or to C++) if jitter is too high.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

MODES = ('feedback_only', 'prediction_assisted')


class ControlNode(Node):

    def __init__(self):
        super().__init__('gimbal_control')
        # Values are TBD (ARCHITECTURE.md 5.3). Set them in gimbal_bringup/config/gimbal.yaml.
        self.declare_parameter('control_period', Parameter.Type.DOUBLE)
        # Runtime switch between controllers (F7).
        self.declare_parameter('mode', 'feedback_only')
        period = self.get_parameter('control_period').value

        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(f'gimbal_control started, period {period} s')

    def step(self):
        mode = self.get_parameter('mode').value
        if mode not in MODES:
            self.get_logger().error(f'unknown mode {mode!r}, using feedback_only')
            mode = 'feedback_only'
        # TODO: read IMU A and IMU B directly (I2C/SPI)
        # TODO: update proposal weights with IMU A, eq. (8), log domain
        # TODO: if prediction_assisted and a valid prediction exists:
        #           weighted mean at t + T_a, eq. (9) -> q_dot_ff
        #       else: q_dot_ff = 0 (feedback-only fallback)
        # TODO: q_dot_fb from e_R, eq. (10)
        # TODO: Limit(...) and send q_dot_cmd to the motor driver
        # TODO: publish internal signals for logging (F8)
        pass

    def stop_motors(self):
        """Safe stop (F10). Called on shutdown and on any fault."""
        # TODO: command zero joint rates / disable the motor driver
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
