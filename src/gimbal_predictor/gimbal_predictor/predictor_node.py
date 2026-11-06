"""Predictor node (ARCHITECTURE.md 4.2).

Runs in its own process on a slower clock than gimbal_control, so that
inference never blocks the control loop.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class PredictorNode(Node):

    def __init__(self):
        super().__init__('predictor')
        # Values are TBD (ARCHITECTURE.md 5.3). Set them in gimbal_bringup/config/gimbal.yaml.
        self.declare_parameter('predictor_period', Parameter.Type.DOUBLE)
        period = self.get_parameter('predictor_period').value

        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(f'predictor started, period {period} s')

    def step(self):
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
