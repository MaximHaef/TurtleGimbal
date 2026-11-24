"""Predictor node (ARCHITECTURE.md 4.2).

Runs in its own process on a slower clock than gimbal_control, so that
inference never blocks the control loop.

Physical part (implemented): at each step it takes the latest executed command
u_k from /base_command (stamp t_k) and the base state measured at t_k (forward
speed from /odom, yaw rate from the Kobuki gyro /sensors/imu_data), rolls out
eq. (6) over F steps, and publishes the predicted translational acceleration
and angular velocity on /gimbal/physical_prediction.

All three inputs are stamped on the robot Pi, so they are compared with each
other only. Whether the command is current is judged by when it arrived here,
which works even if the two Pis' clocks differ.
"""
import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from gimbal_interfaces.msg import BaseMotionPrediction
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from gimbal_predictor.history import History
from gimbal_predictor.physical_model import rollout

HISTORY_LEN = 100  # samples per state signal, about 2 s at 50 Hz


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PredictorNode(Node):

    def __init__(self):
        super().__init__('predictor')
        # TBD values (ARCHITECTURE.md 5.3) have no default: set them in gimbal_bringup/config/gimbal.yaml.
        self.declare_parameter('predictor_period', Parameter.Type.DOUBLE)   # s
        self.declare_parameter('horizon_dt', Parameter.Type.DOUBLE)         # s, = Δt
        self.declare_parameter('horizon_steps', Parameter.Type.INTEGER)     # F
        self.declare_parameter('tau_v', Parameter.Type.DOUBLE)              # s, measured
        self.declare_parameter('tau_omega', Parameter.Type.DOUBLE)          # s, measured
        # A command received longer ago than this is not used (base_command publishes at 50 Hz).
        self.declare_parameter('command_timeout', 0.2)                      # s
        # The state samples used must be at most this much older than t_k.
        self.declare_parameter('state_timeout', 0.1)                        # s

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        period = p('predictor_period')
        self.dt = p('horizon_dt')
        self.steps = p('horizon_steps')
        self.tau_v = p('tau_v')
        self.tau_w = p('tau_omega')
        self.command_timeout = p('command_timeout')
        self.state_timeout = p('state_timeout')
        if self.dt <= 0.0 or self.steps < 1 or self.tau_v <= 0.0 or self.tau_w <= 0.0:
            raise ValueError('horizon_dt, horizon_steps, tau_v and tau_omega must be positive')

        # Latest executed command u_k (robot Pi stamp) and when it arrived here (local clock).
        self.command = None     # TwistStamped
        self.command_rx = None  # rclpy Time
        self.odom_v = History(HISTORY_LEN)
        self.gyro_w = History(HISTORY_LEN)

        self.create_subscription(TwistStamped, '/base_command', self.on_base_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/sensors/imu_data', self.on_gyro, 10)
        self.pub = self.create_publisher(BaseMotionPrediction, '/gimbal/physical_prediction', 10)

        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(
            f'predictor started: period {period} s, horizon {self.steps} x {self.dt} s, '
            f'tau_v {self.tau_v} s, tau_omega {self.tau_w} s')

    def on_base_command(self, msg):
        self.command = msg
        self.command_rx = self.get_clock().now()

    def on_odom(self, msg):
        self.odom_v.add(to_sec(msg.header.stamp), msg.twist.twist.linear.x)

    def on_gyro(self, msg):
        self.gyro_w.add(to_sec(msg.header.stamp), msg.angular_velocity.z)

    def current_command(self):
        """Latest command, or None if none arrived within command_timeout (local time)."""
        if self.command is None:
            return None
        age = (self.get_clock().now() - self.command_rx).nanoseconds * 1e-9
        return self.command if age <= self.command_timeout else None

    def state_at(self, history, t_k, name):
        """Value of the newest sample at or before t_k, or None (with a warning) if missing or too old."""
        sample = history.at_or_before(t_k)
        if sample is None or t_k - sample[0] > self.state_timeout:
            self.get_logger().warn(f'no {name} sample within {self.state_timeout} s before the command, '
                                   'skipping prediction', throttle_duration_sec=5.0)
            return None
        return sample[1]

    def step(self):
        command = self.current_command()
        if command is None:
            self.get_logger().warn('no current /base_command, skipping prediction',
                                   throttle_duration_sec=5.0)
            return
        t_k = to_sec(command.header.stamp)
        v0 = self.state_at(self.odom_v, t_k, '/odom')
        w0 = self.state_at(self.gyro_w, t_k, 'gyro')
        if v0 is None or w0 is None:
            return

        v_cmd, w_cmd = command.twist.linear.x, command.twist.angular.z
        _, accel, omega = rollout(v0, w0, v_cmd, w_cmd, self.tau_v, self.tau_w, self.dt, self.steps)
        self.publish(command, v0, w0, accel, omega)

        # TODO: context C_k = (u_k, last H samples of IMU A)
        # TODO: flow-matching residuals, eq. (7), all M samples in one batch
        # TODO: publish M candidate trajectories with the timestamp of step k

    def publish(self, command, v0, w0, accel, omega):
        msg = BaseMotionPrediction()
        msg.header.stamp = command.header.stamp
        msg.header.frame_id = 'base_footprint'
        msg.dt = float(self.dt)
        msg.command = command.twist
        msg.initial_state.linear.x = float(v0)
        msg.initial_state.angular.z = float(w0)
        msg.linear_acceleration = [Vector3(x=float(a[0]), y=float(a[1]), z=float(a[2])) for a in accel]
        msg.angular_velocity = [Vector3(x=float(w[0]), y=float(w[1]), z=float(w[2])) for w in omega]
        self.pub.publish(msg)


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
