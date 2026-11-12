"""Plays a command sequence on /cmd_vel for data collection (F9, ARCHITECTURE.md 4.1).

    ros2 run base_teleop excitation --ros-args -p sequence_file:=/path/to/steps.yaml

Waits until the base_command relay subscribes to /cmd_vel, plays the segments
at a fixed rate, then sends zero and exits. Ctrl-C also sends zero.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32

from base_teleop.sequence import command_at, load_sequence


class ExcitationNode(Node):

    def __init__(self):
        super().__init__('base_excitation')
        self.declare_parameter('sequence_file', '')
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('start_delay', 2.0)     # s after the relay is found

        path = self.get_parameter('sequence_file').value
        if not path:
            raise RuntimeError('parameter sequence_file is not set')
        self.segments = load_sequence(path)
        self.start_delay = self.get_parameter('start_delay').value
        total = sum(d for d, _, _ in self.segments)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Index of the active segment (-1 = idle), recorded with the bag to cut the data later.
        self.segment_pub = self.create_publisher(Int32, '/excitation/segment', 10)
        self.t0 = None
        self.done = False
        self._segment = None
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.step)
        self.get_logger().info(f'{len(self.segments)} segments, {total:.1f} s, from {path}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        if self.t0 is None:
            if self.pub.get_subscription_count() == 0:
                self.get_logger().info('waiting for a /cmd_vel subscriber (base_command relay)...',
                                       throttle_duration_sec=5.0)
                return
            self.t0 = self.now() + self.start_delay
            self.get_logger().info(f'relay found, starting in {self.start_delay} s')

        active = command_at(self.segments, self.now() - self.t0)
        if active is None:
            self.send(0.0, 0.0, -1)
            if self.now() - self.t0 > 0.0:
                self.get_logger().info('sequence finished')
                self.done = True
            return
        i, v, w = active
        if i != self._segment:
            self.get_logger().info(f'segment {i}: v={v} m/s, w={w} rad/s')
            self._segment = i
        self.send(v, w, i)

    def send(self, v, w, segment):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)
        self.segment_pub.publish(Int32(data=segment))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExcitationNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    finally:
        node.send(0.0, 0.0, -1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
