"""Plays a command sequence on /cmd_vel for data collection (F9, ARCHITECTURE.md 4.1).

    ros2 run base_teleop excitation --ros-args -p sequence_file:=/path/to/steps.yaml

Waits until the base_command relay subscribes to /cmd_vel, plays the segments
at a fixed rate, then sends zero and exits. Ctrl-C also sends zero.

Stops at once (sends zero and exits, never moves again) when
  - any key is pressed in the terminal (only with `ros2 run`, which has a terminal),
  - the Kobuki reports a bumper press, a cliff, a dropped wheel, or one of
    its buttons B0-B2 is pressed (parameter stop_on_kobuki_events, default true).
"""
import os
import select
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32

from base_teleop.safety import KOBUKI_EVENTS, StopLatch, event_stop_reason
from base_teleop.sequence import command_at, load_sequence


class ExcitationNode(Node):

    def __init__(self):
        super().__init__('base_excitation')
        self.declare_parameter('sequence_file', '')
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('start_delay', 2.0)     # s after the relay is found
        self.declare_parameter('stop_on_kobuki_events', True)

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
        self.stop = StopLatch()
        if self.get_parameter('stop_on_kobuki_events').value:
            self._subscribe_kobuki_events()
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.step)
        self.get_logger().info(f'{len(self.segments)} segments, {total:.1f} s, from {path}')

    def _subscribe_kobuki_events(self):
        try:
            import kobuki_ros_interfaces.msg as kmsg
        except ImportError as e:
            raise RuntimeError('kobuki_ros_interfaces not found: cannot stop on bumper/cliff/wheel-drop/button '
                               'events. Install it or set stop_on_kobuki_events:=false') from e
        for topic, (type_name, *_) in KOBUKI_EVENTS.items():
            self.create_subscription(getattr(kmsg, type_name), topic,
                                     lambda msg, t=topic: self.on_event(t, msg), 10)
        self.get_logger().info('stops on Kobuki bumper, cliff, wheel drop and buttons B0-B2')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_event(self, topic, msg):
        reason = event_stop_reason(topic, msg)
        if reason is not None:
            self.emergency_stop(reason)

    def emergency_stop(self, reason):
        """Send zero now and end the run. Safe to call from any thread."""
        if self.stop.trigger(reason):
            self.send(0.0, 0.0, -1)
            self.get_logger().error(f'STOP: {reason} (segment {self._segment}); robot commanded to zero')
        self.done = True

    def step(self):
        if self.stop.stopped:
            self.send(0.0, 0.0, -1)
            self.done = True
            return
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


def watch_keys(node, stream, quit_event):
    """Any key pressed in the terminal stops the run."""
    fd = stream.fileno()
    while not quit_event.is_set():
        ready, _, _ = select.select([fd], [], [], 0.05)
        if ready and os.read(fd, 1):
            node.emergency_stop('key pressed')
            return


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExcitationNode()

    keys, old_term, quit_keys = None, None, threading.Event()
    if sys.stdin.isatty():
        import termios
        import tty
        old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())      # single keys without Enter; Ctrl-C still works
        keys = threading.Thread(target=watch_keys, args=(node, sys.stdin, quit_keys), daemon=True)
        keys.start()
        node.get_logger().info('press ANY KEY (e.g. space) to stop the robot')
    else:
        node.get_logger().warn('no terminal: stop with Ctrl-C (key stop needs ros2 run, not ros2 launch)')

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.emergency_stop('Ctrl-C')
    finally:
        quit_keys.set()
        node.send(0.0, 0.0, -1)
        if old_term is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
