"""Keyboard teleop for the Kobuki, run on the gimbal Pi (ARCHITECTURE.md 4.1).

Publishes /cmd_vel at a fixed rate. The base_command relay on the robot Pi
forwards it to the driver. Hold-to-drive: a key keeps its command for
`key_hold` seconds (long enough to bridge the terminal's auto-repeat delay);
after that the node sends zero.

    ros2 run base_teleop keyboard
"""
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# key -> (sign of v, sign of w); same meaning as helloworld.cpp on the robot Pi.
KEYS = {
    'w': (1, 0),    # forward
    's': (-1, 0),   # backward
    'a': (1, 1),    # forward + left
    'd': (1, -1),   # forward + right
    'l': (-1, -1),  # backward, turning as in helloworld 'l'
    'r': (-1, 1),   # backward, turning as in helloworld 'r'
    'j': (0, 1),    # turn in place left
    'k': (0, -1),   # turn in place right
}
STOP_KEY = ' '
QUIT_KEY = 'q'

HELP = """
Kobuki keyboard teleop (publishes /cmd_vel)
  w forward      s backward
  a fwd + left   d fwd + right
  l back (l)     r back (r)
  j turn left    k turn right
  space stop     q quit
Hold a key to keep driving.
"""


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('base_keyboard')
        self.declare_parameter('linear_speed', 0.2)    # m/s
        self.declare_parameter('angular_speed', 0.8)   # rad/s
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('key_hold', 0.6)        # s

        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        self.key_hold = self.get_parameter('key_hold').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)
        self._key_time = 0.0
        self.quit = threading.Event()
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.step)

    def on_key(self, key):
        if key == QUIT_KEY:
            self.quit.set()
            return
        with self._lock:
            if key in KEYS:
                sv, sw = KEYS[key]
                self._cmd = (sv * self.v, sw * self.w)
                self._key_time = time.monotonic()
            elif key == STOP_KEY:
                self._cmd = (0.0, 0.0)

    def step(self):
        with self._lock:
            v, w = self._cmd
            if time.monotonic() - self._key_time > self.key_hold:
                v, w = 0.0, 0.0
        self.send(v, w)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


def read_keys(node, stream):
    """Read single characters from the terminal until quit."""
    fd = stream.fileno()
    while not node.quit.is_set():
        ready, _, _ = select.select([fd], [], [], 0.1)
        if ready:
            ch = os.read(fd, 1).decode(errors='ignore').lower()
            if ch:
                node.on_key(ch)


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard teleop needs an interactive terminal (use ros2 run, not ros2 launch)')
        return
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardNode()
    print(HELP)

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        read_keys(node, sys.stdin)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.quit.set()
        executor.shutdown()
        spinner.join(timeout=1.0)
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
