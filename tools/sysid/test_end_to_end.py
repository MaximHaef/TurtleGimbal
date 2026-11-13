"""End-to-end: write a synthetic bag like the robot Pi records, run the script on it.

Needs a sourced ROS2 environment; skipped otherwise.
"""
import json
import os
import subprocess
import sys

import numpy as np
import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')

from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from fopdt import simulate  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TAU_W, DELAY_W, TAU_V, DELAY_V = 0.12, 0.05, 0.25, 0.04
BIAS = 0.01  # rad/s


def stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    return msg


def write_bag(path):
    rate, hold_s, t0 = 50.0, 2.5, 1000.0
    # (v, w) segments: rest, yaw steps, forward steps
    seq = [(0, 0), (0, 0.5), (0, 0), (0, -0.5), (0, 0), (0, 1.0), (0, 0),
           (0.1, 0), (0, 0), (-0.1, 0), (0, 0)]
    t_cmd = np.arange(0.0, hold_s * len(seq), 1.0 / rate)
    k = np.minimum((t_cmd / hold_s).astype(int), len(seq) - 1)
    v_cmd = np.array([seq[i][0] for i in k], dtype=float)
    w_cmd = np.array([seq[i][1] for i in k], dtype=float)

    rng = np.random.default_rng(1)
    t_meas = t_cmd + 0.007  # sensors run at the same rate, slightly offset
    wz = simulate(t_cmd, w_cmd, t_meas, TAU_W, DELAY_W, 1.0) + BIAS + rng.normal(0, 0.01, t_meas.shape)
    vx = simulate(t_cmd, v_cmd, t_meas, TAU_V, DELAY_V, 1.0) + rng.normal(0, 0.002, t_meas.shape)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in (('/base_command', 'geometry_msgs/msg/TwistStamped'),
                      ('/sensors/imu_data', 'sensor_msgs/msg/Imu'),
                      ('/odom', 'nav_msgs/msg/Odometry')):
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    for i, t in enumerate(t_cmd):
        c = stamp(TwistStamped(), t0 + t)
        c.twist.linear.x, c.twist.angular.z = v_cmd[i], w_cmd[i]
        writer.write('/base_command', serialize_message(c), int((t0 + t + 0.001) * 1e9))
        g = stamp(Imu(), t0 + t_meas[i])
        g.angular_velocity.z = wz[i]
        writer.write('/sensors/imu_data', serialize_message(g), int((t0 + t_meas[i] + 0.001) * 1e9))
        o = stamp(Odometry(), t0 + t_meas[i])
        o.twist.twist.linear.x = vx[i]
        writer.write('/odom', serialize_message(o), int((t0 + t_meas[i] + 0.001) * 1e9))
    del writer


def test_script_on_synthetic_bag(tmp_path):
    bag = str(tmp_path / 'bag')
    write_bag(bag)
    out = tmp_path / 'res.json'
    png = tmp_path / 'res.png'
    subprocess.run([sys.executable, os.path.join(HERE, 'fit_base_dynamics.py'), bag,
                    '--json', str(out), '--plot', str(png)], check=True, cwd=HERE)
    res = json.loads(out.read_text())
    assert res['yaw']['gyro_bias'] == pytest.approx(BIAS, abs=0.003)
    assert res['yaw']['overall']['tau'] == pytest.approx(TAU_W, abs=0.01)
    assert res['yaw']['overall']['delay'] == pytest.approx(DELAY_W, abs=0.01)
    assert res['forward']['overall']['tau'] == pytest.approx(TAU_V, abs=0.02)
    assert res['forward']['overall']['delay'] == pytest.approx(DELAY_V, abs=0.01)
    assert len(res['yaw']['steps']) == 6
    assert png.stat().st_size > 0
