"""Tests of check_bag.py.

The checks run on synthetic arrays without ROS. test_script_on_synthetic_bag
writes a real bag with the real message types and runs the script on it; it
needs a sourced ROS2 environment and is skipped otherwise.
"""
import os
import subprocess
import sys

import numpy as np
import pytest
import yaml

import check_bag as cb

HERE = os.path.dirname(os.path.abspath(__file__))
SEGMENTS = [(2.0, 0.0, 0.0), (1.0, 0.1, 0.0), (0.5, 0.0, 0.5), (1.0, 0.05, -0.3), (2.0, 0.0, 0.0)]
BIAS = np.array([0.002, -0.001, 0.003])
META = {'terrain': 'floor', 'date': '2026-11-25', 'battery_start': 16.4, 'battery_end': 16.1,
        'gimbal_mounted': True, 'seed': 0, 'sequence_file': 'seq.yaml', 'git_commit': 'abc1234',
        'imu_rate': 100, 'imu_calibration': 'none', 'dynamic_calibration': False, 'operator': 'test'}


def synthetic(segments=SEGMENTS, imu_rate=100.0, t0=1000.0, scale=1.0, imu_gap=None, accel=True,
              robot_clock=0.0):
    """Arrays as extract() returns them, for a run that played `segments` perfectly.

    robot_clock: offset [s] of the robot Pi's clock (stamps of /base_command,
    /odom, /sensors/imu_data) from the clock of IMU A and the recorder.
    """
    rng = np.random.default_rng(0)
    total = sum(d for d, _, _ in segments)
    ends = np.cumsum([d for d, _, _ in segments])

    def command(t):
        i = np.minimum(np.searchsorted(ends, t, side='right'), len(segments) - 1)
        return np.array([segments[k][1] for k in i]), np.array([segments[k][2] for k in i])

    data = {}
    t_ex = np.arange(0.0, total + 0.5, 0.05)          # excitation at 20 Hz, idle after the end
    index = np.minimum(np.searchsorted(ends, t_ex + 1e-9, side='right'), len(segments))
    index[index >= len(segments)] = -1
    data['/excitation/segment'] = {'t_rec': t0 + t_ex, 't': t0 + t_ex, 'index': index}
    v, w = command(t_ex)
    v[index < 0] = 0.0
    w[index < 0] = 0.0
    data['/cmd_vel'] = {'t_rec': t0 + t_ex, 't': t0 + t_ex, 'v': v, 'w': w}

    t_bc = np.arange(-0.5, total + 1.0, 0.02)          # relay at 50 Hz, 3 ms behind
    v, w = command(t_bc - 0.003)
    v[(t_bc < 0) | (t_bc > total)] = 0.0
    w[(t_bc < 0) | (t_bc > total)] = 0.0
    data['/base_command'] = {'t_rec': t0 + t_bc + 0.004, 't': t0 + robot_clock + t_bc,  # robot clock
                             'v': scale * v, 'w': scale * w}
    speed = 1000 * (np.abs(scale * v) + np.abs(scale * w) * cb.TRACK_WIDTH / 2)
    data['/debug/raw_control_command'] = {'t_rec': t0 + t_bc + 0.004, 't': t0 + t_bc + 0.004, 'speed': speed}
    data['/odom'] = {'t_rec': t0 + t_bc + 0.005, 't': t0 + robot_clock + t_bc, 'v': v}
    data['/sensors/imu_data'] = {'t_rec': t0 + t_bc + 0.005, 't': t0 + robot_clock + t_bc,
                                 'gyro_z': w + 0.004 + rng.normal(0, 0.002, t_bc.shape)}

    t_imu = np.arange(-0.5, total + 1.0, 1.0 / imu_rate)
    if imu_gap is not None:
        t_imu = t_imu[(t_imu < imu_gap[0]) | (t_imu > imu_gap[1])]
    _, w_imu = command(t_imu)
    gyro = np.zeros((t_imu.size, 3)) + BIAS + rng.normal(0, 0.001, (t_imu.size, 3))
    gyro[:, 2] += w_imu
    data['/gimbal/imu_a'] = {'t_rec': t0 + t_imu + 0.001, 't': t0 + t_imu,
                             'gyro': gyro, 'accel_provided': np.full(t_imu.size, accel)}
    data['/gimbal/imu_a/raw'] = dict(data['/gimbal/imu_a'])
    return data


def statuses(res):
    return {name: status for status, name, _ in res}


@pytest.fixture
def bag_dir(tmp_path):
    (tmp_path / 'meta.yaml').write_text(yaml.safe_dump(META))
    return str(tmp_path)


def test_clean_run_passes(bag_dir):
    res = cb.run_checks(synthetic(), SEGMENTS, bag_dir)
    assert all(s != 'FAIL' for s, _, _ in res), res
    text = dict((name, t) for _, name, t in res)
    assert '5/5 segments' in text['segments']
    assert '+0.0020' in text['first rest IMU A gyro']


def test_imu_gap_fails(bag_dir):
    res = cb.run_checks(synthetic(imu_gap=(3.0, 3.4)), SEGMENTS, bag_dir)
    assert statuses(res)['/gimbal/imu_a'] == 'FAIL'


def test_wrong_imu_rate_fails(bag_dir):
    res = cb.run_checks(synthetic(imu_rate=200.0), SEGMENTS, bag_dir, imu_rate=100.0)
    assert statuses(res)['/gimbal/imu_a'] == 'FAIL'


def test_missing_accel_fails(bag_dir):
    assert statuses(cb.run_checks(synthetic(accel=False), SEGMENTS, bag_dir))['IMU A accel'] == 'FAIL'


def test_relay_scaling_fails(bag_dir):
    res = cb.run_checks(synthetic(scale=0.8), SEGMENTS, bag_dir)
    assert statuses(res)['relay saturation'] == 'FAIL'


def test_relay_started_late_fails(bag_dir):
    data = synthetic()
    b = data['/base_command']
    keep = b['t_rec'] > b['t_rec'][0] + 2.5                 # relay only from the middle of the run
    data['/base_command'] = {k: v[keep] for k, v in b.items()}
    assert statuses(cb.run_checks(data, SEGMENTS, bag_dir))['/base_command'] == 'FAIL'


def test_missing_topic_and_meta_fail(tmp_path):
    data = synthetic()
    del data['/odom']
    s = statuses(cb.run_checks(data, SEGMENTS, str(tmp_path)))
    assert s['topics'] == 'FAIL' and s['meta'] == 'FAIL'


def test_interrupted_run_fails(bag_dir):
    data = synthetic()
    seg = data['/excitation/segment']
    keep = seg['t_rec'] < seg['t_rec'][0] + 3.2        # stopped during segment 2
    data['/excitation/segment'] = {k: v[keep] for k, v in seg.items()}
    assert statuses(cb.run_checks(data, SEGMENTS, bag_dir))['segments'] == 'FAIL'


def test_large_bias_fails(bag_dir):
    data = synthetic()
    data['/gimbal/imu_a']['gyro'][:, 0] += 0.05
    assert statuses(cb.run_checks(data, SEGMENTS, bag_dir))['first rest IMU A gyro'] == 'FAIL'


def test_unsynced_clocks_fail(bag_dir):
    s = statuses(cb.run_checks(synthetic(robot_clock=500.0), SEGMENTS, bag_dir))
    assert s['clock sync'] == 'FAIL'
    assert statuses(cb.run_checks(synthetic(robot_clock=0.010), SEGMENTS, bag_dir))['clock sync'] == 'PASS'


def test_missing_raw_imu_fails(bag_dir):
    data = synthetic()
    del data['/gimbal/imu_a/raw']
    assert statuses(cb.run_checks(data, SEGMENTS, bag_dir))['topics'] == 'FAIL'


def test_calibration_file_must_be_in_bag(tmp_path):
    (tmp_path / 'meta.yaml').write_text(yaml.safe_dump(dict(META, imu_calibration='imu_a_calibration.yaml')))
    assert statuses(cb.run_checks(synthetic(), SEGMENTS, str(tmp_path)))['meta'] == 'FAIL'
    (tmp_path / 'imu_a_calibration.yaml').write_text('accel_bias: [0, 0, 0]\n')
    assert statuses(cb.run_checks(synthetic(), SEGMENTS, str(tmp_path)))['meta'] == 'PASS'


def test_topic_stats():
    st = cb.topic_stats([0.0, 0.1, 0.2, 0.5])
    assert st['rate'] == pytest.approx(6.0)
    assert st['max_gap'] == pytest.approx(0.3)
    assert st['monotonic']
    assert not cb.topic_stats([0.0, 0.2, 0.1])['monotonic']


def test_ramp_steps_are_not_scaling():
    # small command steps (as in the ramps) must not look like saturation
    t_cmd = np.arange(0, 5, 0.05)
    v = np.round(np.floor(t_cmd / 0.25) * 0.01, 3)
    t_base = np.arange(0, 5, 0.02) + 0.003
    j = np.searchsorted(t_cmd, t_base - 0.01, side='right') - 1   # relay lags by 10 ms
    assert cb.relay_scaling(t_cmd, v, 0 * v, t_base, v[np.maximum(j, 0)], 0 * t_base) == 0


# ---------------------------------------------------------------- end to end, needs ROS

def write_bag(path, data):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int16MultiArray, Int32

    def stamp(msg, t):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        return msg

    rows = []   # (t_rec, topic, msg)
    d = data['/base_command']
    for tr, t, v, w in zip(d['t_rec'], d['t'], d['v'], d['w']):
        m = stamp(TwistStamped(), t)
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        rows.append((tr, '/base_command', m))
    d = data['/cmd_vel']
    for tr, v, w in zip(d['t_rec'], d['v'], d['w']):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        rows.append((tr, '/cmd_vel', m))
    d = data['/excitation/segment']
    rows += [(tr, '/excitation/segment', Int32(data=int(i))) for tr, i in zip(d['t_rec'], d['index'])]
    d = data['/debug/raw_control_command']
    rows += [(tr, '/debug/raw_control_command', Int16MultiArray(data=[int(s), 0]))
             for tr, s in zip(d['t_rec'], d['speed'])]
    d = data['/odom']
    for tr, t, v in zip(d['t_rec'], d['t'], d['v']):
        m = stamp(Odometry(), t)
        m.twist.twist.linear.x = float(v)
        rows.append((tr, '/odom', m))
    d = data['/sensors/imu_data']
    for tr, t, g in zip(d['t_rec'], d['t'], d['gyro_z']):
        m = stamp(Imu(), t)
        m.angular_velocity.z = float(g)
        rows.append((tr, '/sensors/imu_data', m))
    for topic in cb.IMU_TOPICS:
        d = data[topic]
        for tr, t, g in zip(d['t_rec'], d['t'], d['gyro']):
            m = stamp(Imu(), t)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(x) for x in g)
            m.linear_acceleration.z = 9.81
            m.orientation_covariance[0] = -1.0
            rows.append((tr, topic, m))

    types = {'/base_command': 'geometry_msgs/msg/TwistStamped', '/cmd_vel': 'geometry_msgs/msg/Twist',
             '/excitation/segment': 'std_msgs/msg/Int32', '/debug/raw_control_command': 'std_msgs/msg/Int16MultiArray',
             '/odom': 'nav_msgs/msg/Odometry', '/sensors/imu_data': 'sensor_msgs/msg/Imu',
             '/gimbal/imu_a': 'sensor_msgs/msg/Imu', '/gimbal/imu_a/raw': 'sensor_msgs/msg/Imu'}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    for tr, topic, m in sorted(rows, key=lambda r: r[0]):
        writer.write(topic, serialize_message(m), int(round(tr * 1e9)))
    del writer


@pytest.mark.parametrize('gap,code', [(None, 0), ((3.0, 3.4), 1)])
def test_script_on_synthetic_bag(tmp_path, gap, code):
    bag = str(tmp_path / 'bag')
    write_bag(bag, synthetic(imu_gap=gap))
    with open(os.path.join(bag, 'meta.yaml'), 'w') as f:
        yaml.safe_dump(META, f)
    seq = tmp_path / 'seq.yaml'
    seq.write_text(yaml.safe_dump({'segments': [{'duration': d, 'v': v, 'w': w} for d, v, w in SEGMENTS]}))
    r = subprocess.run([sys.executable, os.path.join(HERE, 'check_bag.py'), bag, '--sequence', str(seq)],
                       capture_output=True, text=True)
    print(r.stdout, r.stderr)
    assert r.returncode == code
    assert r.stdout.strip().splitlines()[-1] == ('PASS' if code == 0 else 'FAIL: /gimbal/imu_a, /gimbal/imu_a/raw')
