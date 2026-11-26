#!/usr/bin/env python3
"""Post-run checks of a data-collection bag (F9, WP1).

    source /opt/ros/humble/setup.bash
    python3 tools/excitation/check_bag.py <bag_dir> --sequence <sequence.yaml>

Run it right after each recording and record again if it prints FAIL.
Checks:
  - every expected topic is present; rate and largest gap per topic (expected:
    /base_command 50 Hz, /cmd_vel 20 Hz, /gimbal/imu_a --imu-rate); header
    stamps in recording order never go backwards
  - IMU A publishes the accelerometer, not only the gyro
  - every segment of the sequence was played, in order, for its duration
  - no saturation: base_command did not scale the /cmd_vel requests down,
    and /debug/raw_control_command stays below 0.9 x the wheel-speed limit
  - gyro bias and noise in the first and the last rest (IMU A, Kobuki gyro)
  - <bag_dir>/meta.yaml exists and has all fields (README.md)
and ends with one line PASS or FAIL. Exit code 0 on PASS, 1 on FAIL.

Topic statistics use the time window in which the sequence was playing.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sysid'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'base_teleop'))
from base_teleop.sequence import load_sequence  # noqa: E402

TRACK_WIDTH = 0.23         # m, unverified (base_command.yaml)
WHEEL_SPEED_LIMIT = 0.7    # m/s, unverified (base_command.yaml)
WHEEL_MARGIN = 0.9

RATE_TOPICS = {'/base_command': 50.0, '/cmd_vel': 20.0}   # Hz; /gimbal/imu_a from --imu-rate
OTHER_TOPICS = ['/odom', '/sensors/imu_data', '/debug/raw_control_command']
ALL_TOPICS = ['/base_command', '/cmd_vel', '/gimbal/imu_a', '/excitation/segment'] + OTHER_TOPICS
META_FIELDS = ['terrain', 'date', 'battery_start', 'battery_end', 'gimbal_mounted', 'seed',
               'sequence_file', 'git_commit', 'imu_rate', 'operator']

RATE_TOLERANCE = 0.05      # relative
GAP_FACTOR = 3.0           # largest allowed gap in nominal periods, for RATE_TOPICS and IMU A
DURATION_TOLERANCE = 0.15  # s, played segment duration vs YAML (3 ticks of the 20 Hz excitation)
SCALED_PERSISTENCE = 5     # consecutive /base_command samples (0.1 s) before a scaling counts
CMD_MAX_AGE = 0.3          # s, older /cmd_vel requests are not compared (relay timeout is 0.5 s)


# ---------------------------------------------------------------- bag -> arrays

def extract(records):
    """{topic: [(t_rec, t_stamp, msg)]} -> {topic: {'t_rec', 't', fields...}} as numpy arrays."""
    out = {}
    for topic, rows in records.items():
        d = {'t_rec': np.array([r[0] for r in rows], dtype=float),
             't': np.array([r[1] for r in rows], dtype=float)}
        msgs = [r[2] for r in rows]
        if topic == '/base_command':
            d['v'] = np.array([m.twist.linear.x for m in msgs], dtype=float)
            d['w'] = np.array([m.twist.angular.z for m in msgs], dtype=float)
        elif topic == '/cmd_vel':
            d['v'] = np.array([m.linear.x for m in msgs], dtype=float)
            d['w'] = np.array([m.angular.z for m in msgs], dtype=float)
        elif topic == '/gimbal/imu_a':
            d['gyro'] = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                                  for m in msgs], dtype=float).reshape(-1, 3)
            d['accel_provided'] = np.array([m.linear_acceleration_covariance[0] != -1.0 for m in msgs])
        elif topic == '/sensors/imu_data':
            d['gyro_z'] = np.array([m.angular_velocity.z for m in msgs], dtype=float)
        elif topic == '/excitation/segment':
            d['index'] = np.array([m.data for m in msgs], dtype=int)
        elif topic == '/debug/raw_control_command':
            d['speed'] = np.array([m.data[0] if len(m.data) else 0 for m in msgs], dtype=float)  # mm/s
        out[topic] = d
    return out


# ---------------------------------------------------------------- checks on arrays

def topic_stats(t):
    """Rate [Hz], largest gap [s] and monotonicity of stamps given in recording order."""
    t = np.asarray(t, dtype=float)
    if t.size < 2:
        return {'n': int(t.size), 'rate': 0.0, 'max_gap': math.inf, 'monotonic': True}
    s = np.sort(t)
    span = s[-1] - s[0]
    return {'n': int(t.size), 'rate': (t.size - 1) / span if span > 0 else math.inf,
            'max_gap': float(np.max(np.diff(s))), 'monotonic': bool(np.all(np.diff(t) >= 0.0))}


def segment_windows(t_rec, index):
    """{segment index: (t_start, t_end)} from the /excitation/segment stream (-1 = idle)."""
    windows = {}
    current, start = None, None
    for t, i in zip(t_rec, index):
        if i != current:
            if current is not None and current >= 0 and current not in windows:
                windows[current] = (start, t)
            current, start = i, t
    return windows


def check_segments(t_rec, index, segments):
    played = [int(i) for i in index if i >= 0]
    order_ok = all(b >= a for a, b in zip(played, played[1:]))
    seen = sorted(set(played))
    missing = sorted(set(range(len(segments))) - set(seen))
    windows = segment_windows(t_rec, index)
    errors = [abs((e - s) - segments[i][0]) for i, (s, e) in windows.items() if i < len(segments)]
    worst = max(errors) if errors else math.inf
    ok = not missing and order_ok and len(windows) == len(segments) and worst <= DURATION_TOLERANCE
    text = (f'{len(seen)}/{len(segments)} segments played'
            + (f', missing {missing[:5]}{"..." if len(missing) > 5 else ""}' if missing else '')
            + ('' if order_ok else ', out of order')
            + f', worst duration error {worst:.3f} s')
    return ok, text, windows


def wheels(v, w, track_width):
    return np.stack([v - 0.5 * track_width * w, v + 0.5 * track_width * w], axis=-1)


def relay_scaling(cmd_t, cmd_v, cmd_w, base_t, base_v, base_w, track_width=TRACK_WIDTH):
    """Number of times base_command scaled a /cmd_vel request down for at least SCALED_PERSISTENCE samples.

    Times are recording times on the same machine. A sample counts as scaled
    when the executed command is s times the request with 0.05 < s < 0.99.
    """
    events, run = 0, 0
    c_all = wheels(np.asarray(cmd_v), np.asarray(cmd_w), track_width)
    b_all = wheels(np.asarray(base_v), np.asarray(base_w), track_width)
    for i, t in enumerate(base_t):
        j = int(np.searchsorted(cmd_t, t, side='right')) - 1
        scaled = False
        if j >= 0 and t - cmd_t[j] <= CMD_MAX_AGE:
            c, b = c_all[j], b_all[i]
            cc = float(c @ c)
            if cc > 1e-8:
                s = float(b @ c) / cc
                residual = np.linalg.norm(b - s * c) / math.sqrt(cc)
                scaled = residual < 0.01 and 0.05 < s < 0.99
        run = run + 1 if scaled else 0
        if run == SCALED_PERSISTENCE:
            events += 1
    return events


def rest_stats(t_rec, values, window, settle):
    """Mean and standard deviation of values (N, k) recorded inside a rest window."""
    start, end = window
    sel = (t_rec >= start + settle) & (t_rec <= end - 0.2)
    v = np.asarray(values, dtype=float)[sel]
    if v.ndim == 1:
        v = v[:, None]
    if len(v) < 2:
        return 0, None, None
    return len(v), v.mean(axis=0), v.std(axis=0)


def check_meta(bag_dir):
    path = os.path.join(bag_dir, 'meta.yaml')
    if not os.path.exists(path):
        return False, 'meta.yaml missing (write it, see README.md)'
    with open(path) as f:
        meta = yaml.safe_load(f) or {}
    missing = [k for k in META_FIELDS if meta.get(k) in (None, '')]
    if missing:
        return False, f'meta.yaml lacks {", ".join(missing)}'
    return True, f'terrain {meta["terrain"]}, battery {meta["battery_start"]} -> {meta["battery_end"]}'


def run_checks(data, segments, bag_dir, imu_rate=100.0, track_width=TRACK_WIDTH,
               wheel_speed_limit=WHEEL_SPEED_LIMIT, max_gyro_bias=0.02, max_gap=0.2, settle=1.0):
    """Return a list of (status, name, text), status in PASS / FAIL / INFO."""
    res = []
    missing = [t for t in ALL_TOPICS if t not in data or data[t]['t'].size == 0]
    res.append(('FAIL' if missing else 'PASS', 'topics',
                f'missing: {", ".join(missing)}' if missing else 'all present'))

    seg = data.get('/excitation/segment')
    if seg is None or not np.any(seg['index'] >= 0):
        res.append(('FAIL', '/excitation/segment', 'no segment was played'))
        return res
    active = seg['t_rec'][seg['index'] >= 0]
    t0, t1 = active[0], active[-1]
    ok, text, windows = check_segments(seg['t_rec'], seg['index'], segments)
    res.append(('PASS' if ok else 'FAIL', 'segments', text))

    expected = dict(RATE_TOPICS, **{'/gimbal/imu_a': imu_rate})
    for topic in ALL_TOPICS:
        if topic in missing or topic == '/excitation/segment':
            continue
        d = data[topic]
        sel = (d['t_rec'] >= t0) & (d['t_rec'] <= t1)
        st = topic_stats(d['t'][sel])
        # The topic must cover the whole run, e.g. a relay started late leaves the start uncovered.
        t_sel = d['t_rec'][sel]
        uncovered = max(t_sel[0] - t0, t1 - t_sel[-1]) if t_sel.size else t1 - t0
        text = f'{st["rate"]:.1f} Hz'
        if topic in expected:
            rate = expected[topic]
            limit = GAP_FACTOR / rate
            ok = abs(st['rate'] - rate) <= RATE_TOLERANCE * rate and st['max_gap'] <= limit
            text += f' (expected {rate:g})'
        else:
            limit = max_gap
            ok = st['max_gap'] <= limit
        ok = ok and st['monotonic'] and uncovered <= limit
        text += (f', max gap {st["max_gap"] * 1e3:.0f} ms' + ('' if st['monotonic'] else ', stamps go BACKWARDS')
                 + (f', missing for {uncovered:.2f} s at the start or end of the run' if uncovered > limit else ''))
        res.append(('PASS' if ok else 'FAIL', topic, text))

    if '/gimbal/imu_a' not in missing:
        provided = data['/gimbal/imu_a']['accel_provided']
        ok = bool(np.all(provided))
        res.append(('PASS' if ok else 'FAIL', 'IMU A accel',
                    'present' if ok else f'missing in {np.sum(~provided)} samples (gimbal.yaml imu_a accel: true)'))

    if '/cmd_vel' not in missing and '/base_command' not in missing:
        c, b = data['/cmd_vel'], data['/base_command']
        n = relay_scaling(c['t_rec'], c['v'], c['w'], b['t_rec'], b['v'], b['w'], track_width)
        res.append(('FAIL' if n else 'PASS', 'relay saturation',
                    f'{n} times base_command scaled the request down' if n else 'none'))
    if '/debug/raw_control_command' not in missing:
        speed = np.abs(data['/debug/raw_control_command']['speed']) / 1000.0
        limit = WHEEL_MARGIN * wheel_speed_limit
        over = int(np.sum(speed > limit + 0.005))
        res.append(('FAIL' if over else 'PASS', 'firmware speed',
                    f'max {speed.max():.3f} m/s, {over} samples above {limit:.3f} m/s'))

    first, last = 0, len(segments) - 1
    for name, i in (('first rest', first), ('last rest', last)):
        if i not in windows:
            res.append(('FAIL', name, 'not played'))
            continue
        if '/gimbal/imu_a' not in missing:
            imu = data['/gimbal/imu_a']
            n, mean, std = rest_stats(imu['t_rec'], imu['gyro'], windows[i], settle)
            if n < 10:
                res.append(('FAIL', f'{name} IMU A', f'only {n} samples'))
            else:
                ok = bool(np.all(np.abs(mean) <= max_gyro_bias))
                res.append(('PASS' if ok else 'FAIL', f'{name} IMU A gyro',
                            f'bias [{", ".join(f"{m:+.4f}" for m in mean)}] rad/s, '
                            f'std [{", ".join(f"{s:.4f}" for s in std)}] rad/s, {n} samples'
                            + ('' if ok else f' (bias above {max_gyro_bias}: calibrated?)')))
        if '/sensors/imu_data' not in missing:
            k = data['/sensors/imu_data']
            n, mean, std = rest_stats(k['t_rec'], k['gyro_z'], windows[i], settle)
            if n >= 2:
                res.append(('INFO', f'{name} Kobuki gyro', f'bias z {mean[0]:+.4f} rad/s, std {std[0]:.4f} rad/s'))

    ok, text = check_meta(bag_dir)
    res.append(('PASS' if ok else 'FAIL', 'meta', text))
    return res


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('bag', help='rosbag2 directory')
    p.add_argument('--sequence', required=True, help='the sequence YAML that was played')
    p.add_argument('--imu-rate', type=float, default=100.0,
                   help='IMU A rate [Hz] = 1 / gimbal.yaml report_interval (default %(default)s)')
    p.add_argument('--track-width', type=float, default=TRACK_WIDTH)
    p.add_argument('--wheel-speed-limit', type=float, default=WHEEL_SPEED_LIMIT)
    p.add_argument('--max-gyro-bias', type=float, default=0.02,
                   help='largest IMU A gyro mean at rest [rad/s] (default %(default)s)')
    p.add_argument('--max-gap', type=float, default=0.2,
                   help='largest gap [s] for /odom, /sensors/imu_data, raw command (default %(default)s)')
    p.add_argument('--settle', type=float, default=1.0, help='s skipped at the start of a rest (default %(default)s)')
    a = p.parse_args(argv)

    from bag_io import read_records   # needs a sourced ROS2 environment

    segments = load_sequence(a.sequence)
    data = extract(read_records(a.bag, ALL_TOPICS))
    res = run_checks(data, segments, a.bag, a.imu_rate, a.track_width, a.wheel_speed_limit,
                     a.max_gyro_bias, a.max_gap, a.settle)

    print(f'bag {a.bag}, sequence {a.sequence}')
    width = max(len(name) for _, name, _ in res)
    for status, name, text in res:
        print(f'[{status}] {name:<{width}}  {text}')
    failed = [name for status, name, _ in res if status == 'FAIL']
    print(f'FAIL: {", ".join(failed)}' if failed else 'PASS')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
