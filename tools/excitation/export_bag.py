#!/usr/bin/env python3
"""Export a data-collection bag to CSV files and overview plots. Needs no ROS.

    pip install rosbags matplotlib
    python tools/excitation/export_bag.py bags/floor_20260314_0246 --sequence excitation_box07_s1.yaml

Writes into <bag>/export/ (or --out):
  <topic>.csv     one file per topic, e.g. odom.csv, base_command.csv
  overview.png    commanded vs. measured forward speed and yaw rate over time,
                  segment index, bumper presses; blocks of the sequence shaded
  path.png        path of the robot from wheel odometry, with the free area

Time column `t` is seconds since the first segment of the sequence started
(recording time of the bag); `t_stamp` is the message header stamp in seconds
where the message has one.
"""
import argparse
import csv
import math
import os
import re
from pathlib import Path

import numpy as np

# Custom Kobuki messages are not in the standard type store (kobuki_ros_interfaces).
KOBUKI_MSGS = {
    'kobuki_ros_interfaces/msg/BumperEvent': 'uint8 LEFT=0\nuint8 CENTER=1\nuint8 RIGHT=2\nuint8 RELEASED=0\nuint8 PRESSED=1\nuint8 bumper\nuint8 state\n',
    'kobuki_ros_interfaces/msg/ButtonEvent': 'uint8 BUTTON0=0\nuint8 BUTTON1=1\nuint8 BUTTON2=2\nuint8 RELEASED=0\nuint8 PRESSED=1\nuint8 button\nuint8 state\n',
    'kobuki_ros_interfaces/msg/CliffEvent': 'uint8 LEFT=0\nuint8 CENTER=1\nuint8 RIGHT=2\nuint8 FLOOR=0\nuint8 CLIFF=1\nuint8 sensor\nuint8 state\nuint16 bottom\n',
    'kobuki_ros_interfaces/msg/WheelDropEvent': 'uint8 LEFT=0\nuint8 RIGHT=1\nuint8 RAISED=0\nuint8 DROPPED=1\nuint8 wheel\nuint8 state\n',
}


def stamp(msg):
    h = getattr(msg, 'header', None)
    return h.stamp.sec + h.stamp.nanosec * 1e-9 if h is not None else None


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def row(msgtype, msg):
    """(column names, values) for the message types of the data-collection bags, or None."""
    if msgtype == 'geometry_msgs/msg/Twist':
        return ['v', 'w'], [msg.linear.x, msg.angular.z]
    if msgtype == 'geometry_msgs/msg/TwistStamped':
        return ['v', 'w'], [msg.twist.linear.x, msg.twist.angular.z]
    if msgtype == 'nav_msgs/msg/Odometry':
        p, tw = msg.pose.pose, msg.twist.twist
        return ['x', 'y', 'yaw', 'v', 'w'], [p.position.x, p.position.y, yaw(p.orientation), tw.linear.x, tw.angular.z]
    if msgtype == 'sensor_msgs/msg/Imu':
        g, a = msg.angular_velocity, msg.linear_acceleration
        return (['gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z', 'yaw'],
                [g.x, g.y, g.z, a.x, a.y, a.z, yaw(msg.orientation)])
    if msgtype == 'sensor_msgs/msg/JointState':
        cols, vals = [], []
        for i, name in enumerate(msg.name):
            cols += [f'{name}_pos', f'{name}_vel']
            vals += [msg.position[i] if i < len(msg.position) else math.nan,
                     msg.velocity[i] if i < len(msg.velocity) else math.nan]
        return cols, vals
    if msgtype == 'std_msgs/msg/Int32':
        return ['data'], [msg.data]
    if msgtype == 'std_msgs/msg/Int16MultiArray':
        d = list(msg.data)
        return ['speed_mm_s', 'radius_mm'], [d[0] if d else 0, d[1] if len(d) > 1 else 0]
    if msgtype.startswith('kobuki_ros_interfaces/msg/'):
        fields = [f for f in ('bumper', 'button', 'sensor', 'wheel', 'state') if hasattr(msg, f)]
        return fields, [getattr(msg, f) for f in fields]
    return None


def read_bag(bag):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name, text in KOBUKI_MSGS.items():
        types.update(get_types_from_msg(text, name))
    store.register(types)

    data, skipped = {}, set()
    with AnyReader([Path(bag)], default_typestore=store) as reader:
        for conn, t_rec, raw in reader.messages(connections=reader.connections):
            msg = reader.deserialize(raw, conn.msgtype)
            r = row(conn.msgtype, msg)
            if r is None:
                skipped.add(f'{conn.topic} ({conn.msgtype})')
                continue
            cols, vals = r
            d = data.setdefault(conn.topic, {'cols': cols, 'rows': []})
            d['rows'].append([t_rec * 1e-9, stamp(msg)] + vals)
    return data, skipped


def write_csv(data, out, t0):
    for topic, d in data.items():
        path = out / (topic.strip('/').replace('/', '_') + '.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 't_stamp'] + d['cols'])
            for r in d['rows']:
                w.writerow([f'{r[0] - t0:.4f}', '' if r[1] is None else f'{r[1]:.6f}'] + r[2:])
        print(f'{path}  ({len(d["rows"])} rows)')


def blocks_from_sequence(path):
    """[(first segment index, label)] from the '# round 1: ...' comments of a make_sequence.py file."""
    blocks, index = [], 0
    with open(path) as f:
        for line in f:
            s = line.strip()
            m = re.match(r'#\s*(round \d+: .*|end)$', s)
            if m and line.startswith('  '):
                blocks.append((index, m.group(1)))
            elif s.startswith('- {'):
                index += 1
    return blocks


def arrays(data, topic, col):
    d = data.get(topic)
    if d is None:
        return np.array([]), np.array([])
    i = d['cols'].index(col) + 2
    return np.array([r[0] for r in d['rows']]), np.array([r[i] for r in d['rows']], dtype=float)


def plot(data, out, t0, blocks, box):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ts, seg = arrays(data, '/excitation/segment', 'data')
    starts = {}
    for t, i in zip(ts, seg):
        if i >= 0 and int(i) not in starts:
            starts[int(i)] = t - t0
    t_end = (ts[seg >= 0][-1] - t0) if np.any(seg >= 0) else None

    fig, ax = plt.subplots(3, 1, figsize=(16, 10), sharex=True, gridspec_kw={'height_ratios': [3, 3, 1.4]})
    colors = plt.cm.Pastel1.colors
    spans = []
    for k, (first, label) in enumerate(blocks):
        if first not in starts:
            continue
        nxt = [starts[f] for f, _ in blocks[k + 1:] if f in starts]
        end = nxt[0] if nxt else t_end
        if end is None:
            continue
        spans.append((starts[first], end, label.split(': ', 1)[-1]))
    for k, (a, b, label) in enumerate(spans):
        for axis in ax:
            axis.axvspan(a, b, color=colors[k % len(colors)], alpha=0.5, lw=0)
        ax[0].text((a + b) / 2, 1.0, label, transform=ax[0].get_xaxis_transform(),
                   ha='center', va='bottom', fontsize=8, rotation=0)

    t, v = arrays(data, '/base_command', 'v')
    ax[0].plot(t - t0, v, color='black', lw=1.0, label='command (/base_command)')
    t, v = arrays(data, '/odom', 'v')
    ax[0].plot(t - t0, v, color='tab:blue', lw=0.8, label='measured, wheels (/odom)')
    ax[0].set_ylabel('forward speed v [m/s]')
    ax[0].legend(loc='lower left', fontsize=8)

    t, w = arrays(data, '/base_command', 'w')
    ax[1].plot(t - t0, w, color='black', lw=1.0, label='command (/base_command)')
    t, w = arrays(data, '/sensors/imu_data', 'gyro_z')
    ax[1].plot(t - t0, w, color='tab:red', lw=0.6, label='measured, Kobuki gyro (/sensors/imu_data)')
    ax[1].set_ylabel('yaw rate w [rad/s]')
    ax[1].legend(loc='lower left', fontsize=8)

    ax[2].step(ts - t0, seg, where='post', color='tab:gray', lw=0.8, label='segment index')
    t, state = arrays(data, '/events/bumper', 'state')
    pressed = t[state == 1] - t0
    if pressed.size:
        ax[2].plot(pressed, np.full(pressed.size, max(seg.max(), 1) * 0.5), 'v', color='tab:orange',
                   label=f'bumper pressed ({pressed.size})')
    ax[2].set_ylabel('segment')
    ax[2].set_xlabel('time since start of the sequence [s]')
    ax[2].legend(loc='upper left', fontsize=8)
    for axis in ax:
        axis.grid(alpha=0.3)
        axis.axhline(0, color='k', lw=0.4)
    fig.tight_layout()
    fig.savefig(out / 'overview.png', dpi=130)
    print(out / 'overview.png')

    # path from wheel odometry, rotated so that the start heading points along +x
    t, x = arrays(data, '/odom', 'x')
    _, y = arrays(data, '/odom', 'y')
    _, th = arrays(data, '/odom', 'yaw')
    if x.size:
        c, s = math.cos(-th[0]), math.sin(-th[0])
        dx, dy = x - x[0], y - y[0]
        px, py = c * dx - s * dy, s * dx + c * dy
        fig, a = plt.subplots(figsize=(7, 7))
        sc = a.scatter(px, py, c=t - t0, s=2, cmap='viridis')
        fig.colorbar(sc, label='time [s]')
        a.plot(px[0], py[0], 'go', ms=9, label='start')
        a.plot(px[-1], py[-1], 'rs', ms=9, label=f'end ({math.hypot(px[-1], py[-1]):.2f} m from start)')
        a.annotate('', xy=(0.15, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='g'))
        if box:
            a.add_patch(plt.Rectangle((-box, -box), 2 * box, 2 * box, fill=False, ls='--', color='tab:orange',
                                      label=f'--box {box} m (commanded path limit)'))
        a.add_patch(plt.Rectangle((-1, -1), 2, 2, fill=False, color='tab:red', label='free area 1 m'))
        a.set_aspect('equal')
        a.set_xlim(-1.2, 1.2)
        a.set_ylim(-1.2, 1.2)
        a.set_xlabel('x [m] (start heading)')
        a.set_ylabel('y [m] (left)')
        a.set_title('Robot path from wheel odometry')
        a.grid(alpha=0.3)
        a.legend(loc='upper left', fontsize=8)
        fig.tight_layout()
        fig.savefig(out / 'path.png', dpi=130)
        print(out / 'path.png')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('bag', help='rosbag2 directory')
    p.add_argument('--sequence', help='the sequence YAML that was played (labels the blocks in the plot)')
    p.add_argument('--box', type=float, help='--box used for the sequence, drawn in path.png')
    p.add_argument('--out', help='output directory (default <bag>/export)')
    p.add_argument('--no-plot', action='store_true')
    a = p.parse_args(argv)

    out = Path(a.out) if a.out else Path(a.bag) / 'export'
    out.mkdir(parents=True, exist_ok=True)
    data, skipped = read_bag(a.bag)
    for s in sorted(skipped):
        print(f'skipped {s}')

    seg = data.get('/excitation/segment')
    active = [r[0] for r in seg['rows'] if r[2] >= 0] if seg else []
    t0 = active[0] if active else min(r[0] for d in data.values() for r in d['rows'])

    write_csv(data, out, t0)
    if not a.no_plot:
        box = a.box
        blocks = []
        if a.sequence:
            blocks = blocks_from_sequence(a.sequence)
            if box is None:
                m = re.search(r'--box (\S+)', open(a.sequence).read())
                box = float(m.group(1)) if m else None
        plot(data, out, t0, blocks, box)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
