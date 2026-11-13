#!/usr/bin/env python3
"""Identify the base time constants and dead time from a step-response bag.

    source /opt/ros/humble/setup.bash
    python3 tools/sysid/fit_base_dynamics.py <bag_dir> [--plot out.png] [--json out.json]

Yaw:     /base_command twist.angular.z  ->  Kobuki gyro /sensors/imu_data angular_velocity.z
Forward: /base_command twist.linear.x   ->  wheel odometry /odom twist.twist.linear.x

For each channel it fits tau * dy/dt = K * u(t - T_d) - y to the whole run, and
again to each step separately (the spread over steps shows how well the
first-order model holds). All stamps come from the robot Pi, so the two Pis'
clock offset does not matter. See README.md in this folder.
"""
import argparse
import json
import sys

import numpy as np

from fopdt import fit, find_steps, hold

REST_SETTLE = 1.0     # s, command must have been zero this long before a sample counts as rest
STEP_WINDOW = 2.5     # s, length of the per-step fit window after a step
STEP_PRE = 0.3        # s, included before each step
MIN_STEP_SAMPLES = 20


def load(bag_dir, cmd_topic, gyro_topic, odom_topic):
    from bag_io import read_topics
    data = read_topics(bag_dir, [cmd_topic, gyro_topic, odom_topic])
    if not data[cmd_topic]:
        sys.exit(f'no messages on {cmd_topic} in {bag_dir}')
    cmd = np.array([(t, m.twist.linear.x, m.twist.angular.z) for t, m in data[cmd_topic]])
    gyro = np.array([(t, m.angular_velocity.z) for t, m in data[gyro_topic]]).reshape(-1, 2)
    odom = np.array([(t, m.twist.twist.linear.x) for t, m in data[odom_topic]]).reshape(-1, 2)
    return cmd, gyro, odom


def rest_mask(t, t_cmd, v_cmd, w_cmd):
    """Samples taken while the command is zero and has been zero for REST_SETTLE s."""
    zero = (np.abs(v_cmd) < 1e-6) & (np.abs(w_cmd) < 1e-6)
    changes = t_cmd[1:][(np.diff(v_cmd) != 0) | (np.diff(w_cmd) != 0)]
    idx = np.searchsorted(changes, t, side='right') - 1
    last_change = np.where(idx >= 0, changes[np.clip(idx, 0, None)], -np.inf)
    return hold(t_cmd, zero, t).astype(bool) & (t - last_change >= REST_SETTLE)


def analyse_channel(name, t_cmd, u_cmd, t_meas, y_meas):
    """Global and per-step FOPDT fits for one channel. Returns a result dict or None."""
    if len(t_meas) < MIN_STEP_SAMPLES or np.ptp(u_cmd) < 1e-3:
        print(f'{name}: no excitation or no data, skipped')
        return None

    overall = fit(t_cmd, u_cmd, t_meas, y_meas)
    steps = []
    events = find_steps(t_cmd, u_cmd)
    for i, (ts, u0, u1) in enumerate(events):
        t_end = ts + STEP_WINDOW
        if i + 1 < len(events):
            t_end = min(t_end, events[i + 1][0])
        sel = (t_meas >= ts - STEP_PRE) & (t_meas <= t_end)
        if sel.sum() < MIN_STEP_SAMPLES:
            continue
        pre = (t_meas >= ts - STEP_PRE) & (t_meas < ts)
        y0 = float(np.mean(y_meas[pre])) if pre.any() else float(y_meas[sel][0])
        r = fit(t_cmd, u_cmd, t_meas[sel], y_meas[sel], y0=y0)
        steps.append({'t': ts, 'from': u0, 'to': u1, **r.__dict__})

    def stats(key):
        vals = np.array([s[key] for s in steps])
        if len(vals) == 0:
            return None
        return {'median': float(np.median(vals)), 'p25': float(np.percentile(vals, 25)),
                'p75': float(np.percentile(vals, 75)), 'min': float(vals.min()), 'max': float(vals.max())}

    return {'overall': overall.__dict__, 'steps': steps,
            'per_step': {k: stats(k) for k in ('tau', 'delay', 'gain')}}


def report(name, unit, res):
    if res is None:
        return
    o = res['overall']
    print(f'\n=== {name} ===')
    print(f'whole run:  tau = {o["tau"]*1e3:6.1f} ms   T_d = {o["delay"]*1e3:6.1f} ms   '
          f'K = {o["gain"]:.3f}   RMSE = {o["rmse"]:.4f} {unit}   R² = {o["r2"]:.3f}   n = {o["n"]}')
    if res['steps']:
        print(f'{"step at":>9} {"from":>7} {"to":>7} {"tau ms":>7} {"T_d ms":>7} {"K":>6} {"R²":>6}')
        for s in res['steps']:
            print(f'{s["t"] - res["t0"]:9.2f} {s["from"]:7.3f} {s["to"]:7.3f} {s["tau"]*1e3:7.1f} '
                  f'{s["delay"]*1e3:7.1f} {s["gain"]:6.3f} {s["r2"]:6.3f}')
        p = res['per_step']
        print(f'per step (median [p25, p75]):  tau = {p["tau"]["median"]*1e3:.1f} '
              f'[{p["tau"]["p25"]*1e3:.1f}, {p["tau"]["p75"]*1e3:.1f}] ms   '
              f'T_d = {p["delay"]["median"]*1e3:.1f} [{p["delay"]["p25"]*1e3:.1f}, {p["delay"]["p75"]*1e3:.1f}] ms   '
              f'K = {p["gain"]["median"]:.3f}')


def plot(path, t0, channels):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from fopdt import simulate

    channels = [c for c in channels if c['res'] is not None]
    fig, axes = plt.subplots(len(channels), 2, figsize=(14, 4 * len(channels)), squeeze=False,
                             gridspec_kw={'width_ratios': [2, 1]})
    for row, c in zip(axes, channels):
        o = c['res']['overall']
        model = simulate(c['t_cmd'], c['u_cmd'], c['t_meas'], o['tau'], o['delay'], o['gain'], c['y_meas'][0])
        ax = row[0]
        ax.step(c['t_cmd'] - t0, c['u_cmd'], where='post', label='command (/base_command)', color='0.5')
        ax.plot(c['t_meas'] - t0, c['y_meas'], '.', ms=2, label=f'measured ({c["source"]})')
        ax.plot(c['t_meas'] - t0, model, label=f'fit: tau={o["tau"]*1e3:.0f} ms, T_d={o["delay"]*1e3:.0f} ms, K={o["gain"]:.2f}')
        ax.set(title=c['name'], xlabel='time [s]', ylabel=c['unit'])
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

        # Step responses on top of each other, each normalized by its own measured
        # change (so the shape is visible independent of the gain). Starts (|u| grows)
        # and stops (|u| shrinks) in different colors.
        ax = row[1]
        labelled = set()
        for ts, u0, u1 in find_steps(c['t_cmd'], c['u_cmd']):
            t, y = c['t_meas'], c['y_meas']
            sel = (t >= ts - 0.2) & (t <= ts + 1.5)
            pre = (t >= ts - STEP_PRE) & (t < ts)
            post = (t >= ts + 1.0) & (t <= ts + 1.5)
            if sel.sum() < 5 or not pre.any() or not post.any():
                continue
            y_before, y_after = np.mean(y[pre]), np.mean(y[post])
            if abs(y_after - y_before) < 1e-3:
                continue
            kind = 'start' if abs(u1) > abs(u0) else 'stop'
            ax.plot(t[sel] - ts, (y[sel] - y_before) / (y_after - y_before), '.-', ms=3, lw=0.5, alpha=0.6,
                    color='tab:blue' if kind == 'start' else 'tab:orange',
                    label=None if kind in labelled else f'{kind}s')
            labelled.add(kind)
        tt = np.linspace(-0.2, 1.5, 400)
        ax.plot(tt, np.where(tt > o['delay'], 1 - np.exp(-(tt - o['delay']) / o['tau']), 0.0),
                'k', lw=2, label='fit (whole run)')
        ax.set(title=f'{c["name"]}: all steps, normalized', xlabel='time since command step [s]',
               ylabel='Δy / measured Δy')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f'\nplot written to {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--cmd-topic', default='/base_command')
    ap.add_argument('--gyro-topic', default='/sensors/imu_data')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--plot', help='write a PNG with data, fits and overlaid steps')
    ap.add_argument('--json', help='write all results as JSON')
    args = ap.parse_args()

    cmd, gyro, odom = load(args.bag, args.cmd_topic, args.gyro_topic, args.odom_topic)
    t_cmd, v_cmd, w_cmd = cmd.T
    t0 = t_cmd[0]

    channels = []
    results = {'bag': args.bag}

    if len(gyro):
        t_g, wz = gyro.T
        keep = (t_g >= t_cmd[0]) & (t_g <= t_cmd[-1])
        t_g, wz = t_g[keep], wz[keep]
        rest = rest_mask(t_g, t_cmd, v_cmd, w_cmd)
        bias = float(np.mean(wz[rest])) if rest.any() else 0.0
        print(f'gyro: {len(t_g)} samples, {np.median(np.diff(t_g))*1e3:.1f} ms median spacing, '
              f'bias {bias*1e3:.2f} mrad/s from {int(rest.sum())} rest samples'
              + ('' if rest.any() else ' (no rest found, bias not removed)'))
        res = analyse_channel('yaw rate', t_cmd, w_cmd, t_g, wz - bias)
        if res is not None:
            res['t0'] = t0
            res['gyro_bias'] = bias
        results['yaw'] = res
        channels.append({'name': 'yaw rate ω_z', 'unit': 'rad/s', 'source': args.gyro_topic, 'res': res,
                         't_cmd': t_cmd, 'u_cmd': w_cmd, 't_meas': t_g, 'y_meas': wz - bias})
    else:
        print(f'no messages on {args.gyro_topic}')

    if len(odom):
        t_o, vx = odom.T
        keep = (t_o >= t_cmd[0]) & (t_o <= t_cmd[-1])
        res = analyse_channel('forward speed', t_cmd, v_cmd, t_o[keep], vx[keep])
        if res is not None:
            res['t0'] = t0
        results['forward'] = res
        channels.append({'name': 'forward speed v', 'unit': 'm/s', 'source': args.odom_topic, 'res': res,
                         't_cmd': t_cmd, 'u_cmd': v_cmd, 't_meas': t_o[keep], 'y_meas': vx[keep]})
    else:
        print(f'no messages on {args.odom_topic}')

    report('yaw rate (gyro): tau_omega', 'rad/s', results.get('yaw'))
    report('forward speed (odometry): tau_v', 'm/s', results.get('forward'))

    if args.plot and channels:
        plot(args.plot, t0, channels)
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results, f, indent=2)
        print(f'results written to {args.json}')


if __name__ == '__main__':
    main()
