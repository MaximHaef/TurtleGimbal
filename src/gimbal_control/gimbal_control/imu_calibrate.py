"""imu_calibrate: interactive static calibration of IMU A (see imu_calibration.py).

Run on the gimbal Pi with no other process reading IMU A (no imu_node, no
gimbal_control). Hold the IMU still in many orientations; the script records
a window in each, checks it is still, and at the end fits the accelerometer
matrix + bias and the gyro bias.

    ros2 run gimbal_control imu_calibrate                       # capture + fit
    ros2 run gimbal_control imu_calibrate --refit DIR/raw.npz   # fit again, no sensor

Writes DIR/raw.npz (every window, for refitting) and DIR/imu_a_calibration.yaml.
Does not need a ROS graph; it only uses the package's Python modules.
"""
import argparse
import os
import threading
import time

import numpy as np

from gimbal_control.imu_calibration import (
    STANDARD_GRAVITY, Window, calibrate, coverage, is_still, min_angle_deg, window_stats)

HELP = """
Hold IMU A still in a new orientation, then press Enter. Aim for 12-20 poses
spread over all directions: each axis up and down (6), plus tilted poses in
between (edges and corners of a cube). Resting it on a surface is best.
Commands:  Enter = record pose   u = undo last   f = finish and fit   q = quit
"""


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bus', type=int, default=3, help='I2C bus (default 3, as imu_a in gimbal.yaml)')
    p.add_argument('--address', type=lambda s: int(s, 0), default=0x4B)
    p.add_argument('--report-interval', type=float, default=0.01, help='s (default 0.01)')
    p.add_argument('--settle', type=float, default=1.0, help='s to wait after Enter before recording')
    p.add_argument('--duration', type=float, default=3.0, help='s recorded per pose')
    p.add_argument('--max-accel-std', type=float, default=0.05, help='m/s^2, stillness threshold per axis')
    p.add_argument('--max-gyro-std', type=float, default=0.01, help='rad/s, stillness threshold per axis')
    p.add_argument('--min-angle', type=float, default=15.0,
                   help='deg, reject a pose closer than this to an earlier one (0 = off)')
    p.add_argument('--gravity', type=float, default=STANDARD_GRAVITY,
                   help='m/s^2, local gravity (default standard gravity)')
    p.add_argument('--out', default=None, help='output directory (default ./imu_a_calib_<time>)')
    p.add_argument('--dynamic-calibration', action='store_true',
                   help="leave the BNO08x's own running calibration on (default off, as imu_node)")
    p.add_argument('--refit', metavar='RAW_NPZ', default=None,
                   help='fit an earlier raw.npz again instead of capturing')
    return p.parse_args(argv)


# --- raw data file --------------------------------------------------------------

def save_raw(path, windows, samples):
    idx = np.concatenate([np.full(len(a), i) for i, (a, _) in enumerate(samples)]) if samples else np.zeros(0)
    np.savez(path,
             accel_mean=np.array([w.accel_mean for w in windows]),
             gyro_mean=np.array([w.gyro_mean for w in windows]),
             accel_std=np.array([w.accel_std for w in windows]),
             gyro_std=np.array([w.gyro_std for w in windows]),
             n=np.array([w.n for w in windows]),
             samples_accel=np.concatenate([a for a, _ in samples]) if samples else np.zeros((0, 3)),
             samples_gyro=np.concatenate([g for _, g in samples]) if samples else np.zeros((0, 3)),
             samples_window=idx)


def load_raw(path):
    d = np.load(path)
    return [Window(d['accel_mean'][i], d['gyro_mean'][i], d['accel_std'][i], d['gyro_std'][i], int(d['n'][i]))
            for i in range(len(d['n']))]


# --- report ---------------------------------------------------------------------

def report(cal):
    i = cal.info
    np.set_printoptions(precision=5, suppress=True)
    print('\n=== IMU A calibration ===')
    print(f"poses: {i['poses']}   coverage: {i['coverage']:.3f} (1/3 = even spread, near 0 = an axis not excited)")
    print(f"| |a| - g | rms: {i['norm_error_rms_raw']:.4f} m/s^2 raw  ->  {i['norm_error_rms']:.4f} m/s^2 calibrated"
          f"   (max {i['norm_error_max']:.4f})")
    print('accel_matrix K =\n', cal.accel_matrix)
    print('accel_bias b_a =', cal.accel_bias, 'm/s^2')
    if 'accel_bias_std' in i:
        print('  1-sigma:       ', np.array(i['accel_bias_std']))
    print('gyro_bias  b_g =', cal.gyro_bias, 'rad/s')
    print('  spread over poses:', np.array(i['gyro_bias_spread']), 'rad/s')


def fit_and_save(windows, out_dir, gravity, meta):
    cal = calibrate(windows, gravity)
    cal.info = {**meta, **cal.info}
    path = os.path.join(out_dir, 'imu_a_calibration.yaml')
    cal.save(path)
    report(cal)
    print(f'\nwrote {path}')
    return cal


# --- capture --------------------------------------------------------------------

class Collector:
    """on_sample callback for ImuReader; keeps samples only while recording."""

    def __init__(self):
        self.lock = threading.Lock()
        self.recording = False
        self.accel, self.gyro = [], []

    def __call__(self, stamp, sample):
        if sample.accel is None:
            return
        with self.lock:
            if self.recording:
                self.accel.append(sample.accel)
                self.gyro.append(sample.gyro)

    def record(self, duration):
        with self.lock:
            self.accel, self.gyro = [], []
            self.recording = True
        time.sleep(duration)
        with self.lock:
            self.recording = False
            return np.array(self.accel), np.array(self.gyro)


def capture(args, out_dir):
    from gimbal_control.imu import ImuReader, open_bno08x

    poll = open_bno08x(args.bus, args.address, args.report_interval, accel=True,
                       dynamic_calibration=args.dynamic_calibration)
    if poll.dynamic_calibration_ack is False:
        print('warning: sensor did not acknowledge turning off dynamic calibration')
    col = Collector()
    reader = ImuReader(poll, on_sample=col)
    reader.start()
    try:
        t0 = time.monotonic()
        while reader.latest() is None or reader.latest()[1].accel is None:
            if time.monotonic() - t0 > 3.0:
                raise RuntimeError(f'no IMU samples after 3 s (last error: {reader.last_error!r})')
            time.sleep(0.05)
        print(f'IMU A: BNO08x on i2c-{args.bus} @0x{args.address:02X}, every {args.report_interval * 1e3:.1f} ms')
        print(HELP)

        windows, samples = [], []
        while True:
            cmd = input(f'[{len(windows)} poses] > ').strip().lower()
            if cmd == 'q':
                print('quit without fitting')
                return None
            if cmd == 'u':
                if windows:
                    windows.pop()
                    samples.pop()
                    print('removed last pose')
                continue
            if cmd == 'f':
                if len(windows) < 9:
                    print(f'need at least 9 poses, have {len(windows)}')
                    continue
                break
            if cmd:
                print('unknown command')
                continue

            interruptions = poll.interruptions
            time.sleep(args.settle)
            a, g = col.record(args.duration)
            if poll.interruptions != interruptions:
                print(f'  IMU reset or stalled during this pose, not used (so far: {poll.resets} resets, '
                      f'{poll.stalls} stalls, {poll.soft_resets} soft resets sent, {poll.invalid_reads} '
                      '0xFF reads). Wait until it records again, then retry. Touching the unconnected '
                      'RST/BOOT pins or a loose wire resets it.')
                continue
            if len(a) < 2:
                print(f'only {len(a)} samples recorded (read errors: {reader.errors}, '
                      f'last: {reader.last_error!r}); try again')
                continue
            w = window_stats(a, g)
            norm = np.linalg.norm(w.accel_mean)
            if not is_still(w, args.max_accel_std, args.max_gyro_std):
                print(f'  moving, not used: accel std {w.accel_std.max():.4f} m/s^2, '
                      f'gyro std {w.gyro_std.max():.4f} rad/s (limits {args.max_accel_std}, {args.max_gyro_std})')
                continue
            angle = min_angle_deg(w.accel_mean, [x.accel_mean for x in windows])
            if angle < args.min_angle:
                print(f'  only {angle:.1f} deg from an earlier pose, not used; pick a more different orientation')
                continue
            windows.append(w)
            samples.append((a, g))
            u = w.accel_mean / norm
            cov = f'{coverage([x.accel_mean for x in windows]):.3f}' if len(windows) >= 3 else '-'
            print(f'  pose {len(windows)}: {w.n} samples, |a| = {norm:.3f} m/s^2, gravity dir '
                  f'[{u[0]:+.2f} {u[1]:+.2f} {u[2]:+.2f}], gyro mean {np.array2string(w.gyro_mean, precision=4)}, '
                  f'coverage {cov}')
            if abs(norm - args.gravity) > 0.1 * args.gravity:
                print('  warning: |a| is more than 10 % away from g; is the accelerometer report in m/s^2?')
            save_raw(os.path.join(out_dir, 'raw.npz'), windows, samples)   # keep data if the script dies
        return windows
    finally:
        reader.stop()


def main(argv=None):
    args = parse_args(argv)
    if args.refit:
        windows = load_raw(args.refit)
        out_dir = args.out or os.path.dirname(os.path.abspath(args.refit))
        os.makedirs(out_dir, exist_ok=True)
        meta = {'source': os.path.abspath(args.refit)}
    else:
        out_dir = args.out or os.path.abspath(time.strftime('imu_a_calib_%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)
        windows = capture(args, out_dir)
        if windows is None:
            return
        meta = {'sensor': f'BNO08x i2c-{args.bus} 0x{args.address:02X}',
                'dynamic_calibration': bool(args.dynamic_calibration),
                'report_interval': args.report_interval,
                'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    fit_and_save(windows, out_dir, args.gravity, meta)


if __name__ == '__main__':
    main()
