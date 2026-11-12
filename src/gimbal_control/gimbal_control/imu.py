"""IMU reading for gimbal_control (ARCHITECTURE.md 4.3).

ImuReader polls a sensor in a background thread, so a slow or stalled I2C read
never blocks the control loop. The loop only takes the newest sample, or gets
every sample through `on_sample` (eq. 8 weights each IMU A sample).

open_bno08x() is the driver for IMU A (BNO08x, ARCHITECTURE.md 2). It
returns a `poll()` function for ImuReader. The Adafruit libraries are imported
inside it, so this module and its tests work on machines without them.
"""
import threading
import time
from typing import NamedTuple, Optional, Tuple

Vec3 = Tuple[float, float, float]


class ImuSample(NamedTuple):
    gyro: Vec3                # rad/s, sensor frame
    accel: Optional[Vec3]     # m/s^2 incl. gravity, sensor frame; None if not enabled / not yet received


class ImuReader:
    """Calls `poll()` in a loop in a daemon thread.

    poll() returns a list of new samples (e.g. ImuSample), possibly empty. Each sample is stamped with `stamp()` when poll() returns.
    Exceptions from poll() are counted and the loop continues.
    """

    def __init__(self, poll, stamp=time.monotonic, on_sample=None, idle_sleep=0.0005):
        self._poll = poll
        self._stamp = stamp
        self._on_sample = on_sample
        self._idle_sleep = idle_sleep
        self._lock = threading.Lock()
        self._latest = None           # (stamp, sample)
        self._last_sample_mono = None  # time.monotonic() of the latest sample
        self._running = False
        self._thread = None
        self.samples = 0
        self.errors = 0
        self.last_error = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name='imu_reader', daemon=True)
        self._thread.start()

    def stop(self, timeout=1.0):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout)

    def latest(self):
        """Newest (stamp, sample), or None before the first sample."""
        with self._lock:
            return self._latest

    def age(self):
        """Seconds since the newest sample (monotonic clock), or None before the first."""
        with self._lock:
            t = self._last_sample_mono
        return None if t is None else time.monotonic() - t

    def _run(self):
        while self._running:
            try:
                new = self._poll()
            except Exception as e:  # noqa: BLE001  (any bus or parse error)
                self.errors += 1
                self.last_error = e
                time.sleep(self._idle_sleep)
                continue
            if not new:
                time.sleep(self._idle_sleep)
                continue
            # Samples from one poll share a stamp. At the poll rate (kHz) vs. the
            # report rate (100-400 Hz) there is normally one sample per poll.
            stamp = self._stamp()
            mono = time.monotonic()
            for sample in new:
                with self._lock:
                    self._latest = (stamp, sample)
                    self._last_sample_mono = mono
                self.samples += 1
                if self._on_sample is not None:
                    self._on_sample(stamp, sample)


def open_bno08x(bus, address, report_interval, accel=True):
    """Open a BNO08x on Linux I2C bus `bus`, enable the calibrated gyroscope and,
    if `accel`, the accelerometer (incl. gravity), both every `report_interval` s.

    Returns poll() for ImuReader, which yields one ImuSample per gyro report,
    paired with the newest accelerometer report.
    Needs `adafruit-circuitpython-bno08x` and `adafruit-extended-bus` (pip).
    """
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_extended_bus import ExtendedI2C

    class _Bno08x(BNO08X_I2C):
        # The library keeps only the latest value per report type. Collect every
        # gyro report instead, so no sample is lost between polls.
        def __init__(self, *args, **kwargs):
            self.new = []
            self.accel = None
            super().__init__(*args, **kwargs)

        def _process_report(self, report_id, report_bytes):
            super()._process_report(report_id, report_bytes)
            if report_id == BNO_REPORT_ACCELEROMETER:
                self.accel = self._readings[report_id]
            elif report_id == BNO_REPORT_GYROSCOPE:
                self.new.append(ImuSample(self._readings[report_id], self.accel))

    sensor = _Bno08x(ExtendedI2C(bus), address=address)
    interval_us = int(round(report_interval * 1e6))
    sensor.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)
    if accel:
        sensor.enable_feature(BNO_REPORT_ACCELEROMETER, interval_us)

    def poll():
        sensor._process_available_packets()
        new, sensor.new = sensor.new, []
        return new

    return poll
