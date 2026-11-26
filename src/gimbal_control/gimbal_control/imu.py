"""IMU reading for gimbal_control (ARCHITECTURE.md 4.3).

ImuReader polls a sensor in a background thread, so a slow or stalled I2C read
never blocks the control loop. The loop only takes the newest sample, or gets
every sample through `on_sample` (eq. 8 weights each IMU A sample).

open_bno08x() is the driver for IMU A (BNO08x, ARCHITECTURE.md 2). It
returns a Bno08xPoller, the `poll()` for ImuReader, which configures the sensor
again when it resets itself. The Adafruit libraries are imported
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


class Bno08xPoller:
    """poll() for ImuReader over a BNO08x that may reset itself.

    A BNO08x can reset at any time (a touched RST/BOOT pin, a loose wire, a
    glitch on the supply). It then forgets its configuration: no more reports,
    and its own dynamic calibration is back on. The sensor announces the reset
    with a "reset complete" packet; `sensor.reset_seen` is set when it arrives.
    The poller then calls `configure()` again (enable the reports, send the
    dynamic-calibration setting), retrying every `retry_interval` s while the
    sensor is still booting (configure() raises). If no sample arrives for
    `stall_timeout` s without an announced reset, it configures again too. If
    that brings no sample either (the sensor hangs, e.g. it only reads 0xFF),
    the next stall sends `soft_reset()` before configuring.

    `sensor` needs `_process_available_packets()`, the list `new` (samples since
    the last poll), `accel` (newest accelerometer report) and `reset_seen`.
    `configure()` returns the dynamic-calibration ack (see open_bno08x).

    Counters (read from other threads, written only by the reader thread):
    resets (reset-complete packets, incl. after our soft resets), stalls
    (stall_timeout without a sample), soft_resets (sent by us),
    invalid_reads (sensor's `invalid_reads`: header read as all 0xFF).
    """

    def __init__(self, sensor, configure, soft_reset=None, stall_timeout=1.0, retry_interval=0.05,
                 clock=time.monotonic):
        self._sensor = sensor
        self._configure = configure
        self._soft_reset = soft_reset
        self._stall_timeout = stall_timeout
        self._retry_interval = retry_interval
        self._clock = clock
        self._need_config = False
        self._next_try = 0.0
        self._last_activity = clock()
        self._stalls_in_a_row = 0
        self.resets = 0
        self.stalls = 0
        self.soft_resets = 0
        self.dynamic_calibration_ack = None

    @property
    def invalid_reads(self):
        return getattr(self._sensor, 'invalid_reads', 0)

    @property
    def interruptions(self):
        """Changes whenever the stream was interrupted (reset, stall, soft reset)."""
        return self.resets + self.stalls + self.soft_resets

    def configure(self):
        self.dynamic_calibration_ack = self._configure()
        self._sensor.reset_seen = False   # the reset-complete of a startup reset may come late
        self._last_activity = self._clock()

    def __call__(self):
        now = self._clock()
        if self._need_config and now >= self._next_try:
            self._next_try = now + self._retry_interval
            self.configure()   # raises while the sensor is booting; ImuReader counts it, we retry
            self._need_config = False

        s = self._sensor
        s._process_available_packets()
        if s.reset_seen:
            s.reset_seen = False
            s.new, s.accel = [], None    # don't pair new gyro reports with an accel from before
            self.resets += 1
            self._need_config = True
            self._next_try = now
            return []

        new, s.new = s.new, []
        if new:
            self._last_activity = now
            self._stalls_in_a_row = 0
        elif not self._need_config and now - self._last_activity > self._stall_timeout:
            self.stalls += 1
            self._stalls_in_a_row += 1
            self._need_config = True
            self._next_try = now
            if self._soft_reset is not None and self._stalls_in_a_row >= 2:
                # Configuring again did not help: reset the sensor (as at startup).
                self._stalls_in_a_row = 0
                self.soft_resets += 1
                s.new, s.accel = [], None
                self._soft_reset()   # may raise; configure follows on the next poll either way
        return new


def open_bno08x(bus, address, report_interval, accel=True, dynamic_calibration=False):
    """Open a BNO08x on Linux I2C bus `bus`, enable the calibrated gyroscope and,
    if `accel`, the accelerometer (incl. gravity), both every `report_interval` s.

    dynamic_calibration=False turns off the sensor's own running bias estimation
    (accel, gyro, mag), so its output only changes with our stored calibration
    (imu_calibration.py). The setting is not saved on the sensor; it is sent at
    every start and after every reset of the sensor.

    Returns a Bno08xPoller (poll() for ImuReader), which yields one ImuSample per
    gyro report, paired with the newest accelerometer report, and configures the
    sensor again after it resets. poll.dynamic_calibration_ack is True if the
    sensor acknowledged the calibration setting (last time it was sent), None if
    not sent. poll.resets, .stalls, .soft_resets, .invalid_reads count problems.
    Needs `adafruit-circuitpython-bno08x` and `adafruit-extended-bus` (pip).
    """
    from adafruit_bno08x import BNO_CHANNEL_EXE, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_extended_bus import ExtendedI2C

    class _Bno08x(BNO08X_I2C):
        # The library keeps only the latest value per report type. Collect every
        # gyro report instead, so no sample is lost between polls.
        def __init__(self, *args, **kwargs):
            self.new = []
            self.accel = None
            self.reset_seen = False
            self.invalid_reads = 0
            super().__init__(*args, **kwargs)

        @property
        def _data_ready(self):
            # As the library's, but counts an all-0xFF header (sensor not driving the
            # bus, e.g. during a reset) instead of printing two lines per read.
            header = self._read_header()
            if header.packet_byte_count == 0x7FFF:
                self.invalid_reads += 1
                return False
            return header.data_length > 0

        def _handle_packet(self, packet):
            # SHTP executable channel, sensor -> host: 1 = reset complete. The library
            # would parse it as a (too short) sensor report and raise.
            if packet.channel_number == BNO_CHANNEL_EXE:
                if packet.header.data_length >= 1 and packet.data[0] == 1:
                    self.reset_seen = True
                return
            super()._handle_packet(packet)

        def _process_report(self, report_id, report_bytes):
            super()._process_report(report_id, report_bytes)
            if report_id == BNO_REPORT_ACCELEROMETER:
                self.accel = self._readings[report_id]
            elif report_id == BNO_REPORT_GYROSCOPE:
                self.new.append(ImuSample(self._readings[report_id], self.accel))

    sensor = _Bno08x(ExtendedI2C(bus), address=address)
    interval_us = int(round(report_interval * 1e6))

    def configure():
        # After a reset, enable_feature() only sends the request: it returns at
        # once because the report id is already in _readings.
        sensor.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)
        if accel:
            sensor.enable_feature(BNO_REPORT_ACCELEROMETER, interval_us)
        if dynamic_calibration:
            return None
        # SH-2 "configure ME calibration": accel, gyro, mag, subcommand 0 (config),
        # planar, on-table, 3 reserved; all 0 = off. Private API of
        # adafruit-circuitpython-bno08x (1.3.3): it waits for the response, and a
        # successful response sets _me_calibration_started_at.
        t0 = time.monotonic()
        sensor._send_me_command([0, 0, 0, 0, 0, 0, 0, 0, 0])
        return getattr(sensor, '_me_calibration_started_at', -1.0) > t0

    poll = Bno08xPoller(sensor, configure, soft_reset=sensor.soft_reset)
    poll.configure()
    return poll
