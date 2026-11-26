"""Bno08xPoller: reconfigure the BNO08x after it resets itself (no hardware)."""
import pytest

from gimbal_control.imu import Bno08xPoller, ImuSample


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeBno:
    """Each _process_available_packets() runs the next scripted step:
    a list of gyro values (reports arrive), 'reset' (reset-complete packet),
    an exception (bus error), or nothing (no data)."""

    def __init__(self, steps=()):
        self.steps = list(steps)
        self.new = []
        self.accel = None
        self.reset_seen = False

    def _process_available_packets(self):
        if not self.steps:
            return
        step = self.steps.pop(0)
        if isinstance(step, Exception):
            raise step
        if step == 'reset':
            self.reset_seen = True
        elif step is not None:
            for g in step:
                self.accel = (0.0, 0.0, 9.8)
                self.new.append(ImuSample((g, 0.0, 0.0), self.accel))


class Configure:
    """configure() for the poller; raises the queued errors first (sensor booting)."""

    def __init__(self, errors=()):
        self.errors = list(errors)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        if self.errors:
            raise self.errors.pop(0)
        return True


def make(steps, errors=(), **kw):
    sensor, cfg, clock = FakeBno(steps), Configure(errors), Clock()
    poll = Bno08xPoller(sensor, cfg, clock=clock, **kw)
    poll.configure()
    return poll, sensor, cfg, clock


def test_samples_pass_through():
    poll, _, cfg, _ = make([[1.0, 2.0], None, [3.0]])
    assert [s.gyro[0] for s in poll()] == [1.0, 2.0]
    assert poll() == []
    assert [s.gyro[0] for s in poll()] == [3.0]
    assert cfg.calls == 1 and poll.resets == 0 and poll.stalls == 0


def test_reset_reconfigures_on_next_poll():
    poll, sensor, cfg, _ = make([[1.0], 'reset', [2.0]])
    poll()
    assert poll() == []
    assert poll.resets == 1 and cfg.calls == 1
    assert [s.gyro[0] for s in poll()] == [2.0]
    assert cfg.calls == 2


def test_reset_drops_pending_samples_and_old_accel():
    poll, sensor, _, _ = make(['reset'])
    sensor.new = [ImuSample((9.0, 0.0, 0.0), (0.0, 0.0, 9.8))]
    sensor.accel = (0.0, 0.0, 9.8)
    assert poll() == []
    assert sensor.accel is None and sensor.new == []


def test_configure_retried_while_sensor_boots():
    poll, _, cfg, clock = make(['reset', None, [5.0]], retry_interval=0.05)
    cfg.errors = [OSError(6, 'No such device or address')] * 2
    poll()                                   # reset seen
    with pytest.raises(OSError):
        poll()                               # 1st configure fails
    assert poll() == []                      # too early to retry, just reads
    clock.t += 0.06
    with pytest.raises(OSError):
        poll()                               # 2nd fails
    clock.t += 0.06
    assert [s.gyro[0] for s in poll()] == [5.0]   # 3rd works
    assert cfg.calls == 4 and poll.resets == 1


def test_second_reset_complete_reconfigures_again():
    # Seen on the hardware: two reset-complete packets 0.15 s apart.
    poll, _, cfg, _ = make(['reset', 'reset', [1.0]])
    poll()
    poll()
    assert [s.gyro[0] for s in poll()] == [1.0]
    assert poll.resets == 2 and cfg.calls == 3


def test_stall_reconfigures_once_per_timeout():
    poll, _, cfg, clock = make([None] * 10, stall_timeout=1.0)
    clock.t = 0.5
    poll()
    assert poll.stalls == 0
    clock.t = 1.1
    poll()                                   # stall detected
    assert poll.stalls == 1 and cfg.calls == 1
    poll()                                   # reconfigured, timer restarted
    assert cfg.calls == 2
    clock.t = 1.5
    poll()
    assert poll.stalls == 1
    clock.t = 2.2
    poll()
    assert poll.stalls == 2


def test_samples_keep_stall_timer_reset():
    poll, _, _, clock = make([[1.0]] * 5, stall_timeout=1.0)
    for _ in range(5):
        clock.t += 0.9
        poll()
    assert poll.stalls == 0


def test_configure_clears_late_startup_reset():
    sensor, cfg = FakeBno(), Configure()
    sensor.reset_seen = True                 # e.g. reset-complete of the startup soft reset
    poll = Bno08xPoller(sensor, cfg, clock=Clock())
    poll.configure()
    poll()
    assert poll.resets == 0 and poll.dynamic_calibration_ack is True


class SoftReset:
    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1


def test_stall_not_fixed_by_configure_sends_soft_reset():
    sensor, cfg, clock, sr = FakeBno([None] * 20), Configure(), Clock(), SoftReset()
    poll = Bno08xPoller(sensor, cfg, soft_reset=sr, clock=clock, stall_timeout=1.0)
    poll.configure()
    clock.t = 1.1
    poll()                                   # 1st stall: only configure again
    poll()
    assert (poll.stalls, sr.calls, cfg.calls) == (1, 0, 2)
    clock.t = 2.2
    poll()                                   # 2nd stall in a row: soft reset
    assert (poll.stalls, sr.calls, poll.soft_resets) == (2, 1, 1)
    poll()
    assert cfg.calls == 3                    # configured after the soft reset
    clock.t = 3.3
    poll()                                   # counting starts over: configure only
    assert sr.calls == 1


def test_samples_between_stalls_prevent_soft_reset():
    sensor, cfg, clock, sr = FakeBno([None, None, [1.0], None, None]), Configure(), Clock(), SoftReset()
    poll = Bno08xPoller(sensor, cfg, soft_reset=sr, clock=clock, stall_timeout=1.0)
    poll.configure()
    clock.t = 1.1
    poll()                                   # stall 1
    poll()                                   # configure, sample arrives
    clock.t = 2.2
    poll()
    clock.t = 3.3
    poll()                                   # stall again, but not in a row
    assert poll.stalls == 2 and sr.calls == 0


def test_interruptions_counter():
    poll, _, _, _ = make(['reset'])
    before = poll.interruptions
    poll()
    assert poll.interruptions == before + 1
    assert poll.invalid_reads == 0           # fake sensor has no counter
