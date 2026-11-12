import threading
import time

from gimbal_control.imu import ImuReader


def wait_for(cond, timeout=1.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.001)
    return False


class FakeSensor:
    """poll() returns queued batches, raises queued exceptions, else []."""

    def __init__(self, items):
        self.items = list(items)
        self.lock = threading.Lock()

    def poll(self):
        with self.lock:
            if not self.items:
                return []
            item = self.items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


def test_no_sample_before_start():
    r = ImuReader(FakeSensor([]).poll)
    assert r.latest() is None
    assert r.age() is None


def test_every_sample_delivered_in_order():
    got = []
    batches = [[(0.1, 0.0, 0.0)], [], [(0.2, 0.0, 0.0), (0.3, 0.0, 0.0)]]
    r = ImuReader(FakeSensor(batches).poll, on_sample=lambda s, g: got.append(g))
    r.start()
    assert wait_for(lambda: r.samples == 3)
    r.stop()
    assert got == [(0.1, 0.0, 0.0), (0.2, 0.0, 0.0), (0.3, 0.0, 0.0)]
    assert r.latest()[1] == (0.3, 0.0, 0.0)
    assert r.age() < 1.0


def test_stamp_is_used():
    stamps = iter([10.0, 11.0])
    r = ImuReader(FakeSensor([[(1.0, 2.0, 3.0)]]).poll, stamp=lambda: next(stamps))
    r.start()
    assert wait_for(lambda: r.samples == 1)
    r.stop()
    assert r.latest() == (10.0, (1.0, 2.0, 3.0))


def test_errors_counted_and_reading_continues():
    items = [OSError(121, 'Remote I/O error'), [(0.5, 0.0, 0.0)]]
    r = ImuReader(FakeSensor(items).poll)
    r.start()
    assert wait_for(lambda: r.samples == 1)
    r.stop()
    assert r.errors == 1
    assert isinstance(r.last_error, OSError)


def test_age_grows_without_new_samples():
    r = ImuReader(FakeSensor([[(0.0, 0.0, 0.0)]]).poll)
    r.start()
    assert wait_for(lambda: r.samples == 1)
    a1 = r.age()
    time.sleep(0.02)
    assert r.age() > a1
    r.stop()


def test_stop_ends_thread():
    r = ImuReader(FakeSensor([]).poll)
    r.start()
    r.stop()
    assert not r._thread.is_alive()
