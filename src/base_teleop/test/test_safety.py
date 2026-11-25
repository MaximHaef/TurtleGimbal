import threading
from types import SimpleNamespace as Msg

from base_teleop.safety import StopLatch, event_stop_reason


def test_bumper():
    assert event_stop_reason('/events/bumper', Msg(bumper=1, state=1)) == 'bumper center pressed'
    assert event_stop_reason('/events/bumper', Msg(bumper=0, state=0)) is None   # released


def test_cliff_wheel_button():
    assert event_stop_reason('/events/cliff', Msg(sensor=2, state=1, bottom=0)) == 'cliff sensor right'
    assert event_stop_reason('/events/cliff', Msg(sensor=2, state=0, bottom=0)) is None   # floor
    assert event_stop_reason('/events/wheel_drop', Msg(wheel=0, state=1)) == 'left wheel dropped'
    assert event_stop_reason('/events/wheel_drop', Msg(wheel=0, state=0)) is None         # raised
    assert event_stop_reason('/events/button', Msg(button=2, state=1)) == 'button B2 pressed on the robot'
    assert event_stop_reason('/events/button', Msg(button=2, state=0)) is None            # released


def test_latch_keeps_first_reason():
    latch = StopLatch()
    assert not latch.stopped
    assert latch.trigger('bumper')
    assert not latch.trigger('key')
    assert latch.stopped and latch.reason == 'bumper'


def test_latch_threads():
    latch = StopLatch()
    first = []
    threads = [threading.Thread(target=lambda i=i: first.append(latch.trigger(i))) for i in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert first.count(True) == 1
