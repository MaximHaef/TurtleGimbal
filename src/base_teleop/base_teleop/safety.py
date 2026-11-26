"""Stop conditions of the excitation run: manual stop and Kobuki safety events.

Kept free of ROS so it can be tested without it. The message fields follow
kobuki_ros_interfaces (BumperEvent, CliffEvent, WheelDropEvent, ButtonEvent).
"""
import threading

# topic -> (message type name in kobuki_ros_interfaces.msg, field, stop value, description)
KOBUKI_EVENTS = {
    '/events/bumper': ('BumperEvent', 'bumper', 1, 'bumper {0} pressed', ('left', 'center', 'right')),
    '/events/cliff': ('CliffEvent', 'sensor', 1, 'cliff sensor {0}', ('left', 'center', 'right')),
    '/events/wheel_drop': ('WheelDropEvent', 'wheel', 1, '{0} wheel dropped', ('left', 'right')),
    '/events/button': ('ButtonEvent', 'button', 1, 'button B{0} pressed on the robot', None),
}


def event_stop_reason(topic, msg):
    """Return why the run must stop for this Kobuki event, or None.

    Every event type uses state == 1 for the unsafe/pressed state
    (PRESSED, CLIFF, DROPPED, PRESSED).
    """
    _, field, stop_state, text, names = KOBUKI_EVENTS[topic]
    if msg.state != stop_state:
        return None
    which = getattr(msg, field)
    name = names[which] if names is not None and which < len(names) else which
    return text.format(name)


class StopLatch:
    """Remembers the first stop reason. Once set it never clears."""

    def __init__(self):
        self._lock = threading.Lock()
        self._reason = None

    def trigger(self, reason):
        """Set the stop. Returns True if this call was the first one."""
        with self._lock:
            if self._reason is not None:
                return False
            self._reason = reason
            return True

    @property
    def reason(self):
        with self._lock:
            return self._reason

    @property
    def stopped(self):
        return self.reason is not None
