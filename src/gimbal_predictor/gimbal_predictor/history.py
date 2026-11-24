"""Short history of timestamped scalar samples."""
import bisect
from collections import deque


class History:
    """Last `maxlen` samples (stamp [s], value), ordered by stamp."""

    def __init__(self, maxlen):
        self._stamps = deque(maxlen=maxlen)
        self._values = deque(maxlen=maxlen)

    def __len__(self):
        return len(self._stamps)

    def add(self, stamp, value):
        """Append a sample. Samples older than the newest one are dropped (out of order)."""
        if self._stamps and stamp < self._stamps[-1]:
            return
        self._stamps.append(stamp)
        self._values.append(value)

    def at_or_before(self, t):
        """Return (stamp, value) of the newest sample with stamp <= t, or None."""
        i = bisect.bisect_right(self._stamps, t) - 1
        if i < 0:
            return None
        return self._stamps[i], self._values[i]
