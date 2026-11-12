"""Holds the latest command request and drops it when it gets too old."""


class CommandHold:
    """Zero-order hold with a timeout.

    The relay publishes at a fixed rate, but requests arrive whenever the
    source sends them. The last request is repeated until it is older than
    `timeout`; after that the output is zero, so the robot stops when the
    command source or the network goes away.
    """

    def __init__(self, timeout):
        if timeout <= 0.0:
            raise ValueError('timeout must be positive')
        self.timeout = timeout
        self._v = 0.0
        self._w = 0.0
        self._t = None

    def update(self, v, w, t):
        """Store a new request (v [m/s], w [rad/s]) received at time t [s]."""
        self._v = float(v)
        self._w = float(w)
        self._t = t

    def get(self, t):
        """Return (v, w, fresh) at time t. Zero and fresh=False if stale."""
        if self._t is None or t - self._t > self.timeout:
            return 0.0, 0.0, False
        return self._v, self._w, True
