"""Wheel-speed saturation of the Kobuki base (Paper_changes.md, eq. 5).

The Kobuki firmware drives the wheels from a (speed, radius) pair and limits
the wheel speed. A command beyond the limit is scaled down along the same
radius, so v and w shrink by the same factor. Applying the same rule here
means the command we publish is the command the base actually executes.
"""


def saturate(v, w, track_width, wheel_speed_limit):
    """Scale (v, w) so that neither wheel exceeds wheel_speed_limit.

    Args:
        v: commanded forward speed [m/s].
        w: commanded yaw rate [rad/s].
        track_width: distance between the wheels b [m].
        wheel_speed_limit: maximum wheel speed v_max [m/s], > 0.

    Returns:
        (v_sat, w_sat, s) with v_sat = s*v, w_sat = s*w and 0 < s <= 1.
    """
    if wheel_speed_limit <= 0.0:
        raise ValueError('wheel_speed_limit must be positive')
    half = 0.5 * track_width * w
    fastest = max(abs(v + half), abs(v - half))
    s = 1.0 if fastest <= wheel_speed_limit else wheel_speed_limit / fastest
    return s * v, s * w, s
