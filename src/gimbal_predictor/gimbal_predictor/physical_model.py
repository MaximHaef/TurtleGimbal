"""Physical model of the differential-drive base (paper eq. 6), without ROS.

State: forward speed v [m/s] and yaw rate w_z [rad/s]. Each follows the
commanded value with a first-order lag:

    tau_v * dv/dt   = v_cmd - v
    tau_w * dw_z/dt = w_cmd - w_z

The command is held constant over the horizon (paper Sec. II-B). The rollout
is the exact solution, so it does not depend on the step size.

Outputs in the base frame (x forward, y left, z up):
    angular velocity  omega = [0, 0, w_z]            (planar model)
    acceleration      a     = [dv/dt, v * w_z, 0]    (forward + centripetal, without gravity)
"""
import numpy as np


def rollout(v0, w0, v_cmd, w_cmd, tau_v, tau_w, dt, steps):
    """Predict the base motion at t_k + j*dt for j = 1..steps.

    Args:
        v0, w0: measured forward speed [m/s] and yaw rate [rad/s] at t_k.
        v_cmd, w_cmd: command u_k [m/s], [rad/s], held over the horizon.
        tau_v, tau_w: time constants [s], > 0.
        dt: spacing of the horizon samples [s], > 0.
        steps: number of samples F, >= 1.

    Returns:
        t_rel: (F,) times after t_k [s].
        accel: (F, 3) translational acceleration [m/s^2].
        omega: (F, 3) angular velocity [rad/s].
    """
    if tau_v <= 0.0 or tau_w <= 0.0:
        raise ValueError('time constants must be positive')
    if dt <= 0.0 or steps < 1:
        raise ValueError('dt must be positive and steps >= 1')

    t_rel = dt * np.arange(1, steps + 1)
    v = v_cmd + (v0 - v_cmd) * np.exp(-t_rel / tau_v)
    w = w_cmd + (w0 - w_cmd) * np.exp(-t_rel / tau_w)

    zeros = np.zeros(steps)
    accel = np.stack([(v_cmd - v) / tau_v, v * w, zeros], axis=1)
    omega = np.stack([zeros, zeros, w], axis=1)
    return t_rel, accel, omega
