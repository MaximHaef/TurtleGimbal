"""First-order-plus-dead-time (FOPDT) model and fit.

Model of one base channel (yaw rate or forward speed), eq. (6) of the paper
plus a dead time T_d:

    tau * dy/dt = K * u(t - T_d) - y

u is the executed command (/base_command, zero-order hold between samples),
y is the measured response (Kobuki gyro or wheel odometry). The paper's eq. (6)
is the special case K = 1, T_d = 0.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

SIM_DT = 1e-3  # s, internal simulation step (much finer than the 50 Hz data)

# Parameter bounds (lower, upper) for tau [s], delay [s], gain [-].
BOUNDS = ((0.005, 0.0, 0.3), (3.0, 1.0, 2.0))


@dataclass
class FitResult:
    tau: float     # s, time constant
    delay: float   # s, dead time T_d
    gain: float    # -, static gain K
    rmse: float    # same unit as y
    r2: float      # coefficient of determination
    n: int         # number of samples used


def hold(t_cmd, u_cmd, t):
    """Zero-order hold of the command samples (t_cmd, u_cmd) at times t.

    Before the first command sample the first value is used.
    """
    idx = np.searchsorted(t_cmd, t, side='right') - 1
    return u_cmd[np.clip(idx, 0, len(u_cmd) - 1)]


def held_integral(t_cmd, u_cmd, t):
    """Integral of the zero-order-held command from t_cmd[0] to t (exact, piecewise linear).

    Before t_cmd[0] the first value is extended, so the integral is negative there.
    """
    t_cmd = np.asarray(t_cmd, dtype=float)
    u_cmd = np.asarray(u_cmd, dtype=float)
    knots = np.concatenate(([0.0], np.cumsum(u_cmd[:-1] * np.diff(t_cmd))))
    idx = np.clip(np.searchsorted(t_cmd, t, side='right') - 1, 0, len(t_cmd) - 1)
    return knots[idx] + u_cmd[idx] * (t - t_cmd[idx])


def simulate(t_cmd, u_cmd, t_out, tau, delay, gain, y0=0.0):
    """Response of the FOPDT model at times t_out.

    Integrated on a fine grid starting at t_out[0] with initial state y0, then
    sampled at t_out. In each SIM_DT step the input is the exact average of the
    delayed, held command over that step, so the response is continuous in the
    delay (needed for the gradient of the fit).
    """
    t_out = np.asarray(t_out, dtype=float)
    grid = np.arange(t_out[0], t_out[-1] + SIM_DT, SIM_DT)
    edges = np.append(grid, grid[-1] + SIM_DT) - delay
    u = gain * np.diff(held_integral(t_cmd, u_cmd, edges)) / SIM_DT
    a = np.exp(-SIM_DT / tau)
    # y[k+1] = a*y[k] + (1-a)*u[k], with y[0] = y0.
    y, _ = lfilter([0.0, 1.0 - a], [1.0, -a], u, zi=[a * y0])
    return np.interp(t_out, grid, y)


def fit(t_cmd, u_cmd, t_meas, y_meas, y0=None, fix_gain=None):
    """Least-squares fit of (tau, delay, gain) to measured data.

    A coarse grid over (tau, delay) gives the start point, so the fit does not
    get stuck in a local minimum of the delay. y0 defaults to the first
    measured value. With fix_gain set, the gain is not fitted.
    """
    t_meas = np.asarray(t_meas, dtype=float)
    y_meas = np.asarray(y_meas, dtype=float)
    if y0 is None:
        y0 = float(y_meas[0])

    def residual(p):
        tau, delay = p[0], p[1]
        gain = fix_gain if fix_gain is not None else p[2]
        return simulate(t_cmd, u_cmd, t_meas, tau, delay, gain, y0) - y_meas

    gain0 = 1.0 if fix_gain is None else fix_gain
    best = None
    for tau in (0.03, 0.08, 0.15, 0.3, 0.6, 1.2):
        for delay in np.arange(0.0, 0.41, 0.02):
            p = [tau, delay, gain0] if fix_gain is None else [tau, delay]
            cost = float(np.sum(residual(p) ** 2))
            if best is None or cost < best[0]:
                best = (cost, p)

    lo, hi = BOUNDS
    if fix_gain is not None:
        lo, hi = lo[:2], hi[:2]
    sol = least_squares(residual, best[1], bounds=(lo, hi), x_scale=[0.1, 0.05, 0.1][:len(lo)])

    tau, delay = sol.x[0], sol.x[1]
    gain = fix_gain if fix_gain is not None else sol.x[2]
    res = residual(sol.x)
    ss_tot = float(np.sum((y_meas - y_meas.mean()) ** 2))
    r2 = 1.0 - float(np.sum(res ** 2)) / ss_tot if ss_tot > 0 else float('nan')
    return FitResult(float(tau), float(delay), float(gain),
                     float(np.sqrt(np.mean(res ** 2))), r2, len(y_meas))


def find_steps(t_cmd, u_cmd, min_change=1e-3):
    """Times and values where the command changes: list of (t, u_before, u_after)."""
    u_cmd = np.asarray(u_cmd, dtype=float)
    steps = []
    for i in np.nonzero(np.abs(np.diff(u_cmd)) > min_change)[0]:
        steps.append((float(t_cmd[i + 1]), float(u_cmd[i]), float(u_cmd[i + 1])))
    return steps
