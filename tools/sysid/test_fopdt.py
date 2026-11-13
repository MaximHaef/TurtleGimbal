"""Synthetic tests: the fit must recover known parameters from noisy 50 Hz data.

    python3 -m pytest tools/sysid -p no:cacheprovider
"""
import numpy as np
import pytest

from fopdt import find_steps, fit, hold, simulate


def staircase(levels, hold_s=2.5, rate=50.0):
    """Command samples at `rate` Hz stepping through `levels`."""
    t = np.arange(0.0, hold_s * len(levels), 1.0 / rate)
    u = np.array(levels)[np.minimum((t / hold_s).astype(int), len(levels) - 1)]
    return t, u


def measure(t_cmd, u_cmd, tau, delay, gain, noise, seed=0, rate=50.0, jitter=0.002):
    rng = np.random.default_rng(seed)
    t = np.arange(0.01, t_cmd[-1], 1.0 / rate) + rng.uniform(-jitter, jitter, size=None)
    y = simulate(t_cmd, u_cmd, t, tau, delay, gain)
    return t, y + rng.normal(0.0, noise, size=t.shape)


def test_hold_is_zero_order():
    t_cmd = np.array([0.0, 1.0, 2.0])
    u_cmd = np.array([0.0, 5.0, -1.0])
    assert list(hold(t_cmd, u_cmd, np.array([-1.0, 0.5, 1.0, 1.99, 3.0]))) == [0.0, 0.0, 5.0, 5.0, -1.0]


def test_simulate_step_matches_analytic():
    t_cmd = np.array([0.0, 1.0])
    u_cmd = np.array([0.0, 1.0])
    t = np.linspace(0.0, 3.0, 301)
    y = simulate(t_cmd, u_cmd, t, tau=0.2, delay=0.1, gain=0.8)
    expected = np.where(t > 1.1, 0.8 * (1 - np.exp(-(t - 1.1) / 0.2)), 0.0)
    assert np.max(np.abs(y - expected)) < 0.01


@pytest.mark.parametrize('tau,delay,gain', [(0.15, 0.06, 0.95), (0.4, 0.0, 1.0), (0.05, 0.12, 1.1)])
def test_fit_recovers_parameters(tau, delay, gain):
    t_cmd, u_cmd = staircase([0, 0.5, 0, -0.5, 0, 1.0, 0, -1.0, 0])
    t, y = measure(t_cmd, u_cmd, tau, delay, gain, noise=0.02)
    r = fit(t_cmd, u_cmd, t, y, y0=0.0)
    assert r.tau == pytest.approx(tau, abs=0.015)
    assert r.delay == pytest.approx(delay, abs=0.01)
    assert r.gain == pytest.approx(gain, abs=0.02)
    assert r.r2 > 0.98


def test_fit_with_fixed_gain():
    t_cmd, u_cmd = staircase([0, 0.6, 0, -0.6, 0])
    t, y = measure(t_cmd, u_cmd, 0.2, 0.05, 1.0, noise=0.01)
    r = fit(t_cmd, u_cmd, t, y, y0=0.0, fix_gain=1.0)
    assert r.gain == 1.0
    assert r.tau == pytest.approx(0.2, abs=0.015)


def test_find_steps():
    t_cmd, u_cmd = staircase([0, 0.5, 0.5, 0], hold_s=1.0)
    steps = find_steps(t_cmd, u_cmd)
    assert [(round(t, 2), a, b) for t, a, b in steps] == [(1.0, 0.0, 0.5), (3.0, 0.5, 0.0)]
