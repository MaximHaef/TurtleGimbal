import numpy as np
import pytest

from gimbal_predictor.history import History
from gimbal_predictor.physical_model import rollout

TAU_V, TAU_W = 0.094, 0.120


def test_shapes_and_times():
    t, a, w = rollout(0.0, 0.0, 0.1, 0.5, TAU_V, TAU_W, 0.02, 7)
    assert t.shape == (7,) and a.shape == (7, 3) and w.shape == (7, 3)
    assert t == pytest.approx(0.02 * np.arange(1, 8))


def test_matches_analytic_step():
    t, a, w = rollout(0.0, 0.0, 0.2, -0.6, TAU_V, TAU_W, 0.01, 50)
    v = 0.2 * (1 - np.exp(-t / TAU_V))
    wz = -0.6 * (1 - np.exp(-t / TAU_W))
    assert w[:, 2] == pytest.approx(wz)
    assert a[:, 0] == pytest.approx(0.2 / TAU_V * np.exp(-t / TAU_V))   # dv/dt
    assert a[:, 1] == pytest.approx(v * wz)                             # centripetal


def test_planar_components_zero():
    _, a, w = rollout(0.1, 0.3, 0.2, -0.4, TAU_V, TAU_W, 0.02, 10)
    assert np.all(w[:, :2] == 0.0)
    assert np.all(a[:, 2] == 0.0)


def test_steady_state_constant():
    _, a, w = rollout(0.15, 0.4, 0.15, 0.4, TAU_V, TAU_W, 0.02, 10)
    assert a[:, 0] == pytest.approx(np.zeros(10))
    assert a[:, 1] == pytest.approx(np.full(10, 0.15 * 0.4))
    assert w[:, 2] == pytest.approx(np.full(10, 0.4))


def test_decays_to_command_from_motion():
    # Robot moving, command to stop: speed and yaw rate decay to zero, deceleration negative.
    _, a, w = rollout(0.2, 0.5, 0.0, 0.0, TAU_V, TAU_W, 0.05, 40)
    assert abs(w[-1, 2]) < 1e-6
    assert np.all(a[:, 0] < 0)


def test_step_size_independent():
    _, _, w1 = rollout(0.0, 0.0, 0.1, 0.5, TAU_V, TAU_W, 0.01, 20)
    _, _, w2 = rollout(0.0, 0.0, 0.1, 0.5, TAU_V, TAU_W, 0.02, 10)
    assert w1[1::2, 2] == pytest.approx(w2[:, 2])


@pytest.mark.parametrize('kw', [dict(tau_v=0.0), dict(dt=0.0), dict(steps=0)])
def test_invalid_arguments(kw):
    args = dict(v0=0.0, w0=0.0, v_cmd=0.1, w_cmd=0.1, tau_v=TAU_V, tau_w=TAU_W, dt=0.02, steps=5)
    args.update(kw)
    with pytest.raises(ValueError):
        rollout(**args)


def test_history_at_or_before():
    h = History(3)
    assert h.at_or_before(1.0) is None
    for t, v in ((1.0, 10), (2.0, 20), (3.0, 30)):
        h.add(t, v)
    assert h.at_or_before(0.5) is None
    assert h.at_or_before(2.0) == (2.0, 20)
    assert h.at_or_before(2.9) == (2.0, 20)
    assert h.at_or_before(9.0) == (3.0, 30)
    h.add(2.5, 25)          # out of order: dropped
    assert h.at_or_before(2.7) == (2.0, 20)
    h.add(4.0, 40)          # maxlen 3: oldest dropped
    assert h.at_or_before(1.5) is None
