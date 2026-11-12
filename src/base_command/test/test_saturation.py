import math

import pytest

from base_command.saturation import saturate

B = 0.230
VMAX = 0.7


def wheels(v, w):
    return v - 0.5 * B * w, v + 0.5 * B * w  # left, right


def test_within_limit_unchanged():
    assert saturate(0.2, 0.5, B, VMAX) == (0.2, 0.5, 1.0)


def test_zero():
    assert saturate(0.0, 0.0, B, VMAX) == (0.0, 0.0, 1.0)


@pytest.mark.parametrize('v', [1.0, -1.0])
def test_straight_over_limit(v):
    vs, ws, s = saturate(v, 0.0, B, VMAX)
    assert vs == pytest.approx(math.copysign(VMAX, v))
    assert ws == 0.0
    assert s == pytest.approx(VMAX)


@pytest.mark.parametrize('w', [10.0, -10.0])
def test_turn_in_place_over_limit(w):
    vs, ws, _ = saturate(0.0, w, B, VMAX)
    assert vs == 0.0
    assert max(abs(x) for x in wheels(vs, ws)) == pytest.approx(VMAX)


@pytest.mark.parametrize('v,w', [(1.0, 1.5), (1.0, -1.5), (-1.0, 1.5), (-1.0, -1.5), (0.6, 3.0)])
def test_over_limit_keeps_radius(v, w):
    vs, ws, s = saturate(v, w, B, VMAX)
    assert 0.0 < s < 1.0
    assert vs / ws == pytest.approx(v / w)          # same turning radius
    assert max(abs(x) for x in wheels(vs, ws)) == pytest.approx(VMAX)


def test_invalid_limit():
    with pytest.raises(ValueError):
        saturate(0.1, 0.1, B, 0.0)
