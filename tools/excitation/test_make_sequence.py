import math
import os
import sys

import pytest
import yaml

import make_sequence as ms

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'base_teleop'))
from base_teleop.sequence import command_at, load_sequence  # noqa: E402

SYSID = os.path.join(ROOT, 'src', 'base_teleop', 'config', 'sysid_steps.yaml')


def write(tmp_path, *args):
    out = tmp_path / 'seq.yaml'
    assert ms.main(['--out', str(out), *args]) == 0
    return out


def euler_path(segments, dt=0.001):
    """Independent pose integration (Euler), to check the generator's own integrator."""
    x = y = th = 0.0
    extent = 0.0
    for d, v, w in segments:
        for _ in range(int(round(d / dt))):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th += w * dt
            extent = max(extent, abs(x), abs(y))
    return extent, (x, y, th)


@pytest.mark.parametrize('args', [['--box', '1.5'], ['--box', '0.8', '--rounds', '2', '--seed', '5']])
def test_file_loads_and_obeys_constraints(tmp_path, args):
    out = write(tmp_path, *args)
    box = float(args[1])
    segs = load_sequence(str(out))
    assert segs[0][1:] == (0.0, 0.0) and segs[-1][1:] == (0.0, 0.0)
    for d, v, w in segs:
        assert abs(d / 0.05 - round(d / 0.05)) < 1e-9
        assert d >= 0.25
        assert abs(v) <= ms.DEFAULT_VMAX + 1e-9 and abs(w) <= ms.DEFAULT_WMAX + 1e-9
        assert abs(v) + abs(w) * 0.23 / 2 <= 0.9 * 0.7 + 1e-9
    assert ms.validate(segs, box) == []
    extent, (x, y, _) = euler_path(segs)
    assert extent <= box + 0.01
    assert math.hypot(x, y) < 0.05           # ends near the start

    total = sum(d for d, _, _ in segs)
    t = 0.0
    while t < total:                          # command_at works over the whole sequence
        assert command_at(segs, t) is not None
        t += 0.05
    assert command_at(segs, total + 0.01) is None


def test_deterministic(tmp_path):
    a = ms.generate(box=1.5, seed=3)[0]
    b = ms.generate(box=1.5, seed=3)[0]
    c = ms.generate(box=1.5, seed=4)[0]
    assert a == b
    assert a != c


def test_contains_designed_blocks():
    segs = ms.generate(box=1.5, seed=0)[0]
    moves = {(v, w) for _, v, w in segs}
    for k in ms.LEVELS:
        assert (0.0, k) in moves and (0.0, -k) in moves                    # turn in place
        assert (round(0.2 * k, 4), 0.0) in moves and (round(-0.2 * k, 4), 0.0) in moves  # straight
    assert (0.1, 0.3) in moves and (-0.1, -0.3) in moves                  # arc and its way back
    assert (0.1, 0.5) in moves                                            # sysid combined step
    assert len(segs) > 100


def test_random_block_duration():
    segs, blocks, _, _ = ms.generate(box=1.5, seed=0)
    starts = dict((label, i) for i, label in blocks)
    random_segs = segs[starts['round 1: random']:starts['round 1: return to start']]
    total = sum(d for d, _, _ in random_segs)
    assert ms.RANDOM_DURATION <= total <= ms.RANDOM_DURATION + 3.0
    assert any(v == 0.0 and w == 0.0 for _, v, w in random_segs)          # some random rests


def test_sysid_steps_match_yaml():
    with open(SYSID) as f:
        data = yaml.safe_load(f)['segments']
    expected = [(float(s['duration']), float(s.get('v', 0.0)), float(s.get('w', 0.0))) for s in data]
    assert ms.SYSID_STEPS == expected[1:]


def test_rounds_scale_duration():
    one = sum(d for d, _, _ in ms.generate(box=1.5, rounds=1)[0])
    three = sum(d for d, _, _ in ms.generate(box=1.5, rounds=3)[0])
    assert three == pytest.approx(3 * one, rel=0.05)


def test_box_too_small_fails(tmp_path, capsys):
    assert ms.main(['--box', '0.3', '--out', str(tmp_path / 'x.yaml')]) == 1
    assert 'box' in capsys.readouterr().err
    assert not (tmp_path / 'x.yaml').exists()


def test_wheel_speed_limit_enforced():
    with pytest.raises(ValueError, match='wheel speed'):
        ms.generate(vmax=0.2, wmax=1.0, box=1.5, wheel_speed_limit=0.2)   # 0.2 m/s > 0.9 x 0.2


def test_validate_catches_problems():
    assert any('multiple' in p for p in ms.validate([(0.0, 0, 0), (0.33, 0, 0), (1.0, 0, 0)], 1.0))
    assert any('below' in p for p in ms.validate([(1.0, 0, 0), (0.2, 0, 0), (1.0, 0, 0)], 1.0))
    assert any('box' in p for p in ms.validate([(1.0, 0, 0), (10.0, 0.2, 0), (1.0, 0, 0)], 1.0))
    assert any('rests' in p for p in ms.validate([(1.0, 0.1, 0)], 1.0))
    assert any('wheel' in p for p in ms.validate([(1.0, 0, 0), (1.0, 0.6, 1.0), (1.0, 0, 0)], 10.0))
