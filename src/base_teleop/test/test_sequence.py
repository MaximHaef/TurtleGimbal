import os

import pytest

from base_teleop.sequence import command_at, load_sequence

EXAMPLE = os.path.join(os.path.dirname(__file__), '..', 'config', 'steps_example.yaml')


def test_example_loads():
    segments = load_sequence(EXAMPLE)
    assert len(segments) > 0
    assert all(d > 0 for d, _, _ in segments)


def test_command_at():
    segs = [(1.0, 0.0, 0.0), (2.0, 0.2, 0.1)]
    assert command_at(segs, -0.1) is None
    assert command_at(segs, 0.0) == (0, 0.0, 0.0)
    assert command_at(segs, 1.5) == (1, 0.2, 0.1)
    assert command_at(segs, 3.0) is None


def test_bad_duration(tmp_path):
    p = tmp_path / 's.yaml'
    p.write_text('segments:\n  - {duration: 0, v: 0.1}\n')
    with pytest.raises(ValueError):
        load_sequence(str(p))
