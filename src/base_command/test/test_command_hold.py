import pytest

from base_command.command_hold import CommandHold


def test_zero_before_first_request():
    assert CommandHold(0.5).get(10.0) == (0.0, 0.0, False)


def test_holds_within_timeout():
    h = CommandHold(0.5)
    h.update(0.2, 0.3, 1.0)
    assert h.get(1.0) == (0.2, 0.3, True)
    assert h.get(1.5) == (0.2, 0.3, True)


def test_zero_after_timeout():
    h = CommandHold(0.5)
    h.update(0.2, 0.3, 1.0)
    assert h.get(1.51) == (0.0, 0.0, False)


def test_new_request_refreshes():
    h = CommandHold(0.5)
    h.update(0.2, 0.3, 1.0)
    h.update(-0.1, 0.0, 1.4)
    assert h.get(1.8) == (-0.1, 0.0, True)


def test_invalid_timeout():
    with pytest.raises(ValueError):
        CommandHold(0.0)
