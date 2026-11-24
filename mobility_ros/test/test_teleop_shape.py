import pytest

from mobility_ros.teleop_joy import shape


def test_deadband():
    assert shape(0.05, 0.08, 0.3) == 0.0


def test_full_scale_preserved():
    assert shape(1.0, 0.08, 0.3) == pytest.approx(1.0)
    assert shape(-1.0, 0.08, 0.3) == pytest.approx(-1.0)


def test_expo_softens_centre():
    assert shape(0.5, 0.0, 0.5) < 0.5
