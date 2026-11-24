import pytest

from mobility_hw.can_protocol import (CanConfig, decode_duties, encode_duties,
                                      encode_heartbeat, shape_duty)


def test_shape_duty_zero_stays_zero():
    assert shape_duty(0.0, deadband=0.1) == 0.0


def test_shape_duty_deadband_lift_and_sign():
    assert shape_duty(0.01, deadband=0.1) == pytest.approx(0.1 + 0.9 * 0.01)
    assert shape_duty(-1.0, deadband=0.1) == pytest.approx(-1.0)


def test_shape_duty_max_clamp():
    assert shape_duty(1.0, deadband=0.0, max_duty=0.8) == pytest.approx(0.8)


def test_encode_roundtrip():
    cfg = CanConfig()
    data = encode_duties(cfg, [1.0, -1.0, 0.5, 0.0])
    assert len(data) == 8
    assert decode_duties(cfg, data) == pytest.approx([1.0, -1.0, 0.5, 0.0], abs=1e-4)


def test_big_endian_option():
    le = encode_duties(CanConfig(little_endian=True), [0.5, 0, 0, 0])
    be = encode_duties(CanConfig(little_endian=False), [0.5, 0, 0, 0])
    assert le[:2] == be[1::-1]


def test_heartbeat_flags():
    hb = encode_heartbeat(300, active=True, estop=True)
    assert hb[0] == 300 & 0xFF and hb[1] == 0b11 and len(hb) == 8
