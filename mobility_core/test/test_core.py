import math

import pytest

from mobility_core import (DEFAULT_GEOMETRY, JointPlant, KinematicsLimits, MotorParams, PID,
                           PIDGains, PlantParams, SlewLimiter, SwerveKinematics,
                           body_twist_from_wheels, motor_torque, wrap_angle)
from mobility_core.controller import WheelController

GEO = DEFAULT_GEOMETRY
DT = 0.01


def settle(kin, cmd, seconds):
    out = None
    for _ in range(int(seconds / DT)):
        out = kin.step(cmd, DT)
    return out


# -- kinematics --------------------------------------------------------------
def test_straight_forward():
    a, s = settle(SwerveKinematics(GEO), (0.5, 0.0, 0.0), 3.0)
    assert all(abs(x) < 1e-6 for x in a)
    assert all(abs(v - 0.5 / GEO.wheel_radius) < 1e-6 for v in s)


def test_reverse_does_not_flip_wheels():
    # atan2(0, -v) = pi; the fold must keep the wheels straight and drive backwards.
    a, s = settle(SwerveKinematics(GEO), (-0.5, 0.0, 0.0), 3.0)
    assert all(abs(x) < 1e-6 for x in a)
    assert all(v < 0 for v in s)


def test_crab_left_all_wheels_parallel():
    a, s = settle(SwerveKinematics(GEO, mode="crab"), (0.0, 0.5, 0.0), 3.0)
    assert all(abs(x - math.pi / 2) < 1e-3 for x in a)
    assert all(v > 0 for v in s)


def test_crab_right_stays_within_steer_limit():
    a, s = settle(SwerveKinematics(GEO, mode="crab"), (0.0, -0.5, 0.0), 3.0)
    assert all(abs(x) <= GEO.steer_limit + 1e-9 for x in a)
    assert all(abs(abs(x) - math.pi / 2) < 1e-3 for x in a)


def test_ackermann_left_turn_counter_steers():
    a, _ = settle(SwerveKinematics(GEO), (0.5, 0.0, 0.5), 3.0)
    fl, fr, bl, br = a
    assert fl > 0 and fr > 0 and bl < 0 and br < 0


def test_ackermann_respects_min_turn_radius():
    kin = SwerveKinematics(GEO, KinematicsLimits(min_turn_radius=0.8))
    _, s = settle(kin, (0.2, 0.0, 5.0), 3.0)
    vx, vy, wz = body_twist_from_wheels(GEO, kin.wheel_angle, s)
    assert abs(wz) <= 0.2 / 0.8 + 1e-3


def test_zero_command_holds_angle():
    kin = SwerveKinematics(GEO, mode="crab")
    settle(kin, (0.3, 0.3, 0.0), 3.0)
    held = list(kin.wheel_angle)
    a, s = settle(kin, (0.0, 0.0, 0.0), 3.0)
    assert a == pytest.approx(held)
    assert all(abs(v) < 1e-9 for v in s)


def test_steer_rate_limited():
    kin = SwerveKinematics(GEO, KinematicsLimits(max_wheel_turn_rate=2.0), mode="crab")
    prev = list(kin.wheel_angle)
    for _ in range(100):
        a, _ = kin.step((0.0, 0.5, 0.0), DT)
        assert all(abs(x - p) <= 2.0 * DT + 1e-9 for x, p in zip(a, prev))
        prev = a


def test_nan_command_is_zero():
    a, s = SwerveKinematics(GEO).step((float("nan"), 0.0, 0.0), DT)
    assert all(v == 0.0 for v in s)


def test_explicit_mode_integrates_aim():
    kin = SwerveKinematics(GEO, mode="explicit")
    settle(kin, (0.0, 0.0, 0.5), 1.0)           # steer rate 0.5 rad/s for 1 s
    assert kin.aim == pytest.approx(0.5, abs=0.02)
    a, s = settle(kin, (0.4, 0.0, 0.0), 2.0)     # rate 0: aim holds, wheels roll
    assert all(x == pytest.approx(kin.aim, abs=1e-6) for x in a)
    assert all(v == pytest.approx(0.4 / GEO.wheel_radius) for v in s)


def test_entering_explicit_sweeps_to_straight():
    kin = SwerveKinematics(GEO, mode="crab")
    settle(kin, (0.0, 0.5, 0.0), 2.0)
    kin.set_mode("explicit")
    a, _ = settle(kin, (0.0, 0.0, 0.0), 4.0)
    assert all(abs(x) < 1e-3 for x in a)


def test_unknown_mode_rejected():
    with pytest.raises(ValueError):
        SwerveKinematics(GEO).set_mode("spot")


def test_forward_kinematics_roundtrip():
    kin = SwerveKinematics(GEO)
    a, s = settle(kin, (0.6, 0.0, 0.4), 4.0)
    vx, vy, wz = body_twist_from_wheels(GEO, a, s)
    assert vx == pytest.approx(0.6, abs=1e-3)
    assert vy == pytest.approx(0.0, abs=1e-3)
    assert wz == pytest.approx(0.4, abs=1e-2)


# -- slew / pid / motor --------------------------------------------------------
def test_slew_limiter():
    sl = SlewLimiter(rate=1.0, decel_rate=4.0)
    for _ in range(50):
        sl.step(1.0, DT)
    assert sl.value == pytest.approx(0.5)
    for _ in range(10):
        sl.step(0.0, DT)
    assert sl.value == pytest.approx(0.1)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_pid_antiwindup():
    pid = PID(PIDGains(kp=1.0, ki=10.0, out_limit=1.0, i_limit=5.0))
    for _ in range(500):
        pid.update(100.0, 0.0, DT)            # hopelessly saturated
    assert pid.integral < 1.0                 # integrator did not run away
    assert pid.output == pytest.approx(1.0)


def test_pid_feedforward_only():
    pid = PID(PIDGains(kv=0.1, ks=0.05))
    assert pid.update(5.0, 5.0, DT) == pytest.approx(0.55)
    assert pid.update(0.0, 0.0, DT) == pytest.approx(0.0)


def test_pid_angular_wraps():
    pid = PID(PIDGains(kp=1.0, angular=True))
    pid.update(math.pi - 0.1, -math.pi + 0.1, DT)
    assert pid.error == pytest.approx(-0.2)


def test_motor_model():
    m = MotorParams()
    assert motor_torque(m, 1.0, 0.0) == pytest.approx(m.stall_torque)
    assert motor_torque(m, 1.0, m.no_load_speed) == pytest.approx(0.0, abs=1e-9)


# -- closed loop: kinematics + 8 PIDs + mock plant ------------------------------
def test_closed_loop_tracks():
    dm, sm = MotorParams(gear_ratio=50), MotorParams(gear_ratio=100)
    steer = PIDGains(kp=3.0, ki=1.0, kd=0.1, i_limit=0.3, d_filter_tau=0.01, deadzone=0.002)
    drive = PIDGains(kp=0.3, ki=1.0, kv=1 / dm.no_load_speed, ks=0.03, i_limit=0.5,
                     hold_i_limit=0.3)
    kin = SwerveKinematics(GEO, mode="crab")
    ctl = WheelController(GEO.names, steer, drive)
    plants = {}
    for w in GEO.names:
        plants[f"{w}_steer"] = JointPlant(sm, PlantParams(0.05, 2.0, 3.0, -1.75, 1.75))
        plants[f"{w}_drive"] = JointPlant(dm, PlantParams(0.3, 0.5, 2.0))
    for _ in range(int(4.0 / DT)):
        a, s = kin.step((0.4, 0.4, 0.0), DT)
        sp = {}
        for i, w in enumerate(GEO.names):
            sp[f"{w}_steer"], sp[f"{w}_drive"] = a[i], s[i]
        u = ctl.update(sp, {j: p.pos for j, p in plants.items()},
                       {j: p.vel for j, p in plants.items()}, DT)
        for j, p in plants.items():
            for _ in range(10):
                p.step(u[j], DT / 10)
    for w in GEO.names:
        assert abs(plants[f"{w}_steer"].pos - math.pi / 4) < math.radians(2)
        want = 0.4 * math.sqrt(2) / GEO.wheel_radius
        assert abs(plants[f"{w}_drive"].vel - want) < 0.05 * want
