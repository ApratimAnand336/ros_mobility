"""Swerve (4-wheel independent steer) kinematics. Pure Python, no ROS.

Inverse kinematics (body command -> per-wheel steer angle + wheel speed) is ported
from rover_gazebo/rover_kinematics_node.py so sim and real behave the same:

  * ackermann / crab ask for a body motion and solve each wheel from the
    velocity field  v_wheel = v_body + omega x r.
  * explicit inverts that: all wheels share one aim angle; angular.z is a
    steering *rate* integrated into the aim, linear.x is one wheel speed for all.

On top of the reference behaviour:
  * the 180-degree fold only picks a target the steering joint can actually reach;
  * a zero command holds the wheel angles instead of swinging them to 0;
  * forward kinematics (least-squares body twist from all wheels) for odometry.

All angles/speeds here are canonical (see geometry.py). Domain sign flips live in
the hardware adapters, never here.
"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import Geometry
from .slew import angle_ramp, clamp, ramp, wrap_angle

MODES = ("ackermann", "crab", "explicit")


@dataclass
class KinematicsLimits:
    max_speed: float = 1.0                  # m/s, |vx|, |vy|
    max_yaw_rate: float = 1.0               # rad/s
    min_turn_radius: float = 0.8            # m, ackermann only
    max_wheel_turn_rate: float = 2.0        # rad/s, how fast a solved angle may move
    speed_ramp_time: float = 1.2            # s, standstill -> max_speed
    steer_ramp_time: float = 0.8            # s, -max_yaw_rate -> +max_yaw_rate
    explicit_max_steer_angle: float = 1.5708  # rad, explicit-mode aim clamp
    explicit_max_steer_rate: float = 1.0      # rad/s, explicit-mode aim rate clamp
    hold_epsilon: float = 1e-3              # m/s, below this a wheel holds its angle


class SwerveKinematics:
    """Stateful IK: call step() at a fixed rate with the latest body command."""

    def __init__(self, geometry: Geometry, limits: KinematicsLimits = None,
                 mode: str = "ackermann"):
        self.geo = geometry
        self.lim = limits or KinematicsLimits()
        self.n = len(geometry.stations)
        self.have = [0.0, 0.0, 0.0]          # ramped vx, vy, wz
        self.wheel_angle = [0.0] * self.n    # current steer setpoints (canonical)
        self.aim = 0.0                       # explicit-mode shared angle
        self.zeroing = False
        self.mode = "ackermann"
        self.set_mode(mode)

    # -- state -------------------------------------------------------------
    def seed(self, angles):
        """Start the setpoints from measured steer angles (avoids a startup snap)."""
        lim = self.geo.steer_limit
        self.wheel_angle = [clamp(float(a), -lim, lim) for a in angles]
        self.aim = self.wheel_angle[0]

    def set_mode(self, name):
        name = name.strip().lower()
        if name not in MODES:
            raise ValueError(f"unknown steer mode '{name}', expected one of {MODES}")
        if name == "explicit" and self.mode != "explicit":
            # Take over from where the front-left wheel is, then sweep to straight.
            self.aim = self.wheel_angle[0]
            self.zeroing = True
        self.mode = name

    def reset_motion(self):
        """Zero the ramped body command (used on e-stop / disarm)."""
        self.have = [0.0, 0.0, 0.0]

    # -- inverse kinematics ------------------------------------------------
    def step(self, cmd, dt):
        """cmd = (vx, vy, wz). Returns (angles[rad], wheel_speeds[rad/s]), canonical."""
        vx_t, vy_t, wz_t = (float(c) for c in cmd)
        if not all(math.isfinite(c) for c in (vx_t, vy_t, wz_t)):
            vx_t = vy_t = wz_t = 0.0
        L = self.lim
        top = L.max_speed
        lin_step = top / max(0.01, L.speed_ramp_time) * dt
        ang_step = 2.0 * L.max_yaw_rate / max(0.01, L.steer_ramp_time) * dt
        self.have[0] = ramp(self.have[0], clamp(vx_t, -top, top), lin_step)
        self.have[1] = ramp(self.have[1], clamp(vy_t, -top, top), lin_step)

        if self.mode == "explicit":
            self.have[2] = 0.0
            return self._explicit_step(self.have[0], wz_t, dt)

        self.have[2] = ramp(self.have[2], clamp(wz_t, -L.max_yaw_rate, L.max_yaw_rate),
                            ang_step)
        vx, vy, w = self.have
        if self.mode == "ackermann":
            vy = 0.0
            cap = abs(vx) / max(1e-6, L.min_turn_radius)
            w = clamp(w, -cap, cap)
        else:  # crab: pure translation
            w = 0.0

        angles, speeds = [], []
        for i, st in enumerate(self.geo.stations):
            wx, wy = vx - w * st.y, vy + w * st.x
            current = self.wheel_angle[i]
            if math.hypot(wx, wy) < L.hold_epsilon:
                target = current      # nothing asked of this wheel: hold, don't snap
            else:
                target = self._fold(math.atan2(wy, wx), current)
            angle = angle_ramp(current, target, L.max_wheel_turn_rate * dt)
            angle = clamp(angle, -self.geo.steer_limit, self.geo.steer_limit)
            self.wheel_angle[i] = angle
            # Project the wanted ground velocity onto where the wheel points: a wheel
            # mid-turn drives less, and a folded (reversed) wheel gets a negative speed.
            speed = wx * math.cos(angle) + wy * math.sin(angle)
            angles.append(angle)
            speeds.append(speed / self.geo.wheel_radius)
        return angles, speeds

    def _fold(self, raw, current):
        """Pick raw or raw+/-pi: reachable within the steer limit and closest to current."""
        lim = self.geo.steer_limit
        best, best_cost = None, None
        for cand in (raw, raw + math.pi, raw - math.pi):
            c = current + wrap_angle(cand - current)
            for c2 in (c, c + 2 * math.pi, c - 2 * math.pi):
                cost = abs(c2 - current)
                if abs(c2) > lim + 1e-9:
                    cost += 10.0       # unreachable: only if nothing else is
                if best_cost is None or cost < best_cost:
                    best, best_cost = c2, cost
        return clamp(best, -lim, lim)

    def _explicit_step(self, speed, rate, dt):
        L = self.lim
        rate = clamp(rate, -L.explicit_max_steer_rate, L.explicit_max_steer_rate)
        if rate != 0.0:
            self.zeroing = False        # operator input takes over the entry sweep
        lim = min(L.explicit_max_steer_angle, self.geo.steer_limit)
        if self.zeroing:
            self.aim = ramp(self.aim, 0.0, L.explicit_max_steer_rate * dt)
            if abs(self.aim) < 1e-3:
                self.aim, self.zeroing = 0.0, False
        else:
            self.aim = clamp(self.aim + rate * dt, -lim, lim)
        # Wheels may not all be at the aim yet (entering from ackermann): sweep each.
        for i in range(self.n):
            self.wheel_angle[i] = angle_ramp(self.wheel_angle[i], self.aim,
                                             L.max_wheel_turn_rate * dt)
        wheel_speed = speed / self.geo.wheel_radius
        return list(self.wheel_angle), [wheel_speed] * self.n


# -- forward kinematics ----------------------------------------------------
def body_twist_from_wheels(geometry: Geometry, angles, wheel_speeds):
    """Least-squares rigid-body fit: returns (vx, vy, wz) from all wheels.

    Each wheel gives two equations:  vx - wz*y = v_i cos(th_i),  vy + wz*x = v_i sin(th_i)
    """
    rows, rhs = [], []
    for st, th, om in zip(geometry.stations, angles, wheel_speeds):
        v = om * geometry.wheel_radius
        rows.append([1.0, 0.0, -st.y])
        rhs.append(v * math.cos(th))
        rows.append([0.0, 1.0, st.x])
        rhs.append(v * math.sin(th))
    sol, *_ = np.linalg.lstsq(np.asarray(rows), np.asarray(rhs), rcond=None)
    return float(sol[0]), float(sol[1]), float(sol[2])
