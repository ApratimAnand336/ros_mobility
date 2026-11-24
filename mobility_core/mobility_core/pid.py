"""PID with feedforward, for the 8 wheel loops. Pure Python, no ROS.

Output is a normalised effort u in [-out_limit, out_limit] (out_limit <= 1): PWM duty
on the real rover, converted to torque by a motor model in sim. Same code everywhere;
only the gains (in YAML) differ per domain.

    u = ff + kp*e + I + D
    ff = kv*sp + ks*sign(sp) + kr*d(sp)/dt   (drive: kv, ks; steer: kr on the setpoint rate)
    D  = -kd * d(measurement)/dt      (derivative on measurement, low-pass filtered)
    I  += ki*e*dt, only while that does not push a saturated output further
         (conditional anti-windup), and clamped to +/- i_limit.

hold_i_limit: when the setpoint is (near) zero, the integrator stays live but is
clamped to this smaller bound, so a parked wheel brakes against a slope with a
bounded effort instead of rolling away (ported from the Rover_control fix).
"""
import math
from dataclasses import dataclass

from .slew import clamp, wrap_angle


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    kv: float = 0.0              # feedforward per unit setpoint
    ks: float = 0.0              # static-friction feedforward, * sign(setpoint)
    kr: float = 0.0              # feedforward per unit setpoint *rate* (steer: duty per rad/s)
    out_limit: float = 1.0       # |u| max
    i_limit: float = 1.0         # |I| max
    hold_i_limit: float = None   # |I| max at zero setpoint (None = same as i_limit)
    d_filter_tau: float = 0.02   # s, first-order low-pass on the D term (0 = off)
    slew_rate: float = 0.0       # |du/dt| max in 1/s (0 = off)
    deadzone: float = 0.0        # |error| below this counts as 0 (steer chatter)
    zero_setpoint_eps: float = 1e-3
    angular: bool = False        # wrap the error to [-pi, pi] (steer loops)


class PID:
    def __init__(self, gains: PIDGains):
        self.g = gains
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_meas = None
        self.d_state = 0.0
        self.output = 0.0
        self.saturated = False
        self.error = 0.0

    def update(self, setpoint, measurement, dt, measured_rate=None, setpoint_rate=0.0):
        g = self.g
        if dt <= 0.0 or not (math.isfinite(setpoint) and math.isfinite(measurement)):
            return self.output
        e = setpoint - measurement
        if g.angular:
            e = wrap_angle(e)
        if abs(e) < g.deadzone:
            e = 0.0
        self.error = e

        # Derivative on measurement (no kick on setpoint steps), filtered.
        if measured_rate is None:
            measured_rate = 0.0 if self.prev_meas is None else (
                (wrap_angle(measurement - self.prev_meas) if g.angular
                 else measurement - self.prev_meas) / dt)
        self.prev_meas = measurement
        d_raw = -g.kd * measured_rate
        if g.d_filter_tau > 0.0:
            a = dt / (g.d_filter_tau + dt)
            self.d_state += a * (d_raw - self.d_state)
        else:
            self.d_state = d_raw

        sign = 0.0 if abs(setpoint) < g.zero_setpoint_eps else math.copysign(1.0, setpoint)
        ff = g.kv * setpoint + g.ks * sign + g.kr * setpoint_rate
        p = g.kp * e

        # Conditional integration: skip if it would push a saturated output further.
        i_lim = g.i_limit
        if sign == 0.0 and g.hold_i_limit is not None:
            i_lim = g.hold_i_limit
        candidate = clamp(self.integral + g.ki * e * dt, -g.i_limit, g.i_limit)
        u_try = ff + p + candidate + self.d_state
        if abs(u_try) < g.out_limit or u_try * e < 0.0:
            self.integral = candidate
        self.integral = clamp(self.integral, -i_lim, i_lim)

        u_raw = ff + p + self.integral + self.d_state
        u = clamp(u_raw, -g.out_limit, g.out_limit)
        self.saturated = abs(u_raw) >= g.out_limit
        if g.slew_rate > 0.0:
            step = g.slew_rate * dt
            u = clamp(u, self.output - step, self.output + step)
        self.output = u
        return u
