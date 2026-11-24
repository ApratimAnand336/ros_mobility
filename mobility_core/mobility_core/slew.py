"""Rate limiting and angle helpers. Pure Python, no ROS."""
import math


def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def ramp(current, target, max_delta):
    """Move current toward target by at most max_delta."""
    if target > current:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def angle_ramp(current, target, max_delta):
    """Move an angle toward target by at most max_delta, the short way round.

    Not wrapped on output: steering joints have hard limits, so the caller keeps the
    angle inside those limits rather than letting it wrap through +/-pi.
    """
    diff = clamp(wrap_angle(target - current), -max_delta, max_delta)
    return current + diff


class SlewLimiter:
    """First-difference limiter: |dy/dt| <= rate. Separate up/down rates let a
    command brake faster than it accelerates (decel_rate defaults to rate)."""

    def __init__(self, rate, decel_rate=None, initial=0.0):
        self.rate = rate
        self.decel_rate = rate if decel_rate is None else decel_rate
        self.value = initial

    def reset(self, value=0.0):
        self.value = value

    def step(self, target, dt):
        # "Decelerating" = moving toward zero, or crossing it.
        moving_toward_zero = abs(target) < abs(self.value) or target * self.value < 0.0
        rate = self.decel_rate if moving_toward_zero else self.rate
        if rate <= 0.0:
            self.value = target
        else:
            self.value = ramp(self.value, target, rate * dt)
        return self.value
