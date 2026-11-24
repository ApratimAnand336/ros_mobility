#!/usr/bin/python3
"""Joystick teleop: /joy -> /cmd_vel (TwistStamped), /mobility/steer_mode, /mobility/estop.

Every axis/button index is a parameter (config/teleop_joy.yaml) because the real
joystick model is not decided yet. Set an index to -1 to disable it.

  axis_vx      forward/back          axis_vy     strafe (crab mode)
  axis_wz      yaw rate, or steering rate in explicit mode
  axis_scale   optional throttle axis -> speed scale (-1..1 maps to 0.1..1)
  button_deadman   must be held to move (set require_deadman:=false to skip)
  button_estop     toggles e-stop      button_mode   cycles ackermann/crab/explicit
  button_scale_up / button_scale_down  speed scale steps
"""
from rclpy.node import Node
from sensor_msgs.msg import Joy

from . import common as C
from .teleop_common import TeleopCore


def shape(x, deadband, expo):
    """Deadband with rescale, then expo: out = (1-expo)*x + expo*x^3."""
    if abs(x) < deadband:
        return 0.0
    s = 1.0 if x > 0 else -1.0
    x = s * (abs(x) - deadband) / (1.0 - deadband)
    return (1.0 - expo) * x + expo * x ** 3


class TeleopJoy(Node):
    def __init__(self):
        super().__init__("teleop_joy")
        p = lambda n, d: C.param(self, n, d)  # noqa: E731
        self.axes = {k: p(f"axis_{k}", d) for k, d in
                     (("vx", 1), ("vy", 0), ("wz", 3), ("scale", -1))}
        self.invert = {k: p(f"invert_{k}", False) for k in ("vx", "vy", "wz", "scale")}
        self.buttons = {k: p(f"button_{k}", d) for k, d in
                        (("deadman", 4), ("estop", 1), ("mode", 0),
                         ("scale_up", 5), ("scale_down", -1))}
        self.require_deadman = p("require_deadman", True)
        self.deadband = p("deadband", 0.08)
        self.expo = p("expo", 0.3)
        self.joy_timeout = p("joy_timeout", 0.5)
        rate = p("update_rate", 50.0)
        self.core = TeleopCore(self)
        self.last_joy, self.last_buttons = None, []
        self.targets, self.enabled = (0.0, 0.0, 0.0), False
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.tick)

    def _axis(self, msg, k):
        i = self.axes[k]
        if i < 0 or i >= len(msg.axes):
            return 0.0
        v = msg.axes[i]
        return -v if self.invert[k] else v

    def _pressed(self, msg, k):
        """Rising edge of button k."""
        i = self.buttons[k]
        if i < 0 or i >= len(msg.buttons):
            return False
        prev = self.last_buttons[i] if i < len(self.last_buttons) else 0
        return msg.buttons[i] == 1 and prev == 0

    def _held(self, msg, k):
        i = self.buttons[k]
        return 0 <= i < len(msg.buttons) and msg.buttons[i] == 1

    def on_joy(self, msg):
        if self._pressed(msg, "estop"):
            self.core.toggle_estop()
        if self._pressed(msg, "mode"):
            self.core.cycle_mode()
        if self._pressed(msg, "scale_up"):
            self.core.change_scale(+1)
        if self._pressed(msg, "scale_down"):
            self.core.change_scale(-1)
        if self.axes["scale"] >= 0:
            self.core.scale = 0.1 + 0.9 * (self._axis(msg, "scale") + 1.0) / 2.0
        self.targets = tuple(shape(self._axis(msg, k), self.deadband, self.expo)
                             for k in ("vx", "vy", "wz"))
        self.enabled = self._held(msg, "deadman") or not self.require_deadman
        self.last_buttons = list(msg.buttons)
        self.last_joy = self.get_clock().now()

    def tick(self):
        fresh = self.last_joy is not None and \
            (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9 < self.joy_timeout
        self.core.publish(self.targets, self.dt, enabled=self.enabled and fresh)


def main():
    C.run(TeleopJoy)


if __name__ == "__main__":
    main()
