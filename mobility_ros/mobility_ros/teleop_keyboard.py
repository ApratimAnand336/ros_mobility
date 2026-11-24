#!/usr/bin/python3
"""Keyboard teleop in a terminal (fallback; prefer teleop_key_window, input:=keyboard).

  w/s  forward/back       a/d  turn (ackermann) · strafe (crab) · steer aim (explicit)
  c    straighten the explicit aim
  1/2/3  ackermann/crab/explicit     m  cycle mode
  +/-  speed scale        space  stop       x  toggle e-stop      Ctrl-C  quit

Limitation: a terminal auto-repeats only the last key pressed, so holding w and then
pressing a stops w repeating, and w counts as released key_hold_time later. Use the
key window (teleop_key_window) to hold several keys at once.

Keys are binary, so each key sets a full-scale *target*; the shared slew limiter
(accel/decel params) ramps the actual command. Terminals send no key-up events: a key
counts as released when no repeat has arrived for key_hold_time. The first auto-repeat
only comes after the OS repeat delay (~250-600 ms), so key_hold_time must exceed that.
"""
import select
import sys
import termios
import tty

from rclpy.node import Node

from . import common as C
from .teleop_common import TeleopCore

HELP = __doc__.split("\n\n")[1]
AXIS_KEYS = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1)}   # (fwd | turn, sign)


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.hold = C.param(self, "key_hold_time", 0.6)    # s
        rate = C.param(self, "update_rate", 50.0)
        self.core = TeleopCore(self)
        self.pressed = {}    # key -> last time seen
        if not sys.stdin.isatty():
            raise RuntimeError("teleop_keyboard needs a terminal: run it with "
                               "`ros2 run mobility_ros teleop_keyboard`")
        self.old_tty = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(HELP, flush=True)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while select.select([sys.stdin], [], [], 0.0)[0]:
            k = sys.stdin.read(1).lower()
            if k in AXIS_KEYS:
                ax = AXIS_KEYS[k][0]
                # Opposite key on the same axis cancels the other immediately.
                for other, (ax2, _) in AXIS_KEYS.items():
                    if ax2 == ax and other != k:
                        self.pressed.pop(other, None)
                self.pressed[k] = now
            elif k == " ":
                self.pressed.clear()
                self.core.stop()
            elif k == "c":
                self.core.recentre_on()
            elif k in "123":
                self.core.set_mode(self.core.modes[int(k) - 1]
                                   if int(k) <= len(self.core.modes) else self.core.mode)
            elif k == "m":
                self.core.cycle_mode()
            elif k in "+=":
                self.core.change_scale(+1)
            elif k in "-_":
                self.core.change_scale(-1)
            elif k == "x":
                self.core.toggle_estop()
        self.pressed = {k: t for k, t in self.pressed.items() if now - t < self.hold}
        axes = [0.0, 0.0]          # fwd, turn
        for k in self.pressed:
            ax, sign = AXIS_KEYS[k]
            axes[ax] = float(sign)
        self.core.drive_keys(axes[0], axes[1], self.dt)

    def on_shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_tty)
        self.core.publish((0.0, 0.0, 0.0), self.dt, enabled=False)


def main():
    C.run(TeleopKeyboard)


if __name__ == "__main__":
    main()
