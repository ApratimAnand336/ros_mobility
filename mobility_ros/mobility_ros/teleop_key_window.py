#!/usr/bin/python3
"""Keyboard teleop in its own window: hold several keys at once, real key releases.

Port of the team's rover_gazebo/teleop_rover_gui.py onto the mobility stack. It
publishes /cmd_vel (TwistStamped), /mobility/steer_mode and /mobility/estop through the
same TeleopCore as the joystick, so the speed ramps and limits are identical.

  W/S        forward / reverse          (hold W+A to drive an arc)
  A/D        ackermann: turn left/right  crab: strafe left/right
             explicit: steer the shared aim left/right
  C          explicit: sweep the aim back to straight
  Shift      boost (drive speed only)    + / -   speed scale
  Space      stop now                    X       toggle e-stop
  1 2 3      ackermann / crab / explicit M       cycle mode
  Esc        quit

Click into the window once so it has keyboard focus. Keys only count while it has
focus; losing focus releases everything.
"""
import math
import signal
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from . import common as C
from .key_state import KeyState
from .teleop_common import TeleopCore

DRIVE_KEYS = ("w", "a", "s", "d", "shift")
HELP = ("W/S drive   A/D turn · strafe · steer (by mode)   Shift boost\n"
        "C straighten aim   Space stop   X e-stop   +/- speed\n"
        "1 Ackermann   2 Crab   3 Explicit   M cycle   Esc quit")

# Neutral palette for the window (Tk has no theme tokens; fixed, readable on both).
BG, FG, DIM, ACCENT, WARN, BAD = "#161d22", "#e3e9ed", "#5e6b74", "#4fc2b9", "#f0a54a", "#ff6b5f"


class TeleopKeyWindow(Node):
    def __init__(self):
        super().__init__("teleop_key_window")
        self.core = TeleopCore(self)
        self.dt = 1.0 / C.param(self, "update_rate", 50.0)
        debounce = C.param(self, "key_release_debounce_ms", 250) / 1000.0
        self.keys = KeyState(DRIVE_KEYS, debounce)
        self.stop_requested = False
        self.has_focus = False

        self.root = tk.Tk()
        self.root.title("Rover teleop")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.request_stop)
        self._build_ui()
        self.root.bind("<KeyPress>", self._on_press)
        self.root.bind("<KeyRelease>", self._on_release)
        self.root.bind("<FocusIn>", self._on_focus_in)
        self.root.bind("<FocusOut>", self._on_focus_out)
        self.root.focus_force()
        self.root.after(int(self.dt * 1000), self._tick)

    # -- UI --------------------------------------------------------------------
    def _label(self, parent, **kw):
        kw.setdefault("bg", BG)
        kw.setdefault("fg", FG)
        return tk.Label(parent, **kw)

    def _build_ui(self):
        pad = {"padx": 14, "pady": 3, "anchor": "w"}
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=14, pady=(12, 4))
        self.mode_var = tk.StringVar()
        self._label(top, textvariable=self.mode_var,
                    font=("TkDefaultFont", 17, "bold")).pack(side="left")
        self.estop_var = tk.StringVar()
        self.estop_lbl = self._label(top, textvariable=self.estop_var,
                                     font=("TkDefaultFont", 11, "bold"))
        self.estop_lbl.pack(side="right")

        keys = tk.Frame(self.root, bg=BG)
        keys.pack(padx=14, pady=8, anchor="w")
        self.key_lbl = {}
        for k in DRIVE_KEYS:
            lbl = self._label(keys, text="SHIFT" if k == "shift" else k.upper(),
                              font=("TkFixedFont", 13, "bold"), width=7 if k == "shift" else 4,
                              relief="groove", bd=2, fg=DIM)
            lbl.pack(side="left", padx=3)
            self.key_lbl[k] = lbl

        self.cmd_var = tk.StringVar()
        self._label(self.root, textvariable=self.cmd_var,
                    font=("TkFixedFont", 11)).pack(**pad)
        self.aim_var = tk.StringVar()
        self._label(self.root, textvariable=self.aim_var,
                    font=("TkFixedFont", 11), fg=ACCENT).pack(**pad)
        self._label(self.root, text=HELP, justify="left", fg=DIM,
                    font=("TkDefaultFont", 9)).pack(padx=14, pady=(6, 2), anchor="w")
        self.focus_var = tk.StringVar()
        self.focus_lbl = self._label(self.root, textvariable=self.focus_var,
                                     font=("TkDefaultFont", 10, "italic"))
        self.focus_lbl.pack(padx=14, pady=(4, 12), anchor="w")
        self.root.minsize(470, 0)

    def _refresh(self):
        c = self.core
        self.mode_var.set(f"{c.mode.capitalize()}   ·   speed ×{c.scale:.1f}")
        self.estop_var.set("E-STOP" if c.estop else "")
        self.estop_lbl.configure(fg=BAD)
        for k, lbl in self.key_lbl.items():
            lbl.configure(fg=ACCENT if self.keys.down[k] else DIM)
        vx, vy, wz = c.last_cmd
        unit = "rad/s steer" if c.mode == "explicit" else "rad/s yaw"
        self.cmd_var.set(f"vx {vx:+.2f} m/s   vy {vy:+.2f} m/s   wz {wz:+.2f} {unit}")
        if c.mode == "explicit":
            sweep = "  (sweeping to straight)" if c.recentre else ""
            self.aim_var.set(f"aim {math.degrees(c.aim):+5.1f}°{sweep}")
        else:
            self.aim_var.set("")
        if self.has_focus:
            self.focus_var.set("Focused: drive with the keyboard")
            self.focus_lbl.configure(fg=DIM)
        else:
            self.focus_var.set("NOT FOCUSED: click this window to drive")
            self.focus_lbl.configure(fg=WARN)

    # -- events ----------------------------------------------------------------
    @staticmethod
    def _key(event):
        low = event.keysym.lower()
        return "shift" if low in ("shift_l", "shift_r") else low

    def _on_press(self, event):
        k, now = self._key(event), time.monotonic()
        if k in DRIVE_KEYS:
            self.keys.press(k, now)
        elif k in ("1", "2", "3") and int(k) <= len(self.core.modes):
            self.keys.clear(["w", "a", "s", "d"])
            self.core.set_mode(self.core.modes[int(k) - 1])
        elif k == "m":
            self.keys.clear(["w", "a", "s", "d"])
            self.core.cycle_mode()
        elif k == "c":
            self.core.recentre_on()
        elif k == "space":
            self.keys.clear(["w", "a", "s", "d"])
            self.core.stop()
        elif k == "x":
            self.keys.clear(["w", "a", "s", "d"])
            self.core.toggle_estop()
        elif k in ("plus", "equal", "kp_add"):
            self.core.change_scale(+1)
        elif k in ("minus", "underscore", "kp_subtract"):
            self.core.change_scale(-1)
        elif k == "escape":
            self.request_stop()

    def _on_release(self, event):
        k = self._key(event)
        if k in DRIVE_KEYS:
            self.keys.release(k, time.monotonic())

    def _on_focus_in(self, event):
        if event.widget is self.root:
            self.has_focus = True

    def _on_focus_out(self, event):
        if event.widget is self.root:
            self.has_focus = False
            self.keys.release_all()

    # -- loop --------------------------------------------------------------------
    def request_stop(self, *_):
        self.stop_requested = True

    def _tick(self):
        if self.stop_requested or not rclpy.ok():
            self.root.quit()
            return
        rclpy.spin_once(self, timeout_sec=0.0)   # deliver steer_aim, run timers
        self.keys.update(time.monotonic())
        fwd = self.keys.axis("w", "s")
        turn = self.keys.axis("a", "d")
        self.core.drive_keys(fwd, turn, self.dt, boost=self.keys.down["shift"])
        self._refresh()
        self.root.after(int(self.dt * 1000), self._tick)

    def shutdown(self):
        # Stop the rover directly: zero command, no ramp.
        self.core.stop()
        self.core.publish((0.0, 0.0, 0.0), self.dt, enabled=False)
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopKeyWindow()
    # Ctrl-C / ros2 launch shutdown: ask the Tk loop to finish on its next tick.
    signal.signal(signal.SIGINT, node.request_stop)
    signal.signal(signal.SIGTERM, node.request_stop)
    try:
        node.root.mainloop()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
