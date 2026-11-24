"""Shared teleop logic: input targets -> slew-limited body command, mode cycling.

Every front end (key window, terminal, joystick) produces *targets* in [-1, 1] per
axis (vx, vy, wz). This turns them into a smooth TwistStamped so binary keys and an
analogue stick drive identically. In explicit mode the wz axis is a steering rate
(rad/s), not a yaw rate, matching the kinematics node.

Keyboard front ends call drive_keys(fwd, turn): the W/S/A/D layout of the team's
Gazebo teleop, where A/D mean turn (ackermann), strafe (crab) or steer the aim
(explicit), and C sweeps the explicit aim back to straight.
"""
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64, String

from mobility_core import MODES, SlewLimiter, clamp

from . import common as C


class TeleopCore:
    def __init__(self, node):
        self.node = node
        p = lambda n, d: C.param(node, n, d)  # noqa: E731
        self.max_speed = p("max_speed", 1.0)            # m/s at full stick, scale 1
        self.max_yaw_rate = p("max_yaw_rate", 1.0)      # rad/s
        self.max_steer_rate = p("max_steer_rate", 0.5)  # rad/s, explicit mode
        self.scale = p("speed_scale", 0.5)              # start at half speed
        self.scale_step = p("speed_scale_step", 0.1)
        self.boost = p("boost", 2.0)                     # Shift multiplier, drive speed only
        self.recentre_gain = p("recentre_gain", 3.0)     # 1/s, C sweep: rate = -gain * aim
        accel = p("accel", 1.0)                          # m/s^2
        decel = p("decel", 2.0)                          # m/s^2 (braking)
        yaw_accel = p("yaw_accel", 2.0)                  # rad/s^2
        self.modes = list(p("modes", list(MODES)))
        self.mode = p("initial_mode", "ackermann")
        self.frame_id = p("frame_id", "base_link")
        self.lim = [SlewLimiter(accel, decel), SlewLimiter(accel, decel),
                    SlewLimiter(yaw_accel, yaw_accel * 2)]
        self.estop = False
        self.aim = 0.0            # rad, explicit-mode aim echoed by the kinematics node
        self.recentre = False     # explicit mode: sweeping the aim back to straight
        self.last_cmd = (0.0, 0.0, 0.0)
        node.create_subscription(Float64, C.STEER_AIM, self._on_aim, 10)
        self.cmd_pub = node.create_publisher(TwistStamped, C.CMD_VEL, 10)
        self.mode_pub = node.create_publisher(String, C.STEER_MODE, C.LATCHED)
        self.estop_pub = node.create_publisher(Bool, C.ESTOP, C.LATCHED)
        self.mode_pub.publish(String(data=self.mode))
        self.estop_pub.publish(Bool(data=False))
        # While *our* e-stop is engaged, repeat it at 2 Hz so a node that (re)starts late
        # still sees it. Never repeat "released" or the mode: that would override an
        # e-stop or mode change coming from another source (CLI, a hardware button node).
        node.create_timer(0.5, self._repeat_estop)

    def _on_aim(self, msg):
        self.aim = msg.data

    def _repeat_estop(self):
        if self.estop:
            self.estop_pub.publish(Bool(data=True))

    # -- events --------------------------------------------------------------
    def set_mode(self, name):
        if name in self.modes and name != self.mode:
            self.mode = name
            self.recentre = False
            for sl in self.lim:
                sl.reset()
            self.mode_pub.publish(String(data=name))
            self.node.get_logger().info(f"mode -> {name}")

    def cycle_mode(self):
        i = self.modes.index(self.mode) if self.mode in self.modes else -1
        self.set_mode(self.modes[(i + 1) % len(self.modes)])

    def stop(self):
        """Space: zero the command at once (no ramp) and cancel a recentre sweep."""
        self.recentre = False
        for sl in self.lim:
            sl.reset()

    def recentre_on(self):
        """C, explicit mode only: sweep the shared aim back to straight ahead."""
        if self.mode == "explicit":
            self.recentre = True

    def change_scale(self, direction):
        self.scale = clamp(self.scale + direction * self.scale_step, 0.1, 1.0)
        self.node.get_logger().info(f"speed scale {self.scale:.1f}")

    def toggle_estop(self):
        self.set_estop(not self.estop)

    def set_estop(self, on):
        self.estop = on
        if on:
            for sl in self.lim:
                sl.reset()
        self.estop_pub.publish(Bool(data=on))
        self.node.get_logger().warn(f"E-STOP {'ON' if on else 'off'}")

    # -- periodic ------------------------------------------------------------
    def drive_keys(self, fwd, turn, dt, boost=False):
        """Keyboard layout -> targets. fwd = W minus S, turn = A minus D, each in [-1, 1]."""
        if turn != 0.0:
            self.recentre = False          # steering by hand overrides the sweep home
        if self.mode == "crab":
            targets = (fwd, turn, 0.0)     # A/D strafe left/right
        elif self.mode == "explicit" and self.recentre:
            # Proportional sweep: full rate while far, easing in near straight, so the
            # aim's feedback latency can't make it overshoot and hunt around zero.
            want = -self.recentre_gain * self.aim / max(1e-6, self.max_steer_rate)
            if abs(self.aim) < 0.005:
                want, self.recentre = 0.0, False
            targets = (fwd, 0.0, clamp(want, -1.0, 1.0))
        else:
            targets = (fwd, 0.0, turn)     # ackermann: yaw; explicit: steer the aim
        self.publish(targets, dt, boost=boost)

    def publish(self, targets, dt, enabled=True, boost=False):
        """targets: (ax_vx, ax_vy, ax_wz) each in [-1, 1]."""
        if not enabled or self.estop:
            targets = (0.0, 0.0, 0.0)
        tx, ty, tw = (clamp(t, -1.0, 1.0) for t in targets)
        v = self.max_speed * self.scale * (self.boost if boost else 1.0)
        rot = self.max_steer_rate if self.mode == "explicit" else self.max_yaw_rate * self.scale
        vx = self.lim[0].step(tx * v, dt)
        vy = self.lim[1].step(ty * v, dt)
        if self.mode == "explicit":
            wz = tw * rot          # steering rate: no ramp, the aim itself integrates
            self.lim[2].reset()
        else:
            wz = self.lim[2].step(tw * rot, dt)
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)
        self.last_cmd = (vx, vy, wz)
