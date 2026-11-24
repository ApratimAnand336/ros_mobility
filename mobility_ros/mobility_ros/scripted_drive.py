#!/usr/bin/python3
"""Demo / acceptance driver: runs a scripted sequence and reports tracking.

Pass criteria follow ROVER_CONTROLS.md section 12 (the team's Gazebo/Unity acceptance
spec), measured over the last `window` seconds of every segment:
  steer: per-joint 95th percentile |target - position| < max_steer_error_deg (3 deg)
  drive: per-joint |mean error| < drive_rel_tol * |target| + drive_abs_tol
         (10 % + 0.1 rad/s)
Columns: worst joint's steer p95 (deg), worst joint's drive mean error as % of its
tolerance (100 = at the limit), and the body twist seen by wheel odometry.
Exits with code 1 if any segment fails.

  ros2 launch mobility_bringup mobility.launch.py domain:=mock demo:=true
"""
import math
import sys

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from . import common as C

# (label, mode, vx, vy, wz, seconds[, checks])  checks: 'all' (default) or 'steer_only'.
# steer_only = steering in place: wheels may roll freely (steer_release), so drive speed
# isn't graded; instead the body must stay put (|odom v| < still_tol).
SEQUENCES = {
    "default": [
        ("settle", "ackermann", 0.0, 0.0, 0.0, 2.0),
        ("forward", "ackermann", 0.6, 0.0, 0.0, 4.0),
        ("arc_left", "ackermann", 0.5, 0.0, 0.5, 5.0),
        ("reverse", "ackermann", -0.4, 0.0, 0.0, 4.0),
        ("stop", "ackermann", 0.0, 0.0, 0.0, 2.0),
        ("crab_45", "crab", 0.3, 0.3, 0.0, 5.0),
        ("crab_right", "crab", 0.0, -0.4, 0.0, 5.0),
        ("explicit_zero", "explicit", 0.0, 0.0, 0.0, 3.0),   # entry sweep to straight
        ("explicit_aim", "explicit", 0.0, 0.0, 0.3, 2.0, "steer_only"),  # rate 0.3 -> aim 0.6

        ("explicit_roll", "explicit", 0.4, 0.0, 0.0, 4.0),
        ("final_stop", "ackermann", 0.0, 0.0, 0.0, 3.0),
    ],
}


class ScriptedDrive(Node):
    def __init__(self):
        super().__init__("scripted_drive")
        self.geo = C.declare_geometry(self)
        self.seq = SEQUENCES[C.param(self, "sequence", "default")]
        self.window = C.param(self, "window", 1.0)
        self.max_steer_err_deg = C.param(self, "max_steer_error_deg", 3.0)
        self.drive_rel_tol = C.param(self, "drive_rel_tol", 0.10)
        self.drive_abs_tol = C.param(self, "drive_abs_tol", 0.1)      # rad/s
        self.still_tol = C.param(self, "still_tol", 0.05)            # m/s
        self.start_delay = C.param(self, "start_delay", 2.0)
        self.cmd_pub = self.create_publisher(TwistStamped, C.CMD_VEL, 10)
        self.mode_pub = self.create_publisher(String, C.STEER_MODE, C.LATCHED)
        self.create_subscription(JointState, C.SETPOINTS, self.on_sp, 10)
        self.create_subscription(JointState, C.JOINT_STATES, self.on_js, 10)
        self.create_subscription(Odometry, C.WHEEL_ODOM, self.on_odom, 10)
        self.sp, self.pos, self.vel, self.twist = {}, {}, {}, (0.0, 0.0, 0.0)
        self.idx, self.seg_t0, self.t0 = -1, None, self.get_clock().now()
        self.results, self.cur = [], None
        self.done = False
        self.create_timer(0.02, self.tick)

    def on_sp(self, m):
        pos, vel, _ = C.joint_state_to_dicts(m)
        self.sp.update({j: pos[j] for j in self.geo.steer_joints if j in pos})
        self.sp.update({j: vel[j] for j in self.geo.drive_joints if j in vel})

    def on_js(self, m):
        pos, vel, _ = C.joint_state_to_dicts(m)
        self.pos.update(pos)
        self.vel.update(vel)

    def on_odom(self, m):
        t = m.twist.twist
        self.twist = (t.linear.x, t.linear.y, t.angular.z)

    def now_s(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def tick(self):
        if self.done:
            return
        t = self.now_s()
        if t < self.start_delay:
            return
        if self.idx < 0 or t - self.seg_t0 >= self.seq[self.idx][5]:
            if self.cur:
                self.results.append(self.cur)
            self.idx += 1
            if self.idx >= len(self.seq):
                self.finish()
                return
            label, mode = self.seq[self.idx][:2]
            checks = self.seq[self.idx][6] if len(self.seq[self.idx]) > 6 else "all"
            self.mode_pub.publish(String(data=mode))
            self.seg_t0 = t
            self.cur = {"label": label, "steer": {}, "drive": {}, "twist": None,
                        "checks": checks, "max_v": 0.0}
            self.get_logger().info(f"segment {label} ({mode})")
        label, mode, vx, vy, wz, dur = self.seq[self.idx][:6]
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)
        if t - self.seg_t0 >= dur - self.window:
            for j in self.geo.steer_joints:
                if j in self.sp and j in self.pos:
                    self.cur["steer"].setdefault(j, []).append(
                        abs(math.degrees(self.sp[j] - self.pos[j])))
            for j in self.geo.drive_joints:
                if j in self.sp and j in self.vel:
                    self.cur["drive"].setdefault(j, []).append(
                        (self.sp[j], self.sp[j] - self.vel[j]))
            self.cur["twist"] = self.twist
            self.cur["max_v"] = max(self.cur["max_v"], math.hypot(*self.twist[:2]))

    def score(self, r):
        """-> (worst steer p95 deg, worst drive error as % of tolerance)."""
        steer = 0.0
        for errs in r["steer"].values():
            e = sorted(errs)
            steer = max(steer, e[min(len(e) - 1, int(0.95 * len(e)))])
        drive = 0.0
        if r["checks"] == "steer_only":
            return steer, 100.0 * r["max_v"] / self.still_tol   # body must stay still
        for samples in r["drive"].values():
            target = sum(abs(sp) for sp, _ in samples) / len(samples)
            mean_err = abs(sum(e for _, e in samples) / len(samples))
            tol = self.drive_rel_tol * target + self.drive_abs_tol
            drive = max(drive, 100.0 * mean_err / tol)
        return steer, drive

    def finish(self):
        self.done = True
        ok = True
        lines = [f"{'segment':14s} {'steer p95°':>10s} {'drive %tol':>10s}   odom vx vy wz"]
        for r in self.results:
            steer, drive = self.score(r)
            bad = steer > self.max_steer_err_deg or drive > 100.0
            ok &= not bad
            tw = r["twist"] or (0, 0, 0)
            lines.append(f"{r['label']:14s} {steer:10.2f} {drive:10.1f}   "
                         f"{tw[0]:+.2f} {tw[1]:+.2f} {tw[2]:+.2f}  {'FAIL' if bad else 'ok'}")
        lines.append("RESULT: " + ("PASS" if ok else "FAIL"))
        self.get_logger().info("\n" + "\n".join(lines))
        self.exit_code = 0 if ok else 1
        raise SystemExit(self.exit_code)


def main():
    import rclpy
    rclpy.init()
    node = ScriptedDrive()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
