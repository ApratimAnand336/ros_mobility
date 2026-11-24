#!/usr/bin/python3
"""cmd_vel -> per-wheel steer angle + wheel speed setpoints (canonical units).

Sub:  /cmd_vel (TwistStamped, or Twist if cmd_vel_stamped:=false)
      /mobility/steer_mode (String, latched)  ackermann | crab | explicit
      /mobility/joint_states (seed steer setpoints once, so startup never snaps)
      /mobility/estop (Bool)
Pub:  /mobility/wheel_setpoints (JointState: *_steer position = angle, velocity = angle
      rate (for feedforward); *_drive velocity = wheel speed)
      /mobility/steer_aim (Float64, explicit mode's shared aim)
"""
import math

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from mobility_core import KinematicsLimits, SwerveKinematics

from . import common as C


class SwerveKinematicsNode(Node):
    def __init__(self):
        super().__init__("swerve_kinematics")
        self.geo = C.declare_geometry(self)
        d = self.declare_parameter
        d("update_rate", 100.0)       # Hz
        d("command_timeout", 0.5)     # s since last cmd_vel before commanding zero
        d("mode", "ackermann")
        d("cmd_vel_stamped", True)
        d("seed_from_feedback", True)
        lim_defaults = KinematicsLimits()
        for k, v in vars(lim_defaults).items():
            d(k, float(v))
        p = self.get_parameter
        limits = KinematicsLimits(**{k: p(k).value for k in vars(lim_defaults)})
        self.kin = SwerveKinematics(self.geo, limits, mode=p("mode").value)
        self.timeout = p("command_timeout").value
        self.dt = 1.0 / p("update_rate").value

        self.want = (0.0, 0.0, 0.0)
        self.last_cmd = None
        self.estop = False
        self.seeded = not p("seed_from_feedback").value
        self.prev_angles = None

        if p("cmd_vel_stamped").value:
            self.create_subscription(TwistStamped, C.CMD_VEL,
                                     lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, C.CMD_VEL, self.on_cmd, 10)
        self.create_subscription(String, C.STEER_MODE, self.on_mode, C.STATE_SUB)
        self.create_subscription(Bool, C.ESTOP, self.on_estop, C.STATE_SUB)
        self.create_subscription(JointState, C.JOINT_STATES, self.on_feedback, 10)
        self.sp_pub = self.create_publisher(JointState, C.SETPOINTS, 10)
        self.aim_pub = self.create_publisher(Float64, C.STEER_AIM, 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f"swerve kinematics up, mode={self.kin.mode}")

    def on_cmd(self, t):
        cmd = (t.linear.x, t.linear.y, t.angular.z)
        if not all(math.isfinite(c) for c in cmd):
            self.get_logger().warn("dropping non-finite cmd_vel", throttle_duration_sec=2.0)
            return
        self.want = cmd
        self.last_cmd = self.get_clock().now()

    def on_mode(self, msg):
        try:
            if msg.data.strip().lower() != self.kin.mode:
                self.kin.set_mode(msg.data)
                self.get_logger().info(f"steer mode -> {self.kin.mode}")
        except ValueError as e:
            self.get_logger().warn(str(e))

    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.kin.reset_motion()

    def on_feedback(self, msg):
        if self.seeded:
            return
        pos, _, _ = C.joint_state_to_dicts(msg)
        if all(j in pos for j in self.geo.steer_joints):
            self.kin.seed([pos[j] for j in self.geo.steer_joints])
            self.seeded = True
            self.get_logger().info("steer setpoints seeded from feedback")

    def step(self):
        want = self.want
        if self.last_cmd is None or self.estop or (
                self.timeout > 0.0 and
                (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout):
            want = (0.0, 0.0, 0.0)
        angles, speeds = self.kin.step(want, self.dt)
        nan = float("nan")
        n = len(angles)
        prev = self.prev_angles or angles
        rates = [(a - p) / self.dt for a, p in zip(angles, prev)]
        self.prev_angles = list(angles)
        msg = C.make_joint_state(self.get_clock().now().to_msg(), self.geo.joints,
                                 position=list(angles) + [nan] * n,
                                 velocity=rates + list(speeds))   # steer: setpoint rate
        self.sp_pub.publish(msg)
        if self.kin.mode == "explicit":
            self.aim_pub.publish(Float64(data=self.kin.aim))


def main():
    C.run(SwerveKinematicsNode)


if __name__ == "__main__":
    main()
