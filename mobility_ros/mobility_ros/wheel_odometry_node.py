#!/usr/bin/python3
"""Wheel odometry: least-squares body twist from all 4 wheels, integrated to a pose.

Sub:  /mobility/joint_states
Pub:  /mobility/wheel_odom (nav_msgs/Odometry, odom -> base_link). Feed it to the EKF.
TF:   off by default (publish_tf), because the EKF normally owns odom -> base_link.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mobility_core import body_twist_from_wheels

from . import common as C


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        self.geo = C.declare_geometry(self)
        self.odom_frame = C.param(self, "odom_frame", "odom")
        self.base_frame = C.param(self, "base_frame", "base_link")
        self.publish_tf = C.param(self, "publish_tf", False)
        self.twist_cov = C.param(self, "twist_covariance", [0.01, 0.01, 0.02])  # vx, vy, wz
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.pub = self.create_publisher(Odometry, C.WHEEL_ODOM, 10)
        self.tf = None
        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, C.JOINT_STATES, self.on_js, 10)

    def on_js(self, msg):
        pos, vel, _ = C.joint_state_to_dicts(msg)
        if not all(j in pos for j in self.geo.steer_joints) or \
                not all(j in vel for j in self.geo.drive_joints):
            return
        vx, vy, wz = body_twist_from_wheels(self.geo,
                                            [pos[j] for j in self.geo.steer_joints],
                                            [vel[j] for j in self.geo.drive_joints])
        t = self.get_clock().now()
        if self.last_t is not None:
            dt = (t - self.last_t).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.x += (vx * c - vy * s) * dt
                self.y += (vx * s + vy * c) * dt
                self.yaw += wz * dt
        self.last_t = t

        o = Odometry()
        o.header.stamp = t.to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = self.twist_cov
        cov[14] = cov[21] = cov[28] = 1e6     # z, roll, pitch: not observed
        o.twist.covariance = cov
        self.pub.publish(o)
        if self.tf:
            tr = TransformStamped()
            tr.header = o.header
            tr.child_frame_id = self.base_frame
            tr.transform.translation.x, tr.transform.translation.y = self.x, self.y
            tr.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(tr)


def main():
    C.run(WheelOdometryNode)


if __name__ == "__main__":
    main()
