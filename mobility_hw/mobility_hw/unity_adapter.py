#!/usr/bin/python3
"""Unity domain (via ROS-TCP-Endpoint). Contract: mobility_bringup/docs/UNITY_BRIDGE.md.

Unity side: MobilityBridge.cs (~/ROS_unity/unity/Assets/Scripts/CoupledDiff), which
auto-attaches to the rover at Play and drives it only while this adapter's command stream
is fresh; otherwise Unity's keyboard controls run as before.

Feedback: /unity/joint_states (JointState, canonical names fl_steer.., rad and rad/s,
  already in the ROS sign convention) -> canonical via the joints.* map (all +1).
Actuation: /unity/effort_cmd (Float64MultiArray, [steer x4, drive x4] in command_order)
  output_mode: torque     -> N*m from the DC motor model (Unity applies it as joint torque)
               normalized -> raw u in [-1, 1]; MobilityBridge applies the motor model (default)
"""
from std_msgs.msg import Float64MultiArray

from mobility_core import motor_torque

from mobility_ros import common as C

from .base import AdapterBase
from .sim_motor import DRIVE_DEFAULT, STEER_DEFAULT, motor_params


class UnityAdapter(AdapterBase):
    def __init__(self):
        super().__init__("unity_adapter", default_rate=100.0)
        self.steer_motor = motor_params(self, "steer_motor", STEER_DEFAULT)
        self.drive_motor = motor_params(self, "drive_motor", DRIVE_DEFAULT)
        self.order = list(C.param(self, "command_order", list(self.geo.names)))
        self.mode = C.param(self, "output_mode", "torque")
        self.pub = self.create_publisher(
            Float64MultiArray, C.param(self, "effort_topic", "/unity/effort_cmd"), 10)
        self.start_joint_state_ingest()
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(f"unity adapter up (output_mode={self.mode}); waiting for "
                               f"the MobilityBridge on {self.maps['fl_steer'].topic}")

    def _out(self, joint, u, motor):
        if self.mode == "normalized":
            val = u
        else:
            val = motor_torque(motor, u, self.vel.get(joint, 0.0))
        return self.maps[joint].cmd_sign * val

    def step(self):
        self.publish_joint_states()
        u = self.current_effort()
        steer = [self._out(f"{w}_steer", u[f"{w}_steer"], self.steer_motor) for w in self.order]
        drive = [self._out(f"{w}_drive", u[f"{w}_drive"], self.drive_motor) for w in self.order]
        self.pub.publish(Float64MultiArray(data=steer + drive))

    def on_shutdown(self):
        self.pub.publish(Float64MultiArray(data=[0.0] * (2 * len(self.order))))


def main():
    C.run(UnityAdapter)


if __name__ == "__main__":
    main()
