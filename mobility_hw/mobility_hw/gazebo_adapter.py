#!/usr/bin/python3
"""Gazebo domain (ROS2_Autonomous_2026_27 rover_gazebo + gz_ros2_control).

Feedback: /joint_states (joint_state_broadcaster, ~1 kHz) -> canonical via joints.* map.
Actuation: u -> DC motor model -> joint torque (N*m) -> Float64MultiArray on
  steer_pid_controller/commands and wheel_pid_controller/commands (effort controllers
  already defined in rover_gazebo/config/controllers.yaml, spawned inactive).
At startup it switches those effort controllers on and the ideal position/velocity
controllers off, so exactly one control law owns the 8 joints.
"""
from controller_manager_msgs.srv import SwitchController
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from mobility_core import motor_torque

from mobility_ros import common as C

from .base import AdapterBase
from .sim_motor import DRIVE_DEFAULT, STEER_DEFAULT, motor_params


class GazeboAdapter(AdapterBase):
    def __init__(self):
        super().__init__("gazebo_adapter", default_rate=100.0)
        self.steer_motor = motor_params(self, "steer_motor", STEER_DEFAULT)
        self.drive_motor = motor_params(self, "drive_motor", DRIVE_DEFAULT)
        self.order = list(C.param(self, "command_order", list(self.geo.names)))
        self.steer_pub = self.create_publisher(
            Float64MultiArray, C.param(self, "steer_command_topic",
                                       "/steer_pid_controller/commands"), 10)
        self.drive_pub = self.create_publisher(
            Float64MultiArray, C.param(self, "drive_command_topic",
                                       "/wheel_pid_controller/commands"), 10)
        self.start_joint_state_ingest(qos=qos_profile_sensor_data)

        self.switched = not C.param(self, "switch_controllers", True)
        self.activate = list(C.param(self, "activate_controllers",
                                     ["steer_pid_controller", "wheel_pid_controller"]))
        self.deactivate = list(C.param(self, "deactivate_controllers",
                                       ["steer_controller", "wheel_controller"]))
        self.switch_cli = self.create_client(
            SwitchController, C.param(self, "switch_service",
                                      "/controller_manager/switch_controller"))
        self.pending = None
        self.create_timer(1.0, self.try_switch)
        self.create_timer(1.0 / self.rate, self.step)

    def try_switch(self):
        if self.switched or self.pending is not None:
            return
        if not self.switch_cli.service_is_ready():
            self.get_logger().info("waiting for controller_manager...",
                                   throttle_duration_sec=5.0)
            return
        req = SwitchController.Request()
        req.activate_controllers = self.activate
        req.deactivate_controllers = self.deactivate
        req.strictness = SwitchController.Request.STRICT   # retry until all exist
        req.activate_asap = True
        self.pending = self.switch_cli.call_async(req)
        self.pending.add_done_callback(self._switched)

    def _switched(self, fut):
        self.pending = None
        res = fut.result()
        if res is not None and res.ok:
            self.switched = True
            self.get_logger().info(f"controllers: +{self.activate} -{self.deactivate}")
        else:
            self.get_logger().warn("controller switch failed, retrying")

    def step(self):
        self.publish_joint_states()
        u = self.current_effort() if self.switched else {j: 0.0 for j in self.geo.joints}
        steer, drive = [], []
        for w in self.order:
            s, d = f"{w}_steer", f"{w}_drive"
            tau_s = motor_torque(self.steer_motor, u[s], self.vel.get(s, 0.0))
            tau_d = motor_torque(self.drive_motor, u[d], self.vel.get(d, 0.0))
            steer.append(self.maps[s].cmd_sign * tau_s)
            drive.append(self.maps[d].cmd_sign * tau_d)
        self.steer_pub.publish(Float64MultiArray(data=steer))
        self.drive_pub.publish(Float64MultiArray(data=drive))

    def on_shutdown(self):
        zeros = [0.0] * len(self.order)
        self.steer_pub.publish(Float64MultiArray(data=zeros))
        self.drive_pub.publish(Float64MultiArray(data=zeros))


def main():
    C.run(GazeboAdapter)


if __name__ == "__main__":
    main()
