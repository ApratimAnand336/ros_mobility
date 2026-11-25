#!/usr/bin/python3
"""8 PID loops: wheel setpoints + measured joint states -> normalised effort u in [-1, 1].

Sub:  /mobility/wheel_setpoints, /mobility/joint_states, /mobility/estop (latched Bool)
Srv:  /mobility/arm (SetBool)  -- outputs stay 0 until armed (auto_arm for sim)
Pub:  /mobility/effort_cmd (JointState, effort field = u per canonical joint)
      /mobility/diagnostics (DiagnosticArray: per-joint error/effort/saturation + state)

remote: true -> this node does NOT compute u. Something else runs the same 8 loops
(Unity's MobilityBridge, inside its physics step, to avoid the ROS-TCP frame delay). This
node then still owns the gains and the safety state machine, and publishes:
      /mobility/controller_config (String, JSON: the resolved per-joint gains, latched + 1 Hz)
      /mobility/controller_enable (Bool: true only while state == running)

Gains are parameters `steer.kp`, `drive.kv`, ... (see config/common.yaml and the
per-domain yaml). Any single joint can override them: `fl_drive.kp: 0.4`.
"""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from mobility_core import PIDGains
from mobility_core.controller import WheelController

from . import common as C

GAIN_FIELDS = [k for k in vars(PIDGains()) if k != "angular"]


class WheelControllerNode(Node):
    def __init__(self):
        super().__init__("wheel_controller",
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.geo = C.declare_geometry_safe(self)
        p = self._param
        self.dt = 1.0 / p("update_rate", 100.0)
        self.fb_timeout = p("feedback_timeout", 0.2)
        self.sp_timeout = p("setpoint_timeout", 0.3)
        self.armed = bool(p("auto_arm", False))
        self.remote = bool(p("remote", False))

        steer = self._gains("steer", PIDGains(kp=3.0, ki=1.0, kd=0.1, i_limit=0.3,
                                              d_filter_tau=0.01, deadzone=0.002))
        drive = self._gains("drive", PIDGains(kp=0.3, ki=1.0, kv=0.0625, ks=0.03,
                                              i_limit=0.5, hold_i_limit=0.3))
        overrides = {}
        for j in self.geo.joints:
            if self.get_parameters_by_prefix(j):
                overrides[j] = self._gains(j, steer if j.endswith("_steer") else drive)
        self.ctl = WheelController(self.geo.names, steer, drive,
                                   cos_scaling=bool(p("cos_scaling", True)),
                                   per_joint_gains=overrides,
                                   steer_release=p("steer_release", 0.0))

        self.setpoints, self.sp_time, self.sp_rates = {}, None, {}
        self.pos, self.vel = {}, {}
        self.fb_time = {}           # joint -> last time it appeared in joint_states
        self.estop = False
        self.state = "init"
        self.last_u = {j: 0.0 for j in self.geo.joints}

        self.create_subscription(JointState, C.SETPOINTS, self.on_setpoints, 10)
        self.create_subscription(JointState, C.JOINT_STATES, self.on_feedback, 10)
        self.create_subscription(Bool, C.ESTOP, self.on_estop, C.STATE_SUB)
        self.create_service(SetBool, C.ARM_SRV, self.on_arm)
        self.eff_pub = self.create_publisher(JointState, C.EFFORT_CMD, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, C.DIAGNOSTICS, 10)
        self.create_timer(self.dt, self.step)
        if self.remote:
            self.cfg_msg = String(data=json.dumps(self.ctl.config()))
            self.cfg_pub = self.create_publisher(String, C.CONTROLLER_CONFIG, C.LATCHED)
            self.en_pub = self.create_publisher(Bool, C.CONTROLLER_ENABLE, 10)
            self.cfg_pub.publish(self.cfg_msg)
            # Re-sent at 1 Hz: the ROS-TCP endpoint does not replay latched messages.
            self.create_timer(1.0, lambda: self.cfg_pub.publish(self.cfg_msg))
            self.get_logger().info("remote mode: the 8 loops run elsewhere (Unity); "
                                   "publishing gains and the enable flag only")
        self.create_timer(1.0 / max(0.1, p("diagnostics_rate", 5.0)), self.publish_diag)
        self.get_logger().info(f"wheel controller up, armed={self.armed}")

    # -- params ------------------------------------------------------------
    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _gains(self, prefix, base: PIDGains):
        vals = dict(vars(base))
        got = self.get_parameters_by_prefix(prefix)
        for k in GAIN_FIELDS:
            if k in got and got[k].value is not None:
                vals[k] = float(got[k].value)
        return PIDGains(**vals)

    # -- inputs ------------------------------------------------------------
    def on_setpoints(self, msg):
        pos, vel, _ = C.joint_state_to_dicts(msg)
        sp = {j: pos[j] for j in self.geo.steer_joints if j in pos}
        sp.update({j: vel[j] for j in self.geo.drive_joints if j in vel})
        self.sp_rates = {j: vel[j] for j in self.geo.steer_joints if j in vel}
        self.setpoints, self.sp_time = sp, self.get_clock().now()

    def on_feedback(self, msg):
        pos, vel, _ = C.joint_state_to_dicts(msg)
        self.pos.update(pos)
        self.vel.update(vel)
        now = self.get_clock().now()
        for j in msg.name:
            self.fb_time[j] = now

    def on_estop(self, msg):
        self.estop = msg.data
        self.get_logger().warn(f"e-stop {'ENGAGED' if self.estop else 'released'}")

    def on_arm(self, req, resp):
        self.armed = req.data
        self.ctl.reset()
        resp.success, resp.message = True, f"armed={self.armed}"
        self.get_logger().info(resp.message)
        return resp

    # -- loop --------------------------------------------------------------
    def _age(self, t):
        return float("inf") if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9

    def step(self):
        if self.estop:
            state = "estop"
        elif not self.armed:
            state = "disarmed"
        elif max(self._age(self.fb_time.get(j)) for j in self.geo.joints) > self.fb_timeout:
            # Every one of the 8 joints must be fresh: one dead wheel stops all of them.
            state = "feedback_stale"
        elif self._age(self.sp_time) > self.sp_timeout:
            state = "setpoints_stale"
        else:
            state = "running"
        if state != self.state:
            text = f"controller state: {self.state} -> {state}"
            if state == "running":
                self.get_logger().info(text)
                self.ctl.reset()       # fresh integrators every time we (re)start
            else:
                self.get_logger().warn(text)
            self.state = state

        if self.remote:
            self.en_pub.publish(Bool(data=state == "running"))
            return
        if state == "running":
            u = self.ctl.update(self.setpoints, self.pos, self.vel, self.dt, self.sp_rates)
        else:
            u = {j: 0.0 for j in self.geo.joints}
        self.last_u = u
        names = self.geo.joints
        self.eff_pub.publish(C.make_joint_state(self.get_clock().now().to_msg(), names,
                                                effort=[u.get(j, 0.0) for j in names]))

    def publish_diag(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        top = DiagnosticStatus(name="mobility/controller", hardware_id="mobility")
        top.level = DiagnosticStatus.OK if self.state == "running" else DiagnosticStatus.WARN
        top.message = self.state
        top.values = [KeyValue(key="armed", value=str(self.armed)),
                      KeyValue(key="estop", value=str(self.estop))]
        arr.status.append(top)
        for j, (err, u, sat) in self.ctl.status().items():
            st = DiagnosticStatus(name=f"mobility/{j}", hardware_id="mobility")
            st.level = DiagnosticStatus.WARN if sat and self.state == "running" \
                else DiagnosticStatus.OK
            st.message = "saturated" if sat else "ok"
            st.values = [KeyValue(key="setpoint", value=f"{self.setpoints.get(j, 0.0):.4f}"),
                         KeyValue(key="error", value=f"{err:.4f}"),
                         KeyValue(key="effort", value=f"{self.last_u.get(j, 0.0):.3f}")]
            arr.status.append(st)
        self.diag_pub.publish(arr)

    def on_shutdown(self):
        if self.remote:
            self.en_pub.publish(Bool(data=False))
            return
        names = self.geo.joints
        self.eff_pub.publish(C.make_joint_state(self.get_clock().now().to_msg(), names,
                                                effort=[0.0] * len(names)))


def main():
    C.run(WheelControllerNode)


if __name__ == "__main__":
    main()
