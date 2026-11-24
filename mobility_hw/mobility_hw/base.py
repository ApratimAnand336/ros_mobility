"""Shared adapter machinery: joint mapping + feedback ingestion + effort watchdog.

Every adapter converts between the canonical interface and one domain:

    /mobility/effort_cmd (u in [-1,1])  --write()-->  domain actuators
    domain sensors  --ingest-->  /mobility/joint_states (rad, rad/s, REP-103 signs)

Per-joint mapping is pure config (`joints.<canonical>.*` in the domain yaml):

    joints:
      fl_steer:
        name: FLE_joint        # joint name in the domain's JointState / actuator list
        topic: /joint_states   # feedback topic carrying it (default: feedback_topic)
        sign: -1.0             # canonical = sign * scale * raw + offset
        scale: 1.0             # e.g. 0.0174533 for degrees, or 1/r for m/s -> rad/s
        offset: 0.0            # rad, steering zero offset
        cmd_sign: -1.0         # actuator sign (defaults to sign); real motors can
                               #   be wired opposite to their encoder
        field: position        # which JointState field holds the main quantity
"""
import math

from rclpy.node import Node
from sensor_msgs.msg import JointState

from mobility_ros import common as C


class JointMap:
    def __init__(self, canonical, name, topic, sign=1.0, scale=1.0, offset=0.0,
                 cmd_sign=None, field=None):
        self.canonical = canonical
        self.is_steer = canonical.endswith("_steer")
        self.name, self.topic = name, topic
        self.sign, self.scale, self.offset = float(sign), float(scale), float(offset)
        self.cmd_sign = self.sign if cmd_sign is None else float(cmd_sign)
        self.field = field or ("position" if self.is_steer else "velocity")

    def to_canonical(self, raw):
        return self.sign * self.scale * raw + (self.offset if self.is_steer else 0.0)

    def rate_to_canonical(self, raw):
        return self.sign * self.scale * raw


class AdapterBase(Node):
    """Subclasses implement write(u: dict) and optionally use the JointState ingest."""

    def __init__(self, name, default_rate=100.0):
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.geo = C.declare_geometry(self)
        self.rate = C.param(self, "update_rate", default_rate)
        self.effort_timeout = C.param(self, "effort_timeout", 0.25)
        # A joint whose source (EKF, Gazebo, Unity) has been silent this long is no longer
        # published, so the controller's feedback_timeout trips instead of the adapter
        # re-sending a frozen value forever.
        self.source_timeout = C.param(self, "source_timeout", 0.15)
        default_topic = C.param(self, "feedback_topic", "/joint_states")
        self.maps = {}
        for j in self.geo.joints:
            got = {k: v.value for k, v in self.get_parameters_by_prefix(f"joints.{j}").items()}
            self.maps[j] = JointMap(j, got.get("name", j), got.get("topic", default_topic),
                                    got.get("sign", 1.0), got.get("scale", 1.0),
                                    got.get("offset", 0.0), got.get("cmd_sign"),
                                    got.get("field"))
        self.u = {j: 0.0 for j in self.geo.joints}
        self.u_time = None
        self.pos, self.vel = {}, {}
        self.src_time = {}          # joint -> time of its last source sample (ingest only)
        self._stale_logged = set()
        self.create_subscription(JointState, C.EFFORT_CMD, self._on_effort, 10)
        self.js_pub = self.create_publisher(JointState, C.JOINT_STATES, 10)

    # -- effort in -----------------------------------------------------------
    def _on_effort(self, msg):
        _, _, eff = C.joint_state_to_dicts(msg)
        for j in self.geo.joints:
            if j in eff:
                self.u[j] = max(-1.0, min(1.0, eff[j]))
        self.u_time = self.get_clock().now()

    def current_effort(self):
        """Latest u per joint, or all zeros if the controller went quiet (watchdog)."""
        if self.u_time is None or \
                (self.get_clock().now() - self.u_time).nanoseconds * 1e-9 > self.effort_timeout:
            return {j: 0.0 for j in self.geo.joints}
        return dict(self.u)

    # -- feedback ingest (JointState sources: Gazebo, Unity, EKF) --------------
    def start_joint_state_ingest(self, qos=10):
        topics = sorted({m.topic for m in self.maps.values()})
        for t in topics:
            self.create_subscription(JointState, t,
                                     lambda msg, t=t: self._on_source(t, msg), qos)
        self.get_logger().info(f"feedback from {topics}")

    def _on_source(self, topic, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        now = self.get_clock().now()
        for m in self.maps.values():
            if m.topic != topic or m.name not in idx:
                continue
            i = idx[m.name]
            arr = getattr(msg, m.field)
            if i < len(arr) and math.isfinite(arr[i]):
                if m.is_steer:
                    self.pos[m.canonical] = m.to_canonical(arr[i])
                else:
                    self.vel[m.canonical] = m.rate_to_canonical(arr[i])
                self.src_time[m.canonical] = now
            # Steer joints: also pass their rate through when the source has one.
            if m.is_steer and m.field == "position" and i < len(msg.velocity) and \
                    math.isfinite(msg.velocity[i]):
                self.vel[m.canonical] = m.rate_to_canonical(msg.velocity[i])

    def _fresh(self, j, now):
        """Joints filled by ingest must have a recent source sample. Joints the adapter
        computes itself (mock plant) have no src_time and are always fresh."""
        t = self.src_time.get(j)
        if t is None or self.source_timeout <= 0.0:
            return True
        ok = (now - t).nanoseconds * 1e-9 <= self.source_timeout
        if not ok and j not in self._stale_logged:
            self.get_logger().warn(f"{j}: no source data for > {self.source_timeout:.2f} s, "
                                   "not publishing it (controller will stop)")
            self._stale_logged.add(j)
        elif ok and j in self._stale_logged:
            self.get_logger().info(f"{j}: source data back")
            self._stale_logged.discard(j)
        return ok

    def publish_joint_states(self):
        now = self.get_clock().now()
        names = [j for j in self.geo.joints
                 if (j in self.pos or j in self.vel) and self._fresh(j, now)]
        if not names:
            return
        nan = float("nan")
        self.js_pub.publish(C.make_joint_state(
            self.get_clock().now().to_msg(), names,
            position=[self.pos.get(j, nan) for j in names],
            velocity=[self.vel.get(j, nan) for j in names]))

    def write(self, u):
        raise NotImplementedError
