#!/usr/bin/python3
"""Real rover: effort u -> PWM duty -> CAN frames to the motor MCU (SmartElex 30D drivers).

Feedback: the EKF already fuses the encoders, so this node only re-maps the EKF's
  topics into /mobility/joint_states (joints.* map: topic, name, field, sign, offset).
Actuation: per joint  d = shape_duty(cmd_sign * u, deadband, max_duty)  then
  can_protocol.encode_duties -> drive frame + steer frame at update_rate,
  heartbeat at heartbeat_rate. Zero frames on disarm (u = 0 from the controller),
  watchdog, and shutdown.

backend: socketcan (real bus, needs python-can) | virtual (python-can in-process bus) |
         dryrun (no python-can; logs frames, for bench checks without hardware)
"""
from std_msgs.msg import Bool

from mobility_ros import common as C

from .base import AdapterBase
from .can_protocol import CanConfig, encode_duties, encode_heartbeat, shape_duty


def _as_int(v):
    """Accept 257, "257" or "0x101" (YAML hex may arrive as a string)."""
    return v if isinstance(v, int) else int(str(v), 0)


class CanAdapter(AdapterBase):
    def __init__(self):
        super().__init__("can_adapter", default_rate=100.0)
        self.cfg = CanConfig(
            drive_id=_as_int(C.param(self, "can.drive_id", "0x101")),
            steer_id=_as_int(C.param(self, "can.steer_id", "0x102")),
            heartbeat_id=_as_int(C.param(self, "can.heartbeat_id", "0x100")),
            extended_id=bool(C.param(self, "can.extended_id", False)),
            duty_scale=float(C.param(self, "can.duty_scale", 32767.0)),
            little_endian=bool(C.param(self, "can.little_endian", True)))
        self.backend = C.param(self, "can.backend", "dryrun")
        self.channel = C.param(self, "can.channel", "can0")
        self.order = list(C.param(self, "command_order", list(self.geo.names)))
        self.max_duty = float(C.param(self, "max_duty", 0.9))
        self.deadband = {j: float(C.param(self, f"joints.{j}.deadband",
                                          C.param(self, "deadband", 0.05)))
                         for j in self.geo.joints}
        self.bus = self._open_bus()
        self.estop = False
        self.counter = 0
        self.create_subscription(Bool, C.ESTOP, self._on_estop, C.STATE_SUB)
        self.start_joint_state_ingest()
        self.create_timer(1.0 / self.rate, self.step)
        self.create_timer(1.0 / C.param(self, "heartbeat_rate", 20.0), self.heartbeat)

    def _open_bus(self):
        if self.backend == "dryrun":
            self.get_logger().warn("CAN backend = dryrun: frames are only logged")
            return None
        try:
            import can   # python-can, only needed on the real rover
        except ImportError as e:
            raise RuntimeError("python-can not installed: `pip install python-can` "
                               "(or set can.backend:=dryrun)") from e
        bus = can.interface.Bus(channel=self.channel, interface=self.backend)
        self.get_logger().info(f"CAN bus open: {self.backend}:{self.channel}")
        return bus

    def _on_estop(self, msg):
        self.estop = msg.data

    def _send(self, arb_id, data, always_log=False):
        if self.bus is None:
            text = f"CAN tx 0x{arb_id:03X} [{data.hex(' ')}]"
            if always_log:
                self.get_logger().info(text)
            else:
                self.get_logger().info(text, throttle_duration_sec=1.0)
            return
        import can
        try:
            self.bus.send(can.Message(arbitration_id=arb_id, data=data,
                                      is_extended_id=self.cfg.extended_id))
        except can.CanError as e:
            self.get_logger().error(f"CAN send failed: {e}", throttle_duration_sec=1.0)

    def duties(self, u):
        steer, drive = [], []
        for w in self.order:
            for j, out in ((f"{w}_steer", steer), (f"{w}_drive", drive)):
                uj = 0.0 if self.estop else u[j]
                out.append(shape_duty(self.maps[j].cmd_sign * uj, self.deadband[j],
                                      self.max_duty))
        return steer, drive

    def step(self):
        self.publish_joint_states()
        steer, drive = self.duties(self.current_effort())
        self._send(self.cfg.drive_id, encode_duties(self.cfg, drive))
        self._send(self.cfg.steer_id, encode_duties(self.cfg, steer))

    def heartbeat(self):
        active = any(abs(v) > 0.0 for v in self.current_effort().values())
        self.counter = (self.counter + 1) & 0xFF
        self._send(self.cfg.heartbeat_id, encode_heartbeat(self.counter, active, self.estop))

    def on_shutdown(self):
        zero = encode_duties(self.cfg, [0.0] * 4)
        for _ in range(3):
            self._send(self.cfg.drive_id, zero, always_log=True)
            self._send(self.cfg.steer_id, zero, always_log=True)
        if self.bus is not None:
            self.bus.shutdown()


def main():
    C.run(CanAdapter)


if __name__ == "__main__":
    main()
