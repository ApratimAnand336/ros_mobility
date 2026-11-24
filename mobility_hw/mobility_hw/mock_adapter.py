#!/usr/bin/python3
"""Mock domain: no simulator. Each of the 8 joints is a DC motor driving an inertia
with friction (mobility_core.motor_model). Good for CI and pipeline checks; not physics.

Params: steer_motor.*, drive_motor.* (MotorParams), steer_plant.*, drive_plant.*
(inertia, viscous, coulomb), substeps, vel_noise_std (rad/s), pos_noise_std (rad),
encoder_counts (quantise drive velocity like a real encoder; 0 = off).
"""
import random

from mobility_core import JointPlant, PlantParams

from mobility_ros import common as C

from .base import AdapterBase
from .sim_motor import DRIVE_DEFAULT, STEER_DEFAULT, motor_params


def plant_params(node, prefix, defaults: PlantParams):
    vals = dict(vars(defaults))
    for k, v in node.get_parameters_by_prefix(prefix).items():
        if k in vals:
            vals[k] = float(v.value)
    return PlantParams(**vals)


class MockAdapter(AdapterBase):
    def __init__(self):
        super().__init__("mock_adapter", default_rate=200.0)
        sm = motor_params(self, "steer_motor", STEER_DEFAULT)
        dm = motor_params(self, "drive_motor", DRIVE_DEFAULT)
        lim = self.geo.steer_limit + 0.1
        sp = plant_params(self, "steer_plant", PlantParams(0.05, 2.0, 3.0, -lim, lim))
        dp = plant_params(self, "drive_plant", PlantParams(0.3, 0.5, 2.0))
        self.plants = {j: JointPlant(sm, sp) for j in self.geo.steer_joints}
        self.plants.update({j: JointPlant(dm, dp) for j in self.geo.drive_joints})
        self.substeps = int(C.param(self, "substeps", 5))
        self.vel_noise = C.param(self, "vel_noise_std", 0.0)
        self.pos_noise = C.param(self, "pos_noise_std", 0.0)
        self.counts = C.param(self, "encoder_counts", 0)   # per wheel rev, 0 = off
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info("mock domain up (motor model, no simulator)")

    def step(self):
        u = self.current_effort()
        h = self.dt / self.substeps
        for j, p in self.plants.items():
            for _ in range(self.substeps):
                p.step(u[j], h)
        for j, p in self.plants.items():
            if j.endswith("_steer"):
                self.pos[j] = p.pos + random.gauss(0.0, self.pos_noise) if self.pos_noise \
                    else p.pos
                self.vel[j] = p.vel
            else:
                v = p.vel + (random.gauss(0.0, self.vel_noise) if self.vel_noise else 0.0)
                if self.counts:
                    # One encoder count per sample period = the velocity resolution.
                    q = 2 * 3.141592653589793 / self.counts / self.dt
                    v = round(v / q) * q
                self.vel[j] = v
        self.publish_joint_states()


def main():
    C.run(MockAdapter)


if __name__ == "__main__":
    main()
