"""DC motor + gearbox model and a simple joint plant. Pure Python, no ROS.

Used by the sim adapters to turn the normalised effort u (PWM duty) into joint torque,
so sim behaves like a voltage-driven motor and the PID gains transfer to the real rover:

    tau = G * kt * (u * V_bus - ke * G * omega) / R          (omega = joint speed)

All parameters are placeholders until measured on the real motors (see OPEN_QUESTIONS).
"""
import math
from dataclasses import dataclass

from .slew import clamp


@dataclass
class MotorParams:
    v_bus: float = 24.0        # V
    gear_ratio: float = 50.0   # motor turns per joint turn
    kt: float = 0.03           # N*m/A at the motor shaft
    ke: float = 0.03           # V*s/rad at the motor shaft
    resistance: float = 0.5    # ohm
    efficiency: float = 1.0    # gearbox efficiency (multiplies torque)
    max_torque: float = 150.0  # N*m at the joint, hard clamp (URDF effort limit)

    @property
    def no_load_speed(self):
        """Joint speed (rad/s) at u = 1 with no load."""
        return self.v_bus / (self.ke * self.gear_ratio)

    @property
    def stall_torque(self):
        """Joint torque (N*m) at u = 1, omega = 0."""
        return self.gear_ratio * self.kt * self.v_bus / self.resistance * self.efficiency


def motor_torque(p: MotorParams, u, omega):
    u = clamp(u, -1.0, 1.0)
    back_emf = p.ke * p.gear_ratio * omega
    current = (u * p.v_bus - back_emf) / p.resistance
    tau = p.gear_ratio * p.kt * current * p.efficiency
    return clamp(tau, -p.max_torque, p.max_torque)


@dataclass
class PlantParams:
    inertia: float = 0.3       # kg*m^2 reflected to the joint
    viscous: float = 0.5       # N*m*s/rad
    coulomb: float = 1.0       # N*m dry friction
    pos_min: float = -math.inf
    pos_max: float = math.inf


class JointPlant:
    """One joint: J*domega = tau - b*omega - coulomb*sign(omega). Mock domain only."""

    def __init__(self, motor: MotorParams, plant: PlantParams):
        self.m, self.p = motor, plant
        self.pos = 0.0
        self.vel = 0.0

    def step(self, u, dt, load_torque=0.0):
        tau = motor_torque(self.m, u, self.vel) + load_torque
        friction = self.p.viscous * self.vel
        # Stiction: if nearly stopped and torque can't beat dry friction, stay stopped.
        if abs(self.vel) < 1e-3 and abs(tau) <= self.p.coulomb:
            self.vel = 0.0
            return
        friction += math.copysign(self.p.coulomb, self.vel if self.vel != 0.0 else tau)
        self.vel += (tau - friction) / self.p.inertia * dt
        self.pos += self.vel * dt
        if self.pos < self.p.pos_min or self.pos > self.p.pos_max:
            self.pos = clamp(self.pos, self.p.pos_min, self.p.pos_max)
            self.vel = 0.0
