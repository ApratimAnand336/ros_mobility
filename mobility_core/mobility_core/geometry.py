"""Rover geometry: wheel stations and wheel radius.

Canonical conventions used across the whole stack (REP-103, base_link axes):
  x forward, y left, z up. Positive steer angle = wheel turned CCW seen from above
  (pointing left). Positive drive speed = wheel rolling the rover forward along the
  direction the wheel points. Wheel order is always the order of `wheel_names`.
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class WheelStation:
    name: str      # e.g. "fl"
    x: float       # m, forward of rover centre
    y: float       # m, left of rover centre


@dataclass
class Geometry:
    wheel_radius: float
    stations: List[WheelStation] = field(default_factory=list)
    steer_limit: float = 1.5708   # rad, |canonical steer angle| allowed by the hardware

    @property
    def names(self):
        return [s.name for s in self.stations]

    @property
    def steer_joints(self):
        return [f"{n}_steer" for n in self.names]

    @property
    def drive_joints(self):
        return [f"{n}_drive" for n in self.names]

    @property
    def joints(self):
        return self.steer_joints + self.drive_joints

    @classmethod
    def from_lists(cls, wheel_radius, names, xs, ys, steer_limit=1.5708):
        if not (len(names) == len(xs) == len(ys)):
            raise ValueError("wheel_names, wheel_x and wheel_y must have equal length")
        return cls(wheel_radius=float(wheel_radius),
                   stations=[WheelStation(n, float(x), float(y))
                             for n, x, y in zip(names, xs, ys)],
                   steer_limit=float(steer_limit))


# Default: the MRT rover, from rover_gazebo/config/rover_kinematics.yaml
# (generated from CAD by tools/derive_ratios.py in ROS2_Autonomous_2026_27).
DEFAULT_GEOMETRY = Geometry.from_lists(
    wheel_radius=0.149850,
    names=["fl", "fr", "bl", "br"],
    xs=[0.372365, 0.372365, -0.372365, -0.371204],
    ys=[0.218, -0.218, 0.218, -0.218],
    steer_limit=1.5708,
)
