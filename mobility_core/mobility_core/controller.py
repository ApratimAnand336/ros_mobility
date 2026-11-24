"""The 8-loop wheel controller (4 steer position loops + 4 drive velocity loops).

Pure Python so the full closed loop can be unit-tested against the mock plant.
Inputs and outputs are dicts keyed by canonical joint name ("fl_steer", "fl_drive", ...).
"""
import math

from .pid import PID, PIDGains
from .slew import wrap_angle


class WheelController:
    def __init__(self, wheel_names, steer_gains: PIDGains, drive_gains: PIDGains,
                 cos_scaling=True, per_joint_gains=None, steer_release=0.0):
        """per_joint_gains: optional {joint_name: PIDGains} overriding the shared ones.
        steer_release: rad. If a wheel's drive target is 0 and its steer error is above
        this, the drive loop lets the wheel roll freely (u = 0, integrator cleared)
        instead of braking, so steering in place doesn't fight a locked wheel. 0 = off."""
        self.wheels = list(wheel_names)
        per_joint_gains = per_joint_gains or {}
        self.pids = {}
        for w in self.wheels:
            sg = per_joint_gains.get(f"{w}_steer", steer_gains)
            sg.angular = True
            self.pids[f"{w}_steer"] = PID(sg)
            self.pids[f"{w}_drive"] = PID(per_joint_gains.get(f"{w}_drive", drive_gains))
        self.cos_scaling = cos_scaling
        self.steer_release = steer_release
        self.drive_target = {}   # after cos scaling, for diagnostics

    @property
    def joints(self):
        return list(self.pids.keys())

    def reset(self):
        for p in self.pids.values():
            p.reset()

    def update(self, setpoints, positions, velocities, dt, setpoint_rates=None):
        """setpoints: {fl_steer: rad, fl_drive: rad/s, ...}
        positions/velocities: measured, same keys (drive position unused).
        setpoint_rates: optional {fl_steer: rad/s, ...} for steer rate feedforward.
        Returns {joint: u in [-1, 1]}."""
        setpoint_rates = setpoint_rates or {}
        out = {}
        for w in self.wheels:
            s, d = f"{w}_steer", f"{w}_drive"
            if s not in setpoints or s not in positions:
                out[s] = out[d] = 0.0
                continue
            out[s] = self.pids[s].update(setpoints[s], positions[s], dt,
                                         measured_rate=velocities.get(s),
                                         setpoint_rate=setpoint_rates.get(s, 0.0))
            target = setpoints.get(d, 0.0)
            if self.cos_scaling:
                # Don't push while the wheel is still far from its heading.
                err = wrap_angle(setpoints[s] - positions[s])
                target *= max(0.0, math.cos(err))
            self.drive_target[d] = target
            if self.steer_release > 0.0 and abs(target) < 1e-6 and \
                    abs(wrap_angle(setpoints[s] - positions[s])) > self.steer_release:
                self.pids[d].reset()
                out[d] = 0.0
            elif d in velocities:
                out[d] = self.pids[d].update(target, velocities[d], dt)
            else:
                out[d] = 0.0
        return out

    def status(self):
        """{joint: (error, output, saturated)} for diagnostics."""
        return {j: (p.error, p.output, p.saturated) for j, p in self.pids.items()}
