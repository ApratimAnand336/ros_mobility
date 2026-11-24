"""Read MotorParams for steer and drive motors from `steer_motor.*` / `drive_motor.*`."""
from mobility_core import MotorParams


def motor_params(node, prefix, defaults: MotorParams):
    vals = dict(vars(defaults))
    got = node.get_parameters_by_prefix(prefix)
    for k in vals:
        if k in got and got[k].value is not None:
            vals[k] = float(got[k].value)
    return MotorParams(**vals)


DRIVE_DEFAULT = MotorParams(gear_ratio=50.0)    # no-load 16 rad/s, stall 72 N*m
STEER_DEFAULT = MotorParams(gear_ratio=100.0)   # no-load 8 rad/s, stall 144 N*m
