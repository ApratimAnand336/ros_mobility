"""PC -> motor-MCU CAN protocol. Pure Python (no python-can) so it is unit-testable.

*** PROVISIONAL *** -- frame IDs, layout and scaling are placeholders until the MCU
firmware is agreed (see OPEN_QUESTIONS.md). Everything is driven by CanConfig, so
changing the protocol is a config edit or a change to this one file.

Default layout (classic CAN, 8 data bytes, little-endian):
  drive_id   (0x101): int16 duty[FL], duty[FR], duty[BL], duty[BR]
  steer_id   (0x102): int16 duty[FL], duty[FR], duty[BL], duty[BR]
  heartbeat_id (0x100): uint8 counter, uint8 flags (bit0 active = non-zero command, bit1 estop), 6 x 0
duty int16 = round(d * duty_scale), d in [-1, 1]; sign = direction (DIR pin on the
SmartElex 30D), |d| = PWM duty. MCU must zero all outputs if no frame arrives for its
watchdog timeout (proposed 100 ms).
"""
import struct
from dataclasses import dataclass

from mobility_core import clamp


@dataclass
class CanConfig:
    drive_id: int = 0x101
    steer_id: int = 0x102
    heartbeat_id: int = 0x100
    extended_id: bool = False
    duty_scale: float = 32767.0
    little_endian: bool = True


def shape_duty(u, deadband=0.0, max_duty=1.0, zero_eps=1e-3):
    """u in [-1,1] -> duty in [-max_duty, max_duty] with deadband compensation:
    any non-zero command is lifted past the motor's static-friction duty."""
    if abs(u) < zero_eps:
        return 0.0
    s = 1.0 if u > 0 else -1.0
    d = deadband + (1.0 - deadband) * min(1.0, abs(u))
    return s * clamp(d, 0.0, max_duty)


def encode_duties(cfg: CanConfig, duties):
    """4 duties in [-1, 1] -> 8 data bytes."""
    if len(duties) != 4:
        raise ValueError("expected 4 duties (FL, FR, BL, BR)")
    vals = [int(round(clamp(d, -1.0, 1.0) * cfg.duty_scale)) for d in duties]
    return struct.pack(("<" if cfg.little_endian else ">") + "4h", *vals)


def decode_duties(cfg: CanConfig, data):
    vals = struct.unpack(("<" if cfg.little_endian else ">") + "4h", bytes(data))
    return [v / cfg.duty_scale for v in vals]


def encode_heartbeat(counter, active, estop):
    flags = (1 if active else 0) | (2 if estop else 0)
    return bytes([counter & 0xFF, flags, 0, 0, 0, 0, 0, 0])
