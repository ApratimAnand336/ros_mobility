# PC → motor MCU CAN protocol (PROVISIONAL)

**Status:** this is a proposal and a placeholder. It must be agreed with whoever writes
the MCU firmware (open question Q2).
- The code is in `mobility_hw/can_protocol.py`, which has no ROS and is unit-tested.
- The IDs, byte order and scale are config values in `config/real.yaml` (`can_adapter.can.*`).

## Hardware chain
```
wheel_controller (u ∈ [-1,1]) → can_adapter → CAN → MCU → PWM + DIR → SmartElex 30D → motor
                                     ↑
         EKF-fused steer θ / wheel ω (ROS topics, not CAN) → /mobility/joint_states
```

## Frames (classic CAN, 8 data bytes, little-endian, standard 11-bit IDs)

| ID | Rate | Payload |
|---|---|---|
| `0x100` heartbeat | 20 Hz | `u8 counter`, `u8 flags` (bit0 = active, meaning some command is non-zero; bit1 = e-stop), then 6 × `0` |
| `0x101` drive duty | 100 Hz | `int16 FL, FR, BL, BR` |
| `0x102` steer duty | 100 Hz | `int16 FL, FR, BL, BR` |

- **Duty values.** `int16 = round(d × 32767)` with d ∈ [-1, 1]. The sign is the
  direction (the SmartElex DIR pin) and |d| is the PWM duty.
- **What the PC already applies to d.** Per-joint `cmd_sign`, deadband lift
  (`d = sign·(deadband + (1−deadband)·|u|)` for any u ≠ 0), and the `max_duty` cap. The MCU
  should output d as-is.

## MCU requirements (safety)
1. **Watchdog.** If no `0x101`/`0x102` frame arrives for 100 ms, all 8 outputs go to 0.
   Do the same if no heartbeat arrives for 200 ms. Without this, a ROS crash leaves the
   motors running.
2. **E-stop.** If heartbeat bit1 is set, hold all outputs at 0 whatever the duty frames say.
3. **Boot state.** Outputs are 0 until the first valid frame arrives.
4. **PWM frequency.** Pick one inside the SmartElex 30D spec, above the audible range if
   possible (to be decided by the firmware owner).

## Changing the protocol
- **IDs, endianness or scale:** edit `real.yaml`. No code change is needed.
- **Layout** (e.g. one frame per motor, or CAN FD): edit `encode_duties()` and
  `encode_heartbeat()` in `can_protocol.py`, then update `test/test_can_protocol.py`.
- **Bench test without hardware:**
  - With `can.backend: dryrun`, frames are only logged.
  - Or create a virtual bus:
    ```
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    ```
    Then set `can.backend: socketcan`, `can.channel: vcan0`, and watch with `candump vcan0`.
  - Real and vcan backends need `pip install python-can`.
