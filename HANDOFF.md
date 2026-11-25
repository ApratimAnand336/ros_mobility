# Handoff: Unity control (updated 2026-11-25)

## Status: editor jitter fixed in headless tests (not yet confirmed in the real editor)
**Cause.** The bridge's own loop stats in `mars_1` showed **10 fps, worst command age
about 103 ms**. ROS-TCP delivers messages once per rendered frame. With the PID in ROS,
the loop therefore had about 100 ms of delay, and 20 physics steps ran back to back on
each stale command. Unity's own mode 5 is stable because its loop runs inside the physics
step.

**Fix: setpoint mode**, the default `unity_control:=setpoint`:
- ROS runs: teleop, kinematics, and `wheel_controller` with `remote:=true`. That node
  keeps the gains and the arm/e-stop/stale-feedback state machine, and publishes
  `/mobility/controller_config` and `/mobility/controller_enable`.
- Unity's `MobilityBridge` runs **`MobilityPid.cs`**, a line-for-line C# port of
  `pid.py` + `controller.py`, every physics step (200 Hz), feeding the same DC-motor model.
- `unity/parity/parity.py` proves the Python and C# versions agree over 4000 steps × 8
  joints (difference 0).
- Unity's `RoverDriveController` is untouched apart from yesterday's two small methods.
  It still drives whenever ROS isn't commanding, which keeps data recording safe.

**Headless results at 10 fps (the editor's rate):**

| run | result |
|---|---|
| effort mode, kp 5.5 @ 30 fps | FAIL, 21–30° oscillation (the editor symptom) |
| **setpoint mode, kp 5.5 @ 10 fps, demo** | **PASS 11/11**, worst steering p95 2.4° |
| **setpoint mode, kp 5.5 @ 10 fps, hold test** | **PASS**: back-and-forth at rest 0.00° / 0.07° / 0.01° (0°, 43°, 90°) |

## Next steps
1. **Confirm in the editor:** run `ros2 launch mobility_bringup mobility.launch.py
   domain:=unity input:=keyboard`, then press Play in `mars_1`.
   - The console should show `controller gains from ROS: steer kp 5.5 …`, then
     `ROS driving (Setpoint: …)`.
   - Check that the wheels sit still at rest. Drive all three modes from the teleop window.
2. If anything still shakes on the `mars_1` terrain:
   - copy the bridge's `loop:` lines and say which mode or angle it happens at;
   - reproduce it with `FPS=<editor fps>` and the hold test (`UNITY_BRIDGE.md`, "Running it").
3. **Optional:**
   - raise `steer.ki` if the slow ~1° creep onto an angle at rest bothers you;
   - find out why `mars_1` only renders 10 fps;
   - publish `/clock`.
4. **Real rover:** the PID runs in ROS there, and its delay is EKF + CAN. Get the EKF rate
   and delay (OPEN Q16), then test the gains with that delay.

## Notes
- **Headless tests must cap fps** (`FPS=10`). Uncapped runs hide delay problems.
- Disk is about 2–3 GB free. The harness project at `<scratchpad>/bridge_test/project` is
  deleted after use.
- Before committing, run `unity/sync_from_unity.sh` so the repo's C# copy is current.
