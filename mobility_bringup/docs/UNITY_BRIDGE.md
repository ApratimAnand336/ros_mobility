# Unity ↔ mobility bridge (MobilityBridge)

**Status (2026-11-24):** implemented and verified headless. With the defaults, the
`demo:=true` acceptance drive passes in Unity: worst steering p95 is 2.5°.

## Where the code lives
The Unity project is `~/ROS_unity/unity`. A reference copy of the files is in this repo
under [`unity/`](../../unity).

| File | What it is |
|---|---|
| `Assets/Scripts/CoupledDiff/MobilityBridge.cs` | **New.** The bridge itself. |
| `Assets/Scripts/CoupledDiff/RoverDriveController.cs` | **Two small additions**: `ResyncFromMeasured()` and `WheelRadiusScale(i)`. Nothing else changed. The original is in `Backups/controls_2026-11-24/`. |
| `Tools/MobilityBridgeTest/` | The headless end-to-end test. It is outside `Assets`, so it never ends up in a build. |

No prefab, scene or ProjectSettings file changed. The bridge attaches itself at Play to
every object with a `RoverDriveController`, through `RuntimeInitializeOnLoadMethod`.

## When the bridge drives, and when the keyboard does
Unity behaves exactly as before **unless** the ros_mobility stack is actively commanding
the rover. ROS drives only while **all** of these hold:
1. `enableRosControl` is ticked (the default);
2. a command arrived less than `streamTimeout` (0.25 s, real time) ago;
3. "keyboard only" is not latched (toggle it with **F8**);
4. `RoverDriveController.useKeyboard` is on. `TrajectoryRecording` turns it off, so an
   automated recording is never taken over.

If any of these stops holding, the bridge does the following:
- zeros the torques;
- resyncs `RoverDriveController` from the measured joints (so nothing snaps);
- re-enables `RoverDriveController`, which brakes the rover to a stop and hands back to the
  keyboard.

The ROS connection being up is not enough. `CameraPublisher` already connects in
`mars_1`, so the bridge waits for an actual command stream.

**Recording without ROS:** nothing changes. Recording *while* the ROS stack runs: press
F8 first, or untick `enableRosControl`.

## Two control modes (`rosControl`, set before Play)

| | **Effort** (default) | **Twist** |
|---|---|---|
| ROS sends | `/unity/effort_cmd`: u ∈ [-1, 1] × 8, the output of our 8 PIDs | `/cmd_vel` (TwistStamped) + `/mobility/steer_mode` |
| What moves the joints | The bridge's DC-motor model (below) | Unity's own tuned servos (`SetCommand`) |
| ROS launch | `domain:=unity` | `domain:=unity unity_control:=twist` (no PID nodes run) |
| Use it for | Testing the real control stack | Nav2 or scripted driving with Unity's smooth servos |

### Effort mode: the motor model runs inside Unity
At every physics step (200 Hz, set by `RoverSolverSettings`), the bridge does what a PWM
driver does to a real motor:
```
torque = G·kt·(u·V − ke·G·ω)/R
  u part:        jointForce = G·kt·V·u/R            (clamped to max torque)
  back-EMF part: ArticulationDrive.damping = G²·kt·ke/R   (PhysX solves this implicitly)
```
Running back-EMF as drive damping keeps the loop stable, even though ROS commands arrive
only about every 10 ms. It's the same trick Unity's own mode 5 uses. The steering joints
also get the URDF friction (0.5 N·m Coulomb, 1.0 N·m·s/rad viscous). The motor constants
are inspector fields and must match `unity.yaml` (V 24, kt = ke = 0.03, R 0.5, G 100 steer
and 50 drive).

## Topics

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/unity/joint_states` | `sensor_msgs/JointState` | Unity → ROS, 100 Hz, while connected | Names `fl_steer … br_drive`. Values are **already canonical**: rad and rad/s, steer + left, drive + forward. Signs come from `RoverDriveController`'s own axis calibration. Wheel speed is scaled by each wheel's relative collider radius (`compensateWheelRadii`). |
| `/unity/effort_cmd` | `std_msgs/Float64MultiArray` | ROS → Unity | 8 × u: steer FL FR BL BR, then drive FL FR BL BR |
| `/cmd_vel`, `/mobility/steer_mode` | TwistStamped, String | ROS → Unity | Twist mode only |

Because the values are already canonical, every `sign` in `unity.yaml` is +1. There is
nothing to verify by hand.

## Gains
Unity's wheel hulls are 32-sided convex meshes. They scrub about 20 N·m when steering at
standstill. With the Gazebo steering gains (kp 0.8 duty/rad), the steering lagged 5–22°.
`unity.yaml` uses Unity's own tuned mode-5 steering gains converted to duty: 800/150/6 N·m
÷ 144 N·m stall gives kp 5.5, ki 1.0, kd 0.04, with kr 0.15. The drive gains are the
Gazebo ones.

Headless result with these defaults:

| segment | steer p95 ° | drive (% of tol) | odom vx vy wz |
|---|---|---|---|
| forward | 0.38 | 17 | +0.56 0.00 +0.02 |
| arc_left | 1.45 | 6 | +0.49 0.00 +0.48 |
| reverse | 0.62 | 12 | −0.40 0.00 −0.02 |
| crab_45 | 0.34 | 9 | +0.31 +0.32 +0.01 |
| crab_right | 0.60 | 9 | 0.00 −0.40 +0.03 |
| explicit_aim | 2.47 | 32 | 0.00 0.00 +0.01 |
| explicit_roll | 0.69 | 8 | +0.34 +0.24 +0.04 |
| **result** | | | **PASS (11/11)** |

**Radius compensation removed most of the crab yaw drift.** It went from wz ±0.10–0.13
to 0.01–0.03 rad/s. The front hulls are about 1 % larger than the rear ones.

## Running it
```zsh
# terminal 1: ROS (needs the endpoint workspace)
source /opt/ros/jazzy/setup.zsh
source ~/ROS_unity/ros_ws/install/setup.zsh
source ~/ros_mobility/install/setup.zsh
ros2 launch mobility_bringup mobility.launch.py domain:=unity input:=keyboard
# Unity: press Play in mars_1. The console shows "[MobilityBridge] ROS driving (Effort)".
```

Headless acceptance test (a throwaway project; it never opens `mars_1`):
```zsh
~/ROS_unity/unity/Tools/MobilityBridgeTest/run_bridge_test.sh <tag> [extra_params.yaml] [seconds]
```
- It needs about 1.2 GB free disk. The throwaway project lands in `$WORK`
  (`/tmp/mobility_bridge_test` by default).
- It caps the open-file limit at 4096, because Unity's mono asserts when the limit is huge.

## Not done / open
- **`/clock` is not published**, so `use_sim_time` stays false for Unity. The bridge's
  timeouts use real time.
- **Twist mode** is compile-checked but has not yet been run end to end.
- **Unity's mode 5 (`ExplicitPidController`)** is left as is: keyboard mode 5 still uses it.
  Retire it once the bridge has been used for a while.
