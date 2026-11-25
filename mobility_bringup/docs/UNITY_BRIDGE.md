# Unity ↔ mobility bridge (MobilityBridge)

**Status (2026-11-25):** setpoint mode, the default, runs our 8 PID loops **inside Unity's
physics step**. Headless at **10 fps** (the editor's real rate in `mars_1`), the
acceptance drive passes 11/11, worst steering p95 2.4°.

**Why it changed.** Over ROS-TCP, Unity only handles ROS messages once per rendered
frame. At 10 fps that put about 100 ms of delay inside the control loop, plus 20 physics
steps run back to back on each stale command, so the steering oscillated in the editor.
Now:
- ROS sends setpoints, gains and an enable flag;
- Unity closes the loop every physics step (5 ms), like its own mode 5;
- the frame delay only affects how fast a new *command* arrives, never stability.

## Where the code lives
The Unity project is `~/ROS_unity/unity`. A reference copy of the files is in this repo
under [`unity/`](../../unity).

| File | What it is |
|---|---|
| `Assets/Scripts/CoupledDiff/MobilityBridge.cs` | **New.** The bridge itself. |
| `Assets/Scripts/CoupledDiff/MobilityPid.cs` | **New.** A line-for-line C# port of `mobility_core/pid.py` + `controller.py`, plain C# with no Unity API. `unity/parity/parity.py` runs both versions on 4000 identical steps and requires identical outputs (currently a difference of exactly 0). **Run it after changing either side.** |
| `Assets/Scripts/CoupledDiff/RoverDriveController.cs` | **Two small additions**: `ResyncFromMeasured()` and `WheelRadiusScale(i)`. Nothing else changed. The original is in `Backups/controls_2026-11-24/`. |
| `Tools/MobilityBridgeTest/` | The headless end-to-end test. It is outside `Assets`, so it never ends up in a build. |

No prefab, scene or ProjectSettings file changed. The bridge attaches itself at Play to
every object with a `RoverDriveController`, through `RuntimeInitializeOnLoadMethod`.

## When the bridge drives, and when the keyboard does
Unity behaves exactly as before **unless** the ros_mobility stack is actively commanding
the rover. ROS drives only while **all** of these hold:
1. `enableRosControl` is ticked (the default);
2. a command arrived less than `streamTimeout` (0.5 s, real time) ago;
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

### Who is driving right now?
There are two different keyboards:
- **The "Rover teleop" window** (ROS, `input:=keyboard`) drives through our PIDs. It moves
  the Unity rover **only while the bridge is engaged**.
- **WASD in Unity's Game view** is the old `RoverDriveController`. It works whenever the
  bridge is **not** engaged, and is ignored while ROS drives (F8 gets it back).

| Unity console | ROS terminal (`unity_adapter`) | Meaning |
|---|---|---|
| `not connected to the ROS endpoint` | `no /unity/joint_states from Unity …` every 5 s | Unity and ROS aren't talking. Old controls drive. Start the launch, then press Play, and check that the ROS HUD (top-left of the Game view) is green. |
| `connected, but no commands … yet` | — | The endpoint is up, but ros_mobility isn't running with `domain:=unity`. |
| `controller gains from ROS: …` then `ROS driving (Setpoint: our 8 PIDs run in Unity …)` | `Unity MobilityBridge connected …` | **Our stack drives.** Use the teleop window; Unity's WASD is ignored. |
| `setpoints arriving but no gains … yet` | — | ROS was launched with `unity_control:=effort`, or an old launch file is in use. |
| `back to keyboard: …` | `Unity joint states stopped` | Handed back to the old controls. |

## Control modes
`rosControl` in the inspector is **Auto** by default: the bridge picks the mode from what
ROS sends. Twist is the only mode you have to choose by hand.

| | **Setpoint** (default) | **Effort** | **Twist** |
|---|---|---|---|
| ROS launch | `domain:=unity` | `domain:=unity unity_control:=effort` | `domain:=unity unity_control:=twist` |
| ROS runs | teleop, kinematics, `wheel_controller` **remote** (gains + arm/e-stop/stale state machine), adapter (feedback) | everything, including the 8 PIDs | teleop only |
| ROS sends | `/mobility/wheel_setpoints`, `/mobility/controller_config` (gains, JSON), `/mobility/controller_enable` | `/unity/effort_cmd` (u × 8) | `/cmd_vel`, `/mobility/steer_mode` |
| Loop runs | **in Unity, every physics step** (MobilityPid.cs) → motor model | in ROS → u over TCP → motor model | Unity's own servos |
| Stable in the editor (~10 fps)? | **yes** | no (fine only at high fps, e.g. headless) | yes |

Safety stays with ROS in setpoint mode. Unity outputs u = 0 unless `wheel_controller` says
`running` (armed, no e-stop, fresh feedback and setpoints). Unity also listens to
`/mobility/estop` directly.

The steering setpoint is carried forward along its rate between messages (at most 0.15 s),
so a frame's burst of physics steps doesn't see a frozen ramp. While it drives, the bridge
publishes `/unity/controller_state` (the setpoints the loops used, and u) at the same
physics instant as the joint states. `demo:=true` grades against it, so the TCP delay
isn't counted as tracking error.

### The motor model (setpoint and effort modes) runs inside Unity
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

## Gains and results
Unity's wheel hulls are 32-sided convex meshes. They scrub about 20 N·m when steering, so
`unity.yaml` uses Unity's own tuned mode-5 steering gains, converted to duty:
800/150/6 N·m ÷ 144 N·m stall gives kp 5.5, ki 1.0, kd 0.04, with kr 0.15. The drive gains
are the Gazebo ones. ROS sends them to Unity; they are not duplicated in C#.

**Headless at 10 fps** (the editor's measured rate), setpoint mode, defaults:

| test | result |
|---|---|
| `demo:=true` (11 segments: forward, arc, reverse, stop, crab ×2, explicit ×3, final stop) | **PASS**. Worst steering p95 2.4° (explicit_aim); driving segments 0.3–1.3°. |
| hold test (`scripted_drive.sequence: hold`: parked 10 s each at 0°, about 43° and about 90°) | **PASS**. Back-and-forth steering at rest: 0.00° / 0.07° / 0.01° (limit 0.5°). |

The same gains in **effort** mode, where the loop runs over TCP, oscillated: at 30 fps the
steering error was 21–30°. That oscillation is what the editor showed on 2026-11-24.

**Radius compensation:** each wheel's speed is scaled by its relative collider radius,
because the front hulls are about 1 % larger. This cut the crab yaw drift from ±0.10 to
about 0.02 rad/s.

## Running it
```zsh
# terminal 1: ROS (needs the endpoint workspace)
source /opt/ros/jazzy/setup.zsh
source ~/ROS_unity/ros_ws/install/setup.zsh
source ~/ros_mobility/install/setup.zsh
ros2 launch mobility_bringup mobility.launch.py domain:=unity input:=keyboard
# Unity: press Play in mars_1 (before or after). The console shows
#   "[MobilityBridge] controller gains from ROS: ..." then "ROS driving (Setpoint: ...)".
```

Headless acceptance test (a throwaway project; it never opens `mars_1`):
```zsh
FPS=10 ~/ROS_unity/unity/Tools/MobilityBridgeTest/run_bridge_test.sh <tag> [extra_params.yaml] [seconds]
```
- **Always set `FPS`** to the editor's frame rate; the bridge's `loop:` console line shows
  it. Uncapped headless runs at hundreds of fps and hides delay problems, which is how
  the first effort-mode tuning passed headless but jittered in the editor.
- For the hold/jitter test, put `scripted_drive: {ros__parameters: {sequence: hold}}` in
  the extra params file.
- Parity check after changing `pid.py`, `controller.py` or `MobilityPid.cs`:
  `source install/setup.zsh && python3 unity/parity/parity.py`.
- It needs about 1.2 GB free disk. The throwaway project lands in `$WORK`
  (`/tmp/mobility_bridge_test` by default).
- It caps the open-file limit at 4096, because Unity's mono asserts when the limit is huge.

## Not done / open
- **`/clock` is not published**, so `use_sim_time` stays false for Unity. The bridge's
  timeouts use real time.
- **Twist mode** is compile-checked but has not yet been run end to end.
- **Not yet seen in the real editor:** setpoint mode in `mars_1` with the GUI. The
  headless test at 10 fps matches its frame rate, but the terrain differs.
- **Steering at an angle converges slowly at rest** (about 1° over several seconds, the
  integral working against the scrub). That's within spec; raise `steer.ki` if it matters.
- **Unity's mode 5 (`ExplicitPidController`)** is left as is: keyboard mode 5 still uses it.
  Retire it once the bridge has been used for a while.
