# ros_mobility

**One drive stack for the MRT rover, in mock, Gazebo, Unity and on the real rover.**

The rover has 4-wheel independent steering, so it runs 8 PID loops: one steering-angle
loop and one wheel-speed loop per wheel. The same nodes, topics and code run in every
domain. Only one thin adapter node and one YAML file change per domain.

- ROS 2 **Jazzy**, Python (`rclpy`), `ament_python`.
- No custom messages: only standard message types.
- The output is a normalized effort **u ∈ [-1, 1]** per joint. On the real rover u is the
  PWM duty. In simulation the same u goes through a DC-motor model, so gains carry over
  between sim and reality.

```
 joystick / keyboard window / Nav2 / scripted demo
            │  /cmd_vel (TwistStamped)       /mobility/steer_mode: ackermann | crab | explicit
            ▼
   swerve_kinematics ──► /mobility/wheel_setpoints   (steer rad + rate, wheel rad/s)
            ▼
   wheel_controller (8 PIDs) ──► /mobility/effort_cmd   (u ∈ [-1,1] per joint)
            ▲                            │
 /mobility/joint_states (SI, REP-103)    ▼
   ──────────── adapter: the ONLY per-domain node ───────────────────────────────
   mock    : built-in DC motor + inertia plant (no simulator; CI and quick checks)
   gazebo  : /joint_states → canonical;  u → motor model → N·m → *_pid_controller
   unity   : /unity/joint_states → canonical;  u → /unity/effort_cmd → MobilityBridge (motor model in Unity)
   real    : EKF θ/ω topics → canonical;  u → deadband/limit → PWM duty → CAN → MCU → SmartElex 30D
```

---

## Repository layout

```
ros_mobility/
├── mobility_core/        pure Python, no ROS; unit-tested math
│   └── mobility_core/    kinematics.py (swerve IK + least-squares FK), pid.py, controller.py
│                         (the 8 loops), motor_model.py, slew.py, geometry.py
├── mobility_ros/         the domain-independent nodes
│   └── mobility_ros/     swerve_kinematics_node, wheel_controller_node, wheel_odometry_node,
│                         teleop_key_window (Tk), teleop_keyboard (terminal), teleop_joy,
│                         scripted_drive (acceptance demo), common.py (topics, QoS, run())
├── mobility_hw/          one adapter per domain
│   └── mobility_hw/      base.py (config-driven joint mapping), mock_adapter, gazebo_adapter,
│                         unity_adapter, can_adapter + can_protocol.py
├── mobility_bringup/
│   ├── launch/mobility.launch.py     the one launch file for every domain
│   ├── config/                       geometry, common, mock, gazebo, unity, real (.yaml)
│   └── docs/                         CAN_PROTOCOL.md, UNITY_BRIDGE.md
├── unity/                reference copy of the Unity-side files (source of truth: ~/ROS_unity)
│   ├── Assets/Scripts/CoupledDiff/MobilityBridge.cs
│   ├── RoverDriveController.additions.patch
│   ├── Tools/MobilityBridgeTest/     headless Unity ↔ ROS acceptance test
│   └── sync_from_unity.sh            refresh this copy from the Unity project
├── NODES.md              node by node: what starts what, each node's tick, start-up, shutdown
├── OPEN_QUESTIONS.md     hardware and team questions (Q1–Q16) with the placeholders in use
└── README.md
```

---

## Dependencies

### Always
- ROS 2 Jazzy (`/opt/ros/jazzy`), `colcon`, `python3-numpy`.
- These ROS packages: `rclpy`, `std_msgs`, `std_srvs`, `sensor_msgs`, `geometry_msgs`,
  `nav_msgs`, `diagnostic_msgs`, `controller_manager_msgs`, `launch_ros`. All come with
  `ros-jazzy-desktop`.
- `python3-tk` for the keyboard window (`input:=keyboard`).
- `ros-jazzy-joy` for `input:=joy`.

### Per domain: other folders this stack talks to
This repo does **not** vendor these folders. They live beside it on the laptop and are
only needed for their own domain.

| Domain | External folder | What is used from it | How |
|---|---|---|---|
| **mock** | none | — | — |
| **gazebo** | **ROS2_Autonomous_2026_27** (`~/ROS2_Autnomous_Mrt/ROS2_Autonomous_2026_27`, GitHub `iitbmartian/ROS2_Autonomous_2026_27`, branch `apratim/new-controls`) | **Launch:** `rover_bringup/launch/rover_sim.launch.py` (Gazebo Harmonic, the rover URDF, `gz_ros2_control`), included when `include_sim:=true`. **Controllers:** `steer_pid_controller` and `wheel_pid_controller` (effort controllers, spawned inactive by that launch); `gazebo_adapter` activates them through `/controller_manager/switch_controller` and deactivates `steer_controller` / `wheel_controller`. **Feedback:** `/joint_states` from `joint_state_broadcaster` (`FLE_joint`… steer, `FLD_joint`… drive). **Geometry and signs:** copied from `rover_gazebo/config/rover_kinematics.yaml` into `config/geometry.yaml` and `config/gazebo.yaml`. | Source its `install/setup.zsh` before this workspace. Its `rover_kinematics` and `explicit_pid_control` nodes still start but lose control once the switch happens (OPEN Q10). |
| **unity** | **ROS_unity** (`~/ROS_unity`; not a git repo yet) | **`unity/`:** Unity project (Unity 6000.1.17f1). Scene `mars_1`, prefab `urdf_full_withdifferential_coupled`. Our additions: `Assets/Scripts/CoupledDiff/MobilityBridge.cs` (new) and two small methods in `RoverDriveController.cs` (see `unity/RoverDriveController.additions.patch`). **`ros_ws/`:** `ros_tcp_endpoint` (ROS-TCP-Endpoint), which `domain:=unity` starts on 127.0.0.1:10000. | Source `~/ROS_unity/ros_ws/install/setup.zsh`. The Unity project is the source of truth for the C# files; `unity/sync_from_unity.sh` refreshes the copy here. Details: [UNITY_BRIDGE.md](mobility_bringup/docs/UNITY_BRIDGE.md). |
| **real** | **Rover EKF + MCU firmware** (team, not on this laptop yet) | Per-wheel fused EKF steer θ and wheel ω (placeholder topics `/wheel_{fl,fr,rl,rr}/steer_encoder`, the format of `rover_ekf`'s splitter). CAN frames to the motor MCU, which drives the SmartElex 30D drivers. | `python3-can` for `backend: socketcan` (the default `dryrun` needs nothing). Everything is a placeholder until OPEN_QUESTIONS Q1–Q3 are answered. |

**For localisation, use the team's `rover_ekf`** (ROS2_Autonomous_2026_27: wheel fit +
IMU). This stack's `wheel_odometry` is off by default; it only runs for `demo:=true`.

---

## Build

### Option A: standalone workspace (this folder *is* the workspace)
```zsh
cd ~/ros_mobility
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install && source install/setup.zsh
colcon test && colcon test-result --verbose    # 36 unit tests
```

### Option B: inside another colcon workspace
Clone this repo into the workspace's `src/` and build as usual. The four packages have no
build-time dependency on the other repos.

---

## Run

```zsh
# Mock (no simulator): scripted drive through every mode, prints PASS/FAIL and exits
ros2 launch mobility_bringup mobility.launch.py domain:=mock demo:=true

# Gazebo
source ~/ROS2_Autnomous_Mrt/ROS2_Autonomous_2026_27/install/setup.zsh && source install/setup.zsh
ros2 launch mobility_bringup mobility.launch.py domain:=gazebo include_sim:=true input:=keyboard
ros2 launch mobility_bringup mobility.launch.py domain:=gazebo include_sim:=true gui:=false world:=flat.sdf demo:=true

# Unity: start this, then press Play in Unity (mars_1)
source ~/ROS_unity/ros_ws/install/setup.zsh && source install/setup.zsh
ros2 launch mobility_bringup mobility.launch.py domain:=unity input:=keyboard
ros2 launch mobility_bringup mobility.launch.py domain:=unity input:=keyboard unity_control:=twist  # Unity's own servos

# Real rover (CAN dry-run until the protocol is fixed; see OPEN_QUESTIONS.md)
ros2 launch mobility_bringup mobility.launch.py domain:=real input:=joy
ros2 service call /mobility/arm std_srvs/srv/SetBool "{data: true}"   # the real rover starts disarmed
```

### Launch arguments

| Argument | Values | Notes |
|---|---|---|
| `domain` | `mock` \| `gazebo` \| `unity` \| `real` | Picks the adapter and `config/<domain>.yaml` |
| `input` | `keyboard` (key window) \| `keyboard_terminal` \| `joy` \| `none` | Use `none` when Nav2 or a script publishes `/cmd_vel` |
| `include_sim`, `world`, `gui` | | Gazebo only |
| `unity_control` | `effort` \| `twist` | Unity only. `effort`: our PIDs drive. `twist`: only `/cmd_vel`, no PID nodes run. |
| `start_endpoint`, `ros_ip` | | Unity only: ROS-TCP endpoint |
| `demo` | `true` \| `false` | Scripted acceptance drive, then shuts down. Also starts `wheel_odometry`. |
| `publish_odom` | default `false` | Start `wheel_odometry` without the demo |
| `use_sim_time` | default `auto` | `auto` is true for gazebo, false for the others |
| `extra_params` | path | One more YAML file, loaded last (local overrides) |
| `keyboard_prefix` | | The terminal that `input:=keyboard_terminal` opens in |

### Keyboard (`input:=keyboard` opens a small "Rover teleop" window; click it once)
- **Several keys at once** work, so `W`+`A` drives an arc. Driving continues while a key is held.
- **`W`/`S`:** forward and back.
- **`A`/`D`:** depend on the mode: turn (ackermann), strafe (crab), steer the shared aim (explicit).
- **`C`:** sweep the explicit aim back to straight.
- **`Shift`:** boost.
- **`1`/`2`/`3`:** ackermann, crab, explicit. **`M`:** cycle modes.
- **`+`/`-`:** speed scale.
- **`Space`:** stop. **`X`:** toggle e-stop. **`Esc`:** close.
- The window is a port of the team's `rover_gazebo/teleop_rover_gui.py`.

### Steering modes

| Mode | What you command | What the rover does |
|---|---|---|
| `ackermann` | vx + yaw rate | Arcs, with front and rear counter-steering. The radius is at least `min_turn_radius`. |
| `crab` | vx + vy | Translates in any direction and keeps its heading |
| `explicit` | vx + **steering rate** | All four wheels share one aim, which the stick sweeps; the rover goes where the wheels point |

All modes go through the same 8 PIDs.

---

## Conventions
- **Joint names:** `fl_steer fr_steer bl_steer br_steer fl_drive fr_drive bl_drive br_drive`.
- **Frame:** REP-103 (x forward, y left).
- **Steer:** positive points the wheel left (CCW seen from above).
- **Drive:** positive rolls the wheel forward, along wherever it points.
- **Adapters own every conversion:** renames, signs, units and offsets, configured in
  `joints.<name>.{name, topic, field, sign, scale, offset, cmd_sign}` in the domain YAML.

## Changing things (mostly YAML)

| You want to… | Edit |
|---|---|
| Change gains | `common.yaml` → `wheel_controller.steer.*` / `drive.*`, or per domain in `<domain>.yaml`. Override one joint with `br_drive: {kp: …}`. |
| Change speed or turn limits | `common.yaml` → `swerve_kinematics.*` |
| Change geometry or steer limit | `geometry.yaml` (single source for every node) |
| Change EKF topics, joint names, signs or zero offsets | `real.yaml` → `can_adapter.joints.*` |
| Change CAN IDs or scaling | `real.yaml` → `can_adapter.can.*`. Change the layout in `can_protocol.py`. |
| Change the joystick mapping | `common.yaml` → `teleop_joy.*` |
| Change the PID rate | `<domain>.yaml` → `wheel_controller.update_rate` (see OPEN Q16) |
| Add a domain | Subclass `mobility_hw.base.AdapterBase`, add `config/<domain>.yaml`, and register it in `ADAPTERS` in the launch file |
| Add a steering mode | `mobility_core/kinematics.py`: add it to `MODES` and to `step()` |

## PID (`mobility_core/pid.py`)
```
u = kv·sp + ks·sign(sp) + kr·d(sp)/dt + kp·e + I − kd·d(meas)/dt (filtered),  clamped to ±out_limit
```
- **Anti-windup:** conditional integration, plus `i_limit`.
- **`hold_i_limit`:** a bounded brake integral, so a parked rover holds on a hill.
- **Feedforward:** drive `kv` (≈ 1/no-load speed) and `ks` (static friction); steer `kr` on
  the setpoint rate.
- **Cos scaling:** the drive target is scaled by cos(steer error), so a wheel doesn't push
  while it's still turning.
- **`steer_release`:** a parked wheel rolls freely while it steers.

## Safety
- **Command timeout:** `swerve_kinematics` commands zero after 0.5 s without `/cmd_vel`.
- **Controller output is u = 0** when the controller is disarmed or e-stopped, when **any
  one** of the 8 joints' feedback is stale (0.2 s), or when setpoints are stale.
- **Dead feedback source:** the adapter stops publishing any joint whose source (EKF,
  Gazebo, Unity) has been silent for `source_timeout` (0.15 s).
- **Adapter watchdog:** each adapter sends zero 0.25 s after the last `effort_cmd`.
- **Shutdown:** every node sends zero on Ctrl-C (CAN: 3 zero frames).
- **Arming:** the real rover starts disarmed (`/mobility/arm`). Sim domains arm automatically.
- **E-stop:** `/mobility/estop` (std_msgs/Bool) from any publisher. The keyboard `X` and the
  joystick button toggle it.
- **Unity:** the bridge drives only while our command stream is fresh. **F8** forces
  keyboard only, and it never takes over a `TrajectoryRecording`.
- **Still required:** an MCU-side watchdog in the firmware (`docs/CAN_PROTOCOL.md`).

## Verification status (2026-11-24)
- **`colcon test`:** 36 unit tests pass. They cover kinematics, PID, motor model, a
  closed loop with all 8 PIDs, CAN encoding, joystick shaping and key handling.
- **Mock demo:** PASS.
- **Gazebo demo** (headless, `flat.sdf`, the team's rover model): **PASS on all 11 segments**
  under the `ROVER_CONTROLS.md` §12 thresholds. Worst steering p95 is 1.9°.
- **Unity demo** (headless, flat ground, the coupled rover prefab, through ROS-TCP):
  **PASS on all 11 segments**, on two runs. Worst steering p95 is 2.5–2.9°. When the ROS
  stack stops, control returns to the keyboard.
- **Real (CAN dry-run, fake EKF topics):** it arms, sends frames, zeros them on e-stop, and
  sends zero frames on Ctrl-C. Killing one wheel's EKF stream stops the controller within
  0.2 s, and it resumes when the stream returns.
- **Not yet tested:**
  - Gazebo slope and obstacle worlds;
  - a real joystick;
  - Unity twist mode, and Unity in the `mars_1` scene with the GUI;
  - anything on the real rover.

## Known issues
- After `ros2 launch` stops, the Gazebo server from `rover_sim.launch.py` sometimes
  survives SIGINT. Clean up with `pkill -f "^gz sim "`.
- Real-rover gains are the sim gains. They need tuning on the rover (OPEN Q15).
- Unity publishes no `/clock`, so `use_sim_time` stays false there.

## More documentation
- [NODES.md](NODES.md): what the launch file starts, what each node does every tick, the
  start-up timeline and the shutdown order.
- [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md): hardware and team answers needed, Q1–Q16.
- [mobility_bringup/docs/UNITY_BRIDGE.md](mobility_bringup/docs/UNITY_BRIDGE.md): the
  Unity bridge, its hand-over rules, motor model and gains.
- [mobility_bringup/docs/CAN_PROTOCOL.md](mobility_bringup/docs/CAN_PROTOCOL.md): the
  provisional PC ↔ MCU frame layout and the watchdog.
