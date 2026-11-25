# Node guide: what starts what, and what each node does

This guide follows one launch from start to finish: which nodes start, in what
order, what each node does on every tick, and how everything stops.
For a quick start and the conventions, see [README.md](README.md).

---

## 1. Who starts what

**Only the launch file starts nodes.** None of our nodes spawns another node.
`mobility.launch.py` reads its arguments once and then starts every node it needs at
the same time, with no start-up ordering. That is safe because each node waits
safely until its inputs show up (section 3).

```
ros2 launch mobility_bringup mobility.launch.py domain:=D input:=I [include_sim] [demo] [publish_odom]
│
├─ 1. Resolve arguments
│     • parameter files, in this order: geometry.yaml → common.yaml → D.yaml → extra_params
│     • use_sim_time: gazebo=true, others=false (unless given explicitly)
│
├─ 2. Simulator / bridge (only for some domains)
│     ├─ gazebo + include_sim:=true → includes rover_bringup/rover_sim.launch.py (team repo)
│     │     gz sim ─┬─ robot_state_publisher, ros_gz_bridge (/clock …)
│     │             └─ create (spawns the rover) ──(on exit)──► joint_state_broadcaster spawner
│     │                    ──(on exit)──► spawners: steer_controller, wheel_controller,
│     │                                    rocker_coupling_controller,
│     │                                    steer_pid_controller (inactive), wheel_pid_controller (inactive)
│     │                                  + rover_kinematics, rocker_coupling, explicit_pid_control
│     │                                    (team nodes; left running, they lose control once
│     │                                    our adapter switches the controllers, see OPEN Q10)
│     └─ unity + start_endpoint:=true → ros_tcp_endpoint (127.0.0.1:10000)
│
├─ 3. Domain adapter: exactly one, chosen by domain
│     mock → mock_adapter │ gazebo → gazebo_adapter │ unity → unity_adapter │ real → can_adapter
│     (unity_control:=setpoint, the default: wheel_controller runs remote:=true, so the 8 loops run in Unity)
│     (unity_control:=twist: no adapter, kinematics or controller; Unity's servos follow /cmd_vel)
│
├─ 4. Core: identical in every domain
│     swerve_kinematics, wheel_controller
│     wheel_odometry only if publish_odom:=true or demo:=true (off by default: rover_ekf does localisation)
│
├─ 5. Operator input, chosen by input
│     keyboard          → teleop_key_window (Tk window)
│     keyboard_terminal → teleop_keyboard (inside gnome-terminal, set by keyboard_prefix)
│     joy               → joy_node + teleop_joy
│     none              → nothing (Nav2 or your own script publishes /cmd_vel)
│
└─ 6. demo:=true → scripted_drive, plus an OnProcessExit handler:
       when scripted_drive exits → Shutdown event → the whole launch stops
```

**Node count, for example**, with `domain:=gazebo include_sim:=true input:=keyboard`:
Gazebo and the team's nodes, plus 4 of ours: `gazebo_adapter`, `swerve_kinematics`,
`wheel_controller`, plus `teleop_key_window`.

---

## 2. Data flow (one control cycle)

```
 teleop_key_window / teleop_joy / teleop_keyboard / scripted_drive / Nav2
     │ /cmd_vel (TwistStamped)   /mobility/steer_mode (latched)   /mobility/estop (latched)
     ▼
 swerve_kinematics  100 Hz ──► /mobility/wheel_setpoints  (steer θ* + θ̇*, wheel ω*)
     │         └──────────────► /mobility/steer_aim (explicit mode) ──► teleop (shows the aim, C recentre)
     ▼
 wheel_controller   100 Hz (gazebo: 200) ──► /mobility/effort_cmd  (u ∈ [-1, 1] × 8)
     ▲                                  └─► /mobility/diagnostics (5 Hz)
     │ /mobility/joint_states (θ, θ̇, ω; canonical)
 ADAPTER  ◄──────────────────────────────── /mobility/effort_cmd
     │  in:  simulator / EKF feedback → renamed, sign- and scale-corrected → /mobility/joint_states
     │  out: u → torque (sim) or PWM duty (real)
     ▼
 mock plant │ Gazebo effort controllers │ Unity bridge │ CAN → MCU → SmartElex 30D
                                                   │
 wheel_odometry ◄── /mobility/joint_states ──► /mobility/wheel_odom
```

**One loop, start to finish:**
1. A key press sets a target twist. Teleop ramps it and publishes `/cmd_vel` at 50 Hz.
2. `swerve_kinematics` turns the twist into 4 steering angles and 4 wheel speeds.
3. `wheel_controller` compares those targets with the measured feedback and runs the
   8 PIDs, producing 8 efforts u.
4. The adapter turns u into the domain's actuation.
5. The plant moves, and the adapter publishes the new feedback. The loop closes.

---

## 3. Start-up timeline: how each node copes until its peers are up

| t | What happens |
|---|---|
| 0 s | All nodes start together. Every one of them declares its parameters from the YAML chain. |
| ~0 s | **swerve_kinematics** publishes setpoints right away. With no `/cmd_vel` it holds zero speed. Its steer setpoints start at 0 until the first `/mobility/joint_states` arrives. They are then *seeded* from the measured angles once, so the wheels don't snap. |
| ~0 s | **wheel_controller** sits in `feedback_stale` (or `disarmed` on real) and publishes u = 0 at its rate. |
| ~0 s | **Adapter**: it publishes `/mobility/joint_states` as soon as its source sends data. Mock: immediately, since it is its own source. Gazebo: once `/joint_states` flows. Unity: once the bridge publishes. Real: once the EKF topics arrive. |
| gazebo, ~5–15 s | **gazebo_adapter** retries `/controller_manager/switch_controller` every 1 s until the call succeeds. The call is STRICT: it activates `steer_pid_controller` and `wheel_pid_controller` and deactivates the team's `steer_controller` and `wheel_controller`. Until then it sends zero torque. |
| next tick | **wheel_controller** sees fresh feedback and setpoints and moves to `running`, resetting all integrators. Sim domains are `auto_arm: true`. Real needs `ros2 service call /mobility/arm std_srvs/srv/SetBool "{data: true}"`. |
| after | **teleop** publishes the initial mode (latched) and starts sending `/cmd_vel`. **scripted_drive** first waits `start_delay` (15 s in Gazebo). |

Latched topics (`steer_mode`, `estop`) reach late-joining subscribers. That's why
the start order doesn't matter.

---

## 4. Node by node

Each card lists: **starts from**, **in** (subscriptions), **out** (publications),
**timer**, **what one tick does**, and **when things go wrong**.

### 4.1 `mobility_hw` adapters: the only per-domain nodes

All four subclass `AdapterBase` (`mobility_hw/base.py`). The base class handles:
- **Joint map.** Each `joints.<canonical>.{name, topic, field, sign, scale, offset, cmd_sign}`
  entry describes one joint. Canonical joints are `fl_steer … br_drive`.
- **Ingest** (`start_joint_state_ingest`). It subscribes to every distinct source topic.
  For each sample it computes `value = sign·scale·raw + offset` and stores the latest one.
  Steer joints also pass their rate through when the source has one.
- **`publish_joint_states()`** sends the latest values on `/mobility/joint_states`.
  A joint whose source has sent nothing for `source_timeout` (0.15 s) is left out, so
  a dead EKF or simulator stream can't be re-sent forever as a frozen value. The
  controller then sees that joint go stale and stops. Mock joints have no external
  source and are always published.
- **Effort input.** It subscribes to `/mobility/effort_cmd`. `current_effort()` returns
  0 for every joint if no command has arrived for `effort_timeout` (0.25 s). This is the
  adapter watchdog.

| Adapter | Starts from | In | Out | Timer | One tick | On failure / shutdown |
|---|---|---|---|---|---|---|
| **mock_adapter** | `domain:=mock` | `/mobility/effort_cmd` | `/mobility/joint_states` | 200 Hz | For each of the 8 joints, computes torque τ = motor_model(u, ω), then integrates its `JointPlant` (inertia, viscous and Coulomb friction, steering end-stops) in substeps. Optionally adds noise and encoder quantisation. Then publishes. | Watchdog sets u = 0 and the plant coasts to a stop. |
| **gazebo_adapter** | `domain:=gazebo` | `/joint_states` (Gazebo, sensor QoS), `/mobility/effort_cmd` | `/mobility/joint_states`, `/steer_pid_controller/commands`, `/wheel_pid_controller/commands` (Float64MultiArray, N·m), plus a `switch_controller` service call | 200 Hz + 1 Hz switch retry | Feedback in: `FLE_joint` → `fl_steer` (sign −1) and so on; `BRD` sign −1. Effort out: u → DC motor model → N·m × cmd_sign → the two effort controllers. Sends 0 until the switch succeeds. | Shutdown: publishes zero torque. |
| **unity_adapter** | `domain:=unity` | `/unity/joint_states`, `/mobility/effort_cmd` | `/mobility/joint_states`; in `unity_control:=effort` only, also `/unity/effort_cmd` (Float64MultiArray `[steer×4, drive×4]`) | 100 Hz | The bridge already publishes canonical names and signs, so the joint map is all +1. **setpoint (default):** feedback only; the loops run in Unity. **effort:** sends raw u, and the bridge applies the motor model. | Warns every 5 s while Unity is silent. The bridge hands back to Unity's keyboard 0.5 s after its stream stops (`docs/UNITY_BRIDGE.md`). |
| **can_adapter** | `domain:=real` | EKF topics (`/wheel_{fl,fr,rl,rr}/steer_encoder`, placeholders), `/mobility/effort_cmd`, `/mobility/estop` | `/mobility/joint_states`, CAN frames | 100 Hz frames + 20 Hz heartbeat | u → `shape_duty` (sign, per-joint deadband lift, `max_duty` 0.9) → int16 → drive frame (0x101) and steer frame (0x102). The heartbeat (0x100) carries a counter plus `active` and `estop` flags. Backend: `dryrun` (logs, the default), `socketcan` or `virtual`. | E-stop → zero frames. Shutdown → 3 zero frames. The MCU must also run its own watchdog (`docs/CAN_PROTOCOL.md`). |

### 4.2 `swerve_kinematics` (mobility_ros/swerve_kinematics_node.py)
- **Starts from:** the launch file, always.
- **In:**
  - `/cmd_vel` (TwistStamped; plain Twist if `cmd_vel_stamped:=false`).
  - `/mobility/steer_mode` and `/mobility/estop`.
  - `/mobility/joint_states`, used only to seed the steer setpoints, once.
- **Out:**
  - `/mobility/wheel_setpoints` (JointState). `*_steer` carries position = θ* (rad) and
    velocity = θ̇*, which the controller uses for feedforward. `*_drive` carries
    velocity = ω* (rad/s); its position is NaN.
  - `/mobility/steer_aim` (Float64, explicit mode only).
- **Timer:** 100 Hz.
- **One tick:**
  1. Picks the command: the latest `/cmd_vel`, or (0, 0, 0) if none has arrived yet, the
     e-stop is engaged, or the last one is older than 0.5 s.
  2. Calls `SwerveKinematics.step(cmd, dt)` (`mobility_core/kinematics.py`):
     - **ackermann / crab:** each wheel's velocity is (vx − wz·y, vy + wz·x). That gives
       its angle and speed. The angle is folded by π to stay within ±steer_limit and to
       take the shorter turn. Ackermann also caps curvature at `min_turn_radius`.
     - **explicit:** `angular.z` is a steering *rate*, integrated into one shared aim
       angle; `linear.x` is the wheel speed for all 4 wheels.
     - **Every mode:** the angle is rate-limited, wheel speed is ramped, and speed is
       projected onto the wheel's current heading. At a zero command the wheels hold
       their angle instead of snapping to 0.
  3. Publishes the setpoints, with θ̇* = Δθ*/dt.
- **When things go wrong:** non-finite commands are dropped. An unknown mode is logged
  and ignored. An e-stop resets the motion ramps.

### 4.3 `wheel_controller` (mobility_ros/wheel_controller_node.py): the 8 PIDs
- **Starts from:** the launch file, always.
- **Remote mode** (`remote: true`, set by the launch for Unity `unity_control:=setpoint`):
  the node computes no u. It still runs the state machine below, and publishes:
  - `/mobility/controller_config`: String, JSON with the resolved per-joint gains; latched
    and re-sent at 1 Hz;
  - `/mobility/controller_enable`: Bool, true only in `running`.

  Unity's `MobilityBridge` runs the same loops (`MobilityPid.cs`, kept identical by
  `unity/parity/parity.py`) inside its physics step, and outputs u = 0 unless enabled.
- **In:**
  - `/mobility/wheel_setpoints` and `/mobility/joint_states`.
  - `/mobility/estop`.
  - Service `/mobility/arm` (SetBool).
- **Out:**
  - `/mobility/effort_cmd` (JointState, effort field, u ∈ [-1, 1]).
  - `/mobility/diagnostics` at 5 Hz: state, and per joint the setpoint, error, effort and
    saturation.
- **Timer:** `update_rate`, 100 Hz (200 Hz in gazebo.yaml).
- **One tick:**
  1. Checks the state, in priority order. It moves to `running` only when none of these
     apply, and it resets every integrator on entering `running`.
     - `estop`
     - `disarmed`
     - `feedback_stale`: any of the 8 joints missing from joint_states for 0.2 s
       (one dead wheel stops all of them)
     - `setpoints_stale`: no setpoints for 0.3 s
  2. In `running`, it calls `WheelController.update()` (`mobility_core/controller.py`):
     - **Steer** (×4): wrapped angle error, kp·e + I − kd·θ̇ (filtered) + kr·θ̇*, with a
       deadzone.
     - **Drive** (×4): target ω* × cos(steer error), plus feedforward kv·ω* + ks·sign(ω*),
       plus PID. A bounded hold integral keeps a parked rover in place.
     - **steer_release:** a parked wheel is allowed to roll while its steering turns.
     - Each output is clamped to ±1, with optional slew.
  3. Publishes u. When not running, all 8 values are 0.
- **When things go wrong / shutdown:** publishes a zero effort.

### 4.4 `wheel_odometry` (mobility_ros/wheel_odometry_node.py)
- **Starts from:** the launch file when `publish_odom:=true` or `demo:=true`. It is off by default.
- **In:** `/mobility/joint_states`.
- **Out:** `/mobility/wheel_odom` (nav_msgs/Odometry). It can also publish TF, but
  `publish_tf` is false by default; leave TF to the EKF / robot_localization.
- **Each message:**
  1. Computes `body_twist_from_wheels`, a least-squares fit of (vx, vy, wz) to the 4
     wheel velocity vectors (θ, ω·r).
  2. Integrates the pose.
  3. Publishes.
- Nothing in the control loop depends on it. Today its only user is the demo's checks.
  For localisation the team's `rover_ekf` already runs the same fit
  (`wheel_body_velocity.py` → `/wheel_odom/body`) and fuses it with the IMU, so use that.
  An IMU alone can't give speed: integrating acceleration drifts within seconds.

### 4.5 Operator input: all three share `TeleopCore` (mobility_ros/teleop_common.py)
`TeleopCore` holds:
- the mode;
- the speed scale;
- the slew limiter (accel 1, decel 2, yaw_accel 2);
- the e-stop toggle (re-sent at 2 Hz only while engaged);
- the explicit aim, read from `/mobility/steer_aim`, which the C recentre uses.

It publishes `/cmd_vel` (TwistStamped, `base_link`) and, latched, `/mobility/steer_mode`
and `/mobility/estop`.

| Node | Starts from | Extra in | Tick | Notes |
|---|---|---|---|---|
| **teleop_key_window** | `input:=keyboard` | Tk key press and release events | 50 Hz Tk `after()` loop, which also runs `rclpy.spin_once` | Several keys can be held at once (`KeyState`, with a 250 ms auto-repeat filter). All keys are released when the window loses focus. W/S drive. A/D turn, strafe or steer the aim, depending on mode. C, Shift, 1/2/3/M, +/−, Space, X, Esc. Closing it publishes zero. |
| **teleop_keyboard** | `input:=keyboard_terminal` (inside `keyboard_prefix`) | raw stdin | 50 Hz | Same keys, but a terminal can only hold one key at a time. Kept as a fallback. |
| **joy_node** | `input:=joy` | the device | (driver) | Standard ROS `joy` package. Publishes `/joy`. |
| **teleop_joy** | `input:=joy` | `/joy` | 50 Hz | Deadband plus expo on the axes. Dead-man button required. Buttons toggle the e-stop and cycle the mode. Axis and button numbers are in `common.yaml` (OPEN Q4). |

### 4.6 `scripted_drive` (mobility_ros/scripted_drive.py): only with `demo:=true`
- **In:** `/mobility/wheel_setpoints`, `/mobility/joint_states`, `/mobility/wheel_odom`.
- **Out:** `/cmd_vel` and `/mobility/steer_mode`.
- **Tick:** 50 Hz.
- **What it does:**
  1. Waits `start_delay`.
  2. Drives 11 segments: settle, forward, arc_left, reverse, stop, crab_45, crab_right,
     explicit_zero, explicit_aim, explicit_roll, final_stop.
  3. In each segment it records steer error (p95 < 3°) and drive error (mean < 10 % ·
     |target| + 0.1 rad/s). It prints the odometry twist for each segment. In
     `explicit_aim` the body must stay still (|v| < `still_tol`).
  4. Prints PASS or FAIL, then exits with 0 or 1.
- Its exit triggers the launch-wide Shutdown.

---

## 5. Shutdown sequence
On Ctrl-C (or when the demo ends):
1. `ros2 launch` sends SIGINT to every node.
2. Each node runs `C.run()` → `on_shutdown()` *before* the ROS context closes. This
   relies on `SignalHandlerOptions.NO`. Each node then sends its last safe output:
   - teleop: zero `/cmd_vel`;
   - wheel_controller: zero effort;
   - gazebo and unity adapters: zero torque / zero u;
   - can_adapter: 3 zero frames.
3. The team's Gazebo server sometimes survives SIGINT. Kill it with `pkill -f "^gz sim "`.

---

## 6. Topic and service reference

| Name | Type | Publisher → subscribers |
|---|---|---|
| `/cmd_vel` | TwistStamped | teleop / scripted_drive / Nav2 → swerve_kinematics |
| `/mobility/steer_mode` | String, latched | teleop / scripted_drive → swerve_kinematics |
| `/mobility/estop` | Bool, latched | teleop (or anything) → swerve_kinematics, wheel_controller, can_adapter |
| `/mobility/steer_aim` | Float64 | swerve_kinematics → teleop |
| `/mobility/wheel_setpoints` | JointState | swerve_kinematics → wheel_controller, scripted_drive |
| `/mobility/joint_states` | JointState | adapter → wheel_controller, swerve_kinematics (seed), wheel_odometry, scripted_drive |
| `/mobility/effort_cmd` | JointState (effort) | wheel_controller → adapter |
| `/mobility/wheel_odom` | Odometry | wheel_odometry → Nav2 / EKF / scripted_drive |
| `/mobility/diagnostics` | DiagnosticArray | wheel_controller → rqt / you |
| `/mobility/arm` | SetBool service | you → wheel_controller |
| gazebo: `/joint_states`, `/steer_pid_controller/commands`, `/wheel_pid_controller/commands` | | Gazebo ↔ gazebo_adapter |
| unity: `/unity/joint_states`, `/unity/effort_cmd` | | MobilityBridge (Unity) ↔ unity_adapter |
| real: EKF topics (placeholders), CAN 0x100 / 0x101 / 0x102 | | EKF → can_adapter → MCU |

**Useful checks while the stack runs:**
```zsh
ros2 node list                                   # check which nodes are up
ros2 topic hz /mobility/joint_states             # adapter alive
ros2 topic echo /mobility/diagnostics --once     # controller state and per-joint error
rqt_graph                                        # see the graph above, live
```
