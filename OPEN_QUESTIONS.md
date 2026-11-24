# Open questions: answers needed from the team

Every item below has a **placeholder** in the code today and is **one config edit** to
change, unless marked *code*. Answer them in this file, keeping the numbering, so the
configs can be updated in one pass.

## A. Real-rover hardware (these block running on the rover)

| # | Question | Placeholder now | Where it lives |
|---|---|---|---|
| Q1 | **EKF feedback topics.** For each wheel: topic name, message type (JointState? Float64? custom?), joint names inside it, units (steer rad or deg? wheel rad/s or m/s?) and publish rate. Who runs that EKF node? | Per-wheel `sensor_msgs/JointState` on `/wheel_{fl,fr,rl,rr}/steer_encoder` with names `[drive, steer]`, in rad and rad/s, at ≥ 50 Hz (copied from `rover_ekf`'s `steer_wheel_splitter`) | `config/real.yaml` → `can_adapter.joints.*` (`topic`, `name`, `field`, `scale`). A non-JointState type needs a small *code* addition in `mobility_hw/base.py` |
| Q2 | **CAN protocol.** Bitrate, frame IDs, payload layout, standard or extended IDs, endianness. Which MCU, and who writes its firmware? Does the MCU watchdog exist, and with what timeout? Does the MCU send anything back (currents, faults)? | 0x100 heartbeat, 0x101 drive, 0x102 steer; int16 duty ×32767, little-endian; 100 ms watchdog. See `docs/CAN_PROTOCOL.md` | `real.yaml` → `can_adapter.can.*`; layout in `mobility_hw/can_protocol.py` |
| Q3 | **Motors and drivers.** For drive and steer motors: gear ratio, rated voltage, kt/ke (or no-load speed and stall torque), winding resistance. SmartElex 30D minimum useful duty (the deadband). Max duty you allow. Bus voltage. | 24 V; G = 50 drive / 100 steer; kt = ke = 0.03; R = 0.5 Ω; deadband 0.05; max duty 0.9 | `real.yaml` (`deadband`, `max_duty`); sim motor models in `mock.yaml` / `gazebo.yaml` (`*_motor`) |
| Q4 | **Steering mechanics.** Real steering range (cable or hard-stop limits) per wheel. Is 0 straight ahead? Does positive θ from the EKF mean turned left (counter-clockwise from above)? | ±90° (`steer_limit: 1.5708`); θ already zeroed; positive = left | `geometry.yaml` → `steer_limit`; `real.yaml` → `joints.*.sign`, `offset` |
| Q5 | **Motor polarity.** For each of the 8 motors: does a positive duty move the joint in the encoder's positive direction? Check on a test stand. | Same as the encoder sign | `real.yaml` → `joints.<joint>.cmd_sign` |
| Q6 | **Joystick.** Model, and which axis or button should do each of: forward, strafe, turn/steer, speed scale, deadman, e-stop, mode change. Is a deadman wanted? | Xbox-style layout: left stick = vx/vy, right stick X = turn, LB = deadman, B = e-stop, A = mode, RB = speed up | `config/common.yaml` → `teleop_joy.*` (find indices with `ros2 topic echo /joy`) |
| Q7 | **Geometry check.** Are the real wheelbase, track and wheel radius the CAD values (0.7447 m, 0.436 m, 0.14985 m)? | CAD values | `config/geometry.yaml` |
| Q8 | **Hardware e-stop.** Is there a physical e-stop (cutting motor power)? Should it be visible to ROS (GPIO, CAN)? | Software e-stop only (`/mobility/estop`, joystick button) | New input → publish `std_msgs/Bool` on `/mobility/estop` |

## B. Integration and team decisions

| # | Question | Default now |
|---|---|---|
| Q9 | **Where does this code live?** A standalone repo, or the placeholder `rover_controls` package in `ROS2_Autonomous_2026_27` (following CONTRIBUTING: your-name/package-branch, PR to main)? | Standalone in `~/ros_mobility` |
| Q10 | **Gazebo coexistence.** `rover_bringup/rover_sim.launch.py` always starts its own `rover_kinematics` and `explicit_pid_control` nodes. They are harmless now, because our adapter switches their controllers off. Should we add a `drive_stack:=external` argument there (a small PR) so they don't start at all? | Leave the reference repo untouched |
| Q11 | **Unity follow-ups.** The `MobilityBridge` is in (2026-11-24, `docs/UNITY_BRIDGE.md`). Still open: should Unity publish `/clock` (sim time)? When can Unity's own mode-5 PID (`ExplicitPidController`) be retired? Should `ROS_unity` become a git repo so the C# changes are tracked there? | Bridge done; `/clock` not published; mode 5 kept |
| Q13 | **Nav2.** MPPI currently uses `motion_model: DiffDrive` with `/cmd_vel` as TwistStamped. Do we want Omni later, to use crab? Our stack accepts TwistStamped (`cmd_vel_stamped: true`). | Ackermann mode handles Nav2's vx + wz |
| Q14 | **Default joystick mode and mode button.** | Start in ackermann; one button cycles ackermann → crab → explicit |
| Q15 | **Real-rover gains.** These get tuned on the rover, wheels off the ground first, then on the ground. Do you want a tuning helper (step-response recorder and plotter)? | Gains copied from the sim (`common.yaml`) |
| Q16 | **PID loop rate on the rover.** The PIDs should run at the rate fresh feedback arrives. What rate does the per-wheel EKF publish θ and ω at, and with how much delay after the encoder sample? Do steer and drive come at the same rate? How often can the MCU accept duty frames (CAN load at 100 Hz is about 5 % of 500 kbit)? Should the loop run on its own timer, or once per EKF message? | `wheel_controller.update_rate: 100` (200 in Gazebo); `swerve_kinematics` 100 Hz; `can_adapter` 100 Hz frames + 20 Hz heartbeat; `feedback_timeout` 0.2 s, `source_timeout` 0.15 s |

## C. Already decided (for reference)
- Modes are ackermann, crab and explicit. Spot is dropped, and explicit_pid is merged
  into explicit, because every mode now goes through our PIDs.
- The PIDs run on the PC in ROS, and the MCU only does PWM/DIR plus a watchdog.
- Python (rclpy), ROS 2 Jazzy, our own nodes (not ros2_control).
- Drivers are SmartElex 30D over CAN; feedback is EKF-fused θ and ω.
- (Was Q12) Localisation and `odom → base_link` belong to the team's `rover_ekf` (wheel
  fit + IMU). Our `wheel_odometry` is off by default and runs only for the demo checks
  (decided 2026-11-24).
