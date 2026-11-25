using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Lets the ros_mobility stack (~/ros_mobility, domain:=unity) drive this rover, without
/// changing anything when that stack is not running.
///
/// Put it next to RoverDriveController on the rover prefab root. It does two things:
///
///   1. Always: publishes the eight joints on /unity/joint_states (fl_steer .. br_drive,
///      rad and rad/s, already in the ROS convention: steer positive left, drive positive
///      rolling forward). Signs come from RoverDriveController's own calibration.
///      Only while a ROS connection is up.
///
///   2. Only while ROS is really commanding: takes the rover over.
///        Effort mode (default): /unity/effort_cmd carries u in [-1, 1] per joint, the
///          output of the 8 PIDs in ros_mobility. This bridge turns u into torque with a
///          DC motor model every physics step, like a PWM driver does to a real motor:
///             torque = G*kt*(u*V - ke*G*w)/R
///          The u part is joint torque; the back-EMF part (G^2*kt*ke/R per rad/s) runs as
///          the joint drive's damping, which PhysX solves implicitly, so it stays stable
///          at Unity's 200 Hz even though ROS commands arrive less often.
///        Twist mode: /cmd_vel (TwistStamped) and /mobility/steer_mode go straight into
///          RoverDriveController.SetCommand, i.e. Unity's own tuned servos drive.
///
/// Take-over rules: ROS drives only while all of these hold, otherwise the keyboard
/// controls work exactly as before:
///   - enableRosControl is ticked,
///   - its command stream is fresh (a message within streamTimeout, real time),
///   - F8 has not latched "keyboard only",
///   - nobody else is driving through SetCommand (RoverDriveController.useKeyboard is on;
///     TrajectoryRecording turns it off, so a recording is never taken over).
/// When the stream stops, F8 is pressed or the component is disabled, the torques are
/// zeroed and RoverDriveController takes back over from where the joints are (it brakes
/// to a stop; it never coasts on a stale command).
/// </summary>
[DisallowMultipleComponent]
public class MobilityBridge : MonoBehaviour
{
    /// <summary>
    /// Auto: whatever ros_mobility sends. Setpoints + gains (wheel_controller remote:=true,
    /// the default unity_control:=setpoint) -> the 8 loops run HERE, in the physics step
    /// (MobilityPid.cs, a line-for-line port). Raw efforts (unity_control:=effort) -> applied
    /// as they come (only stable when Unity renders fast, e.g. headless).
    /// Twist: only /cmd_vel, Unity's own servos drive.
    /// </summary>
    public enum RosControl { Auto, Twist }
    enum Active { None, Setpoint, Effort, Twist }

    [Header("When ROS may drive")]
    public bool enableRosControl = true;
    [Tooltip("Effort: ros_mobility's 8 PIDs drive the joints. Twist: ROS only sends the " +
             "body command, Unity's own servos drive. Set before Play.")]
    public RosControl rosControl = RosControl.Auto;
    [Tooltip("Seconds (real time) without a command before control goes back to the keyboard. " +
             "Above the editor's worst frame time (~0.1 s at 10 fps).")]
    public float streamTimeout = 0.5f;
    [Tooltip("Toggles 'keyboard only': ROS can never take over while it is on.")]
    public KeyCode keyboardOnlyKey = KeyCode.F8;
    public bool keyboardOnly;

    [Header("Topics")]
    public string jointStateTopic = "/unity/joint_states";
    public string effortTopic = "/unity/effort_cmd";
    public string setpointTopic = "/mobility/wheel_setpoints";
    public string configTopic = "/mobility/controller_config";
    public string enableTopic = "/mobility/controller_enable";
    public string estopTopic = "/mobility/estop";
    public string controllerStateTopic = "/unity/controller_state";
    [Tooltip("Setpoint mode: advance the steering setpoint along its rate between ROS " +
             "messages (they arrive once per rendered frame), at most this long, s.")]
    public float maxSetpointExtrapolation = 0.15f;
    public string cmdVelTopic = "/cmd_vel";
    public string steerModeTopic = "/mobility/steer_mode";
    public bool publishJointStates = true;
    public float jointStateRate = 100f;   // Hz
    [Tooltip("Report each wheel's speed scaled by its collider radius relative to the mean, " +
             "so equal reported speeds mean equal ground speeds. The imported hulls differ ~1% " +
             "front to rear, which otherwise yaws the rover when crabbing.")]
    public bool compensateWheelRadii = true;

    [Header("Motor model (setpoint and effort modes); same numbers as ros_mobility unity.yaml")]
    public float busVoltage = 24f;          // V
    public float kt = 0.03f;                // N m / A, motor side
    public float ke = 0.03f;                // V s / rad, motor side
    public float resistance = 0.5f;         // ohm
    public float steerGearRatio = 100f;
    public float driveGearRatio = 50f;
    public float steerMaxTorque = 150f;     // N m, joint side
    public float driveMaxTorque = 300f;     // N m, joint side

    [Header("Steering joint friction (setpoint and effort modes), as in the URDF")]
    public bool modelSteerFriction = true;
    public float steerCoulombFriction = 0.5f;    // N m
    public float steerFrictionSmoothing = 0.1f;  // rad/s
    public float steerViscousFriction = 1f;      // N m s/rad

    [Header("State (read-only)")]
    public bool engaged;
    public string status = "idle";
    [Tooltip("Last u per joint: steer FL FR BL BR, drive FL FR BL BR.")]
    public float[] u = new float[8];
    [Tooltip("Setpoint mode: gains received from ROS (wheel_controller remote mode).")]
    public string gains = "none yet";
    public bool rosEnabled;
    public bool estop;

    RoverDriveController rc;
    ROSConnection ros;
    float cmdTime = -1e9f;                // realtimeSinceStartup of the last command
    float effortTime = -1e9f, setpointTime = -1e9f, enableTime = -1e9f;
    Active active = Active.None;

    // Setpoint mode: our controller, its inputs and outputs (FL FR BL BR, canonical units).
    Mobility.WheelController ctl;
    string configJson;
    bool wasRunning;
    double spStamp;                       // Time.timeAsDouble when the setpoints arrived
    readonly double[] spS = new double[4], spRate = new double[4], spD = new double[4];
    readonly double[] spUse = new double[4], pS = new double[4], rS = new double[4], vD = new double[4];
    readonly double[] uS = new double[4], uD = new double[4];
    readonly string[] spNames = { "fl_steer", "fr_steer", "bl_steer", "br_steer" };
    readonly string[] dNames = { "fl_drive", "fr_drive", "bl_drive", "br_drive" };
    Vector3 twist;
    string wantMode;
    bool tookKeyboard;
    bool wasConnected, sawCommand;
    float nextHint;

    [Header("Diagnostics")]
    [Tooltip("While ROS drives, log fps, command rate and command age every few seconds.")]
    public bool logLoopStats = true;
    int statCmds, statFrames;
    float statT0, statMaxAge, statMaxFrame;
    float pubAccum;
    readonly string[] names =
        { "fl_steer", "fr_steer", "bl_steer", "br_steer", "fl_drive", "fr_drive", "bl_drive", "br_drive" };
    readonly double[] jsPos = new double[8], jsVel = new double[8], jsEff = new double[0];

    // -------------------------------------------------------------------- setup

    /// <summary>
    /// Adds the bridge to every rover at Play, so no prefab or scene has to change.
    /// Set autoAttach to false (in code) to only use bridges added by hand.
    /// </summary>
    public static bool autoAttach = true;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AttachToRovers()
    {
        if (!autoAttach) return;
        foreach (var r in FindObjectsByType<RoverDriveController>(FindObjectsSortMode.None))
            if (r.GetComponent<MobilityBridge>() == null) r.gameObject.AddComponent<MobilityBridge>();
    }

    void Start()
    {
        rc = GetComponent<RoverDriveController>();
        if (rc == null) rc = FindFirstObjectByType<RoverDriveController>();
        if (rc == null)
        {
            Debug.LogError("[MobilityBridge] no RoverDriveController found; disabled.");
            enabled = false;
            return;
        }
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(jointStateTopic);
        if (rosControl == RosControl.Auto)
        {
            ros.RegisterPublisher<JointStateMsg>(controllerStateTopic);
            ros.Subscribe<JointStateMsg>(setpointTopic, OnSetpoints);
            ros.Subscribe<StringMsg>(configTopic, OnConfig);
            ros.Subscribe<BoolMsg>(enableTopic, m => { rosEnabled = m.data; enableTime = Time.realtimeSinceStartup; });
            ros.Subscribe<BoolMsg>(estopTopic, m => estop = m.data);
            ros.Subscribe<Float64MultiArrayMsg>(effortTopic, OnEffort);
        }
        else
        {
            ros.Subscribe<TwistStampedMsg>(cmdVelTopic, OnTwist);
            ros.Subscribe<StringMsg>(steerModeTopic, m => wantMode = m.data);
        }
        Debug.Log($"[MobilityBridge] ready: {rosControl} mode, joint states on {jointStateTopic}. " +
                  $"{keyboardOnlyKey} = keyboard only. Without ROS commands the rover keeps " +
                  "its own keyboard controls.");
    }

    void OnEffort(Float64MultiArrayMsg m)
    {
        if (m.data == null || m.data.Length < 8) return;
        for (int i = 0; i < 8; i++)
        {
            float v = (float)m.data[i];
            u[i] = float.IsFinite(v) ? Mathf.Clamp(v, -1f, 1f) : 0f;
        }
        effortTime = Time.realtimeSinceStartup;
        if (active != Active.Setpoint) { cmdTime = effortTime; statCmds++; }
        NoteFirstCommand();
    }

    void OnSetpoints(JointStateMsg m)
    {
        if (m.name == null) return;
        for (int k = 0; k < m.name.Length; k++)
        {
            string n = m.name[k];
            for (int i = 0; i < 4; i++)
            {
                if (n == spNames[i])
                {
                    if (m.position != null && k < m.position.Length && double.IsFinite(m.position[k])) spS[i] = m.position[k];
                    spRate[i] = m.velocity != null && k < m.velocity.Length && double.IsFinite(m.velocity[k]) ? m.velocity[k] : 0.0;
                }
                else if (n == dNames[i] && m.velocity != null && k < m.velocity.Length && double.IsFinite(m.velocity[k]))
                    spD[i] = m.velocity[k];
            }
        }
        spStamp = Time.timeAsDouble;
        setpointTime = Time.realtimeSinceStartup;
        if (active != Active.Effort) { cmdTime = setpointTime; statCmds++; }
        NoteFirstCommand();
    }

    void OnConfig(StringMsg m)
    {
        if (m.data == configJson) return;
        Mobility.ControllerConfig cfg;
        try { cfg = JsonUtility.FromJson<Mobility.ControllerConfig>(m.data); }
        catch (System.Exception e) { Debug.LogError($"[MobilityBridge] bad controller config: {e.Message}"); return; }
        if (cfg == null || cfg.joints == null || cfg.joints.Length < 8)
        {
            Debug.LogError("[MobilityBridge] controller config has fewer than 8 joints; ignored.");
            return;
        }
        configJson = m.data;
        ctl = new Mobility.WheelController(cfg);
        var s0 = ctl.steer[0].g; var d0 = ctl.drive[0].g;
        gains = $"steer kp {s0.kp} ki {s0.ki} kd {s0.kd} kr {s0.kr} | drive kp {d0.kp} ki {d0.ki} kv {d0.kv}";
        Debug.Log($"[MobilityBridge] controller gains from ROS: {gains}");
    }

    /// <summary>
    /// Loop timing while ROS drives. Messages from ROS are delivered once per rendered
    /// frame, so a low frame rate adds that much delay to every PID loop.
    /// </summary>
    void LoopStats()
    {
        if (!logLoopStats || !engaged) { statT0 = 0f; return; }
        float now = Time.realtimeSinceStartup;
        if (statT0 == 0f) { statT0 = now; statCmds = statFrames = 0; statMaxAge = statMaxFrame = 0f; return; }
        statMaxAge = Mathf.Max(statMaxAge, now - cmdTime);
        if (now - statT0 >= 3f)
        {
            float span = now - statT0;
            Debug.Log($"[MobilityBridge] loop: {statFrames / span:F0} fps (worst frame {statMaxFrame * 1000f:F0} ms), " +
                      $"{statCmds / span:F0} commands/s, worst command age {statMaxAge * 1000f:F0} ms, " +
                      $"physics {1f / Time.fixedDeltaTime:F0} Hz");
            statT0 = now; statCmds = statFrames = 0; statMaxAge = statMaxFrame = 0f;
        }
    }

    void NoteFirstCommand()
    {
        if (sawCommand) return;
        sawCommand = true;
        Debug.Log("[MobilityBridge] commands from ros_mobility arriving.");
    }

    /// <summary>Console hints, so "nothing happens" always says why.</summary>
    void ReportConnection()
    {
        bool connected = ros != null && ros.HasConnectionThread && !ros.HasConnectionError;
        if (connected != wasConnected)
        {
            wasConnected = connected;
            Debug.Log(connected
                ? "[MobilityBridge] connected to the ROS endpoint."
                : "[MobilityBridge] not connected to the ROS endpoint; keyboard controls as usual.");
        }
        if (connected && !engaged && Time.realtimeSinceStartup > nextHint)
        {
            nextHint = Time.realtimeSinceStartup + 10f;
            if (!enableRosControl) Debug.Log("[MobilityBridge] connected, but enableRosControl is off.");
            else if (keyboardOnly) Debug.Log($"[MobilityBridge] connected, but keyboard only is on ({keyboardOnlyKey}).");
            else if (!sawCommand)
                Debug.Log("[MobilityBridge] connected, but no commands yet: " +
                          "is ros_mobility running (domain:=unity)?");
            else if (rosControl == RosControl.Auto && ctl == null && effortTime < 0f)
                Debug.Log($"[MobilityBridge] setpoints arriving but no gains on {configTopic} yet " +
                          "(wheel_controller remote:=true is set by the launch for unity_control:=setpoint).");
        }
    }

    void OnTwist(TwistStampedMsg m)
    {
        var t = m.twist;
        var v = new Vector3((float)t.linear.x, (float)t.linear.y, (float)t.angular.z);
        if (!float.IsFinite(v.x) || !float.IsFinite(v.y) || !float.IsFinite(v.z)) return;
        twist = v;
        cmdTime = Time.realtimeSinceStartup;
        NoteFirstCommand();
    }

    // -------------------------------------------------------------------- loop

    void Update()
    {
        statFrames++;
        statMaxFrame = Mathf.Max(statMaxFrame, Time.unscaledDeltaTime);
        if (Input.GetKeyDown(keyboardOnlyKey))
        {
            keyboardOnly = !keyboardOnly;
            Debug.Log($"[MobilityBridge] keyboard only {(keyboardOnly ? "ON: ROS cannot drive" : "OFF")}");
        }
    }

    void FixedUpdate()
    {
        if (rc == null || !rc.IsReady) return;
        float dt = Time.fixedDeltaTime;
        ReportConnection();
        LoopStats();

        float now = Time.realtimeSinceStartup;
        Active want = Active.None;
        if (enableRosControl && !keyboardOnly)
        {
            if (rosControl == RosControl.Twist) { if (now - cmdTime < streamTimeout) want = Active.Twist; }
            else if (ctl != null && now - setpointTime < streamTimeout) want = Active.Setpoint;
            else if (now - effortTime < streamTimeout) want = Active.Effort;
        }
        if (engaged && want != active)
            Release(keyboardOnly ? "keyboard only" : !enableRosControl ? "ROS control disabled"
                    : want == Active.None ? "command stream stopped" : $"switching to {want}");
        if (!engaged && want != Active.None && rc.useKeyboard && rc.enabled) Engage(want);

        if (engaged)
        {
            if (active == Active.Setpoint) RunController(dt);
            if (active == Active.Twist) DriveTwist();
            else ApplyMotors();
        }

        pubAccum += dt;
        if (publishJointStates && pubAccum >= 1f / Mathf.Max(1f, jointStateRate))
        {
            pubAccum = 0f;
            PublishJointStates();
        }
    }

    // -------------------------------------------------------------------- hand-over

    void Engage(Active mode)
    {
        active = mode;
        if (mode != Active.Twist)
        {
            // Disabling the controller stops its FixedUpdate; its OnDisable also leaves
            // mode 5's torque mode cleanly.
            rc.enabled = false;
            for (int i = 0; i < 4; i++)
            {
                SetMotorDrive(rc.SteerJoint(i), BemfDamping(steerGearRatio), steerMaxTorque);
                SetMotorDrive(rc.DriveJoint(i), BemfDamping(driveGearRatio), driveMaxTorque);
            }
        }
        else
        {
            rc.useKeyboard = false;
            tookKeyboard = true;
        }
        engaged = true;
        wasRunning = false;
        if (ctl != null) ctl.Reset();
        status = $"ROS driving ({mode}{(mode == Active.Setpoint ? ": our 8 PIDs run in Unity at physics rate" : "")})";
        Debug.Log($"[MobilityBridge] {status}. {keyboardOnlyKey} = back to keyboard.");
    }

    void Release(string why)
    {
        if (active != Active.Twist)
        {
            for (int i = 0; i < 4; i++)
            {
                rc.SteerJoint(i).jointForce = new ArticulationReducedSpace(0f);
                rc.DriveJoint(i).jointForce = new ArticulationReducedSpace(0f);
            }
            // Its FixedUpdate re-applies the ideal drives on the next step.
            rc.ResyncFromMeasured();
            rc.enabled = true;
        }
        else
        {
            rc.SetCommand(0f, 0f, 0f);
            if (tookKeyboard) rc.useKeyboard = true;
            tookKeyboard = false;
        }
        engaged = false;
        active = Active.None;
        status = $"keyboard ({why})";
        for (int i = 0; i < 8; i++) u[i] = 0f;
        Debug.Log($"[MobilityBridge] back to keyboard: {why}");
    }

    void OnDisable()
    {
        if (engaged) Release("bridge disabled");
    }

    // -------------------------------------------------------------------- effort mode

    float BemfDamping(float gear) => gear * gear * kt * ke / Mathf.Max(1e-6f, resistance);
    float StallTorque(float gear) => gear * kt * busVoltage / Mathf.Max(1e-6f, resistance);

    static void SetMotorDrive(ArticulationBody j, float damping, float forceLimit)
    {
        ArticulationDrive d = j.xDrive;   // keeps the steering limits
        d.stiffness = 0f;
        d.damping = damping;              // N m per rad/s (measured in Unity 6000.1)
        d.forceLimit = forceLimit;
        d.target = j.jointPosition[0] * Mathf.Rad2Deg;
        d.targetVelocity = 0f;
        j.xDrive = d;
        j.jointForce = new ArticulationReducedSpace(0f);
    }

    void ApplyMotors()
    {
        float stallS = StallTorque(steerGearRatio), stallD = StallTorque(driveGearRatio);
        for (int i = 0; i < 4; i++)
        {
            // u is in the ROS convention; the joint's own sense comes from the calibration.
            var s = rc.SteerJoint(i);
            float tauS = Mathf.Clamp(u[i] * stallS, -steerMaxTorque, steerMaxTorque) * rc.SteerSign(i);
            if (modelSteerFriction) tauS += SteerFriction(s.jointVelocity[0]);
            s.jointForce = new ArticulationReducedSpace(tauS);

            var d = rc.DriveJoint(i);
            float tauD = Mathf.Clamp(u[4 + i] * stallD, -driveMaxTorque, driveMaxTorque) * rc.DriveSign(i);
            d.jointForce = new ArticulationReducedSpace(tauD);
        }
    }

    float SteerFriction(float w)
    {
        float s = Mathf.Max(1e-4f, steerFrictionSmoothing);
        return -steerCoulombFriction * Mathf.Clamp(w / s, -1f, 1f) - steerViscousFriction * w;
    }

    // -------------------------------------------------------------------- setpoint mode

    /// <summary>
    /// One step of ros_mobility's 8 loops, here at the physics rate: no transport delay
    /// inside the loop. ROS keeps the safety state machine: unless wheel_controller says
    /// "running" (armed, no e-stop, fresh feedback and setpoints), u = 0.
    /// </summary>
    void RunController(float dt)
    {
        float now = Time.realtimeSinceStartup;
        bool running = rosEnabled && now - enableTime < streamTimeout && !estop;
        if (!running)
        {
            for (int i = 0; i < 8; i++) u[i] = 0f;
            wasRunning = false;
            return;
        }
        if (!wasRunning) { ctl.Reset(); wasRunning = true; }   // fresh integrators, like wheel_controller

        // Setpoints arrive once per rendered frame; carry the steering setpoint along its
        // rate through the frame's burst of physics steps instead of holding it.
        double age = System.Math.Min(System.Math.Max(0.0, Time.fixedTimeAsDouble - spStamp), maxSetpointExtrapolation);
        for (int i = 0; i < 4; i++)
        {
            spUse[i] = spS[i] + spRate[i] * age;
            pS[i] = rc.MeasuredSteerAngle(i);
            rS[i] = rc.SteerSign(i) * rc.SteerJoint(i).jointVelocity[0];
            float k = compensateWheelRadii ? rc.WheelRadiusScale(i) : 1f;
            vD[i] = k * rc.DriveSign(i) * rc.DriveJoint(i).jointVelocity[0];
        }
        ctl.Update(spUse, spRate, spD, pS, rS, vD, dt, uS, uD);
        for (int i = 0; i < 4; i++)
        {
            u[i] = (float)uS[i];
            u[4 + i] = (float)uD[i];
        }
    }

    // -------------------------------------------------------------------- twist mode

    void DriveTwist()
    {
        if (!string.IsNullOrEmpty(wantMode))
        {
            switch (wantMode.Trim().ToLowerInvariant())
            {
                case "ackermann": rc.mode = RoverDriveController.SteerMode.Ackermann; break;
                case "crab": rc.mode = RoverDriveController.SteerMode.Crab; break;
                case "explicit": rc.mode = RoverDriveController.SteerMode.Explicit; break;
                default: Debug.LogWarning($"[MobilityBridge] unknown steer mode '{wantMode}'"); break;
            }
            wantMode = null;
        }
        rc.SetCommand(twist.x, twist.y, twist.z);
    }

    // -------------------------------------------------------------------- feedback

    void PublishJointStates()
    {
        if (ros == null || !ros.HasConnectionThread || ros.HasConnectionError) return;
        for (int i = 0; i < 4; i++)
        {
            var s = rc.SteerJoint(i);
            var d = rc.DriveJoint(i);
            jsPos[i] = rc.MeasuredSteerAngle(i);
            jsVel[i] = rc.SteerSign(i) * s.jointVelocity[0];
            float k = compensateWheelRadii ? rc.WheelRadiusScale(i) : 1f;
            jsPos[4 + i] = k * rc.DriveSign(i) * d.jointPosition[0];
            jsVel[4 + i] = k * rc.DriveSign(i) * d.jointVelocity[0];
        }
        ros.Publish(jointStateTopic, new JointStateMsg
        {
            header = new HeaderMsg(),
            name = names,
            position = (double[])jsPos.Clone(),
            velocity = (double[])jsVel.Clone(),
            effort = jsEff,
        });
        if (active == Active.Setpoint)
        {
            // What the in-Unity loops are doing: setpoint used (steer rad / drive target
            // rad/s after cos scaling) and u, for plotting and ros2 topic echo.
            var sp = new double[8]; var eff = new double[8];
            for (int i = 0; i < 4; i++)
            {
                sp[i] = spUse[i]; sp[4 + i] = ctl.driveTarget[i];
                eff[i] = u[i]; eff[4 + i] = u[4 + i];
            }
            ros.Publish(controllerStateTopic, new JointStateMsg
            {
                header = new HeaderMsg(), name = names, position = sp,
                velocity = new double[0], effort = eff,
            });
        }
    }
}
