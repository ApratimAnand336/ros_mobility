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
    public enum RosControl { Effort, Twist }

    [Header("When ROS may drive")]
    public bool enableRosControl = true;
    [Tooltip("Effort: ros_mobility's 8 PIDs drive the joints. Twist: ROS only sends the " +
             "body command, Unity's own servos drive. Set before Play.")]
    public RosControl rosControl = RosControl.Effort;
    [Tooltip("Seconds (real time) without a command before control goes back to the keyboard.")]
    public float streamTimeout = 0.25f;
    [Tooltip("Toggles 'keyboard only': ROS can never take over while it is on.")]
    public KeyCode keyboardOnlyKey = KeyCode.F8;
    public bool keyboardOnly;

    [Header("Topics")]
    public string jointStateTopic = "/unity/joint_states";
    public string effortTopic = "/unity/effort_cmd";
    public string cmdVelTopic = "/cmd_vel";
    public string steerModeTopic = "/mobility/steer_mode";
    public bool publishJointStates = true;
    public float jointStateRate = 100f;   // Hz
    [Tooltip("Report each wheel's speed scaled by its collider radius relative to the mean, " +
             "so equal reported speeds mean equal ground speeds. The imported hulls differ ~1% " +
             "front to rear, which otherwise yaws the rover when crabbing.")]
    public bool compensateWheelRadii = true;

    [Header("Motor model (effort mode); same numbers as ros_mobility unity.yaml")]
    public float busVoltage = 24f;          // V
    public float kt = 0.03f;                // N m / A, motor side
    public float ke = 0.03f;                // V s / rad, motor side
    public float resistance = 0.5f;         // ohm
    public float steerGearRatio = 100f;
    public float driveGearRatio = 50f;
    public float steerMaxTorque = 150f;     // N m, joint side
    public float driveMaxTorque = 300f;     // N m, joint side

    [Header("Steering joint friction (effort mode), as in the URDF")]
    public bool modelSteerFriction = true;
    public float steerCoulombFriction = 0.5f;    // N m
    public float steerFrictionSmoothing = 0.1f;  // rad/s
    public float steerViscousFriction = 1f;      // N m s/rad

    [Header("State (read-only)")]
    public bool engaged;
    public string status = "idle";
    [Tooltip("Last u per joint: steer FL FR BL BR, drive FL FR BL BR.")]
    public float[] u = new float[8];

    RoverDriveController rc;
    ROSConnection ros;
    float cmdTime = -1e9f;                // realtimeSinceStartup of the last command
    Vector3 twist;
    string wantMode;
    bool tookKeyboard;
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
        if (rosControl == RosControl.Effort)
            ros.Subscribe<Float64MultiArrayMsg>(effortTopic, OnEffort);
        else
        {
            ros.Subscribe<TwistStampedMsg>(cmdVelTopic, OnTwist);
            ros.Subscribe<StringMsg>(steerModeTopic, m => wantMode = m.data);
        }
        Debug.Log($"[MobilityBridge] ready: {rosControl} mode, joint states on {jointStateTopic}, " +
                  $"commands on {(rosControl == RosControl.Effort ? effortTopic : cmdVelTopic)}. " +
                  $"{keyboardOnlyKey} = keyboard only.");
    }

    void OnEffort(Float64MultiArrayMsg m)
    {
        if (m.data == null || m.data.Length < 8) return;
        for (int i = 0; i < 8; i++)
        {
            float v = (float)m.data[i];
            u[i] = float.IsFinite(v) ? Mathf.Clamp(v, -1f, 1f) : 0f;
        }
        cmdTime = Time.realtimeSinceStartup;
    }

    void OnTwist(TwistStampedMsg m)
    {
        var t = m.twist;
        var v = new Vector3((float)t.linear.x, (float)t.linear.y, (float)t.angular.z);
        if (!float.IsFinite(v.x) || !float.IsFinite(v.y) || !float.IsFinite(v.z)) return;
        twist = v;
        cmdTime = Time.realtimeSinceStartup;
    }

    // -------------------------------------------------------------------- loop

    void Update()
    {
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

        bool fresh = Time.realtimeSinceStartup - cmdTime < streamTimeout;
        bool want = enableRosControl && !keyboardOnly && fresh;
        if (!engaged && want && rc.useKeyboard && rc.enabled) Engage();
        else if (engaged && !want)
            Release(keyboardOnly ? "keyboard only" : !enableRosControl ? "ROS control disabled"
                                                   : "command stream stopped");

        if (engaged)
        {
            if (rosControl == RosControl.Effort) ApplyMotors();
            else DriveTwist();
        }

        pubAccum += dt;
        if (publishJointStates && pubAccum >= 1f / Mathf.Max(1f, jointStateRate))
        {
            pubAccum = 0f;
            PublishJointStates();
        }
    }

    // -------------------------------------------------------------------- hand-over

    void Engage()
    {
        if (rosControl == RosControl.Effort)
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
        status = $"ROS driving ({rosControl})";
        Debug.Log($"[MobilityBridge] {status}. {keyboardOnlyKey} = back to keyboard.");
    }

    void Release(string why)
    {
        if (rosControl == RosControl.Effort)
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
    }
}
