using System;

/// <summary>
/// C# port of ros_mobility's mobility_core/pid.py and controller.py (the 8 wheel loops),
/// line for line, so Unity can run the *same* controller inside its physics step.
///
/// Why it exists: over ROS-TCP, messages reach Unity once per rendered frame. In the
/// editor (mars_1, ~10 fps) that puts ~100 ms of delay inside the loop and the steering
/// oscillates. Run here at the physics rate (200 Hz), the loop has no transport delay,
/// like RoverDriveController's mode 5. ROS still owns kinematics, gains, arming and
/// e-stop; only the loop arithmetic runs here.
///
/// Keep it identical to the Python: ros_mobility/unity/parity/ runs both on the same
/// inputs and fails on any difference (run it after changing either side).
/// No UnityEngine dependency on purpose, so that test can run it outside Unity.
/// </summary>
namespace Mobility
{
    [Serializable]
    public class PidGains
    {
        public string name = "";          // joint, e.g. "fl_steer" (for the config message)
        public double kp, ki, kd;
        public double kv;                 // feedforward per unit setpoint
        public double ks;                 // static-friction feedforward * sign(setpoint)
        public double kr;                 // feedforward per unit setpoint rate
        public double out_limit = 1.0;
        public double i_limit = 1.0;
        public double hold_i_limit = -1.0; // < 0 means "not set" (Python None)
        public double d_filter_tau = 0.02;
        public double slew_rate;
        public double deadzone;
        public double zero_setpoint_eps = 1e-3;
        public bool angular;

        public PidGains Clone() => (PidGains)MemberwiseClone();
    }

    /// <summary>The resolved controller config ROS sends on /mobility/controller_config.</summary>
    [Serializable]
    public class ControllerConfig
    {
        public bool cos_scaling = true;
        public double steer_release;
        public PidGains[] joints = new PidGains[0];   // all 8, fully resolved
    }

    public static class MathUtil
    {
        /// <summary>Wrap to [-pi, pi), with Python's floor modulo.</summary>
        public static double WrapAngle(double a)
        {
            double twoPi = 2.0 * Math.PI;
            double m = (a + Math.PI) % twoPi;
            if (m < 0.0) m += twoPi;
            return m - Math.PI;
        }

        public static double Clamp(double x, double lo, double hi) => x < lo ? lo : x > hi ? hi : x;
    }

    public class Pid
    {
        public readonly PidGains g;
        public double integral, dState, output, error;
        public bool saturated;
        double prevMeas;
        bool havePrev;

        public Pid(PidGains gains) { g = gains; Reset(); }

        public void Reset()
        {
            integral = 0.0; havePrev = false; prevMeas = 0.0;
            dState = 0.0; output = 0.0; saturated = false; error = 0.0;
        }

        /// <param name="measuredRate">NaN = not available: finite-difference the measurement.</param>
        public double Update(double setpoint, double measurement, double dt,
                             double measuredRate = double.NaN, double setpointRate = 0.0)
        {
            if (dt <= 0.0 || !double.IsFinite(setpoint) || !double.IsFinite(measurement))
                return output;
            double e = setpoint - measurement;
            if (g.angular) e = MathUtil.WrapAngle(e);
            if (Math.Abs(e) < g.deadzone) e = 0.0;
            error = e;

            // Derivative on measurement (no kick on setpoint steps), filtered.
            if (double.IsNaN(measuredRate))
                measuredRate = !havePrev ? 0.0 :
                    (g.angular ? MathUtil.WrapAngle(measurement - prevMeas) : measurement - prevMeas) / dt;
            prevMeas = measurement; havePrev = true;
            double dRaw = -g.kd * measuredRate;
            if (g.d_filter_tau > 0.0)
            {
                double a = dt / (g.d_filter_tau + dt);
                dState += a * (dRaw - dState);
            }
            else dState = dRaw;

            double sign = Math.Abs(setpoint) < g.zero_setpoint_eps ? 0.0 : (setpoint >= 0.0 ? 1.0 : -1.0);
            double ff = g.kv * setpoint + g.ks * sign + g.kr * setpointRate;
            double p = g.kp * e;

            // Conditional integration: skip if it would push a saturated output further.
            double iLim = g.i_limit;
            if (sign == 0.0 && g.hold_i_limit >= 0.0) iLim = g.hold_i_limit;
            double candidate = MathUtil.Clamp(integral + g.ki * e * dt, -g.i_limit, g.i_limit);
            double uTry = ff + p + candidate + dState;
            if (Math.Abs(uTry) < g.out_limit || uTry * e < 0.0) integral = candidate;
            integral = MathUtil.Clamp(integral, -iLim, iLim);

            double uRaw = ff + p + integral + dState;
            double u = MathUtil.Clamp(uRaw, -g.out_limit, g.out_limit);
            saturated = Math.Abs(uRaw) >= g.out_limit;
            if (g.slew_rate > 0.0)
            {
                double step = g.slew_rate * dt;
                u = MathUtil.Clamp(u, output - step, output + step);
            }
            output = u;
            return u;
        }
    }

    /// <summary>4 steer position loops + 4 drive velocity loops, wheels FL FR BL BR.</summary>
    public class WheelController
    {
        public static readonly string[] Wheels = { "fl", "fr", "bl", "br" };
        public readonly Pid[] steer = new Pid[4], drive = new Pid[4];
        public bool cosScaling = true;
        public double steerRelease;
        public readonly double[] driveTarget = new double[4];

        public WheelController(ControllerConfig cfg)
        {
            cosScaling = cfg.cos_scaling;
            steerRelease = cfg.steer_release;
            for (int i = 0; i < 4; i++)
            {
                var sg = Find(cfg, Wheels[i] + "_steer") ?? new PidGains();
                sg.angular = true;
                steer[i] = new Pid(sg);
                drive[i] = new Pid(Find(cfg, Wheels[i] + "_drive") ?? new PidGains());
            }
        }

        static PidGains Find(ControllerConfig cfg, string name)
        {
            foreach (var j in cfg.joints) if (j != null && j.name == name) return j.Clone();
            return null;
        }

        public void Reset()
        {
            for (int i = 0; i < 4; i++) { steer[i].Reset(); drive[i].Reset(); }
        }

        /// <summary>
        /// All arrays FL FR BL BR, canonical units (rad, rad/s). uSteer/uDrive are filled.
        /// steerRate: measured steering rate (NaN = finite-difference).
        /// </summary>
        public void Update(double[] spSteer, double[] spSteerRate, double[] spDrive,
                           double[] posSteer, double[] steerRate, double[] velDrive, double dt,
                           double[] uSteer, double[] uDrive)
        {
            for (int i = 0; i < 4; i++)
            {
                uSteer[i] = steer[i].Update(spSteer[i], posSteer[i], dt, steerRate[i], spSteerRate[i]);
                double target = spDrive[i];
                double err = MathUtil.WrapAngle(spSteer[i] - posSteer[i]);
                if (cosScaling) target *= Math.Max(0.0, Math.Cos(err));   // don't push while turning
                driveTarget[i] = target;
                if (steerRelease > 0.0 && Math.Abs(target) < 1e-6 && Math.Abs(err) > steerRelease)
                {
                    drive[i].Reset();
                    uDrive[i] = 0.0;
                }
                else uDrive[i] = drive[i].Update(target, velDrive[i], dt);
            }
        }
    }
}
