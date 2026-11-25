using System;
using System.Globalization;
using System.IO;
using UnityEngine;

/// Logs the rover once per 0.5 s to a CSV (-traceOut) and quits after -runSeconds.
/// Columns: t, engaged, x, z, yaw_deg, speed, steer FL FR BL BR (deg), u x8.
public class BridgeTestRunner : MonoBehaviour
{
    float runSeconds = 240f, next;
    StreamWriter w;
    RoverDriveController rc;
    MobilityBridge br;
    ArticulationBody chassis;

    static string Arg(string n, string d)
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++) if (a[i] == n) return a[i + 1];
        return d;
    }

    void Start()
    {
        runSeconds = float.Parse(Arg("-runSeconds", "240"), CultureInfo.InvariantCulture);
        // -targetFps N caps the frame rate like the editor with cameras does; ROS messages
        // are delivered once per frame, so this reproduces the editor's loop delay.
        int fps = int.Parse(Arg("-targetFps", "0"), CultureInfo.InvariantCulture);
        if (fps > 0) { QualitySettings.vSyncCount = 0; Application.targetFrameRate = fps; }
        Debug.Log($"[BridgeTest] targetFrameRate {Application.targetFrameRate}");
        w = new StreamWriter(Arg("-traceOut", "bridge_trace.csv")) { AutoFlush = true };
        w.WriteLine("t,engaged,x,z,yaw_deg,speed,sFL,sFR,sBL,sBR,uSFL,uSFR,uSBL,uSBR,uDFL,uDFR,uDBL,uDBR");
        Debug.Log($"[BridgeTest] running {runSeconds} s, fixedDeltaTime {Time.fixedDeltaTime}");
    }

    void FixedUpdate()
    {
        if (rc == null) { rc = FindFirstObjectByType<RoverDriveController>(); return; }
        if (br == null) br = rc.GetComponent<MobilityBridge>();
        if (chassis == null) chassis = rc.chassis;
        float t = Time.realtimeSinceStartup;
        if (t >= next && chassis != null && rc.IsReady)
        {
            next = t + 0.5f;
            var p = chassis.transform.position;
            var c = CultureInfo.InvariantCulture;
            string s = string.Format(c, "{0:F2},{1},{2:F3},{3:F3},{4:F2},{5:F3}", t, br != null && br.engaged ? 1 : 0,
                                     p.x, p.z, chassis.transform.eulerAngles.y, chassis.linearVelocity.magnitude);
            for (int i = 0; i < 4; i++) s += string.Format(c, ",{0:F2}", rc.MeasuredSteerAngle(i) * Mathf.Rad2Deg);
            for (int i = 0; i < 8; i++) s += string.Format(c, ",{0:F3}", br != null ? br.u[i] : 0f);
            w.WriteLine(s);
        }
        if (Time.realtimeSinceStartup > runSeconds)
        {
            Debug.Log("[BridgeTest] done");
            w.Close();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.Exit(0);
#endif
        }
    }
}
