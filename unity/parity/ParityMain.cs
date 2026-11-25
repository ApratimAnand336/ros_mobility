// Runs Mobility.WheelController (Unity's C# port) on the inputs parity.py wrote and
// prints its outputs as JSON. Built and run by parity.py with Unity's bundled .NET 6.
using System;
using System.IO;
using System.Text.Json;
using Mobility;

static class ParityMain
{
    static double[] D(JsonElement e, string k)
    {
        var a = e.GetProperty(k); var r = new double[a.GetArrayLength()];
        int i = 0; foreach (var v in a.EnumerateArray()) r[i++] = v.ValueKind == JsonValueKind.Null ? double.NaN : v.GetDouble();
        return r;
    }

    static int Main(string[] args)
    {
        var doc = JsonDocument.Parse(File.ReadAllText(args[0])).RootElement;
        var cfgEl = doc.GetProperty("config");
        var cfg = new ControllerConfig
        {
            cos_scaling = cfgEl.GetProperty("cos_scaling").GetBoolean(),
            steer_release = cfgEl.GetProperty("steer_release").GetDouble(),
        };
        var list = new System.Collections.Generic.List<PidGains>();
        foreach (var j in cfgEl.GetProperty("joints").EnumerateArray())
        {
            var g = new PidGains();
            foreach (var f in typeof(PidGains).GetFields())
            {
                if (!j.TryGetProperty(f.Name, out var v)) continue;
                if (f.FieldType == typeof(double)) f.SetValue(g, v.GetDouble());
                else if (f.FieldType == typeof(bool)) f.SetValue(g, v.GetBoolean());
                else if (f.FieldType == typeof(string)) f.SetValue(g, v.GetString());
            }
            list.Add(g);
        }
        cfg.joints = list.ToArray();
        var ctl = new WheelController(cfg);
        double dt = doc.GetProperty("dt").GetDouble();
        var us = new double[4]; var ud = new double[4];
        using var w = new StreamWriter(Console.OpenStandardOutput());
        w.Write("[");
        bool first = true;
        foreach (var s in doc.GetProperty("steps").EnumerateArray())
        {
            if (s.TryGetProperty("reset", out var rs) && rs.GetBoolean()) ctl.Reset();
            ctl.Update(D(s, "sp_s"), D(s, "sp_rate"), D(s, "sp_d"), D(s, "pos"), D(s, "srate"), D(s, "vel"), dt, us, ud);
            if (!first) w.Write(",");
            first = false;
            w.Write("[" + string.Join(",", Array.ConvertAll(us, x => x.ToString("R"))) + "," +
                    string.Join(",", Array.ConvertAll(ud, x => x.ToString("R"))) + "]");
        }
        w.Write("]");
        return 0;
    }
}
