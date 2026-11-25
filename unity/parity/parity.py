#!/usr/bin/python3
"""Parity test: ros_mobility's Python WheelController vs Unity's C# port (MobilityPid.cs).

Drives both with the same random-but-physical input sequence (setpoint sweeps, noisy
feedback, zero-setpoint holds, saturation, resets, with and without a measured steer
rate) and fails if any output differs by more than 1e-9.

  python3 unity/parity/parity.py [path/to/MobilityPid.cs]
Needs Unity 6000.1.17f1 (its bundled Roslyn + .NET 6) at the path below.
"""
import json, math, os, random, subprocess, sys, tempfile

from mobility_core import PIDGains
from mobility_core.controller import WheelController

UNITY = os.environ.get("UNITY_EDITOR", "/home/apratim/Unity/Hub/Editor/6000.1.17f1/Editor")
HERE = os.path.dirname(os.path.abspath(__file__))
CS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "Assets", "Scripts",
                                                       "CoupledDiff", "MobilityPid.cs")
WHEELS = ["fl", "fr", "bl", "br"]


def make_controller():
    steer = PIDGains(kp=1.8, ki=0.8, kd=0.08, kr=0.15, i_limit=0.4, deadzone=0.004,
                     d_filter_tau=0.01, slew_rate=40.0)
    drive = PIDGains(kp=0.1, ki=0.6, kv=0.0625, ks=0.03, i_limit=0.4, hold_i_limit=0.3)
    over = {"br_drive": PIDGains(kp=0.35, ki=0.5, kv=0.06, ks=0.02, i_limit=0.4,
                                 out_limit=0.8)}
    return WheelController(WHEELS, steer, drive, cos_scaling=True, per_joint_gains=over,
                           steer_release=0.035)


def main():
    rnd = random.Random(7)
    dt, n = 0.005, 4000
    ctl = make_controller()
    cfg = ctl.config()
    steps, py_out = [], []
    th = [0.0] * 4; om = [0.0] * 4
    for k in range(n):
        t = k * dt
        sp_s = [0.9 * math.sin(0.7 * t + i) if (k // 800) % 2 == 0 else 0.0 for i in range(4)]
        sp_rate = [0.63 * math.cos(0.7 * t + i) if (k // 800) % 2 == 0 else 0.0 for i in range(4)]
        sp_d = [6.0 * math.sin(0.3 * t) if (k // 500) % 3 else 0.0 for _ in range(4)]
        th = [a + 0.02 * (s - a) + rnd.gauss(0, 0.003) for a, s in zip(th, sp_s)]
        om = [w + 0.05 * (s - w) + rnd.gauss(0, 0.05) for w, s in zip(om, sp_d)]
        no_rate = (k // 1000) == 2                       # exercise the finite difference
        srate = [None if no_rate else r + rnd.gauss(0, 0.01) for r in sp_rate]
        reset = k in (1500, 3200)
        if reset:
            ctl.reset()
        sp = {f"{w}_steer": sp_s[i] for i, w in enumerate(WHEELS)}
        sp.update({f"{w}_drive": sp_d[i] for i, w in enumerate(WHEELS)})
        pos = {f"{w}_steer": th[i] for i, w in enumerate(WHEELS)}
        vel = {f"{w}_drive": om[i] for i, w in enumerate(WHEELS)}
        if not no_rate:
            vel.update({f"{w}_steer": srate[i] for i, w in enumerate(WHEELS)})
        rates = {f"{w}_steer": sp_rate[i] for i, w in enumerate(WHEELS)}
        u = ctl.update(sp, pos, vel, dt, rates)
        py_out.append([u[f"{w}_steer"] for w in WHEELS] + [u[f"{w}_drive"] for w in WHEELS])
        steps.append({"sp_s": sp_s, "sp_rate": sp_rate, "sp_d": sp_d, "pos": th,
                      "srate": srate, "vel": om, "reset": reset})

    work = tempfile.mkdtemp(prefix="mobility_parity_")
    inp = os.path.join(work, "in.json")
    json.dump({"config": cfg, "dt": dt, "steps": steps}, open(inp, "w"))
    fw = os.path.join(UNITY, "Data", "NetCoreRuntime", "shared", "Microsoft.NETCore.App")
    fw = os.path.join(fw, sorted(os.listdir(fw))[-1])
    refs = [f"-r:{os.path.join(fw, f)}" for f in os.listdir(fw)
            if f.endswith(".dll") and (f.startswith("System.") or f in ("netstandard.dll", "mscorlib.dll"))]
    exe = os.path.join(work, "parity.dll")
    rsp = os.path.join(work, "csc.rsp")
    open(rsp, "w").write("\n".join(["-nologo", "-nostdlib+", "-target:exe", "-nullable:disable",
                                    f"-out:{exe}", *refs, CS, os.path.join(HERE, "ParityMain.cs")]))
    dotnet = os.path.join(UNITY, "Data", "NetCoreRuntime", "dotnet")
    csc = os.path.join(UNITY, "Data", "DotNetSdkRoslyn", "csc.dll")
    r = subprocess.run([dotnet, csc, f"@{rsp}"], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr); sys.exit("C# build failed")
    ver = os.path.basename(fw)
    json.dump({"runtimeOptions": {"tfm": "net6.0", "framework": {"name": "Microsoft.NETCore.App",
                                                                  "version": ver}}},
              open(os.path.join(work, "parity.runtimeconfig.json"), "w"))
    r = subprocess.run([dotnet, exe, inp], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:], r.stderr[-2000:]); sys.exit("C# run failed")
    cs_out = json.loads(r.stdout)
    worst, where = 0.0, None
    for k, (a, b) in enumerate(zip(py_out, cs_out)):
        for i, (x, y) in enumerate(zip(a, b)):
            d = abs(x - y)
            if d > worst:
                worst, where = d, (k, i)
    names = [f"{w}_steer" for w in WHEELS] + [f"{w}_drive" for w in WHEELS]
    sat = sum(1 for row in py_out for v in row if abs(v) >= 0.799)
    print(f"{n} steps x 8 joints, saturated samples {sat}, worst |python - C#| = {worst:.3g}"
          + (f" at step {where[0]} {names[where[1]]}" if where else ""))
    if len(cs_out) != len(py_out) or worst > 1e-9:
        sys.exit("PARITY FAIL")
    print("PARITY OK")


if __name__ == "__main__":
    main()
