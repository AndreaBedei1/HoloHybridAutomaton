"""Calibrate the closed-loop plant used by the formal models (HoloOcean, BlueROV2).

Measures, with the *deployed* low-level controller (DVL velocity loop + compass heading):

1. braking: from 0.45 m/s cruise, command v=0 at each authority level -> stopping
   distance/time and effective deceleration (feeds ``Envelope.a_brake``);
2. reversal: from 0.45 m/s, command the opposite direction at escape authority;
3. current rejection: station keeping (v_d = 0, nominal authority) under lateral
   effective drift 0.25/0.40/0.60/0.80 m/s -> residual drift velocity (feeds
   ``Envelope.w_drift_max`` / ``current_drift_max``).

Writes results/calibration/plant_calibration.json and figures/calibration_plant.png.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import thruster_command  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.control.lowlevel import AUTHORITY, LowLevelController  # noqa: E402
from holo_fleet.perception.nav import DeadReckoning  # noqa: E402
from holo_fleet.sim.currents import drift_to_command  # noqa: E402

TPS = 30
DT = 0.1


def sensors():
    return [
        {"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "VelocitySensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "DVLSensor", "sensor_name": "DVLSensor", "socket": "DVLSocket", "Hz": 10,
         "configuration": {"VelSigma": 0.01, "ReturnRange": False}},
        {"sensor_type": "MagnetometerSensor", "sensor_name": "Compass", "socket": "IMUSocket", "Hz": 30,
         "configuration": {"Sigma": 0.005}},
        {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "DepthSensor", "sensor_name": "DepthSensor", "socket": "DepthSocket", "Hz": 30},
    ]


def main() -> int:
    import holoocean

    levels = ["nominal", "brake", "escape"]
    drifts = [0.25, 0.40, 0.60, 0.80]
    agents = []
    names = []
    for k, lvl in enumerate(levels):
        names.append(f"brake_{lvl}")
        agents.append({"agent_name": names[-1], "agent_type": "BlueROV2", "location": [-20.0, 30.0 + 8 * k, -5.0],
                       "rotation": [0, 0, 0], "control_scheme": 0, "sensors": sensors()})
    names.append("reverse_escape")
    agents.append({"agent_name": names[-1], "agent_type": "BlueROV2", "location": [-20.0, 30.0 + 8 * 3, -5.0],
                   "rotation": [0, 0, 0], "control_scheme": 0, "sensors": sensors()})
    for k, w in enumerate(drifts):
        names.append(f"hold_{w:.2f}")
        agents.append({"agent_name": names[-1], "agent_type": "BlueROV2", "location": [20.0, 30.0 + 8 * k, -5.0],
                       "rotation": [0, 0, 0], "control_scheme": 0, "sensors": sensors()})
    scen = {"name": "calib", "world": "OpenWater", "package_name": "Ocean", "main_agent": names[0],
            "ticks_per_sec": TPS, "frames_per_sec": False, "agents": agents}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False)
    env.reset()
    latest = {n: {} for n in names}
    nav = {n: DeadReckoning(np.array(a["location"], float), 0.0) for n, a in zip(names, agents)}
    low = {n: LowLevelController(DEFAULT) for n in names}
    log = {n: [] for n in names}
    t = 0.0
    T_CRUISE, T_END = 10.0, 25.0
    for step in range(int(T_END / DT)):
        for n in names:
            st = latest[n]
            s = nav[n].update(t, DT, st.get("Compass"), st.get("DVLSensor"), st.get("DepthSensor"), st.get("IMUSensor"))
            if n.startswith("hold_"):
                v_d, auth = np.zeros(3), "nominal"
                w = float(n.split("_")[1])
                env.set_ocean_currents(n, [0.0, drift_to_command(w) if t > 2.0 else 0.0, 0.0])
            elif n.startswith("brake_"):
                auth = n.split("_", 1)[1]
                v_d = np.array([0.45, 0.0, 0.0]) if t < T_CRUISE else np.zeros(3)
                if t < T_CRUISE:
                    auth = "nominal"
                env.set_ocean_currents(n, [0.0, 0.0, 0.0])
            else:  # reverse_escape
                v_d = np.array([0.45, 0.0, 0.0]) if t < T_CRUISE else np.array([-DEFAULT.env.v_escape, 0.0, 0.0])
                auth = "nominal" if t < T_CRUISE else "escape"
                env.set_ocean_currents(n, [0.0, 0.0, 0.0])
            v_d = v_d.copy()
            v_d[2] = float(np.clip(-0.6 * (s.p[2] - (-5.0)), -0.3, 0.3))
            cmd = low[n].command(s, v_d, 0.0, auth, DT)
            env.act(n, thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))
        for _ in range(int(round(DT * TPS))):
            raw = env.tick()
            for n in names:
                latest[n].update(raw.get(n, {}))
        t = round(t + DT, 6)
        for n in names:
            pose = np.asarray(latest[n]["PoseSensor"])
            v = np.asarray(latest[n]["VelocitySensor"]).ravel()
            log[n].append({"t": t, "x": float(pose[0, 3]), "y": float(pose[1, 3]), "z": float(pose[2, 3]),
                           "vx": float(v[0]), "vy": float(v[1]), "vz": float(v[2])})
    env.__exit__(None, None, None)

    res = {"authority_levels": AUTHORITY, "braking": {}, "reversal": {}, "station_keeping": {}}
    for lvl in levels:
        L = log[f"brake_{lvl}"]
        v0 = np.mean([r["vx"] for r in L if T_CRUISE - 1.0 <= r["t"] < T_CRUISE])
        x0 = [r["x"] for r in L if r["t"] >= T_CRUISE][0]
        stop = next((r for r in L if r["t"] > T_CRUISE and r["vx"] <= 0.02), L[-1])
        d_stop = stop["x"] - x0
        t_stop = stop["t"] - T_CRUISE
        res["braking"][lvl] = {"v0": round(float(v0), 3), "stop_distance_m": round(d_stop, 3), "stop_time_s": round(t_stop, 2),
                               "a_eff_from_distance": round(float(v0 ** 2 / (2 * max(d_stop, 1e-3))), 3),
                               "a_eff_from_time": round(float(v0 / max(t_stop, 1e-3)), 3)}
    L = log["reverse_escape"]
    x0 = [r["x"] for r in L if r["t"] >= T_CRUISE][0]
    xmax = max(r["x"] for r in L if r["t"] >= T_CRUISE)
    t_rev = next((r["t"] - T_CRUISE for r in L if r["t"] > T_CRUISE and r["vx"] <= -0.3), None)
    res["reversal"] = {"overshoot_m": round(xmax - x0, 3), "time_to_minus_0.3_m_s": t_rev}
    for w in drifts:
        L = log[f"hold_{w:.2f}"]
        late = [r for r in L if r["t"] >= 12.0]
        vy = float(np.mean([r["vy"] for r in late]))
        dy = L[-1]["y"] - L[0]["y"]
        res["station_keeping"][f"{w:.2f}"] = {"residual_drift_m_s": round(vy, 4), "net_displacement_m": round(dy, 3),
                                              "max_abs_vy": round(max(abs(r["vy"]) for r in L), 3)}
    out = ROOT / "results" / "calibration"
    out.mkdir(parents=True, exist_ok=True)
    (out / "plant_calibration.json").write_text(json.dumps(res, indent=2))
    (out / "plant_calibration_log.json").write_text(json.dumps(log))
    print(json.dumps(res, indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for lvl in levels:
        L = log[f"brake_{lvl}"]
        ax[0].plot([r["t"] for r in L], [r["vx"] for r in L], label=f"stop @ {lvl} ({AUTHORITY[lvl]:.2f})")
    L = log["reverse_escape"]
    ax[0].plot([r["t"] for r in L], [r["vx"] for r in L], "--", label="reverse @ escape")
    ax[0].axvline(T_CRUISE, color="k", lw=0.8, ls=":")
    ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("true surge velocity [m/s]"); ax[0].set_title("Braking / reversal (true velocity)")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    for w in drifts:
        L = log[f"hold_{w:.2f}"]
        ax[1].plot([r["t"] for r in L], [r["y"] - L[0]["y"] for r in L], label=f"drift {w:.2f} m/s")
    ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("lateral displacement [m]"); ax[1].set_title("Station keeping in lateral current (nominal authority)")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    (ROOT / "figures").mkdir(exist_ok=True)
    fig.savefig(ROOT / "figures" / "calibration_plant.png", dpi=140)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
