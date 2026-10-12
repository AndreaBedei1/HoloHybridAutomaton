"""Head-current authority of the deployed low-level loop (HoloOcean, BlueROV2, one short session).

    python probe/probe_head_current_authority.py

Seven BlueROV2s in one OpenWater session, no sonars.  Each one runs the deployed LowLevelController
(DVL velocity loop + compass heading, NOMINAL authority) with the survey velocity as desired velocity:
0.30 m/s along +x, heading +x, depth held at -5 m.  From t = 2 s an effective current drift ramps up
in 2 s (sin^2, as the scenario jets) and then stays constant:

* six head currents against the motion: 0.30, 0.35, 0.40, 0.45, 0.50, 0.60 m/s;
* one lateral current of 0.60 m/s, as a cross-check of the lateral bound at survey speed.

Measured on t in [14, 32] s (more than five integrator time constants after the ramp):
* mean ground velocity and mean through-water speed |v_ground - w|;
* fraction of control steps with a saturated horizontal command, longest saturated stretch;
* whether the onboard EnvelopeMonitor rule (leaky 4 s of saturation) would declare ENVELOPE_VIOLATION.

The through-water speed of the saturated drones measures the speed available at the nominal authority
(Envelope.v_tw_nominal); the unsaturated ones show where the survey velocity is still held.
Writes results/calibration/head_current_authority.json (+ _log.json, 2 Hz samples).  Ground truth
(PoseSensor, VelocitySensor) is used here for measuring only, as in scripts/calibrate_plant.py.
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
V_SURVEY = 0.30
T_ON, RAMP, T_END = 2.0, 2.0, 32.0
WINDOW = (14.0, 32.0)
CASES = [("head", 0.30), ("head", 0.35), ("head", 0.40), ("head", 0.45), ("head", 0.50), ("head", 0.60),
         ("lateral", 0.60)]


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


def drift_vector(kind: str, w: float, t: float) -> np.ndarray:
    if t < T_ON:
        return np.zeros(3)
    x = min((t - T_ON) / RAMP, 1.0)
    s = math.sin(0.5 * math.pi * x) ** 2
    return s * (np.array([-w, 0.0, 0.0]) if kind == "head" else np.array([0.0, w, 0.0]))


def leaky_violation(sat, dt=DT, t_enter=4.0):
    """The EnvelopeMonitor rule: +dt while saturated, -dt otherwise; violation when >= t_enter."""
    acc = 0.0
    for i, s in enumerate(sat):
        acc = acc + dt if s else max(0.0, acc - dt)
        if acc >= t_enter - 1e-9:
            return i
    return None


def main() -> int:
    import holoocean

    names, agents = [], []
    for k, (kind, w) in enumerate(CASES):
        names.append(f"{kind}_{w:.2f}")
        agents.append({"agent_name": names[-1], "agent_type": "BlueROV2", "location": [-20.0, 30.0 + 8 * k, -5.0],
                       "rotation": [0, 0, 0], "control_scheme": 0, "sensors": sensors()})
    scen = {"name": "head_authority", "world": "OpenWater", "package_name": "Ocean", "main_agent": names[0],
            "ticks_per_sec": TPS, "frames_per_sec": False, "agents": agents}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False)
    env.reset()
    latest = {n: {} for n in names}
    nav = {n: DeadReckoning(np.array(a["location"], float), 0.0) for n, a in zip(names, agents)}
    low = {n: LowLevelController(DEFAULT) for n in names}
    log = {n: [] for n in names}
    t = 0.0
    for _step in range(int(T_END / DT)):
        for (kind, w), n in zip(CASES, names):
            st = latest[n]
            s = nav[n].update(t, DT, st.get("Compass"), st.get("DVLSensor"), st.get("DepthSensor"), st.get("IMUSensor"))
            drift = drift_vector(kind, w, t)
            mag = float(np.linalg.norm(drift))
            cmd_vec = [0.0, 0.0, 0.0] if mag < 1e-9 else list(drift_to_command(mag) * drift / mag)
            env.set_ocean_currents(n, cmd_vec)
            v_d = np.array([V_SURVEY, 0.0, float(np.clip(-0.6 * (s.p[2] - (-5.0)), -0.3, 0.3))])
            cmd = low[n].command(s, v_d, 0.0, "nominal", DT)
            env.act(n, thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))
            log[n].append({"t": round(t, 2), "sat": bool(low[n].saturated), "drift": [round(float(v), 4) for v in drift],
                           "cur_est": [round(float(v), 4) for v in low[n].current_estimate()],
                           "cmd": [round(cmd["surge"], 4), round(cmd["sway"], 4)]})
        for _ in range(int(round(DT * TPS))):
            raw = env.tick()
            for n in names:
                latest[n].update(raw.get(n, {}))
        t = round(t + DT, 6)
        for n in names:
            pose = np.asarray(latest[n]["PoseSensor"])
            v = np.asarray(latest[n]["VelocitySensor"]).ravel()
            log[n][-1].update({"x": round(float(pose[0, 3]), 4), "y": round(float(pose[1, 3]), 4),
                               "vx": round(float(v[0]), 4), "vy": round(float(v[1]), 4), "vz": round(float(v[2]), 4)})
    env.__exit__(None, None, None)

    res = {"authority": AUTHORITY["nominal"], "survey_speed_m_s": V_SURVEY, "window_s": list(WINDOW),
           "definition": "drift = steady drift of an unactuated BlueROV2 (Envelope convention); through-water speed = "
                         "|v_ground - drift| (true velocity); saturated = |(surge, sway)| command at the authority limit",
           "cases": {}}
    for (kind, w), n in zip(CASES, names):
        L = log[n]
        win = [r for r in L if WINDOW[0] <= r["t"] <= WINDOW[1]]
        vg = np.array([[r["vx"], r["vy"]] for r in win])
        dr = np.array([r["drift"][:2] for r in win])
        tw = np.linalg.norm(vg - dr, axis=1)
        sat = [r["sat"] for r in win]
        longest, cur = 0, 0
        for s_ in sat:
            cur = cur + 1 if s_ else 0
            longest = max(longest, cur)
        i_v = leaky_violation([r["sat"] for r in L])
        sat_tw = tw[np.array(sat, dtype=bool)]
        res["cases"][n] = {
            "kind": kind, "drift_m_s": w,
            "mean_ground_velocity_m_s": [round(float(x), 3) for x in vg.mean(axis=0)],
            "along_track_deficit_m_s": round(V_SURVEY - float(vg[:, 0].mean()), 3),
            "mean_through_water_speed_m_s": round(float(tw.mean()), 3),
            "through_water_speed_when_saturated_m_s": round(float(sat_tw.mean()), 3) if len(sat_tw) else None,
            "saturated_fraction": round(float(np.mean(sat)), 3),
            "longest_saturation_s": round(DT * longest, 1),
            "monitor_violation_at_s": None if i_v is None else L[i_v]["t"],
            "current_estimate_m_s": round(float(np.mean([np.linalg.norm(r["cur_est"][:2]) for r in win])), 3),
        }
    out = ROOT / "results" / "calibration"
    out.mkdir(parents=True, exist_ok=True)
    (out / "head_current_authority.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    (out / "head_current_authority_log.json").write_text(
        json.dumps({n: [r for r in L if int(round(r["t"] * 10)) % 5 == 0] for n, L in log.items()}), encoding="utf-8")
    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
