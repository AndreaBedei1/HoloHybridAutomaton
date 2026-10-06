"""Calibrate the proximity-sonar neighbour estimator against ground truth (static aspects + motion lag).

Uses the deployed ProximityProcessor on real HoloOcean ring data.  Own pose comes from the PoseSensor
here because this is a SENSOR CALIBRATION probe, not a controller.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import holoocean

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception.proximity import ProximityProcessor  # noqa: E402
from holo_fleet.perception.sensor_suite import onboard_sensor_configs, ring_name  # noqa: E402

RINGS = [ring_name(e) for e in DEFAULT.perc.ring_elevations_deg]


def rot(yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def main():
    sensors = onboard_sensor_configs(DEFAULT, with_fls=False, with_camera=False)
    sensors.append({"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": 30})
    scen = {"name": "bias", "world": "OpenWater", "package_name": "Ocean", "main_agent": "A", "ticks_per_sec": 30,
            "frames_per_sec": False, "agents": [
                {"agent_name": "A", "agent_type": "BlueROV2", "location": [0, 0, -5], "rotation": [0, 0, 0],
                 "control_scheme": 0, "sensors": sensors},
                {"agent_name": "T", "agent_type": "BlueROV2", "location": [20, 20, -5], "rotation": [0, 0, 0],
                 "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "sensor_name": "PoseSensor",
                                                   "socket": "IMUSocket", "Hz": 30}]}]}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    proc = ProximityProcessor([], DEFAULT)
    results = {"static": [], "dynamic": []}

    def tick_hold(pa, ya, pt, yt, n=6):
        st = {}
        for _ in range(n):
            env.agents["A"].teleport(list(pa), [0, 0, ya])
            env.agents["T"].teleport(list(pt), [0, 0, yt])
            env.act("A", np.zeros(8))
            env.act("T", np.zeros(8))
            raw = env.tick()
            st.setdefault("A", {}).update(raw.get("A", {}))
            st.setdefault("T", {}).update(raw.get("T", {}))
        return st

    # ---- static aspects
    for (rel, yt, ya) in [((3, 0, 0), 0, 0), ((3, 0, 0), 180, 0), ((3, 0, 0), 90, 0), ((0, 3, 0), 0, 0), ((0, 3, 0), 90, 0),
                          ((2.1, 2.1, 0), 0, 0), ((2.1, 2.1, 0), 225, 0), ((-3, 0, 0), 0, 0), ((1.8, 0, 0), 180, 0),
                          ((5, 0, 0), 180, 0), ((3, 0, 0.6), 180, 0), ((3, 0, 0), 180, 90)]:
        pa = np.array([0.0, 0.0, -5.0])
        pt = pa + rot(ya) @ np.array(rel, dtype=float)
        st = tick_hold(pa, ya, pt, yt)
        PA = np.asarray(st["A"]["PoseSensor"])[:3, 3]
        PT = np.asarray(st["T"]["PoseSensor"])[:3, 3]
        res = proc.process(0.0, {n: st["A"][n] for n in RINGS}, PA, rot(ya))
        true_rel = PT - PA
        if res.detections:
            d = min(res.detections, key=lambda x: np.linalg.norm(x.rel - true_rel))
            err = d.rel - true_rel
            nearest = d.range_m
        else:
            err, nearest = None, None
        results["static"].append({"rel_body": rel, "target_yaw": yt, "observer_yaw": ya,
                                  "err_world": None if err is None else np.round(err, 3).tolist(),
                                  "err_along_los": None if err is None else round(float(err @ true_rel / np.linalg.norm(true_rel)), 3),
                                  "nearest_return": nearest, "true_dist": round(float(np.linalg.norm(true_rel)), 3)})
        print("static", results["static"][-1])
    # ---- dynamic: both move toward each other at 0.3 m/s (kinematic teleport each tick)
    pa = np.array([-6.0, 0.0, -5.0])
    pt = np.array([6.0, 0.0, -5.0])
    v = 0.3
    dt = 1.0 / 30
    last_ring_stamp = None
    for k in range(int(14 / dt)):
        pa = pa + np.array([v * dt, 0, 0])
        pt = pt - np.array([v * dt, 0, 0])
        env.agents["A"].teleport(list(pa), [0, 0, 0])
        env.agents["T"].teleport(list(pt), [0, 0, 180])
        env.act("A", np.zeros(8))
        env.act("T", np.zeros(8))
        raw = env.tick()
        a = raw.get("A", {})
        tpose = raw.get("T", {}).get("PoseSensor")
        if all(n in a for n in RINGS) and "PoseSensor" in a and tpose is not None:
            PA = np.asarray(a["PoseSensor"])[:3, 3]
            PT = np.asarray(tpose)[:3, 3]
            res = proc.process(0.0, {n: a[n] for n in RINGS}, PA, rot(0))
            true_rel = PT - PA
            if res.detections:
                d = min(res.detections, key=lambda x: np.linalg.norm(x.rel - true_rel))
                results["dynamic"].append({"true_dist": round(float(true_rel[0]), 3),
                                           "err_x": round(float(d.rel[0] - true_rel[0]), 3)})
    dyn = np.array([[r["true_dist"], r["err_x"]] for r in results["dynamic"]])
    sel = dyn[(dyn[:, 0] > 1.5) & (dyn[:, 0] < 9)]
    print("dynamic head-on (closing 0.6 m/s): mean err_x %.3f, std %.3f, n=%d" % (sel[:, 1].mean(), sel[:, 1].std(), len(sel)))
    results["dynamic_summary"] = {"mean_err_x": float(sel[:, 1].mean()), "std": float(sel[:, 1].std()), "n": int(len(sel))}
    (Path(__file__).with_name("out") / "perception_bias.json").write_text(json.dumps(results, indent=1))
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
