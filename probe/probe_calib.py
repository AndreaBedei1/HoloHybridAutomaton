"""Calibrate RangeFinder ring conventions (azimuth direction, elevation sign).

Observer A sits at the origin; target T is teleported to known relative positions.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import holoocean

RING_ELEV = [-30, -15, 0, 15, 30]
N_BEAMS = 72


def ring_sensors():
    out = [{"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
           {"sensor_type": "MagnetometerSensor", "socket": "IMUSocket", "Hz": 30}]
    for e in RING_ELEV:
        out.append({"sensor_type": "RangeFinderSensor", "sensor_name": f"R{e}", "socket": "IMUSocket", "Hz": 30,
                    "configuration": {"LaserMaxDistance": 12, "LaserCount": N_BEAMS, "LaserAngle": e}})
    return out


def main():
    yaw_a = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    scenario = {
        "name": "calib", "world": "OpenWater", "package_name": "Ocean", "main_agent": "A",
        "ticks_per_sec": 30, "frames_per_sec": False,
        "agents": [
            {"agent_name": "A", "agent_type": "BlueROV2", "location": [0, 0, -5], "rotation": [0, 0, yaw_a],
             "control_scheme": 0, "sensors": ring_sensors()},
            {"agent_name": "T", "agent_type": "BlueROV2", "location": [20, 20, -5], "rotation": [0, 0, 0],
             "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30}]},
        ],
    }
    env = holoocean.make(scenario_cfg=scenario, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    rel = [(3, 0, 0), (0, 3, 0), (0, -3, 0), (-3, 0, 0), (3, 0, 1.0), (3, 0, -1.0), (2.5, 2.5, 0.0), (5, 0, 0), (8, 0, 0)]
    results = []
    for r in rel:
        state = {}
        for _ in range(4):
            env.agents["A"].teleport([0, 0, -5], [0, 0, yaw_a])
            env.agents["T"].teleport([r[0], r[1], -5 + r[2]], [0, 0, 0])
            env.act("A", np.zeros(8)); env.act("T", np.zeros(8))
            raw = env.tick()
            for k, v in raw.get("A", {}).items():
                state[k] = v
        hits = {}
        for e in RING_ELEV:
            arr = np.asarray(state[f"R{e}"]).ravel()
            hits[e] = [(round(i * 360.0 / N_BEAMS, 1), round(float(v), 2)) for i, v in enumerate(arr) if 0 < v < 12]
        pose = np.asarray(state["PoseSensor"])
        results.append({"rel": r, "A_pos": pose[:3, 3].round(2).tolist(), "mag": np.asarray(state["MagnetometerSensor"]).round(3).tolist(), "hits": hits})
        print(r, json.dumps(hits))
    Path(__file__).with_name("out").mkdir(exist_ok=True)
    (Path(__file__).with_name("out") / f"calib_yaw{int(yaw_a)}.json").write_text(json.dumps(results, indent=1))
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
