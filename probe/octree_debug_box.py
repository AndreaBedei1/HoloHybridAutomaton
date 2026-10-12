"""Diagnostic: one runtime box in front of a sonar, patched vs official engine.

  python probe/octree_debug_box.py patched_rebuild   # sonar first, box after, env.rebuild_sonar_octree()
  python probe/octree_debug_box.py patched_late      # patched engine, box first, sonar added afterwards
  python probe/octree_debug_box.py official_late     # official engine, box first, sonar added afterwards

Prints the echo bins seen from several observer positions around the box.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
MODE = sys.argv[1]
if MODE.startswith("patched"):
    from holo_fleet.sim.holoocean_setup import use_patched_holoocean

    use_patched_holoocean()
else:
    os.environ.pop("HOLODECKPATH", None)

from holo_fleet.sim.holoocean_setup import ENV_MAX, ENV_MIN, OCTREE_MAX  # noqa: E402

TPS, HZ, SONAR_X = 30, 10, 0.24
R_MIN, R_MAX, BINS = 0.3, 12.0, 234
RANGES = R_MIN + (np.arange(BINS) + 0.5) * (R_MAX - R_MIN) / BINS
BASE = np.array([0.0, -14.0, -5.0])
BOX = BASE + np.array([SONAR_X + 3.0 + 0.25, 0.0, 0.0])
OCT_MIN = 0.06 if MODE.startswith("patched") else 0.07          # official: an unused private size (min7)


def main():
    import holoocean
    from holoocean.sensors import SensorDefinition

    cfg = {"OpeningAngle": 120, "RangeMin": R_MIN, "RangeMax": R_MAX, "RangeBins": BINS, "ShowWarning": False,
           "InitOctreeRange": 20}
    sensors = []
    if MODE == "patched_rebuild":
        sensors = [{"sensor_type": "SinglebeamSonar", "sensor_name": "Sonar", "location": [SONAR_X, 0, 0], "Hz": HZ,
                    "configuration": dict(cfg)}]
    scen = {"name": "dbg", "world": "OpenWater", "package_name": "Ocean", "main_agent": "obs", "ticks_per_sec": TPS,
            "frames_per_sec": False, "octree_min": OCT_MIN, "octree_max": OCTREE_MAX, "env_min": ENV_MIN,
            "env_max": ENV_MAX,
            "agents": [{"agent_name": "obs", "agent_type": "BlueROV2", "location": list(BASE), "rotation": [0, 0, 0],
                        "control_scheme": 0, "sensors": sensors}]}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False)
    pose = [list(BASE), [0, 0, 0]]
    last = {}

    def tick(n=1):
        for _ in range(n):
            env.agents["obs"].set_physics_state(pose[0], pose[1], [0, 0, 0], [0, 0, 0])
            env.act("obs", np.zeros(8))
            raw = env.tick()
            if "Sonar" in raw:
                last["p"] = np.asarray(raw["Sonar"], dtype=float)

    tick(20)
    if MODE == "patched_rebuild":
        tick(env.rebuild_sonar_octree())          # clean start without the box
    env.spawn_prop("box", location=[float(v) for v in BOX], rotation=[0, 0, 0], scale=0.5, sim_physics=False,
                   material="steel")
    tick(10)
    if MODE == "patched_rebuild":
        tick(env.rebuild_sonar_octree())
    else:
        env._has_sonar = True
        env.agents["obs"].add_sensors(SensorDefinition("obs", "BlueROV2", "Sonar", "SinglebeamSonar",
                                                       location=[SONAR_X, 0, 0], config=dict(cfg), tick_every=3))
        tick(30)
    for label, loc, yaw, expect in [
        ("front 3 m", BASE, 0, 3.0),
        ("front 3 m again", BASE, 0, 3.0),
        ("front 5 m", BASE - np.array([2.0, 0, 0]), 0, 5.0),
        ("side view, 4 m", BOX + np.array([0.0, -4.25 - SONAR_X, 0.0]), 90, 4.0),
        ("front 3 m third time", BASE, 0, 3.0),
    ]:
        pose[0], pose[1] = [float(v) for v in loc], [0, 0, float(yaw)]
        tick(12)
        p = last["p"]
        idx = np.flatnonzero(p > 1e-3)
        print(f"{MODE:16s} {label:22s} expect {expect:4.1f}  echoes: {np.round(RANGES[idx][:8], 3).tolist()} n={idx.size}",
              flush=True)
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
