"""Visualisation-only HoloOcean capture: the fleet queued at gate G06 of the marine arena, with the
simulator's own debug drawing of the sensors switched on (imaging-sonar view region, proximity-sonar
rays).  Writes figures/scene_sensor_debug.png.  Not an experiment: no controller, no referee.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates, make_spawner, visual_gates  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception.sensor_suite import onboard_sensor_configs, ring_name  # noqa: E402


def main() -> int:
    import holoocean
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    G = DEFAULT.gate
    gates = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}
    g6 = gates["G06"]
    slots = {"drone_0": (G.s_queue, G.queue_lateral), "drone_1": (G.s_queue, -G.queue_lateral),
             "drone_2": (G.s_queue - G.queue_center_back, 0.0)}
    yaw = float(np.degrees(np.arctan2(g6.axis[1], g6.axis[0])))
    agents = []
    for name, (s, l) in slots.items():
        sensors = onboard_sensor_configs(DEFAULT, with_fls=True, with_camera=True)
        for sc in sensors:
            if name == "drone_0" and sc.get("sensor_name") == "FrontSonar":
                sc["configuration"]["ViewRegion"] = True
            if name == "drone_2" and sc.get("sensor_name") in (ring_name(0), ring_name(-15)):
                sc["configuration"]["LaserDebug"] = True
        if name == "drone_2":
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "ViewBehind", "socket": "IMUSocket",
                            "location": [-4.0, 0.0, 1.4], "rotation": [0.0, 12.0, 0.0], "Hz": 30,
                            "configuration": {"CaptureWidth": 960, "CaptureHeight": 540, "FovAngle": 80}})
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "ViewSide", "socket": "IMUSocket",
                            "location": [2.0, 7.5, 1.5], "rotation": [0.0, 10.0, -90.0], "Hz": 30,
                            "configuration": {"CaptureWidth": 960, "CaptureHeight": 540, "FovAngle": 80}})
        p = g6.from_gate_frame([s, l, 0.0])
        agents.append({"agent_name": name, "agent_type": "BlueROV2", "location": [float(v) for v in p],
                       "rotation": [0.0, 0.0, yaw], "control_scheme": 0, "sensors": sensors})
    scen = {"name": "sensor_views", "world": "OpenWater", "package_name": "Ocean", "main_agent": "drone_2",
            "ticks_per_sec": 30, "frames_per_sec": False, "agents": agents}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    make_spawner(env).spawn_gate_bars([bar for vg in visual_gates(HORSESHOE_TRACK) for bar in vg.bars])
    latest = {}
    for _ in range(45):
        for name in slots:
            env.act(name, np.zeros(8))
        raw = env.tick()
        latest.update(raw.get("drone_2", {}))
    env.__exit__(None, None, None)
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.4))
    for ax, key, title in ((axes[0], "ViewBehind", "Fleet queued at gate G06 (view from behind drone_2)"),
                           (axes[1], "ViewSide", "Side view: queue points and the gate (debug rays / sonar region on)")):
        img = np.asarray(latest[key])[:, :, :3][:, :, ::-1]
        ax.imshow(img)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(title, fontsize=9)
    fig.suptitle("HoloOcean scene with sensor debug drawing (visualisation only, not an experiment)", fontsize=10)
    fig.tight_layout()
    out = ROOT / "figures" / "scene_sensor_debug.png"
    fig.savefig(out, dpi=130)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
