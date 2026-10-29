"""Map commanded set_ocean_currents magnitude -> drift velocity of an unactuated BlueROV2."""
import json
from pathlib import Path

import numpy as np
import holoocean

MAGS = [0.3, 1.0, 2.0, 4.0]


def main():
    agents = []
    for i, m in enumerate(MAGS):
        agents.append({"agent_name": f"A{i}", "agent_type": "BlueROV2", "location": [0, 10.0 * i, -5], "rotation": [0, 0, 0],
                       "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
                                                         {"sensor_type": "VelocitySensor", "socket": "IMUSocket", "Hz": 30}]})
    scenario = {"name": "cur", "world": "OpenWater", "package_name": "Ocean", "main_agent": "A0", "ticks_per_sec": 30,
                "frames_per_sec": False, "agents": agents}
    env = holoocean.make(scenario_cfg=scenario, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    res = {m: [] for m in MAGS}
    for k in range(30 * 20):
        for i, m in enumerate(MAGS):
            env.set_ocean_currents(f"A{i}", [m, 0.0, 0.0])
            env.act(f"A{i}", np.zeros(8))
        st = env.tick()
        if k % 30 == 29:
            for i, m in enumerate(MAGS):
                v = np.asarray(st[f"A{i}"]["VelocitySensor"]).ravel()
                res[m].append(round(float(v[0]), 3))
    env.__exit__(None, None, None)
    for m in MAGS:
        print(f"current {m:4.1f} m/s -> vx per second: {res[m]}")
    (Path(__file__).with_name("out") / "current_map.json").write_text(json.dumps({str(k): v for k, v in res.items()}))


if __name__ == "__main__":
    main()
