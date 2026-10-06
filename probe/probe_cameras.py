"""Check visualisation-camera offsets/rotations (pitch sign, location units)."""
from pathlib import Path

import cv2
import numpy as np
import holoocean

OUT = Path(__file__).with_name("out")
OUT.mkdir(exist_ok=True)


def main():
    cams = [
        {"sensor_type": "RGBCamera", "sensor_name": "ChaseLow", "socket": "IMUSocket", "location": [-6.0, 0.0, 1.2],
         "rotation": [0.0, 10.0, 0.0], "Hz": 30, "configuration": {"CaptureWidth": 480, "CaptureHeight": 300, "FovAngle": 75}},
        {"sensor_type": "RGBCamera", "sensor_name": "SideYawPos", "socket": "IMUSocket", "location": [1.0, 8.0, 0.8],
         "rotation": [0.0, 5.0, 90.0], "Hz": 30, "configuration": {"CaptureWidth": 480, "CaptureHeight": 300, "FovAngle": 75}},
        {"sensor_type": "RGBCamera", "sensor_name": "SideYawNeg", "socket": "IMUSocket", "location": [1.0, 8.0, 0.8],
         "rotation": [0.0, 5.0, -90.0], "Hz": 30, "configuration": {"CaptureWidth": 480, "CaptureHeight": 300, "FovAngle": 75}},
    ]
    agents = [{"agent_name": "C", "agent_type": "BlueROV2", "location": [0, 0, -4.5], "rotation": [0, 0, 0],
               "control_scheme": 0, "sensors": cams}]
    for i, (x, y) in enumerate(((2.5, 1.75), (2.5, -1.75))):
        agents.append({"agent_name": f"N{i}", "agent_type": "BlueROV2", "location": [x, y, -4.5], "rotation": [0, 0, 0],
                       "control_scheme": 0, "sensors": [{"sensor_type": "DepthSensor", "socket": "DepthSocket", "Hz": 30}]})
    scen = {"name": "cam", "world": "OpenWater", "package_name": "Ocean", "main_agent": "C", "ticks_per_sec": 30,
            "frames_per_sec": False, "agents": agents}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    st = {}
    for _ in range(10):
        for a in ("C", "N0", "N1"):
            env.act(a, np.zeros(8))
        raw = env.tick()
        st.update(raw.get("C", {}))
    for k in ("ChaseLow", "SideYawPos", "SideYawNeg"):
        cv2.imwrite(str(OUT / f"cam_{k}.png"), np.asarray(st[k])[:, :, :3])
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
