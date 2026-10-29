"""Calibrate BlueROV2 actuation (arena thruster mapping) and DVL/IMU frames."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import holoocean

ARENA_ROOT = Path.home() / "Desktop" / "HoloDroneCompetition"
sys.path.insert(0, str(ARENA_ROOT))


def thrusters(surge, sway, heave, yaw, limit=12.0):
    # identical to marine_race_arena BaseRaceAdapter.command_to_bluerov2_thrusters
    c = lambda v: max(-1.0, min(1.0, v))
    surge, sway, heave, yaw = c(surge), c(sway), c(heave), c(yaw)
    vertical = [heave] * 4
    horizontal = [surge + sway + 0.35 * yaw, surge - sway - 0.35 * yaw, surge + sway - 0.35 * yaw, surge - sway + 0.35 * yaw]
    return np.array([max(-limit, min(limit, v * limit)) for v in vertical + horizontal])


def main():
    scenario = {
        "name": "motion", "world": "OpenWater", "package_name": "Ocean", "main_agent": "A",
        "ticks_per_sec": 30, "frames_per_sec": False,
        "agents": [{"agent_name": "A", "agent_type": "BlueROV2", "location": [0, 0, -5], "rotation": [0, 0, 30],
                    "control_scheme": 0, "sensors": [
                        {"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
                        {"sensor_type": "VelocitySensor", "socket": "IMUSocket", "Hz": 30},
                        {"sensor_type": "DVLSensor", "socket": "DVLSocket", "Hz": 30, "configuration": {"ReturnRange": False}},
                        {"sensor_type": "IMUSensor", "socket": "IMUSocket", "Hz": 30},
                        {"sensor_type": "MagnetometerSensor", "socket": "IMUSocket", "Hz": 30},
                        {"sensor_type": "DepthSensor", "socket": "DepthSocket", "Hz": 30}]}],
    }
    env = holoocean.make(scenario_cfg=scenario, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.reset()
    phases = [("idle", (0, 0, 0, 0), 2), ("surge+0.3", (0.3, 0, 0, 0), 6), ("surge+1", (1, 0, 0, 0), 6), ("stop", (0, 0, 0, 0), 4),
              ("sway+0.5", (0, 0.5, 0, 0), 6), ("stop", (0, 0, 0, 0), 4), ("heave+0.3", (0, 0, 0.3, 0), 4),
              ("heave-0.3", (0, 0, -0.3, 0), 4), ("yaw+0.3", (0, 0, 0, 0.3), 4), ("stop", (0, 0, 0, 0), 3),
              ("current_y", (0, 0, 0, 0), 6)]
    log = []
    t = 0.0
    for name, cmd, dur in phases:
        for k in range(int(dur * 30)):
            if name == "current_y":
                env.set_ocean_currents("A", [0.0, 0.3, 0.0])
            else:
                env.set_ocean_currents("A", [0.0, 0.0, 0.0])
            env.act("A", thrusters(*cmd))
            raw = env.tick(); s = raw.get("A", raw)
            t += 1 / 30
            pose = np.asarray(s["PoseSensor"]); R = pose[:3, :3]
            vw = np.asarray(s["VelocitySensor"]).ravel()
            vb_true = R.T @ vw
            yaw_true = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            mag = np.asarray(s["MagnetometerSensor"]).ravel()
            log.append({"t": round(t, 3), "phase": name, "pos": pose[:3, 3].round(3).tolist(), "yaw_true": round(yaw_true, 2),
                        "yaw_mag": round(math.degrees(math.atan2(mag[1], mag[0])), 2),
                        "v_world": vw.round(3).tolist(), "v_body_true": vb_true.round(3).tolist(),
                        "dvl": np.asarray(s["DVLSensor"]).ravel()[:3].round(3).tolist(),
                        "imu": np.asarray(s["IMUSensor"]).round(3).tolist(),
                        "depth": float(np.asarray(s["DepthSensor"]).ravel()[0])})
    env.__exit__(None, None, None)
    Path(__file__).with_name("out").mkdir(exist_ok=True)
    (Path(__file__).with_name("out") / "motion.json").write_text(json.dumps(log))
    # summary at end of each phase
    last = {}
    for row in log:
        last[row["phase"] + "@" + str(int(row["t"] // 1))] = row
    prev = None
    for row in log:
        if prev is not None and row["phase"] != prev["phase"]:
            print(f"{prev['phase']:>10s} end t={prev['t']:.1f} pos={prev['pos']} yaw={prev['yaw_true']} mag_yaw={prev['yaw_mag']} vb_true={prev['v_body_true']} dvl={prev['dvl']} imu_gyro={prev['imu'][1] if len(prev['imu'])>1 else None}")
        prev = row
    print(f"{prev['phase']:>10s} end t={prev['t']:.1f} pos={prev['pos']} vw={prev['v_world']} vb_true={prev['v_body_true']} dvl={prev['dvl']}")


if __name__ == "__main__":
    main()
