"""Phase-0 probe: which HoloOcean sensors see other agents and spawned gate props?

Spawns three BlueROV2 at known relative poses plus one gate of the Horseshoe Bay
arena (re-used through marine_race_arena's own loader/factory/spawner), lets them
sit for a few ticks and prints what each candidate sensor returns.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ARENA_ROOT = Path.home() / "Desktop" / "HoloDroneCompetition"
sys.path.insert(0, str(ARENA_ROOT))

import holoocean  # noqa: E402
from marine_race_arena.config.loader import load_track_config  # noqa: E402
from marine_race_arena.arena.gate_factory import GateFactory  # noqa: E402
from marine_race_arena.adapters.visual_spawner import HoloOceanVisualSpawner  # noqa: E402

OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

RING_ELEV = [-30, -15, 0, 15, 30]
N_BEAMS = 72


def sensors(with_sonar: bool):
    s = [
        {"sensor_type": "DepthSensor", "socket": "DepthSocket", "Hz": 30},
        {"sensor_type": "IMUSensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "DVLSensor", "socket": "DVLSocket", "Hz": 10,
         "configuration": {"Elevation": 22.5, "ReturnRange": True, "MaxRange": 50}},
        {"sensor_type": "MagnetometerSensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
        {"sensor_type": "CollisionSensor", "Hz": 30},
        {"sensor_type": "RangeFinderSensor", "sensor_name": "RangeUp", "socket": "IMUSocket", "Hz": 10,
         "configuration": {"LaserMaxDistance": 20, "LaserCount": 1, "LaserAngle": 90}},
        {"sensor_type": "RangeFinderSensor", "sensor_name": "RangeDown", "socket": "IMUSocket", "Hz": 10,
         "configuration": {"LaserMaxDistance": 40, "LaserCount": 1, "LaserAngle": -90}},
        {"sensor_type": "RGBCamera", "sensor_name": "FrontCamera", "socket": "CameraSocket", "Hz": 10,
         "configuration": {"CaptureWidth": 320, "CaptureHeight": 240, "FovAngle": 90}},
    ]
    for e in RING_ELEV:
        s.append({"sensor_type": "RangeFinderSensor", "sensor_name": f"Ring{e:+d}".replace("+", "p").replace("-", "m"),
                  "socket": "IMUSocket", "Hz": 10,
                  "configuration": {"LaserMaxDistance": 12, "LaserCount": N_BEAMS, "LaserAngle": e,
                                    "LaserDebug": False}})
    if with_sonar:
        s.append({"sensor_type": "ImagingSonar", "sensor_name": "FrontSonar", "socket": "SonarSocket", "Hz": 5,
                  "configuration": {"Azimuth": 90, "Elevation": 20, "RangeMin": 0.5, "RangeMax": 12,
                                    "RangeBins": 128, "AzimuthBins": 128, "InitOctreeRange": 30,
                                    "ShowWarning": False}})
    return s


def main():
    with_sonar = "--sonar" in sys.argv
    poses = {
        "A": ([0.0, 0.0, -4.0], [0, 0, 0]),
        "B": ([3.0, 0.0, -4.0], [0, 0, 0]),      # 3 m ahead of A
        "C": ([0.0, -2.5, -4.5], [0, 0, 0]),     # 2.5 m to -y of A, 0.5 m lower
    }
    scenario = {
        "name": "probe", "world": "OpenWater", "package_name": "Ocean", "main_agent": "A",
        "ticks_per_sec": 30, "frames_per_sec": False, "window_width": 800, "window_height": 600,
        "agents": [{"agent_name": n, "agent_type": "BlueROV2", "location": p, "rotation": r,
                    "control_scheme": 0, "sensors": sensors(with_sonar)} for n, (p, r) in poses.items()],
    }
    t0 = time.time()
    env = holoocean.make(scenario_cfg=scenario, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    print(f"make() took {time.time() - t0:.1f}s")
    env.reset()

    cfg = load_track_config(str(ARENA_ROOT / "marine_race_arena/tracks/marine_race_horseshoe_bay.json"),
                            benchmark_task="clean_gate", current_profile="none", seed=0)
    factory = GateFactory(cfg)
    gates = factory.build_gates()
    # Place a copy of gate G01 6 m in front of A (axis along +x) to see if the ring sees props.
    import dataclasses
    g = gates[0]
    print("Gate fields:", [f.name for f in dataclasses.fields(g)] if dataclasses.is_dataclass(g) else dir(g))
    spawner = HoloOceanVisualSpawner(env)
    vis = factory.build_visual_gate(g)
    shifted = []
    dx = 6.0 - g.center[0]; dy = 0.0 - g.center[1]; dz = -4.0 - g.center[2]
    for bar in vis.bars:
        shifted.append(dataclasses.replace(bar, position=(bar.position[0] + dx, bar.position[1] + dy, bar.position[2] + dz),
                                           rotation_rpy_deg=(0.0, 0.0, 0.0)))
    spawner.spawn_gate_bars(shifted)
    print("spawn report:", spawner.report)

    zero = np.zeros(8)
    t0 = time.time()
    state = {n: {} for n in poses}
    for k in range(60):
        for n in poses:
            env.act(n, zero)
        raw = env.tick()
        for n in poses:
            state[n].update(raw.get(n, {}))
    print(f"60 ticks took {time.time() - t0:.2f}s")
    report = {}
    for n in poses:
        st = state[n]
        rep = {"keys": sorted(st.keys())}
        pose = np.asarray(st["PoseSensor"]); rep["true_pos"] = pose[:3, 3].round(3).tolist()
        rep["mag"] = np.asarray(st["MagnetometerSensor"]).round(3).tolist()
        rep["dvl"] = np.asarray(st["DVLSensor"]).round(3).tolist()
        rep["depth"] = np.asarray(st["DepthSensor"]).round(3).tolist()
        rep["up"] = float(np.asarray(st["RangeUp"]).ravel()[0]); rep["down"] = float(np.asarray(st["RangeDown"]).ravel()[0])
        for e in RING_ELEV:
            key = f"Ring{e:+d}".replace("+", "p").replace("-", "m")
            r = np.asarray(st[key]).ravel()
            hits = [(i * 360.0 / N_BEAMS, round(float(v), 2)) for i, v in enumerate(r) if 0 < v < 12]
            rep[key] = hits
        if "FrontSonar" in st:
            img = np.asarray(st["FrontSonar"])
            rep["sonar_shape"] = list(img.shape); rep["sonar_max"] = float(img.max()); rep["sonar_nonzero"] = int((img > 0.05).sum())
            np.save(OUT / f"sonar_{n}.npy", img)
        cam = np.asarray(st["FrontCamera"])
        rep["cam_shape"] = list(cam.shape)
        try:
            import cv2
            cv2.imwrite(str(OUT / f"cam_{n}.png"), cam[:, :, :3])
        except Exception as exc:
            rep["cam_err"] = str(exc)
        report[n] = rep
    print(json.dumps(report, indent=1))
    (OUT / f"probe_report{'_sonar' if with_sonar else ''}.json").write_text(json.dumps(report, indent=1))
    env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
