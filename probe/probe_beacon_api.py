"""Probe: what HoloOcean 2.3's AcousticBeaconSensor and OpticalModemSensor really deliver.

    python probe/probe_beacon_api.py

Pinned BlueROV2s on a line at 0, 5, 20 and 45 m (z = -5), plus one hidden behind a box prop.  Each carries
an AcousticBeaconSensor (MaxDistance 40 m, CheckVisible on) and an OpticalModemSensor (MaxDistance 8 m,
LaserAngle 60 deg).  A scripted sequence sends one-way acoustic messages (OWAY, OWAYU), two simultaneous
transmissions, and optical messages, and records at every tick what each receiver returns (client data and the
raw engine buffer).  Writes results/robustness/probe/beacon_api_probe.json.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.sim import holoocean_setup as hs  # noqa: E402

OUT = ROOT / "results" / "robustness" / "probe"
POS = {"a0": [0.0, 0.0, -5.0], "a5": [5.0, 0.0, -5.0], "a20": [20.0, 0.0, -5.0], "a45": [45.0, 0.0, -5.0],
       "hid": [0.0, 12.0, -5.0], "side": [3.0, 3.0, -5.0], "behind": [-4.0, 0.0, -5.0]}
BOX = {"type": "box", "location": [0.0, 6.0, -5.0], "scale": [3.0, 0.5, 3.0]}


def agent(name, k):
    return {"agent_name": name, "agent_type": "BlueROV2", "location": POS[name], "rotation": [0.0, 0.0, 0.0],
            "control_scheme": 0, "sensors": [
                {"sensor_type": "AcousticBeaconSensor", "sensor_name": "Beacon", "socket": "", "location": [0, 0, 0.1],
                 "configuration": {"id": k, "MaxDistance": 40, "CheckVisible": True}},
                {"sensor_type": "OpticalModemSensor", "sensor_name": "Optical", "socket": "", "location": [0.25, 0, 0],
                 "configuration": {"id": k, "MaxDistance": 8, "LaserAngle": 60}},
                {"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30}]}


def main() -> int:
    hs.use_patched_holoocean()
    import holoocean
    from holoocean.sensors import AcousticBeaconSensor, OpticalModemSensor

    hs.wait_for_engine_semaphores()
    names = list(POS)
    scen = {"name": "probe_beacon", "world": "OpenWater", "package_name": "Ocean", "main_agent": names[0],
            "ticks_per_sec": 30, "frames_per_sec": False, "octree_min": hs.OCTREE_MIN, "octree_max": hs.OCTREE_MAX,
            "env_min": [-30.0, -30.0, -20.0], "env_max": [60.0, 30.0, 2.0],
            "agents": [agent(n, k) for k, n in enumerate(names)]}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    env.spawn_prop(BOX["type"], location=BOX["location"], rotation=[0, 0, 0], scale=BOX["scale"], sim_physics=False,
                   material="steel")
    ids = {n: k for k, n in enumerate(names)}
    log = []
    # (tick, kind, args)
    plan = [(40, "ac", ("a0", "all", "OWAY")), (80, "ac", ("a0", "all", "OWAYU")),
            (120, "ac2", (("a0", "OWAY"), ("a5", "OWAY"))),            # simultaneous transmissions
            (160, "ac", ("a5", "all", "OWAY")), (200, "ac", ("a0", "a5", "OWAY")),
            (240, "op", ("a0", "all")), (280, "op", ("side", "all")), (320, "ac", ("hid", "all", "OWAY")),
            (360, "ac", ("a0", "all", "OWAY")), (361, "ac", ("a0", "all", "OWAY"))]   # re-send while transmitting
    t0 = time.time()
    try:
        for tick in range(420):
            for n in names:
                env.agents[n].set_physics_state(POS[n], [0.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0])
                env.act(n, np.zeros(8))
            for (tk, kind, a) in plan:
                if tk != tick:
                    continue
                if kind == "ac":
                    to = -1 if a[1] == "all" else ids[a[1]]
                    env.send_acoustic_message(ids[a[0]], to, a[2], None)
                elif kind == "ac2":
                    for frm, typ in a:
                        env.send_acoustic_message(ids[frm], -1, typ, None)
                elif kind == "op":
                    env.send_optical_message(ids[a[0]], -1, {"id": ids[a[0]]})
                log.append({"tick": tick, "sent": [kind, list(map(str, a))]})
            state = env.tick()
            for n in names:
                d = state.get(n, {})
                for key in ("Beacon", "Optical"):
                    if key in d and d[key] is not None:
                        v = d[key]
                        log.append({"tick": tick, "rx": n, "sensor": key,
                                    "data": [x if isinstance(x, (str, int, type(None), dict)) else float(x) for x in v]
                                    if isinstance(v, (list, tuple)) else (v if isinstance(v, dict) else str(v))})
            raw = {n: env.agents[n].sensors["Beacon"]._sensor_data_buffer.tolist() for n in names}
            if any(not np.all(np.isnan(np.asarray(v, float))) for v in raw.values()):
                log.append({"tick": tick, "raw_beacon_buffers": raw})
            status = {n: AcousticBeaconSensor.instances[ids[n]].status for n in names}
            if any(s != "Idle" for s in status.values()):
                log.append({"tick": tick, "transmitting": [n for n, s in status.items() if s != "Idle"],
                            "sending_to": {n: list(AcousticBeaconSensor.instances[ids[n]].sending_to) for n in names
                                           if AcousticBeaconSensor.instances[ids[n]].sending_to}})
    finally:
        env.__exit__(None, None, None)
    out = {"positions": POS, "box": BOX, "wall_s": round(time.time() - t0, 1), "log": log}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "beacon_api_probe.json").write_text(json.dumps(out, indent=1, default=str), encoding="utf-8")
    for e in log:
        if "raw_beacon_buffers" in e:
            e = {"tick": e["tick"], "raw": {k: v for k, v in e["raw_beacon_buffers"].items()
                                            if not np.all(np.isnan(np.asarray(v, float)))}}
        print(json.dumps(e, default=str)[:300])
    return 0


if __name__ == "__main__":
    sys.exit(main())
