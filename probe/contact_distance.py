"""Largest centre distance at which two BlueROV2 collision shapes touch, per direction (CollisionSensor / displacement)."""
import sys, json
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from holo_fleet.sim.holo_env import HoloFleetSim, agent_origin_from_pose
from holo_fleet.sim.spec import bench_spec
O = np.array([0.0, -20.0, -5.0]); FAR = O + np.array([0.0, 30.0, 0.0])
sim = HoloFleetSim(bench_spec("contact", [O, FAR])); sim.start()
obs, tgt = sim.names; sim.pin(obs, O, (0, 0, 0))
dirs = {f"{x:+d}{y:+d}{z:+d}": np.array([x, y, z], float) / np.linalg.norm([x, y, z])
        for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)}
res = {}
for label, u in dirs.items():
    contact = None
    for rho in np.arange(1.40, 0.45, -0.05):
        sim.pin(tgt, FAR, (0, 0, 0)); sim.step({n: None for n in sim.names}, 0.2)        # leave, then place
        p = O + rho * u
        sim.pin(tgt, p, (0, 0, 0)); sim.step({n: None for n in sim.names}, 0.2)
        got = agent_origin_from_pose(sim.latest[tgt]["PoseSensor"])
        col = bool(np.asarray(sim.latest[tgt].get("CollisionSensor", False)).any()) or bool(np.asarray(sim.latest[obs].get("CollisionSensor", False)).any())
        if col or np.linalg.norm(got - p) > 0.05:
            contact = round(float(rho), 2); break
    res[label] = contact
    print(label, "contact at centre distance", contact, flush=True)
sim.close()
out = ROOT / "results" / "v2" / "sonar_bench" / "contact_distance.json"; out.write_text(json.dumps(res, indent=1))
print("max contact distance:", max(v for v in res.values() if v is not None))
