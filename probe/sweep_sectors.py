"""Diagnostic: which sonar sees a BlueROV2 placed around the observer (azimuth and elevation sweeps)."""
import math, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.sim.holo_env import HoloFleetSim
from holo_fleet.sim.spec import bench_spec
R = DEFAULT.perc.sonar.ranges(); THR = DEFAULT.perc.sonar.threshold
O = np.array([0.0, -20.0, -5.0])
sim = HoloFleetSim(bench_spec("sweep", [O, O + np.array([0, 40.0, 0])])); sim.start()
obs, tgt = sim.names; sim.pin(obs, O, (0, 0, 0))
def look(u, rho=2.0):
    sim.pin(tgt, O + rho * np.asarray(u, float), (0, 0, 0)); sim.step({n: None for n in sim.names}, 0.4)
    out = {}
    for s in sg.SECTORS:
        p = np.asarray(sim.latest[obs][sg.sonar_sensor_name(s)], float); idx = np.flatnonzero((p > THR) & (R < rho + 0.6))
        if idx.size: out[s[:2]] = round(float(R[idx[0]]), 2)
    return out
print("horizontal sweep, 2 m (azimuth from bow, CCW = left):")
for az in range(0, 360, 15):
    a = math.radians(az); print(f"  az {az:3d}: {look((math.cos(a), math.sin(a), 0.0))}")
print("vertical sweep at azimuth 90 (left), 2 m:")
for el in range(-90, 91, 15):
    e = math.radians(el); print(f"  el {el:+3d}: {look((0.0, math.cos(e), math.sin(e)))}")
print("near pure-left with small x offsets:")
for dx in (0.0, 0.02, 0.05, 0.1, 0.3):
    print(f"  dx {dx}: {look((dx / 2.0, 1.0, 0.0))}")
sim.close()
