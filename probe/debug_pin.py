import sys, math
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.sim.holo_env import HoloFleetSim, agent_origin_from_pose
from holo_fleet.sim.spec import bench_spec
R = DEFAULT.perc.sonar.ranges(); THR = DEFAULT.perc.sonar.threshold
O = np.array([0.0, -20.0, -5.0])
sim = HoloFleetSim(bench_spec("dbg", [O, O + np.array([0, 40.0, 0])])); sim.start()
obs, tgt = sim.names; sim.pin(obs, O, (0, 0, 0))
seq = [((-1, 1, 1), 5.0, 45.0), ((0, -1, -1), 1.0, 0.0), ((0, -1, 0), 2.0, 0.0), ((0, 1, 0), 2.0, 0.0), ((1, 0, 0), 2.0, 0.0), ((0, -1, 0), 2.0, 0.0), ((0, -1, 0), 2.0, 1.0)]
for u, rho, yaw in seq:
    u = np.asarray(u, float); u /= np.linalg.norm(u); p = O + rho * u
    sim.pin(tgt, p, (0, 0, yaw)); sim.step({n: None for n in sim.names}, 0.4)
    truth = agent_origin_from_pose(sim.latest[tgt]["PoseSensor"]); otruth = agent_origin_from_pose(sim.latest[obs]["PoseSensor"])
    seen = {}
    for s in sg.SECTORS:
        pr = np.asarray(sim.latest[obs][sg.sonar_sensor_name(s)], float); idx = np.flatnonzero(pr > THR)
        if idx.size: seen[s[:2]] = round(float(R[idx[0]]), 2)
    print(f"cmd {np.round(p,2)} yaw {yaw}: target truth {np.round(truth,2)} observer truth {np.round(otruth,2)} echoes {seen}")
sim.close()
