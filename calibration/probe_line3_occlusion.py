import json
import os
from pathlib import Path
import numpy as np
from calibration.sonar_bench import O, first_echo, wait_captures, BENCH_CFG
from holo_fleet.formations import TEMPLATES
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.sim import engine_lease
from holo_fleet.sim.holo_env import HoloFleetSim, agent_origin_from_pose
from holo_fleet.sim.spec import bench_spec
from scripts.run_sensor_robustness_campaign import be_nice

be_nice()
if "HHA_P1_SHARED_LEASE_DIR" in os.environ:
    engine_lease.SLOTS = Path(os.environ["HHA_P1_SHARED_LEASE_DIR"])
positions = O + np.array(sorted(TEMPLATES["line3"].offsets().tolist(), key=lambda p: p[1]))
sim = HoloFleetSim(bench_spec("p1_line3_shadow", positions), BENCH_CFG)
try:
    sim.start()
    a, b, c = sim.names
    print("Isolated engine:", sim.env._holo_engine["listen_ports"], flush=True)
    assert 1985 not in sim.env._holo_engine["listen_ports"]
    for name, pos in zip(sim.names, positions):
        sim.pin(name, pos)
    rows = {}
    for label, middle, far in (
        ("line3", positions[1], positions[2]),
        ("closer_behind_middle", positions[1], positions[2] - [0, 0.5, 0]),
        ("middle_out_of_range", positions[1] + [20, 0, 0], positions[2]),
    ):
        sim.pin(b, middle)
        sim.pin(c, far)
        wait_captures(sim, 3)
        for name, desired in zip(sim.names, (positions[0], middle, far)):
            actual = agent_origin_from_pose(sim.latest[name]["PoseSensor"])
            assert np.linalg.norm(actual - desired) < 0.05, (label, name, desired, actual)
        captures = []
        for _ in range(20):
            wait_captures(sim, 0)
            hits = {}
            for sector in sg.SECTORS:
                prof = sim.latest[a][sg.sonar_sensor_name(sector)]
                hits[sector] = {"near": first_echo(prof, 2.5, 4.1), "far": first_echo(prof, 5.6, 7.7)}
            captures.append(hits)
        rows[label] = {"observer": positions[0].tolist(), "middle": middle.tolist(), "far": far.tolist(),
                       "captures": captures}
        counts = {f"{sector}_{band}": sum(row[sector][band] is not None for row in captures)
                  for sector in sg.SECTORS for band in ("near", "far")}
        print(label, {key: value for key, value in counts.items() if value}, flush=True)
    out = Path(os.environ["HHA_P1_OBS"])
    evidence = {"profile": "CLEAN", "listeners": sim.env._holo_engine["listen_ports"], "cases": rows}
    (out / "observations.json").write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
finally:
    sim.close()
