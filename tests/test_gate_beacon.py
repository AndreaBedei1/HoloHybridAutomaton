"""Gate beacon (perception/gate_beacon.py, sim/gate_beacon.py): a fixed pinger above the gate gives a position fix
relative to the gate that corrects the classification pose (v39: a 0.72 m lateral drift made the bars an obstacle)."""

import math

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception.gate_beacon import GateBeaconFix, beacon_position, fix_bound
from holo_fleet.sim.gate_beacon import GateBeaconSim
from holo_fleet.sim.scenarios import SCENARIOS

PLAN = SCENARIOS["line_parallel_mutex"](DEFAULT).plans[0]
G = PLAN.gates[0]


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_the_fix_recovers_a_lateral_drift_within_its_bound():
    yaw = math.atan2(G.axis[1], G.axis[0])
    R = _R(yaw)
    truth = np.asarray(G.from_gate_frame([-1.2, 0.0, -0.8]), float)
    est = np.asarray(G.from_gate_frame([-1.2, 0.72, -0.8]), float)
    sim = GateBeaconSim(["d"], [G], DEFAULT, seed=3)
    fix = GateBeaconFix(DEFAULT)
    t = 0.0
    while t < 3.0:
        for gid, _tm, r, az, el in sim.measure(t, {"d": (truth, R)})["d"]:
            fix.add(t, G, r, az, el, est, R)
        t += 0.1
    off, bound = fix.offset(t - 0.1)
    assert np.linalg.norm(est + off - truth) <= bound
    assert bound <= DEFAULT.gate_beacon.max_bound_m


def test_too_few_or_disagreeing_fixes_give_nothing():
    fix = GateBeaconFix(DEFAULT)
    R = np.eye(3)
    p = np.asarray(G.from_gate_frame([-2.0, 0.0, 0.0]), float)
    fix.add(0.0, G, 2.5, 0.0, 0.5, p, R)
    assert fix.offset(0.0) is None
    for k in range(3):
        fix.add(0.1 * k, G, 2.5 + 3.0 * k, 0.0, 0.5, p, R)
    assert fix.offset(0.3) is None


def test_the_beacon_is_above_the_gate_and_the_bound_grows_with_range():
    bp = beacon_position(G, DEFAULT)
    assert abs(bp[2] - G.center[2] - DEFAULT.gate_beacon.up_m) < 1e-9 and np.allclose(bp[:2], G.center[:2])
    assert fix_bound(DEFAULT, 2.0) < fix_bound(DEFAULT, 8.0)
