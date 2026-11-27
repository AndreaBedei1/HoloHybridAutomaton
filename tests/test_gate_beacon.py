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
        for gid, _tm, r, az, el in sim.measure(t, {"d": (truth, yaw)})["d"]:
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


def test_the_controller_uses_the_beacon_and_the_fit_without_breaking_the_step():
    """End to end through DroneController.step (v40 SUT_ERROR: the fit branch shadowed the frame data): a structure fit
    and gate-beacon receptions set the classification pose, the beacon first."""
    from holo_fleet.control.controller import DroneController
    from holo_fleet.perception.frame import SensorFrame
    from holo_fleet.perception.gate_beacon import beacon_position
    from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name

    from dataclasses import replace

    cfg = replace(DEFAULT, gate_beacon=replace(DEFAULT.gate_beacon, max_bound_m=2.0))  # the launch point is ~8 m away
    c = DroneController(PLAN, cfg)
    fit = {"matched": 4, "offset": [0.0, 0.5], "rms": 0.03, "u": 0.2}

    def fake_update(*a, **k):
        c.perception.aiding.last = dict(fit)
        return None

    c.perception.aiding.update = fake_update
    bp = beacon_position(G, cfg)
    for i in range(40):
        t = round(0.1 * i, 3)
        data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
                "IMUSensor": np.zeros((4, 3))}
        for s in SECTORS:
            data[sonar_sensor_name(s)] = np.zeros(DEFAULT.perc.sonar.ranges().size)
        stamp = {k: t for k in data}
        rx = []
        if i > 0:
            p = c.perception.nav.state.p + np.array([0.0, 0.7, 0.0])
            yaw = c.perception.nav.state.yaw
            rel = _R(yaw).T @ (bp - p)
            r = float(np.linalg.norm(rel))
            rx = [(G.gate_id, t, r, math.atan2(rel[1], rel[0]), math.asin(rel[2] / r))]
        data["GateBeaconRx"] = tuple(rx)
        c.step(SensorFrame(t, data, stamp), 0.1)
        if i == 1:
            assert c.perception.cls_source == "fit"
    assert c.perception.cls_source == "beacon"
    assert np.linalg.norm(c.perception.cls_used[:2] - np.array([0.0, 0.7])) < 0.05
