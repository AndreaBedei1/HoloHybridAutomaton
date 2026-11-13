"""Variant B2 never reaches the safety layer: same sonar frames with or without acoustic receptions give the same
separation, collision-avoidance, sensing-health and escape inputs; only perception modules read the beacon."""

import ast
import dataclasses
from pathlib import Path

import numpy as np
import pytest

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.control.controller import DroneController
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
from holo_fleet.sim.beacon import BeaconTransport
from holo_fleet.sim.scenarios import SCENARIOS

PKG = Path(__file__).resolve().parents[1] / "holo_fleet"
B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
DT = 0.1
RANGES = DEFAULT.perc.sonar.ranges()
SAFETY_MODES = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT")


def _frame(t, rx=(), echo=None):
    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3))}
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(RANGES.size)
    if echo is not None:                                 # a compact unexplained echo ahead: a vehicle
        data[sonar_sensor_name("FRONT")][(RANGES >= echo[0]) & (RANGES <= echo[1])] = 0.9
    stamp = {k: t for k in data}
    if rx:
        data["AcousticRx"] = tuple(rx)
        stamp["AcousticRx"] = max(x[1] for x in rx)
    return SensorFrame(t, data, stamp)


def _run(cfg, loud, scenario, steps=70):
    plan = SCENARIOS[scenario](cfg).plans[0]
    c = DroneController(plan, cfg)
    c.perception.acoustic.set_identities({k: k for k in range(len(plan.slots))})
    records = []
    for i in range(steps):
        t = round(DT * i, 3)
        # every neighbour heard 4 m to the left at every step (loud) or never (mute)
        rx = [(k, t - 0.01, 4.0, np.pi / 2, 0.0) for k in range(len(plan.slots)) if k != plan.slot_index] if loud else ()
        echo = (1.8, 2.0) if 2.0 <= t <= 4.0 else None
        c.step(_frame(t, rx, echo), DT)
        records.append(c.last_record)
    return records


@pytest.mark.parametrize("scenario", ["gate_single", "lost_drone_rejoin"])
def test_acoustic_receptions_cannot_override_the_safety_layer(scenario):
    loud, mute = _run(B2, True, scenario), _run(B2, False, scenario)
    assert any(r["mode"] in SAFETY_MODES for r in mute)          # the echo does trigger the safety layer
    for a, b in zip(loud, mute):
        for key in ("d_min", "sense_ok", "env_ok", "targets", "escape"):
            assert a[key] == b[key], (a["t"], key)
        if a["mode"] in SAFETY_MODES or b["mode"] in SAFETY_MODES:
            assert a["mode"] == b["mode"], a["t"]


# modules allowed to mention the beacon / acoustic receptions: simulator side, configuration, run bookkeeping, and the
# perception modules whose B2 rules are documented (perception.py, acoustic*.py, formation / gate perception)
ALLOWED = {"config.py", "runner.py", "sim/beacon.py", "sim/holo_env.py", "sim/acoustic_channel.py",
           "perception/acoustic.py", "perception/acoustic_gate.py", "perception/perception.py",
           "perception/formation_perception.py", "perception/gate_perception.py", "perception/sensor_suite.py",
           "perception/payload.py", "perception/nav.py",       # H6 payload: packet format, own navigation bound
           "perception/structure_aiding.py",    # H6: payload balls as the identity premise of a fit (no acoustic fix read)
           "control/controller.py", "referee/robustness_metrics.py", "referee/referee.py",
           "referee/p3.py"}                    # offline: the premise A of the P3 contract reads the logged silence


def test_only_perception_modules_read_the_beacon():
    used = {p.relative_to(PKG).as_posix() for p in PKG.rglob("*.py")
            if any(w in p.read_text(encoding="utf-8") for w in ("AcousticRx", "acoustic", "beacon"))}
    assert used <= ALLOWED, used - ALLOWED
    # the automaton, the guards, the flows and the sonar safety chain never touch it
    for rel in ("ha/spec.py", "ha/automaton.py", "control/flows.py", "control/escape.py", "perception/targets.py",
                "perception/sonar_geometry.py", "perception/sonar_processing.py"):
        assert rel not in used


def test_controller_mentions_the_beacon_only_in_its_log_record():
    tree = ast.parse((PKG / "control/controller.py").read_text(encoding="utf-8"))
    in_dict = {id(n) for d in ast.walk(tree) if isinstance(d, ast.Dict) for n in ast.walk(d)}
    uses = [n for n in ast.walk(tree) if isinstance(n, ast.Attribute) and n.attr.startswith("acoustic")]
    assert all(id(n) in in_dict for n in uses)


def test_transport_reports_the_longest_absence():
    tr = BeaconTransport(["a", "b"], B2.beacon, seed=0)
    m = (0.0, 3.0, 0.0, 0.0)
    tr.due = [(1.0, "b", 0, 0.999, m), (2.0, "a", 1, 1.999, m), (4.0, "b", 0, 3.999, m)]
    tr.deliver(5.0)
    tr.t_now = 6.0
    # b hears a at 1 and 4 (gaps 1, 3, then 2 to the end); a hears b at 2 (gap 2, then 4 to the end)
    assert tr.summary()["longest_absence_s"] == 4.0
