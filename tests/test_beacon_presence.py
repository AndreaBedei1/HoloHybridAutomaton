"""Variant B1 (one-way acoustic presence beacon): rule R1, and what the beacon can never do.

A ping carries only the sender's static identity.  Its single onboard use (rule R1, perception.py): a neighbour
heard within ttl_s is declared MISSING after NEIGHBOUR_MEMORY_S + missing_grace_s without a sonar match instead of
NEIGHBOUR_MEMORY_S.  Synthetic frames with empty sonar profiles (no neighbour is ever matched); receptions are
injected the way the simulator delivers them (frame["BeaconRx"]: (identity, reception time)).
"""

import ast
import dataclasses
from pathlib import Path

import numpy as np
import pytest

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.control.controller import DroneController
from holo_fleet.perception.formation_perception import NEIGHBOUR_MEMORY_S
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
from holo_fleet.sim.beacon import BeaconTransport
from holo_fleet.sim.scenarios import SCENARIOS

PKG = Path(__file__).resolve().parents[1] / "holo_fleet"
B1 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_PRESENCE"))
DT = 0.1
GRACE, TTL = B1.beacon.missing_grace_s, B1.beacon.ttl_s
RANGES = DEFAULT.perc.sonar.ranges()
NEIGHBOURS = (1, 2, 3)


def _frame(t, rx=(), echo=None):
    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3))}
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(RANGES.size)
    if echo is not None:                                 # a compact unexplained echo ahead: a vehicle
        data[sonar_sensor_name("FRONT")][(RANGES >= echo[0]) & (RANGES <= echo[1])] = 0.9
    stamp = {k: t for k in data}
    if rx:
        data["BeaconRx"] = tuple(rx)
    return SensorFrame(t, data, stamp)


def _every(period, slots=NEIGHBOURS, start=0.0):
    return lambda t: slots if t >= start - 1e-9 and abs(t / period - round(t / period)) < 1e-6 else ()


def _run(cfg, pings, scenario="lost_drone_rejoin", steps=60, echo=None):
    """``pings(t)``: slots whose ping is received at t.  Returns the controller, the first MISSING time of every
    neighbour and the per-step records."""
    plan = SCENARIOS[scenario](cfg).plans[0]
    c = DroneController(plan, cfg)
    c.perception.presence.set_identities({k: k for k in range(len(plan.slots))})
    first, records = {}, []
    for i in range(steps):
        t = round(DT * i, 3)
        c.step(_frame(t, [(k, t) for k in pings(t)], None if echo is None else echo(t)), DT)
        records.append(c.last_record)
        for k, v in c.perception.missing_state.items():
            if v:
                first.setdefault(k, t)
    return c, first, records


def test_beacon_presence_defers_a_missing_declaration_by_at_most_the_grace():
    _c, b0, _ = _run(DEFAULT, _every(1.0))              # the baseline ignores receptions
    _c, b1, _ = _run(B1, _every(1.0))
    assert set(b0) == set(b1) == set(NEIGHBOURS)
    for k in NEIGHBOURS:
        assert b0[k] == pytest.approx(NEIGHBOUR_MEMORY_S + DT)
        assert b1[k] == pytest.approx(NEIGHBOUR_MEMORY_S + GRACE + DT)    # deferred, never longer than the grace


def test_beacon_absence_means_nothing():
    """No reception: B1 declares exactly when the baseline does (silence is not evidence of absence)."""
    assert _run(B1, lambda t: ())[1] == _run(DEFAULT, lambda t: ())[1]


def test_stale_beacon_presence_expires_after_the_ttl():
    _c, b1, _ = _run(B1, lambda t: NEIGHBOURS if abs(t - 0.2) < 1e-6 else ())
    for k in NEIGHBOURS:
        assert b1[k] == pytest.approx(0.2 + TTL + DT)               # held only while the ping is recent


def test_beacon_cannot_clear_a_declared_missing():
    """Pings that start after the declaration change nothing: only a sonar confirmation near the slot brings a
    neighbour back, and the missing timer runs as in the baseline."""
    late = _every(0.5, start=2.0)
    c1, b1, _ = _run(B1, late)
    c0, b0, _ = _run(DEFAULT, late)
    assert b1 == b0 and all(c1.perception.missing_state[k] for k in NEIGHBOURS)
    assert c1.perception.missing_t == c0.perception.missing_t and c1.perception.presence_held == []


SAFETY_MODES = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT")


@pytest.mark.parametrize("scenario", ["gate_single", "lost_drone_rejoin"])
def test_beacon_cannot_authorize_pass_clear_cr_or_override_safety(scenario):
    """Same frames, a vehicle echo ahead from 2 s to 4 s, the beacon hearing every neighbour at every step or
    never: separation, collision avoidance, sensing health and every gate input are identical."""
    echo = lambda t: (1.8, 2.0) if 2.0 <= t <= 4.0 else None
    _c, _f, loud = _run(B1, lambda t: tuple(range(8)), scenario=scenario, steps=70, echo=echo)
    _c, _f, mute = _run(B1, lambda t: (), scenario=scenario, steps=70, echo=echo)
    assert any(r["mode"] in SAFETY_MODES for r in mute)          # the echo does trigger the safety layer
    assert any(r["form"]["beacon_held"] for r in loud)            # and rule R1 is active in the loud run
    for a, b in zip(loud, mute):
        for key in ("d_min", "sense_ok", "env_ok", "targets", "escape", "gate"):
            assert a[key] == b[key], (a["t"], key)
        if a["mode"] in SAFETY_MODES or b["mode"] in SAFETY_MODES:
            assert a["mode"] == b["mode"], a["t"]


ALLOWED = {"config.py", "runner.py", "sim/beacon.py", "sim/holo_env.py", "sim/sensor_noise.py",
           "perception/beacon_presence.py", "perception/perception.py", "perception/sensor_suite.py",
           "control/controller.py", "referee/robustness_metrics.py"}


def test_no_safety_or_gate_module_reads_the_beacon():
    """Structure: the automaton, the safety layer, the gate perception and the classifiers never see the beacon;
    in the perception it enters only rule R1, in the controller only the log record."""
    used = {p.relative_to(PKG).as_posix() for p in PKG.rglob("*.py")
            if "beacon" in p.read_text(encoding="utf-8").lower() or "presence" in p.read_text(encoding="utf-8")}
    assert used <= ALLOWED, used - ALLOWED

    def owners(rel):
        tree = ast.parse((PKG / rel).read_text(encoding="utf-8"))
        out = []
        for fn in ast.walk(tree):
            if isinstance(fn, ast.FunctionDef):
                for node in ast.walk(fn):
                    if isinstance(node, ast.Attribute) and node.attr in ("presence", "presence_held"):
                        out.append((fn.name, node))
        return tree, out

    _tree, uses = owners("perception/perception.py")
    assert {name for name, _n in uses} <= {"__init__", "update", "track_missing"}
    tree, uses = owners("control/controller.py")
    in_dict = {id(n) for d in ast.walk(tree) if isinstance(d, ast.Dict) for n in ast.walk(d)}
    assert uses and all(id(node) in in_dict for _name, node in uses)    # only inside the log record


def test_transport_reports_the_longest_absence():
    tr = BeaconTransport(["a", "b"], B1.beacon, seed=0)
    tr.due = [(1.0, "b", 0, 0.999), (2.0, "a", 1, 1.999), (4.0, "b", 0, 3.999)]
    tr.deliver(5.0)
    tr.t_now = 6.0
    # b hears a at 1 and 4 (gaps 1, 3, then 2 to the end); a hears b at 2 (gap 2, then 4 to the end)
    assert tr.summary()["longest_absence_s"] == 4.0
