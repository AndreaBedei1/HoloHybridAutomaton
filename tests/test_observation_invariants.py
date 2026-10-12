"""Observation consistency: semantic invariants of the abstract observation and the runtime check."""

import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest
import z3

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import DroneController
from holo_fleet.ha.automaton import AbstractObservation
from holo_fleet.ha.observation_invariants import INVARIANTS, PY_INV, consistent, violated
from holo_fleet.ha.spec import BOOL_VARS, REAL_VARS, Mode
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.perception import sonar_sensor_name
from holo_fleet.perception.sonar_processing import SECTORS
from holo_fleet.sim.scenarios import SCENARIOS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
from common import Obs, Z3L  # noqa: E402


def test_valid_observations_pass():
    assert violated(AbstractObservation()) == []                          # cruising, nothing in view
    queued = AbstractObservation(mutex_zone=True, at_queue=True, has_prio=True, occ_busy=True)
    assert consistent(queued)                                              # priority + busy CR: separate beliefs
    recovered = AbstractObservation(form_err=0.2, t_ok=3.0, neighbors_ok=True)
    assert consistent(recovered)
    latched = AbstractObservation(occ_busy=True, committed=True)           # latches may outlive the zone
    assert consistent(latched)


def test_at_queue_outside_the_approach_zone_is_inconsistent():
    assert violated(AbstractObservation(at_queue=True, mutex_zone=False)) == ["I2"]


def test_incoherent_gate_states():
    assert violated(AbstractObservation(passed=True, mutex_zone=True)) == ["I3"]
    assert violated(AbstractObservation(has_prio=True, at_queue=False)) == ["I1"]
    # passed and still queued: the queue point is in the zone (I2), a passed gate is not (I3)
    assert set(violated(AbstractObservation(passed=True, at_queue=True, mutex_zone=False))) == {"I2"}
    assert set(violated(AbstractObservation(passed=True, at_queue=True, mutex_zone=True))) == {"I3"}


def test_stale_t_ok_and_malformed_numbers():
    assert violated(AbstractObservation(t_ok=1.0, form_err=DEFAULT.form.e_ok)) == ["I4"]
    # a missing neighbour is not an error of the drone itself: its own 'ok' counter keeps running
    assert consistent(AbstractObservation(t_ok=1.0, neighbors_ok=False))
    assert consistent(AbstractObservation(t_ok=1.0, neighbors_ok=True, degraded=True))
    assert violated(AbstractObservation(form_err=-0.1)) == ["N1"]
    assert "N0" in violated(AbstractObservation(d_min=float("nan")))     # NaN would disable both hazard guards
    assert "N0" in violated(AbstractObservation(t_ok=math.inf))
    assert "N0" in violated(AbstractObservation(mutex_zone=None))


def _random_obs(rng):
    return AbstractObservation(
        d_min=rng.uniform(-0.5, 4.0), form_err=rng.uniform(0, 2.0), t_ok=rng.choice([0.0, rng.uniform(0, 4.0)]),
        sense_ok=rng.random() < 0.9, env_ok=rng.random() < 0.9, mutex_zone=rng.random() < 0.5,
        at_queue=rng.random() < 0.5, occ_busy=rng.random() < 0.5, has_prio=rng.random() < 0.5,
        passed=rng.random() < 0.3, neighbors_ok=rng.random() < 0.8, degraded=rng.random() < 0.3,
        committed=rng.random() < 0.3)


def test_python_and_z3_invariants_agree():
    rng = random.Random(3)
    sym = Obs()
    for _ in range(400):
        o = _random_obs(rng)
        subs = [(getattr(sym, v), z3.RealVal(str(getattr(o, v)))) for v in REAL_VARS] + \
               [(getattr(sym, v), z3.BoolVal(bool(getattr(o, v)))) for v in BOOL_VARS]
        for inv in INVARIANTS:
            py = bool(inv.pred(o, PY_INV, DEFAULT))
            zv = z3.is_true(z3.simplify(z3.substitute(inv.pred(sym, Z3L, DEFAULT), *subs)))
            assert py == zv, (inv.name, o)


def _frame(t):
    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3))}
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(234)
    return SensorFrame(t, data, {k: t for k in data})


def _controller():
    return DroneController(SCENARIOS["p1_head_on"](DEFAULT).plans[0], DEFAULT)


def test_consistent_observations_reach_the_automaton_unchanged():
    c = _controller()
    for k in range(5):
        c.step(_frame(0.1 * k), 0.1)
    assert c.observation_violations == 0 and c.last_record["obs_consistent"]
    assert c.ha.mode == Mode.FORMATION_FOLLOW
    assert not [e for e in c.events if e["type"] == "OBSERVATION_INCONSISTENT"]


def test_inconsistent_observation_goes_to_failsafe_and_is_logged():
    c = _controller()
    c.step(_frame(0.0), 0.1)
    real_update = c.perception.update

    def corrupted(*a, **k):                     # a perception defect: priority without the queue point
        obs = real_update(*a, **k)
        obs.ab.has_prio, obs.ab.at_queue = True, False
        return obs

    c.perception.update = corrupted
    c.step(_frame(0.1), 0.1)
    assert c.ha.mode == Mode.FAILSAFE_HOLD_OR_RETREAT                     # existing fault edge, no new mode
    ev = [e for e in c.events if e["type"] == "OBSERVATION_INCONSISTENT"]
    assert len(ev) == 1 and ev[0]["violated"] == ["I1"] and ev[0]["t"] == pytest.approx(0.1)
    assert ev[0]["observation"]["has_prio"] is True and ev[0]["observation"]["at_queue"] is False
    assert c.observation_violations == 1 and c.last_record["obs_consistent"] is False
    c.perception.update = real_update                                     # defect gone: back to normal
    for k in range(2, 8):
        c.step(_frame(0.1 * k), 0.1)
    assert c.ha.mode != Mode.FAILSAFE_HOLD_OR_RETREAT and c.observation_violations == 1


@pytest.mark.parametrize("n", [3, 4, 5, 6])
def test_gate_perception_keeps_the_queue_point_inside_the_approach_zone(n):
    """I2 by construction, also for n >= 5 whose outer queue points lie beyond corridor_half_width."""
    sc = SCENARIOS["gate_single"](DEFAULT)
    plan = sc.plans[0]
    g = plan.gates[0]
    G = DEFAULT.gate
    left = np.array([-g.axis[1], g.axis[0], 0.0])
    yaw = math.atan2(g.axis[1], g.axis[0])
    n_queued = 0
    for q_l in G.queue_laterals(n):
        plan.queue_lateral, plan.queue_s = q_l, G.queue_s(n)
        gp = GatePerception(plan, DEFAULT)
        for ds in (-0.3, 0.0, 0.3, 8.0):                    # around the queue point, and beyond the exit
            for dl in (-0.3, 0.0, 0.3):
                p = g.center + (plan.queue_s + ds) * g.axis + (q_l + dl) * left
                o = gp.update(0.0, p, yaw, [], {})
                n_queued += o.at_queue
                if o.at_queue:
                    assert o.in_zone, (n, q_l, ds, dl)
                assert not (o.passed and o.in_zone)
    assert n_queued == 9 * n                                # every queue point and its tolerance box
