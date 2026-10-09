"""Lost drones without communication (DI-29): missing timers, slot vacancy, the formation-keeping modes,
rejoin from behind, the envelope clear rule and the referee's degraded verdict."""

from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import EnvelopeMonitor
from holo_fleet.control.current_envelope import check_onboard
from holo_fleet.control.flows import Flows
from holo_fleet.formations import TEMPLATES
from holo_fleet.ha.automaton import AbstractObservation, LocalHybridAutomaton
from holo_fleet.ha.spec import Mode
from holo_fleet.mission import FormationClock, MissionPlan, Path, Slot
from holo_fleet.perception.formation_perception import FormationObs, NeighbourCheck
from holo_fleet.perception.perception import Perception
from holo_fleet.perception.targets import Target
from holo_fleet.referee.referee import Referee

FR = DEFAULT.form
DT = 0.1


def _plan(k=0, clock=None, template="square"):
    tm = TEMPLATES[template]
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(tm.slots),
                       path=Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=70.0,
                       clock=clock or FormationClock(0.0, 0.3, 0.0, s_end=70.0))


def _form(p, t, seen):
    """Synthetic formation observation for drone 0 of the square: slots 1-3 expected, ``seen`` matched now."""
    o = FormationObs()
    for k in (1, 2, 3):
        o.checks.append(NeighbourCheck(slot=k, expected_d=3.5, expected_pattern="", seen=k in seen,
                                       residual=0.1 if k in seen else float("nan")))
        o.required.append(k)
        if k in seen:
            p.neighbour_seen[k] = t
        elif t - p.neighbour_seen.get(k, -1e9) > 1.0:
            o.missing.append(k)
    return o


def _run(p, t0, t1, seen, in_formation=True):
    t, o = t0, None
    while t < t1 - 1e-9:
        o = _form(p, t, seen)
        p.track_missing(t, DT, o, in_formation)
        t = round(t + DT, 6)
    return o, t


def test_t_rejoin_comes_from_the_catch_up_margin():
    assert FR.t_rejoin == pytest.approx(FR.neighbour_range_m / (FR.v_recovery_max - FR.v_nominal)) == pytest.approx(37.5)


def test_missing_neighbour_is_declared_vacant_after_t_rejoin_and_reincluded():
    p = Perception(_plan(0), DEFAULT)
    o, t = _run(p, 0.0, 5.0, {1, 2, 3})
    assert o.neighbors_ok and not p.vacant
    o, t = _run(p, t, t + 30.0, {1, 3})                      # slot 2 disappears
    assert not o.neighbors_ok and not p.vacant and 0.0 < p.missing_t[2] < FR.t_rejoin
    o, t = _run(p, t, t + 9.0, {1, 3})
    assert 2 in p.vacant and o.neighbors_ok                  # vacant: no longer required
    ev = [e for e in p.events if e["type"] == "SLOT_DECLARED_VACANT"]
    assert len(ev) == 1 and ev[0]["slot"] == 2 and 36.0 <= ev[0]["t"] - 5.0 <= 38.6
    o, t = _run(p, t, t + 0.5, {1, 2, 3})                    # seen again, not yet for t_reinclude
    assert 2 in p.vacant
    o, t = _run(p, t, t + 1.3, {1, 2, 3})
    assert not p.vacant and o.neighbors_ok
    assert [e["type"] for e in p.events][-1] == "SLOT_REOCCUPIED"


def test_isolated_spurious_association_does_not_end_the_wait():
    """A single echo of another neighbour that fits the missing one's range gate (seen in the logs, once every
    1-2 s) neither ends FORMATION_WAIT_REJOIN nor resets the timer; a confirmed return does."""
    p = Perception(_plan(0), DEFAULT)
    o, t = _run(p, 0.0, 3.0, {1, 2, 3})
    o, t = _run(p, t, t + 3.0, {1, 3})
    assert not o.neighbors_ok
    tau = p.missing_t[2]
    for _ in range(10):                                      # one spurious association every 1.5 s
        o, t = _run(p, t, t + 0.1, {1, 2, 3})
        assert not o.neighbors_ok
        o, t = _run(p, t, t + 1.4, {1, 3})
    assert p.missing_t[2] > tau + 14.0
    o = _form(p, t, {1, 2, 3})                               # an echo of another neighbour at the edge of the gate
    o.checks[1].residual = -1.1
    for _ in range(10):
        p.track_missing(t, DT, o, True)
        t = round(t + DT, 6)
    assert not o.neighbors_ok
    o, t = _run(p, t, t + 0.4, {1, 2, 3})                    # the drone is really back: confirmed
    assert o.neighbors_ok and p.missing_t[2] == 0.0


def test_timers_frozen_outside_formation_keeping_and_during_planned_holds():
    p = Perception(_plan(0), DEFAULT)
    _run(p, 0.0, 2.0, {1, 2, 3})
    _run(p, 2.0, 60.0, {1, 3}, in_formation=False)          # e.g. the drone itself recovering or in a gate mode
    assert not p.vacant and p.missing_t.get(2, 0.0) == 0.0
    hold = FormationClock(0.0, 0.3, 0.0, holds=((3.0, 100.0),), s_end=70.0)   # reference holds from t = 10 s
    p = Perception(_plan(0, clock=hold), DEFAULT)
    _run(p, 0.0, 2.0, {1, 2, 3})
    _run(p, 2.0, 80.0, {1, 3})
    assert not p.vacant and p.missing_t[2] == pytest.approx(10.0 - 3.0, abs=0.3)   # counted only while moving


def _obs(**kw):
    o = AbstractObservation(form_err=0.2, t_ok=3.0)
    for k, v in kw.items():
        setattr(o, k, v)
    return o


def test_a_stacked_pair_seen_from_the_side_is_confirmed_as_a_group():
    """Seen from the right drone of the stack_pair template, the upper and the lower drone are in LEFT at the same
    range: one echo for both (seen in the mutex_deadlock_resolution run).  A range-only sonar cannot separate them, so
    one association confirms both; otherwise one of them would be missing for ever and declared vacant."""
    from holo_fleet.perception.formation_perception import check_formation

    slots = [Slot(0.0, 1.75, 1.75), Slot(0.0, 1.75, -1.75), Slot(0.0, -1.75, 0.0)]       # formations "stack_pair"
    plan = MissionPlan(drone_id="drone_2", slot_index=2, slots=slots, path=Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0),
                       gates=[], structures=[], launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=2, s_end=70.0)
    healthy = {s: True for s in ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")}
    d = float(np.hypot(3.5, 1.75))
    tg = Target(pattern=frozenset({"LEFT"}), r_min=d - 0.6, d_lower=d - 0.6, age=0.0, closing_rate=0.0, cls="DYNAMIC",
                ranges={"LEFT": d - 0.6})
    o = check_formation(plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), [tg], healthy, 0.0, {})
    assert o.neighbors_ok and all(c.seen for c in o.checks) and len(o.checks) == 2
    o = check_formation(plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), [], healthy, 5.0, {})
    assert not o.neighbors_ok and sorted(o.missing) == [0, 1]      # nothing seen: both missing


def test_formation_keeping_modes():
    ha = LocalHybridAutomaton()
    ha.step(_obs(neighbors_ok=False), 0.0, DT)
    assert ha.mode == Mode.FORMATION_WAIT_REJOIN             # own slot ok, a neighbour missing: wait, not recover
    ha.step(_obs(neighbors_ok=True, degraded=True), 0.1, DT)
    assert ha.mode == Mode.DEGRADED_FORMATION                # slot declared vacant
    ha.step(_obs(neighbors_ok=True, degraded=False), 0.2, DT)
    assert ha.mode == Mode.FORMATION_FOLLOW                  # the drone came back: formation restored
    ha.step(_obs(form_err=1.5, t_ok=0.0, neighbors_ok=False), 0.3, DT)
    assert ha.mode == Mode.FORMATION_RECOVERY                # own slot lost: recovery first
    ha.step(_obs(form_err=0.3, t_ok=1.0, neighbors_ok=False), 0.4, DT)
    assert ha.mode == Mode.FORMATION_RECOVERY                # not yet recovered for t_ok_hold
    ha.step(_obs(form_err=0.3, t_ok=2.5, neighbors_ok=False), 0.5, DT)
    assert ha.mode == Mode.FORMATION_WAIT_REJOIN             # own slot recovered, the neighbour still missing
    assert ha.determinism_violations == 0


def _flow_obs(p, t, plan):
    slot = np.array([plan.path.frame_at(plan.clock.s(t) + plan.my_slot.along)[0][0],
                     plan.my_slot.lateral, plan.path.depth_z + plan.my_slot.dz])
    return SimpleNamespace(t=t, p=np.asarray(p, float), slot_pos=slot, targets=[], R_wb=np.eye(3), yaw=0.0,
                           form=SimpleNamespace(along_corr=0.0, lateral_corr=0.0))


def test_rejoin_from_behind_never_cuts_across_the_fleet():
    plan = _plan(2)                                          # rear-left slot: along -1.75, lateral +1.75
    fl = Flows(plan, DEFAULT)
    t = 20.0
    s_ref = plan.clock.s(t)
    # level with the fleet but 2.5 m off its lane (on the right): drop back, no lateral motion
    o = _flow_obs([s_ref - 1.75, 1.75 - 2.5, -5.0], t, plan)
    v, _ = fl.formation(o, DT, recovering=True)
    assert fl.rejoin_phase == "DROP_BACK" and abs(v[1]) < 1e-9 and 0.0 <= v[0] <= 0.3 + 1e-9
    # behind the rear of the formation: move to the own lane (behind the rear-most slot)
    o = _flow_obs([s_ref - 7.0, 1.75 - 2.5, -5.0], t, plan)
    v, _ = fl.formation(o, DT, recovering=True)
    assert fl.rejoin_phase == "TO_LANE" and v[1] > 0.0
    # on its lane, behind its slot: straight up the lane to the slot
    o = _flow_obs([s_ref - 6.0, 1.75 - 0.3, -5.0], t, plan)
    v, _ = fl.formation(o, DT, recovering=True)
    assert fl.rejoin_phase == "" and v[0] > 0.3
    v, _ = fl.formation(o, DT, recovering=False)             # not recovering: plain slot tracking
    assert fl.rejoin_phase == ""


def test_failsafe_cancels_a_give_way():
    """A head-on give-way decided while holding in FAILSAFE would displace the target after the drone resumes
    (seen in the lost_drone_rejoin run: a closing-rate spike of a re-acquired echo)."""
    fl = Flows(_plan(2), DEFAULT)
    tg = Target(pattern=frozenset({"FRONT"}), r_min=5.0, d_lower=5.0, age=0.0, closing_rate=1.4, cls="DYNAMIC")
    obs = SimpleNamespace(t=0.0, p=np.zeros(3), R_wb=np.eye(3), yaw=0.0, targets=[tg])
    fl.traffic_offset(obs, DT)
    assert fl.giveway == "RIGHT"                             # active: the rule reacts
    fl2 = Flows(_plan(2), DEFAULT)
    fl2.traffic_offset(obs, DT, active=False)
    assert fl2.giveway == "" and np.linalg.norm(fl2.offset) < 1e-9     # holding: no decision
    for k in range(80):                                      # a running give-way is cancelled and decays to zero
        obs.t = 0.1 * (k + 1)
        fl.traffic_offset(obs, DT, active=False)
    assert fl.giveway == "" and np.linalg.norm(fl.offset) < 1e-6


def test_envelope_violation_clears_only_when_the_vehicle_delivers():
    m = EnvelopeMonitor()
    ok = check_onboard(np.zeros(3), np.array([0.1, 0.0, 0.0]), DEFAULT, 0.5)
    zero = np.zeros(3)
    for _ in range(50):                                      # thrusters dead while following: saturated
        m.update(True, ok, DT, v_nav=zero, v_req=np.array([0.3, 0.0, 0.0]))
    assert not m.ok
    for _ in range(100):                                     # FAILSAFE hold, nothing requested: undecidable, paused
        m.update(False, ok, DT, v_nav=zero, v_req=zero)
    assert not m.ok
    probe = [np.array([0.0, 0.0, 0.15 * np.sin(2 * np.pi * k * DT / 8.0)]) for k in range(400)]
    for k in range(100):                                     # heave probe requested, the vehicle does not move
        m.update(False, ok, DT, v_nav=zero + 0.01 * np.sin(k), v_req=probe[k])
    assert not m.ok and "not delivered" in m.reason
    for k in range(100, 200):                                # thrusters back: the probe is followed with a lag
        m.update(False, ok, DT, v_nav=0.8 * probe[k - 6], v_req=probe[k])
    assert m.ok and m.reason == ""


def test_referee_declares_a_drone_absent_and_judges_the_degraded_formation():
    tm = TEMPLATES["square"]
    path = Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0)
    clock = FormationClock(10.0, 0.3, 0.0, s_end=70.0)
    ref = Referee([f"drone_{k}" for k in range(4)], tm.offsets(), path, [], DEFAULT, True, clock=clock)
    stop = None
    for i in range(int(60.0 / DT)):
        t = i * DT
        s = clock.s(t)
        P = np.array([[s + sl.along, sl.lateral, -5.0 + sl.dz] for sl in tm.slots])
        if t >= 5.0:                                         # drone 2 stops for good at t = 5 s
            stop = P[2].copy() if stop is None else stop
            P[2] = stop
        truth = SimpleNamespace(t=t, positions=P, collision=np.zeros(4, bool), current_drift=np.zeros((4, 3)),
                                velocities=np.zeros((4, 3)))
        ref.update(truth, ["FORMATION_FOLLOW"] * 4)
    m = ref.metrics()
    deg = m["P3_degraded"]
    assert deg["exercised"] and deg["absent_at_end"] == ["drone_2"] and deg["holds"]
    t_abs = deg["absence_log"][0]["t"]
    assert 5.0 + FR.t_rejoin < t_abs < 5.0 + FR.t_rejoin + 6.0
    assert not m["P3_formation_recovery"]["all_recovered_within_run"]      # the nominal P3 is not claimed
    assert deg["final_form_err_present"] < 0.1
