"""Lost drones without communication (DI-29, DI-31): missing timers, slot vacancy, the timeout derived from the
catch-up, the reduced progress of a waiting formation (local formation time), the formation-keeping modes and
their transition reasons, rejoin from behind, the envelope clear rule and the referee's degraded verdict."""

import dataclasses

from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import EnvelopeMonitor
from holo_fleet.control.current_envelope import check_onboard
from holo_fleet.control.flows import Flows
from holo_fleet.control.formation_time import FormationTime
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


def test_t_rejoin_is_derived_from_the_catch_up_of_the_waiting_formation():
    """t_rejoin = min(t_rejoin_max, d_rejoin / (v_recovery - v_wait) + margin), d_rejoin = the sensing range of the
    neighbour check (no onboard estimate of the gap after the last association): no magic number."""
    assert FR.catch_up == pytest.approx(FR.v_recovery_max - FR.v_wait)
    assert FR.t_rejoin == pytest.approx(min(FR.t_rejoin_max, FR.neighbour_range_m / FR.catch_up + FR.t_rejoin_margin))
    assert FR.t_rejoin == pytest.approx(30.0)
    # the gap a rejoining drone can close before the timeout is the sensing range
    assert FR.catch_up * (FR.t_rejoin - FR.t_rejoin_margin) == pytest.approx(FR.neighbour_range_m)
    # it follows the configured speeds, within the cap
    slower = dataclasses.replace(FR, v_wait=0.25)
    assert slower.t_rejoin == pytest.approx(7.5 / 0.25 + 5.0) and slower.t_rejoin > FR.t_rejoin
    assert dataclasses.replace(FR, v_wait=0.45).t_rejoin == FR.t_rejoin_max
    with pytest.raises(ValueError):
        _ = dataclasses.replace(FR, v_wait=0.5).t_rejoin


def test_recovery_is_faster_than_the_waiting_formation_with_margin():
    """The waiting formation slows down but does not stop; a recovering drone closes on it faster than on a
    formation at the survey speed, also against the residual drift of the envelope."""
    assert 0.0 < FR.v_wait < FR.v_nominal
    assert FR.v_recovery_max - FR.v_wait > FR.v_recovery_max - FR.v_nominal > 0.0
    w = DEFAULT.env.w_drift_max
    assert FR.v_recovery_max - FR.v_wait - w >= 2.0 * (FR.v_recovery_max - FR.v_nominal - w)
    assert FR.v_recovery_max <= DEFAULT.env.v_max_nominal


def _ftime_run(ft, t0, t1, incomplete, mutex=False):
    t, speeds = t0, []
    while t < t1 - 1e-9:
        ft.update(t, DT, incomplete, mutex)
        speeds.append(ft.speed(t + DT, DT))
        t = round(t + DT, 6)
    return t, speeds


def test_reduced_progress_slows_the_formation_reference_with_a_ramp_and_restores_it():
    clock = FormationClock(0.0, FR.v_nominal, 0.0, s_end=1e4)
    ft = FormationTime(clock, DEFAULT)
    t, sp = _ftime_run(ft, 0.0, 5.0, False)
    assert ft.tau == 0.0 and sp[-1] == pytest.approx(FR.v_nominal)
    t, sp = _ftime_run(ft, t, t + 15.0, True)                 # a neighbour missing: reduced progress
    assert ft.reduced and sp[-1] == pytest.approx(FR.v_wait)
    steps = np.abs(np.diff([FR.v_nominal] + sp))
    assert steps.max() <= FR.a_wait * DT + 1e-9               # ramp, no step
    t_ramp = (FR.v_nominal - FR.v_wait) / FR.a_wait
    assert ft.tau == pytest.approx((1.0 - FR.v_wait / FR.v_nominal) * (15.0 - t_ramp / 2.0), abs=0.15)
    s_before = clock.s(ft.now(t))
    t, sp = _ftime_run(ft, t, t + 10.0, False)                # back: nominal speed again, gradually
    assert not ft.reduced and sp[-1] == pytest.approx(FR.v_nominal)
    assert np.abs(np.diff(sp)).max() <= FR.a_wait * DT + 1e-9
    tau = ft.tau
    t, sp = _ftime_run(ft, t, t + 10.0, False)
    assert ft.tau == pytest.approx(tau)                       # the delay is kept: no sprint to recover the plan
    assert clock.s(ft.now(t)) > s_before                      # never stopped


def test_reduced_progress_lasts_at_most_t_rejoin_and_not_in_the_mutex_zone_or_a_hold():
    clock = FormationClock(0.0, FR.v_nominal, 0.0, s_end=1e4)
    ft = FormationTime(clock, DEFAULT)
    t, sp = _ftime_run(ft, 0.0, FR.t_rejoin - 1.0, True)
    assert ft.reduced
    t, sp = _ftime_run(ft, t, t + 20.0, True)                 # never comes back: budget exhausted
    assert not ft.reduced and sp[-1] == pytest.approx(FR.v_nominal)
    ft = FormationTime(clock, DEFAULT)
    t, sp = _ftime_run(ft, 0.0, 20.0, True, mutex=True)       # gate flow: nominal rate, episode frozen
    assert not ft.reduced and ft.tau == 0.0 and ft.episode_t == 0.0
    hold = FormationClock(0.0, FR.v_nominal, 0.0, holds=((3.0, 100.0),), s_end=1e4)
    ft = FormationTime(hold, DEFAULT)
    t, sp = _ftime_run(ft, 0.0, 10.0, False)
    tau = ft.tau
    t, sp = _ftime_run(ft, t, t + 30.0, True)                 # planned hold: nothing to slow down
    assert ft.tau == tau and ft.episode_t == 0.0


def test_slot_reference_and_feed_forward_follow_the_own_formation_time():
    """Slowing the vehicle alone would leave its slot moving at the planned speed (a formation error): the slot
    of the perception, the slot of the flow and the feed-forward use the same formation time."""
    plan = _plan(0)
    fl = Flows(plan, DEFAULT)
    p = Perception(plan, DEFAULT)
    fl.ftime.tau, fl.ftime.rate = 4.0, FR.v_wait / FR.v_nominal
    t = 30.0
    slot, _R, s_ref = p.slot_reference(fl.ftime.now(t), fl.sigma)
    _n, _R2, s_flow = fl._slot(SimpleNamespace(t=t))
    assert s_ref == pytest.approx(s_flow) == pytest.approx(plan.clock.s(t - 4.0))
    obs = SimpleNamespace(t=t, p=slot.copy(), slot_pos=slot, targets=[], R_wb=np.eye(3), yaw=0.0,
                          form=SimpleNamespace(along_corr=0.0, lateral_corr=0.0))
    v, _ = fl.formation(obs, DT)
    assert v[0] == pytest.approx(FR.v_wait, abs=1e-6)          # on the slot: moves at the reduced speed


def test_reduced_progress_is_a_private_state_of_each_drone():
    """No hidden channel: every drone owns its formation time, and the module keeps no state of its own (no
    module or class variable, no ``global``), so one drone slowing down changes nothing in another drone."""
    import ast
    import inspect

    from holo_fleet.control import formation_time

    a, b = Flows(_plan(0), DEFAULT), Flows(_plan(1), DEFAULT)
    assert a.ftime is not b.ftime
    _ftime_run(a.ftime, 0.0, 15.0, True)
    assert a.ftime.reduced and a.ftime.tau > 0.0
    assert not b.ftime.reduced and b.ftime.tau == 0.0 and b.ftime.rate == 1.0
    tree = ast.parse(inspect.getsource(formation_time))
    assert not [n for n in tree.body if isinstance(n, (ast.Assign, ast.AnnAssign, ast.AugAssign))]
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "FormationTime")
    assert not [n for n in cls.body if isinstance(n, (ast.Assign, ast.AnnAssign))]
    assert not [n for n in ast.walk(tree) if isinstance(n, (ast.Global, ast.Nonlocal))]


def _lane_run(ft, t0, t1, incomplete, residual, keeping=True):
    t, sp = t0, []
    while t < t1 - 1e-9:
        r = residual(t) if callable(residual) else residual
        ft.update(t, DT, incomplete, False, front_residual=r, keeping=keeping)
        sp.append(ft.speed(t + DT, DT))
        t = round(t + DT, 6)
    return t, sp


def test_same_lane_pairs_depend_only_on_the_template():
    from holo_fleet.perception.formation_perception import same_lane_ahead

    assert [same_lane_ahead(_plan(k)) for k in range(4)] == [[], [], [0], [1]]     # square: rear drones
    assert same_lane_ahead(_plan(1, template="pair_column")) == [0]
    for tm in ("triangle", "line3", "stack_pair", "line6"):
        n = len(TEMPLATES[tm].slots)
        assert all(same_lane_ahead(_plan(k, template=tm)) == [] for k in range(n)), tm


def test_a_drone_follows_the_slowdown_of_the_drone_ahead_without_noticing_anything():
    """It never noticed the absence (range-only: another echo stands in for the missing drone), but the drone
    ahead slows: the spacing shrinks, so it slows too - faster than the drone ahead did - and it keeps slowing
    until the drone ahead pulls away again."""
    clock = FormationClock(0.0, FR.v_nominal, 0.0, s_end=1e4)
    ft = FormationTime(clock, DEFAULT)
    t, sp = _lane_run(ft, 0.0, 3.0, False, 0.0)
    assert not ft.reduced and sp[-1] == pytest.approx(FR.v_nominal)
    t, sp = _lane_run(ft, t, t + 1.0, False, -0.05)           # calm-flight residual: nothing happens
    assert not ft.reduced
    t, sp = _lane_run(ft, t, t + 3.0, False, -FR.follow_on_m - 0.01)
    assert ft.following and ft.reduced and sp[-1] == pytest.approx(FR.v_wait)
    assert np.abs(np.diff(sp)).max() <= FR.a_follow * DT + 1e-9
    t, sp = _lane_run(ft, t, t + 5.0, False, 0.0)             # spacing restored, the drone ahead still slow: hold
    assert ft.reduced and sp[-1] == pytest.approx(FR.v_wait)
    t, sp = _lane_run(ft, t, t + 4.0, False, FR.follow_on_m + 0.01)     # it pulls away: resume
    assert not ft.reduced and sp[-1] == pytest.approx(FR.v_nominal)


def test_in_a_lane_the_slowdown_ends_front_to_back_and_a_late_noticer_does_not_fall_behind():
    clock = FormationClock(0.0, FR.v_nominal, 0.0, s_end=1e4)
    ft = FormationTime(clock, DEFAULT)
    t, sp = _lane_run(ft, 0.0, 10.0, True, 0.0)               # both slowed: same spacing
    assert ft.reduced
    t, sp = _lane_run(ft, t, t + 5.0, False, 0.0)             # its own reason ended before the drone ahead's
    assert ft.reduced                                         # keeps slowing: it would close on it otherwise
    ft2 = FormationTime(clock, DEFAULT)
    t, sp = _lane_run(ft2, 0.0, 10.0, True, FR.follow_on_m + 0.02)     # notices late, the drone ahead runs on
    assert not ft2.reduced and ft2.tau == pytest.approx(0.0)
    ft3 = FormationTime(clock, DEFAULT)                       # out of its slot (rejoining from behind): the
    t, sp = _lane_run(ft3, 0.0, 10.0, True, 1.0, keeping=False)        # residual is its own lag, not a resume
    assert ft3.reduced and sp[-1] == pytest.approx(FR.v_wait)


def test_progress_consensus_runs_only_near_the_own_slot():
    """A drone left behind sees every neighbour 'ahead': far from its slot the range residuals measure its own
    displacement, and integrating them would carry its reference away from the fleet; near its slot (in any mode)
    the consensus is what re-aligns the references of the drones."""
    plan = _plan(2)
    fl = Flows(plan, DEFAULT)
    far = _flow_obs([plan.clock.s(20.0) - 1.75 - 3.0, 1.75, -5.0], 20.0, plan)      # 3 m behind its slot
    far.form.along_corr = 0.9
    for _ in range(50):
        fl.formation(far, DT, recovering=True)
    assert fl.sigma == 0.0
    near = _flow_obs([plan.clock.s(20.0) - 1.75 - 0.3, 1.75, -5.0], 20.0, plan)     # 0.3 m behind, recovering
    near.form.along_corr = 0.9
    for _ in range(50):
        fl.formation(near, DT, recovering=True)
    assert fl.sigma > 0.0


def test_missing_neighbour_is_declared_vacant_after_t_rejoin_and_reincluded():
    p = Perception(_plan(0), DEFAULT)
    o, t = _run(p, 0.0, 5.0, {1, 2, 3})
    assert o.neighbors_ok and not p.vacant
    o, t = _run(p, t, t + FR.t_rejoin, {1, 3})               # slot 2 disappears
    assert not o.neighbors_ok and not p.vacant and 0.0 < p.missing_t[2] < FR.t_rejoin
    o, t = _run(p, t, t + 3.0, {1, 3})
    assert 2 in p.vacant and o.neighbors_ok                  # vacant: no longer required
    ev = [e for e in p.events if e["type"] == "SLOT_DECLARED_VACANT"]
    assert len(ev) == 1 and ev[0]["slot"] == 2 and FR.t_rejoin + 0.5 <= ev[0]["t"] - 5.0 <= FR.t_rejoin + 1.6
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


def test_every_mode_change_logs_the_cause_from_the_guard_that_fired():
    ha = LocalHybridAutomaton()
    seq = [(_obs(neighbors_ok=False), Mode.FORMATION_WAIT_REJOIN, "neighbor_missing"),
           (_obs(neighbors_ok=False), Mode.FORMATION_WAIT_REJOIN, ""),
           (_obs(neighbors_ok=True), Mode.FORMATION_FOLLOW, "neighbor_rejoined"),
           (_obs(neighbors_ok=False), Mode.FORMATION_WAIT_REJOIN, "neighbor_missing"),
           (_obs(neighbors_ok=True, degraded=True), Mode.DEGRADED_FORMATION, "rejoin_timeout"),
           (_obs(neighbors_ok=True, degraded=False), Mode.FORMATION_FOLLOW, "slot_reoccupied"),
           (_obs(form_err=1.5, t_ok=0.0), Mode.FORMATION_RECOVERY, "own_slot_lost"),
           (_obs(form_err=0.3, t_ok=2.5), Mode.FORMATION_FOLLOW, "slot_recovered"),
           (_obs(sense_ok=False), Mode.FAILSAFE_HOLD_OR_RETREAT, "sensor_fault"),
           (_obs(form_err=0.8, t_ok=0.0), Mode.FORMATION_RECOVERY, "fault_cleared"),
           (_obs(env_ok=False), Mode.FAILSAFE_HOLD_OR_RETREAT, "envelope_violation"),
           (_obs(d_min=2.0), Mode.SEPARATION_WARNING, "fault_cleared"),
           (_obs(d_min=1.5), Mode.COLLISION_AVOIDANCE, "collision_risk"),
           (_obs(d_min=2.1), Mode.SEPARATION_WARNING, "collision_risk_cleared"),
           (_obs(mutex_zone=True, at_queue=True, has_prio=False), Mode.MUTEX_YIELD, "separation_clear"),
           (_obs(mutex_zone=True, at_queue=True, has_prio=True, occ_busy=True), Mode.MUTEX_YIELD, ""),
           (_obs(mutex_zone=True, at_queue=True, has_prio=True), Mode.MUTEX_PASS, "mutex_priority_and_free"),
           (_obs(mutex_zone=True, passed=True), Mode.FORMATION_RECOVERY, "mutex_exit_passed"),
           (_obs(mutex_zone=True, at_queue=False), Mode.MUTEX_APPROACH, "mutex_zone_entered"),
           (_obs(mutex_zone=True, at_queue=True, occ_busy=True, has_prio=True), Mode.MUTEX_YIELD, "mutex_occupied")]
    for i, (o, mode, reason) in enumerate(seq):
        rec = ha.step(o, 0.1 * i, DT)
        assert (ha.mode, rec.reason) == (mode, reason), i
        assert rec.label                                      # every edge has a readable guard
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


def test_no_give_way_while_catching_up_with_the_own_fleet():
    """Seen in a lost_drone_rejoin probe (DI-31): a drone rejoining from behind at the catch-up speed sees the drone
    ahead close in its FRONT sector faster than the head-on threshold and, 3 m behind its slot, does not recognise
    it as a neighbour; a give-way decided then pushed it 2 m towards the next lane."""
    plan = _plan(2)
    t = 20.0
    tg = Target(pattern=frozenset({"FRONT"}), r_min=6.0, d_lower=6.0, age=0.0, closing_rate=0.6, cls="DYNAMIC")
    fl = Flows(plan, DEFAULT)
    o = _flow_obs([plan.clock.s(t) - 1.75 - 3.0, 1.75, -5.0], t, plan)           # 3 m behind its slot, on its lane
    o.targets = [tg]
    fl.formation(o, DT, recovering=True)
    assert fl.giveway == "" and np.linalg.norm(fl.offset) < 1e-9
    fl2 = Flows(plan, DEFAULT)
    o2 = _flow_obs([plan.clock.s(t) - 1.75, 1.75, -5.0], t, plan)                 # on its slot: a real head-on
    o2.targets = [tg]
    fl2.formation(o2, DT)
    assert fl2.giveway == "RIGHT"
