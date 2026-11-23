"""Gate-aware avoidance (control/gate_safety.py, the P2-restricted selection of control/escape.py).

A drone that is not admitted to the critical region (CR) chooses its separation command only among the commands whose
backup keeps its true centre out of the CR under the declared bounds.  These tests use the deployed planner, Flows and
GateConstraint on the real gate of a scenario plan; the threats are sector patterns as the perception reports them.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control import gate_safety as gs
from holo_fleet.control.controller import DroneController
from holo_fleet.control.escape import EscapePlanner
from holo_fleet.control.flows import Flows
from holo_fleet.ha.spec import Mode
from holo_fleet.sim.scenarios import SCENARIOS

CFG = DEFAULT
G, ENV = CFG.gate, CFG.env
PLAN = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
GATE = PLAN.gates[0]
YAW = math.atan2(GATE.axis[1], GATE.axis[0])
R = np.array([[math.cos(YAW), -math.sin(YAW), 0.0], [math.sin(YAW), math.cos(YAW), 0.0], [0.0, 0.0, 1.0]])
ROOT = Path(__file__).resolve().parents[1]


def _gc(s, l=0.0, dz=0.0, v_gate=(0.0, 0.0, 0.0)):
    p = GATE.from_gate_frame([s, l, dz])
    a = np.asarray(GATE.axis, float)
    left = np.array([-a[1], a[0], 0.0])
    v = v_gate[0] * a + v_gate[1] * left + np.array([0.0, 0.0, v_gate[2]])
    return gs.constraint_for(CFG, GATE, p, v, authorized=False)


def _p2(gc):
    return lambda V: gc.margins_body(V, R)


def _hold(gc):
    anchor = gc.q.copy()
    return R.T @ gc.hold_velocity(anchor), gc.hold_margin(anchor)


def _warn(gc, patterns, v_des=np.zeros(3), unhealthy=(), planner=None, hold=True):
    ep = planner or EscapePlanner(CFG)
    th = [(frozenset(p), ENV.v_open) for p in patterns]
    return ep.warning_velocity(np.asarray(v_des, float), th, list(unhealthy), YAW, ENV.v_max_nominal, p2=_p2(gc),
                               degraded=bool(unhealthy), hold=_hold(gc) if hold else None)


def _escape(gc, patterns, planner=None):
    ep = planner or EscapePlanner(CFG)
    th = [(frozenset(p), 0.0) for p in patterns]
    return ep.escape_velocity(th, [], YAW, ENV.v_escape, p2=_p2(gc), hold=_hold(gc))


def _opening(ep, patterns, v_body):
    """Worst-case opening speed of a body velocity over the threats (the guarantee tables of the planner)."""
    sp = float(np.linalg.norm(v_body))
    if sp < 1e-9:
        return 0.0
    i = int(np.argmin(np.linalg.norm(ep.cands - v_body / sp, axis=1)))
    return min(sp * float(ep.guarantee(frozenset(p))[i]) for p in patterns)


def test_the_v11_planner_answers_a_threat_from_behind_at_the_hold_by_moving_towards_the_cr():
    """The defect: without the constraint, a drone held before the CR moves forward from a REAR threat."""
    gc = _gc(G.stage2_hold_s)
    ep = EscapePlanner(CFG)
    w = ep.warning_velocity(np.zeros(3), [(frozenset({"REAR"}), ENV.v_open)], [], YAW, ENV.v_max_nominal)
    e = EscapePlanner(CFG).escape_velocity([(frozenset({"REAR"}), 0.0)], [], YAW, ENV.v_escape)
    for ch in (w, e):
        assert ch.v_body[0] > 0.2 and float(gc.margins_body(ch.v_body, R)[0]) <= 0.0


@pytest.mark.parametrize("pattern", [{"REAR"}, {"REAR", "LEFT"}, {"REAR", "RIGHT"}, {"REAR", "UP"}, {"REAR", "DOWN"}])
def test_a_threat_from_behind_at_the_hold_is_answered_by_a_p2_safe_command(pattern):
    gc = _gc(G.stage2_hold_s)
    for ch in (_warn(gc, [pattern]), _escape(gc, [pattern])):
        j = ch.joint
        assert j is not None and j.p2_margin > 0.0
        assert float(gc.margins_body(ch.v_body, R)[0]) > 0.0 or ch.label == "HOLD"
        assert j.status in (gs.FEASIBLE, gs.UNCERTAIN)
        if j.status == gs.UNCERTAIN:
            assert j.share == "NON_CLOSING" and "P1" in j.uncertified and j.opening >= -1e-9 and "S1g" in j.reason


def test_a_persistent_rear_threat_never_moves_the_true_centre_into_the_cr():
    """A REAR threat that stays (the threat does not open): the warning filter and the escape keep choosing UNCERTAIN,
    non-closing, P2-safe commands - a creep towards the CR while it stays P2-safe (the largest worst-case opening, the
    fallback rule), then station keeping - and the TRUE centre stays out of the CR with the declared worst navigation
    error (truth 1 m ahead of the estimate) and the residual drift towards the CR."""
    for avoid in ("warning", "escape"):
        ep = EscapePlanner(CFG)
        bias = np.array([-G.nav_err_m, 0.0, 0.0])
        s_true, v = G.stage2_hold_s + G.nav_err_m, np.zeros(3)
        anchor = None
        for k in range(600):
            gc = _gc(s_true + bias[0], v_gate=tuple(v))
            if anchor is None:
                anchor = gc.q.copy()
            hold = (R.T @ gc.hold_velocity(anchor), gc.hold_margin(anchor))
            if avoid == "warning":
                ch = ep.warning_velocity(np.zeros(3), [(frozenset({"REAR"}), ENV.v_open)], [], YAW, ENV.v_max_nominal,
                                         p2=_p2(gc), hold=hold)
            else:
                ch = ep.escape_velocity([(frozenset({"REAR"}), 0.0)], [], YAW, ENV.v_escape, p2=_p2(gc), hold=hold)
            if ch.label != "HOLD":
                anchor = None
            assert ch.joint.status == gs.UNCERTAIN and ch.joint.opening >= -1e-9 and ch.joint.p2_margin > 0.0
            cmd = R @ ch.v_body
            u = float(cmd @ np.asarray(GATE.axis, float))
            v = np.array([v[0] + float(np.clip(u - v[0], -ENV.a_brake * 0.1, ENV.a_brake * 0.1)), 0.0, 0.0])
            s_true += (v[0] + ENV.w_drift_max) * 0.1
            assert s_true < G.cr_s_min, (avoid, k, s_true)


def test_an_occupant_ahead_is_escaped_backwards_jointly_feasible():
    gc = _gc(G.stage2_hold_s)
    ep = EscapePlanner(CFG)
    for ch in (_warn(gc, [{"FRONT"}], planner=ep), _escape(gc, [{"FRONT"}])):
        assert ch.joint.status == gs.FEASIBLE and ch.joint.share == "FULL"
        assert ch.v_body[0] < -0.1 and float(gc.margins_body(ch.v_body, R)[0]) > 0.0
    w = _warn(gc, [{"FRONT"}])
    assert _opening(ep, [{"FRONT"}], w.v_body) >= ENV.v_open - 0.05


def test_degraded_observations_downgrade_a_full_share_to_uncertain():
    gc = _gc(G.stage2_hold_s)
    th = [(frozenset({"FRONT"}), ENV.v_open)]
    ch = EscapePlanner(CFG).warning_velocity(np.zeros(3), th, [], YAW, ENV.v_max_nominal, p2=_p2(gc), degraded=True,
                                             hold=_hold(gc))
    assert ch.joint.status == gs.UNCERTAIN and "degraded" in ch.joint.reason and ch.v_body[0] < 0.0
    for uh in (["LEFT"], ["UP"], ["REAR"]):
        ch = _warn(gc, [{"FRONT"}], unhealthy=uh)
        assert ch.joint.status != gs.FEASIBLE and "P1" in ch.joint.uncertified and ch.joint.p2_margin > 0.0


def test_outside_the_invariant_the_decision_is_jointly_infeasible_and_says_why():
    gc = _gc(-1.0)
    assert float(gc.margins_world(np.zeros(3))[0]) <= 0.0
    for ch in (_warn(gc, [{"REAR"}]), _escape(gc, [{"REAR"}])):
        j = ch.joint
        assert j.status == gs.INFEASIBLE and "P2" in j.uncertified and j.reason
        assert j.opening >= -1e-9


def test_jointly_feasible_whenever_a_p2_safe_full_share_exists_on_a_grid():
    """Over positions before the CR, current velocities and single-threat patterns: the class equals gs.classify on the
    same candidates, the chosen command is P2-safe whenever one is, and a FEASIBLE command has the full share."""
    rng = np.random.default_rng(3)
    ep = EscapePlanner(CFG)
    pats = [frozenset(p) for p in ep.regions]
    for _ in range(120):
        s = float(rng.uniform(-4.5, -1.6))
        gc = _gc(s, float(rng.uniform(-0.5, 0.5)), float(rng.uniform(-0.5, 0.5)),
                 (float(rng.uniform(-0.3, 0.5)), float(rng.uniform(-0.2, 0.2)), 0.0))
        pat = pats[int(rng.integers(len(pats)))]
        ch = EscapePlanner(CFG).warning_velocity(np.zeros(3), [(pat, ENV.v_open)], [], YAW, ENV.v_max_nominal, p2=_p2(gc))
        speeds = [0.15, 0.25, 0.35, ENV.v_max_nominal]
        V = np.concatenate([sp * ep.cands for sp in speeds] + [np.zeros((1, 3))])
        g = ep.guarantee(pat)
        op = np.concatenate([sp * g for sp in speeds] + [np.zeros(1)])
        m = gc.margins_body(V, R)
        assert ch.joint.status == gs.classify(op, m, ENV.v_open)
        if (m > 0).any():
            assert ch.joint.p2_margin > 0.0
        if ch.joint.status == gs.FEASIBLE:
            assert _opening(ep, [pat], ch.v_body) >= ENV.v_open - 1e-6


def test_the_warning_choice_is_kept_within_the_safe_set_by_the_hysteresis():
    gc = _gc(G.stage2_hold_s)
    ep = EscapePlanner(CFG)
    first = _warn(gc, [{"FRONT"}], v_des=np.array([0.0, 0.05, 0.0]), planner=ep)
    for k in range(10):
        ch = _warn(gc, [{"FRONT"}], v_des=np.array([0.0, 0.05 - 0.01 * k, 0.0]), planner=ep)
        assert ch.label == first.label and float(gc.margins_body(ch.v_body, R)[0]) > 0.0


def test_far_from_the_gate_or_admitted_there_is_no_constraint():
    assert gs.constraint_for(CFG, GATE, GATE.from_gate_frame([-20.0, 0.0, 0.0]), np.zeros(3), authorized=False) is None
    assert gs.constraint_for(CFG, GATE, GATE.from_gate_frame([-2.0, 0.0, 0.0]), np.zeros(3), authorized=True) is None
    assert gs.constraint_for(CFG, None, np.zeros(3), np.zeros(3), authorized=False) is None


def test_without_a_constraint_the_planner_is_the_v11_planner_on_stored_inputs():
    """P1 regression: the p2=None path (every drone away from the CR, and an admitted one) returns exactly what the v11
    planner (commit 0e85d2d) returned on 300 random inputs, hysteresis included (tests/fixtures, generated once from the
    repository history: no git needed here)."""
    import json

    ref = json.loads((Path(__file__).resolve().parent / "fixtures" / "escape_v11_reference.json").read_text())
    new = EscapePlanner(CFG)
    assert len(ref["cases"]) == 300
    for c in ref["cases"]:
        th = [(frozenset(p.split("+")), r) for p, r in c["threats"]]
        a = new.warning_velocity(np.array(c["v_des"]), th, c["unhealthy"], c["yaw"], ENV.v_max_nominal)
        assert np.allclose(a.v_body, c["warning"]["v"]) and a.label == c["warning"]["label"]
        assert a.feasible == c["warning"]["feasible"]
        b = new.escape_velocity(th, c["unhealthy"], c["yaw"], ENV.v_escape, np.array(c["current"]), np.array(c["mission"]),
                                set(c["occupied"]), tuple(c["room"]))
        assert np.allclose(b.v_body, c["escape"]["v"]) and b.label == c["escape"]["label"]
        assert b.feasible == c["escape"]["feasible"]


def test_an_admissible_command_leaves_braking_p2_safe_at_the_next_step():
    """The robust margin: every command the constraint accepts (positive margin) leaves braking P2-safe one control
    period later from the worst-case state (position and velocity on the envelope after dt).  A one-step margin alone
    accepts commands that ride the boundary and leave no P2-safe brake behind them."""
    b = gs.P2Bounds.from_cfg(CFG)
    lo, hi = G.cr_lo(), G.cr_hi()
    a = np.asarray(GATE.axis, float)
    checked = 0
    near = np.arange(gs.hold_s(CFG) - 0.03, gs.hold_s(CFG) + 0.002, 0.002)  # the one-step boundary of a hold
    for s in np.concatenate([np.arange(G.cr_center_s - 3.2, G.cr_center_s - 2.05, 0.02), near]):
        gc = _gc(float(s))
        for u in np.arange(0.0, 0.55, 0.02):
            v = u * a
            if float(gc.margins_world(v)[0]) <= 0.0:
                continue
            checked += 1
            # worst case after dt towards the CR: the command reached at once, plus the drift; then a zero command
            q1 = gc.q + np.array([(u + b.w) * b.dt, 0.0, 0.0])
            m = float(gs.cr_margin(q1, np.array([u, 0.0, 0.0]), np.zeros((1, 3)), b, lo, hi)[0])
            assert m > 0.0, (s, u, m)
    assert checked > 50

# -- the order of DroneController.step, with the real flows and the controller's own record of the step --------------
def _target(pattern, d):
    return NS(cls="DYNAMIC", pattern=frozenset(pattern), d_lower=d, r_min=d + 0.3, closing_rate=0.0,
              possible_neighbour=False)


def _obs(s, targets, t=10.0, loc_ok=True, entry_ok=False):
    p = GATE.from_gate_frame([s, 0.0, 0.0])
    go = NS(gate_id=GATE.gate_id, s=s, l=0.0, dz=0.0, queue_s=PLAN.queue_s, queue_l=PLAN.queue_lateral,
            queue_dz=PLAN.queue_dz, loc_ok=loc_ok, entry_ok=entry_ok, passed=False)
    return NS(t=t, p=np.asarray(p, float), v_world=np.zeros(3), R_wb=R, yaw=YAW, targets=list(targets), unhealthy=[],
              gate=go, gate_index=0)


@pytest.mark.parametrize("mode", [Mode.MUTEX_PASS, Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE,
                                  Mode.FAILSAFE_HOLD_OR_RETREAT])
def test_the_final_command_of_a_not_admitted_drone_at_the_hold_is_p2_safe_in_every_mode(mode):
    ctrl = DroneController(PLAN, CFG)
    fl = ctrl.flows
    targets = [_target({"REAR"}, 2.0 if mode == Mode.SEPARATION_WARNING else 1.5)]
    obs = _obs(G.stage2_hold_s, targets)
    for k in range(3):
        obs.t = 10.0 + 0.1 * k
        v_mis, _ = fl.gate(obs, "MUTEX_PASS")
        gc = fl.gate_constraint(obs, authorized=False)
        assert gc is not None
        mf = gc.filter_mission(v_mis, ENV.v_max_nominal)
        v_mis = mf.v
        if mode == Mode.SEPARATION_WARNING:
            v = fl.separation_warning(obs, v_mis, gc)
        elif mode == Mode.COLLISION_AVOIDANCE:
            v = fl.collision_avoidance(obs, np.zeros(3), v_mis, gc)
        elif mode == Mode.FAILSAFE_HOLD_OR_RETREAT:
            fl.last_choice = None
            mf = gc.filter_mission(fl.failsafe(obs), ENV.v_max_nominal)
            v = mf.v
        else:
            fl.last_choice = None
            v = v_mis
        v_pre = np.asarray(v, float)
        v = fl.structure_filter(obs, v)
        ctrl._gate_safety_step(obs.t, mode, gc, mf, v_pre, v)
        rec = ctrl.gate_safety
        assert rec["margin"] > 0.0 and "guard" not in rec
        if mode in (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE):
            assert rec["joint"]["status"] == gs.UNCERTAIN and rec["joint"]["p2_margin"] > 0.0
    if mode in (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE):
        assert sum(e["type"] == "JOINT_SAFETY" for e in ctrl.events) == 1


def test_station_keeping_does_not_accumulate_the_drift_that_a_zero_command_lets_through():
    """The HOLD candidate is a position loop on a latched anchor: against the declared residual drift its margin stays
    positive, while repeated zero-velocity commands let the drone drift into the CR (found by formal/check_f1p_kinematic)."""
    s0 = G.stage2_hold_s
    gc = _gc(s0)
    anchor = gc.q.copy()
    w = ENV.w_drift_max
    s, v = s0, 0.0
    for _ in range(600):
        gc_k = _gc(s, v_gate=(v, 0.0, 0.0))
        cmd = float(gc_k.hold_velocity(anchor) @ np.asarray(GATE.axis, float))
        v = v + float(np.clip(cmd - v, -ENV.a_brake * 0.1, ENV.a_brake * 0.1))
        s += (v + w) * 0.1
        assert gc_k.hold_margin(anchor) > 0.0
    assert s < G.cr_s_min - G.nav_err_m
    drift_only = s0 + 600 * 0.1 * w
    assert drift_only > G.cr_s_min


def test_the_hold_candidate_moving_faster_than_the_drift_is_rated_by_its_direction():
    ep = EscapePlanner(CFG)
    g = ep.guarantee(frozenset({"REAR"}))
    back = np.array([-1.0, 0.0, 0.0])
    assert ep._hold_opening(0.9 * ENV.w_drift_max * back, g) == 0.0
    assert ep._hold_opening(0.25 * back, g) < 0.0


def test_a_drone_converging_on_the_hold_point_does_not_anchor_its_station_keeping_there():
    """Two drones committed together converge on the one hold point: each station-keeps where it is, not on the shared
    point (anchoring both there made them close on each other, formal/check_f1p_kinematic); a drone already holding at
    the point (within e_track) keeps it as its anchor."""
    fl = Flows(PLAN, CFG)
    fl.hold_target = np.array([G.stage2_hold_s, 0.0, 0.0])
    far = _gc(G.stage2_hold_s - 0.5, 1.1)
    fl._hold(far, R)
    assert np.allclose(fl.p2_anchor, far.q)
    fl.p2_anchor = None
    near = _gc(G.stage2_hold_s + 0.2, 0.6)
    fl._hold(near, R)
    assert fl.p2_anchor[0] == pytest.approx(G.stage2_hold_s) and fl.p2_anchor[1] == pytest.approx(near.q[1])
    assert near.hold_margin(fl.p2_anchor) > 0.0 and float(near.margins_world(np.zeros(3))[0]) <= 0.0
    assert float(np.linalg.norm(near.hold_velocity(fl.p2_anchor))) <= ENV.w_drift_max + 1e-9


# -- review items: the proved selection where the CR does not bind, selection among admissible candidates only ------
def test_where_the_cr_removes_no_candidate_the_planner_is_the_proved_p1_selection():
    gc = _gc(-6.0, 0.0, 0.0)
    for pats in ([{"FRONT"}], [{"REAR"}], [{"LEFT"}, {"FRONT"}]):
        a = EscapePlanner(CFG).warning_velocity(np.zeros(3), [(frozenset(p), ENV.v_open) for p in pats], [], YAW,
                                                ENV.v_max_nominal, p2=_p2(gc), hold=_hold(gc))
        b = EscapePlanner(CFG).warning_velocity(np.zeros(3), [(frozenset(p), ENV.v_open) for p in pats], [], YAW,
                                                ENV.v_max_nominal)
        assert np.allclose(a.v_body, b.v_body) and a.label == b.label and a.joint.n_p2_rejected == 0
        e1 = EscapePlanner(CFG).escape_velocity([(frozenset(p), 0.0) for p in pats], [], YAW, ENV.v_escape, p2=_p2(gc),
                                                hold=_hold(gc))
        e2 = EscapePlanner(CFG).escape_velocity([(frozenset(p), 0.0) for p in pats], [], YAW, ENV.v_escape)
        assert np.allclose(e1.v_body, e2.v_body) and e1.label == e2.label
        assert "removes no candidate" in e1.joint.reason


def test_the_escape_picks_only_an_admissible_candidate_when_station_keeping_is_the_only_one():
    """Only the HOLD is P2-safe and (speed 0) the full share is 0: the HOLD, never index 0 of the grid."""
    ch = EscapePlanner(CFG).escape_velocity([(frozenset({"REAR"}), 0.0)], [], YAW, 0.0,
                                            p2=lambda V: np.full(len(V), -1.0), hold=(np.zeros(3), 1.0))
    assert ch.label == "HOLD" and ch.joint.p2_margin > 0.0


def test_an_escape_without_threats_is_still_rated_for_p2():
    gc = _gc(-1.9, v_gate=(0.3, 0.0, 0.0))
    ch = EscapePlanner(CFG).escape_velocity([], [], YAW, ENV.v_escape, p2=_p2(gc), hold=_hold(gc))
    assert not (ch.joint.status == gs.FEASIBLE and ch.joint.p2_margin <= 0.0)
    assert ch.joint.status == gs.INFEASIBLE and "P2" in ch.joint.uncertified


def test_degraded_only_when_an_unhealthy_sector_changes_a_threat_guarantee():
    ep = EscapePlanner(CFG)
    th = [(frozenset({"FRONT"}), ENV.v_open)]
    changes = [s for s in ("LEFT", "RIGHT", "UP", "DOWN", "REAR")
               if not np.array_equal(ep.guarantee(frozenset({"FRONT"}), [s]), ep.guarantee(frozenset({"FRONT"})))]
    assert ep._degraded(th, changes[:1]) and not ep._degraded(th, [])


def test_the_mission_filter_accepts_station_keeping_only_for_the_hold_loop_itself():
    gc = _gc(G.stage2_hold_s + 0.2)
    anchor = np.array([G.stage2_hold_s, 0.0, 0.0])
    loop = gc.from_gate(G.k_track * (anchor - gc.q))
    assert float(gc.margins_world(loop)[0]) <= 0.0
    f = gc.filter_mission(loop, ENV.v_max_nominal, anchor=anchor, gain=G.k_track)
    assert not f.modified and f.margin > 0.0 and "station keeping" in f.reason
    fwd = gc.from_gate(np.array([0.3, 0.0, 0.0]))
    f = gc.filter_mission(fwd, ENV.v_max_nominal, anchor=anchor, gain=G.k_track)
    assert f.modified and float(f.v @ np.asarray(GATE.axis, float)) <= 1e-9


def test_a_slower_dvl_or_an_outage_only_shrinks_the_p2_margin():
    base = _gc(G.stage2_hold_s)
    noisy = gs.constraint_for(CFG, GATE, GATE.from_gate_frame([G.stage2_hold_s, 0.0, 0.0]), np.zeros(3), False, v_err=0.1)
    out = gs.constraint_for(CFG, GATE, GATE.from_gate_frame([G.stage2_hold_s, 0.0, 0.0]), np.zeros(3), False, v_err=0.7)
    V = np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [-0.3, 0.2, 0.0]])
    a, b, c = base.margins_world(V), noisy.margins_world(V), out.margins_world(V)
    assert np.all(b <= a + 1e-12) and np.all(c <= b + 1e-12) and float(b[0]) > 0.0


def test_the_cr_column_forbids_passing_under_or_over_the_cr():
    under = gs.constraint_for(CFG, GATE, GATE.from_gate_frame([0.0, 0.0, -2.5]), np.zeros(3), False)
    assert float(under.margins_world(np.zeros(3))[0]) < 0.0
    beside = gs.constraint_for(CFG, GATE, GATE.from_gate_frame([0.0, 3.5, 0.0]), np.zeros(3), False)
    assert float(beside.margins_world(np.zeros(3))[0]) > 0.0


def test_a_structure_edit_that_makes_the_applied_command_unsafe_is_guarded_and_logged():
    ctrl = DroneController(PLAN, CFG)
    obs = _obs(G.stage2_hold_s, [])
    gc = ctrl.flows.gate_constraint(obs, authorized=False)
    ctrl.flows.last_choice = None
    v_pre = np.zeros(3)
    v_bad = gc.from_gate(np.array([0.5, 0.0, 0.0]))
    ctrl._gate_safety_step(obs.t, Mode.MUTEX_PASS, gc, None, v_pre, v_bad, obs)
    rec = ctrl.gate_safety
    assert "guard" in rec and rec["margin"] >= 0.0 and any(e["type"] == "P2_GUARD" for e in ctrl.events)
    assert float(gc.margins_world(rec["_v_guard"])[0]) > 0.0


def test_a_cut_mission_command_becomes_station_keeping_and_the_drift_does_not_carry_the_drone_in():
    """An uncommitted drone whose mission leads along the axis into the CR, residual drift towards the CR: the P2
    filter replaces the cut command by station keeping on the point where the cut began (Flows.p2_filter latches it);
    repeated scaled-down velocity commands would let the drift accumulate (the recovery chatter of the review)."""
    fl = Flows(PLAN, CFG)
    s, v = -4.0, 0.0
    a = np.asarray(GATE.axis, float)
    recov = 0
    for _ in range(900):
        gc = _gc(s, v_gate=(v + ENV.w_drift_max, 0.0, 0.0))
        mf = fl.p2_filter(gc, 0.3 * a, {})
        recov += mf.status == gs.RECOVERY
        u = float(mf.v @ a)
        v = v + float(np.clip(u - v, -ENV.a_brake * 0.1, ENV.a_brake * 0.1))
        s += (v + ENV.w_drift_max) * 0.1
        assert s < G.cr_s_min - G.nav_err_m
    assert recov == 0 and fl.p2_station is not None


# -- integration: the real DroneController.step ------------------------------------------------------------------
from holo_fleet.perception.frame import SensorFrame  # noqa: E402
from holo_fleet.perception.sonar_geometry import SECTORS  # noqa: E402
from holo_fleet.perception.sonar_geometry import sonar_sensor_name  # noqa: E402


def _frame(t, dvl=True):
    """Minimal sensor frame (compass, depth, IMU, empty sonar profiles; the DVL only when ``dvl``)."""
    data = {"Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-4.3]), "IMUSensor": np.zeros((4, 3))}
    if dvl:
        data["DVLSensor"] = np.zeros(7)
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(234)
    return SensorFrame(t, data, {k: t for k in data})


def _scripted_perception(ctrl, place):
    """The real perception runs on every frame; its observation is then placed by ``place(obs, t)`` (the only stand-in:
    the drone's gate-frame situation and the sonar target, which synthetic profiles of a moving hull would only
    reproduce less directly).  Everything downstream - automaton, flows, P2 filter, planner, guard, record - is the
    deployed DroneController.step."""
    real = ctrl.perception.update

    def update(*a, **k):
        obs = real(*a, **k)
        place(obs, obs.t)
        return obs

    ctrl.perception.update = update


def _place_at(obs, s, targets, commit_phase, drift=ENV.w_drift_max):
    obs.p = np.asarray(GATE.from_gate_frame([s, 0.0, 0.0]), float)
    obs.yaw, obs.R_wb = YAW, R
    obs.v_world = drift * np.asarray(GATE.axis, float)
    go = obs.gate
    go.gate_id, go.s, go.l, go.dz = GATE.gate_id, s, 0.0, 0.0
    go.queue_s, go.queue_l, go.queue_dz = PLAN.queue_s, PLAN.queue_lateral, PLAN.queue_dz
    go.in_zone, go.passed, go.loc_ok, go.entry_ok = True, False, True, False
    go.at_queue = go.has_prio = commit_phase
    go.occ_busy = False
    obs.gate_index = 0
    obs.targets = list(targets)
    obs.d_min = min([tg.d_lower for tg in targets] + [1e9])
    ab = obs.ab
    ab.d_min, ab.mutex_zone, ab.at_queue, ab.has_prio, ab.occ_busy, ab.passed = obs.d_min, True, commit_phase, commit_phase, False, False


def test_the_real_step_keeps_a_held_drone_out_of_the_cr_against_a_threat_from_behind():
    """Committed, not admitted, at the stage-2 hold, a REAR threat in the warning band and a forward drift: every applied
    velocity request of DroneController.step has a positive P2 margin, never moves towards the CR beyond a creep, and the
    record reports the joint-feasibility class (fails if step() stops calling the gate-safety layer)."""
    ctrl = DroneController(PLAN, CFG)
    rear = _target({"REAR"}, 2.0)
    _scripted_perception(ctrl, lambda obs, t: _place_at(obs, PLAN.queue_s if t < 0.35 else G.stage2_hold_s,
                                                        [] if t < 0.35 else [rear], t < 0.35))
    for k in range(4):
        ctrl.step(_frame(0.1 * k), 0.1)
    assert ctrl.ha.committed
    seen = set()
    for k in range(4, 60):
        ctrl.step(_frame(0.1 * k), 0.1)
        rec = ctrl.last_record["gate_safety"]
        assert rec is not None and rec["margin"] > 0.0
        assert float(ctrl.v_requested @ np.asarray(GATE.axis, float)) <= 0.1
        if ctrl.ha.mode in (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE):
            seen.add(rec["joint"]["applied_status"])
            assert rec["joint"]["p2_margin"] > 0.0
    assert ctrl.ha.mode == Mode.SEPARATION_WARNING and seen and seen <= {gs.FEASIBLE, gs.UNCERTAIN}
    assert any(e["type"] == "JOINT_SAFETY" for e in ctrl.events)


def test_the_real_step_in_failsafe_holds_on_the_failsafe_anchor_with_the_outage_velocity_error():
    """A DVL outage at the hold: sense_ok fails and the automaton holds (FAILSAFE); the P2 filter is given the failsafe hold
    law (mission_anchor with failsafe=True) and the velocity error of an outage (the speed bound of NavBound)."""
    ctrl = DroneController(PLAN, CFG)
    calls = []
    real_anchor = ctrl.flows.mission_anchor

    def spy(gc, failsafe=False, probe=False):
        calls.append(failsafe)
        return real_anchor(gc, failsafe=failsafe, probe=probe)

    ctrl.flows.mission_anchor = spy
    _scripted_perception(ctrl, lambda obs, t: _place_at(obs, PLAN.queue_s if t < 0.35 else G.stage2_hold_s, [], t < 0.35))
    for k in range(4):
        ctrl.step(_frame(0.1 * k), 0.1)
    assert ctrl.ha.committed
    ctrl.step(_frame(0.4), 0.1)
    rec = ctrl.last_record["gate_safety"]
    assert rec["v_err"] == pytest.approx(ctrl.perception.nav_bound.k_rw, abs=1e-3)
    for k in range(5, 25):
        ctrl.step(_frame(0.1 * k, dvl=False), 0.1)
        rec = ctrl.last_record["gate_safety"]
        assert float(ctrl.v_requested @ np.asarray(GATE.axis, float)) <= 1e-9
    assert ctrl.ha.mode == Mode.FAILSAFE_HOLD_OR_RETREAT and True in calls
    assert rec["v_err"] == pytest.approx(ctrl.perception.nav_bound.v_bound, abs=1e-3)
    # with the velocity unknown up to the speed bound the hold is not certified: the filter retreats (P2_RECOVERY)
    assert rec["mission"]["status"] == gs.RECOVERY



@pytest.mark.parametrize("dist,constrained", [(9.0, False), (3.0, True)])
def test_the_real_step_engages_the_p2_layer_only_within_its_active_range(dist, constrained):
    """A committed drone on the gate axis, ``dist`` metres before the CR box: at 9 m (beyond ACTIVE_RANGE_M) the step
    applies the command of the flows unchanged and records no constraint; at 3 m the constraint is recorded."""
    assert 3.0 < gs.ACTIVE_RANGE_M < 9.0
    ctrl = DroneController(PLAN, CFG)
    s0 = G.cr_s_min - dist
    _scripted_perception(ctrl, lambda obs, t: _place_at(obs, PLAN.queue_s if t < 0.35 else s0, [], t < 0.35, drift=0.0))
    for k in range(8):
        ctrl.step(_frame(0.1 * k), 0.1)
    rec = ctrl.last_record
    assert (rec["gate_safety"] is not None) == constrained
    if not constrained:
        assert np.allclose(rec["v_cmd"], rec["v_mis"], atol=1e-3)
