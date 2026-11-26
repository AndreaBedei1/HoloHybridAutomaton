"""Stage-2 hold before the critical region (CR) under navigation error (control/flows.py Flows.gate,
control/gate_safety.py).

The earlier hold sat at s = -1.18 m, 0.38 m from the CR (the former symmetric CR |s| <= 0.8; the same rule gives -1.43 m with
the CR s in [-1.05, 0.55] of that time), with a declared gate-frame navigation error of 1 m: a drone held there on an
estimate 1 m short of the truth had its centre inside the CR.  The hold is now the nearest point where
holding is P2-safe under the declared error, minus the measured holding error, and the queue lines keep the merge
clearance from it.  The closed loops use formal/check_f1p_kinematic.World: the deployed Flows.gate, keep-out and P2
filters in the order of DroneController.step, a velocity loop limited by a_brake and the declared residual drift towards
the CR; the localization tests use the deployed StructureAiding on the synthetic bar echoes of tests/test_structure_aiding.
"""

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control import gate_safety as gs
from holo_fleet.control.flows import ACQ_OFFSETS, ACQ_PERIOD_S
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_geometry import HULL_HALF, R_OUT
from holo_fleet.perception.sonar_processing import EchoClassifier
from holo_fleet.perception.structure_aiding import StructureAiding

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
import check_f1p_kinematic as K  # noqa: E402
from test_structure_aiding import BARS, GATES, _R, _profiles, _world  # noqa: E402

CFG = DEFAULT
G, ENV = CFG.gate, CFG.env
NEAR_HOLD = G.cr_s_min - (float(HULL_HALF[0]) + 0.15)
DRIFT = (ENV.w_drift_max, 0.0, 0.0)


def test_the_hold_is_the_derived_p2_safe_hold():
    b = gs.P2Bounds.from_cfg(CFG)
    assert b.e_track == pytest.approx(ENV.w_drift_max / 0.5) and b.tau_c == pytest.approx(ENV.dt + ENV.tau_max)
    boundary = G.cr_s_min - (G.nav_err_m + ENV.w_drift_max * b.tau_c + b.e_track + gs.SLACK_M)
    assert boundary == pytest.approx(-(0.8 + 1.0 + 0.0525 + 0.30 + 0.05), abs=1e-9)
    assert gs.hold_s(CFG) == pytest.approx(boundary, abs=1e-6)
    assert G.stage2_hold_s <= gs.hold_s(CFG) - G.hold_err_m
    gate = K.make_gate()

    def margin(s):
        return float(gs.constraint_for(CFG, gate, gate.from_gate_frame([s, 0.0, 0.0]), np.zeros(3), False)
                     .margins_world(np.zeros(3))[0])

    assert margin(G.stage2_hold_s) > 0.0 and margin(gs.hold_s(CFG) + 0.01) < 0.0
    assert margin(NEAR_HOLD) < -0.9
    # the station-keeping certificate at rest (gate_safety.sk_band, the recursive argument) has the same band: holding on
    # the target at rest is certified with exactly the measured holding error as margin, also with the DVL error bound
    assert gs.hold_reach(b) == pytest.approx(gs.sk_hold(b), abs=1e-12)
    from holo_fleet.perception.nav import NavBound

    for v_err in (0.0, float(NavBound(CFG).k_rw)):
        gc = gs.constraint_for(CFG, gate, gate.from_gate_frame([G.stage2_hold_s, 0.0, 0.0]), np.zeros(3), False, v_err=v_err)
        assert gc.hold_margin(gc.q) == pytest.approx(gs.hold_s(CFG) - G.stage2_hold_s, abs=1e-9)


@pytest.mark.parametrize("n", [1, 2, 3, 4])
def test_the_queue_lines_keep_the_merge_clearance_from_the_hold(n):
    """Bound (d): no queue point ahead of the stage-2 hold; bound (c), two or more columns: every queue point keeps the
    merge clearance from the hold, within the lateral allowance of the (vertical) acquisition offsets.  With the near hold
    point (NEAR_HOLD) the n = 2 line violates (c) at the stage-2 hold."""
    qs = G.queue_s(n)
    assert qs <= G.stage2_hold_s + 1e-9
    lats = G.queue_laterals(n)
    if n >= 2:
        for lat in lats:
            dl = max(abs(lat) - G.hold_lateral_allow, 0.0)
            assert math.hypot(qs - G.stage2_hold_s, dl) >= G.merge_clearance - 1e-9
    if n == 2:
        near = dataclasses.replace(G, stage2_hold_s=NEAR_HOLD)
        dl = max(abs(lats[0]) - G.hold_lateral_allow, 0.0)
        assert math.hypot(near.queue_s(2) - G.stage2_hold_s, dl) < G.merge_clearance


def _holder(bias, variant="CORRECTED", loc_ok=True, entry_ok=False, drift=DRIFT, horizon=40.0, trace=True):
    """One committed drone of the abreast queue of two, held by the stage-2 check, with an estimate - truth bias."""
    cfg = CFG if variant == "CORRECTED" else K.near_hold_config(CFG)
    gate = K.make_gate()
    qs, lanes = cfg.gate.queue_s(2), cfg.gate.queue_laterals(2)
    b = np.asarray(bias, float)
    q = (qs, lanes[1], 0.0)
    a = K.Agent("B", "GATE", gate.from_gate_frame(q) - b, bias=b, queue=q, commit_t=0.0)
    w = K.World(CFG, gate, [a], variant, drift=drift, avoidance=False, loc_ok=loc_ok, entry_ok=entry_ok, trace=trace)
    w.run(horizon)
    return w, a, gate


BIASES = [(-1.0, 0.0), (-0.5, 0.0), (0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (-0.8, 0.6), (-0.8, -0.6), (0.6, 0.8)]


DRIFTS = [DRIFT, (0.0, ENV.w_drift_max, 0.0), (ENV.w_drift_max / math.sqrt(2), -ENV.w_drift_max / math.sqrt(2), 0.0)]


def test_the_lateral_allowance_of_the_queue_bound_covers_the_lateral_deviation_of_a_hold():
    """The drone at the hold stays within hold_lateral_allow of the axis (the queue bound (c) assumes it), also under the
    declared lateral drift, where the steady deviation is w_drift_max / k_track (0.25 m): the covers it with 0.02 m to spare."""
    for drift, inside in ((DRIFT, True), ((0.0, ENV.w_drift_max, 0.0), True)):
        w, a, gate = _holder((0.0, 0.0, 0.0), drift=drift)
        held = [abs(float(gate.to_gate_frame(np.asarray(r["agents"]["B"]["est"]))[1])) for r in w.rows
                if r["agents"]["B"]["entry"] == "HOLD" and r["t"] > 20.0]
        assert held
        assert (max(held) <= G.hold_lateral_allow) == inside
        assert max(held) <= ENV.w_drift_max / G.k_track + 0.02 <= G.hold_lateral_allow + 1e-9


@pytest.mark.slow
@pytest.mark.parametrize("drift", DRIFTS[1:])
@pytest.mark.parametrize("bs,bl", [(-1.0, 0.0), (-0.8, 0.6), (0.5, 0.0)])
def test_a_held_drone_never_enters_the_cr_with_lateral_drift(bs, bl, drift):
    w, a, gate = _holder((bs, bl, 0.0), drift=drift)
    assert not w.entries and not any(r["cr"] for r in w.rows)


@pytest.mark.slow
@pytest.mark.parametrize("bs", [-1.0, -0.5, 0.5, 1.0])
def test_a_held_drone_with_a_drone_approaching_from_behind_never_enters_the_cr(bs):
    """Closed loop with the separation layer on: a committed drone held by the stage-2 check (navigation bias ``bs``),
    a third drone approaching it from behind along the axis (uncommitted, the deployed avoidance), drift towards the CR:
    the held drone's TRUE centre never enters the CR (with the NEAR_HOLD rule the avoidance moves it in)."""
    gate = K.make_gate()
    qs, lanes = G.queue_s(2), G.queue_laterals(2)
    b = np.array([bs, 0.0, 0.0])
    q = (qs, lanes[1], 0.0)
    held = K.Agent("B", "GATE", gate.from_gate_frame(q) - b, bias=b, queue=q, commit_t=0.0)
    rear = K.Agent("C", "TRANSIT", gate.from_gate_frame([G.stage2_hold_s - 8.0, 0.0, 0.0]),
                   goal=gate.from_gate_frame([G.rally_s + 6.0, 0.0, 0.0]), speed=0.3, queue=(qs, 0.0, 0.0))
    w = K.World(CFG, gate, [held, rear], "CORRECTED", drift=DRIFT, avoidance=True, entry_ok=False)
    w.run(50.0)
    assert any(r for r in w.reasons) or w.joint[gs.FEASIBLE] + w.joint[gs.UNCERTAIN] > 0
    assert not [e for e in w.entries if e["drone"] == "B"] and w.double is None


@pytest.mark.slow
@pytest.mark.parametrize("bs,bl", BIASES)
def test_a_held_drone_never_enters_the_cr_whatever_the_declared_navigation_error(bs, bl):
    """Estimate - truth (bs along the axis, bl lateral, norm <= nav_err_m), residual drift towards the CR, avoidance
    disabled: the TRUE centre stays out of the CR while the drone is not admitted; logged per step: estimated and true
    position, the clearance of the hull's nose to the CR, the commanded velocity, the admission state, CR occupancy."""
    w, a, gate = _holder((bs, bl, 0.0))
    assert not w.entries and not any(r["cr"] for r in w.rows)
    log = [(r["t"], r["agents"]["B"]["est"], r["agents"]["B"]["true"], r["agents"]["B"]["v_cmd"], r["agents"]["B"]["admitted"],
            r["agents"]["B"]["entry"], r["cr"]) for r in w.rows]
    s_true = np.array([float(gate.to_gate_frame(np.asarray(x[2]))[0]) for x in log])
    nose = G.cr_s_min - (s_true + float(HULL_HALF[0]))
    assert not any(x[4] for x in log)
    assert float(s_true.max()) < G.cr_s_min
    assert {x[5] for x in log} >= {"HOLD"}
    if bs <= 0.0:
        assert float(nose.min()) > -float(HULL_HALF[0]) - 1e-9


def test_the_near_hold_point_puts_the_true_centre_in_the_cr_with_the_declared_error():
    """The reproduced defect (mutation): the controller held 0.379 m before the CR (NEAR_HOLD) on an estimate 1 m short of the
    truth."""
    w, a, gate = _holder((-1.0, 0.0, 0.0), variant="NEAR_HOLD")
    assert w.entries and w.entries[0]["drone"] == "B" and not w.entries[0]["by_avoidance"]
    assert any(r["cr"] == ["B"] and not r["agents"]["B"]["admitted"] for r in w.rows)


def _acquired(bias_s, bias_l=0.0, offsets=ACQ_OFFSETS, captures=20):
    """Visit the acquisition offsets at the hold (estimate) with the truth at estimate - bias; validated localization?"""
    clf = EchoClassifier(gate_bars=BARS, cfg=CFG)
    R = _R()
    for k, (dl, dz) in enumerate(offsets):
        aid = StructureAiding(clf, CFG, gates=GATES)
        p = _world(G.stage2_hold_s, dl, dz)
        truth = _world(G.stage2_hold_s - bias_s, dl - bias_l, dz)
        for n in range(captures):
            t = 0.1 * (n + 1)
            c = aid.update(p, R, _profiles(clf, truth, R), {s: 0.0 for s in sg.SECTORS}, 0.25, t=t)
            if c is not None:
                p = p + c
            if aid.loc_t > 0.0 and aid.valid(t):
                return k
    return None


def test_the_acquisition_pattern_fits_in_the_bounded_hold():
    assert len(ACQ_OFFSETS) * ACQ_PERIOD_S <= G.t_occ_max + G.t_clear
    for dl, dz in ACQ_OFFSETS:
        assert abs(dl) <= G.hold_lateral_allow and abs(dz) <= G.cr_half_height


@pytest.mark.parametrize("bias", [-1.0, -0.7, -0.5, -0.3, 0.0, 0.2, 0.3])
def test_the_localization_is_acquired_at_the_hold(bias):
    """The vertical acquisition offsets +-0.8 m (depth is measured directly, so they need no lateral room): validated
    localization from the stage-2 hold (-2.55 m) with the estimate up to 1 m behind the truth and at +0.2, +0.3 m ahead
    of it (synthetic B0 scan, bar echoes only).  Synthetic gaps at -0.1 and +0.1 m: from the hold the two matched sectors
    see horizontal bars only, the lateral offset is the tie-break of a flat cost valley and can exceed CONSIST_M (a
    progress limit; the six former offsets covered -0.1 m on the teleported scan but were not reached in HoloOcean)."""
    assert _acquired(bias) is not None


@pytest.mark.parametrize("bias", [-0.1, 0.1, 0.4, 0.5, 0.6, 0.7, 1.0])
def test_the_localization_is_not_acquired_at_the_hold(bias):
    """The B0 acquisition gap of the stage-2 hold at -2.55 m (0.25 m farther from the frame than the -2.30 m P2-safe hold of
    the present CR s in [-0.8, 0.0], which acquired at +0.5, +0.6 m too): not at +0.1 m, and
    not from +0.4 m (the truth about 3 m or more from the gate plane).  A progress limit, not a safety one."""
    assert _acquired(bias) is None


@pytest.mark.parametrize("bs,bl,offset", [(-0.5, -0.3, 0), (-0.5, 0.3, None), (-0.5, 0.6, None), (0.0, -0.3, 0), (0.0, -0.6, None),
                                           (0.0, 0.6, None), (0.3, -0.3, 0), (0.3, 0.3, 0), (0.3, -0.6, 0), (0.5, -0.3, None),
                                           (0.5, 0.3, None), (0.5, 0.6, None)])
def test_the_localization_is_acquired_with_a_lateral_error_and_by_which_offset(bs, bl, offset):
    """Along-axis and lateral estimate errors within the 1 m norm: the acquisition offset (index in ACQ_OFFSETS) that
    first validates the localization, None where the synthetic B0 scan does not acquire it."""
    assert bs * bs + bl * bl <= G.nav_err_m ** 2 + 1e-9
    assert _acquired(bs, bl) == offset


def test_beyond_the_acquisition_boundary_the_hold_is_bounded_and_ends_in_the_give_up():
    """The documented outcome where the localization is not acquired (B0, estimate 0.4 m or more ahead of the truth:
    the truth about 3 m or more from the gate plane, the side sonars do not see the bars; advancing without a validated
    localization is not P2-safe): the drone holds at most t_occ_max + t_clear, logs ENTRY_HOLD_TIMEOUT, requests the
    give-up behind the plane and never enters the CR.  A P2 / progress conflict, not a deadlock."""
    w, a, gate = _holder((1.0, 0.0, 0.0), loc_ok=False, entry_ok=True, horizon=45.0)
    ev = [e for e in a.flows.events if e["type"] == "ENTRY_HOLD_TIMEOUT"]
    assert ev and not ev[0]["loc_ok"] and a.gave_up >= 1
    assert not w.entries and not any(r["agents"]["B"]["admitted"] for r in w.rows)


def test_an_admitted_drone_passes_the_gate_and_the_p2_filter_releases_it():
    w, a, gate = _holder((0.0, 0.0, 0.0), entry_ok=None, horizon=60.0)
    adm = [r["t"] for r in w.rows if r["agents"]["B"]["admitted"]]
    assert adm and a.done and not w.entries
    s_adm = float(gate.to_gate_frame(np.asarray(next(r for r in w.rows if r["agents"]["B"]["admitted"])["agents"]["B"]["est"]))[0])
    assert G.stage2_hold_s - 0.4 - 0.05 <= s_adm <= G.stage2_hold_s + 0.1
    assert any(r["cr"] == ["B"] and r["agents"]["B"]["admitted"] for r in w.rows)


def test_without_localization_the_hold_times_out_and_retreats_behind_the_give_up_plane():
    w, a, gate = _holder((0.0, 0.0, 0.0), loc_ok=False, entry_ok=True, horizon=45.0)
    ev = [e for e in a.flows.events if e["type"] == "ENTRY_HOLD_TIMEOUT"]
    assert ev and not ev[0]["loc_ok"]
    hold_start = next(r["t"] for r in w.rows if r["agents"]["B"]["entry"] == "HOLD")
    assert ev[0]["t"] - hold_start == pytest.approx(G.t_occ_max + G.t_clear, abs=0.3)
    assert a.gave_up >= 1 and not w.entries and not any(r["agents"]["B"]["admitted"] for r in w.rows)
    s_abort = G.cr_s_min - (float(R_OUT) + G.nav_err_m)
    assert G.stage2_hold_s < s_abort
    lat = {tuple(np.round(r["agents"]["B"]["est"][1:], 1)) for r in w.rows if r["agents"]["B"]["entry"] == "HOLD"}
    assert len(lat) >= 3


def test_a_standoff_behind_a_stopped_drone_is_bounded_and_ends_in_the_give_up():
    """A DYNAMIC echo that stays in the FRONT sector of a committed drone far behind the hold (a stopped drone): the
    standoff holds on the point where it began and times out after t_occ_max + t_clear, requesting the give-up edge."""
    from types import SimpleNamespace as NS

    from holo_fleet.control.flows import Flows

    gate = K.make_gate()
    lanes = G.queue_laterals(2)
    plan = NS(gates=[gate], slots=[NS(lateral=lanes[0], dz=0.0, along=0.0)], slot_index=0,
              path=NS(depth_z=float(gate.center[2])), clock=None, structures=[])
    fl = Flows(plan, CFG)
    echo = NS(cls="DYNAMIC", pattern=frozenset({"FRONT"}), d_lower=2.5, r_min=2.8, closing_rate=0.0, possible_neighbour=False)
    qs = G.queue_s(2)
    targets = []
    for k in range(int((G.t_occ_max + G.t_clear + 2.0) / 0.1)):
        s = qs + 0.02 * min(k, 10)
        go = NS(gate_id="K", s=s, l=lanes[0], dz=0.0, queue_s=qs, queue_l=lanes[0], queue_dz=0.0, loc_ok=True,
                entry_ok=False, passed=False)
        obs = NS(t=0.1 * k, p=gate.from_gate_frame([s, lanes[0], 0.0]), targets=[echo], gate=go, gate_index=0)
        fl.gate(obs, "MUTEX_PASS")
        targets.append(None if fl.hold_target is None else float(fl.hold_target[0]))
        if fl.abort_request:
            break
    assert fl.standoff or fl.hold_timeout
    assert any(e["type"] == "ENTRY_HOLD_TIMEOUT" and e.get("standoff") for e in fl.events)
    assert fl.abort_request
    held = [x for x in targets[:-1] if x is not None]
    assert held and max(held) - min(held) < 1e-9
