"""The P2 recursive argument (control/gate_safety.py module docstring, formal/check_p2_recursive.py): a command certified by
its robust margin hands the next step a certificate for station keeping on the next estimate, a station keeping engaged
with a positive margin is latched (a later recomputed drop does not cut it), the hold law is restoring on every axis, the
certificates are dropped when their premises change, and an applied command without a certificate is logged.

The states are the witnesses of formal/check_p2_recursive.py: 0.5 m/s along the axis 2.0 m before the CR entry plane, whose
worst next state has no onboard-certified braking and no station keeping certified from the fresh measurement (the
counterexample of the braking lookahead, P2R2m); the stage-2 hold at the edge of its deviation band with the measured
velocity at the DVL error bound (the drop of the recomputed margin, P2B4).
"""

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control import gate_safety as gs
from holo_fleet.control.controller import DroneController
from holo_fleet.mission import GateSpec
from holo_fleet.perception.nav import NavBound
from holo_fleet.sim.scenarios import SCENARIOS

CFG = DEFAULT
G, ENV = CFG.gate, CFG.env
B = gs.P2Bounds.from_cfg(CFG)
NB = NavBound(CFG)
VE = float(NB.k_rw)
RHO = VE + float(NB.k_lin) * float(NB.v_bound)
GATE = GateSpec("SYNTH", np.array([0.0, 0.0, -5.0]), np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]), 2.0, 2.0)


def _gc(q, v0=(0.0, 0.0, 0.0), t=1.0, ve=VE, rho=RHO, gate=GATE):
    return gs.GateConstraint(gate, np.asarray(q, float), np.asarray(v0, float), B, G.cr_lo(), G.cr_hi(), v_err=ve, rho=rho,
                             t=t)


def _worst_next(q, v0, u):
    """The state after dt on the upper envelope towards +s, the next estimate rho dt further, measured +v_err."""
    xp, vp, _, _ = gs._after(np.asarray(v0, float)[None, :] + VE, np.asarray(u, float)[None, :], B)
    q1 = np.asarray(q, float) + np.array([xp[0, 0] + RHO * B.dt, 0.0, 0.0])
    return q1, np.array([vp[0, 0] + VE, 0.0, 0.0])


def test_a_robust_command_hands_the_next_step_a_certified_station_keeping():
    q, v0, u = np.array([G.cr_s_min - 2.0, 0.0, 0.0]), np.zeros(3), np.array([0.5, 0.0, 0.0])
    gc = _gc(q, v0)
    kept = gc.filter_mission(u, ENV.v_max_nominal)
    assert not kept.modified and kept.margin > 0.0
    cert = gc.certify(u, (), (), None)
    assert cert is not None and cert.kind == gs.NEXT and cert.margin > 0.0
    q1, vm1 = _worst_next(q, v0, u)
    g1 = _gc(q1, vm1, t=1.0 + B.dt)
    # the next step cannot certify anything from its fresh measurement: neither braking nor station keeping
    assert float(gs.cr_margin(q1, vm1, np.zeros((1, 3)), B, G.cr_lo(), G.cr_hi(), v_err=VE)[0]) <= 0.0
    assert g1.hold_margin(q1) <= 0.0
    assert g1.filter_mission(np.zeros(3), ENV.v_max_nominal).status == gs.RECOVERY
    # with the handed certificate station keeping where the drone is remains certified
    f = g1.filter_mission(u, ENV.v_max_nominal, cert=cert)
    assert f.status == gs.FEASIBLE and np.allclose(f.station, q1) and f.margin == pytest.approx(cert.margin)
    c1 = g1.certify(f.v, [f.station], (), cert)
    assert c1 is not None and c1.kind == gs.SK and np.allclose(c1.anchor, q1)


def test_the_lookahead_covers_the_next_onboard_state_that_the_braking_lookahead_missed():
    """The braking lookahead (braking from the true worst state, re-certified with the fresh measurement) is replaced by the
    band of station keeping on the next estimate (P2R2): positive here, and it bounds the band seen from the next state."""
    q, v0, u = np.array([G.cr_s_min - 2.0, 0.0, 0.0]), np.zeros(3), np.array([0.5, 0.0, 0.0])
    dp = float(gs.next_band(v0[None, :], u[None, :], B, VE, RHO)[0][0, 0])
    q1, vm1 = _worst_next(q, v0, u)
    X, T = gs.brake_reach(vm1[0] - VE, B)
    assert q1[0] + max(float(X) + RHO * float(T), gs.sk_hold(B)) <= q[0] + (q1[0] - q[0]) + dp + 1e-12
    assert float(_gc(q, v0).margins_world(u)[0]) > 0.0


def test_a_latched_station_keeping_is_not_cut_by_a_recomputed_drop():
    """R-c: at the edge of its band (0.3 m towards the CR, the loop cancelling the drift, the measured velocity +v_err) the
    margin recomputed from the fresh measurement is not positive; the certificate latched when the hold was engaged keeps
    the stage-2 hold loop (P2R3)."""
    a = np.array([gs.hold_s(CFG) - G.hold_err_m, 0.0, 0.0])
    g0 = _gc(a, (0.0, 0.0, 0.0), t=1.0)
    cert = g0.sk_certificate(a)
    assert cert is not None and cert.kind == gs.SK and cert.margin > 0.0
    q = a + np.array([B.e_track, 0.0, 0.0])
    g = _gc(q, (VE, 0.0, 0.0), t=9.0)
    assert g.hold_margin(a) <= 0.0
    loop = G.k_track * (a - q)
    cut = g.filter_mission(loop, ENV.v_max_nominal, anchor=a, gain=G.k_track, cap=G.v_pass)
    assert cut.modified
    kept = g.filter_mission(loop, ENV.v_max_nominal, anchor=a, gain=G.k_track, cap=G.v_pass, cert=cert)
    assert not kept.modified and np.allclose(kept.v, loop) and kept.margin == pytest.approx(cert.margin)
    c = g.certify(loop, (), [{"anchor": a, "gain": G.k_track, "amp": 0.0, "cap": G.v_pass}], cert)
    assert c is cert


def test_station_keeping_is_restoring_on_every_axis():
    """A large lateral deviation does not scale down the along-axis command (per-axis cap, formal P2R3a / P2R3c): with the
    former 2-norm cap the along-axis component fell below the declared drift."""
    q = np.array([-4.0, 0.0, 0.0])
    a = q + np.array([0.2, 2.0, 0.0])
    v = _gc(q).hold_velocity(a)
    assert v[0] == pytest.approx(gs.K_HOLD * 0.2) and v[0] > 0.0
    assert v[1] == pytest.approx(gs.HOLD_V_MAX)
    assert abs(v[0]) >= min(gs.K_HOLD * 0.2, gs.HOLD_V_MAX) - 1e-12


def test_certificates_apply_only_under_their_premises():
    a = np.array([G.cr_s_min - 3.0, 0.0, 0.0])
    g0 = _gc(a, (0.3, 0.0, 0.0), t=1.0)
    sk = g0.sk_certificate(a)
    assert sk is not None and sk.t_end > 1.0 + B.tau_c - 1e-9
    nxt = g0.certify(np.zeros(3), (), (), None)
    assert nxt is not None and nxt.kind == gs.NEXT
    other = GateSpec("OTHER", np.array([0.0, 0.0, -5.0]), np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]), 2.0, 2.0)
    assert not _gc(a, gate=other, t=1.1).cert_valid(sk)
    assert _gc(a, t=1.1).cert_valid(nxt) and not _gc(a, t=1.3).cert_valid(nxt)
    outage = dict(ve=float(NB.v_bound), rho=float(NB.v_bound) + float(NB.k_lin) * float(NB.v_bound))
    assert not _gc(a, t=1.1, **outage).cert_valid(nxt)
    assert not _gc(a, t=sk.t_end - 0.05, **outage).cert_valid(sk)
    assert _gc(a, t=sk.t_end + 0.05, **outage).cert_valid(sk)


def test_the_controller_drops_the_certificates_on_a_position_correction_and_logs_an_uncertified_command():
    plan = SCENARIOS["lost_drone_mutex"](CFG).plans[0]
    ctrl = DroneController(plan, CFG)
    assert not ctrl._p2_correction()
    ctrl.flows.p2_cert = gs.P2Certificate(gs.NEXT, 0.1, 0.0, RHO, 1.0, plan.gates[0].gate_id)
    assert not ctrl._p2_correction() and ctrl.flows.p2_cert is not None
    ctrl.perception.apply_position_correction(np.array([0.4, 0.0, 0.0]))
    assert ctrl._p2_correction() and ctrl.flows.p2_cert is None
    g = plan.gates[0]
    inside = gs.constraint_for(CFG, g, g.from_gate_frame([G.cr_s_min + 0.2, 0.0, 0.0]), np.zeros(3), False, v_err=VE,
                               rho=RHO, t=2.0)
    ctrl.gate_safety = {"margin": -1.0}
    ctrl._p2_certify_step(2.0, ctrl.ha.mode, inside, np.zeros(3), [{}], True)
    ev = [e for e in ctrl.events if e["type"] == "P2_UNCERTIFIED"]
    assert len(ev) == 1 and ev[0]["after_correction"] and ctrl.gate_safety["cert"] is None
    ctrl._p2_certify_step(2.1, ctrl.ha.mode, inside, np.zeros(3), [{}], True)
    assert len([e for e in ctrl.events if e["type"] == "P2_UNCERTIFIED"]) == 1

def test_a_certificate_issued_with_outage_reserve_survives_a_dvl_outage():
    q = np.array([G.cr_s_min - 4.0, 0.0, 0.0])
    reserve = 2 * float(NB.v_bound) + float(NB.k_lin) * float(NB.v_bound)
    fresh = _gc(q, rho=reserve, t=1.0)
    cert = fresh.next_certificate(np.zeros(3))
    assert cert is not None
    stale = _gc(q, ve=float(NB.v_bound), rho=reserve, t=1.0 + B.dt)
    assert np.array_equal(stale.cert_anchor(cert), q)


def test_failed_outage_reserve_preserves_the_nominal_handover():
    plan = SCENARIOS["lost_drone_mutex"](CFG).plans[0]
    ctrl = DroneController(plan, CFG)
    q, v0, u = np.array([-2.8, 0.0, 0.0]), np.zeros(3), np.array([0.5, 0.0, 0.0])
    ctrl.flows.p2_cert = _gc(q, v0).next_certificate(u)
    q1, vm1 = _worst_next(q, v0, u)
    next_gc = _gc(q1, vm1, t=1.1)
    reserve = 2 * float(NB.v_bound) + float(NB.k_lin) * float(NB.v_bound)

    ctrl._p2_certify_step(1.1, ctrl.ha.mode, next_gc, np.zeros(3), (), False, reserve)

    assert ctrl.gate_safety["cert"] is not None
    assert ctrl.gate_safety["cert"]["kind"] == gs.SK
    assert not any(e["type"] == "P2_UNCERTIFIED" for e in ctrl.events)


def test_a_continued_hold_keeps_its_earlier_end_of_braking():
    anchor = np.array([-2.4, 0.0, 0.0])
    initial = _gc(anchor, v0=(0.29, 0.0, 0.0)).sk_certificate(anchor)
    assert initial is not None
    nominal = _gc(anchor, t=1.5)
    assert nominal.hold_margin(anchor) > initial.margin
    ongoing = nominal.certify(np.zeros(3), [anchor], cert=initial)
    reserve = 2 * float(NB.v_bound) + float(NB.k_lin) * float(NB.v_bound)
    outage = _gc(anchor, t=1.8, ve=float(NB.v_bound), rho=reserve)

    assert outage.hold_margin(anchor) <= 0.0
    assert outage.certified_hold(anchor, ongoing) > 0.0


def test_a_hold_loop_can_use_a_narrower_fresh_band():
    anchor = np.array([-2.4, 0.0, 0.0])
    initial = _gc(anchor + [0.0, 1.0, 0.0]).sk_certificate(anchor)
    assert initial is not None and not gs.latchable(initial.dev, G.k_track, G.v_pass)
    nominal = _gc(anchor, t=4.0)
    loop = {"anchor": anchor, "gain": G.k_track, "cap": G.v_pass}

    fresh = nominal.certify(np.zeros(3), loops=[loop], cert=initial)

    assert fresh is not None and fresh.kind == gs.SK
    assert gs.latchable(fresh.dev, G.k_track, G.v_pass)
