"""The operating scope: the exclusion zone of a gate (holo_fleet/scope.py), every failure scenario outside it, and rule D only
outside it (perception/acoustic_gate.py)."""

import dataclasses
import math

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception import payload as pl
from holo_fleet.scope import current_bound, failure_excluded, obstruction_half
from holo_fleet.sim.scenarios import SCENARIOS

ONBOARD_MARGIN_M = 1.5          # what a receiver adds to a last fix (fix region, own bound, one TDMA period of motion): kept as budget


def _gate():
    return SCENARIOS["lost_drone_mutex"](DEFAULT).judged_gates[0]


def test_the_critical_region_and_the_space_beside_the_frame_are_excluded():
    g = _gate()
    h = obstruction_half(g)
    assert failure_excluded(g, g.from_gate_frame([0.0, 0.0, 0.0]), 0.0)
    assert failure_excluded(g, g.from_gate_frame([-(h[0] + 0.9), 0.0, 0.0]), 0.0)          # within the coast
    assert not failure_excluded(g, g.from_gate_frame([-(h[0] + 1.1), 0.0, 0.0]), 0.0)
    assert failure_excluded(g, g.from_gate_frame([-(h[0] + 1.1), 0.0, 0.0]), 0.0, margin=0.2)


def test_the_zone_grows_with_the_current_and_covers_everything_when_the_hull_cannot_rise():
    g = _gate()
    p = g.from_gate_frame([-8.0, 0.0, 0.0])
    assert not failure_excluded(g, p, 0.0) and failure_excluded(g, p, 0.3)
    assert failure_excluded(g, g.from_gate_frame([-60.0, 30.0, 0.0]), 0.0, w_down=DEFAULT.scope.v_rise_min_mps)


def test_a_hull_just_above_the_frame_can_coast_down_into_it_and_one_far_above_and_aside_cannot():
    g = _gate()
    h = obstruction_half(g)
    assert failure_excluded(g, g.from_gate_frame([0.0, 0.0, h[2] + 0.01]), 0.5)
    assert not failure_excluded(g, g.from_gate_frame([h[0] + 1.5, 0.0, h[2] + 1.0]), 0.5)


def test_every_launch_and_every_failure_point_is_outside_the_zone_with_the_onboard_margin():
    """Launch (in the scenarios with a permanent failure): a drone never heard failed at its launch (rule D treats it as outside every zone).  A failure at t_on: the drone has
    moved at most v_max_nominal * t_on from its spawn, towards the gate in the worst case; a failure at a position (``at``) on the
    axis, the worst lateral position.  Both with the margin a receiver adds to a fix, so that rule D can apply."""
    v = DEFAULT.env.v_max_nominal
    n = 0
    for name, make in SCENARIOS.items():
        sc = make(DEFAULT)
        w_h, w_dn = current_bound(sc.sim.current)
        lost = any(f["type"] == "TOTAL_FAILURE" and f.get("t_off") is None for f in sc.sim.faults)
        for g in sc.judged_gates if lost else ():
            for q in sc.sim.spawn_positions:
                assert not failure_excluded(g, q, w_h, w_dn, margin=ONBOARD_MARGIN_M), (name, "launch")
        for f in sc.sim.faults:
            if f["type"] != "TOTAL_FAILURE" or f.get("t_off") is not None:
                continue
            for g in sc.judged_gates:
                if f.get("at"):
                    p = g.from_gate_frame([float(f["at"]["s_min"]), 0.0, 0.0])
                else:
                    p0 = np.asarray(sc.sim.spawn_positions[sc.sim.names.index(f["drone"])], float)
                    u = np.asarray(g.center, float) - p0
                    u[2] = 0.0
                    p = p0 + min(v * float(f["t_on"]), float(np.linalg.norm(u))) * u / max(float(np.linalg.norm(u)), 1e-9)
                assert not failure_excluded(g, p, w_h, w_dn, margin=ONBOARD_MARGIN_M), name
                n += 1
    assert n >= 9                                         # the lost_drone_mutex family, the stress batch, the failures near the gate


def _receiver(w_h=0.0):
    from test_status_payload import _observer, _rx, P
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    ae.cfg = dataclasses.replace(ae.cfg, scope=dataclasses.replace(ae.cfg.scope, w_h_max_mps=w_h))
    ae.g = gp.plan.gates[0]
    return tr, ae, _rx, P


def test_rule_d_releases_a_silent_claim_only_outside_the_zone():
    """A drone that reported admitted and fell silent: dead after t_silent_gate_s if last heard far from the gate, never if last heard
    where its hull could reach the gate (in the CR; or 6 m before it in a 0.3 m/s current)."""
    from holo_fleet.config import DEFAULT as D
    for q, w_h, released in (([-14.0, 0.0, -5.], 0.0, True), ([0.0, 0.0, -5.], 0.0, False), ([-6.5, 0.0, -5.], 0.3, False)):
        tr, ae, _rx, P = _receiver(w_h)
        _rx(tr, P, 1.0, 1, q, phase=pl.GP_ADMITTED)
        t = 1.0 + D.acoustic.t_silent_gate_s + 5.0
        for tt in np.arange(2.0, t, 1.0):                   # the other drone heard every second: the tracker prunes old fixes
            _rx(tr, P, float(tt), 2, [-20.0, 6.0, -5.], phase=pl.GP_NONE)
        assert ae.dead(1, t) == released and (ae.claims(t) == set()) == released, (q, w_h)
        assert ae.rule_d[1]["released"] == released and ae.rule_d[1]["fix_age_s"] > 70.0     # the decision is logged


def test_the_declared_current_is_the_bound_of_the_field():
    sc = SCENARIOS["st_gate_sinus"](DEFAULT)
    w_h, w_dn = current_bound(sc.sim.current)
    assert math.isclose(w_h, 0.10 + 0.15, abs_tol=0.01) and w_dn == 0.0
