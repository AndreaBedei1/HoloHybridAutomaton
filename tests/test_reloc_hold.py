"""Relocalization hold (control/flows.py): a committed drone enters the critical region only with a valid gate
localization; short of it, it holds on its path before the CR (mutex, priority and the automaton unchanged)."""

from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import Flows
from holo_fleet.sim.scenarios import SCENARIOS

G = DEFAULT.gate


def _obs(plan, s, loc_ok, t, entry_ok=True, l=0.0):
    g = plan.gates[0]
    p = g.from_gate_frame([s, l, 0.0])
    gate = SimpleNamespace(s=s, l=l, dz=0.0, queue_s=plan.queue_s, queue_l=plan.queue_lateral, queue_dz=plan.queue_dz,
                           loc_ok=loc_ok, entry_ok=entry_ok)
    return SimpleNamespace(t=t, p=np.asarray(p, float), gate=gate, gate_index=0)


def _along(plan, v):
    return float(np.asarray(plan.gates[0].axis, float) @ v)


def test_no_entry_into_the_critical_region_without_a_valid_gate_localization():
    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    s_hold = -(G.cr_half_len + 0.229 + 0.15)
    v, _ = fl.gate(_obs(plan, -3.0, False, 0.0), "MUTEX_PASS")           # far: the drone advances on its path
    assert _along(plan, v) > 0.3 and not fl.reloc_hold
    v, _ = fl.gate(_obs(plan, s_hold + 0.02, False, 1.0), "MUTEX_PASS")  # at the hold point: does not advance
    assert fl.reloc_hold and _along(plan, v) <= 0.0
    v, _ = fl.gate(_obs(plan, s_hold - 0.02, False, 8.0), "MUTEX_PASS")  # relocalization: backs off, then returns
    assert fl.reloc_hold and _along(plan, v) < 0.0
    v, _ = fl.gate(_obs(plan, s_hold - 0.05, True, 13.0), "MUTEX_PASS")  # valid near the hold point: enters
    assert not fl.reloc_hold and _along(plan, v) > 0.3
    v, _ = fl.gate(_obs(plan, -0.5, False, 15.0), "MUTEX_PASS")          # inside the CR it does not stop again
    assert not fl.reloc_hold and _along(plan, v) > 0.3


def test_a_valid_localization_far_from_the_hold_point_does_not_clear_the_entry():
    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    fl.gate(_obs(plan, -3.0, True, 0.0), "MUTEX_PASS")
    s_hold = -(G.cr_half_len + 0.229 + 0.15)
    fl.gate(_obs(plan, s_hold + 0.02, False, 4.0), "MUTEX_PASS")
    assert fl.reloc_hold


def test_stage_two_holds_a_localized_drone_while_a_hull_is_seen_or_heard_in_the_cr():
    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    s_hold = -(G.cr_half_len + 0.229 + 0.15)
    v, _ = fl.gate(_obs(plan, s_hold + 0.02, True, 1.0, entry_ok=False), "MUTEX_PASS")
    assert fl.reloc_hold and fl.entry_state == "HOLD" and _along(plan, v) <= 0.0
    v, _ = fl.gate(_obs(plan, s_hold + 0.02, True, 2.0, entry_ok=True), "MUTEX_PASS")
    assert not fl.reloc_hold and fl.entry_state == "ADMIT" and _along(plan, v) > 0.3


def test_the_stage_two_hold_is_logged_once_beyond_its_bound_and_never_forced():
    """Beyond t_occ_max + t_clear the hold is logged once (ENTRY_HOLD_TIMEOUT); the drone keeps holding before the CR:
    no retreat (it would stay committed without the stage-1 check) and no forced entry."""
    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    s_hold = -(G.cr_half_len + 0.229 + 0.15)
    t_max = G.t_occ_max + G.t_clear
    fl.gate(_obs(plan, s_hold + 0.02, False, 0.0), "MUTEX_PASS")
    fl.gate(_obs(plan, s_hold + 0.02, False, t_max + 0.1), "MUTEX_PASS")
    v, _ = fl.gate(_obs(plan, s_hold + 0.02, False, t_max + 0.2), "MUTEX_PASS")
    assert [e["type"] for e in fl.events] == ["ENTRY_HOLD_TIMEOUT"]
    assert fl.reloc_hold and _along(plan, v) <= 0.0


def test_give_up_only_behind_the_entry_plane_and_never_once_admitted():
    """After the bounded stage-2 hold the drone backs off (still committed) behind the give-up plane, and only there
    requests the automaton's give-up edge; an admitted drone never requests it (sticky-flag regression)."""
    from holo_fleet.perception.sonar_geometry import R_OUT

    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    s_hold = -(G.cr_half_len + 0.229 + 0.15)
    assert G.nav_err_m == DEFAULT.acoustic.nav_err_gate_m          # one declared gate-frame navigation error
    s_abort = -(G.cr_half_len + R_OUT + G.nav_err_m)
    fl = Flows(plan, DEFAULT)
    fl.gate(_obs(plan, s_hold + 0.02, False, 0.0), "MUTEX_PASS")
    t_out = G.t_occ_max + G.t_clear + 0.5
    v, _ = fl.gate(_obs(plan, s_hold, False, t_out), "MUTEX_PASS")
    assert fl.hold_timeout and not fl.abort_request and _along(plan, v) < 0.0      # backs off first
    fl.gate(_obs(plan, s_abort + 0.1, False, t_out + 2.0), "MUTEX_PASS")
    assert not fl.abort_request                                                   # not yet behind the plane
    fl.gate(_obs(plan, s_abort - 0.05, False, t_out + 4.0), "MUTEX_PASS")
    assert fl.abort_request                                                       # behind it: give up
    # the same timeout, but the drone is admitted meanwhile (e.g. during a hazard step): no request any more
    fl.gate(_obs(plan, s_hold - 0.05, True, t_out + 5.0), "MUTEX_PASS")
    assert fl.cr_cleared and not fl.abort_request
    fl.gate(_obs(plan, -0.5, False, t_out + 8.0), "MUTEX_PASS")                   # inside the CR
    assert not fl.abort_request
