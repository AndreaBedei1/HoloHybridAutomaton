"""Timed latch release (STALE): a committed drone is admitted no earlier than the last corridor sighting plus
GateRule.t_admit_stale, and the give-up timer of the stage-2 hold runs only from then (no commit/give-up loop)."""

from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import Flows
from holo_fleet.sim.scenarios import SCENARIOS

G = DEFAULT.gate


def _obs(plan, s, t, entry_after, loc_ok=True):
    g = plan.gates[0]
    gate = SimpleNamespace(s=s, l=0.0, dz=0.0, queue_s=plan.queue_s, queue_l=plan.queue_lateral, queue_dz=plan.queue_dz,
                           loc_ok=loc_ok, entry_ok=True, admit_wait_s=0.0, entry_after=entry_after)
    return SimpleNamespace(t=t, p=np.asarray(g.from_gate_frame([s, 0.0, 0.0]), float), gate=gate, gate_index=0, targets=[])


def _plan():
    return SCENARIOS["line_parallel_mutex"](DEFAULT).plans[2]


def test_no_admission_before_the_stale_bound_and_admission_right_after():
    """Dwell in the admission zone from t = 0; last sighting at 10 s: admission strictly after 10 s + t_admit_stale."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    bound = 10.0 + G.t_admit_stale
    t = 0.0
    while t < bound - 0.1:
        fl.gate(_obs(plan, G.stage2_hold_s + 0.02, t, bound), "MUTEX_PASS")
        assert not fl.cr_cleared and fl.entry_state == "HOLD"
        t += 0.5
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, bound, bound), "MUTEX_PASS")
    assert not fl.cr_cleared
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, bound + 0.1, bound), "MUTEX_PASS")
    assert fl.cr_cleared and fl.entry_state == "ADMIT"


def test_the_timed_bound_also_runs_from_the_dwell_in_the_admission_zone():
    """An old sighting does not shorten the wait: the bound is at least t_admit_stale after the zone was entered
    (formal/check_stale.py ST1, case 1: an occupant admitted before the dwell has left by then)."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    old = 1.0 + G.t_admit_stale
    fl.gate(_obs(plan, -5.0, 30.0, old), "MUTEX_PASS")
    t = 40.0
    while t < 40.0 + G.t_admit_stale - 0.1:
        fl.gate(_obs(plan, G.stage2_hold_s + 0.02, t, old), "MUTEX_PASS")
        assert not fl.cr_cleared
        t += 0.5
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 40.0 + G.t_admit_stale + 0.1, old), "MUTEX_PASS")
    assert fl.cr_cleared


def test_leaving_the_admission_zone_restarts_the_dwell():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    old = 1.0 + G.t_admit_stale
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 40.0, old), "MUTEX_PASS")
    fl.gate(_obs(plan, G.stage2_hold_s - 1.0, 60.0, old), "MUTEX_PASS")
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 70.0, old), "MUTEX_PASS")
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 40.0 + G.t_admit_stale + 1.0, old), "MUTEX_PASS")
    assert not fl.cr_cleared and fl.zone_since == 70.0


def test_the_hold_is_not_given_up_before_the_stale_bound_has_elapsed():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    bound = 5.0 + G.t_admit_stale
    t = 0.0
    while t < bound - 0.1:
        fl.gate(_obs(plan, G.stage2_hold_s + 0.02, t, bound, loc_ok=False), "MUTEX_PASS")
        assert not fl.cr_cleared and not fl.hold_timeout and not fl.abort_request
        t += 0.5
    assert G.t_admit_stale > G.t_occ_max + G.t_clear
    while t < bound + G.t_occ_max + G.t_clear + 1.0:
        fl.gate(_obs(plan, G.stage2_hold_s + 0.02, t, bound, loc_ok=False), "MUTEX_PASS")
        t += 0.5
    assert fl.hold_timeout


def test_without_a_timed_release_admission_is_unchanged():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 3.0, -1e9), "MUTEX_PASS")
    assert fl.cr_cleared
