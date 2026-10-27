"""Relocalization hold (control/flows.py): a committed drone enters the critical region only with a valid gate
localization; short of it, it holds on its path before the CR (mutex, priority and the automaton unchanged)."""

from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import Flows
from holo_fleet.sim.scenarios import SCENARIOS

G = DEFAULT.gate


def _obs(plan, s, loc_ok, t):
    g = plan.gates[0]
    p = g.from_gate_frame([s, 0.0, 0.0])
    gate = SimpleNamespace(s=s, l=0.0, dz=0.0, queue_s=plan.queue_s, queue_l=plan.queue_lateral, queue_dz=plan.queue_dz,
                           loc_ok=loc_ok)
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
