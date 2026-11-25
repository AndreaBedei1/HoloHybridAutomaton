"""P1 avoidance under sector-association ambiguity (Flows._robust_choice).

A target seen by several sectors may be one hull or several: the widened hypotheses cover every sub-pattern.  When a
direction opens every hypothesis it is chosen; when none does, the choice feasible for the measured association is used
(a HOLD gives no opening against a closing peer under any association) and the fallback is recorded."""

from types import SimpleNamespace as NS

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import Flows
from holo_fleet.sim.scenarios import SCENARIOS

CFG = DEFAULT
PLAN = SCENARIOS["p1_two_lines"](CFG).plans[0]


def _target(ranges, d):
    return NS(cls="DYNAMIC", pattern=frozenset(ranges), d_lower=d, r_min=d + 0.3, closing_rate=0.0,
              possible_neighbour=False, ranges=dict(ranges))


def _obs(targets):
    gate = NS(gate_id=None, s=0.0, l=0.0, dz=0.0, passed=False)
    return NS(t=10.0, p=np.array([0.0, 0.0, -5.0]), v_world=np.zeros(3), R_wb=np.eye(3), yaw=0.0,
              targets=list(targets), unhealthy=[], gate=gate, gate_index=0)


def test_diagonal_peer_in_two_sectors_gets_an_opening_command_not_a_hold():
    fl = Flows(PLAN, CFG)
    obs = _obs([_target({"FRONT": 0.9, "LEFT": 1.0}, 0.9)])
    v = fl.collision_avoidance(obs, np.zeros(3), np.zeros(3))
    assert np.linalg.norm(v) > 0.1, "a diagonal peer must not freeze the drone"
    assert fl.last_choice.feasible and fl.last_choice.label != "HOLD"
    assert fl.association_fallback
    measured = fl.escape.guarantee(frozenset({"FRONT", "LEFT"}), [])
    k = int(np.argmax(fl.escape.cands @ fl.last_choice.direction))
    assert measured[k] > 0.0, "the command opens the measured association"


def test_three_sector_target_falls_back_only_when_no_robust_direction_exists():
    fl = Flows(PLAN, CFG)
    obs = _obs([_target({"FRONT": 0.9, "LEFT": 1.0, "UP": 1.0}, 0.9)])
    wide = fl.threats(obs, CFG.sep.d_ca_exit)
    worst = np.min([fl.escape.guarantee(p, []) for p, _ in wide], axis=0)
    fl.collision_avoidance(obs, np.zeros(3), np.zeros(3))
    assert fl.association_fallback == (not np.any(worst > 0.0))
    assert fl.last_choice.feasible


def test_single_sector_target_uses_the_robust_choice_without_fallback():
    fl = Flows(PLAN, CFG)
    obs = _obs([_target({"FRONT": 0.9}, 0.9)])
    v = fl.collision_avoidance(obs, np.zeros(3), np.zeros(3))
    assert fl.last_choice.feasible and not fl.association_fallback
    assert float(v @ np.array([1.0, 0.0, 0.0])) < 0.0, "moves away from a FRONT threat"


def test_a_robust_direction_is_preferred_to_the_fallback():
    """Two sectors that a single direction can open together (FRONT and UP: back and down): no fallback."""
    fl = Flows(PLAN, CFG)
    obs = _obs([_target({"FRONT": 0.9, "UP": 1.0}, 0.9)])
    wide = fl.threats(obs, CFG.sep.d_ca_exit)
    worst = np.min([fl.escape.guarantee(p, []) for p, _ in wide], axis=0)
    fl.collision_avoidance(obs, np.zeros(3), np.zeros(3))
    if np.any(worst > 0.0):
        assert not fl.association_fallback
        k = int(np.argmax(fl.escape.cands @ fl.last_choice.direction))
        assert worst[k] > 0.0, "the chosen direction opens every hypothesis"


def test_no_feasible_association_keeps_the_infeasible_widened_choice():
    """Threats on every side: neither the widened nor the measured association is feasible; the planner is not
    reported as feasible and no fallback is claimed."""
    fl = Flows(PLAN, CFG)
    sides = ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")
    obs = _obs([_target({s: 0.9}, 0.9) for s in sides])
    fl.collision_avoidance(obs, np.zeros(3), np.zeros(3))
    assert not fl.last_choice.feasible and not fl.association_fallback
