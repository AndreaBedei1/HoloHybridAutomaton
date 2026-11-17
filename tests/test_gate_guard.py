"""Gate guard of the avoidance layer (control/flows.gate_guard, control/escape.py): st_lost_middle s5201, where the
separation filter drove a held, committed drone forward into the critical region without admission."""

import dataclasses
from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.escape import EscapePlanner
from holo_fleet.control.flows import GUARD_GAIN, Flows
from holo_fleet.perception.sonar_geometry import HULL_HALF
from holo_fleet.sim.scenarios import SCENARIOS

ENV, G = DEFAULT.env, DEFAULT.gate
S_HOLD = -(G.cr_half_len + float(HULL_HALF[0]) + 0.15)
REAR = [(frozenset({"LEFT", "REAR"}), ENV.v_open), (frozenset({"REAR"}), ENV.v_open)]
X = np.array([1.0, 0.0, 0.0])


def test_s5201_the_separation_filter_no_longer_drives_a_held_drone_into_the_region():
    pl = EscapePlanner(DEFAULT)
    v11 = pl.warning_velocity(np.zeros(3), REAR, [], 0.0, ENV.v_max_nominal)
    assert v11.label == "FWD" and v11.v_body[0] > 0.4          # the logged choice of s5201 (0.49 m/s forward)
    pl.reset()
    w = pl.warning_velocity(np.zeros(3), REAR, [], 0.0, ENV.v_max_nominal, cap=(X, GUARD_GAIN * (S_HOLD - (-1.15))))
    assert w.guard == "HOLD" and w.v_body @ X <= 1e-9 and np.allclose(w.v_body, 0.0)


def test_the_escape_overrides_the_guard_only_when_no_allowed_direction_opens_every_threat():
    pl = EscapePlanner(DEFAULT)
    e = pl.escape_velocity(REAR, [], 0.0, ENV.v_escape, cap=(X, 0.0))
    assert e.guard == "OVERRIDE" and e.v_body @ X > 0.0
    pl.reset()
    e = pl.escape_velocity([(frozenset({"LEFT", "REAR"}), ENV.v_open)], [], 0.0, ENV.v_escape, cap=(X, 0.0))
    assert e.guard == "ACTIVE" and e.v_body @ X <= 1e-9 and e.feasible


def test_a_guard_hold_moves_back_only_when_that_closes_on_no_threat():
    pl = EscapePlanner(DEFAULT)
    h = pl.guard_hold((X, -0.2), [(frozenset({"FRONT"}), ENV.v_open)], [])
    assert h.label == "GUARD_RETREAT" and abs(h.v_body @ X + 0.2) < 1e-9     # back, away from a threat in front
    assert pl.guard_hold((X, -0.2), REAR, []).label == "GUARD_HOLD"            # back would close on it: still
    assert pl.guard_hold((X, -0.2), [(frozenset({"UP"}), ENV.v_open)], []).label == "GUARD_HOLD"   # the UP cone leans back
    assert pl.guard_hold((X, 0.1), [(frozenset({"FRONT"}), ENV.v_open)], []).label == "GUARD_HOLD"  # behind the bound


def _obs(plan, s, l=0.0, dz=0.0, passed=False):
    g = plan.gates[0]
    gate = SimpleNamespace(gate_id="G06", s=s, l=l, dz=dz, passed=passed)
    return SimpleNamespace(t=0.0, p=np.asarray(g.from_gate_frame([s, l, dz]), float), gate=gate, gate_index=0)


def test_the_guard_is_active_only_for_a_drone_near_the_region_and_not_admitted():
    plan = SCENARIOS["gate_single"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    a, u = fl.gate_guard(_obs(plan, -1.5))
    assert np.allclose(a[:2], np.asarray(plan.gates[0].axis[:2], float) / np.linalg.norm(plan.gates[0].axis[:2]))
    assert abs(u - GUARD_GAIN * (S_HOLD + 1.5)) < 1e-9
    assert fl.gate_guard(_obs(plan, -5.4)) is None                  # the queue line: outside the keep-out box
    assert fl.gate_guard(_obs(plan, -1.5, l=4.0)) is None           # beside the box
    assert fl.gate_guard(_obs(plan, -1.5, passed=True)) is None
    fl.cr_cleared = True                                            # admitted by the stage-2 check
    assert fl.gate_guard(_obs(plan, -1.0)) is None
    fl.cr_cleared, fl.past_centre = False, True                     # carried past the centre: goes on to the exit
    assert fl.gate_guard(_obs(plan, 0.2)) is None
    off = dataclasses.replace(DEFAULT, gate=dataclasses.replace(G, avoidance_guard=False))
    assert Flows(plan, off).gate_guard(_obs(plan, -1.5)) is None    # the v11 behaviour, for the comparison runs


def test_the_guard_bounds_every_choice_of_the_separation_filter_over_the_sector_patterns():
    pl = EscapePlanner(DEFAULT)
    for P in pl.regions:
        for u in (-0.3, 0.0, 0.2):
            for vd in (np.zeros(3), np.array([0.4, 0.0, 0.0])):
                pl.reset()
                w = pl.warning_velocity(vd, [(P, ENV.v_open)], [], 0.0, ENV.v_max_nominal, cap=(X, u))
                assert w.v_body @ X <= max(u, 0.0) + 1e-9, (P, u)


def test_a_guard_hold_keeps_station_where_it_began_and_never_moves_back_beyond_it():
    plan = SCENARIOS["gate_single"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    obs = _obs(plan, -1.0)
    obs.R_wb = np.eye(3)
    hold = SimpleNamespace(v_body=np.zeros(3), label="GUARD_HOLD", guard="HOLD")
    cap = fl.gate_guard(obs)
    a = cap[0]
    assert np.allclose(fl._guarded(obs, hold, cap, "SEPARATION_WARNING"), 0.0)        # the hold begins at s = -1.0
    ahead = _obs(plan, -0.9)
    ahead.R_wb = np.eye(3)
    v = fl._guarded(ahead, hold, fl.gate_guard(ahead), "SEPARATION_WARNING")          # drifted 0.1 m forward
    assert abs(v @ a + 0.1) < 1e-9
    behind = _obs(plan, -1.2)
    behind.R_wb = np.eye(3)
    assert np.allclose(fl._guarded(behind, hold, fl.gate_guard(behind), "SEPARATION_WARNING"), 0.0)   # never forward
