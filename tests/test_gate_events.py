"""Gate events E1-E8 and the stall detector (referee/gate_events.py) on synthetic traces."""

import importlib.util
from pathlib import Path

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.referee.gate_events import gate_events

ROOT = Path(__file__).resolve().parents[1]
G = DEFAULT.gate
PLANS = [{"drone": "drone_0", "queue_lateral": 0.0, "queue_s": -5.0},
         {"drone": "drone_1", "queue_lateral": 3.5, "queue_s": -5.0}]


def _trace(T=60.0, dt=0.1):
    t = np.round(np.arange(0.0, T, dt), 1)
    gf = np.zeros((len(t), 2, 3))
    gf[:, 0, 0] = -20.0                   # drone 0 far before the gate
    gf[:, 1, :] = [-5.0, 3.5, 0.0]        # drone 1 at its queue point
    st = [[{"committed": False, "gate": {"id": "g", "occ": "FREE", "at_queue": False}} for _ in t] for _k in range(2)]
    for i in range(len(t)):
        st[1][i]["gate"]["at_queue"] = "True"          # logged as a string: still parsed as queued
    return t, gf, st


def test_premature_release_and_false_free_are_counted():
    t, gf, st = _trace()
    a, b = np.searchsorted(t, 10.0), np.searchsorted(t, 14.0)
    gf[a:b, 0, :] = [0.3, 0.0, 0.0]                    # drone 0 truly inside the CR from 10 to 14 s
    for i in range(np.searchsorted(t, 9.0), np.searchsorted(t, 12.0)):
        st[1][i]["gate"]["occ"] = "BUSY"
    st[1][np.searchsorted(t, 12.0)]["gate"].update(occ="EXITING", exit_witness={"sector": "FRONT", "range": 8.0})
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert ev["E2"]["SONAR"] == 1 and ev["E2"]["total"] == 1 and ev["E2"]["to_free"] == 1   # EXITING -> FREE inside
    assert ev["E1"]["false_free_s"] > 1.5              # FREE again from 12.1 s while drone 0 is still inside
    assert ev["E3"] == 0 and ev["E4"] == 0 and ev["E4_source"] == "commit" and ev["E7"] is None


def test_commit_into_an_occupied_cr_is_an_unsafe_action():
    t, gf, st = _trace()
    a, b = np.searchsorted(t, 10.0), np.searchsorted(t, 14.0)
    gf[a:b, 0, :] = [0.0, 0.0, 0.0]
    for i in range(np.searchsorted(t, 11.0), len(t)):
        st[1][i]["committed"] = True
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert ev["E3"] == 1 and ev["E4"] == 1             # no stage 2 logged: the commit is the entry decision


def test_stage_two_admission_and_hold_are_separated():
    t, gf, st = _trace()
    a, b = np.searchsorted(t, 10.0), np.searchsorted(t, 14.0)
    gf[a:b, 0, :] = [0.0, 0.0, 0.0]
    for i in range(np.searchsorted(t, 11.0), len(t)):
        st[1][i]["committed"] = True
        st[1][i]["gate"]["entry"] = "HOLD" if t[i] < 15.0 else "ADMIT"
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert ev["E3"] == 1 and ev["E4"] == 0 and ev["E7"] == 1 and ev["E4_source"] == "stage2"


def test_idle_gate_beyond_the_release_bound_is_a_stall():
    t, gf, st = _trace(T=G.t_occ_max + G.t_clear + 10.0)
    for i in range(len(t)):
        st[1][i]["gate"]["occ"] = "BUSY"               # believes the CR busy, CR empty, nobody committed
        st[0][i]["gate"]["at_queue"] = True            # every healthy drone that has not passed is queued
    gf[:, 0, :] = [-5.0, -3.5, 0.0]
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert ev["stalled"] and ev["E8"]["idle_stalls"] and ev["E1"]["false_busy_frac"] > 0.49   # drone 1 of the two
    t2, gf2, st2 = _trace(T=G.t_occ_max)                 # shorter than the bound: a wait, not a stall
    assert not gate_events(t2, gf2, st2, [0, 1], "g", PLANS, DEFAULT)["stalled"]


def test_decisions_are_judged_against_the_truth_when_they_were_made():
    t, gf, st = _trace()
    i = np.searchsorted(t, 11.0)
    gf[:i, 0, :] = [0.0, 0.0, 0.0]                     # drone 0 inside until the row before the commit record
    for k in range(i, len(t)):
        st[1][k]["committed"] = True
    assert gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)["E3"] == 1   # the truth at row i-1 had it inside
    gf[i - 1, 0, :] = [-5.0, 0.0, 0.0]
    assert gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)["E3"] == 0


def test_waiting_for_an_approaching_drone_is_not_an_idle_stall():
    t, gf, st = _trace(T=G.t_occ_max + G.t_clear + 10.0)
    for i in range(len(t)):
        st[1][i]["gate"]["occ"] = "BUSY"
        st[0][i]["gate"]["at_queue"] = False           # drone 0 still on its way to the queue: not all waiting
    gf[:, 0, :] = [-9.0, 0.0, 0.0]
    assert not gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)["stalled"]


def test_committed_drone_that_never_passes_is_a_stall():
    L = abs(PLANS[1]["queue_lateral"]) + G.exit_s - PLANS[1]["queue_s"]
    t, gf, st = _trace(T=L / G.v_pass + G.t_occ_max + G.t_clear + 20.0)
    gf[:, 1, :] = [-1.9, 0.0, 0.0]                     # held before the CR
    for i in range(len(t)):
        st[1][i]["committed"] = True
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert ev["stalled"] and ev["E8"]["committed_stalls"][0][0] == 1


def test_a_slow_committed_passage_violates_the_crossing_budget():
    L = abs(PLANS[1]["queue_lateral"]) + G.exit_s - PLANS[1]["queue_s"]
    budget = L / G.v_pass + G.t_occ_max + G.t_clear
    t, gf, st = _trace(T=budget + 40.0)
    gf[:, 1, :] = [-1.9, 0.0, 0.0]
    i_in, i_out = np.searchsorted(t, budget + 20.0), np.searchsorted(t, budget + 25.0)
    gf[i_in:i_out, 1, :] = [0.0, 0.0, 0.0]               # finally through the CR ...
    gf[i_out:, 1, :] = [G.exit_s + 1.0, 0.0, 0.0]        # ... and beyond the exit, long after the budget
    for i in range(i_out):
        st[1][i]["committed"] = True
    ev = gate_events(t, gf, st, [0, 1], "g", PLANS, DEFAULT)
    assert 1 in ev["E8"]["passed"] and ev["stalled"] and ev["E8"]["committed_stalls"][0][0] == 1
