"""Entries into the CR that bypass the stage-2 check (st_lost_middle s5201): the referee classification and the claim of the moved drone."""

import sys
from pathlib import Path

from holo_fleet.perception.passage import report_phase
from holo_fleet.perception.payload import GP_COMMITTED, GP_NONE
from holo_fleet.referee.robustness_metrics import cr_entry_kind

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))


def _rec(committed, entry=None):
    return {"committed": committed, "gate": {"entry": entry}}


def test_an_entry_is_admitted_pushed_or_uncommitted():
    admitted = [_rec(False), _rec(True, "HOLD"), _rec(True, "ADMIT"), _rec(True, "ADMIT")]
    pushed = [_rec(False), _rec(True, "HOLD"), _rec(True, "HOLD"), _rec(True, None)]
    assert cr_entry_kind(admitted, 3) == "ADMITTED"
    assert cr_entry_kind(pushed, 3) == "PUSHED"
    assert cr_entry_kind([_rec(False)], 0) == "UNCOMMITTED"
    older = [_rec(True, "ADMIT"), _rec(False), _rec(True, "HOLD")]
    assert cr_entry_kind(older, 2) == "PUSHED"


def test_a_drone_moved_into_the_keepout_box_reports_a_claim_whether_committed_or_not():
    assert report_phase(False, False, True, True) == GP_COMMITTED
    assert report_phase(False, False, False, True) == GP_COMMITTED
    assert report_phase(False, False, False, False) == GP_NONE


def test_the_fleet_model_keeps_the_invariant_under_pushes_of_a_committed_drone_but_not_of_a_queued_one():
    import z3
    from check_fleet import PREMISES, Fleet, _run, timers

    T = timers()
    for push, want in (("committed", z3.unsat), ("any", z3.sat)):
        fl = Fleet(2, 1, 1, T, free=True, push=push, **PREMISES[2])
        S = z3.Solver()
        S.add(*_run(fl, z3.Not(fl.inv(1))))
        assert S.check() == want
