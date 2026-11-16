"""Premise C (a queued drone learns of a commitment within the S2 bound): the referee monitor commitment_visibility."""

import numpy as np

from holo_fleet.referee.robustness_metrics import commitment_visibility


def _states(n, m):
    return [[{"mode": "MUTEX_YIELD", "committed": False, "gate": {"at_queue": True, "occ": "FREE"}} for _ in range(m)] for _ in range(n)]


def test_the_delay_is_measured_from_the_commit_edge_to_the_latch_of_each_queued_drone():
    t = np.arange(0.0, 60.0, 0.1)
    st = _states(3, len(t))
    for i in range(100, 400):
        st[0][i]["committed"] = True
    for i in range(130, 400):
        st[1][i]["gate"] = {"at_queue": True, "occ": "BUSY"}
    for i in range(110, 400):
        st[2][i]["gate"] = {"at_queue": True, "occ": "EXITING"}
    r = commitment_visibility(["d0", "d1", "d2"], t, st, len(t))
    assert r["episodes"] == 2 and abs(r["max_s"] - 3.0) < 1e-6 and r["violations"] == 0 and r["within_5s"] == 2


def test_a_late_latch_and_a_blind_commit_are_violations_and_a_give_up_is_censored():
    t = np.arange(0.0, 100.0, 0.1)
    st = _states(3, len(t))
    for i in range(10, 900):
        st[0][i]["committed"] = True
    for i in range(10 + 330, 900):
        st[1][i]["gate"] = {"at_queue": True, "occ": "BUSY"}        # 33 s after the commit: beyond the bound
    for i in range(50, 900):
        st[2][i]["committed"] = True                                 # commits with its latch FREE: no evidence
    r = commitment_visibility(["d0", "d1", "d2"], t, st, len(t))
    assert r["committed_without_evidence"] == 1 and r["violations"] == 2
    st = _states(2, len(t))
    for i in range(10, 50):
        st[0][i]["committed"] = True                                 # gives up before the other learns of it
    assert commitment_visibility(["d0", "d1"], t, st, len(t))["censored"] == 1
