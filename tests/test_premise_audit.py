"""Premise C audit by the order of events (holo_fleet/referee/premise_audit.py): no time threshold decides a class."""

import numpy as np

from holo_fleet.referee.premise_audit import audit, classify_pair, first_claim_emission

T = np.round(np.arange(0.0, 80.0, 0.1), 2)


def _states(n, m=len(T)):
    return [[{"t": float(T[i]), "mode": "MUTEX_YIELD", "committed": False,
              "gate": {"at_queue": True, "occ": "FREE", "tx": [0, 0]}} for i in range(m)] for _ in range(n)]


def _commit(st, k, i0, i1, admit_at=None):
    for i in range(i0, i1):
        st[k][i]["committed"] = True
        st[k][i]["gate"] = {**st[k][i]["gate"], "tx": [0, 1]}
    if admit_at is not None:
        st[k][admit_at]["gate"] = {**st[k][admit_at]["gate"], "entry": "ADMIT"}


def _block(st, j, i0, i1):
    for i in range(i0, i1):
        st[j][i]["gate"] = {**st[j][i]["gate"], "occ": "BUSY"}


def _cls(st, k=0, j=1, ep=None, deliver=0.15, b2=True):
    ep = ep or (100, 400)
    t_emit = first_claim_emission(T, st[k], k, ep[0], ep[1], 1.0, len(st)) if b2 else None
    return classify_pair(T, st, k, ep, j, len(T), deliver, t_emit, b2)


def test_a_queued_drone_that_never_commits_during_the_claim_is_satisfied_whatever_the_delay_of_its_evidence():
    st = _states(2)
    _commit(st, 0, 100, 500)
    _block(st, 1, 450, 500)                  # 35 s after the commit (the old monitor's 32 s bound): the order is what counts
    r = _cls(st, ep=(100, 500))
    assert r["class"] == "SATISFIED" and r["t_evidence"] == 45.0 and r["t_commit"] is None


def test_the_first_claim_emission_follows_the_tdma_slot_of_the_claimant():
    st = _states(3)
    _commit(st, 2, 100, 400)                 # commit at 10.0 s; slot 2 of 3 in a 1 s period: 10.667 s
    assert abs(first_claim_emission(T, st[2], 2, 100, 400, 1.0, 3) - (10.0 + 2.0 / 3.0)) < 1e-6


def test_a_commit_before_the_claim_could_arrive_is_concurrent_and_needs_evidence_before_admission():
    st = _states(2)
    _commit(st, 0, 100, 400, admit_at=150)
    _commit(st, 1, 101, 400)                 # 0.1 s later: drone 0's first emission at 10.0 s arrives at 10.15 s
    _block(st, 0, 120, 400)
    _block(st, 1, 125, 400)
    r = _cls(st)
    assert r["class"] == "CONCURRENT" and r["t_evidence_pair"] == [12.0, 12.5]
    st[0][110]["gate"] = {**st[0][110]["gate"], "entry": "ADMIT"}   # admitted before drone 1 knew of it
    assert _cls(st)["class"] == "VIOLATED"


def test_a_blind_commit_after_the_claim_could_arrive_is_a_violation():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _commit(st, 1, 200, 400)                 # 10 s after the claim, no evidence at drone 1
    r = _cls(st)
    assert r["class"] == "VIOLATED" and r["tdma_opportunities"] >= 9


def test_a_commit_after_a_release_while_the_claim_holds_is_a_violation():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _block(st, 1, 120, 150)                  # blocked, then released while drone 0 still claims
    _commit(st, 1, 160, 400)
    assert _cls(st)["class"] == "VIOLATED"


def test_an_undeliverable_claim_latency_beyond_acceptance_makes_a_blind_commit_a_violation():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _commit(st, 1, 200, 400)
    _block(st, 0, 210, 400)
    _block(st, 1, 210, 400)
    r = _cls(st, deliver=None)
    assert r["class"] == "VIOLATED" and "undeliverable" in r["why"]


def test_b0_runs_leave_a_commit_without_evidence_inconclusive():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _commit(st, 1, 200, 400)
    assert _cls(st, deliver=None, b2=False)["class"] == "INCONCLUSIVE"


def test_a_drone_arriving_at_its_queue_during_the_claim_is_audited():
    st = _states(2)
    for i in range(0, 300):
        st[1][i]["gate"] = {"at_queue": False, "occ": "FREE", "tx": [0, 0]}
    _commit(st, 0, 100, 400)
    _block(st, 1, 300, 400)
    r = _cls(st)
    assert r["arrived_during_claim"] and r["class"] == "SATISFIED"


def test_the_physical_entries_include_an_entry_that_was_not_admitted():
    class Gate:
        gate_id = "G"

        def to_gate_frame(self, p):
            return np.asarray(p, float)

    from holo_fleet.config import DEFAULT

    st = _states(2)
    _commit(st, 0, 100, 400)                 # committed, never admitted
    P = np.zeros((len(T), 2, 3))
    P[:, 0, 0] = np.where(T < 20.0, -3.0, 0.0)
    P[:, 1, 0] = -5.0
    r = audit(["d0", "d1"], T, P, st, [Gate()], DEFAULT, {"enabled": True, "period_s": 1.0, "latency_s": 0.15})
    assert r["entry_counts"]["PUSHED"] == 1 and r["entries_with_another_inside"] == 0
