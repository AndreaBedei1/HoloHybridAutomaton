"""Premise C audit (holo_fleet/referee/premise_audit.py): behavioural-blocking contract, integer-tick TDMA timing,
stale rejection, and the classification of commits, recommitments, withdrawals, releases and missing evidence."""

import re
from pathlib import Path

import numpy as np

from holo_fleet.referee.premise_audit import (UNKNOWN, Geometry, Link, audit, classify, earliest_use,
                                              first_claim_emission, ticks)

T = np.round(np.arange(0.0, 80.0, 0.1), 2)
CTRL = [int(x) for x in ticks(T)]
M = len(T)


def _states(n):
    return [[{"t": float(T[i]), "mode": "MUTEX_YIELD", "committed": False,
              "gate": {"at_queue": True, "occ": "FREE", "has_prio": False, "tx": [0, 0], "dead": []}} for i in range(M)]
            for _ in range(n)]


def _set(st, k, i0, i1, **g):
    for i in range(i0, i1):
        st[k][i]["gate"] = {**st[k][i]["gate"], **g}


def _commit(st, k, i0, i1, admit_at=None):
    for i in range(i0, i1):
        st[k][i]["committed"] = True
        st[k][i]["mode"] = "MUTEX_PASS"
    _set(st, k, i0, i1, tx=[0, 1])
    if admit_at is not None:
        _set(st, k, admit_at, admit_at + 1, entry="ADMIT")


def _geo(n, d=3.0):
    P = np.zeros((M, n, 3))
    P[:, :, 0] = np.arange(n)[None, :] * d
    return Geometry(T, P)


def _rows(st, k=0, j=1, ep=None, link=None, geo=None):
    n = len(st)
    link = Link(n=n, latency_s=0.25) if link is None else link
    ep = ep or (99, 400)
    return classify(CTRL, st, geo or _geo(n), link, k, ep, j, M)


def test_the_tdma_slots_are_integer_ticks_for_two_three_and_four_drones():
    assert [Link(n=n).slot_ticks for n in (2, 3, 4)] == [15, 10, 7]
    assert Link(n=4).next_emission(3, 300) == 321                  # 10.7 s, not 10.75 s
    assert Link(n=3).next_emission(2, 300) == 320
    assert Link(n=2).next_emission(1, 300) == 315
    assert Link(n=4, period_s=8.0).slot_ticks == 60


def test_the_tick_rate_matches_the_simulator():
    src = (Path(__file__).resolve().parents[1] / "holo_fleet" / "sim" / "holo_env.py").read_text(encoding="utf-8")
    assert int(re.search(r"^TICKS_PER_SEC = (\d+)", src, re.M).group(1)) == 30


def test_a_ping_carries_the_phase_of_the_control_step_before_it():
    st = _states(2)
    _commit(st, 0, 100, 400)                 # committed in the record of 10.0 s: the ping at tick 300 carried 9.9 s
    assert abs(first_claim_emission(T, st[0], 0, 100, 400, 1.0, 2) - 11.0) < 1e-9
    st = _states(2)
    _commit(st, 0, 99, 400)                  # committed at 9.9 s: the ping at 10.0 s carries the claim
    assert abs(first_claim_emission(T, st[0], 0, 99, 400, 1.0, 2) - 10.0) < 1e-9


def test_earliest_use_follows_the_delivery_tick_the_control_step_and_the_stale_rule():
    assert earliest_use(330, 3.0, Link(n=2, latency_s=0.25), CTRL) == 339      # due 11.252 s -> tick 338 -> step 11.3 s
    assert earliest_use(330, 3.0, Link(n=2, latency_s=0.35), CTRL) == 342      # age 0.398 s <= 0.4 s
    assert earliest_use(330, 3.0, Link(n=2, latency_s=0.40), CTRL) is None     # first use at 11.5 s: age 0.498 s, stale
    assert earliest_use(330, 3.0, Link(n=2, latency_s=0.45), CTRL) is None
    assert earliest_use(330, 60.0, Link(n=2, latency_s=0.25), CTRL) is None    # beyond max_range_m
    assert earliest_use(330, None, Link(n=2, latency_s=0.25), CTRL) == UNKNOWN


def test_a_commit_at_the_first_step_the_claim_could_be_used_is_a_violation_one_step_earlier_is_concurrent():
    st = _states(2)
    _commit(st, 0, 99, 400)                  # first claim ping 10.0 s, usable at drone 1 from 10.3 s
    _commit(st, 1, 103, 120)
    assert _rows(st)[0]["class"] == "VIOLATED"
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 102, 120)                 # 10.2 s: before the claim could be used
    r = _rows(st)[0]
    assert r["class"] == "CONCURRENT" and r["resolution"] == "WITHDRAWAL", r


def test_a_release_exactly_at_the_commit_step_is_not_satisfied():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 1, 105, 150, occ="BUSY")
    _commit(st, 1, 150, 200)                 # BUSY until 14.9 s, FREE and committed at 15.0 s
    r = _rows(st)[0]
    assert r["class"] == "VIOLATED" and r["release_at_commit"]


def test_every_recommitment_during_the_claim_is_classified():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 101, 110)                 # concurrent, withdrawn without admission
    _commit(st, 1, 200, 220)                 # recommitted while the claim could be used
    rows = _rows(st)
    assert [r["class"] for r in rows] == ["CONCURRENT", "VIOLATED"]
    assert rows[0]["resolution"] == "WITHDRAWAL"


def test_a_concurrent_drone_admitted_while_the_claim_could_be_used_is_a_violation():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 101, 300, admit_at=150)
    r = _rows(st)[0]
    assert r["class"] == "VIOLATED" and "drone admitted" in r["why"]


def test_concurrent_drones_both_admitted_before_either_could_learn_are_a_violation():
    st = _states(2)
    _commit(st, 0, 100, 300, admit_at=101)
    _commit(st, 1, 101, 300, admit_at=102)
    r = _rows(st, ep=(100, 300))[0]
    assert r["class"] == "VIOLATED", r


def test_a_drone_blocked_from_the_first_use_of_the_claim_to_its_end_is_satisfied():
    st = _states(2)
    _commit(st, 0, 99, 300)
    _set(st, 0, 300, M, tx=[0, 3])           # passed: claim ended, the end is usable at drone 1 shortly after
    _set(st, 1, 0, M, has_prio=True)
    _set(st, 1, 99, 320, occ="BUSY")
    r = _rows(st, ep=(99, 300))[0]
    assert r["class"] == "SATISFIED" and r["basis"] == "BLOCKED", r


def test_a_justified_release_after_the_claim_end_is_usable_does_not_create_a_pair():
    st = _states(2)
    _commit(st, 0, 99, 300)
    _set(st, 0, 300, M, tx=[0, 3])
    _set(st, 1, 0, M, has_prio=True)
    _set(st, 1, 99, 320, occ="BUSY")
    _commit(st, 1, 320, 350)                 # after the end of the claim could be used
    rows = _rows(st, ep=(99, 300))
    assert [r["class"] for r in rows] == ["SATISFIED"]


def test_a_release_and_commit_before_the_end_of_the_claim_is_usable_is_a_violation():
    st = _states(2)
    _commit(st, 0, 99, 300)
    _set(st, 0, 300, M, tx=[0, 3])
    _set(st, 1, 103, 299, occ="BUSY")
    _commit(st, 1, 299, 350)                 # FREE and committed while drone 0 still claimed
    assert _rows(st, ep=(99, 300))[0]["class"] == "VIOLATED"


def test_a_drone_arriving_at_its_queue_during_the_claim_is_audited():
    st = _states(2)
    _set(st, 1, 0, 250, at_queue=False)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 260, 300)
    assert _rows(st)[0]["class"] == "VIOLATED"


def test_unrelated_blocking_before_the_claim_is_usable_does_not_satisfy_a_later_commit():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 1, 90, 101, occ="BUSY")         # a corridor echo before the claim could be used, then released
    _commit(st, 1, 101, 120)
    assert _rows(st)[0]["class"] == "CONCURRENT"
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 1, 0, M, entry_ok=False)        # the stage-2 check closed: no commit blocking
    _commit(st, 1, 200, 220)
    assert _rows(st)[0]["class"] == "VIOLATED"


def test_missing_or_ambiguous_evidence_is_inconclusive():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 0, 99, 400, tx=None)
    assert _rows(st)[0]["class"] == "INCONCLUSIVE"
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 220)
    assert _rows(st, geo=Geometry(T, None))[0]["class"] == "INCONCLUSIVE"
    st = _states(2)
    _commit(st, 0, 99, M)                    # the claim holds until the logs end
    _set(st, 1, 0, M, occ="BUSY")
    assert _rows(st, ep=(99, M))[0]["class"] == "INCONCLUSIVE"
    st = _states(2)
    _commit(st, 0, 99, 300)
    _set(st, 0, 300, M, tx=[0, 3])
    _set(st, 1, 0, M, has_prio=True)         # at the queue, priority, latch FREE, no commit: not explained
    assert _rows(st, ep=(99, 300))[0]["class"] == "INCONCLUSIVE"


def test_without_a_claim_payload_a_commit_is_inconclusive_and_a_blocked_drone_satisfied():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _commit(st, 1, 200, 220)
    assert classify(CTRL, st, _geo(2), None, 0, (100, 400), 1, M)[0]["class"] == "INCONCLUSIVE"
    st = _states(2)
    _commit(st, 0, 100, 400)
    _set(st, 1, 0, M, has_prio=True)
    _set(st, 1, 100, 400, occ="BUSY")
    r = classify(CTRL, st, _geo(2), None, 0, (100, 400), 1, M)[0]
    assert r["class"] == "SATISFIED" and r["basis"] == "BLOCKED"


def test_a_claim_that_can_never_be_used_makes_a_later_commit_a_violation():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 220)
    r = _rows(st, link=Link(n=2, latency_s=0.45))[0]
    assert r["class"] == "VIOLATED" and "never" in r["why"]


def test_four_drones_use_the_integer_slot_of_the_claimant():
    st = _states(4)
    _commit(st, 3, 99, 400)                  # slot 3 of 4: tick 321 after 9.9 s is 10.7 s, usable at 11.0 s
    _commit(st, 0, 109, 130)                 # 10.9 s: before the claim could be used
    _commit(st, 1, 110, 130)                 # 11.0 s: the claim could be used
    g = _geo(4)
    link = Link(n=4, latency_s=0.25)
    assert classify(CTRL, st, g, link, 3, (99, 400), 0, M)[0]["class"] == "CONCURRENT"
    assert classify(CTRL, st, g, link, 3, (99, 400), 1, M)[0]["class"] == "VIOLATED"


def test_a_commit_in_the_same_step_as_the_claimant_is_counted_once():
    st = _states(2)
    _commit(st, 0, 100, 150)
    _commit(st, 1, 100, 150)
    assert len(_rows(st, k=0, j=1, ep=(100, 150))) == 1
    assert all(r.get("t_commit") is None for r in _rows(st, k=1, j=0, ep=(100, 150)))


def test_the_physical_entries_include_an_entry_that_was_not_admitted():
    class Gate:
        gate_id = "G"

        def to_gate_frame(self, p):
            return np.asarray(p, float)

    from holo_fleet.config import DEFAULT

    st = _states(2)
    _commit(st, 0, 100, 400)
    P = np.zeros((M, 2, 3))
    P[:, 0, 0] = np.where(T < 20.0, -3.0, 0.0)
    P[:, 1, 0] = -5.0
    r = audit(["d0", "d1"], T, P, st, [Gate()], DEFAULT, {"enabled": True, "payload": True, "period_s": 1.0,
                                                           "latency_s": 0.25})
    assert r["entry_counts"]["PUSHED"] == 1 and r["entries_with_another_inside"] == 0


def test_at_latency_035_s_the_third_slot_of_three_is_always_stale_and_the_others_are_used():
    link = Link(n=3, latency_s=0.35)
    for k, ok in ((0, True), (1, True), (2, False)):
        e = link.next_emission(k, 300)
        assert (earliest_use(e, 3.0, link, CTRL) is not None) == ok, k


def test_without_a_claim_payload_a_commit_after_a_witnessed_exit_of_the_claimant_is_satisfied():
    st = _states(2)
    _commit(st, 0, 100, 400)
    _set(st, 1, 100, 200, occ="BUSY")
    _set(st, 1, 200, 210, occ="EXITING")
    _commit(st, 1, 210, 260)
    inside = np.zeros((M, 2), bool)
    inside[150:180, 0] = True                # the claimant crossed the CR before the exit witness
    geo = Geometry(T, _geo(2).P, inside)
    r = classify(CTRL, st, geo, None, 0, (100, 400), 1, M)[0]
    assert r["class"] == "SATISFIED" and r["basis"] == "EXIT_WITNESSED" and r["sonar_exit_witnessed"]
    inside[:, 0] = False                     # no crossing in the ground truth: not explained
    assert classify(CTRL, st, Geometry(T, _geo(2).P, inside), None, 0, (100, 400), 1, M)[0]["class"] == "INCONCLUSIVE"


def test_a_blackout_channel_never_lets_a_claim_be_used():
    assert earliest_use(330, 3.0, Link(n=2, latency_s=0.1, blackout=True), CTRL) is None
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 220)
    r = _rows(st, link=Link(n=2, latency_s=0.1, blackout=True))[0]
    assert r["class"] == "VIOLATED" and "BLACKOUT" in r["why"]
