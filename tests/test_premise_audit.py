"""Premise C audit (holo_fleet/referee/premise_audit.py) on synthetic logs: the integer-tick TDMA and delivery timing, the two
separate verdicts (knowledge, blocking), every commit edge of a claim, the end of a claim, concurrent resolutions."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.referee import premise_audit as pa
from holo_fleet.referee.premise_audit import Link, audit, beacon_of_run, earliest_use, first_claim_emission, ticks

T = np.round(np.arange(0.0, 80.0, 0.1), 2)
B2 = {"enabled": True, "payload": True, "period_s": 1.0, "latency_s": 0.10, "latency_max_s": 0.4}
B0 = {"enabled": False}


class Gate:
    gate_id = "G"

    def to_gate_frame(self, p):
        return np.asarray(p, float)


def _states(n, m=len(T)):
    return [[{"t": float(T[i]), "mode": "MUTEX_YIELD", "committed": False,
              "gate": {"at_queue": True, "occ": "FREE", "has_prio": False, "tx": [0, 0]}} for i in range(m)] for _ in range(n)]


def _g(st, k, i, **kw):
    st[k][i]["gate"] = {**st[k][i]["gate"], **kw}


def _commit(st, k, i0, i1, admit_at=None, passed_at=None, edge="pass_done"):
    """k committed over records [i0, i1): claim (tx 1, then 2 from the admission) until passed_at (tx 3 from there on)."""
    for i in range(i0, len(T)):
        if i < i1:
            st[k][i]["committed"] = True
        ph = 1 if i < i1 else (3 if edge == "pass_done" else 0)
        if admit_at is not None and admit_at <= i < i1:
            ph = 2
        if passed_at is not None and i >= passed_at:
            ph = 3
        _g(st, k, i, tx=[0, ph])
    if admit_at is not None:
        _g(st, k, admit_at, entry="ADMIT")
    if i1 < len(T):
        st[k][i1]["transition"] = {"edge": edge}


def _set(st, j, i0, i1, **kw):
    for i in range(i0, i1):
        _g(st, j, i, **kw)


def _P(n, inside=None):
    """Drones 3 m apart, 5 m before the CR box; ``inside``: {drone: (i0, i1)} records with the drone at the gate centre."""
    P = np.zeros((len(T), n, 3))
    for k in range(n):
        P[:, k] = [-5.0, 3.0 * k, 0.0]
    for k, (a, b) in (inside or {}).items():
        P[a:b, k] = [0.0, 0.0, 0.0]
    return P


def _rows(st, beacon=B2, P=None, k=None, j=None, ids=None):
    n = len(st)
    r = audit([f"d{x}" for x in range(n)], T, _P(n) if P is None else P, st, [Gate()], DEFAULT, beacon, identities=ids)
    return [p for p in r["pairs"] if (k is None or p["claimant"] == f"d{k}") and (j is None or p["drone"] == f"d{j}")]


def test_the_audit_schedule_is_the_emission_schedule_of_the_simulator_transport():
    # sim/beacon.BeaconTransport.before_tick driven over 4 periods: every emission tick of every drone is predicted by Link
    import types

    from holo_fleet.config import BeaconConfig
    from holo_fleet.sim import holo_env
    from holo_fleet.sim.beacon import BeaconTransport

    assert holo_env.TICKS_PER_SEC == pa.TICKS_PER_SEC
    env = types.SimpleNamespace(_client=types.SimpleNamespace(command_center=types.SimpleNamespace(enqueue_command=lambda c: None)))
    for n in (2, 3, 4, 5, 6):
        names = [f"drone_{k}" for k in range(n)]
        bt = BeaconTransport(names, BeaconConfig(), seed=1, ticks_per_sec=pa.TICKS_PER_SEC)
        for tick in range(1, 4 * 30 + 1):
            bt.before_tick(env, round(tick / pa.TICKS_PER_SEC, 6))
        seen = {k: [int(round(t * pa.TICKS_PER_SEC)) for t, nm in bt.tx_log if nm == names[k]] for k in range(n)}
        link = Link(n=n)
        for k in range(n):
            pred, tk = [], 0
            while True:
                tk = link.next_emission(k, tk)
                if tk > 120:
                    break
                pred.append(tk)
            assert seen[k] == pred, (n, k, seen[k], pred)


def test_tdma_slots_are_integer_ticks_four_drones_slot_three_at_21_ticks():
    # 30 ticks per period, 4 drones: slot_ticks = 30 // 4 = 7, slot 3 at 0.700 s (a float slot of 0.25 s would say 0.75 s)
    st = _states(4)
    _commit(st, 3, 100, 400)                 # committed in the record of 10.0 s; the 10.7 s ping carries it
    assert abs(first_claim_emission(T, st[3], 3, 100, 1.0, 4) - 10.7) < 1e-6
    assert Link(n=4).slot_ticks == 7 and Link(n=3).slot_ticks == 10


def test_a_ping_carries_the_phase_of_the_control_step_before_it():
    st = _states(2)
    _commit(st, 0, 100, 400)                 # committed at 10.0 s: misses the 10.0 s ping (taken after the 9.9 s step)
    assert abs(first_claim_emission(T, st[0], 0, 100, 1.0, 2) - 11.0) < 1e-6
    st = _states(2)
    _commit(st, 0, 99, 400)
    assert abs(first_claim_emission(T, st[0], 0, 99, 1.0, 2) - 10.0) < 1e-6


def test_earliest_use_adds_propagation_delivery_tick_control_step_and_the_stale_rule():
    ctrl = ticks(T)
    link = Link(n=2, latency_s=0.10)
    assert earliest_use(300, 3.0, link, ctrl) == 306          # 10.0 + 0.002 + 0.10 -> tick 304 -> control step 10.2 s
    assert earliest_use(300, 60.0, link, ctrl) is None        # beyond max_range_m
    assert earliest_use(300, 3.0, Link(n=2, blackout=True), ctrl) is None
    assert earliest_use(300, None, link, ctrl) == pa.UNKNOWN
    # 0.35 s latency, 3 drones: slot 0 (tick 300) used at 10.4 s, age 0.398 s; slot 2 (tick 320) only at 11.1 s, age 0.43 s
    lat = Link(n=3, latency_s=0.35)
    assert earliest_use(300, 3.0, lat, ctrl) == 312
    assert earliest_use(320, 3.0, lat, ctrl) is None


def test_a_commit_before_the_claim_could_be_used_is_concurrent_not_a_violation():
    # claim emitted at 10.0 s, usable at 10.2 s (not at 10.1 s = emission + latency): a commit at 10.1 s is concurrent
    # both hold at the stage-2 check; drone 1 gives up at 20 s, drone 0 is admitted at 22 s, after the end of drone 1's claim
    st = _states(2)
    _commit(st, 0, 99, 300, admit_at=220, passed_at=250)
    _commit(st, 1, 101, 200, edge="pass_abort")
    _set(st, 1, 103, 200, occ="BUSY", entry_ok=False)
    (r,) = _rows(st, k=0, j=1)
    assert r["knowledge"] == "BEFORE_USE" and r["class"] == "CONCURRENT" and r["resolution"] == "WITHDRAWAL", r
    st[0][150]["gate"]["entry"] = "ADMIT"   # admitted at 15 s, while drone 1's claim could be used at drone 0
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "VIOLATED" and "claimant admitted" in r["why"]


def test_a_release_at_the_commit_sample_is_a_commit_like_any_other():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 1, 103, 200, occ="BUSY")         # BUSY from 10.3 s until 19.9 s, FREE and committed at 20.0 s
    _commit(st, 1, 200, 400)
    (r,) = _rows(st, k=0, j=1)
    assert r["latch_before_commit"] == "BUSY"
    assert r["knowledge"] == "NOT_HELD" and r["blocking"] == "VIOLATED" and r["class"] == "VIOLATED", r
    assert r["uses_before_commit"] == 10


def test_every_recommitment_during_a_claim_is_classified():
    st = _states(2)
    _commit(st, 0, 99, 500)
    _commit(st, 1, 101, 140, edge="pass_abort")   # concurrent, withdrawn
    _commit(st, 1, 180, 300)                      # recommitted while drone 0's claim could be used
    rows = _rows(st, k=0, j=1)
    assert [r["t_commit"] for r in rows] == [10.1, 18.0]
    assert rows[0]["knowledge"] == "BEFORE_USE" and rows[1]["knowledge"] == "NOT_HELD"
    assert rows[1]["class"] == "VIOLATED"


def test_the_claim_ends_when_its_end_can_be_used_not_at_the_logged_end_of_the_commitment():
    st = _states(2)
    _commit(st, 0, 99, 300, admit_at=150, passed_at=250)   # reports PASSED from 25.0 s, committed until 30.0 s
    _set(st, 1, 103, 255, occ="BUSY")
    _commit(st, 1, 270, 400)                                # commits at 27.0 s, after the end of the claim could be used
    (r,) = _rows(st, k=0, j=1)
    assert r["t_commit"] is None and r["class"] == "SATISFIED" and r["knowledge"] == "NO_COMMIT" and r["basis"] == "BLOCKED"
    assert r["t_claim_end_usable"] == 26.2


def test_a_claim_never_usable_is_undeliverable_and_a_commit_needs_an_exit_witness():
    st = _states(3)
    _commit(st, 2, 99, 400)                   # slot 2 of 3 at 0.35 s latency: never accepted
    _commit(st, 1, 200, 400)
    lat = {**B2, "latency_s": 0.35}
    (r,) = _rows(st, beacon=lat, k=2, j=1)
    assert r["knowledge"] == "UNDELIVERABLE" and r["class"] == "VIOLATED"
    _set(st, 1, 150, 190, occ="EXITING")
    P = _P(3, inside={2: (120, 160)})         # drone 2 crossed the CR box and left it before drone 1's commit
    (r,) = _rows(st, beacon=lat, P=P, k=2, j=1)
    assert r["knowledge"] == "UNDELIVERABLE" and r["blocking"] == "SATISFIED" and r["basis"] == "EXIT_WITNESSED"


def test_b0_handoff_after_an_exit_witness_is_satisfied_even_with_a_release_before_the_commit():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _set(st, 1, 103, 180, occ="BUSY")
    _set(st, 1, 180, 190, occ="EXITING")      # exit witness, FREE from 19.0 s, commit at 20.0 s while drone 0 is committed
    _commit(st, 1, 200, 400)
    P = _P(2, inside={0: (150, 175)})
    (r,) = _rows(st, beacon=B0, P=P, k=0, j=1)
    assert r["knowledge"] == "NOT_APPLICABLE" and r["class"] == "SATISFIED" and r["basis"] == "EXIT_WITNESSED"
    (r,) = _rows(st, beacon=B0, P=_P(2), k=0, j=1)   # drone 0 never crossed: not explained by M3
    assert r["class"] == "INCONCLUSIVE"


def test_concurrent_admission_while_the_other_claim_could_be_used_is_a_violation():
    st = _states(2)
    _commit(st, 0, 99, 300, admit_at=150, passed_at=250)
    _commit(st, 1, 101, 300, admit_at=160, passed_at=260)   # admitted at 16 s while drone 0's claim could be used
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "VIOLATED" and "drone admitted while the claimant's claim could be used" in r["why"]


def test_concurrent_episode_that_ends_without_admission_is_reported():
    st = _states(2)
    _commit(st, 0, 99, 300, admit_at=220, passed_at=250)
    _commit(st, 1, 101, 200, edge="pass_done")               # crossed without admission (pushed)
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "CONCURRENT" and r["resolution"] == "PASSED_WITHOUT_ADMISSION" and r["ends"][1] == "PASSED_WITHOUT_ADMISSION"


def test_two_commitments_at_the_same_sample_form_one_row():
    st = _states(2)
    _commit(st, 0, 99, 200, edge="pass_abort")
    _commit(st, 1, 99, 300, admit_at=250, passed_at=280)
    rows = _rows(st)
    assert len([r for r in rows if r.get("t_commit") is not None]) == 1


def test_a_queued_drone_that_never_commits_is_satisfied_and_knowledge_is_not_certified():
    st = _states(2)
    _commit(st, 0, 99, 500)
    _set(st, 1, 450, 500, occ="BUSY")
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "SATISFIED" and r["knowledge"] == "NO_COMMIT" and r["steps"]["BLOCKED"] == 50


def test_the_logs_ending_while_the_claim_holds_is_inconclusive():
    st = _states(2)
    _commit(st, 0, 99, len(T))
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "INCONCLUSIVE"


def test_beacon_of_run_blackout_and_payload():
    # ``payload: false`` is read from the run_config of an archived run of the payload-free ablation (no such option exists any more)
    rc = lambda **b: {"fleet_config": {"beacon": {"mode": "ACOUSTIC_RB", **b}, "acoustic": {"latency_max_s": 0.4}}}
    assert beacon_of_run(rc(channel="BLACKOUT"))["blackout"] is True
    no_payload = {"payload": False}
    assert pa.link_of(beacon_of_run(rc(channel="MEDIUM", **no_payload)), 3, DEFAULT) is None
    assert pa.link_of(beacon_of_run(rc(channel="MEDIUM")), 3, DEFAULT).latency_s == 0.25
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 400)
    (r,) = _rows(st, beacon=beacon_of_run(rc(channel="BLACKOUT")), k=0, j=1)
    assert r["knowledge"] == "UNDELIVERABLE"


def test_the_physical_entries_include_an_entry_that_was_not_admitted():
    st = _states(2)
    _commit(st, 0, 100, 400)                 # committed, never admitted
    P = np.zeros((len(T), 2, 3))
    P[:, 0, 0] = np.where(T < 20.0, -3.0, 0.0)
    P[:, 1, 0] = -5.0
    r = audit(["d0", "d1"], T, P, st, [Gate()], DEFAULT, B2)
    assert r["entry_counts"]["PUSHED"] == 1 and r["entries_with_another_inside"] == 0


def test_both_admitted_before_either_could_learn_is_a_violation_whatever_the_order():
    # drone 0 commits at 9.8 s and is admitted at 9.9 s; drone 1 commits at 10.0 s and is admitted at 10.1 s; drone 0's claim
    # can first be used at drone 1 at 10.2 s, drone 1's at drone 0 at 10.7 s
    st = _states(2)
    _commit(st, 0, 98, 300, admit_at=99, passed_at=250)
    _commit(st, 1, 100, 300, admit_at=101, passed_at=260)
    (r,) = _rows(st, k=0, j=1)
    assert r["t_claim_usable"] == 10.2 and r["knowledge"] == "BEFORE_USE"
    assert r["class"] == "VIOLATED" and "both admitted before either claim could be used" in r["why"], r


def test_a_concurrent_pair_that_both_withdraw_is_reported():
    st = _states(2)
    _commit(st, 0, 99, 200, edge="pass_abort")
    _commit(st, 1, 101, 210, edge="pass_abort")
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "CONCURRENT" and r["resolution"] == "BOTH_WITHDRAWN"


def test_a_dead_claimant_ends_its_claim_when_the_receiver_lists_its_slot():
    # drone 0 flies slot 1 and drone 1 slot 0; drone 0 dies committed at 20 s, drone 2 declares slot 1 dead at 25 s and
    # commits at 26 s: after the end of the claim at drone 2
    st = _states(3)
    _commit(st, 0, 99, 400)
    for i in range(200, len(T)):
        st[0][i]["mode"] = "DEAD"
    _set(st, 2, 250, len(T), dead=[1])
    _commit(st, 2, 260, 400)
    ids = {0: {1: 0, 2: 2}, 1: {0: 1, 2: 2}, 2: {0: 1, 1: 0}}
    (r,) = _rows(st, k=0, j=2, ids=ids)
    assert r["t_claim_end_usable"] == 25.0 and r["t_commit"] is None and r["class"] == "SATISFIED", r
    _set(st, 2, 250, len(T), dead=[0])        # slot 0 is drone 1: drone 0's claim still holds at drone 2
    (r,) = _rows(st, k=0, j=2, ids=ids)
    assert r["knowledge"] == "NOT_HELD" and r["class"] == "VIOLATED"


def test_a_receiver_that_ignores_the_claimants_code_cannot_use_its_claim():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 400)
    (r,) = _rows(st, k=0, j=1, ids={0: {}, 1: {}})
    assert r["knowledge"] == "UNDELIVERABLE"


def test_a_claim_of_another_gate_index_is_not_audited_as_a_competing_claim():
    st = _states(2)
    _commit(st, 0, 99, 400)
    for i in range(len(T)):
        st[0][i]["gate"]["tx"] = [1, st[0][i]["gate"]["tx"][1]]
    _commit(st, 1, 200, 400)
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "INCONCLUSIVE" and r["knowledge"] == "UNKNOWN" and "gate index" in r["why"], r


def test_a_claim_withdrawn_before_its_first_ping_makes_a_same_sample_commit_concurrent():
    st = _states(2)
    _commit(st, 0, 100, 105, edge="pass_abort")      # first ping of drone 0 at 11.0 s, withdrawn at 10.5 s
    _commit(st, 1, 100, 300, admit_at=150, passed_at=250)
    (r,) = _rows(st, k=0, j=1)
    assert r["claim_emissions"] == 0 and r["knowledge"] == "NOT_EMITTED"
    assert r["class"] == "CONCURRENT" and r["resolution"] == "WITHDRAWAL", r


def test_a_claim_whose_first_ping_is_after_the_logs_is_inconclusive():
    st = _states(2)
    _commit(st, 0, 795, len(T))                      # 79.5 s: the first ping (80.0 s) is beyond the logs
    _commit(st, 1, 796, len(T))
    (r,) = _rows(st, k=0, j=1)
    assert r["class"] == "INCONCLUSIVE" and r["knowledge"] == "UNKNOWN", r


def test_missing_geometry_is_inconclusive():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 400)
    rows = pa.classify(ticks(T), st, pa.Geometry(T, None), Link(n=2), 0, (99, 400), 1, len(T))
    assert [(r["knowledge"], r["class"]) for r in rows] == [("UNKNOWN", "INCONCLUSIVE")]


def test_a_b2_run_with_unknown_channel_latency_is_inconclusive():
    st = _states(2)
    _commit(st, 0, 99, 400)
    _commit(st, 1, 200, 400)
    (r,) = _rows(st, beacon={"enabled": True, "payload": True, "latency_s": None}, k=0, j=1)
    assert r["class"] == "INCONCLUSIVE" and "channel latency" in r["why"]
