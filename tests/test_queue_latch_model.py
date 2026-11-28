"""The premise-Q latch model of formal/check_mutex.py (M3q / M3hq / M4dq) without the BMC, and the campaign audit's check
that the queue plans it judges premise Q2 against are the recorded ones.

The catalogue run takes about two hours; these tests catch a change to the samples or to the sign of the outward offset in
seconds.  Premise Q: Q1, the navigation error of a queued drone is at most 1.0 m per gate axis (estimate within ``queue_tol``
of the queue point, ``at_queue``); Q2, its FRONT sonar is at most QUEUE_OUTWARD_M outward of the queue point.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "formal"))
sys.path.insert(0, str(ROOT / "scripts"))

from audit_final_campaign import plans_reproduced  # noqa: E402
from check_mutex import _q_samples, geometry  # noqa: E402

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.referee.robustness_metrics import FRONT_LEVER_M, QUEUE_CLS_ERR_M, QUEUE_OUTWARD_M  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402

G = DEFAULT.gate
B, Q, LIM = QUEUE_CLS_ERR_M, G.queue_tol, QUEUE_OUTWARD_M
HEAD = float(G.heading_tol_deg)
EPS, ROUND = 1e-9, 1e-4


def _exposure_s(est_off, err, view_l=3.5, n=3):
    g = geometry(DEFAULT, n, err=err, est_off=est_off, cone_extra_deg=HEAD)
    v = next(w for w in g["views"] if abs(w["queue_l"] - view_l) < 1e-6)
    assert v["one_masked_interval"]
    return v["exposure_raw_m"] / G.v_pass


def test_off_axis_samples_stay_in_q1_and_q2_and_reach_the_vertex():
    for view_l in (1.75, 3.5):
        for grid in (False, True):
            smp = _q_samples(G, B, view_l, grid=grid)
            assert smp
            for o0, o1, e0, e1 in smp:
                assert abs(e0) <= B + EPS and abs(e1) <= B + EPS  # Q1
                # at_queue at the FRONT sonar, and Q2 on the true outward offset (o - e, outward positive); the samples
                # are rounded to 0.1 mm
                assert abs(o0) <= Q + EPS and abs(o1) <= Q + FRONT_LEVER_M + ROUND
                assert o1 - e1 <= LIM + ROUND
            assert max(o1 - e1 for _, o1, _, e1 in smp) > LIM - EPS  # Q2 is reached, not avoided
        # the vertex where Q1 and Q2 meet: the estimate inward by b - LIM, the full error, the truth LIM outward
        vertex = tuple(round(x, 9) for x in (Q, LIM - B, -B, -B))
        assert vertex in [tuple(round(x, 9) for x in s) for s in _q_samples(G, B, view_l)]


def test_on_axis_samples_take_the_full_lateral_error_both_ways():
    smp = _q_samples(G, B, 0.0)
    assert {round(e1, 9) for *_, e1 in smp} >= {-B, B}
    assert max(abs(o1) for _, o1, _, _ in smp) > Q + FRONT_LEVER_M - 1e-3


def test_outward_is_away_from_the_axis_and_q2_is_what_keeps_the_exposure():
    vertex = _exposure_s((Q, LIM - B), (-B, -B))
    assert abs(vertex - 0.58) < 0.01  # the M4dq minimum, above the 0.30 s of three captures
    assert abs(_exposure_s((Q, LIM - B), (-B, -B), view_l=-3.5) - vertex) < 1e-9  # the other side of the axis
    assert _exposure_s((Q, LIM - B), (-B, B)) > vertex + 0.5  # the truth inward sees the crossing drone longer
    assert _exposure_s((Q, B - LIM), (-B, -B)) < 0.05  # 1.3 m outward, beyond Q2: no exposure (as M3ql)


def _recorded(name):
    sc = SCENARIOS[name](DEFAULT)
    return {"scenario": name, "communication": DEFAULT.beacon.mode,
            "plans": [{"drone": p.drone_id, "slot": p.slot_index, "queue_lateral": p.queue_lateral,
                       "queue_dz": p.queue_dz, "queue_s": round(p.queue_s, 3)} for p in sc.plans]}


def test_the_audit_rejects_queue_plans_the_scenario_no_longer_reproduces():
    rc = _recorded("line_parallel_mutex")
    assert rc["plans"] and plans_reproduced(rc)
    shifted = {**rc, "plans": [{**p, "queue_lateral": p["queue_lateral"] + 0.05} if i == 0 else p
                               for i, p in enumerate(rc["plans"])]}
    assert not plans_reproduced(shifted)
    assert not plans_reproduced({**rc, "plans": rc["plans"][1:]})
    assert not plans_reproduced({**rc, "plans": [{**p, "slot": p["slot"] + 1} for p in rc["plans"]]})
