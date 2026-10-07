"""Communication-free critical-region rule on sector patterns (sampled checks; the proofs are in
formal/check_mutex.py)."""

import math
import random

from holo_fleet.config import DEFAULT
from holo_fleet.ha.gate_rule import PRIORITY, RANK, WAIT, classify_pattern, decide

AX = {"FRONT": 0.0, "LEFT": 90.0, "REAR": 180.0, "RIGHT": -90.0}


def pattern(beta, fuzz, rng):
    """Horizontal sectors that see a neighbour at bearing beta; within +-fuzz of an edge: random."""
    out = set()
    for s, a in AX.items():
        d = abs((beta - a + 180.0) % 360.0 - 180.0)
        if d <= 60.0 - fuzz or (d < 60.0 + fuzz and rng.random() < 0.5):
            out.add(s)
    return out


def test_table():
    assert classify_pattern({"FRONT"}) == WAIT and classify_pattern({"LEFT"}) == WAIT
    assert classify_pattern({"FRONT", "RIGHT"}) == WAIT and classify_pattern({"LEFT", "REAR"}) == WAIT
    assert classify_pattern({"RIGHT"}) == PRIORITY and classify_pattern({"REAR"}) == PRIORITY
    assert classify_pattern({"REAR", "RIGHT"}) == PRIORITY
    assert classify_pattern({"UP"}) == RANK and classify_pattern({"DOWN"}) == RANK


def test_never_both_priority_within_the_heading_tolerance():
    rng = random.Random(0)
    dmax = 2 * DEFAULT.gate.heading_tol_deg
    fuzz = (30.0 - dmax) / 2.0
    for _ in range(40000):
        beta, delta = rng.uniform(-180, 180), rng.uniform(-dmax, dmax)
        pi = classify_pattern(pattern(beta, fuzz, rng))
        pj = classify_pattern(pattern(beta + 180.0 - delta, fuzz, rng))
        assert not (pi == PRIORITY and pj == PRIORITY), (beta, delta)


def test_abreast_queue_leftmost_goes_first():
    G = DEFAULT.gate
    for n in range(2, 7):
        lats = G.queue_laterals(n)
        decisions = []
        for i in range(n):
            rels = [classify_pattern({"LEFT"} if lats[j] > lats[i] else {"RIGHT"})
                    for j in range(n) if j != i and abs(lats[j] - lats[i]) - 0.6 <= G.queue_bracket_m]
            decisions.append(decide(rels, my_rank=i))
        assert decisions[0] == PRIORITY and all(d == WAIT for d in decisions[1:]), (n, decisions)


def test_static_rank_only_for_stacked_neighbours():
    assert decide([RANK], my_rank=2, their_ranks=[0]) == "WAIT_RANK"
    assert decide([RANK], my_rank=0, their_ranks=[1]) == PRIORITY
    assert decide([PRIORITY, RANK], my_rank=1, their_ranks=[0]) == "WAIT_RANK"


def test_queue_geometry_is_far_enough_back():
    G = DEFAULT.gate
    for n in range(2, 7):
        q, lats = G.queue_s(n), G.queue_laterals(n)
        l_max = max(abs(x) for x in lats)
        bearing = math.degrees(math.atan2(l_max + G.cr_half_width, -q - 0.24 - G.cr_half_len))
        assert bearing + G.heading_tol_deg + 5.0 <= 60.0 + 1e-9, n
