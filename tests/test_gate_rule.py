"""Randomised checks of the communication-free priority rule (complements the Z3 proofs)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.ha.gate_rule import BACKOFFS, decide

G, EPS = DEFAULT.gate, DEFAULT.env.eps_rel


def _pair(rng):
    true = rng.uniform([-4, -5, -1.2], [4, 5, 1.2])
    ei = rng.uniform(-EPS, EPS, 3)
    ej = rng.uniform(-EPS, EPS, 3)
    return decide(*(true + ei), G, EPS), decide(*(-true + ej), G, EPS)


def test_never_double_commit_and_never_mutual_wait():
    rng = np.random.default_rng(0)
    for _ in range(50000):
        a, b = _pair(rng)
        assert not (a == "COMMIT" and b == "COMMIT")
        assert not (a.startswith("WAIT") and b.startswith("WAIT"))


def test_planned_queue_geometry_has_no_needless_backoff():
    # side slots at the queue line (lateral +-2.6), centre slot 2.3 m behind
    assert decide(0.0, 5.2, 0.0, G, EPS) == "COMMIT"          # left side drone goes first
    assert decide(0.0, -5.2, 0.0, G, EPS) == "WAIT_ROBUST"
    assert decide(2.3, 2.6, 0.0, G, EPS) == "COMMIT"
    assert decide(-2.3, -2.6, 0.0, G, EPS) == "WAIT_ROBUST"   # centre drone waits, no retreat
    assert decide(-2.3, 2.6, 0.0, G, EPS) == "WAIT_ROBUST"


def test_some_drone_acts_in_every_sampled_tie():
    rng = np.random.default_rng(1)
    for _ in range(20000):
        true = rng.uniform([-0.8, -0.8, -0.8], [0.8, 0.8, 0.8])
        a = decide(*(true + rng.uniform(-EPS, EPS, 3)), G, EPS)
        b = decide(*(-true + rng.uniform(-EPS, EPS, 3)), G, EPS)
        assert a == "COMMIT" or b == "COMMIT" or a in BACKOFFS or b in BACKOFFS
