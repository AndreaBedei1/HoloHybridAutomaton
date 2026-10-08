"""The guards executed at runtime and the guards verified in Z3 are the same functions and agree."""

import itertools
import random
import sys
from pathlib import Path

import z3

from holo_fleet.config import DEFAULT
from holo_fleet.ha import gate_rule
from holo_fleet.ha.automaton import AbstractObservation, LocalHybridAutomaton
from holo_fleet.ha.observation_invariants import consistent
from holo_fleet.ha.spec import BOOL_VARS, MODES, PY_LOGIC, REAL_VARS, build_edges

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
from common import Obs, Z3L  # noqa: E402


def _random_obs(rng):
    return AbstractObservation(
        d_min=rng.uniform(-0.5, 4.0), form_err=rng.uniform(0, 2.0), t_ok=rng.uniform(0, 4.0),
        sense_ok=rng.random() < 0.9, env_ok=rng.random() < 0.9, gate_zone=rng.random() < 0.5,
        at_queue=rng.random() < 0.5, occ_busy=rng.random() < 0.5, has_prio=rng.random() < 0.5,
        passed=rng.random() < 0.3, neighbors_ok=rng.random() < 0.8, committed=rng.random() < 0.3)


def test_exactly_one_edge_enabled_runtime():
    # every Boolean combination, consistent with the observation invariants or not: the guards partition
    # the observation space by themselves (formal O4/O5); the invariants are not needed for determinism
    rng = random.Random(1)
    ha = LocalHybridAutomaton()
    n_consistent = 0
    for _ in range(5000):
        o = _random_obs(rng)
        n_consistent += consistent(o)
        ha.mode = rng.choice(MODES)
        ha.committed = o.committed
        ha.step(o, 0.0, 0.1)
    assert ha.determinism_violations == 0
    assert 0 < n_consistent < 5000                  # both kinds were exercised


def test_python_and_z3_evaluations_agree():
    rng = random.Random(2)
    edges = build_edges(DEFAULT)
    sym = Obs()
    for _ in range(300):
        o = _random_obs(rng)
        src = rng.choice(MODES)
        subs = [(getattr(sym, v), z3.RealVal(str(getattr(o, v)))) for v in REAL_VARS] + \
               [(getattr(sym, v), z3.BoolVal(bool(getattr(o, v)))) for v in BOOL_VARS]
        for e in edges[src]:
            py = bool(e.guard(o, PY_LOGIC))
            zv = z3.is_true(z3.simplify(z3.substitute(e.guard(sym, Z3L), *subs)))
            assert py == zv, (src, e.name, o)


def test_gate_rule_python_and_z3_agree_on_every_pattern():
    names = ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")
    for flags in itertools.product((False, True), repeat=6):
        pattern = {n for n, f in zip(names, flags) if f}
        py = gate_rule.classify_pattern(pattern)
        rel = gate_rule.relation(*[z3.BoolVal(f) for f in flags], Z3L)
        zt = [k for k in (gate_rule.WAIT, gate_rule.PRIORITY, gate_rule.RANK, "NONE") if z3.is_true(z3.simplify(rel[k]))]
        assert zt == [py], (pattern, zt, py)
