"""The guards executed at runtime and the guards verified in Z3 are the same functions and agree."""

import random

import pytest
import z3

from holo_fleet.config import DEFAULT
from holo_fleet.ha import gate_rule
from holo_fleet.ha.automaton import AbstractObservation, LocalHybridAutomaton
from holo_fleet.ha.spec import BOOL_VARS, MODES, PY_LOGIC, REAL_VARS, build_edges

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
from common import Obs, Z3L  # noqa: E402


def _random_obs(rng):
    return AbstractObservation(
        d_min=rng.uniform(-0.5, 4.0), form_err=rng.uniform(0, 2.0), t_ok=rng.uniform(0, 4.0),
        sense_ok=rng.random() < 0.9, env_ok=rng.random() < 0.9, gate_zone=rng.random() < 0.5,
        at_queue=rng.random() < 0.5, occ_busy=rng.random() < 0.5, has_prio=rng.random() < 0.5,
        passed=rng.random() < 0.3, neighbors_ok=rng.random() < 0.8, committed=rng.random() < 0.3)


def test_exactly_one_edge_enabled_runtime():
    rng = random.Random(1)
    ha = LocalHybridAutomaton()
    for _ in range(5000):
        o = _random_obs(rng)
        if o.passed:                     # perception guarantee (see formal/common.Obs.legal)
            o.at_queue, o.gate_zone = True, False
        ha.mode = rng.choice(MODES)
        ha.committed = o.committed
        ha.step(o, 0.0, 0.1)
    assert ha.determinism_violations == 0


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


def test_gate_rule_python_and_z3_agree():
    rng = random.Random(3)
    G, eps = DEFAULT.gate, DEFAULT.env.eps_rel
    ms, ml, mz = z3.Reals("ms ml mz")
    zabs = lambda x: z3.If(x >= 0, x, -x)  # noqa: E731
    P = gate_rule.decision_predicates(ms, ml, mz, G, eps, Z3L, zabs)
    for _ in range(500):
        ds, dl, dz = rng.uniform(-4, 4), rng.uniform(-5, 5), rng.uniform(-1.5, 1.5)
        py = gate_rule.decide(ds, dl, dz, G, eps)
        subs = [(ms, z3.RealVal(str(ds))), (ml, z3.RealVal(str(dl))), (mz, z3.RealVal(str(dz)))]
        zt = [k for k, v in P.items() if z3.is_true(z3.simplify(z3.substitute(v, *subs)))]
        assert zt == [py], (ds, dl, dz, zt, py)
