"""Discrete-structure checks of the local hybrid automaton (Z3, exhaustive over observations).

The guards are NOT re-implemented here: ``holo_fleet.ha.spec.build_edges`` is
instantiated with Z3 symbols, i.e. the very functions executed at runtime.

Checks, for every source mode:
  D1  local determinism  - no two edges enabled on the same legal observation;
  D2  completeness       - some edge (possibly a self-loop) is always enabled;
  H1  fault  => FAILSAFE                         (top priority);
  H2  no fault & d_min < d_ca => COLLISION_AVOIDANCE (collision avoidance prevails over the gate);
  H3  calm & d_ca_exit <= d_min < d_warning => SEPARATION_WARNING;
  G1  the only way into GATE_PASS without being committed is the commit edge, whose guard implies
      at_queue & !occ_busy & has_prio (local mutual-exclusion entry condition);
  G2  a committed drone never falls back to GATE_APPROACH/GATE_YIELD (no oscillation PASS->YIELD);
  G3  a hazard (d_min < d_warning) never coexists with a commit decision;
  M*  mutation tests: deliberately broken automata must produce counterexamples (non-vacuity).
"""

from __future__ import annotations

import itertools
import sys

import z3

from common import Obs, Report, Z3L, check  # noqa: E402  (formal/ is on sys.path when run as a script)

from holo_fleet.config import DEFAULT
from holo_fleet.ha.spec import MODES, Edge, Mode, Predicates, build_edges

ENC = "holo_fleet/ha/spec.py (shared guards) + formal/check_determinism.py"


def guards_of(edges, o):
    return [(e, e.guard(o, Z3L)) for e in edges]


def run(cfg=DEFAULT, verbose=True) -> Report:
    rep = Report("determinism")
    edges = build_edges(cfg)
    P = Predicates(cfg)
    for src in MODES:
        o = Obs()
        legal = o.legal(cfg)
        gs = guards_of(edges[src], o)
        # D1 pairwise disjointness
        r = check(f"D1 determinism [{src.value}]",
                  "forall legal obs: at most one outgoing guard enabled", ENC,
                  [legal, z3.Or(*[z3.And(g1, g2) for (_, g1), (_, g2) in itertools.combinations(gs, 2)])])
        if not r.passed:      # name the overlapping edges to make the counterexample actionable
            r.note = "overlapping: " + ", ".join(
                f"{e1.name}&{e2.name}" for (e1, g1), (e2, g2) in itertools.combinations(gs, 2)
                if check("", "", ENC, [legal, g1, g2]).verdict != "unsat")
        rep.add(r, verbose)
        # D2 completeness
        rep.add(check(f"D2 completeness [{src.value}]", "forall legal obs: some guard enabled", ENC,
                      [legal, z3.Not(z3.Or(*[g for _, g in gs]))]), verbose)
        # H1 fault => FAILSAFE
        bad = [g for e, g in gs if e.target != Mode.FAILSAFE_HOLD_OR_RETREAT]
        rep.add(check(f"H1 fault=>FAILSAFE [{src.value}]", "fault -> target = FAILSAFE", ENC,
                      [legal, P.fault(o, Z3L), z3.Or(*bad)]), verbose)
        # H2 collision risk => CA (from every mode, including GATE_PASS)
        bad = [g for e, g in gs if e.target != Mode.COLLISION_AVOIDANCE]
        rep.add(check(f"H2 d<d_ca=>COLLISION_AVOIDANCE [{src.value}]",
                      "!fault & d_min < d_ca -> target = COLLISION_AVOIDANCE", ENC,
                      [legal, z3.Not(P.fault(o, Z3L)), o.d_min < cfg.sep.d_ca, z3.Or(*bad)]), verbose)
        # H3 warning band => SW
        bad = [g for e, g in gs if e.target != Mode.SEPARATION_WARNING]
        rep.add(check(f"H3 warning band=>SEPARATION_WARNING [{src.value}]",
                      "!fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING", ENC,
                      [legal, z3.Not(P.fault(o, Z3L)), o.d_min >= cfg.sep.d_ca_exit, o.d_min < cfg.sep.d_warning,
                       z3.Or(*bad)]), verbose)
        # G1 entering GATE_PASS uncommitted requires the mutual-exclusion entry condition
        into_gp = [g for e, g in gs if e.target == Mode.GATE_PASS]
        if into_gp:
            rep.add(check(f"G1 GATE_PASS entry condition [{src.value}]",
                          "!committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio", ENC,
                          [legal, z3.Not(o.committed), z3.Or(*into_gp),
                           z3.Not(z3.And(o.at_queue, z3.Not(o.occ_busy), o.has_prio))]), verbose)
        # G2 committed never goes back to approach/yield
        back = [g for e, g in gs if e.target in (Mode.GATE_APPROACH, Mode.GATE_YIELD)]
        if back:
            rep.add(check(f"G2 no PASS->YIELD fallback [{src.value}]",
                          "committed -> target not in {GATE_APPROACH, GATE_YIELD}", ENC,
                          [legal, o.committed, z3.Or(*back)]), verbose)
        # G3 no commit decision under a separation hazard
        commits = [g for e, g in gs if e.set_committed is True]
        if commits:
            rep.add(check(f"G3 no commit under hazard [{src.value}]",
                          "commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW)", ENC,
                          [legal, z3.Or(*commits), o.d_min < cfg.sep.d_warning]), verbose)

    # ------------------------------------------------------------- mutation tests
    def mutated(drop_prio: bool = False, drop_ca_priority: bool = False):
        ed = build_edges(cfg)
        out = {}
        for src, lst in ed.items():
            new = []
            for e in lst:
                if drop_prio and e.name == "commit":
                    new.append(Edge(e.name, lambda o, L, s=src: L.And(P.calm(o, L, s), L.Not(o.committed), o.gate_zone,
                                                                        o.at_queue, L.Not(o.occ_busy)),
                                    e.target, e.set_committed, e.decision))
                elif drop_ca_priority and e.name == "pass_continue" and src == Mode.GATE_PASS:
                    # broken design: "once committed, keep passing whatever happens"
                    new.append(Edge(e.name, lambda o, L: L.And(L.Not(P.fault(o, L)), o.committed, L.Not(o.passed)),
                                    e.target, e.set_committed, e.decision))
                else:
                    new.append(e)
            out[src] = new
        return out

    o = Obs()
    me = mutated(drop_prio=True)
    gs = guards_of(me[Mode.GATE_YIELD], o)
    into_gp = [g for e, g in gs if e.target == Mode.GATE_PASS]
    rep.add(check("M1 mutation: commit without priority must violate G1",
                  "expect counterexample", ENC,
                  [o.legal(cfg), z3.Not(o.committed), z3.Or(*into_gp),
                   z3.Not(z3.And(o.at_queue, z3.Not(o.occ_busy), o.has_prio))], expect="sat"), verbose)
    me = mutated(drop_ca_priority=True)
    gs = guards_of(me[Mode.GATE_PASS], o)
    bad = [g for e, g in gs if e.target != Mode.COLLISION_AVOIDANCE]
    rep.add(check("M2 mutation: gate pass ignoring collision risk must violate H2",
                  "expect counterexample", ENC,
                  [o.legal(cfg), z3.Not(P.fault(o, Z3L)), o.d_min < cfg.sep.d_ca, z3.Or(*bad)], expect="sat"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    n_ok = sum(r.passed for r in rep.results)
    print(f"determinism: {n_ok}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
