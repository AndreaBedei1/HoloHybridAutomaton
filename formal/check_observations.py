"""Observation consistency: the abstract observation consumed by the automaton (Z3, exhaustive).

Kept distinct from the local-determinism suite (check_determinism.py), which quantifies over the same
domain.  The domain is the conjunction of the semantic invariants of
``holo_fleet/ha/observation_invariants.py`` (N1, I1-I4; N0 is a runtime type check, trivially true
for typed Z3 symbols): the very predicate that the runtime consistency check evaluates between
perception and automaton.

  O1  the invariants are jointly satisfiable;
  O2  each invariant excludes observations admitted by all the others (none is redundant or vacuous);
  O3  non-vacuity: in every mode, every outgoing edge is enabled on some consistent observation, so the
      proofs over the domain are not proofs about an empty or truncated set;
  O4  (A) consistent observation -> exactly one outgoing edge enabled, in every mode;
  O5  runtime monitor: for ANY observation, the monitored one (sense_ok := sense_ok & consistent)
      enables exactly one edge, and an inconsistent one only the fault edge to
      FAILSAFE_HOLD_OR_RETREAT (no new mode);
  Q1  a drone holding its queue point (calm, not committed) is handled by the gate protocol: the
      enabled edge is commit or yield (this one relies on I2);
mutations, each with its expected verdict:
  Om1 without I2, Q1 has a counterexample: at_queue & !gate_zone takes a formation edge (SAT);
  Om2 the v1 domain constraint passed -> at_queue makes pass_done unreachable (UNSAT): the v1
      determinism proof silently excluded every reachable 'passed' observation;
  Om3 without the monitor an inconsistent observation is taken by a mission edge, not by FAILSAFE (SAT).
"""

from __future__ import annotations

import itertools
import sys
import time

import z3

from common import CheckResult, Obs, Report, Z3L, check  # noqa: E402

from holo_fleet.config import DEFAULT
from holo_fleet.ha import observation_invariants as OI
from holo_fleet.ha.spec import MODES, Mode, Predicates, build_edges

ENC = "holo_fleet/ha/observation_invariants.py + holo_fleet/ha/spec.py (shared guards)"
FAILSAFE = Mode.FAILSAFE_HOLD_OR_RETREAT


def not_exactly_one(gs):
    """No guard enabled, or two at once."""
    return z3.Or(z3.Not(z3.Or(*gs)), *[z3.And(a, b) for a, b in itertools.combinations(gs, 2)])


class Monitored:
    """The observation handed to the automaton by the runtime consistency check."""

    def __init__(self, o: Obs, consistent):
        for k, v in o.__dict__.items():
            setattr(self, k, v)
        self.sense_ok = z3.And(o.sense_ok, consistent)


def all_sat(prop: str, formula: str, queries, note: str = "") -> CheckResult:
    """One result for several satisfiability queries (each must be SAT)."""
    t0 = time.time()
    missing = [label for label, cons in queries if check("", "", ENC, cons, expect="sat").verdict != "sat"]
    r = CheckResult(prop=prop, formula=formula, encoding=ENC, expect="sat",
                    verdict="sat" if not missing else "unsat", passed=not missing, note=note)
    r.seconds = round(time.time() - t0, 3)
    if missing:
        r.note = (note + "; " if note else "") + "never enabled: " + ", ".join(missing)
    return r


def queue_violation(cfg, edges, o, legal):
    """Some mode, calm, not committed, at the queue point, enabling an edge other than commit / yield."""
    P = Predicates(cfg)
    bad = []
    for src in MODES:
        gs = [(e, e.guard(o, Z3L)) for e in edges[src]]
        bad.append(z3.And(P.calm(o, Z3L, src), z3.Not(o.committed), o.at_queue,
                          z3.Or(*[g for e, g in gs if e.name not in ("commit", "yield")])))
    return [legal, z3.Or(*bad)]


def run(cfg=DEFAULT, verbose=True) -> Report:
    rep = Report("observations")
    edges = build_edges(cfg)
    o = Obs()
    legal = o.legal(cfg)
    # ---------------------------------------------------------------- O1, O2: the invariants themselves
    rep.add(check("O1 the observation invariants are jointly satisfiable", "exists obs: N1 & I1 & I2 & I3 & I4",
                  ENC, [legal], expect="sat"), verbose)
    for inv in OI.INVARIANTS:
        short = inv.name.split()[0]
        if short == "N0":
            continue                   # type check: trivially true for typed symbols (tested at runtime)
        rep.add(check(f"O2 {inv.name}: excludes observations admitted by the other invariants",
                      f"exists obs: others & !({inv.formula})", ENC,
                      [o.legal(cfg, drop=[short]), z3.Not(inv.pred(o, Z3L, cfg))], expect="sat",
                      note=f"excludes: {inv.excludes}"), verbose)
    # ---------------------------------------------------------------- O3 non-vacuity, O4 exactly one edge
    for src in MODES:
        gs = [(e, e.guard(o, Z3L)) for e in edges[src]]
        rep.add(all_sat(f"O3 non-vacuity [{src.value}]: every edge enabled on some consistent observation",
                        "for each edge e: exists consistent obs with guard(e)",
                        [(e.name, [legal, g]) for e, g in gs], note=f"{len(gs)} edges"), verbose)
    for src in MODES:
        gs = [g for _, g in [(e, e.guard(o, Z3L)) for e in edges[src]]]
        rep.add(check(f"O4 consistent observation -> exactly one edge [{src.value}]",
                      "invariants -> exactly one outgoing guard enabled", ENC, [legal, not_exactly_one(gs)]), verbose)
    # ---------------------------------------------------------------- O5 runtime monitor
    m = Monitored(o, legal)
    bad = []
    for src in MODES:
        gs = [(e, e.guard(m, Z3L)) for e in edges[src]]
        bad.append(z3.Or(not_exactly_one([g for _, g in gs]),
                         z3.And(z3.Not(legal), z3.Or(*[g for e, g in gs if e.target != FAILSAFE]))))
    rep.add(check("O5 monitor: any observation -> exactly one edge; inconsistent -> only the fault edge to FAILSAFE",
                  "sense_ok' = sense_ok & consistent: exactly one guard, and !consistent -> target FAILSAFE", ENC,
                  [z3.Or(*bad)]), verbose)
    # ---------------------------------------------------------------- Q1 queued drones stay in the gate protocol
    rep.add(check("Q1 calm, not committed, at the queue point -> commit or yield (every mode)",
                  "at_queue -> enabled edge in {commit, yield}", ENC, queue_violation(cfg, edges, o, legal)), verbose)
    # ---------------------------------------------------------------- mutations
    rep.add(check("Om1 mutation: without I2 a queued drone takes a formation edge (Q1 must fail)",
                  "expect counterexample", ENC, queue_violation(cfg, edges, o, o.legal(cfg, drop=["I2"])), expect="sat"),
            verbose)
    v1 = o.legal(cfg, extra=[z3.Implies(o.passed, o.at_queue)])
    g_done = [e.guard(o, Z3L) for e in edges[Mode.GATE_PASS] if e.name == "pass_done"][0]
    rep.add(check("Om2 mutation: v1 constraint passed -> at_queue makes pass_done unreachable",
                  "expect UNSAT (reachability lost)", ENC, [v1, g_done], expect="unsat",
                  note="v1 formal/common.Obs.legal had passed -> at_queue; with I2 and I3 no 'passed' observation "
                       "remains, so a proof over that domain says nothing about the exit edge"), verbose)
    gs = [(e, e.guard(o, Z3L)) for e in edges[Mode.FORMATION_FOLLOW]]
    rep.add(check("Om3 mutation: no monitor -> an inconsistent observation drives a mission edge",
                  "expect counterexample", ENC,
                  [z3.Not(legal), z3.Or(*[g for e, g in gs if e.target != FAILSAFE])], expect="sat"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    n_ok = sum(r.passed for r in rep.results)
    print(f"observations: {n_ok}/{len(rep.results)} checks as expected -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
