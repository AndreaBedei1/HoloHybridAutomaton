"""Run every formal check and write formal/results/SUMMARY.json and SUMMARY.csv (one row per check).

    python formal/check_properties.py            # all suites
    python formal/check_properties.py --quick    # skip the slowest suite (P1: triangle lemma, ~60 s)

Exit status 0 iff every check returned its expected verdict.  An expected verdict is not a proof of safety: every check
carries a ``kind`` (see KINDS) and only the kinds "proved" and "bounded" support a guarantee; an expected counterexample of kind
"limit" is an open limitation of the guarantee, never evidence for it.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import check_acoustic  # noqa: E402
import check_determinism  # noqa: E402
import check_fleet  # noqa: E402
import check_formation  # noqa: E402
import check_gate_safety  # noqa: E402
import check_mutex  # noqa: E402
import check_observations  # noqa: E402
import check_separation  # noqa: E402
import check_status  # noqa: E402
from common import RESULTS_DIR  # noqa: E402

SUITES = [
    ("Local determinism & priority hierarchy", check_determinism, "holo_fleet/ha/spec.py"),
    ("Observation consistency (perception -> automaton interface)", check_observations,
     "holo_fleet/ha/observation_invariants.py + holo_fleet/ha/spec.py"),
    ("P1 inter-vehicle separation", check_separation, "formal/check_separation.py"),
    ("P2 critical-region mutual exclusion", check_mutex, "holo_fleet/ha/mutex_rule.py + formal/check_mutex.py"),
    ("P3 formation recovery (liveness, ranking functions)", check_formation, "formal/check_formation.py"),
    ("Variant B2 range-bearing beacon (own assumptions; B0 checks unchanged)", check_acoustic,
     "perception/acoustic.py + perception/acoustic_gate.py + formal/check_acoustic.py"),
    ("Payload v2: gate status, silence, verified passage", check_status,
     "perception/payload.py + perception/acoustic_gate.py + perception/passage.py + formal/check_status.py"),
    ("Interacting automata of the fleet at the gate (composition, failures, timers, recovery)", check_fleet, "formal/check_fleet.py"),
    ("Gate-aware joint safety (P2 as a constraint on the final command, P1 x P2 feasibility)", check_gate_safety,
     "holo_fleet/control/gate_safety.py + holo_fleet/control/escape.py + formal/check_gate_safety.py + formal/check_stage2.py"),
]


KINDS = {
    "proved": "UNSAT of a universal or inductive property (local predicate, inductive invariant, k-induction, ranking function)",
    "bounded": "UNSAT over a bounded horizon (bounded model checking or bounded response from every state of an invariant)",
    "numeric": "property checked by finite enumeration or sampling of a continuous domain (not a proof outside the samples)",
    "limit": "declared limitation: an expected counterexample outside the guarantee, left unresolved",
    "scope": "expected counterexample that marks where an assumption or a bound of the guarantee ends",
    "mutation": "expected counterexample of a deliberately broken variant: the check can fail",
    "witness": "expected satisfiable query: non-vacuity, tightness of a bound, independence of the invariants",
    "reported": "quantity reported, not guaranteed",
}
_BOUNDED = {"P1": r"^(S1[ad]|S4) ", "P2": r"^M3h? ", "Payload": r"^S2h? ", "P3": r"^F6 ", "Interacting": r"^(F2p?|F1d\d+n?|F1p) ",
            "Gate-aware": r"^S1ga "}


def kind(title: str, check: dict) -> str:
    """Kind of a check (KINDS), from the suite and the declared role in the property name."""
    prop, expect = check["prop"], check["expect"]
    if "reported" in prop:
        return "reported"
    if "declared limit" in prop:
        return "limit"
    if "mutation" in prop:
        return "mutation"
    if "scope" in prop or "domain boundary" in prop:
        return "scope"
    if expect in ("unsat", "holds"):
        if any(title.startswith(k) and re.search(rx, prop) for k, rx in _BOUNDED.items()):
            return "bounded"
        return "numeric" if expect == "holds" else "proved"
    return "witness"


def write_csv(summary: dict) -> None:
    """One row per check: suite, encoding, property, formula, expected and returned verdict, pass, seconds, note,
    counterexample (expected SAT checks and failures; the same as SUMMARY.json)."""
    import csv

    cols = ["suite", "encoding", "kind", "prop", "formula", "expect", "verdict", "as_expected", "seconds", "note", "counterexample"]
    with open(RESULTS_DIR / "SUMMARY.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for s in summary["suites"]:
            for c in s["checks"]:
                ce = c.get("counterexample")
                w.writerow([s["title"], s["encoding"], c["kind"], c["prop"], c["formula"], c["expect"], c["verdict"], c["as_expected"],
                            c["seconds"], c["note"], "" if ce is None else json.dumps(ce, default=str)])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args(argv)
    summary = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "suites": []}
    all_ok = True
    for title, mod, enc in SUITES:
        if args.quick and mod is check_separation:
            print(f"== {title}: skipped (--quick)")
            continue
        print(f"== {title}")
        t0 = time.time()
        rep = mod.run(verbose=True)
        rep.save()
        ok = rep.ok
        all_ok &= ok
        summary["suites"].append({
            "title": title, "encoding": enc, "seconds": round(time.time() - t0, 1), "all_passed": ok,
            "checks": [{"prop": r.prop, "formula": r.formula, "expect": r.expect, "verdict": r.verdict,
                        "as_expected": r.passed, "seconds": r.seconds, "note": r.note,
                        "counterexample": r.counterexample if (r.expect == "sat" or not r.passed) else None}
                       for r in rep.results],
        })
    for su in summary["suites"]:
        for c in su["checks"]:
            c["kind"] = kind(su["title"], c)
    counts = {k: sum(c["kind"] == k for su in summary["suites"] for c in su["checks"]) for k in KINDS}
    summary["all_as_expected"] = all_ok
    summary["kinds"] = {k: {"count": counts[k], "meaning": v} for k, v in KINDS.items()}
    summary["open_limits"] = [c["prop"] for su in summary["suites"] for c in su["checks"] if c["kind"] == "limit"]
    summary["unexpected"] = [c["prop"] for su in summary["suites"] for c in su["checks"] if not c["as_expected"]]
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / "SUMMARY.json").write_text(json.dumps(summary, indent=2, default=str))
    write_csv(summary)
    print(f"\nOVERALL: {'PASS' if all_ok else 'FAIL'} -> {RESULTS_DIR / 'SUMMARY.json'}, {RESULTS_DIR / 'SUMMARY.csv'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
