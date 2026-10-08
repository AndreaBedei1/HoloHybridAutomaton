"""Run every formal check and write formal/results/SUMMARY.{json,md}.

    python formal/check_properties.py            # all suites
    python formal/check_properties.py --quick    # skip the slowest suite (P1: triangle lemma, ~60 s)

Exit status 0 iff every check returned its expected verdict.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import check_determinism  # noqa: E402
import check_formation  # noqa: E402
import check_mutex  # noqa: E402
import check_observations  # noqa: E402
import check_separation  # noqa: E402
from common import RESULTS_DIR  # noqa: E402

SUITES = [
    ("Local determinism & priority hierarchy", check_determinism, "holo_fleet/ha/spec.py"),
    ("Observation consistency (perception -> automaton interface)", check_observations,
     "holo_fleet/ha/observation_invariants.py + holo_fleet/ha/spec.py"),
    ("P1 inter-vehicle separation", check_separation, "formal/check_separation.py"),
    ("P2 critical-region mutual exclusion", check_mutex, "holo_fleet/ha/gate_rule.py + formal/check_mutex.py"),
    ("P3 formation recovery (liveness, ranking functions)", check_formation, "formal/check_formation.py"),
]


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
                        "passed": r.passed, "seconds": r.seconds, "note": r.note,
                        "counterexample": r.counterexample if (r.expect == "sat" or not r.passed) else None}
                       for r in rep.results],
        })
    summary["all_passed"] = all_ok
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / "SUMMARY.json").write_text(json.dumps(summary, indent=2, default=str))
    lines = ["# Formal verification summary", "", f"Generated {summary['started']} by `formal/check_properties.py`.", "",
             "UNSAT = the negated property has no model in the abstraction, i.e. the property HOLDS for the model; "
             "SAT is expected for the satisfiability / non-vacuity checks (a witness must exist) and for the mutation "
             "tests (deliberately broken designs must yield a counterexample); mutation Om2 expects UNSAT (a broken "
             "observation domain loses the reachability of an edge).  The observation domain of every suite is the "
             "conjunction of the invariants of holo_fleet/ha/observation_invariants.py.", ""]
    for s in summary["suites"]:
        n_ok = sum(c["passed"] for c in s["checks"])
        lines += [f"## {s['title']}  ({n_ok}/{len(s['checks'])} as expected, {s['seconds']} s)", "",
                  f"Encoding: `{s['encoding']}`", "", "| check | formula | expected | verdict | ok |", "|---|---|---|---|---|"]
        for c in s["checks"]:
            lines.append(f"| {c['prop']} | {c['formula'].replace('|', '/')} | {c['expect'].upper()} | {c['verdict'].upper()} | "
                         f"{'yes' if c['passed'] else '**NO**'} |")
        lines.append("")
    lines.append(f"**Overall: {'ALL CHECKS AS EXPECTED' if all_ok else 'SOME CHECKS FAILED'}**")
    (RESULTS_DIR / "SUMMARY.md").write_text("\n".join(lines) + "\n")
    print(f"\nOVERALL: {'PASS' if all_ok else 'FAIL'} -> {RESULTS_DIR / 'SUMMARY.md'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
