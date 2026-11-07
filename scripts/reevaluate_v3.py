"""Offline re-evaluation of a finished campaign under criteria v3 (referee/criteria.py), next to its stored v2 class.

    python scripts/reevaluate_v3.py [--raw results/robustness/raw_final] [--out results/robustness/v3_reeval] [-j 8]

Reads every <protocol>/<profile>/<scenario>/s<k>/ run directory, runs referee/robustness_metrics.analyze_run (the v3
classifier, with the TerminalMonitor replayed on the logs) and writes one JSON per run plus SUMMARY.md / SUMMARY.csv:
v2 class vs v3 class per cell, the gate events E1-E8 and the stall counts.  Historical runs were stopped by criteria
v2; a run whose log ends before the v3 deadline without an outcome is CENSORED (never counted as PASS or failure).
Nothing in the run directories is modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _one(args):
    d, out = map(Path, args)
    from holo_fleet.referee.robustness_metrics import analyze_run

    try:
        res = analyze_run(d)
    except Exception as e:  # noqa: BLE001 - reported, never hidden
        res = {"class": "ERROR", "error": repr(e)}
    v2 = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    res["class_v2"] = v2.get("class")
    res["failed_v2"] = v2.get("failed_requirements")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
    return str(d), res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=str(ROOT / "results" / "robustness" / "raw_final"))
    ap.add_argument("--out", default=str(ROOT / "results" / "robustness" / "v3_reeval"))
    ap.add_argument("-j", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    a = ap.parse_args()
    raw, out = Path(a.raw), Path(a.out)
    dirs = sorted(p.parent for p in raw.glob("*/*/*/s*/run_metrics.json"))
    jobs = [(str(d), str(out / d.relative_to(raw) / "run_metrics_v3.json")) for d in dirs]
    rows = []
    with ProcessPoolExecutor(max_workers=a.j) as ex:
        for d, res in ex.map(_one, jobs, chunksize=4):
            proto, prof, sc, seed = Path(d).relative_to(raw).parts
            ge = (res.get("P2") or {}).get("events") or {}
            e8 = ge.get("E8") or {}
            rows.append({"protocol": proto, "profile": prof, "scenario": sc, "seed": seed, "class_v2": res["class_v2"],
                         "class_v3": res["class"], "outcome_v3": (res.get("terminal") or {}).get("outcome"),
                         "t_end_v3": (res.get("terminal") or {}).get("t_end"),
                         "deadline_v3": (res.get("terminal") or {}).get("deadline_s"),
                         "missing_v3": "+".join((res.get("terminal") or {}).get("missing") or []),
                         "E2": (ge.get("E2") or {}).get("total"), "E3": ge.get("E3"), "E4": ge.get("E4"),
                         "E5": ge.get("E5"), "false_free_frac": (ge.get("E1") or {}).get("false_free_frac"),
                         "false_busy_frac": (ge.get("E1") or {}).get("false_busy_frac"),
                         "idle_stalls": len(e8.get("idle_stalls") or []),
                         "committed_stalls": len(e8.get("committed_stalls") or []),
                         "chattering": (res.get("automaton") or {}).get("chattering_events")})
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "SUMMARY.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r["protocol"], r["profile"], r["scenario"], r["seed"])))
    cells = defaultdict(list)
    for r in rows:
        cells[(r["protocol"], r["profile"])].append(r)
    classes = ("PASS", "INCOMPLETE", "STALLED", "SAFE_DEGRADED", "CENSORED", "SAFETY_FAILURE", "INVALID", "ERROR")
    lines = ["# Criteria v3 re-evaluation of " + str(raw.relative_to(ROOT) if raw.is_relative_to(ROOT) else raw), "",
             "Historical runs were stopped by criteria v2: CENSORED = the log ends before the v3 deadline without an "
             "outcome.  Stalls are reported for every run (also PASS runs that recovered from one).", "",
             "| cell | n | v2 PASS | " + " | ".join(f"v3 {c}" for c in classes) + " | E2 | E3 | idle-stall runs | committed-stall runs |",
             "|---|---|---|" + "---|" * (len(classes) + 4)]
    for (proto, prof), rs in sorted(cells.items()):
        c3 = Counter(r["class_v3"] for r in rs)
        lines.append(f"| {proto} / {prof} | {len(rs)} | {sum(r['class_v2'] == 'PASS' for r in rs)} | "
                     + " | ".join(str(c3.get(c, 0)) for c in classes)
                     + f" | {sum(r['E2'] or 0 for r in rs)} | {sum(r['E3'] or 0 for r in rs)} | "
                       f"{sum(1 for r in rs if r['idle_stalls'])} | {sum(1 for r in rs if r['committed_stalls'])} |")
    tr = Counter((r["class_v2"], r["class_v3"]) for r in rows)
    lines += ["", "## v2 -> v3 transitions (all cells)", "", "| v2 | v3 | runs |", "|---|---|---|"]
    lines += [f"| {a} | {b} | {n} |" for (a, b), n in sorted(tr.items())]
    (out / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
