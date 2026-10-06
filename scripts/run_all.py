"""Reproduce every demonstrative run, then analyse, render and summarise.

    python scripts/run_all.py                 # all experiments (~1.5 h wall time on the reference PC)
    python scripts/run_all.py --only gate_arena_s0 stress_s0
    python scripts/run_all.py --skip-sim      # only re-analyse existing runs

Each simulation runs in its own process (one HoloOcean engine at a time).
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

EXPERIMENTS = [
    # run_id, scenario, extra CLI args, experiment label
    ("pair_crossing_s0", "pair_crossing", [], "Exp 0 - two drones head-on (separation sanity check)"),
    ("formation_medium_s0", "formation_current", ["--current", "medium", "--no-jet"],
     "Exp 1a - formation under lateral current 0.25 m/s (inside envelope)"),
    ("formation_high_s0", "formation_current", ["--current", "high", "--no-jet"],
     "Exp 1b - formation under lateral current 0.40 m/s (inside envelope)"),
    ("formation_medium_gust_s0", "formation_current", ["--current", "medium"],
     "Exp 1c - medium current + 0.8 m/s gust (beyond nominal authority): disturbance and recovery"),
    ("gate_arena_s0", "gate_arena", ["--current", "low"], "Exp 2 - gate mutual exclusion in the marine arena"),
    ("stress_s0", "stress", [], "Exp 3 - no-communication stress test"),
    ("gate_arena_comms_s0", "gate_arena_comms", ["--current", "low"], "Exp 4 - optional intermittent communication (comparison)"),
]


def sh(args, log: Path) -> int:
    with open(log, "w", encoding="utf-8") as fh:
        return subprocess.call([sys.executable] + args, cwd=ROOT, stdout=fh, stderr=subprocess.STDOUT)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--skip-sim", action="store_true")
    args = ap.parse_args(argv)
    todo = [e for e in EXPERIMENTS if args.only is None or e[0] in args.only]
    logs = ROOT / "results" / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    if not args.skip_sim and args.only is None:
        print(f"[{time.strftime('%H:%M:%S')}] plant calibration with the deployed low-level controller ...", flush=True)
        sh(["scripts/calibrate_plant.py"], logs / "calibrate_plant.log")
    for run_id, scenario, extra, label in todo:
        run_dir = ROOT / "results" / run_id
        if not args.skip_sim:
            t0 = time.time()
            print(f"[{time.strftime('%H:%M:%S')}] {label}: simulating {run_id} ...", flush=True)
            rc = sh(["scripts/run_experiment.py", "--scenario", scenario, "--run-id", run_id, "--out", "results"] + extra,
                     logs / f"{run_id}.log")
            print(f"    exit={rc} in {time.time() - t0:.0f}s", flush=True)
        if (run_dir / "referee_metrics.json").exists():
            sh(["scripts/render_report.py", "--run", str(run_dir), "--copy-figures"], logs / f"{run_id}_render.log")
    runs = [ROOT / "results" / e[0] for e in EXPERIMENTS if (ROOT / "results" / e[0] / "referee_metrics.json").exists()]
    if runs:
        sh(["scripts/validate_assumptions.py"] + [str(r) for r in runs], logs / "validate_assumptions.log")
    # aggregate summary
    rows = []
    for run_id, scenario, extra, label in EXPERIMENTS:
        p = ROOT / "results" / run_id / "summary.csv"
        if p.exists():
            r = list(csv.DictReader(open(p)))[0]
            r["experiment"] = label
            rows.append(r)
    if rows:
        keys = ["experiment"] + [k for k in rows[0] if k != "experiment"]
        with open(ROOT / "results" / "SUMMARY.csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        md = ["# Demonstrative runs - referee verdicts (ground truth)", "",
              "| experiment | run | P1 holds (min d) | P2 holds (max occ) | P3 holds (max recovery after perturbation) | inside envelope (max drift) | determinism violations |",
              "|---|---|---|---|---|---|---|"]
        for r in rows:
            p2 = "n/a" if r["P2_holds"] in ("", "None") else f"{r['P2_holds']} ({r['P2_max_occupancy']})"
            p3 = "n/a" if r["P3_holds"] in ("", "None") else f"{r['P3_holds']} ({r['P3_max_recovery_s'] or '-'} s)"
            md.append(f"| {r['experiment']} | `{r['run_id']}` | {r['P1_holds']} ({r['min_pair_distance_m']} m) | {p2} | {p3} | "
                      f"{r['inside_envelope']} ({r['max_drift_m_s']} m/s) | {r['determinism_violations']} |")
        (ROOT / "results" / "SUMMARY.md").write_text("\n".join(md) + "\n")
        print("\n".join(md))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
