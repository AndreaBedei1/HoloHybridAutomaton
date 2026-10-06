"""Analyse a run: referee verdicts, perception-vs-truth statistics and control figures.

Usage::

    python scripts/analyze_results.py --run results/gate_arena_s0
    python scripts/analyze_results.py --run gate_arena_s0 --copy-figures
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.analysis import load_run, make_all_figures, perception_errors  # noqa: E402


def resolve(run: str) -> Path:
    p = Path(run)
    if p.is_dir():
        return p
    for base in (ROOT / "results", ROOT / "results" / "dev"):
        if (base / run).is_dir():
            return base / run
    raise SystemExit(f"run directory not found: {run}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--copy-figures", action="store_true", help="also copy figures to figures/<run>__*.png")
    args = ap.parse_args(argv)
    run_dir = resolve(args.run)
    made = make_all_figures(run_dir, copy_to=(ROOT / "figures") if args.copy_figures else None)
    rd = load_run(run_dir)
    m = rd.metrics
    print(f"== {rd.name}: {rd.config['description']}")
    print(f"   status={m['run']['status']} sim_time={m['run']['sim_time_s']}s wall={m['run']['wall_time_s']}s")
    p1, p2, p3, env = m["P1_separation"], m["P2_mutual_exclusion"], m["P3_formation_recovery"], m["envelope"]
    print(f"   P1 holds={p1['holds']} min_d={p1['min_distance_overall']} m  violations={p1['violations_d_lt_d_safe']}"
          f"  contacts={p1['physical_contacts_d_lt_d_collision']}")
    print(f"   P2 holds={p2['holds']} max_occ={p2['max_occupancy']} order={p2['entry_order']}")
    print(f"   P3 holds={p3['holds']} episodes={p3['n_episodes']} max_recovery_after_perturbation={p3['max_recovery_time_s']}"
          f" final_err={p3['final_form_err']}")
    print(f"   envelope: inside={env['inside_envelope']} max_drift={env['max_effective_drift_applied']}"
          f" flags={len(env['out_of_envelope_flags'])}")
    print(f"   determinism monitor violations={m['run']['determinism_monitor_violations']}")
    stats = json.loads((run_dir / "perception_stats.json").read_text())
    print(f"   perception: {json.dumps(stats)}")
    for k, v in made.items():
        print(f"   figure {k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
