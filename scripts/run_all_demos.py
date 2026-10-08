"""Run every demonstration scenario once (one seed), headless, with dashboard frames and a GIF each.

    python scripts/run_all_demos.py                  # all scenarios + sonar_classification
    python scripts/run_all_demos.py gate_single      # a subset

Writes results/v2/demos/<scenario>/ and results/v2/demos/SUMMARY.{csv,md}.  To watch a scenario live
use scripts/run_demo.py instead.
"""

from __future__ import annotations

import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.runner import print_summary, run, summary_row  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402
from holo_fleet.ui.live import DemoUI  # noqa: E402

OUT = ROOT / "results" / "v2" / "demos"
ORDER = ["p1_head_on", "p1_vertical_escape", "p1_two_lines", "p1_close_encounter", "formation_triangle",
         "formation_square", "formation_six", "formation_recovery_head_current", "formation_gust", "gate_single",
         "integrated_short"]


def onboard_summary(run: Path) -> dict:
    """What the controllers logged (onboard knowledge only): time per mode, escape and give-way choices, depth span."""
    cfg = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    out = {}
    for k in range(cfg["n_drones"]):
        p = run / f"drone_{k}_state.jsonl"
        if not p.exists():
            continue
        recs = [json.loads(l) for l in open(p, encoding="utf-8")]
        modes, esc, gw = {}, set(), set()
        for r in recs:
            modes[r["mode"]] = round(modes.get(r["mode"], 0.0) + 0.1, 1)
            if r.get("escape"):
                esc.add(r["escape"]["dir"])
            if r.get("giveway"):
                gw.add(r["giveway"])
        z = [r["nav_p"][2] for r in recs]
        out[f"drone_{k}"] = {"time_in_mode_s": modes, "escape_directions": sorted(esc), "giveway": sorted(gw),
                             "depth_span_m": round(max(z) - min(z), 2),
                             "min_onboard_distance_m": min((r["d_min"] for r in recs if r.get("d_min") is not None), default=None)}
    (run / "onboard_summary.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    return out


def refresh_envelope(run: Path) -> None:
    """(Re-)evaluate the control-feasible envelope of a logged run (DI-27) into referee_metrics.json and
    summary.csv; the same evaluation the runner performs at the end of a new run."""
    from holo_fleet.runner import envelope_block

    p = run / "referee_metrics.json"
    m = json.loads(p.read_text(encoding="utf-8"))
    m["envelope"] = envelope_block(run, m["envelope"], DEFAULT)
    p.write_text(json.dumps(m, indent=1), encoding="utf-8")
    summ = summary_row(run.name, m)
    with open(run / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summ))
        w.writeheader()
        w.writerow(summ)


def small_gif(name: str) -> None:
    """Repository copy of the dashboard GIF: 640 px, every third frame (about 1.8 s of simulation per frame)."""
    frames = sorted((OUT / name / "dashboard").glob("dash_*.jpg"))
    if frames:
        make_gif(frames, ROOT / "figures" / "v2" / "gifs" / f"{name}.gif", width=640, fps=4.0, every=3, colors=80)


def summary_table() -> None:
    rows = []
    for name in ORDER:
        p = OUT / name / "referee_metrics.json"
        if not p.exists():
            continue
        onboard_summary(OUT / name)
        from experiment_metrics import summarize

        refresh_envelope(OUT / name)
        summarize(OUT / name)
        m = json.loads(p.read_text(encoding="utf-8"))
        r = summary_row(name, m)
        perf = m["run"].get("perf", {})
        r.update({"mean_tick_ms": perf.get("mean_tick_ms"), "p95_tick_ms": perf.get("p95_tick_ms"),
                  "rtf": perf.get("real_time_factor"), "controller_ms_all": perf.get("controller_ms_mean_all_drones"),
                  "n_drones": json.loads((OUT / name / "run_config.json").read_text(encoding="utf-8"))["n_drones"]})
        rec = [e for e in m["P3_formation_recovery"]["episodes"]]
        r["recovery_after_perturbation_s"] = max([e["recovery_after_perturbation_s"] or 0 for e in rec], default=None)
        intr = m["P1_separation"].get("intruder")
        r["intruder_clearance"] = None if not intr else intr["min_distance"]
        r["self_declared_envelope_violations"] = m["run"]["self_declared_envelope_violations"]
        r["envelope_verdict"] = m["envelope"].get("verdict")
        r["determinism_violations"] = sum(m["run"]["determinism_violations"].values())
        inc = m["run"].get("observation_consistency_violations")
        r["observation_consistency_violations"] = None if inc is None else sum(inc.values())
        rows.append(r)
    if not rows:
        return
    keys = list(rows[0])
    with open(OUT / "SUMMARY.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    lines = ["| scenario | drones | status | P1 min d [m] | P2 max occ | P3 episodes / max recovery [s] | collisions | messages | "
             "control-feasible envelope | self-declared envelope violations | determinism viol. | observation consistency viol. | "
             "static rank | mean tick [ms] | RTF |", "|" + "---|" * 15]
    for r in rows:
        p3 = "-" if r["P3_episodes"] is None else f"{r['P3_episodes']} / {r['max_recovery_time_s'] if r['max_recovery_time_s'] is not None else '-'}"
        lines.append(f"| {r['scenario']} | {r['n_drones']} | {r['status']} | {r['min_distance']} | "
                     f"{r['max_occupancy'] if r['max_occupancy'] is not None else '-'} | {p3} | {r['collisions']} | "
                     f"{r['messages']} | {r['envelope_verdict']} | "
                     f"{r['self_declared_envelope_violations']} | {r['determinism_violations']} | "
                     f"{'-' if r['observation_consistency_violations'] is None else r['observation_consistency_violations']} | "
                     f"{r['static_rank_uses']} | {r['mean_tick_ms']} | {r['rtf']} |")
    (OUT / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv) -> int:
    names = argv or (["sonar_classification"] + ORDER)
    for name in names:
        t0 = time.time()
        print(f"=== {name}", flush=True)
        if name == "sonar_classification":
            from demo_classification import run_classification_demo

            run_classification_demo(OUT, headless=True, show=False)
            small_gif("sonar_classification")
            continue
        sc = SCENARIOS[name](DEFAULT)
        ui = DemoUI(sc, DEFAULT, sc.sim.names, show=False, draw_viewport=False)
        m = run(name, OUT, headless=True, run_id=name, ui=ui)
        print_summary(name, m)
        make_gif((OUT / name / "dashboard").glob("dash_*.jpg"), OUT / name / f"{name}.gif")
        small_gif(name)
        print(f"    wall {time.time() - t0:.0f} s", flush=True)
    summary_table()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
