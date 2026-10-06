"""Print the Markdown tables of REPORT.md from the run metrics (no hand-copied numbers)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.run_all import EXPERIMENTS  # noqa: E402


def load(run_id):
    d = ROOT / "results" / run_id
    if not (d / "referee_metrics.json").exists():
        return None
    m = json.loads((d / "referee_metrics.json").read_text())
    events = [json.loads(line) for line in open(d / "events.jsonl")] if (d / "events.jsonl").exists() else []
    perc = json.loads((d / "perception_stats.json").read_text()) if (d / "perception_stats.json").exists() else {}
    return m, events, perc


def modes_entered(events, name):
    return sum(1 for e in events if e.get("to") == name and e.get("from") != name)


def main() -> int:
    print("| experiment | sim time [s] | P1: min d [m] (d_safe 1.0) | warning / avoidance entries | P2: max occupancy, order | "
          "gate decisions | P3: episodes, recovery after perturbation [s] | envelope (max drift m/s, flags) | self-declared envelope violations |")
    print("|---|---|---|---|---|---|---|---|---|")
    for run_id, scenario, extra, label in EXPERIMENTS:
        r = load(run_id)
        if r is None:
            continue
        m, ev, perc = r
        p1, p2, p3, env = m["P1_separation"], m["P2_mutual_exclusion"], m["P3_formation_recovery"], m["envelope"]
        sw = modes_entered(ev, "SEPARATION_WARNING")
        ca = modes_entered(ev, "COLLISION_AVOIDANCE")
        dec = {}
        for e in ev:
            if e.get("type") == "decision" and e.get("decision") in ("PASS", "YIELD", "RETRY_PASS"):
                dec[e["decision"]] = dec.get(e["decision"], 0) + 1
        p2s = "-" if not p2["gates"] else f"{max(p2['max_occupancy'].values())}; " + " / ".join(
            f"{g}: {'>'.join(x.replace('drone_', 'd') for x in o)}" for g, o in p2["entry_order"].items())
        rec = [e["recovery_after_perturbation"] for e in p3["episodes"] if e["recovery_after_perturbation"] is not None]
        p3s = "-" if not p3["enabled"] else (f"{p3['n_episodes']}, " + (", ".join(f"{x:.1f}" for x in rec) if rec else "-")
                                             + (f" (unrecovered {p3['unrecovered_at_end']})" if p3["unrecovered_at_end"] else ""))
        envs = f"{env['max_effective_drift_applied']}, {'inside' if env['inside_envelope'] else 'EXCEEDED'}"
        selfdecl = sum(1 for e in ev if e.get("type") == "ENVELOPE_VIOLATION")
        print(f"| {label} (`{run_id}`) | {m['run']['sim_time_s']} | {p1['min_distance_overall']:.2f} ({'holds' if p1['holds'] else 'VIOLATED'}) | "
              f"{sw} / {ca} | {p2s} | {json.dumps(dec) if dec else '-'} | {p3s} | {envs} | {selfdecl} |")
    print()
    from holo_fleet.analysis import load_run, structure_clearance

    print("| run | collision-sensor contacts | min clearance to arena structures [m] (where) |")
    print("|---|---|---|")
    for run_id, scenario, extra, label in EXPERIMENTS:
        if load(run_id) is None:
            continue
        sc = structure_clearance(load_run(ROOT / "results" / run_id))
        worst = min((v for k, v in sc.items() if k.startswith("drone_")), key=lambda v: v["min_clearance_m"])
        print(f"| `{run_id}` | {sc['collision_sensor_contacts']} | {worst['min_clearance_m']:.2f} ({worst['gate']}) |")
    print()
    print("| run | neighbour estimates | coverage <= 8 m | p99 abs error x/y/z [m] | max abs error x/y/z [m] | share within eps | messages sent / delivered |")
    print("|---|---|---|---|---|---|---|")
    for run_id, scenario, extra, label in EXPERIMENTS:
        r = load(run_id)
        if r is None:
            continue
        m, ev, perc = r
        cov = [v for k, v in perc.items() if k.startswith("coverage")]
        print(f"| `{run_id}` | {perc.get('n_samples')} | {cov[0] * 100 if cov and cov[0] is not None else float('nan'):.1f}% | "
              f"{perc.get('abs_err_p99_xyz')} | {perc.get('abs_err_max_xyz')} | {perc.get('frac_component_err_le_eps', float('nan')) * 100:.1f}% | "
              f"{m['run'].get('inter_agent_messages_sent')} / {m['run'].get('inter_agent_messages_delivered')} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
