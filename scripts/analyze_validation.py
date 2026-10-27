"""Analysis of the validation campaign in the realistic operating domain (CLEAN, LOW, MEDIUM x ONE / SUBSET / ALL).

    python scripts/analyze_validation.py [--raw results/robustness/raw_validation]

Reads <raw>/B0/<PROFILE>[-<SCOPE>]/<scenario>/s<seed>/run_metrics.json (scripts/run_sensor_robustness_campaign.py
--phase validation; each run ends on SUCCESS, FAILURE or DEADLINE, holo_fleet/referee/terminal.py) and writes
results/validation/{SUMMARY.md, runs.csv, failure_cases.json, noise_profiles.json} and figures/validation/*.png.

Per run:
* P1 = no two drones closer than d_safe, no contact (drones, gate, the non-cooperative vehicle), that vehicle never
  closer than d_safe;
* P2 = at most one drone in a critical region (where a gate is judged) and every healthy drone through the gate;
* P3 = the scenario's formation requirement (recovered, or re-formed with the lost drone's slot declared vacant) and
  no slot still falsely declared vacant at the end (only where the reference judges a formation);
* Determinism = no determinism violation of the automaton and no observation-consistency violation;
* Result = PASS when the run reached SUCCESS with every applicable property holding.
Proportions carry a Wilson 95 % interval; INVALID runs (infrastructure) are excluded from every denominator.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from make_figures_v2 import INK, SERIES, STATUS, style  # noqa: E402

RAW = ROOT / "results" / "robustness" / "raw_validation"
OUT = ROOT / "results" / "validation"
FIG = ROOT / "figures" / "validation"
LABELS = ("CLEAN", "LOW-ONE", "LOW-SUBSET", "LOW-ALL", "MEDIUM-ONE", "MEDIUM-SUBSET", "MEDIUM-ALL")
LETTER = {"PASS": "P", "SAFE_DEGRADED": "D", "LIVENESS_FAILURE": "L", "SAFETY_FAILURE": "S", "INVALID": "I"}


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return (math.nan, math.nan)
    c = (k + z * z / 2) / (n + z * z)
    h = z * math.sqrt(k * (n - k) / n + z * z / 4) / (n + z * z)
    return (max(0.0, c - h), min(1.0, c + h))


def frac(k, n):
    if n == 0:
        return "n/a"
    lo, hi = wilson(k, n)
    return f"{k}/{n} ({100 * lo:.0f}-{100 * hi:.0f} %)"


def g(d, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or d.get(k) is None:
            return default
        d = d[k]
    return d


def row_of(m) -> dict:
    req = m.get("requirements") or {}
    p1 = m.get("P1") or {}
    intr = p1.get("intruder_min_m")
    term = m.get("terminal") or {}
    gate = bool(g(m, "P2", "gates"))
    p3_keys = [k for k in ("P3_recovered", "P3_degraded_holds") if k in req]
    label = m["profile"]
    prof, _, scope = label.partition("-")
    return {
        "run": f"{label}/{m['scenario']}/s{m['seed']}", "label": label, "profile": prof, "scope": scope or "-",
        "scenario": m["scenario"], "seed": m["seed"], "class": m["class"],
        "affected": " ".join(str(k) for k in (g(m, "noise_drones") or [])),
        "P1": bool(p1.get("holds")) and not p1.get("collisions") and (intr is None or intr >= 1.0),
        "min_sep_m": p1.get("min_true_separation_m"), "vehicle_min_m": intr, "contacts": p1.get("collisions"),
        "warning_entries": p1.get("warning_entries"), "ca_entries": p1.get("ca_entries"),
        "gate": gate, "P2": (bool(g(m, "P2", "holds")) and bool(req.get("gate_completed"))) if gate else None,
        "max_occupancy": g(m, "P2", "max_occupancy"), "longest_stall_s": g(m, "P2", "longest_stall_s"),
        "P3": (all(req[k] for k in p3_keys) and req.get("no_permanent_false_vacancy", True)) if p3_keys else None,
        "false_missing_episodes": g(m, "P3", "false_missing_episodes"), "false_vacancy": g(m, "P3", "false_vacancy"),
        "false_vacancy_at_end": g(m, "P3", "false_vacancy_at_end"),
        "recovery_max_s": g(m, "P3", "recovery_time_max_s"),
        "determinism": bool(req.get("automaton_consistent")),
        "outcome": term.get("outcome"), "t_end": term.get("t_end"), "t_ref": term.get("t_ref"),
        "deadline_s": term.get("deadline_s"), "terminal_reason": term.get("reason"),
        "progress": req.get("progress"), "failsafe_entries_sensor": g(m, "automaton", "failsafe_entries_sensor_fault_healthy"),
        "failsafe_time_s": g(m, "automaton", "failsafe_time_healthy_s"), "chattering": g(m, "automaton", "chattering_events"),
        "nav_err_max_m": g(m, "sensor", "navigation", "pos_err_max_m"), "heading_err_p95_deg": g(m, "sensor", "heading", "err_p95_abs_deg"),
        "sonar_longest_stale_s": g(m, "sensor", "sonar", "longest_stale_s"),
        "failed_requirements": " ".join(m.get("failed_requirements") or []),
        "code_sha": g(m, "code_version", "sha"), "code_dirty": bool(g(m, "code_version", "dirty_files")),
    }


def load(raw: Path):
    runs = []
    for p in sorted(raw.glob("B0/*/*/s*/run_metrics.json")):
        m = json.loads(p.read_text(encoding="utf-8"))
        try:
            rc = json.loads((p.parent / "run_config.json").read_text(encoding="utf-8"))
            m["noise_drones"] = rc.get("noise_drones")
        except (OSError, ValueError):
            m["noise_drones"] = None
        m["_dir"] = p.parent
        runs.append(m)
    return runs


def table_main(rows) -> str:
    lines = ["| Profile | Affected drones | Runs (valid) | P1 | P2 | P3 | Determinism | Result (PASS) |",
             "|---|---|---|---|---|---|---|---|"]
    for lab in LABELS:
        allr = [r for r in rows if r["label"] == lab]
        if not allr:
            continue
        v = [r for r in allr if r["class"] != "INVALID"]
        prof, _, scope = lab.partition("-")
        aff = {"": "none (deployed sensors)", "ONE": "one drone", "SUBSET": "half of the fleet", "ALL": "every drone"}[scope]
        cells = []
        for key in ("P1", "P2", "P3"):
            ap = [r for r in v if r[key] is not None]
            cells.append(frac(sum(bool(r[key]) for r in ap), len(ap)))
        lines.append(f"| {prof} | {aff} | {len(allr)} ({len(v)}) | " + " | ".join(cells) +
                     f" | {frac(sum(r['determinism'] for r in v), len(v))} | {frac(sum(r['class'] == 'PASS' for r in v), len(v))} |")
    return "\n".join(lines)


def table_scenarios(rows, scenarios) -> str:
    labs = [lab for lab in LABELS if any(r["label"] == lab for r in rows)]
    lines = ["| scenario | " + " | ".join(labs) + " |", "|---|" + "---|" * len(labs)]
    for sc in scenarios:
        cells = []
        for lab in labs:
            sel = sorted((r for r in rows if r["scenario"] == sc and r["label"] == lab), key=lambda r: r["seed"])
            cells.append(" ".join(LETTER[r["class"]] for r in sel) or "-")
        lines.append(f"| `{sc}` | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def table_completion(rows, scenarios) -> str:
    labs = [lab for lab in LABELS if any(r["label"] == lab for r in rows)]
    lines = ["| scenario | T_ref [s] | deadline [s] | " + " | ".join(labs) + " |", "|---|---|---|" + "---|" * len(labs)]
    for sc in scenarios:
        sel = [r for r in rows if r["scenario"] == sc]
        if not sel:
            continue
        cells = []
        for lab in labs:
            ts = [r["t_end"] for r in sel if r["label"] == lab and r["outcome"] == "SUCCESS"]
            nd = sum(1 for r in sel if r["label"] == lab and r["outcome"] == "DEADLINE")
            cells.append(("-" if not ts else f"{np.median(ts):.0f}") + (f" ({nd} deadline)" if nd else ""))
        lines.append(f"| `{sc}` | {sel[0]['t_ref']:.0f} | {sel[0]['deadline_s']:.0f} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def table_metrics(rows) -> str:
    spec = [("min true separation [m] (min)", "min_sep_m", np.min), ("non-cooperative vehicle, min distance [m]", "vehicle_min_m", np.min),
            ("COLLISION_AVOIDANCE entries per run (mean)", "ca_entries", np.mean),
            ("max critical-region occupancy (max)", "max_occupancy", np.max), ("longest gate stall [s] (max)", "longest_stall_s", np.max),
            ("false missing episodes per run (mean)", "false_missing_episodes", np.mean),
            ("false vacancies per run (mean)", "false_vacancy", np.mean), ("sensor-fault FAILSAFE entries per run (mean)", "failsafe_entries_sensor", np.mean),
            ("navigation error max [m] (max)", "nav_err_max_m", np.max), ("heading error p95 [deg] (max)", "heading_err_p95_deg", np.max),
            ("longest sonar gap [s] (max)", "sonar_longest_stale_s", np.max)]
    labs = [lab for lab in LABELS if any(r["label"] == lab for r in rows)]
    lines = ["| metric | " + " | ".join(labs) + " |", "|---|" + "---|" * len(labs)]
    for label, key, fn in spec:
        vals = []
        for lab in labs:
            xs = [r[key] for r in rows if r["label"] == lab and r["class"] != "INVALID" and r.get(key) is not None
                  and not (isinstance(r[key], float) and math.isnan(r[key]))]
            vals.append("-" if not xs else f"{float(fn(xs)):.3g}")
        lines.append(f"| {label} | " + " | ".join(vals) + " |")
    return "\n".join(lines)


def fig_properties(rows, path: Path):
    labs = [lab for lab in LABELS if any(r["label"] == lab for r in rows)]
    fig, ax = plt.subplots(figsize=(7.6, 3.4))
    x = np.arange(len(labs))
    for i, key in enumerate(("P1", "P2", "P3", "PASS")):
        ks, lo, hi = [], [], []
        for lab in labs:
            v = [r for r in rows if r["label"] == lab and r["class"] != "INVALID"]
            ap = v if key == "PASS" else [r for r in v if r[key] is not None]
            k = sum((r["class"] == "PASS") if key == "PASS" else bool(r[key]) for r in ap)
            ks.append(k / len(ap) if ap else np.nan)
            a, b = wilson(k, len(ap))
            lo.append(a)
            hi.append(b)
        ks, lo, hi = np.array(ks), np.array(lo), np.array(hi)
        ax.errorbar(x + (i - 1.5) * 0.09, ks, yerr=[ks - lo, hi - ks], marker="o", ms=5, capsize=2, ls="none",
                    color=SERIES[i], label=key)
    ax.set_xticks(x)
    ax.set_xticklabels([lab.replace("-", "\n") for lab in labs], fontsize=8)
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("fraction of valid runs (Wilson 95 %)")
    ax.set_title("P1, P2, P3 in the realistic operating domain")
    ax.legend(fontsize=8, loc="lower left", ncol=4)
    save(fig, path)


def fig_completion(rows, scenarios, path: Path):
    labs = [lab for lab in LABELS if any(r["label"] == lab for r in rows)]
    fig, ax = plt.subplots(figsize=(7.6, 3.4))
    for i, lab in enumerate(labs):
        ys = [r["t_end"] / r["t_ref"] for r in rows if r["label"] == lab and r["outcome"] == "SUCCESS" and r["t_ref"]]
        if ys:
            jit = np.linspace(-0.12, 0.12, len(ys)) if len(ys) > 1 else np.zeros(1)
            ax.scatter(np.full(len(ys), i) + jit, ys, s=10, color=SERIES[0], alpha=0.6)
            ax.plot([i - 0.2, i + 0.2], [np.median(ys)] * 2, color=INK["primary"], lw=2)
    dl = sorted({r["deadline_s"] / r["t_ref"] for r in rows if r["t_ref"]})
    for y in dl[:1]:
        ax.axhline(y, color=STATUS["critical"], lw=1, ls="--")
        ax.text(len(labs) - 0.5, y, "deadline (shortest)", ha="right", va="bottom", fontsize=7, color=INK["secondary"])
    ax.set_xticks(range(len(labs)))
    ax.set_xticklabels([lab.replace("-", "\n") for lab in labs], fontsize=8)
    ax.set_ylabel("completion time / reference time")
    ax.set_title("Time to complete every required outcome (successful runs)")
    save(fig, path)


def save(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  ", path.relative_to(ROOT))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=str(RAW))
    a = ap.parse_args()
    style()
    runs = load(Path(a.raw))
    if not runs:
        print("no runs")
        return 1
    rows = [row_of(m) for m in runs]
    OUT.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(OUT / "runs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    from holo_fleet.sim.sensor_noise import PROFILES, VALIDATION_LEVELS

    from run_sensor_robustness_campaign import VALIDATION_SCENARIOS, scope_drones

    (OUT / "noise_profiles.json").write_text(json.dumps(
        {"levels": {k: PROFILES[k].as_dict() for k in VALIDATION_LEVELS},
         "scopes": {sc: {s: scope_drones(sc, s) for s in ("ONE", "SUBSET", "ALL")} for sc in VALIDATION_SCENARIOS},
         "justification": "docs/SENSOR_ROBUSTNESS_AND_BEACONING.md, section 1"}, indent=1), encoding="utf-8")
    diag_path = OUT / "diagnoses.json"
    diag = json.loads(diag_path.read_text(encoding="utf-8")) if diag_path.exists() else {}
    fails = [{"run": r["run"], "class": r["class"], "outcome": r["outcome"], "reason": r["terminal_reason"],
              "failed_requirements": r["failed_requirements"], "diagnosis": diag.get(r["run"]),
              "evidence": {k: r[k] for k in ("min_sep_m", "vehicle_min_m", "contacts", "max_occupancy", "longest_stall_s",
                                             "false_vacancy", "false_vacancy_at_end", "failsafe_entries_sensor",
                                             "nav_err_max_m", "heading_err_p95_deg", "t_end", "t_ref")}}
             for r in rows if r["class"] != "PASS"]
    (OUT / "failure_cases.json").write_text(json.dumps(fails, indent=1, default=str), encoding="utf-8")
    scen = [s for s in VALIDATION_SCENARIOS if any(r["scenario"] == s for r in rows)]
    shas = sorted({(r["code_sha"] or "?")[:12] + (" (dirty)" if r["code_dirty"] else "") for r in rows})
    n_inv = sum(r["class"] == "INVALID" for r in rows)
    lines = ["# Validation in the realistic operating domain (communication-free baseline)", "",
             "Generated by `scripts/analyze_validation.py` from `scripts/run_sensor_robustness_campaign.py --phase "
             "validation` (patched HoloOcean engine, headless).  Levels: CLEAN (the deployed sensors), LOW and MEDIUM "
             "(datasheet / literature anchored, [docs](../../docs/SENSOR_ROBUSTNESS_AND_BEACONING.md)), each applied to "
             "ONE drone, a SUBSET (half of the fleet) or ALL drones; seeds 0, 1, 2.  Every run ends on SUCCESS (every "
             "required outcome reached), FAILURE (a safety violation) or DEADLINE (1.5 x the reference time + 20 s).", "",
             f"Runs: {len(rows)} ({n_inv} invalid, excluded).  Code: {', '.join(shas)}.", "",
             "## Main result", "", table_main(rows), "",
             "P1: no two drones closer than d_safe, no contact, the non-cooperative vehicle never closer than d_safe.  "
             "P2 (gate scenarios): at most one drone in the critical region and every healthy drone through the gate.  "
             "P3 (formation scenarios): formation recovered / re-formed with the lost slot declared vacant, and no slot "
             "still falsely vacant at the end.  Determinism: no automaton determinism or observation-consistency "
             "violation.  Result: SUCCESS with every applicable property.", "",
             "## Per scenario (seeds 0, 1, 2; P pass, D safe degraded, L liveness failure, S safety failure, I invalid)", "",
             table_scenarios(rows, scen), "", "## Completion time (median of successful runs, seconds)", "",
             table_completion(rows, scen), "", "## Behaviour and sensor metrics", "", table_metrics(rows), "",
             "## Runs that are not PASS", ""]
    if fails:
        lines += ["| run | class | terminal | failed requirements | diagnosis |", "|---|---|---|---|---|"]
        for f in fails:
            dg = f["diagnosis"] or {}
            lines.append(f"| `{f['run']}` | {f['class']} | {f['outcome']}: {f['reason']} | {f['failed_requirements'] or '-'} | "
                         f"{(dg.get('category', '') + ' ' + dg.get('cause', '')).strip() or 'to be diagnosed'} |")
    else:
        lines.append("None.")
    lines += ["", "Per-run data: [runs.csv](runs.csv), [failure_cases.json](failure_cases.json); levels and affected "
              "drones: [noise_profiles.json](noise_profiles.json).", "", "## Figures", "",
              "![properties](../../figures/validation/properties_by_profile.png)", "",
              "![completion](../../figures/validation/completion_time.png)", ""]
    (OUT / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    print("  ", (OUT / "SUMMARY.md").relative_to(ROOT))
    fig_properties(rows, FIG / "properties_by_profile.png")
    fig_completion(rows, scen, FIG / "completion_time.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
