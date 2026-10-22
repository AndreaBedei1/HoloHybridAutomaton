"""Analysis of the sensor-robustness campaign and of the beacon ablation (raw runs -> versioned summaries).

    python scripts/analyze_sensor_robustness.py [--raw results/robustness/raw]

Reads every results/robustness/raw/<protocol>/<profile>/<scenario>/s<seed>/run_metrics.json written by
scripts/run_sensor_robustness_campaign.py and writes:

* results/robustness/  summary.csv (one row per run), sensor_metrics.csv, noise_profiles.json,
                       failure_cases.json (every run that is not PASS, with its evidence and timeline),
                       SUMMARY.md, figures/*.png;
* results/beaconing/   comparison.csv, beacon_metrics.csv, SUMMARY.md, figures/*.png (when B1 runs exist).

Hand-written diagnoses of failure cases (category A-E, cause, responsible sensor) are read from
results/robustness/diagnoses.json and merged into failure_cases.json.  Proportions carry a Wilson 95 %
interval; B0 / B1 differences are paired by (profile, scenario, seed) with a seeded bootstrap 95 % interval and
a Wilcoxon signed-rank test.  Three seeds per configuration: small samples, read the intervals.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from make_figures_v2 import INK, SERIES, STATUS, style  # noqa: E402

RAW = ROOT / "results" / "robustness" / "raw"
OUT = ROOT / "results" / "robustness"
BOUT = ROOT / "results" / "beaconing"
LEVELS = ("CLEAN", "LOW", "MEDIUM", "HIGH")
CLASSES = ("PASS", "SAFE_DEGRADED", "LIVENESS_FAILURE", "SAFETY_FAILURE", "INVALID")
LETTER = {"PASS": "P", "SAFE_DEGRADED": "D", "LIVENESS_FAILURE": "L", "SAFETY_FAILURE": "S", "INVALID": "I"}
CLASS_COLOR = {"PASS": STATUS["good"], "SAFE_DEGRADED": STATUS["warning"], "LIVENESS_FAILURE": STATUS["serious"],
               "SAFETY_FAILURE": STATUS["critical"], "INVALID": INK["muted"]}
FAMILY = (("P1 separation", "p1_"), ("formation", "formation_"), ("lost drone", "lost_drone_"),
          ("gate / mutex", ""))


def family(sc: str) -> str:
    for name, prefix in FAMILY[:3]:
        if sc.startswith(prefix):
            return name
    return "gate / mutex"


def phase(m) -> str:
    if m["protocol"] != "B0":
        return "beacon"
    if m["profile"].startswith("ISO_"):
        return "isolation"
    return "high" if m["profile"] == "HIGH" else "main"


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return (math.nan, math.nan)
    c = (k + z * z / 2) / (n + z * z)
    h = z * math.sqrt(k * (n - k) / n + z * z / 4) / (n + z * z)
    return (max(0.0, c - h), min(1.0, c + h))


def g(d, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or d.get(k) is None:
            return default
        d = d[k]
    return d


def nanmean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.mean(xs)) if xs else None


# ------------------------------------------------------------------------------------------------ load
def load(raw: Path):
    runs = []
    for p in sorted(raw.glob("*/*/*/s*/run_metrics.json")):
        m = json.loads(p.read_text(encoding="utf-8"))
        m["_dir"] = p.parent
        m["_id"] = f"{m['protocol']}/{m['profile']}/{m['scenario']}/s{m['seed']}"
        runs.append(m)
    return runs


def flat(m) -> dict:
    req = m.get("requirements") or {}
    delays = [d for d in (g(m, "P3", "true_missing_delay_s") or []) if d is not None]
    return {
        "run": m["_id"], "phase": phase(m), "protocol": m["protocol"], "profile": m["profile"],
        "scenario": m["scenario"], "family": family(m["scenario"]), "seed": m["seed"], "class": m["class"],
        "failed_requirements": " ".join(m.get("failed_requirements") or []),
        "p1_ok": bool(g(m, "P1", "holds")) and g(m, "P1", "collisions", default=0) == 0,
        "min_sep_m": g(m, "P1", "min_true_separation_m"), "intruder_min_m": g(m, "P1", "intruder_min_m"),
        "collisions": g(m, "P1", "collisions"), "warning_entries": g(m, "P1", "warning_entries"),
        "ca_entries": g(m, "P1", "ca_entries"), "time_warning_s": g(m, "P1", "time_warning_s"),
        "time_ca_s": g(m, "P1", "time_ca_s"),
        "gate": bool(g(m, "P2", "gates")), "p2_ok": bool(g(m, "P2", "holds")),
        "max_occupancy": g(m, "P2", "max_occupancy"), "gate_completed": g(m, "P2", "completed"),
        "completion_time_s": g(m, "P2", "completion_time_s"), "queue_wait_mean_s": g(m, "P2", "queue_wait_mean_s"),
        "time_without_leader_s": g(m, "P2", "time_without_leader_s"), "deadlock_events": g(m, "P2", "deadlock_events"),
        "longest_stall_s": g(m, "P2", "longest_stall_s"), "false_busy_s": g(m, "P2", "false_busy_s"),
        "false_free_s": g(m, "P2", "false_free_s"),
        "p3_required": "P3_recovered" in req or "P3_degraded_holds" in req,
        "p3_ok": all(req[k] for k in ("P3_recovered", "P3_degraded_holds") if k in req),
        "p3_episodes": g(m, "P3", "episodes"), "recovery_mean_s": g(m, "P3", "recovery_time_mean_s"),
        "recovery_max_s": g(m, "P3", "recovery_time_max_s"), "max_form_err_m": g(m, "P3", "max_formation_error_m"),
        "final_form_err_m": g(m, "P3", "final_formation_error_m"),
        "false_missing_episodes": g(m, "P3", "false_missing_episodes"),
        "false_missing_steps": g(m, "P3", "false_missing_steps"), "missing_episodes": g(m, "P3", "missing_episodes"),
        "missing_delay_mean_s": nanmean(delays), "false_rejoin": g(m, "P3", "false_rejoin"),
        "false_vacancy": g(m, "P3", "false_vacancy"), "vacancy": g(m, "P3", "vacancy"),
        "reinclusions": g(m, "P3", "reinclusions"), "wait_rejoin_s_mean": g(m, "P3", "wait_rejoin_s_mean"),
        "degraded_entries": g(m, "P3", "degraded_entries"),
        "determinism_violations": g(m, "automaton", "determinism_violations"),
        "observation_violations": g(m, "automaton", "observation_consistency_violations"),
        "transitions_per_drone_min": g(m, "automaton", "transitions_per_drone_min"),
        "chattering": g(m, "automaton", "chattering_events"), "failsafe_entries": g(m, "automaton", "failsafe_entries"),
        "failsafe_sensor_healthy": g(m, "automaton", "failsafe_entries_sensor_fault_healthy"),
        "failsafe_time_healthy_s": g(m, "automaton", "failsafe_time_healthy_s"),
        "end_err_max_m": max(m.get("end_position_error_m") or [math.nan]),
        "wall_s": m.get("wall_s"), "exit_code": m.get("exit_code"),
    }


def sensor_row(m) -> dict:
    s = m.get("sensor") or {}
    pct = g(s, "sonar", "range_err_m_p01_p05_p50_p95_p99") or [None] * 5
    w = m.get("noise_wrapper_stats") or {}
    return {"run": m["_id"], "phase": phase(m), "protocol": m["protocol"], "profile": m["profile"],
            "scenario": m["scenario"], "seed": m["seed"],
            "sonar_range_err_p50_m": pct[2], "sonar_range_err_p95_m": pct[3], "sonar_range_err_p99_m": pct[4],
            "sonar_range_err_max_m": g(s, "sonar", "range_err_max_m"),
            "sonar_frac_beyond_eps_far": g(s, "sonar", "frac_beyond_eps_far"),
            "sonar_frac_beyond_eps_near": g(s, "sonar", "frac_beyond_eps_near"),
            "echo_recall": g(s, "sonar", "recall"), "missed_detections": g(s, "sonar", "missed"),
            "false_dynamic": g(s, "sonar", "false_dynamic"), "false_dynamic_frac": g(s, "sonar", "false_dynamic_frac"),
            "stale_fraction": g(s, "sonar", "stale_fraction"), "longest_stale_s": g(s, "sonar", "longest_stale_s"),
            "dvl_vel_rmse_m_s": g(s, "dvl", "vel_err_rms_m_s"), "dvl_fresh_fraction": g(s, "dvl", "fresh_fraction"),
            "dvl_longest_gap_s": g(s, "dvl", "longest_gap_s"), "heading_err_rms_deg": g(s, "heading", "err_rms_deg"),
            "heading_err_p95_deg": g(s, "heading", "err_p95_abs_deg"), "depth_err_std_m": g(s, "depth", "err_std_m"),
            "gyro_err_rms_rad_s": g(s, "gyro", "yaw_rate_err_rms_rad_s"),
            "nav_err_max_m": g(s, "navigation", "pos_err_max_m"), "nav_err_rms_m": g(s, "navigation", "pos_err_rms_m"),
            **{f"wrapper_{k}": v for k, v in w.items()}}


def write_csv(path: Path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def timeline(run_dir: Path, limit: int = 60):
    out = []
    try:
        for line in open(run_dir / "events.jsonl", encoding="utf-8"):
            e = json.loads(line)
            if e.get("type") == "transition":
                out.append([round(e["t"], 1), e.get("drone"), e.get("from"), e.get("to"), e.get("reason")])
            elif e.get("type") in ("SLOT_DECLARED_VACANT", "SLOT_REOCCUPIED", "ENVELOPE_VIOLATION", "SENSOR_FAULT",
                                   "OBSERVATION_INCONSISTENT", "FAULT_INJECTED", "FAULT_CLEARED"):
                out.append([round(e["t"], 1), e.get("drone"), e["type"], e.get("slot"), e.get("reason")])
    except OSError:
        pass
    return out[:limit] + ([["...", len(out) - limit, "more events", None, None]] if len(out) > limit else [])


def evidence(m) -> dict:
    keys = {"P1": ("min_true_separation_m", "intruder_min_m", "collisions", "warning_entries", "ca_entries"),
            "P2": ("holds", "max_occupancy", "completed", "deadlock_events", "longest_stall_s", "false_busy_s",
                   "false_free_s", "queue_wait_mean_s"),
            "P3": ("all_recovered", "episodes", "recovery_time_max_s", "false_missing_episodes", "missing_episodes",
                   "false_rejoin", "false_vacancy", "vacancy", "degraded_entries", "absent_at_end"),
            "automaton": ("failsafe_entries", "failsafe_entries_sensor_fault_healthy", "failsafe_time_healthy_s",
                          "chattering_events", "end_modes")}
    ev = {sec: {k: g(m, sec, k) for k in ks} for sec, ks in keys.items()}
    s = m.get("sensor") or {}
    ev["sensor"] = {"sonar_recall": g(s, "sonar", "recall"), "false_dynamic_frac": g(s, "sonar", "false_dynamic_frac"),
                    "frac_beyond_eps_far": g(s, "sonar", "frac_beyond_eps_far"),
                    "longest_stale_s": g(s, "sonar", "longest_stale_s"), "nav_err_max_m": g(s, "navigation", "pos_err_max_m"),
                    "heading_err_p95_deg": g(s, "heading", "err_p95_abs_deg"),
                    "dvl_longest_gap_s": g(s, "dvl", "longest_gap_s")}
    ev["end_position_error_m"] = m.get("end_position_error_m")
    return ev


# ------------------------------------------------------------------------------------------------ tables
def class_counts(rows):
    c = Counter(r["class"] for r in rows)
    return [c.get(k, 0) for k in CLASSES]


def frac(k, n):
    if n == 0:
        return "-"
    lo, hi = wilson(k, n)
    return f"{k}/{n} ({100 * lo:.0f}-{100 * hi:.0f} %)"


def table_overview(rows, protocols=("B0",)):
    lines = ["| noise | protocol | runs | P1 pass | P2 pass | P3 pass | PASS | safe degraded | liveness failures | "
             "safety failures | invalid |", "|" + "---|" * 11]
    for prof in LEVELS:
        for pr in protocols:
            sel = [r for r in rows if r["profile"] == prof and r["protocol"] == pr and r["class"] != "INVALID"]
            allr = [r for r in rows if r["profile"] == prof and r["protocol"] == pr]
            if not allr:
                continue
            gate = [r for r in sel if r["gate"]]
            p3 = [r for r in sel if r["p3_required"]]
            cc = class_counts(allr)
            lines.append(f"| {prof} | {pr} | {len(allr)} | {frac(sum(r['p1_ok'] for r in sel), len(sel))} | "
                         f"{frac(sum(r['p2_ok'] for r in gate), len(gate))} | {frac(sum(r['p3_ok'] for r in p3), len(p3))} | "
                         f"{cc[0]} | {cc[1]} | {cc[2]} | {cc[3]} | {cc[4]} |")
    return "\n".join(lines)


def table_matrix(rows, scenarios, profiles, protocol="B0"):
    lines = ["| scenario | " + " | ".join(profiles) + " |", "|---|" + "---|" * len(profiles)]
    for sc in scenarios:
        cells = []
        for prof in profiles:
            sel = sorted((r for r in rows if r["scenario"] == sc and r["profile"] == prof and r["protocol"] == protocol),
                         key=lambda r: r["seed"])
            cells.append(" ".join(LETTER[r["class"]] for r in sel) or "-")
        lines.append(f"| `{sc}` | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def med(rows, key, fn=np.median):
    xs = [r[key] for r in rows if r.get(key) is not None and not (isinstance(r[key], float) and math.isnan(r[key]))]
    return None if not xs else round(float(fn(xs)), 3)


def table_metrics(rows, srows, profiles, protocol="B0"):
    spec = [("min true separation [m] (min)", rows, "min_sep_m", np.min),
            ("min true separation [m] (median)", rows, "min_sep_m", np.median),
            ("warning entries per run (mean)", rows, "warning_entries", np.mean),
            ("CA entries per run (mean)", rows, "ca_entries", np.mean),
            ("false missing episodes per run (mean)", rows, "false_missing_episodes", np.mean),
            ("false vacancies per run (mean)", rows, "false_vacancy", np.mean),
            ("P3 recovery time [s] (mean)", rows, "recovery_mean_s", np.mean),
            ("gate completion time [s] (mean)", rows, "completion_time_s", np.mean),
            ("queue wait per drone [s] (mean)", rows, "queue_wait_mean_s", np.mean),
            ("FAILSAFE entries per run (mean)", rows, "failsafe_entries", np.mean),
            ("chattering events per run (mean)", rows, "chattering", np.mean),
            ("transitions per drone-minute (mean)", rows, "transitions_per_drone_min", np.mean),
            ("sonar range error p95 [m] (median)", srows, "sonar_range_err_p95_m", np.median),
            ("sonar range error beyond eps_far (mean fraction)", srows, "sonar_frac_beyond_eps_far", np.mean),
            ("echo recall (median)", srows, "echo_recall", np.median),
            ("false dynamic associations (mean fraction)", srows, "false_dynamic_frac", np.mean),
            ("longest stale sonar interval [s] (max)", srows, "longest_stale_s", np.max),
            ("DVL velocity RMSE [m/s] (median)", srows, "dvl_vel_rmse_m_s", np.median),
            ("heading error RMS [deg] (median)", srows, "heading_err_rms_deg", np.median),
            ("depth error std [m] (median)", srows, "depth_err_std_m", np.median),
            ("navigation error max [m] (max)", srows, "nav_err_max_m", np.max)]
    lines = ["| metric | " + " | ".join(profiles) + " |", "|---|" + "---|" * len(profiles)]
    for label, rs, key, fn in spec:
        vals = [med([r for r in rs if r["profile"] == p and r["protocol"] == protocol], key, fn) for p in profiles]
        lines.append(f"| {label} | " + " | ".join("-" if v is None else f"{v:g}" for v in vals) + " |")
    return "\n".join(lines)


def table_isolation(rows, srows):
    iso = [r for r in rows if r["phase"] == "isolation"]
    if not iso:
        return "(no isolation runs)"
    scen = sorted({r["scenario"] for r in iso})
    clean = [r for r in rows if r["profile"] == "CLEAN" and r["protocol"] == "B0" and r["scenario"] in scen]
    s_by = {r["run"]: r for r in srows}
    lines = ["| factor (HIGH level, alone) | " + " | ".join(f"`{s}`" for s in scen) +
             " | false missing / run | FAILSAFE (sensor) / run | min sep [m] | nav err max [m] |",
             "|---|" + "---|" * (len(scen) + 4)]
    for prof in ["CLEAN"] + sorted({r["profile"] for r in iso}):
        sel = clean if prof == "CLEAN" else [r for r in iso if r["profile"] == prof]
        cells = [" ".join(LETTER[r["class"]] for r in sorted((x for x in sel if x["scenario"] == s), key=lambda r: r["seed"])) or "-"
                 for s in scen]
        nav = [s_by[r["run"]]["nav_err_max_m"] for r in sel if r["run"] in s_by and s_by[r["run"]]["nav_err_max_m"] is not None]
        lines.append(f"| {prof.replace('ISO_', '')} | " + " | ".join(cells) +
                     f" | {nanmean([r['false_missing_episodes'] for r in sel]) or 0:.2f} | "
                     f"{nanmean([r['failsafe_sensor_healthy'] for r in sel]) or 0:.2f} | "
                     f"{min((r['min_sep_m'] for r in sel if r['min_sep_m'] is not None), default=math.nan):.2f} | "
                     f"{max(nav, default=math.nan):.2f} |")
    return "\n".join(lines)


# ------------------------------------------------------------------------------------------------ figures
def fig_classes(rows, path: Path, protocols=("B0",), title="Run classes by noise level"):
    groups = [(p, pr) for p in LEVELS for pr in protocols if any(r["profile"] == p and r["protocol"] == pr for r in rows)]
    if not groups:
        return
    fig, ax = plt.subplots(figsize=(1.2 + 1.1 * len(groups), 3.4))
    bottom = np.zeros(len(groups))
    for cls in CLASSES:
        vals = np.array([sum(1 for r in rows if r["profile"] == p and r["protocol"] == pr and r["class"] == cls) /
                         max(1, sum(1 for r in rows if r["profile"] == p and r["protocol"] == pr)) for p, pr in groups])
        if vals.any():
            ax.bar(range(len(groups)), vals, bottom=bottom, color=CLASS_COLOR[cls], width=0.62, label=cls.lower().replace("_", " "),
                   edgecolor=INK["surface"], linewidth=1.0)
        bottom += vals
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([p if len(protocols) == 1 else f"{p}\n{pr}" for p, pr in groups])
    ax.set_ylim(0, 1)
    ax.set_ylabel("fraction of runs")
    ax.set_title(title)
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=8)
    save(fig, path)


def fig_success_by_family(rows, path: Path):
    fams = [f[0] for f in FAMILY]
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    x = np.arange(len(LEVELS))
    for i, fam in enumerate(fams):
        ks, ns, lo, hi = [], [], [], []
        for p in LEVELS:
            sel = [r for r in rows if r["protocol"] == "B0" and r["profile"] == p and r["family"] == fam and r["class"] != "INVALID"]
            k = sum(r["class"] == "PASS" for r in sel)
            ks.append(k / len(sel) if sel else np.nan)
            a, b = wilson(k, len(sel))
            lo.append(a)
            hi.append(b)
        off = (i - 1.5) * 0.08
        ks, lo, hi = np.array(ks), np.array(lo), np.array(hi)
        ax.errorbar(x + off, ks, yerr=[ks - lo, hi - ks], marker="o", ms=5, capsize=2, color=SERIES[i], label=fam)
    ax.set_xticks(x)
    ax.set_xticklabels(LEVELS)
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("PASS rate (Wilson 95 %)")
    ax.set_title("Success rate vs noise level (B0)")
    ax.legend(fontsize=8, loc="lower left")
    save(fig, path)


def fig_metric(rows, key, ylabel, title, path: Path, hlines=(), protocols=("B0",), only=None):
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    x = np.arange(len(LEVELS))
    for i, pr in enumerate(protocols):
        for j, p in enumerate(LEVELS):
            vals = [r[key] for r in rows if r["protocol"] == pr and r["profile"] == p and r.get(key) is not None
                    and not (isinstance(r[key], float) and math.isnan(r[key])) and (only is None or only(r))]
            if not vals:
                continue
            off = (i - (len(protocols) - 1) / 2) * 0.22
            jit = np.linspace(-0.06, 0.06, len(vals)) if len(vals) > 1 else np.zeros(1)
            ax.scatter(np.full(len(vals), j + off) + jit, vals, s=10, color=SERIES[i], alpha=0.55)
            ax.plot([j + off - 0.1, j + off + 0.1], [np.median(vals)] * 2, color=INK["primary"], lw=2)
    for y, lab in hlines:
        ax.axhline(y, color=STATUS["critical"] if "safe" in lab else INK["muted"], lw=1, ls="--")
        ax.text(len(LEVELS) - 0.55, y, lab, va="bottom", ha="right", fontsize=7, color=INK["secondary"])
    ax.set_xticks(x)
    ax.set_xticklabels(LEVELS)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if len(protocols) > 1:
        handles = [plt.Line2D([], [], marker="o", ls="", color=SERIES[i], label=pr) for i, pr in enumerate(protocols)]
        ax.legend(handles=handles, fontsize=8)
    save(fig, path)


def save(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  ", path.relative_to(ROOT))


# ------------------------------------------------------------------------------------------------ beacon A/B
AB_METRICS = (("false_missing_episodes", "false missing episodes per run", -1),
              ("false_vacancy", "false vacancies (false DEGRADED) per run", -1),
              ("degraded_entries", "DEGRADED_FORMATION entries per run", -1),
              ("wait_rejoin_s_mean", "time in WAIT_REJOIN per drone [s]", -1),
              ("missing_delay_mean_s", "true missing detection delay [s]", -1),
              ("recovery_mean_s", "P3 recovery time [s]", -1),
              ("queue_wait_mean_s", "gate queue wait per drone [s]", -1),
              ("completion_time_s", "gate completion time [s]", -1),
              ("failsafe_entries", "FAILSAFE entries per run", -1),
              ("chattering", "chattering events per run", -1),
              ("min_sep_m", "min true separation [m]", +1))


def paired(rows, key, profiles):
    by = {(r["protocol"], r["profile"], r["scenario"], r["seed"]): r for r in rows if r["class"] != "INVALID"}
    a, b = [], []
    for (pr, p, sc, s), r in by.items():
        if pr != "B1" or p not in profiles:
            continue
        r0 = by.get(("B0", p, sc, s))
        if r0 is None or r0.get(key) is None or r.get(key) is None:
            continue
        if any(isinstance(v, float) and math.isnan(v) for v in (r0[key], r[key])):
            continue
        a.append(float(r0[key]))
        b.append(float(r[key]))
    return np.array(a), np.array(b)


def ab_stats(a, b, sign):
    n = len(a)
    if n == 0:
        return None
    d = b - a
    rng = np.random.default_rng(0)
    boots = d[rng.integers(0, n, size=(10000, n))].mean(axis=1) if n > 1 else np.array([d.mean()])
    lo, hi = np.percentile(boots, [2.5, 97.5])
    p = None
    if n >= 6 and np.any(d != 0):
        from scipy.stats import wilcoxon

        p = float(wilcoxon(d, zero_method="zsplit").pvalue)
    better = int(np.sum(sign * d > 0))
    worse = int(np.sum(sign * d < 0))
    verdict = "no clear difference"
    if (sign > 0 and lo > 0) or (sign < 0 and hi < 0):
        verdict = "better with the beacon"
    elif (sign > 0 and hi < 0) or (sign < 0 and lo > 0):
        verdict = "worse with the beacon"
    return {"n_pairs": n, "mean_B0": round(float(a.mean()), 3), "mean_B1": round(float(b.mean()), 3),
            "mean_diff": round(float(d.mean()), 3), "ci95_lo": round(float(lo), 3), "ci95_hi": round(float(hi), 3),
            "wilcoxon_p": None if p is None else round(p, 4), "pairs_better": better, "pairs_worse": worse,
            "verdict": verdict}


def beacon_section(rows, runs):
    b1 = [r for r in rows if r["protocol"] == "B1"]
    if not b1:
        return
    profiles = sorted({r["profile"] for r in b1}, key=lambda p: LEVELS.index(p) if p in LEVELS else 9)
    comp = []
    for scope in [("all", tuple(profiles))] + [(p, (p,)) for p in profiles]:
        for key, label, sign in AB_METRICS:
            st = ab_stats(*paired(rows, key, scope[1]), sign)
            if st:
                comp.append({"profiles": scope[0], "metric": key, "label": label, **st})
    pass_pairs = defaultdict(lambda: [0, 0, 0, 0])            # (B0 pass, B1 pass): both, only B0, only B1, neither
    by = {(r["protocol"], r["profile"], r["scenario"], r["seed"]): r for r in rows}
    for (pr, p, sc, s), r in by.items():
        if pr != "B1":
            continue
        r0 = by.get(("B0", p, sc, s))
        if r0 is None:
            continue
        x0, x1 = r0["class"] == "PASS", r["class"] == "PASS"
        pass_pairs[p][0 if x0 and x1 else 1 if x0 else 2 if x1 else 3] += 1
    write_csv(BOUT / "comparison.csv", comp)
    brows = []
    for m in runs:
        if m["protocol"] != "B1":
            continue
        b = m.get("beacon") or {}
        brows.append({"run": m["_id"], "profile": m["profile"], "scenario": m["scenario"], "seed": m["seed"],
                      **{k: v for k, v in b.items() if not isinstance(v, (list, dict))}})
    write_csv(BOUT / "beacon_metrics.csv", brows)
    scen = sorted({r["scenario"] for r in b1})
    lines = ["# One-way presence beaconing: B0 (communication-free) vs B1 (one-way presence beacon)", "",
             "Generated by `scripts/analyze_sensor_robustness.py` from the raw runs; same scenarios, noise profiles and "
             "seeds for both protocols (pairs keyed by profile, scenario and seed).  Differences B1 - B0 with a seeded "
             "bootstrap 95 % interval and a Wilcoxon signed-rank p-value; \"better\" or \"worse\" only when the interval "
             "excludes zero.", "", "## Run classes", "",
             table_overview([r for r in rows if r["scenario"] in scen and r["profile"] in profiles], ("B0", "B1")), "",
             "Paired PASS outcomes (both / only B0 / only B1 / neither): " +
             "; ".join(f"{p}: {v[0]} / {v[1]} / {v[2]} / {v[3]}" for p, v in sorted(pass_pairs.items())), "",
             "## Paired metrics", "", "| profiles | metric | pairs | B0 mean | B1 mean | B1 - B0 [95 % CI] | Wilcoxon p | verdict |",
             "|---|---|---|---|---|---|---|---|"]
    for c in comp:
        lines.append(f"| {c['profiles']} | {c['label']} | {c['n_pairs']} | {c['mean_B0']} | {c['mean_B1']} | "
                     f"{c['mean_diff']:+g} [{c['ci95_lo']:+g}, {c['ci95_hi']:+g}] | {c['wilcoxon_p'] if c['wilcoxon_p'] is not None else '-'} | "
                     f"{c['verdict']} |")
    if brows:
        keys = [k for k in ("transmissions", "receptions", "reception_rate", "lost_collision", "lost_channel",
                            "lost_dropout", "latency_mean_s", "latency_p95_s") if k in brows[0]]
        lines += ["", "## Beacon channel (per run, mean by profile)", "", "| profile | " + " | ".join(keys) + " |",
                  "|---|" + "---|" * len(keys)]
        for p in profiles:
            sel = [r for r in brows if r["profile"] == p]
            lines.append(f"| {p} | " + " | ".join(f"{nanmean([r.get(k) for r in sel]):.4g}" if nanmean([r.get(k) for r in sel]) is not None else "-"
                                                  for k in keys) + " |")
    lines += ["", "## Class matrix (seeds 0, 1, 2; P pass, D safe degraded, L liveness failure, S safety failure, I invalid)", ""]
    for pr in ("B0", "B1"):
        lines += [f"**{pr}**", "", table_matrix(rows, scen, profiles, pr), ""]
    lines += ["## Figures", "", "![classes](figures/classes_b0_b1.png)", "", "![false missing](figures/false_missing_b0_b1.png)", "",
              "![recovery](figures/recovery_b0_b1.png)", "", "![gate wait](figures/gate_wait_b0_b1.png)", ""]
    (BOUT / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    ab = [r for r in rows if r["scenario"] in scen and r["profile"] in profiles]
    fig_classes(ab, BOUT / "figures" / "classes_b0_b1.png", ("B0", "B1"), "Run classes, B0 vs B1")
    fig_metric(ab, "false_missing_episodes", "false missing episodes per run", "False missing, B0 vs B1",
               BOUT / "figures" / "false_missing_b0_b1.png", protocols=("B0", "B1"))
    fig_metric(ab, "recovery_mean_s", "P3 recovery time [s]", "Recovery time, B0 vs B1",
               BOUT / "figures" / "recovery_b0_b1.png", protocols=("B0", "B1"))
    fig_metric(ab, "queue_wait_mean_s", "queue wait per drone [s]", "Gate queue wait, B0 vs B1",
               BOUT / "figures" / "gate_wait_b0_b1.png", protocols=("B0", "B1"), only=lambda r: r["gate"])
    print("  ", (BOUT / "SUMMARY.md").relative_to(ROOT))


# ------------------------------------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=str(RAW))
    a = ap.parse_args()
    style()
    runs = load(Path(a.raw))
    if not runs:
        print("no runs")
        return 1
    rows = [flat(m) for m in runs]
    srows = [sensor_row(m) for m in runs]
    write_csv(OUT / "summary.csv", rows)
    write_csv(OUT / "sensor_metrics.csv", srows)

    from holo_fleet.sim.sensor_noise import FACTORS, PROFILES, isolated

    (OUT / "noise_profiles.json").write_text(json.dumps(
        {"note": "simulation stress levels, not real sensor accuracy (docs/SENSOR_ROBUSTNESS_AND_BEACONING.md)",
         "profiles": {k: p.as_dict() for k, p in PROFILES.items()},
         "isolation_factors": {f: {"fields": list(v), "profile": isolated(f).as_dict()} for f, v in FACTORS.items()}},
        indent=1), encoding="utf-8")
    diag_path = OUT / "diagnoses.json"
    diag = json.loads(diag_path.read_text(encoding="utf-8")) if diag_path.exists() else {}
    fails = [{"run": m["_id"], "phase": phase(m), "class": m["class"], "failed_requirements": m.get("failed_requirements"),
              "invalid_reason": m.get("invalid_reason"), "diagnosis": diag.get(m["_id"]), "evidence": evidence(m),
              "timeline": timeline(m["_dir"])} for m in runs if m["class"] != "PASS"]
    (OUT / "failure_cases.json").write_text(json.dumps(fails, indent=1, default=str), encoding="utf-8")

    base = [r for r in rows if r["protocol"] == "B0"]
    sbase = [r for r in srows if r["protocol"] == "B0"]
    scen = list(dict.fromkeys(r["scenario"] for r in sorted(base, key=lambda r: r["scenario"])))
    from run_sensor_robustness_campaign import ALL_SCENARIOS

    scen = [s for s in ALL_SCENARIOS if s in scen]
    profs = [p for p in LEVELS if any(r["profile"] == p for r in base)]
    n_inv = sum(r["class"] == "INVALID" for r in rows)
    lines = ["# Sensor robustness of the communication-free baseline (B0)", "",
             "Generated by `scripts/analyze_sensor_robustness.py` from the raw runs of "
             "`scripts/run_sensor_robustness_campaign.py` (patched HoloOcean engine, headless, seeds 0, 1, 2).  Noise "
             "levels are simulation stress levels, not real sensor accuracy ([noise_profiles.json](noise_profiles.json), "
             "[method](../../docs/SENSOR_ROBUSTNESS_AND_BEACONING.md)).", "",
             f"Runs: {len(rows)} ({len(base)} B0, {len(rows) - len(base)} B1), {n_inv} invalid.  Classes: PASS, safe degraded "
             "(no safety violation, conservative degradation), liveness failure (safe, a required outcome missing), safety "
             "failure (P1 or P2 violated or a contact), invalid (infrastructure).", "",
             "## Overview (B0)", "", table_overview(base), "",
             "## Class matrix (B0; seeds 0, 1, 2; P pass, D safe degraded, L liveness failure, S safety failure, I invalid)",
             "", table_matrix(base, scen, profs), "", "## Isolation of the phenomena (one factor at its HIGH level)", "",
             table_isolation(rows, srows), "", "## Sensor and behaviour metrics by noise level (B0)", "",
             table_metrics(base, sbase, profs), "", "## Failure cases", ""]
    if fails:
        lines += ["| run | class | failed requirements | diagnosis |", "|---|---|---|---|"]
        for f in fails:
            dg = f["diagnosis"] or {}
            lines.append(f"| `{f['run']}` | {f['class']} | {' '.join(f['failed_requirements'] or []) or f['invalid_reason'] or '-'} | "
                         f"{(dg.get('category', '') + ' ' + dg.get('cause', '')).strip() or 'not yet diagnosed'} |")
    else:
        lines.append("None.")
    lines += ["", "Evidence and automaton timelines of every case: [failure_cases.json](failure_cases.json); per-run "
              "metrics: [summary.csv](summary.csv), [sensor_metrics.csv](sensor_metrics.csv).", "", "## Figures", ""]
    figs = [("success_rate_vs_noise.png", "success rate vs noise"), ("classes_vs_noise.png", "failure-mode distribution"),
            ("min_separation_vs_noise.png", "minimum separation vs noise"), ("recovery_time_vs_noise.png", "recovery time vs noise"),
            ("gate_vs_noise.png", "gate completion time vs noise"), ("false_missing_vs_noise.png", "false missing vs noise")]
    lines += [f"![{lab}](figures/{fn})" for fn, lab in figs]
    (OUT / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("  ", (OUT / "SUMMARY.md").relative_to(ROOT))

    main_rows = [r for r in base if r["phase"] in ("main", "high")]
    F = OUT / "figures"
    fig_success_by_family(main_rows, F / "success_rate_vs_noise.png")
    fig_classes(main_rows, F / "classes_vs_noise.png")
    fig_metric(main_rows, "min_sep_m", "min true separation [m]", "Minimum separation vs noise (B0)",
               F / "min_separation_vs_noise.png", hlines=((1.0, "d_safe 1.0 m"), (2.4, "warning 2.4 m")))
    fig_metric(main_rows, "recovery_mean_s", "P3 recovery time [s]", "Recovery time vs noise (B0)", F / "recovery_time_vs_noise.png")
    fig_metric(main_rows, "completion_time_s", "gate completion time [s]", "Gate completion vs noise (B0)",
               F / "gate_vs_noise.png", only=lambda r: r["gate"])
    fig_metric(main_rows, "false_missing_episodes", "false missing episodes per run", "False missing vs noise (B0)",
               F / "false_missing_vs_noise.png")
    beacon_section(rows, runs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
