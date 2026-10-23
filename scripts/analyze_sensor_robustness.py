"""Analysis of the sensor-robustness campaign and of the beacon ablation (raw runs -> versioned summaries).

    python scripts/analyze_sensor_robustness.py [--raw results/robustness/raw_final] [--pre results/robustness/raw]

Reads every <raw>/<protocol>/<profile>/<scenario>/s<seed>/run_metrics.json written by
scripts/run_sensor_robustness_campaign.py.  ``--raw``: the official campaign (one frozen commit, checked in the
Provenance section); ``--pre``: the pre-freeze campaign, used only for the isolation diagnosis (one factor at a
time) and for a run-to-run agreement check of the baseline.  Writes:

* results/robustness/  summary.csv (one row per official run), sensor_metrics.csv, isolation_summary.csv (pre-freeze
                       isolation runs), noise_profiles.json,
                       failure_cases.json (every run that is not PASS, with its evidence and timeline),
                       SUMMARY.md; figures in figures/robustness/;
* results/beaconing/   comparison.csv, beacon_metrics.csv, SUMMARY.md; figures in figures/beaconing/ (when B1 runs
                       exist).  (results/**/figures/ is git-ignored: the versioned figures live under figures/.)

Every run that is not PASS gets a diagnosis (category A real bug, B assumption exceeded, C conservative false
positive, D liveness issue, E simulation artifact; INFRA for an invalid run) from the rules of ``diagnose`` applied
to its evidence; a case the rules do not explain is marked "?" for manual analysis.  Hand-written diagnoses in
results/robustness/diagnoses.json (key: run id, "pre:" + run id for the pre-freeze runs) override the rules.  Proportions carry a Wilson 95 %
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

RAW = ROOT / "results" / "robustness" / "raw_final"
RAW_PRE = ROOT / "results" / "robustness" / "raw"
OUT = ROOT / "results" / "robustness"
BOUT = ROOT / "results" / "beaconing"
FIG = ROOT / "figures" / "robustness"
BFIG = ROOT / "figures" / "beaconing"
LEVELS = ("CLEAN", "LOW", "MEDIUM", "HIGH")
# position of each level with respect to the sensing assumptions of the verified design (docs, "Profiles")
ASSUMPTIONS = {"CLEAN": "INSIDE", "LOW": "INSIDE", "MEDIUM": "NEAR", "HIGH": "OUTSIDE"}
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
        try:
            sep = json.loads((p.parent / "referee_metrics.json").read_text(encoding="utf-8"))["P1_separation"]
            m["_drone_contacts"] = int(sep.get("physical_contacts") or 0)
        except (OSError, ValueError, KeyError):
            m["_drone_contacts"] = None
        runs.append(m)
    return runs


def flat(m) -> dict:
    req = m.get("requirements") or {}
    delays = [d for d in (g(m, "P3", "true_missing_delay_s") or []) if d is not None]
    return {
        "run": m["_id"], "phase": phase(m), "protocol": m["protocol"], "profile": m["profile"],
        "scenario": m["scenario"], "family": family(m["scenario"]), "seed": m["seed"], "class": m["class"],
        "failed_requirements": " ".join(m.get("failed_requirements") or []),
        # P1 is the property among the drones (no drone closer than d_safe, no drone-drone contact); contacts with
        # the gate or a non-cooperative vehicle and that vehicle closer than d_safe are safety failures as well
        "p1_ok": bool(g(m, "P1", "holds")) and not m.get("_drone_contacts"),
        "vehicle_clear": None if g(m, "P1", "intruder_min_m") is None else
        bool(g(m, "P1", "intruder_min_m") >= 1.0 and not g(m, "P1", "collisions")),
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
        "r1_held_episodes": g(m, "P3", "beacon_r1", "held_episodes"),
        "r1_held_then_missing": g(m, "P3", "beacon_r1", "held_then_missing"),
        "r1_stale_presence_episodes": g(m, "P3", "beacon_r1", "stale_presence_episodes"),
        "r1_stale_presence_s": g(m, "P3", "beacon_r1", "stale_presence_s"),
        "r1_held_s": g(m, "P3", "beacon_r1", "held_s"), "r1_heard_fraction": g(m, "P3", "beacon_r1", "heard_fraction"),
        "code_sha": g(m, "code_version", "sha"), "code_dirty": bool(g(m, "code_version", "dirty_files")),
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


GATE_SCENARIOS = ("gate_single", "integrated_short", "mutex_deadlock_resolution", "line_parallel_mutex", "lost_drone_mutex")


def diagnose(m) -> dict:
    """Rule-based category and cause of a non-PASS run (see the module docstring)."""
    prof, sc, cls = m["profile"], m["scenario"], m["class"]
    level = "HIGH" if prof.startswith("ISO_") else prof
    where = ASSUMPTIONS.get(level, "OUTSIDE")
    nav = g(m, "sensor", "navigation", "pos_err_max_m")
    stale = g(m, "sensor", "sonar", "longest_stale_s")
    fs_n = g(m, "automaton", "failsafe_entries_sensor_fault_healthy", default=0)
    fs_t = g(m, "automaton", "failsafe_time_healthy_s", default=0)
    fails = set(m.get("failed_requirements") or [])
    out = {"assumptions": where, "source": "rule"}
    if cls == "INVALID":
        return {**out, "category": "INFRA", "cause": f"infrastructure ({m.get('invalid_reason')}): excluded, retried"}
    if cls == "SAFETY_FAILURE":
        if g(m, "P1", "intruder_min_m") is not None and g(m, "P1", "holds") and not m.get("_drone_contacts"):
            return {**out, "category": "B", "cause":
                    f"the non-cooperative vehicle came within {g(m, 'P1', 'intruder_min_m')} m of a drone: sonar data older "
                    f"than tau_max (longest gap {stale} s) kept healthy drones in sensor-fault FAILSAFE for {fs_t} s and the "
                    "noisy ranging changed the escape direction between captures; P1 among the drones holds"}
        return {**out, "category": "?", "cause": "safety failure among the drones or with the gate: manual analysis"}
    if cls == "SAFE_DEGRADED":
        cat = "B" if where == "OUTSIDE" else "C"
        if g(m, "P3", "false_vacancy"):
            return {**out, "category": cat, "cause":
                    f"false vacancy: with a navigation error up to {nav} m a returning or present neighbour was not "
                    "confirmed near its slot before t_rejoin, then re-included"}
        return {**out, "category": cat, "cause":
                f"{fs_n} sensor-fault FAILSAFE entries of healthy drones ({fs_t} s): sonar data older than tau_max "
                f"(longest gap {stale} s); missing outcome: {', '.join(sorted(fails)) or '-'}"}
    if cls == "LIVENESS_FAILURE":
        factor = prof[4:] if prof.startswith("ISO_") else None
        fdyn = g(m, "sensor", "sonar", "false_dynamic_frac") or 0.0
        if sc == "p1_vertical_escape":
            return {**out, "category": "D", "cause":
                    "finite horizon: formation recovery after the vertical escape not completed within the run "
                    "(also in CLEAN); safe"}
        if factor == "imu_bias":
            return {**out, "category": "B", "cause":
                    "gyro bias far beyond a calibrated IMU (random walk to several deg/s): the heading hold damps with "
                    "the raw gyro rate, so it holds the heading off by (k_r / k_psi) x bias, beyond the 6 deg heading "
                    "tolerance of the queue point; the drone waits at its queue point without ever counting as there "
                    "(safe, no PASS)"}
        if factor == "latency":
            return {**out, "category": "B", "cause":
                    f"sonar latency 0.12 s: echoes ranged from a pose 0.12 s old (range error beyond eps_far in "
                    f"{100 * (g(m, 'sensor', 'sonar', 'frac_beyond_eps_far') or 0):.0f} % of the samples), false busy "
                    f"critical region {g(m, 'P2', 'false_busy_s')} s: slower gate passage, formation still recovering "
                    "at the end of the run (safe)"}
        if fdyn > 0.2:
            return {**out, "category": "B", "cause":
                    f"spurious echoes: {100 * fdyn:.0f} % of the dynamic associations are false (intensity noise beyond "
                    f"the detection threshold), {g(m, 'P1', 'ca_entries')} COLLISION_AVOIDANCE entries and a false busy "
                    f"critical region ({g(m, 'P2', 'false_busy_s')} s); the fleet stays safe but does not complete"}
        if sc in GATE_SCENARIOS and nav is not None and nav > 0.30:
            return {**out, "category": "B", "cause":
                    f"navigation error up to {nav} m beyond the gate-map tolerance (0.30 m): bar echoes are not "
                    "explained by the map and stay obstacles, the avoidance keeps the drone off the gate (safe livelock)"}
        return {**out, "category": "?", "cause": f"liveness failure ({', '.join(sorted(fails))}): manual analysis"}
    return {**out, "category": "?", "cause": "manual analysis"}


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
    """Profile | Valid runs | P1 / P2 / P3 pass | classes.  INVALID runs (infrastructure) are excluded from every
    denominator and counted in the last column only."""
    lines = ["| Profile | Assumptions | Protocol | Valid runs | P1 pass (drones) | P2 pass | P3 pass | PASS | Safe degraded | "
             "Liveness failures | Safety failures | Invalid (excluded) |", "|" + "---|" * 12]
    for prof in LEVELS:
        for pr in protocols:
            sel = [r for r in rows if r["profile"] == prof and r["protocol"] == pr and r["class"] != "INVALID"]
            allr = [r for r in rows if r["profile"] == prof and r["protocol"] == pr]
            if not allr:
                continue
            gate = [r for r in sel if r["gate"]]
            p3 = [r for r in sel if r["p3_required"]]
            cc = class_counts(allr)
            lines.append(f"| {prof} | {ASSUMPTIONS.get(prof, '-')} | {pr} | {len(sel)} | {frac(sum(r['p1_ok'] for r in sel), len(sel))} | "
                         f"{frac(sum(r['p2_ok'] for r in gate), len(gate))} | {frac(sum(r['p3_ok'] for r in p3), len(p3))} | "
                         f"{cc[0]} | {cc[1]} | {cc[2]} | {cc[3]} | {cc[4]} |")
    return "\n".join(lines)


FIRST_FAILURE = (
    ("P1: drones never closer than d_safe, no drone-drone contact", lambda r: True, lambda r: r["p1_ok"]),
    ("clearance from the non-cooperative vehicle (>= d_safe, no contact)", lambda r: r["vehicle_clear"] is not None,
     lambda r: r["vehicle_clear"]),
    ("P2: at most one drone in a critical region", lambda r: r["gate"], lambda r: r["p2_ok"]),
    ("P3: formation requirement of the scenario (recovered / degraded holds)", lambda r: r["p3_required"],
     lambda r: r["p3_ok"]),
    ("gate completed by every drone (liveness)", lambda r: r["gate"], lambda r: "gate_completed" not in r["failed_requirements"]),
    ("mission progress within 3 m of the reference end (liveness)", lambda r: True,
     lambda r: "progress" not in r["failed_requirements"]),
    ("no healthy drone in FAILSAFE at the end", lambda r: True, lambda r: "no_failsafe_at_end" not in r["failed_requirements"]),
    ("no false vacancy (no false DEGRADED_FORMATION)", lambda r: True, lambda r: not r["false_vacancy"]),
    ("run classified PASS", lambda r: True, lambda r: r["class"] == "PASS"),
)


def table_first_failure(rows):
    """First noise level at which each property fails in at least one valid run, and where that level stands with
    respect to the assumptions."""
    lines = ["| property | first failing level | assumptions at that level | failing / valid runs there | scenarios |",
             "|---|---|---|---|---|"]
    for label, applies, ok in FIRST_FAILURE:
        first = None
        for prof in LEVELS:
            sel = [r for r in rows if r["profile"] == prof and r["class"] != "INVALID" and applies(r)]
            bad = [r for r in sel if not ok(r)]
            if bad:
                first = (prof, len(bad), len(sel), sorted({r["scenario"] for r in bad}))
                break
        if first is None:
            lines.append(f"| {label} | none (holds at every level) | - | - | - |")
        else:
            prof, nb, ns, scs = first
            lines.append(f"| {label} | {prof} | {ASSUMPTIONS.get(prof, '-')} | {nb}/{ns} | "
                         + ", ".join(f"`{x}`" for x in scs[:6]) + (" ..." if len(scs) > 6 else "") + " |")
    return "\n".join(lines)


def table_provenance(runs, label):
    by = Counter((g(m, "code_version", "sha") or "?", bool(g(m, "code_version", "dirty_files"))) for m in runs)
    lines = [f"| {label}: commit | working tree | runs |", "|---|---|---|"]
    for (sha, dirty), n in sorted(by.items(), key=lambda kv: -kv[1]):
        lines.append(f"| `{sha[:12]}` | {'dirty' if dirty else 'clean'} | {n} |")
    return "\n".join(lines)


def table_agreement(rows, pre_rows):
    """Run-to-run agreement of the baseline: the same (profile, scenario, seed) in the pre-freeze campaign and in
    the official one (HoloOcean is not bit-reproducible, so an identical class is not guaranteed)."""
    pre = {(r["profile"], r["scenario"], r["seed"]): r for r in pre_rows if r["protocol"] == "B0" and r["phase"] != "isolation"}
    pairs = [(pre[k], r) for r in rows if r["protocol"] == "B0" for k in [(r["profile"], r["scenario"], r["seed"])]
             if k in pre and pre[k]["class"] != "INVALID" and r["class"] != "INVALID"]
    if not pairs:
        return "(no common runs)", []
    diff = [(a, b) for a, b in pairs if a["class"] != b["class"]]
    lines = ["| profile | common runs | same class | different class |", "|---|---|---|---|"]
    for prof in LEVELS:
        sel = [(a, b) for a, b in pairs if b["profile"] == prof]
        if sel:
            nd = sum(a["class"] != b["class"] for a, b in sel)
            lines.append(f"| {prof} | {len(sel)} | {len(sel) - nd} | {nd} |")
    return "\n".join(lines), diff


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
              ("missing_episodes", "missing declarations per run (true and false)", -1),
              ("false_vacancy", "false vacancies (false DEGRADED) per run", -1),
              ("degraded_entries", "DEGRADED_FORMATION entries per run", -1),
              ("wait_rejoin_s_mean", "time in WAIT_REJOIN per drone [s]", -1),
              ("missing_delay_mean_s", "true missing detection delay [s]", -1),
              ("recovery_mean_s", "P3 recovery time [s]", -1),
              ("queue_wait_mean_s", "gate queue wait per drone [s]", -1),
              ("completion_time_s", "gate completion time [s]", -1),
              ("failsafe_entries", "FAILSAFE entries per run", -1),
              ("chattering", "chattering events per run", -1),
              ("ca_entries", "COLLISION_AVOIDANCE entries per run", -1),
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
    lines = ["# One-way acoustic presence beaconing: B0 communication-free vs B1 beacon-assisted", "",
             "B0: decentralized, communication-free coordination driven by shared mission constraints and local "
             "sensing.  B1: the same architecture augmented with periodic one-way acoustic presence beacons (static "
             "identity only; one onboard use, rule R1).", "",
             "Generated by `scripts/analyze_sensor_robustness.py` from the raw runs; same scenarios, noise profiles and "
             "seeds for both protocols (pairs keyed by profile, scenario and seed).  Differences B1 - B0 with a seeded "
             "bootstrap 95 % interval and a Wilcoxon signed-rank p-value; \"better\" or \"worse\" only when the interval "
             "excludes zero.", "", "## Run classes", "",
             table_overview([r for r in rows if r["scenario"] in scen and r["profile"] in profiles], ("B0", "B1")), "",
             "Paired PASS outcomes (both / only B0 / only B1 / neither): " +
             "; ".join(f"{p}: {v[0]} / {v[1]} / {v[2]} / {v[3]}" for p, v in sorted(pass_pairs.items())), "",
             "## Paired metrics (all noise levels)", "",
             "| Metric | B0 communication-free | B1 beacon-assisted | Difference B1 - B0 [95 % CI] | pairs | Wilcoxon p | verdict |",
             "|---|---|---|---|---|---|---|"]
    for c in comp:
        if c["profiles"] != "all":
            continue
        lines.append(f"| {c['label']} | {c['mean_B0']} | {c['mean_B1']} | {c['mean_diff']:+g} [{c['ci95_lo']:+g}, {c['ci95_hi']:+g}] | "
                     f"{c['n_pairs']} | {c['wilcoxon_p'] if c['wilcoxon_p'] is not None else '-'} | {c['verdict']} |")
    lines += ["", "## Paired metrics by noise level", "",
              "| profile | Metric | B0 communication-free | B1 beacon-assisted | Difference B1 - B0 [95 % CI] | pairs | Wilcoxon p | verdict |",
              "|---|---|---|---|---|---|---|---|"]
    for c in comp:
        if c["profiles"] == "all":
            continue
        lines.append(f"| {c['profiles']} | {c['label']} | {c['mean_B0']} | {c['mean_B1']} | "
                     f"{c['mean_diff']:+g} [{c['ci95_lo']:+g}, {c['ci95_hi']:+g}] | {c['n_pairs']} | "
                     f"{c['wilcoxon_p'] if c['wilcoxon_p'] is not None else '-'} | {c['verdict']} |")
    if brows:
        keys = [k for k in ("transmissions", "expected_receptions", "receptions", "reception_rate", "lost_collision",
                            "lost_channel", "lost_dropout", "latency_mean_ms", "latency_max_ms", "longest_absence_s")
                if k in brows[0]]
        lines += ["", "## Beacon channel (per run, mean by profile)", "", "| profile | " + " | ".join(keys) + " |",
                  "|---|" + "---|" * len(keys)]
        for p in profiles:
            sel = [r for r in brows if r["profile"] == p]
            lines.append(f"| {p} | " + " | ".join(f"{nanmean([r.get(k) for r in sel]):.4g}" if nanmean([r.get(k) for r in sel]) is not None else "-"
                                                  for k in keys) + " |")
    r1keys = ("r1_held_episodes", "r1_held_then_missing", "r1_stale_presence_episodes", "r1_stale_presence_s",
              "r1_held_s", "r1_heard_fraction")
    lines += ["", "## Rule R1 onboard (per run, mean by profile)", "",
              "Episodes in which presence deferred a MISSING declaration; *then missing*: the deferral ended in the "
              "declaration (delayed by at most missing_grace_s), the others ended with the sonar finding the neighbour "
              "again; *stale presence*: deferred while the neighbour was really away from its slot (beyond e_lost).", "",
              "| profile | " + " | ".join(k[3:].replace("_", " ") for k in r1keys) + " |", "|---|" + "---|" * len(r1keys)]
    for p in profiles:
        sel = [r for r in b1 if r["profile"] == p and r["class"] != "INVALID"]
        vals = [nanmean([r.get(k) for r in sel]) for k in r1keys]
        lines.append(f"| {p} | " + " | ".join("-" if v is None else f"{v:.3g}" for v in vals) + " |")
    lines += ["", "## Class matrix (seeds 0, 1, 2; P pass, D safe degraded, L liveness failure, S safety failure, I invalid)", ""]
    for pr in ("B0", "B1"):
        lines += [f"**{pr}**", "", table_matrix(rows, scen, profiles, pr), ""]
    lines += ["## Figures", ""] + [f"![{lab}](../../figures/beaconing/{fn})\n" for fn, lab in (
        ("classes_b0_b1.png", "classes"), ("false_missing_b0_b1.png", "false missing"), ("recovery_b0_b1.png", "recovery"),
        ("gate_wait_b0_b1.png", "gate wait"))]
    (BOUT / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    ab = [r for r in rows if r["scenario"] in scen and r["profile"] in profiles]
    fig_classes(ab, BFIG / "classes_b0_b1.png", ("B0", "B1"), "Run classes, B0 vs B1")
    fig_metric(ab, "false_missing_episodes", "false missing episodes per run", "False missing, B0 vs B1",
               BFIG / "false_missing_b0_b1.png", protocols=("B0", "B1"))
    fig_metric(ab, "recovery_mean_s", "P3 recovery time [s]", "Recovery time, B0 vs B1",
               BFIG / "recovery_b0_b1.png", protocols=("B0", "B1"))
    fig_metric(ab, "queue_wait_mean_s", "queue wait per drone [s]", "Gate queue wait, B0 vs B1",
               BFIG / "gate_wait_b0_b1.png", protocols=("B0", "B1"), only=lambda r: r["gate"])
    print("  ", (BOUT / "SUMMARY.md").relative_to(ROOT))


def fig_properties(rows, path: Path):
    """P1 / P2 / P3 pass rates and the PASS rate per noise level (B0, Wilson 95 % intervals)."""
    spec = [("P1 (drones)", lambda r: True, lambda r: r["p1_ok"]), ("P2", lambda r: r["gate"], lambda r: r["p2_ok"]),
            ("P3", lambda r: r["p3_required"], lambda r: r["p3_ok"]), ("PASS", lambda r: True, lambda r: r["class"] == "PASS")]
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    x = np.arange(len(LEVELS))
    for i, (lab, applies, ok) in enumerate(spec):
        ks, lo, hi = [], [], []
        for p in LEVELS:
            sel = [r for r in rows if r["protocol"] == "B0" and r["profile"] == p and r["class"] != "INVALID" and applies(r)]
            k = sum(bool(ok(r)) for r in sel)
            ks.append(k / len(sel) if sel else np.nan)
            a, b = wilson(k, len(sel))
            lo.append(a)
            hi.append(b)
        ks, lo, hi = np.array(ks), np.array(lo), np.array(hi)
        ax.errorbar(x + (i - 1.5) * 0.08, ks, yerr=[ks - lo, hi - ks], marker="o", ms=5, capsize=2, color=SERIES[i], label=lab)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{p}\n{ASSUMPTIONS[p].lower()}" for p in LEVELS])
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("pass rate (Wilson 95 %)")
    ax.set_title("Properties vs noise level (B0)")
    ax.legend(fontsize=8, loc="lower left")
    save(fig, path)


# ------------------------------------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=str(RAW), help="official campaign (one frozen commit)")
    ap.add_argument("--pre", default=str(RAW_PRE), help="pre-freeze campaign: isolation diagnosis, agreement check")
    a = ap.parse_args()
    style()
    runs = load(Path(a.raw))
    if not runs:
        print("no runs")
        return 1
    pre_runs = load(Path(a.pre)) if Path(a.pre).resolve() != Path(a.raw).resolve() else []
    rows = [flat(m) for m in runs]
    srows = [sensor_row(m) for m in runs]
    pre_rows = [flat(m) for m in pre_runs]
    pre_srows = [sensor_row(m) for m in pre_runs]
    write_csv(OUT / "summary.csv", rows)
    write_csv(OUT / "sensor_metrics.csv", srows)

    from holo_fleet.sim.sensor_noise import FACTORS, PROFILES, isolated

    (OUT / "noise_profiles.json").write_text(json.dumps(
        {"note": "simulation stress levels, not real sensor accuracy (docs/SENSOR_ROBUSTNESS_AND_BEACONING.md)",
         "assumptions": ASSUMPTIONS,
         "profiles": {k: p.as_dict() for k, p in PROFILES.items()},
         "isolation_factors": {f: {"fields": list(v), "profile": isolated(f).as_dict()} for f, v in FACTORS.items()}},
        indent=1), encoding="utf-8")
    diag_path = OUT / "diagnoses.json"
    diag = json.loads(diag_path.read_text(encoding="utf-8")) if diag_path.exists() else {}

    def case(m, source):
        return {"run": m["_id"], "source": source, "phase": phase(m), "class": m["class"],
                "failed_requirements": m.get("failed_requirements"), "invalid_reason": m.get("invalid_reason"),
                "diagnosis": diag.get(m["_id"] if source == "official" else "pre:" + m["_id"]) or diagnose(m),
                "evidence": evidence(m), "timeline": timeline(m["_dir"])}

    fails = [case(m, "official") for m in runs if m["class"] != "PASS"]
    fails += [case(m, "pre-freeze isolation") for m in pre_runs if phase(m) == "isolation" and m["class"] != "PASS"]
    (OUT / "failure_cases.json").write_text(json.dumps(fails, indent=1, default=str), encoding="utf-8")

    base = [r for r in rows if r["protocol"] == "B0"]
    sbase = [r for r in srows if r["protocol"] == "B0"]
    from run_sensor_robustness_campaign import ALL_SCENARIOS

    scen = [s for s in ALL_SCENARIOS if any(r["scenario"] == s for r in base)]
    profs = [p for p in LEVELS if any(r["profile"] == p for r in base)]
    n_inv = sum(r["class"] == "INVALID" for r in rows)
    iso_rows = [r for r in pre_rows if r["phase"] == "isolation" or (r["profile"] == "CLEAN" and r["protocol"] == "B0")]
    write_csv(OUT / "isolation_summary.csv", [r for r in iso_rows if r["phase"] == "isolation"])
    agree, diffs = table_agreement(rows, pre_rows)
    veh = [r for r in base if r["vehicle_clear"] is not None]
    lines = ["# Sensor robustness of the communication-free baseline (B0)", "",
             "Generated by `scripts/analyze_sensor_robustness.py` from the raw runs of "
             "`scripts/run_sensor_robustness_campaign.py` (patched HoloOcean engine, headless, seeds 0, 1, 2).  B0: "
             "decentralized, communication-free coordination driven by shared mission constraints and local sensing.  "
             "Noise levels are simulation stress levels, not real sensor accuracy ([noise_profiles.json](noise_profiles.json), "
             "[method](../../docs/SENSOR_ROBUSTNESS_AND_BEACONING.md)).  Empirical robustness, not formal verification: "
             "the formal results hold under their assumptions and are not re-proved here.", "",
             f"Runs: {len(rows)} ({len(base)} B0, {len(rows) - len(base)} B1), {n_inv} invalid (infrastructure, excluded "
             "from every denominator).  Classes: PASS; safe degraded (no safety violation, a conservative reaction to the "
             "sensors: false vacancy, or an outcome missing while healthy drones entered FAILSAFE on a sensor fault); "
             "liveness failure (safe, a required outcome missing); safety failure (P1 or P2 violated, a contact, or the "
             "non-cooperative vehicle closer than d_safe); invalid.", "",
             "## Provenance", "", table_provenance(runs, "official runs"), ""]
    if pre_runs:
        lines += [table_provenance(pre_runs, "pre-freeze runs (diagnostics only)"), ""]
    lines += ["## Overview (B0)", "", table_overview(base), "",
              "Assumptions: INSIDE = every anchor respected (sonar staleness <= tau_max 0.25 s, range errors within eps, "
              "navigation error within the structure tolerance); NEAR = short excursions beyond them (sonar blackouts "
              "and latency push the staleness to 0.30 s); OUTSIDE = exceeded on purpose.  P1 is the property among the "
              "drones; the clearance from the non-cooperative vehicle of `p1_vertical_escape` is reported apart: "
              + ("; ".join(f"{p} {sum(bool(r['vehicle_clear']) for r in veh if r['profile'] == p and r['class'] != 'INVALID')}/"
                           f"{sum(1 for r in veh if r['profile'] == p and r['class'] != 'INVALID')}" for p in profs)) + " runs clear.", "",
              "## First failure level per property (B0)", "", table_first_failure(base), "",
              "## Class matrix (B0; seeds 0, 1, 2; P pass, D safe degraded, L liveness failure, S safety failure, I invalid)",
              "", table_matrix(base, scen, profs), "", "## Isolation of the phenomena (one factor at its HIGH level, pre-freeze runs)", "",
              table_isolation(iso_rows, pre_srows), "", "## Sensor and behaviour metrics by noise level (B0)", "",
              table_metrics(base, sbase, profs), "",
              "## Run-to-run agreement (pre-freeze vs official baseline runs, same profile, scenario and seed)", "",
              "HoloOcean is not bit-reproducible (sensor noise of the engine), so a configuration can take a different "
              "branch from one run to the next; the B0 code path did not change between the two campaigns.", "", agree, ""]
    if diffs:
        lines += ["Different classes: " + "; ".join(f"`{b['run']}` {a['class'].lower()} -> {b['class'].lower()}"
                                                     for a, b in diffs[:30]) + (" ..." if len(diffs) > 30 else ""), ""]
    lines += ["## Failure cases", ""]
    if fails:
        cats = Counter((f["source"], (f["diagnosis"] or {}).get("category", "?")) for f in fails)
        lines += ["Categories: A real bug, B assumption exceeded, C conservative false positive, D liveness issue, "
                  "E simulation artifact, INFRA invalid run.  " + "; ".join(f"{src} {cat}: {n}" for (src, cat), n in sorted(cats.items())),
                  "", "| run | source | class | failed requirements | category | assumptions | cause | diagnosed by |",
                  "|---|---|---|---|---|---|---|---|"]
        for f in fails:
            dg = f["diagnosis"] or {}
            lines.append(f"| `{f['run']}` | {f['source']} | {f['class']} | "
                         f"{' '.join(f['failed_requirements'] or []) or f['invalid_reason'] or '-'} | {dg.get('category', '?')} | "
                         f"{dg.get('assumptions', '-')} | {dg.get('cause', '-')} | {dg.get('source', 'manual')} |")
    else:
        lines.append("None.")
    lines += ["", "Evidence and automaton timelines of every case: [failure_cases.json](failure_cases.json); per-run "
              "metrics: [summary.csv](summary.csv), [sensor_metrics.csv](sensor_metrics.csv).", "", "## Figures", ""]
    figs = [("properties_vs_noise.png", "P1, P2, P3 vs noise"), ("classes_vs_noise.png", "failure-mode distribution"),
            ("success_rate_vs_noise.png", "success rate by family vs noise"),
            ("min_separation_vs_noise.png", "minimum separation vs noise"), ("recovery_time_vs_noise.png", "recovery time vs noise"),
            ("gate_vs_noise.png", "gate completion time vs noise"), ("false_missing_vs_noise.png", "false missing vs noise"),
            ("false_vacancy_vs_noise.png", "false vacancy vs noise")]
    lines += [f"![{lab}](../../figures/robustness/{fn})\n" for fn, lab in figs]
    (OUT / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("  ", (OUT / "SUMMARY.md").relative_to(ROOT))

    main_rows = [r for r in base if r["phase"] in ("main", "high")]
    F = FIG
    fig_properties(main_rows, F / "properties_vs_noise.png")
    fig_success_by_family(main_rows, F / "success_rate_vs_noise.png")
    fig_classes(main_rows, F / "classes_vs_noise.png")
    fig_metric(main_rows, "min_sep_m", "min true separation [m]", "Minimum separation vs noise (B0)",
               F / "min_separation_vs_noise.png", hlines=((1.0, "d_safe 1.0 m"), (2.4, "warning 2.4 m")))
    fig_metric(main_rows, "recovery_mean_s", "P3 recovery time [s]", "Recovery time vs noise (B0)", F / "recovery_time_vs_noise.png")
    fig_metric(main_rows, "completion_time_s", "gate completion time [s]", "Gate completion vs noise (B0)",
               F / "gate_vs_noise.png", only=lambda r: r["gate"])
    fig_metric(main_rows, "false_missing_episodes", "false missing episodes per run", "False missing vs noise (B0)",
               F / "false_missing_vs_noise.png")
    fig_metric(main_rows, "false_vacancy", "false vacancies per run", "False vacancy vs noise (B0)",
               F / "false_vacancy_vs_noise.png")
    beacon_section(rows, runs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
