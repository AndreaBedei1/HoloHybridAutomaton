"""Targeted HoloOcean validation of the gate-aware safety layer (results/gate_safety_v1): per-run summary.

    python scripts/analyze_gate_safety_validation.py <results/gate_safety_v1> [--v13 <archive v13 root>]

Reads every run directory under <root>/runs, <root>/runs_9b42cec and <root>/runs_v11 (campaign layout <protocol>/<profile>/<scenario>/s<seed>/,
scripts/run_sensor_robustness_campaign.py --phase targeted) and writes <root>/summary.json and <root>/summary.csv.  Per run:
the referee verdicts (P1 minimum true separation and collisions, P2 maximum occupancy, P3, terminal outcome and class of the
campaign metrics) and, from the logs, the quantities of the gate-safety change: CR entries without admission (a drone whose
TRUE centre is inside the critical region of G06 while its own controller has not admitted it: not committed with
cr_cleared, entry state "ADMIT"), the largest true s of a drone while it holds before the CR (clearance to the CR face),
the along-axis navigation error at the hold, the gate events (ENTRY_HOLD_TIMEOUT, P2_FILTER, P2_GUARD, JOINT_SAFETY by
class), the joint-feasibility class of the APPLIED command per step (gate_safety.joint.applied_status, else .status), the
steps whose command the P2 guard corrected, the time in HOLD and in STANDOFF.  Ground truth is used here only, by the analysis (never by a controller).  The v13 archive
is read only.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402

G = DEFAULT.gate



def _gate(gid="G06"):
    return {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}[gid]


def _truth(d: Path):
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    n = sum(1 for k in rows[0] if k.startswith("x") and k[1:].isdigit())
    t = np.array([float(r["t"]) for r in rows])
    P = np.array([[[float(r[f"x{k}"]), float(r[f"y{k}"]), float(r[f"z{k}"])] for k in range(n)] for r in rows])
    return t, P


def _states(d: Path, k: int):
    f = d / f"drone_{k}_state.jsonl.gz"
    op = gzip.open if f.exists() else open
    f = f if f.exists() else d / f"drone_{k}_state.jsonl"
    with op(f, "rt", encoding="utf-8") as fh:
        for line in fh:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def analyze(d: Path, g) -> dict:
    m = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    rc = json.loads((d / "run_config.json").read_text(encoding="utf-8"))
    out = {"run": str(d.relative_to(d.parents[3])), "protocol": m.get("protocol"), "profile": m.get("profile"),
           "scenario": m.get("scenario"), "seed": m.get("seed"), "class": m.get("class"),
           "failed": m.get("failed_requirements"), "terminal": (m.get("terminal") or {}).get("outcome"),
           "P1_min_sep_m": (m.get("P1") or {}).get("min_true_separation_m"),
           "P1_collisions": (m.get("P1") or {}).get("collisions"), "P1_intruder_min_m": (m.get("P1") or {}).get("intruder_min_m"),
           "P1_holds": (m.get("P1") or {}).get("holds"),
           "P2_max_occupancy": (m.get("P2") or {}).get("max_occupancy"), "P2_holds": (m.get("P2") or {}).get("holds"),
           "P2_completion_s": (m.get("P2") or {}).get("completion_time_s"),
           "P3_all_recovered": (m.get("P3") or {}).get("all_recovered"), "wall_s": m.get("wall_s"),
           "sha": (m.get("code_version") or {}).get("sha"), "nav_init_err": rc.get("nav_init_err") or {},
           "engine_ports": (m.get("process") or {}).get("engine_listen_ports"),
           "engine_root": ((m.get("engine") or {}) or {}).get("root") if isinstance(m.get("engine"), dict) else None}
    if not (d / "referee_timeseries.csv").exists():
        return out
    t, P = _truth(d)
    n = P.shape[1]
    ev = Counter()
    joint_ev = Counter()
    for line in open(d / "events.jsonl", encoding="utf-8"):
        try:
            e = json.loads(line)
        except ValueError:
            continue
        ty = e.get("type")
        if ty in ("ENTRY_HOLD_TIMEOUT", "P2_GUARD", "UNCONFIRMED_CORRECTIONS_UNDONE"):
            ev[ty] += 1
        elif ty == "P2_FILTER":
            ev[f"P2_FILTER_{e.get('status')}"] += 1
        elif ty == "JOINT_SAFETY":
            joint_ev[e.get("status")] += 1
    unauth = []
    hold_s_true_max, hold_err = -1e9, []
    standoff_s = hold_time_s = 0.0
    joint_steps = Counter()
    guard_steps = 0
    for k in range(n):
        prev_t = None
        for r in _states(d, k):
            tk = float(r.get("t", 0.0))
            gr = r.get("gate") or {}
            i = int(np.clip(np.searchsorted(t, tk), 0, len(t) - 1))
            q = g.to_gate_frame(P[i, k])
            inside = bool(G.in_cr(*q))
            admitted = bool(r.get("committed")) and gr.get("entry") == "ADMIT"
            if inside and not admitted:
                unauth.append((k, round(tk, 1)))
            if gr.get("entry") in ("HOLD", "STANDOFF") and gr.get("id") == g.gate_id:
                hold_s_true_max = max(hold_s_true_max, float(q[0]))
                if r.get("nav_p") is not None:
                    e = g.vec_to_gate_frame(np.asarray(r["nav_p"], float) - P[i, k])
                    hold_err.append(float(e[0]))
            if prev_t is not None and gr.get("entry") == "STANDOFF":
                standoff_s += tk - prev_t
            if prev_t is not None and gr.get("entry") == "HOLD":
                hold_time_s += tk - prev_t
            gsr = r.get("gate_safety") or {}
            jt = gsr.get("joint") or {}
            js = jt.get("applied_status") or jt.get("status")
            if js:
                joint_steps[js] += 1
            if gsr.get("guard"):
                guard_steps += 1
            prev_t = tk
    first = sorted({k for k, _ in unauth})
    out.update({"cr_entries_without_admission_steps": len(unauth),
                "cr_entries_without_admission_drones": [f"drone_{k}" for k in first],
                "cr_entries_without_admission_first_t": {f"drone_{k}": min(tt for kk, tt in unauth if kk == k) for k in first},
                "hold_true_s_max": None if hold_s_true_max < -1e8 else round(hold_s_true_max, 3),
                "hold_clearance_to_cr_m": None if hold_s_true_max < -1e8 else round(G.cr_s_min - hold_s_true_max, 3),
                "hold_nav_err_s_m": None if not hold_err else {"min": round(min(hold_err), 3), "max": round(max(hold_err), 3),
                                                               "mean": round(float(np.mean(hold_err)), 3)},
                "events": dict(ev), "joint_safety_events": dict(joint_ev), "joint_safety_steps": dict(joint_steps),
                "standoff_s": round(standoff_s, 1), "hold_s": round(hold_time_s, 1), "p2_guard_steps": guard_steps})
    return out


def v13_table(v13: Path) -> dict:
    tab = {}
    for f in sorted(v13.rglob("run_metrics.json")):
        try:
            m = json.loads(f.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        tab.setdefault(m.get("scenario"), []).append({
            "protocol": m.get("protocol"), "profile": m.get("profile"), "seed": m.get("seed"), "class": m.get("class"),
            "P1_min_sep_m": (m.get("P1") or {}).get("min_true_separation_m"),
            "P2_max_occupancy": (m.get("P2") or {}).get("max_occupancy"),
            "P2_completion_s": (m.get("P2") or {}).get("completion_time_s"),
            "terminal": (m.get("terminal") or {}).get("outcome")})
    return tab


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--v13", type=Path, default=None)
    a = ap.parse_args()
    g = _gate()
    rows = []
    for sub, version in (("runs", "v12 (corrected)"), ("runs_9b42cec", "v12 (corrected, 9b42cec queue line)"),
                         ("runs_v11", "v11 (reference)")):
        base = a.root / sub
        if not base.exists():
            continue
        for f in sorted(base.glob("*/*/*/s*/run_metrics.json")):
            r = analyze(f.parent, g)
            r["controller"] = version
            rows.append(r)
    v13 = v13_table(a.v13) if a.v13 else {}
    for r in rows:
        r["v13"] = v13.get(r["scenario"], [])
    (a.root / "summary.json").write_text(json.dumps({"runs": rows}, indent=1, default=str), encoding="utf-8")
    cols = ["controller", "scenario", "protocol", "profile", "seed", "class", "terminal", "failed", "P1_min_sep_m",
            "P1_collisions", "P1_intruder_min_m", "P2_max_occupancy", "P2_completion_s", "P3_all_recovered",
            "cr_entries_without_admission_steps", "cr_entries_without_admission_drones", "hold_clearance_to_cr_m",
            "hold_nav_err_s_m", "events", "joint_safety_events", "joint_safety_steps", "p2_guard_steps", "hold_s", "standoff_s",
            "wall_s", "sha"]
    with open(a.root / "summary.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for r in rows:
            w.writerow([json.dumps(r.get(c), default=str) if isinstance(r.get(c), (dict, list)) else r.get(c) for c in cols])
    print(f"{len(rows)} runs -> {a.root / 'summary.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
