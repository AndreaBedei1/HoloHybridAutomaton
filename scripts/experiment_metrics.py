"""Per-run experiment metrics for P1 encounters and P3 recoveries (offline, ground truth allowed).

    python scripts/experiment_metrics.py results/v2/demos/p1_close_encounter

Writes <run>/experiment_summary.json.  The P1 section is computed for every run, the P3 section for runs
that judge a formation.  Onboard quantities come from drone_<k>_state.jsonl (what the controllers logged),
true ones from referee_timeseries.csv and from the scenario's current field evaluated at the true positions.
Nothing here feeds back into a controller.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.sim.currents import CurrentComponent, CurrentField  # noqa: E402

T_START = 0.3          # first two sonar captures (an echo needs 2 of 3 to be confirmed)
AVOID = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE")


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load(run: Path) -> Dict:
    cfg = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    n = cfg["n_drones"]
    rows = list(csv.DictReader(open(run / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([_f(r["t"]) for r in rows])
    P = np.stack([np.array([[_f(r[f"x{k}"]), _f(r[f"y{k}"]), _f(r[f"z{k}"])] for r in rows]) for k in range(n)], axis=1)
    states = [[json.loads(l) for l in open(run / f"drone_{k}_state.jsonl", encoding="utf-8")] for k in range(n)]
    events = [json.loads(l) for l in open(run / "events.jsonl", encoding="utf-8")]
    met = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    return {"run": run, "cfg": cfg, "n": n, "rows": rows, "t": t, "P": P, "states": states, "events": events, "met": met}


def mode_episodes(states: List[Dict]) -> List[Dict]:
    eps: List[Dict] = []
    for r in states:
        if not eps or eps[-1]["mode"] != r["mode"]:
            eps.append({"mode": r["mode"], "t0": round(r["t"], 2), "t1": round(r["t"], 2)})
        else:
            eps[-1]["t1"] = round(r["t"], 2)
    return eps


def _rate(t: np.ndarray, d: np.ndarray, half: int = 3) -> np.ndarray:
    """d d/dt by a centred difference over 2*half samples (0.6 s at 10 Hz): sample noise averaged out."""
    out = np.full_like(d, np.nan)
    for i in range(half, len(d) - half):
        out[i] = (d[i + half] - d[i - half]) / max(t[i + half] - t[i - half], 1e-6)
    return out


def _common(R) -> Dict:
    m = R["met"]
    p1 = m["P1_separation"]
    run = m["run"]
    return {"collisions": p1["physical_contacts"] + len(p1["collision_sensor_edges"]),
            "messages": run["inter_agent_messages"],
            "envelope": {**m["envelope"], "self_declared_envelope_violations": run["self_declared_envelope_violations"],
                         "self_declared_events": [{"t": round(e["t"], 2), "drone": e.get("drone"), "type": e["type"]}
                                                  for e in R["events"] if e.get("type") in ("ENVELOPE_VIOLATION", "ENVELOPE_RESTORED")]},
            "determinism_violations": sum(run["determinism_violations"].values()),
            "observation_consistency_violations": sum((run.get("observation_consistency_violations") or {}).values())
            if run.get("observation_consistency_violations") is not None else None}


def _true_drift(R) -> np.ndarray:
    """(T, n, 3) current drift at the true drone positions (the logged current field, offline)."""
    field = CurrentField([CurrentComponent(**c) for c in R["cfg"].get("current", [])])
    t, P = R["t"], R["P"]
    return np.array([[field.drift_at(P[i, k], t[i]) for k in range(R["n"])] for i in range(len(t))])


def p1_section(R) -> Dict:
    sep = DEFAULT.sep
    t, P, n = R["t"], R["P"], R["n"]
    pairs = [(a, b) for a in range(n) for b in range(a + 1, n)]
    D = {pr: np.linalg.norm(P[:, pr[0]] - P[:, pr[1]], axis=1) for pr in pairs}
    pr_min = min(pairs, key=lambda pr: np.nanmin(D[pr]))
    d = D[pr_min]
    i_min = int(np.nanargmin(d))
    rate = _rate(t, d)
    drones = {}
    sw_first = []
    for k in range(n):
        st = R["states"][k]
        eps = mode_episodes(st)
        sw = [e for e in eps if e["mode"] == "SEPARATION_WARNING"]
        ca = [e for e in eps if e["mode"] == "COLLISION_AVOIDANCE"]
        if sw or ca:
            sw_first.append(min(e["t0"] for e in sw + ca))
        d_on = [(r["d_min"], r["t"]) for r in st if r.get("d_min") is not None and r["t"] >= T_START]
        dm = min(d_on) if d_on else (None, None)
        avoid = [r for r in st if r["mode"] in AVOID]
        patterns = sorted({tg["pattern"] for r in avoid for tg in r.get("targets", []) if tg.get("r", 99) < 4.0})
        drones[f"drone_{k}"] = {
            "SW_entries": len(sw), "SW_entry_times_s": [e["t0"] for e in sw],
            "CA_entries": len(ca), "CA_entry_times_s": [e["t0"] for e in ca],
            "time_in_SW_s": round(sum(e["t1"] - e["t0"] + 0.1 for e in sw), 1),
            "time_in_CA_s": round(sum(e["t1"] - e["t0"] + 0.1 for e in ca), 1),
            "min_onboard_conservative_distance_m": dm[0], "at_s": dm[1],
            "escape_directions": sorted({r["escape"]["dir"] for r in avoid if r.get("escape")}),
            "giveway": sorted({r["giveway"] for r in st if r.get("giveway")}),
            "sonar_patterns_within_4m_in_SW_CA": patterns,
            "sonar_sectors_involved": sorted({s for p in patterns for s in p.split("+")}),
            "max_onboard_current_estimate_m_s": round(max(float(np.linalg.norm(r["current_est"][:2])) for r in st), 3),
            "mode_sequence": [e["mode"] for e in eps],
        }
    t_int = min(sw_first) if sw_first else None
    before = (t <= t_int) if t_int is not None else np.ones_like(t, dtype=bool)
    after = (t >= t[i_min]) & (t <= t[i_min] + 5.0)
    i_int = int(np.argmin(np.abs(t - t_int))) if t_int is not None else None
    drift = _true_drift(R)
    return {
        "thresholds_m": {"d_warning": sep.d_warning, "d_warning_exit": sep.d_warning_exit, "d_ca": sep.d_ca,
                         "d_ca_exit": sep.d_ca_exit, "d_safe": sep.d_safe},
        "min_true_pair_distance_m": round(float(d[i_min]), 3), "at_s": round(float(t[i_min]), 2),
        "closest_pair": [f"drone_{pr_min[0]}", f"drone_{pr_min[1]}"],
        "first_intervention_s": t_int,
        "true_distance_at_first_intervention_m": None if i_int is None else round(float(d[i_int]), 3),
        "max_closing_speed_before_intervention_m_s": round(float(np.nanmax(-rate[before])), 3),
        "closing_speed_at_first_intervention_m_s": None if i_int is None else round(float(-rate[i_int]), 3),
        "max_opening_speed_5s_after_closest_m_s": round(float(np.nanmax(rate[after])), 3),
        "max_true_current_m_s": round(float(np.max(np.linalg.norm(drift[:, :, :2], axis=2))), 3),
        "drones": drones, **_common(R),
    }


def p3_section(R) -> Dict:
    m = R["met"]
    t, P, n = R["t"], R["P"], R["n"]
    fe = np.array([_f(r["form_err"]) for r in R["rows"]])
    i_fe = int(np.nanargmax(fe))
    windows = R["cfg"].get("disturbance_windows") or []
    t_end = max((b for _a, b in windows), default=None)
    eps = []
    for e in m["P3_formation_recovery"]["episodes"]:
        eps.append({**e, "recovery_from_end_of_disturbance_s":
                    None if (e["t_recovered"] is None or t_end is None) else round(e["t_recovered"] - t_end, 2)})
    drift = _true_drift(R)
    drones = {}
    for k in range(n):
        st = R["states"][k]
        mm = min(len(st), len(t))
        nav = max(float(np.linalg.norm(np.asarray(st[i]["nav_p"]) - P[i, k])) for i in range(mm))
        ages = [v["age"] for r in st if r["t"] >= T_START for v in r["sectors"].values() if v.get("healthy")]
        sat = [bool(r.get("saturated")) for r in st]
        longest, cur = 0, 0
        for s in sat:
            cur = cur + 1 if s else 0
            longest = max(longest, cur)
        fo = [(r["form"]["form_err"], r["t"]) for r in st if r.get("form") and r["t"] >= 3.0]
        drones[f"drone_{k}"] = {
            "max_true_current_at_drone_m_s": round(float(np.max(np.linalg.norm(drift[:, k, :2], axis=1))), 3),
            "max_onboard_form_err_m": round(max(fo)[0], 3) if fo else None, "at_s": max(fo)[1] if fo else None,
            "nav_error_max_m": round(nav, 3), "sonar_age_max_s": round(max(ages), 3) if ages else None,
            "saturated_time_s": round(0.1 * sum(sat), 1), "longest_saturation_s": round(0.1 * longest, 1),
            "max_onboard_current_estimate_m_s": round(max(float(np.linalg.norm(r["current_est"][:2])) for r in st), 3),
            "mode_episodes": mode_episodes(st),
        }
    return {
        "e_lost_m": DEFAULT.ref.e_lost, "e_ok_m": DEFAULT.ref.e_ok, "t_ok_hold_s": DEFAULT.ref.t_ok_hold,
        "disturbance_windows_s": windows,
        "max_true_formation_error_m": round(float(fe[i_fe]), 3), "at_s": round(float(t[i_fe]), 2),
        "episodes": eps, "all_recovered_within_run": m["P3_formation_recovery"]["all_recovered_within_run"],
        "min_P1_distance_m": m["P1_separation"]["min_distance"], "drones": drones, **_common(R),
    }


def summarize(run: Path) -> Dict:
    R = load(run)
    out = {"scenario": R["cfg"]["scenario"], "p1": p1_section(R)}
    if R["met"]["P3_formation_recovery"]["enabled"]:
        out["p3"] = p3_section(R)
    (run / "experiment_summary.json").write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")
    return out


if __name__ == "__main__":
    for a in sys.argv[1:]:
        print(json.dumps(summarize(Path(a)), indent=1, default=float)[:4000])
