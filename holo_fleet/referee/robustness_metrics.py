"""Offline metrics and verdict of one robustness-campaign run (referee side: ground truth allowed).

``analyze_run(run_dir, reference)`` reads a run directory written by holo_fleet.runner (controller logs = what
the drones knew, referee time series = ground truth) and returns the P1 / P2 / P3 / automaton / sensor metrics
and the run class:

* INVALID           the run did not complete (simulator or infrastructure problem);
* SAFETY_FAILURE    P1 or P2 violated, a contact, or a scripted vehicle closer than d_safe;
* LIVENESS_FAILURE  safe, but a required liveness outcome is missing and no conservative declaration explains it
                    (no progress, no recovery, a gate not completed while the drones stay in their nominal modes);
* SAFE_DEGRADED     safe, but the fleet degraded conservatively: a slot falsely declared vacant (false DEGRADED),
                    or a required outcome missing while healthy drones entered FAILSAFE on a sensor fault;
* PASS              every property the scenario's reference run achieves holds again.

The required outcomes are those of the scenario's committed CLEAN reference run (results/v2/demos/<scenario>):
P3 recovered / P3-deg holds when the reference achieves it, every healthy drone through the gate, healthy drones
ending within ``PROGRESS_TOL_M`` of the reference end positions and not in FAILSAFE.
"""

from __future__ import annotations

import csv
import gzip
import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT

PROGRESS_TOL_M = 3.0      # end position vs the reference run: 10 s of survey travel at 0.3 m/s
DEADLOCK_S = 20.0         # queued drones with an empty CR and nobody passing for this long = no progress
CHATTER_S = 1.0           # A -> B -> A within this = one chattering event
CLASSES = ("PASS", "SAFE_DEGRADED", "LIVENESS_FAILURE", "SAFETY_FAILURE", "INVALID")
AVOID = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE")
FS = "FAILSAFE_HOLD_OR_RETREAT"
KEEPING = ("FORMATION_FOLLOW", "FORMATION_WAIT_REJOIN", "DEGRADED_FORMATION")


def _open(path: Path):
    if path.exists():
        return open(path, encoding="utf-8")
    gz = path.with_name(path.name + ".gz")
    return gzip.open(gz, "rt", encoding="utf-8")


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load_reference(demo_dir: Path) -> Optional[Dict]:
    """Outcome flags and end positions of a committed CLEAN reference run."""
    try:
        met = json.loads((demo_dir / "referee_metrics.json").read_text(encoding="utf-8"))
        rows = list(csv.DictReader(open(demo_dir / "referee_timeseries.csv", encoding="utf-8")))
    except OSError:
        return None
    n = sum(1 for k in rows[0] if k.startswith("x") and k[1:].isdigit())
    last = rows[-1]
    deg = met.get("P3_degraded") or {}
    return {"P3_recovered": bool(met["P3_formation_recovery"]["enabled"] and met["P3_formation_recovery"]["all_recovered_within_run"]),
            "P3_degraded_holds": deg.get("holds") is True,
            "end": [[_f(last[f"x{k}"]), _f(last[f"y{k}"]), _f(last[f"z{k}"])] for k in range(n)]}


def _intervals(mask: np.ndarray, t: np.ndarray) -> List[tuple]:
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((float(t[start]), float(t[i])))
            start = None
    if start is not None:
        out.append((float(t[start]), float(t[-1])))
    return out


def analyze_run(run_dir: Path, reference: Optional[Dict] = None, cfg=DEFAULT) -> Dict:
    from holo_fleet.referee.referee import Referee
    from holo_fleet.sim.scenarios import SCENARIOS

    run_dir = Path(run_dir)
    out: Dict = {"run_dir": str(run_dir)}
    try:
        status = json.loads((run_dir / "run_status.json").read_text(encoding="utf-8"))
    except OSError:
        status = {"status": "MISSING"}
    if status.get("status") != "COMPLETE" or not (run_dir / "referee_metrics.json").exists():
        out.update({"class": "INVALID", "invalid_reason": status.get("reason") or status.get("status")})
        return out
    rc = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    met = json.loads((run_dir / "referee_metrics.json").read_text(encoding="utf-8"))
    name, n = rc["scenario"], rc["n_drones"]
    names = [f"drone_{k}" for k in range(n)]
    rows = list(csv.DictReader(open(run_dir / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([_f(r["t"]) for r in rows])
    P = np.stack([np.array([[_f(r[f"x{k}"]), _f(r[f"y{k}"]), _f(r[f"z{k}"])] for r in rows]) for k in range(n)], axis=1)
    states = []
    for k in range(n):
        with _open(run_dir / f"drone_{k}_state.jsonl") as f:
            states.append([json.loads(line) for line in f])
    m = min(len(t), min(len(s) for s in states))
    t, P = t[:m], P[:m]
    modes = np.array([[states[k][i]["mode"] for k in range(n)] for i in range(m)])
    events = [json.loads(line) for line in open(run_dir / "events.jsonl", encoding="utf-8")]
    dt = float(np.median(np.diff(t))) if m > 1 else 0.1
    faults = rc.get("faults") or []

    def faulted(k: int, tt: float) -> bool:
        return any(fl["drone"] == names[k] and fl["t_on"] <= tt and (fl.get("t_off") is None or tt < fl["t_off"])
                   for fl in faults)

    permanently_faulted = {fl["drone"] for fl in faults if fl.get("t_off") is None}
    healthy = [k for k in range(n) if names[k] not in permanently_faulted]
    sc = SCENARIOS[name](cfg)

    # ------------------------------------------------------------------ P1
    p1 = met["P1_separation"]
    intr = p1.get("intruder") or {}
    contacts = p1["physical_contacts"] + len(p1["collision_sensor_edges"]) + int(intr.get("contacts") or 0)
    tr = [e for e in events if e.get("type") == "transition"]
    out["P1"] = {"holds": bool(p1["holds"]), "min_true_separation_m": p1["min_distance"],
                 "intruder_min_m": intr.get("min_distance"), "collisions": contacts,
                 "warning_entries": sum(1 for e in tr if e["to"] == "SEPARATION_WARNING"),
                 "ca_entries": sum(1 for e in tr if e["to"] == "COLLISION_AVOIDANCE"),
                 "time_warning_s": round(float((modes == "SEPARATION_WARNING").sum()) * dt, 1),
                 "time_ca_s": round(float((modes == "COLLISION_AVOIDANCE").sum()) * dt, 1)}

    # ------------------------------------------------------------------ P2
    p2 = met["P2_mutual_exclusion"]
    p2o: Dict = {"holds": bool(p2["holds"]), "gates": p2["gates"]}
    if sc.judged_gates:
        G = cfg.gate
        g = sc.judged_gates[0]
        gf = np.array([[g.to_gate_frame(P[i, k]) for k in range(n)] for i in range(m)])      # (m, n, 3)
        inside = (np.abs(gf[:, :, 0]) <= G.cr_half_len) & (np.abs(gf[:, :, 1]) <= G.cr_half_width) & \
                 (np.abs(gf[:, :, 2]) <= G.cr_half_height)
        passed_t = {}
        for k in healthy:
            idx = np.flatnonzero(gf[:, k, 0] > G.exit_s)
            # passed = beyond the exit after having been before the gate
            before = np.flatnonzero(gf[:, k, 0] < -G.cr_half_len)
            if idx.size and before.size and idx[idx > before[0]].size:
                passed_t[names[k]] = float(t[idx[idx > before[0]][0]])
        yielding = (modes == "MUTEX_YIELD").any(axis=1)
        passing = (modes == "MUTEX_PASS").any(axis=1)
        idle = yielding & ~inside.any(axis=1) & ~passing
        stalls = [(a, b) for a, b in _intervals(idle, t) if b - a >= DEADLOCK_S]
        wait = [float((modes[:, k] == "MUTEX_YIELD").sum()) * dt for k in healthy]
        false_busy = false_free = 0
        for k in healthy:
            others = [j for j in range(n) if j != k]
            for i in range(m):
                gs = states[k][i].get("gate") or {}
                if gs.get("id") is None or not gs.get("at_queue"):
                    continue
                occ = gs.get("occ")
                near = any(np.linalg.norm(P[i, j] - g.center) <= 4.0 for j in others)
                in_cr = any(inside[i, j] for j in others)
                false_busy += int(occ not in (None, "FREE") and not near)
                false_free += int(occ == "FREE" and in_cr)
        p2o.update({"max_occupancy": max(p2["max_occupancy"].values(), default=0), "entry_order": p2["entry_order"],
                    "passed": sorted(passed_t), "completed": all(names[k] in passed_t for k in healthy),
                    "completion_time_s": round(max(passed_t.values()), 1) if passed_t and len(passed_t) == len(healthy) else None,
                    "queue_wait_mean_s": round(float(np.mean(wait)), 1) if wait else None,
                    "queue_wait_max_s": round(float(np.max(wait)), 1) if wait else None,
                    "time_without_leader_s": round(float(idle.sum()) * dt, 1),
                    "deadlock_events": len(stalls), "longest_stall_s": round(max((b - a for a, b in _intervals(idle, t)), default=0.0), 1),
                    "false_busy_s": round(false_busy * dt, 1), "false_free_s": round(false_free * dt, 1)})
    out["P2"] = p2o

    # ------------------------------------------------------------------ P3 and missing neighbours
    p3 = met["P3_formation_recovery"]
    deg = met.get("P3_degraded") or {}
    rec = [e["recovery_time_s"] for e in p3["episodes"] if e["recovery_time_s"] is not None]
    fe = np.array([_f(r.get("form_err")) for r in rows[:m]]) if p3["enabled"] else np.array([])
    p3o: Dict = {"enabled": p3["enabled"], "episodes": p3["n_episodes"], "all_recovered": p3["all_recovered_within_run"],
                 "recovery_time_max_s": max(rec) if rec else None, "recovery_time_mean_s": round(float(np.mean(rec)), 1) if rec else None,
                 "max_formation_error_m": round(float(np.nanmax(fe)), 3) if fe.size else None,
                 "final_formation_error_m": p3.get("final_form_err"),
                 "degraded_exercised": deg.get("exercised"), "degraded_holds": deg.get("holds"),
                 "absent_at_end": deg.get("absent_at_end")}
    slot_drone = {pl["slot"]: names.index(pl["drone"]) for pl in rc["plans"]}
    fm = {"false_missing_steps": 0, "false_missing_episodes": 0, "missing_episodes": 0, "false_vacancy": 0,
          "vacancy": 0, "false_rejoin": 0, "reinclusions": 0, "true_missing_delay_s": []}
    if sc.template is not None and p3["enabled"] and n >= 3:
        ref = Referee(names, sc.template.offsets(), sc.path, sc.judged_gates, cfg, sc.formation_enabled,
                      clock=sc.plans[0].clock if sc.formation_enabled else None)
        dev = np.array([ref.deviations(P[i], list(range(n))) for i in range(m)])        # (m, n)
        e_lost, rng_lim = cfg.form.e_lost, cfg.form.neighbour_range_m + 1.0

        def present(i: int, obs: int, j: int) -> bool:
            return dev[i, j] <= e_lost and np.linalg.norm(P[i, obs] - P[i, j]) <= rng_lim and not faulted(j, t[i])

        for k in healthy:
            prev: set = set()
            for i in range(m):
                # judged only while the observer keeps its slot (the perception's missing timers run only then)
                if modes[i, k] not in KEEPING:
                    prev = set()
                    continue
                miss = set(states[k][i].get("form", {}).get("missing") or [])
                for sl in miss:
                    j = slot_drone.get(sl)
                    if j is None:
                        continue
                    fm["false_missing_steps"] += int(present(i, k, j))
                    if sl not in prev:
                        fm["missing_episodes"] += 1
                        fm["false_missing_episodes"] += int(present(i, k, j))
                for sl in prev - miss:
                    j = slot_drone.get(sl)
                    vac = sl in (states[k][i].get("form", {}).get("vacant") or [])
                    if j is not None and not vac and dev[i, j] > e_lost:
                        fm["false_rejoin"] += 1
                prev = miss
        ti = {round(float(x), 2): i for i, x in enumerate(t)}
        for e in events:
            if e.get("type") not in ("SLOT_DECLARED_VACANT", "SLOT_REOCCUPIED"):
                continue
            i = ti.get(round(float(e["t"]), 2), int(np.searchsorted(t, float(e["t"])).clip(0, m - 1)))
            j = slot_drone.get(e["slot"])
            if j is None:
                continue
            if e["type"] == "SLOT_DECLARED_VACANT":
                fm["vacancy"] += 1
                fm["false_vacancy"] += int(dev[i, j] <= e_lost and not faulted(j, t[i]))
            else:
                fm["reinclusions"] += 1
                fm["false_rejoin"] += int(dev[i, j] > e_lost)
        for fl in faults:
            j = names.index(fl["drone"])
            sl = next((s for s, d in slot_drone.items() if d == j), None)
            dep = np.flatnonzero((t >= fl["t_on"]) & (dev[:, j] > e_lost))
            if sl is None or not dep.size:
                continue
            t_dep = float(t[dep[0]])
            for k in healthy:
                if k == j:
                    continue
                hit = [float(t[i]) for i in range(m) if t[i] >= fl["t_on"] and sl in (states[k][i].get("form", {}).get("missing") or [])]
                fm["true_missing_delay_s"].append(round(hit[0] - t_dep, 1) if hit else None)
    p3o.update(fm)
    p3o["wait_rejoin_s_mean"] = round(float(np.mean([(modes[:, k] == "FORMATION_WAIT_REJOIN").sum() * dt for k in healthy])), 1)
    p3o["degraded_entries"] = sum(1 for e in tr if e["to"] == "DEGRADED_FORMATION")
    out["P3"] = p3o

    # ------------------------------------------------------------------ automaton
    run = met["run"]
    per_drone_tr = {nm: [e for e in tr if e.get("drone") == nm] for nm in names}
    chatter = 0
    for nm, lst in per_drone_tr.items():
        for a, b in zip(lst, lst[1:]):
            if b["to"] == a["from"] and b["from"] == a["to"] and b["t"] - a["t"] <= CHATTER_S + 1e-9:
                chatter += 1
    fs_tr = [e for e in tr if e["to"] == FS]
    fs_sensor = [e for e in fs_tr if "sensor_fault" in (e.get("reason") or "") and names.index(e["drone"]) in healthy]
    minutes = max(t[-1], 1e-6) / 60.0 if m else 1.0
    out["automaton"] = {"determinism_violations": sum((run.get("determinism_violations") or {}).values()),
                        "observation_consistency_violations": sum((run.get("observation_consistency_violations") or {}).values()),
                        "transitions_per_drone_min": round(len(tr) / max(1, n) / minutes, 2),
                        "chattering_events": chatter, "failsafe_entries": len(fs_tr),
                        "failsafe_entries_sensor_fault_healthy": len(fs_sensor),
                        "failsafe_time_healthy_s": round(float(sum((modes[:, k] == FS).sum() for k in healthy)) * dt, 1),
                        "end_modes": [str(modes[-1, k]) for k in range(n)] if m else []}
    out["sensor"] = met.get("sensor_audit")
    out["noise_wrapper_stats"] = run.get("noise_wrapper_stats")
    out["beacon"] = run.get("beacon")

    # ------------------------------------------------------------------ verdict
    req: Dict[str, bool] = {}
    if reference is not None:
        if reference["P3_recovered"]:
            req["P3_recovered"] = bool(p3["all_recovered_within_run"])
        if reference["P3_degraded_holds"]:
            req["P3_degraded_holds"] = deg.get("holds") is True
        if len(reference["end"]) == n and m:
            dist_end = [float(np.linalg.norm(P[-1, k] - np.asarray(reference["end"][k]))) for k in healthy]
            out["end_position_error_m"] = [round(d, 2) for d in dist_end]
            req["progress"] = max(dist_end, default=0.0) <= PROGRESS_TOL_M
    if sc.judged_gates:
        req["gate_completed"] = bool(p2o.get("completed"))
    req["no_failsafe_at_end"] = not any(modes[-1, k] == FS for k in healthy) if m else True
    out["requirements"] = req
    safe = out["P1"]["holds"] and contacts == 0 and p2o["holds"] and not (intr.get("below_d_safe") or False)
    live = all(req.values())
    conservative = fm["false_vacancy"] > 0 or len(fs_sensor) > 0
    if not safe:
        cls = "SAFETY_FAILURE"
    elif fm["false_vacancy"] > 0 or (not live and conservative):
        cls = "SAFE_DEGRADED"
    elif not live:
        cls = "LIVENESS_FAILURE"
    else:
        cls = "PASS"
    out["class"] = cls
    out["conservative_events"] = {"false_vacancy": fm["false_vacancy"], "failsafe_sensor_fault_entries": len(fs_sensor)}
    out["failed_requirements"] = [k for k, v in req.items() if not v]
    return out
