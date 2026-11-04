"""Offline metrics and verdict of one robustness-campaign run (referee side: ground truth allowed).

``analyze_run(run_dir, reference)`` reads a run directory written by holo_fleet.runner (controller logs = what
the drones knew, referee time series = ground truth) and returns the P1 / P2 / P3 / automaton / sensor metrics
and the run class of criteria v3 (referee/criteria.py):

* INVALID         the run did not complete for a reason outside the system under test (simulator, infrastructure);
* SUT_ERROR       the run stopped on an exception raised in the drones' own code (control, perception, automaton):
                  a failure of the system under test, never retried as INVALID;
* SAFETY_FAILURE  P1 or P2 violated, a contact, or a scripted vehicle closer than d_safe;
* UNSAFE_ACTION   no violation, but an entry into the CR was authorised while another drone was truly in it (E4,
                  referee/gate_events.py: an unsafe control action in the STPA sense);
* SPEC_VIOLATION  a determinism or observation-consistency violation of the automaton at run time;
* STALLED         a bounded-response requirement of the gate was violated (gate liveness), whether or not the run
                  completed later;
* PASS            every v3 requirement reached by the deadline (replay of the TerminalMonitor on the logs);
* CENSORED        historical runs only: the log ends (criteria v2 stop) before the v3 deadline without an outcome;
* SAFE_DEGRADED   requirements missed, and the miss is CAUSED by healthy drones that hold in FAILSAFE at the end on a
                  sensor fault of their injected noise profile: every missed requirement involves only those drones
                  (v2 used ANY sensor-fault FAILSAFE entry at any time, which absorbed nearly every liveness failure);
* INCOMPLETE      requirements missed at the deadline otherwise (slow or diverging).

Every metric is judged on the replay's horizon [0, t_end] (the v3 stop), so a historical run is not judged on the
longer v2 log for some quantities and on the v3 stop for others.

The requirements are a-priori (mission plan, protocol bounds, profile definitions) and the same for every protocol;
the deadline's FAILSAFE term depends on the sensor profile (criteria.failsafe_allowance_s).  v2 took the requirements
from the scenario's committed CLEAN reference run.
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
from holo_fleet.referee.criteria import (CRITERIA_VERSION, E_TOL_M, deadline_terms, deadline_v3, profile_family,
                                         t_hold_s)
from holo_fleet.referee.gate_events import gate_events

DEADLOCK_S = 20.0         # v2 report only (deadlock_events); the v3 stall detector is referee/gate_events.py
CHATTER_S = 1.0           # A -> B -> A within this = one chattering event
CLASSES = ("PASS", "SAFE_DEGRADED", "INCOMPLETE", "STALLED", "SPEC_VIOLATION", "UNSAFE_ACTION", "SAFETY_FAILURE",
           "SUT_ERROR", "CENSORED", "REPLAY_MISMATCH", "INVALID")
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


def _queued(gs: Dict) -> bool:
    return gs.get("at_queue") in (True, "True", "true")     # logged as a bool or a string ("False" is truthy)


def analyze_run(run_dir: Path, reference: Optional[Dict] = None, cfg=DEFAULT, profile: Optional[str] = None) -> Dict:
    """``reference`` is accepted for compatibility and no longer used (criteria v3 has a-priori requirements)."""
    from holo_fleet.referee.referee import Referee
    from holo_fleet.sim.scenarios import SCENARIOS

    run_dir = Path(run_dir)
    out: Dict = {"run_dir": str(run_dir)}
    try:
        status = json.loads((run_dir / "run_status.json").read_text(encoding="utf-8"))
    except OSError:
        status = {"status": "MISSING"}
    if status.get("status") == "ERROR" and status.get("error_kind") == "SUT":
        out.update({"class": "SUT_ERROR", "error": status.get("error"), "traceback": status.get("traceback")})
        return out
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

    # ------------------------------------------------------------------ criteria v3: replay, then one horizon
    profile = profile or ((rc.get("terminal") or {}).get("profile")) or _profile_from_path(run_dir)
    logged = (met.get("run") or {}).get("terminal")
    term, mon, ref3 = replay_terminal(sc, cfg, names, t, P, states, modes, profile, logged=logged)
    if term.get("t_end") is not None and m:
        m = max(1, min(m, int(np.searchsorted(t, float(term["t_end"]) + 1e-6))))
        t, P, modes, rows = t[:m], P[:m], modes[:m], rows[:m]
        states = [st[:m] for st in states]
    out["horizon_s"] = round(float(t[-1]), 2) if m else None
    # ------------------------------------------------------------------ H6 premise: the sent navigation bound
    nb_exc, nb_min, nb_n = 0, None, 0
    for k in range(n):
        for i in range(m):
            b = states[k][i].get("nav_bound")
            if b is None or states[k][i].get("nav_p") is None:
                continue
            e = float(np.linalg.norm(np.asarray(states[k][i]["nav_p"], float) - P[i, k]))
            nb_n += 1
            if math.isfinite(float(b)):
                nb_exc += int(e > float(b))
                nb_min = (float(b) - e) if nb_min is None else min(nb_min, float(b) - e)
    out["nav_bound"] = {"samples": nb_n, "exceeded": nb_exc,
                        "min_margin_m": None if nb_min is None else round(nb_min, 3)}

    # ------------------------------------------------------------------ P1
    p1 = met["P1_separation"]
    intr = p1.get("intruder") or {}
    contacts = p1["physical_contacts"] + len(p1["collision_sensor_edges"]) + int(intr.get("contacts") or 0)
    tr = [e for e in events if e.get("type") == "transition" and float(e.get("t", 0.0)) <= (t[-1] if m else 0.0) + 1e-6]
    dmin_t = np.array([min(float(np.linalg.norm(P[i, a] - P[i, b])) for a in range(n) for b in range(a + 1, n))
                       for i in range(m)]) if n > 1 and m else np.array([np.inf])
    t_nom = term.get("t_nom") or 0.0
    out["P1"] = {"holds": bool(dmin_t.min() >= cfg.sep.d_safe), "min_true_separation_m": round(float(dmin_t.min()), 3),
                 "min_true_separation_t_nom_m": round(float(dmin_t[t <= t_nom].min()), 3) if (t <= t_nom).any() else None,
                 "intruder_min_m": intr.get("min_distance"), "collisions": contacts,
                 "warning_entries": sum(1 for e in tr if e["to"] == "SEPARATION_WARNING"),
                 "ca_entries": sum(1 for e in tr if e["to"] == "COLLISION_AVOIDANCE"),
                 "time_warning_s": round(float((modes == "SEPARATION_WARNING").sum()) * dt, 1),
                 "time_ca_s": round(float((modes == "COLLISION_AVOIDANCE").sum()) * dt, 1)}

    # ------------------------------------------------------------------ P2
    p2 = met["P2_mutual_exclusion"]
    p2o: Dict = {"holds": bool(p2["holds"]), "gates": p2["gates"]}     # recomputed on the horizon below
    if sc.judged_gates:
        G = cfg.gate
        g = sc.judged_gates[0]
        gf = np.array([[g.to_gate_frame(P[i, k]) for k in range(n)] for i in range(m)])      # (m, n, 3)
        inside = (np.abs(gf[:, :, 0]) <= G.cr_half_len) & (np.abs(gf[:, :, 1]) <= G.cr_half_width) & \
                 (np.abs(gf[:, :, 2]) <= G.cr_half_height)
        p2o["holds"] = bool(inside.sum(axis=1).max() <= 1)
        passed_t = {}
        for k in healthy:
            idx = np.flatnonzero(gf[:, k, 0] > G.exit_s)
            # passed = beyond the exit after having been before the gate
            before = np.flatnonzero(gf[:, k, 0] < -G.cr_half_len)
            if idx.size and before.size and idx[idx > before[0]].size:
                b = idx[idx > before[0]][0]
                if inside[before[0]:b + 1, k].any():                 # through the CR cross-section, not around it
                    passed_t[names[k]] = float(t[b])
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
                if gs.get("id") is None or not _queued(gs):
                    continue
                occ = gs.get("occ")
                near = any(np.linalg.norm(P[i, j] - g.center) <= 4.0 for j in others)
                in_cr = any(inside[i, j] for j in others)
                false_busy += int(occ not in (None, "FREE") and not near)
                false_free += int(occ == "FREE" and in_cr)
        # releases of the latch (BUSY -> EXITING on a witness, -> FREE on the t_occ_max fallback) while another drone
        # is truly in the CR: false releases, by kind of evidence (SONAR, ACOUSTIC = variant B2, TIMEOUT)
        rel = {"releases": {"SONAR": 0, "ACOUSTIC": 0, "TIMEOUT": 0}, "false": {"SONAR": 0, "ACOUSTIC": 0, "TIMEOUT": 0}}
        for k in healthy:
            others = [j for j in range(n) if j != k]
            prev = None
            for i in range(m):
                gs = states[k][i].get("gate") or {}
                occ = (gs.get("occ"), gs.get("id"))
                # same gate: BUSY -> FREE on advancing to the next gate (the observer's own passage) is no release
                if prev is not None and prev[0] == "BUSY" and prev[1] == occ[1] and occ[0] in ("EXITING", "FREE"):
                    w = gs.get("exit_witness") or {}
                    kind = "TIMEOUT" if occ[0] == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")
                    rel["releases"][kind] += 1
                    rel["false"][kind] += int(any(inside[i, j] for j in others))
                prev = occ
        p2o["latch_releases"] = rel["releases"]
        p2o["false_releases"] = rel["false"]
        p2o["false_release_total"] = sum(rel["false"].values())
        p2o.update({"max_occupancy": max(p2["max_occupancy"].values(), default=0), "entry_order": p2["entry_order"],
                    "passed": sorted(passed_t), "completed": all(names[k] in passed_t for k in healthy),
                    "completion_time_s": round(max(passed_t.values()), 1) if passed_t and len(passed_t) == len(healthy) else None,
                    "queue_wait_mean_s": round(float(np.mean(wait)), 1) if wait else None,
                    "queue_wait_max_s": round(float(np.max(wait)), 1) if wait else None,
                    "time_without_leader_s": round(float(idle.sum()) * dt, 1),
                    "deadlock_events": len(stalls), "longest_stall_s": round(max((b - a for a, b in _intervals(idle, t)), default=0.0), 1),
                    "false_busy_s": round(false_busy * dt, 1), "false_free_s": round(false_free * dt, 1)})
        p2o["events"] = gate_events(t, gf, states, healthy, g.gate_id, rc.get("plans") or [], cfg)
    out["P2"] = p2o

    # ------------------------------------------------------------------ P3 and missing neighbours
    m3 = ref3.metrics()                                   # v3 referee (one band, scripted absences) on the horizon
    p3 = m3["P3_formation_recovery"]
    deg = m3.get("P3_degraded") or {}
    rec = [e["recovery_time_s"] for e in p3["episodes"] if e["recovery_time_s"] is not None]
    fe = np.array([_f(r.get("form_err")) for r in ref3.rows[:m]]) if p3["enabled"] else np.array([])
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
        e_lost, rng_lim = E_TOL_M, cfg.form.neighbour_range_m + 1.0          # the referee's band, not the controller's

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
        if m:                                     # a slot still declared vacant at the end while its drone is there
            fm["false_vacancy_at_end"] = sum(
                1 for k in healthy for sl in ((states[k][m - 1].get("form") or {}).get("vacant") or [])
                if slot_drone.get(sl) is not None and dev[m - 1, slot_drone[sl]] <= e_lost
                and not faulted(slot_drone[sl], t[m - 1]))
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

    # ------------------------------------------------------------------ verdict (criteria v3, referee/criteria.py)
    out["terminal_logged"] = logged
    out["terminal"] = term
    ge = p2o.get("events") or {}
    safe = out["P1"]["holds"] and contacts == 0 and p2o["holds"] and not (intr.get("below_d_safe") or False)
    # unsafe control actions (STPA): an entry (E4) or an approach (E3) authorised into an occupied CR, or a release to
    # FREE while it is occupied (E2.to_free, which permits a commit)
    unsafe_action = int(ge.get("E4") or 0) + int(ge.get("E3") or 0) + int((ge.get("E2") or {}).get("to_free") or 0)
    consistent = (out["automaton"]["determinism_violations"] == 0
                  and not out["automaton"]["observation_consistency_violations"])
    out["requirements"] = {r: r not in term["missing"] for r in term["required"]}
    if sc.judged_gates:
        out["requirements"]["gate liveness"] = not ge.get("stalled")
    out["requirements"]["automaton_consistent"] = consistent
    out["failsafe_at_end"] = any(modes[-1, k] == FS for k in healthy) if m else False
    fs_last = {}
    for e in fs_tr:
        fs_last[e["drone"]] = e.get("reason") or ""
    noisy = set(rc.get("noise_drones") or []) if rc.get("noise") else set()
    fs_end = [k for k in healthy if m and modes[-1, k] == FS and "sensor_fault" in fs_last.get(names[k], "")]
    caused = (_degradation_explains(term, mon, ref3, P, fs_end, healthy, t, cfg)
              if (fs_end and set(fs_end) <= noisy) else False)
    if term.get("replay_consistent") is False:
        cls = "REPLAY_MISMATCH"                # the offline replay disagrees with the online stop: not judged
    elif not safe:
        cls = "SAFETY_FAILURE"
    elif unsafe_action:
        cls = "UNSAFE_ACTION"
    elif not consistent:
        cls = "SPEC_VIOLATION"
    elif ge.get("stalled"):
        cls = "STALLED"                        # a bounded-response requirement of the gate violated
    elif term["outcome"] == "SUCCESS":
        cls = "PASS"
    elif term["outcome"] == "CENSORED":
        cls = "CENSORED"                       # the log ends before the v3 deadline (historical runs only)
    elif caused and profile_family(profile) != "CLEAN":
        cls = "SAFE_DEGRADED"                  # causal: the drones in sensor-fault FAILSAFE explain every miss
    else:
        cls = "INCOMPLETE"                     # requirements missed at the deadline
    out["class"] = cls
    out["criteria"] = CRITERIA_VERSION
    out["conservative_events"] = {"false_vacancy": fm["false_vacancy"], "failsafe_sensor_fault_entries": len(fs_sensor),
                                  "failsafe_sensor_fault_at_end": [names[k] for k in fs_end],
                                  "failsafe_explains_missed": caused}
    out["failed_requirements"] = [k for k, v in out["requirements"].items() if not v]
    out["cause_tags"] = cause_tags(out, fm, fs_sensor, term, states, healthy, m, dt, met)
    return out


def _degradation_explains(term, mon, ref, P, fs_end, healthy, t=None, cfg=DEFAULT) -> bool:
    """Every missed requirement involves only the drones in sensor-fault FAILSAFE at the end: their progress, their
    gate passage, their waiting, the vacancy they did not declare, and a formation that is within E_TOL_M once they
    are left out."""
    fs = set(fs_end)
    others = [k for k in healthy if k not in fs]
    for r in term["missing"]:
        if r == "progress" and not all(mon.near[k] for k in others):
            return False
        if r == "gate" and not all(mon.passed[k] is not None for k in others):
            return False
        if r in ("P3 recovered", "P3-deg re-formed") and len(others) >= 2:
            # the drones that are not holding must satisfy the same P3 semantics: within E_TOL_M (inclusive) for the
            # last T_HOLD_S of the retained history, not only at the final sample
            hold = cfg.ref.t_ok_hold
            idx = range(len(P)) if t is None else [i for i in range(len(P)) if t[i] >= t[-1] - hold]
            if t is not None and t[-1] - t[0] < hold:
                return False
            if any(ref.present_error(P[i], others) > E_TOL_M for i in idx):
                return False
        if r in ("not waiting", "P3-deg vacancy", "no false vacancy"):
            return False                       # decided by the records of drones that are not holding: not explained
    return True


def _profile_from_path(run_dir: Path) -> Optional[str]:
    """Campaign layout <protocol>/<profile>/<scenario>/s<k>: the profile part, if it looks like one."""
    parts = Path(run_dir).parts
    if len(parts) >= 3 and parts[-3].split("-")[0] in ("CLEAN", "LOW", "MEDIUM", "HIGH"):
        return parts[-3]
    return None


def replay_terminal(sc, cfg, names, t, P, states, modes, profile, logged: Optional[Dict] = None):
    """Replay the v3 TerminalMonitor on the logged ground truth and records: the same requirement set as the online
    stop.  A log that ends before the deadline without an outcome (a historical run stopped by criteria v2) is
    CENSORED.  For a v3 run the logged deadline is used and the replayed outcome is compared with the logged one
    (``replay_consistent``).  Returns (summary, monitor, referee)."""
    from types import SimpleNamespace

    from holo_fleet.referee.referee import Referee
    from holo_fleet.referee.terminal import TerminalMonitor

    n, m = len(names), len(t)
    faults = getattr(sc.sim, "faults", ()) or ()
    ref = Referee(names, sc.template.offsets() if sc.template is not None else None, sc.path, sc.judged_gates, cfg,
                  sc.formation_enabled, clock=sc.plans[0].clock if sc.formation_enabled else None, faults=faults)
    v3 = bool(logged) and logged.get("criteria") in ("v3", CRITERIA_VERSION)   # same deadline and monitor
    dl = float(logged["deadline_s"]) if v3 else deadline_v3(sc, cfg, profile)
    mon = TerminalMonitor(sc, cfg, names, dl)
    z = np.zeros((n, 3))
    for i in range(m):
        truth = SimpleNamespace(t=float(t[i]), positions=P[i], collision=np.zeros(n, bool), current_drift=z,
                                velocities=z, intruders=None, rotations=None)
        ref.update(truth, list(modes[i]))
        if mon.update(float(t[i]), P[i], ref, [states[k][i] for k in range(n)]) is not None:
            break
    if mon.outcome is None:
        if m and t[-1] >= dl - 0.2:
            mon.finalize(float(t[-1]), ref, [states[k][-1] for k in range(n)])
        else:
            mon.outcome, mon.reason = "CENSORED", "log ends before the v3 deadline: " + ", ".join(mon.last_missing)
            mon.t_end = round(float(t[-1]), 2) if m else None
    s = mon.summary()
    s["required"] = [r for r in ("progress", "gate", "P3 recovered", "P3-deg re-formed", "P3-deg vacancy",
                                 "no false vacancy", "not waiting")
                     if _applies(r, mon)]
    s["deadline_terms"] = deadline_terms(sc, cfg, profile)
    if v3:
        s["replay_consistent"] = (logged.get("outcome") == s["outcome"]
                                  and abs(float(logged.get("t_end") or 0.0) - float(s["t_end"] or 0.0)) <= 0.25)
    return s, mon, ref


def _applies(req: str, mon) -> bool:
    if req == "gate":
        return mon.gate is not None
    if req in ("P3 recovered",):
        return mon.judge_formation and not mon.permanent
    if req in ("P3-deg re-formed", "P3-deg vacancy"):
        return mon.judge_formation and bool(mon.permanent)
    if req == "no false vacancy":
        return mon.judge_formation
    return True


def cause_tags(out: Dict, fm: Dict, fs_sensor, term, states, healthy, m: int, dt: float, met: Dict) -> List[str]:
    """Diagnostic mechanism tags of a run (never used by the class, fixed before the final campaign): which known
    mechanisms were active, so that a non-PASS run can be read without re-deriving its cause by hand."""
    tags = []
    if (out.get("nav_bound") or {}).get("exceeded"):
        tags.append("NAV_BOUND_PREMISE_VIOLATED")        # the H6 payload premise failed: no P2 claim rests on this run
    if not out["P1"]["holds"] or out["P1"]["collisions"]:
        tags.append("P1_violation")
    if not out["P2"]["holds"]:
        tags.append("P2_violation")
    if fm.get("false_rejoin"):
        tags.append("false_reoccupation")                 # a vacant slot re-included while its drone was away (C3)
    if fm.get("false_vacancy"):
        tags.append("false_vacancy")
    if fs_sensor:
        tags.append("sensor_fault_failsafe")
    hold = max((sum(1 for i in range(m) if ((states[k][i].get("gate") or {}).get("reloc_hold"))) * dt for k in healthy),
               default=0.0)
    if hold >= 10.0:
        tags.append("relocalization_hold")               # held before the CR without a gate localization (C5)
    if int(met.get("occupancy_timeouts") or 0) > 0:
        tags.append("occupancy_timeout_release")         # CR released by the t_occ_max fallback (assumption A-cross)
    reason = (term or {}).get("reason") or ""
    if "not waiting" in reason:
        tags.append("waiting_for_missing_neighbour")
    if "P3-deg vacancy" in reason:
        tags.append("vacancy_not_declared")
    if out.get("failed_requirements") and "gate" in out["failed_requirements"]:
        tags.append("gate_not_completed")
    if (term or {}).get("gate_bypass"):
        tags.append("GATE_BYPASS")                       # past the gate around or over the frame, not through the CR
    if (out.get("P2") or {}).get("events", {}).get("E8", {}).get("a_cross_ok") is False:
        tags.append("A_CROSS_VIOLATED")                  # a CR dwell longer than t_occ_max (latch model premise)
    return tags
