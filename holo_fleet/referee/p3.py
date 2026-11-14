"""P3, the formation contract (criteria v8, REFEREE side: ground truth and the controllers' logs, never fed to a controller).

P3 is a conditional liveness property in three parts, each with a deadline derived from the mission geometry and the vehicle
dynamics, and judged against premises monitored on ground truth.  Tolerance E_TOL_M = 1 m and hold T_HOLD = 4 / k_slot = 8.9 s
(the 2 % settling time of the slot loop) as before.

P3a  individual slot recovery after the gate.  Healthy drone k crosses the gate at t_x (true centre beyond the exit plane
     after having been before the CR).  Within D_a(k) of t_x - or of the second crossing, for the first drone - its deviation
     from its template slot, relative to the healthy drones already across (median translation), is <= E_TOL for T_HOLD.
P3b  fleet formation after the gate.  After the last healthy crossing t_last, the formation error of the healthy drones on
     their original slots is <= E_TOL for T_HOLD within D_b of t_last (the deadline formula below, e0 the formation error at t_last).  A mission without a gate: from the end
     of the last declared disturbance, within t_rejoin + T_HOLD.
P3c  degraded formation after an admissible permanent failure (at t_f): every healthy drone that expects the lost drone as a
     neighbour declares its slot vacant within t_silent_s + 2 s of t_f (premise of a working receiver; the last survivor, which
     hears nobody, within t_rejoin of running time, the B0 timer being frozen in the mutex modes and while the plan clock holds
     at the rendezvous, as formal F6g bounds it); no healthy drone holds
     vacant the slot of a drone that is not lost; and P3b holds for the healthy drones (it starts at max(t_last, the last vacancy)).
Deadline  D = e0 / v_close + t_rejoin_margin + T_HOLD, e0 the deviation (P3a) or the formation error (P3b) at the start of the
     window, v_close = v_slot_max - v_nominal [PROT] the slowest closing speed the protocol leaves (a drone following at its speed
     cap while the reference of the others has resumed the planned speed; 0.10 m/s); every deadline is extended by the time the drone spends in a
     mode that pre-empts the formation by priority (P1: SEPARATION_WARNING, COLLISION_AVOIDANCE; FAILSAFE) and by the time
     inside declared disturbance windows.  Guaranteed (not required) bound: the ranking functions of formal/check_formation.py.
Premises, monitored per window [start, deadline] on ground truth (a verdict NOT_MET becomes PREMISE_VIOLATED only with one of
     them measured, never otherwise):
  W  current: the true current at the drone keeps the velocity of its mission reference control-feasible (control/current_envelope.py), all but
     at most 2 s in a row;
  N  navigation (a premise only beyond the validation profiles; under CLEAN, LOW and MEDIUM with the nominal channel it is a tag,
     the navigation being part of what is validated): the horizontal error of the onboard estimate is <= e_ok = 0.5 m (the controller's band inside E_TOL), all but at
     most 2 s in a row; an excess that follows a correction of the drone's own map fit within 30 s is the controller's, not the
     premise's (self-induced: counted as NOT_MET);
  A  acoustics (P3c): the receiver hears another drone during the window, and no live sender is silent for t_silent_s;
  O  path: no dead hull within d_warning_exit of the drone for more than 10 s in the window (a physical obstruction);
  S  admissible failures: outside the exclusion zone (holo_fleet/scope.py), at most n - 2 for n >= 3, none while committed for n = 2.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.referee.criteria import E_TOL_M, expects, t_hold_s

V_CLOSE = lambda cfg: cfg.form.v_slot_max - cfg.form.v_nominal
PREEMPT = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT")
GAP_S = 2.0
SELF_S = 30.0


def _frame_dev(sc, P_i: np.ndarray, idx: List[int], k: Optional[int] = None) -> float:
    """Deviation of drone k (or the max over idx) from its template slot, mean translation fitted on idx, path frame."""
    tm = sc.template.offsets()
    c = P_i[idx].mean(axis=0)
    s_c, _l, _kk = sc.path.project(c[:2])
    _p, tan, nrm = sc.path.frame_at(s_c)
    X = np.stack([P_i[:, :2] @ tan, P_i[:, :2] @ nrm, P_i[:, 2]], axis=1)
    D = X - tm
    off = D[idx].mean(axis=0)
    dev = np.linalg.norm(D - off, axis=1)
    return float(dev[k]) if k is not None else float(dev[idx].max())


def _held(t, ok, i0: int, t_last_start: float, hold: float) -> Optional[float]:
    """Earliest t_r >= t[i0], t_r <= t_last_start, with ok on [t_r, t_r + hold]; None if none."""
    run = None
    for i in range(i0, len(t)):
        if ok[i]:
            run = t[i] if run is None else run
            if t[i] - run >= hold - 1e-9:
                return run if run <= t_last_start + 1e-9 else None
        else:
            if run is not None and run > t_last_start:
                return None
            run = None
    return None


def _longest(t, bad, i0, i1) -> float:
    best, st = 0.0, None
    for i in range(i0, min(i1 + 1, len(t))):
        if bad[i]:
            st = t[i] if st is None else st
            best = max(best, t[i] - st)
        else:
            st = None
    return best


def evaluate(sc, cfg, names, faults, t, P, states, modes, m, validation: bool = True) -> Dict:
    from holo_fleet.control.current_envelope import check_current
    from holo_fleet.scope import current_bound, failure_excluded

    n = len(names)
    if m < 2 or sc.template is None or n < 2 or not sc.formation_enabled:
        return {"applies": False}
    F, G = cfg.form, cfg.gate
    hold = t_hold_s(cfg)
    onset = {f["drone"]: float(f["t_on"]) for f in faults if f.get("t_off") is None and float(f["t_on"]) < 1e8}
    lost = [k for k in range(n) if names[k] in onset]
    healthy = [k for k in range(n) if k not in lost]
    t, P = np.asarray(t[:m], float), np.asarray(P[:m], float)
    windows = list(getattr(sc, "disturbance_windows", []) or [])

    def ext(k, a, b):
        """Time of drone k in pre-empting modes and inside declared disturbances between a and b."""
        i0, i1 = int(np.searchsorted(t, a)), int(np.searchsorted(t, b))
        dt = float(np.median(np.diff(t))) if m > 1 else 0.1
        pre = float(sum(1 for i in range(i0, min(i1, m)) if modes[i, k] in PREEMPT)) * dt
        dis = sum(max(0.0, min(b, w1) - max(a, w0)) for w0, w1 in windows)
        return pre + dis

    def deadline(k, start, base):
        d = base
        for _ in range(3):
            d = base + ext(k, start, start + d)
        return start + d

    nav_err = np.array([[np.inf if states[k][i].get("nav_p") is None else
                         float(np.hypot(*(np.asarray(states[k][i]["nav_p"], float)[:2] - P[i, k, :2]))) for k in range(n)]
                        for i in range(m)])
    applied = np.array([[float(((states[k][i].get("nav_aid") or {}).get("applied_m")) or 0.0) for k in range(n)] for i in range(m)])
    w_bad = np.zeros((m, n), bool)
    for i in range(0, m):
        for k in healthy:
            v = states[k][i].get("v_mis") or states[k][i].get("v_cmd")
            if v is None:
                continue
            w = sc.sim.current.drift_at(P[i, k], t[i]) if hasattr(sc.sim.current, "drift_at") else np.zeros(3)
            w_bad[i, k] = not check_current(w, v, cfg).feasible
    dead_near = np.zeros((m, n), bool)
    for j in lost:
        on = t >= onset[names[j]]
        for k in healthy:
            dead_near[:, k] |= on & (np.linalg.norm(P[:, k] - P[:, j], axis=1) < cfg.sep.d_warning_exit)

    def premises(k, a, b, acoustic=False) -> List[str]:
        i0, i1 = int(np.searchsorted(t, a)), int(np.searchsorted(t, b))
        out = []
        if _longest(t, w_bad[:, k], i0, i1) > GAP_S:
            out.append("W")
        bad = nav_err[:, k] > cfg.form.e_ok
        if _longest(t, bad, i0, i1) > GAP_S:
            first = next((i for i in range(i0, min(i1 + 1, m)) if bad[i]), None)
            if first is not None:
                jb = int(np.searchsorted(t, t[first] - SELF_S))
                if applied[first, k] - applied[jb, k] <= 1e-6:
                    out.append("N")
                else:
                    out.append("N_self")
        dt = float(np.median(np.diff(t)))
        if float(dead_near[i0:i1 + 1, k].sum()) * dt > 10.0:
            out.append("O")
        if acoustic:
            for i in range(i0, min(i1 + 1, m)):
                sil = (states[k][i].get("gate") or {}).get("silence")
                if not sil:
                    continue
                others = [j for j in range(n) if j != k]
                live_sil = [s for j, s in zip(others, sil) if j in healthy]
                deaf = not any(s < cfg.form.t_silent_s for s in sil)
                if deaf or any(s >= cfg.form.t_silent_s for s in live_sil):
                    out.append("A")
                    break
        return out

    def verdict(met, prem):
        if met:
            return "MET"
        ext_p = [p for p in prem if p != "N_self" and not (validation and p == "N")]
        return "PREMISE_VIOLATED" if ext_p else "NOT_MET"

    w_h, w_dn = current_bound(sc.sim.current)
    gates = list(sc.judged_gates)
    scope_ok = all(not any(failure_excluded(g, P[int(np.searchsorted(t, onset[names[j]])) if np.searchsorted(t, onset[names[j]]) < m else -1, j],
                                             w_h, w_dn, cfg) for g in gates) for j in lost)
    committed_fail = any(bool(states[j][min(int(np.searchsorted(t, onset[names[j]])), m - 1)].get("committed")) for j in lost)
    n_ok = (len(lost) <= max(n - 2, 0)) if n >= 3 else (len(lost) <= 1 and not committed_fail)
    out = {"applies": True, "tol_m": E_TOL_M, "hold_s": round(hold, 2), "admissible": bool(scope_ok and n_ok)}

    tx = {}
    if gates:
        g = gates[0]
        for k in healthy:
            sgf = np.array([float(g.to_gate_frame(P[i, k])[0]) for i in range(m)])
            before = np.flatnonzero(sgf < -G.cr_half_len)
            if before.size:
                after = np.flatnonzero((sgf > G.exit_s) & (np.arange(m) > before[0]))
                if after.size:
                    tx[k] = float(t[after[0]])
    p3a = {}
    if gates:
        g = gates[0]
        for k in healthy:
            if k not in tx:
                p3a[names[k]] = {"verdict": "NA", "reason": "gate not crossed"}
                continue
            crossed_sorted = sorted(tx.values())
            start = tx[k] if (len(crossed_sorted) < 2 or tx[k] > crossed_sorted[0]) else crossed_sorted[1]
            if len(crossed_sorted) < 2:
                p3a[names[k]] = {"verdict": "NA", "reason": "only drone across"}
                continue
            i0 = int(np.searchsorted(t, start))
            idx0 = [j for j in healthy if j in tx and tx[j] <= t[min(i0, m - 1)] + 1e-9]
            d_k = _frame_dev(sc, P[min(i0, m - 1)], idx0, k)
            dl = deadline(k, start, d_k / V_CLOSE(cfg) + F.t_rejoin_margin + hold)
            ok = np.zeros(m, bool)
            for i in range(i0, m):
                idx = [j for j in healthy if j in tx and tx[j] <= t[i] + 1e-9]
                ok[i] = len(idx) >= 2 and _frame_dev(sc, P[i], idx, k) <= E_TOL_M
            tr = _held(t, ok, i0, dl - hold, hold)
            late = None if tr is not None else _held(t, ok, i0, float(t[-1]), hold)
            prem = premises(k, start, dl)
            p3a[names[k]] = {"verdict": "LATE" if (tr is None and late is not None and not [x for x in prem if x != "N_self" and not (validation and x == "N")]) else verdict(tr is not None, prem), "late_recovered": late, "start": round(start, 2), "deadline": round(dl, 2),
                             "recovered": None if tr is None else round(tr, 2), "d_m": round(d_k, 2), "premises": prem,
                             "censored": tr is None and t[-1] < dl}
    out["P3a"] = p3a

    vac = {}
    for j in lost:
        tf = onset[names[j]]
        for k in healthy:
            if not expects(sc, cfg, k, j):
                continue
            slot_j = sc.plans[j].slot_index
            tv = next((float(t[i]) for i in range(m) if t[i] >= tf and slot_j in (((states[k][i].get("form") or {}).get("vacant")) or [])), None)
            dl = tf + cfg.form.t_silent_s + 2.0
            others_live = [h for h in healthy if h != k]
            if not others_live:
                dt = float(np.median(np.diff(t)))
                i0 = int(np.searchsorted(t, tf))
                run, frozen = 0.0, 0.0
                clk = sc.plans[k].clock
                for i in range(i0, m):
                    if str(modes[i, k]).startswith("MUTEX") or (clk is not None and not clk.moving(float(t[i]))):
                        frozen += dt
                    else:
                        run += dt
                    if run >= F.t_rejoin:
                        break
                dl = tf + F.t_rejoin + frozen + 2.0
            prem = premises(k, tf, dl, acoustic=bool(others_live))
            vac[f"{names[k]}<-{names[j]}"] = {"verdict": verdict(tv is not None and tv <= dl, prem), "declared": tv, "deadline": round(dl, 2),
                                              "premises": prem}
    false_vac = []
    for k in healthy:
        for i in range(m):
            v = ((states[k][i].get("form") or {}).get("vacant")) or []
            bad = [s for s in v if any(sc.plans[h].slot_index == s for h in healthy)]
            if bad:
                false_vac.append({"t": round(float(t[i]), 2), "drone": names[k], "slots": bad})
                break
    if gates and len(tx) == len(healthy) and healthy:
        start = max(tx.values())
    elif not gates:
        start = max([w1 for _w0, w1 in windows] + [0.0])
    else:
        start = None
    if lost and start is not None:
        last_vac = max([v["declared"] for v in vac.values() if v["declared"] is not None] + [start])
        start = max(start, last_vac)
    if start is None:
        out["P3b"] = {"verdict": "NA", "reason": "not every healthy drone crossed the gate"}
    elif len(healthy) < 2:
        out["P3b"] = {"verdict": "NA", "reason": "one healthy drone"}
    else:
        i0 = int(np.searchsorted(t, start))
        e0 = _frame_dev(sc, P[min(i0, m - 1)], healthy)
        base = e0 / V_CLOSE(cfg) + F.t_rejoin_margin + hold
        dls = [deadline(k, start, base) for k in healthy]
        dl = max(dls)
        i0 = int(np.searchsorted(t, start))
        ok = np.array([_frame_dev(sc, P[i], healthy) <= E_TOL_M for i in range(m)])
        tr = _held(t, ok, i0, dl - hold, hold)
        late = None if tr is not None else _held(t, ok, i0, float(t[-1]), hold)
        prem = sorted({p for k in healthy for p in premises(k, start, dl)})
        out["P3b"] = {"verdict": "LATE" if (tr is None and late is not None and not [x for x in prem if x != "N_self" and not (validation and x == "N")]) else verdict(tr is not None, prem), "late_recovered": late, "start": round(start, 2), "deadline": round(dl, 2), "e0_m": round(e0, 2),
                      "recovered": None if tr is None else round(tr, 2), "premises": prem, "censored": tr is None and t[-1] < dl}
    if lost:
        vs = [v["verdict"] for v in vac.values()]
        p3c = "MET" if (all(v == "MET" for v in vs) and not false_vac and out["P3b"]["verdict"] in ("MET", "NA")) else (
            "PREMISE_VIOLATED" if any(v == "PREMISE_VIOLATED" for v in vs) or out["P3b"]["verdict"] == "PREMISE_VIOLATED" else "NOT_MET")
        if not out["admissible"]:
            p3c = "NOT_ADMISSIBLE"
        out["P3c"] = {"verdict": p3c, "vacancy": vac, "false_vacancy": false_vac[:3]}
    return out
