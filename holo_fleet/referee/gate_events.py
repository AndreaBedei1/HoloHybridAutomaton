"""Gate events E1-E8 and the gate liveness monitor (REFEREE side, criteria v3; offline, from the logs of a run).

P2 itself is unchanged: G(sum_i inside_CR_i <= 1) on ground truth (E5).  The events separate what the drones
believed, what they decided and what physically happened, in the terms of hazard analysis (STPA: Leveson, Engineering
a Safer World, MIT Press 2012, doi:10.7551/mitpress/8179.001.0001): a wrong belief is a flaw of the controller's
process model; it is hazardous only through an UNSAFE CONTROL ACTION, here an authorisation to enter an occupied CR.

Timing: the record of step i is the drone's decision at its time t_i, made before the simulator step; the referee
row i is the truth after that step.  A decision is therefore judged against the truth at row i - 1 (the instant it
was made).

  E1  incorrect occupancy belief of a queued healthy drone: FREE while another drone is truly in the CR (false-FREE)
      or BUSY/EXITING while the CR is empty (false-BUSY), seconds and fraction of the queued time;
  E2  premature exit evidence: BUSY -> EXITING/FREE while another drone is truly in the CR (by evidence kind);
      E2_free: any change to FREE while another drone is truly in the CR (the release that permits a commit);
  E3  approach authorisation (commit) while another drone is truly in the CR;
  E4  entry authorisation while another drone is truly in the CR: the stage-2 admission when the drones log one
      (gate.entry == "ADMIT"), else the commit itself (without a second check the commit is the entry decision);
  E5  simultaneous occupancy (a P2 violation), samples;
  E6  contacts (from the P1 metrics, added by the caller);
  E7  detected and cancelled unsafe approach: a stage-2 hold started while the CR is truly occupied (None when the
      drones log no stage 2);
  E8  waiting and passage of the healthy drones: queue wait, passage (through the CR cross-section), bypass, entry
      gap, CR dwell, and the bounded-response checks below.

Gate liveness (bounded response G(trigger -> F<=T response); Dwyer, Avrunin & Corbett, ICSE 1999,
doi:10.1145/302405.302672), with bounds taken from the protocol (config.GateRule):
  idle      while every healthy drone that has not passed is queued, the CR is truly empty and no healthy drone is
            committed, a commit must follow within T_release = t_occ_max + t_clear, the slowest release the protocol
            admits without exit evidence;
  committed a committed healthy drone must pass within its crossing budget L_k / v_pass + t_occ_max + t_clear
            (L_k = |queue_lateral| + exit_s - queue_s), the per-drone term of the deadline (referee/criteria.py);
  A-cross   the latch model's premise (formal M3): every CR dwell <= t_occ_max (reported as a validity flag).
A violation is a liveness failure whether or not the run completes later (class STALLED).  An episode still open at
the end of the log is inconclusive (finite-trace semantics), not a violation.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np


def _q(gs: Dict) -> bool:
    return gs.get("at_queue") in (True, "True", "true")     # logged as a bool or a string (v2 bug: "False" truthy)


def _intervals(mask: np.ndarray, t: np.ndarray) -> List[tuple]:
    """Closed intervals (t_start, t_end, open_at_end) of the True runs of mask."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((float(t[start]), float(t[i]), False))
            start = None
    if start is not None:
        out.append((float(t[start]), float(t[-1]), True))
    return out


def gate_events(t: np.ndarray, gf: np.ndarray, states: Sequence[Sequence[Dict]], healthy: Sequence[int], gate_id: str,
                plans: Sequence[Dict], cfg) -> Dict:
    """``gf``: (m, n, 3) true gate-frame positions; ``states``: per-drone logged records aligned with ``t``;
    ``plans``: run_config plans (queue_lateral, queue_s)."""
    G = cfg.gate
    m, n = gf.shape[0], gf.shape[1]
    healthy = list(healthy)
    dt = float(np.median(np.diff(t))) if m > 1 else 0.1
    inside = (np.abs(gf[:, :, 0]) <= G.cr_half_len) & (np.abs(gf[:, :, 1]) <= G.cr_half_width) & \
             (np.abs(gf[:, :, 2]) <= G.cr_half_height)

    def gs(k, i):
        g = states[k][i].get("gate") or {}
        return g if g.get("id") == gate_id else {}

    committed = np.array([[bool(states[k][i].get("committed")) and bool(gs(k, i)) for k in range(n)] for i in range(m)])
    # truth at the decision instant: row i - 1 for the record of step i
    in_dec = np.vstack([inside[:1], inside[:-1]]) if m else inside
    other_in = np.array([[in_dec[i, :].sum() - int(in_dec[i, k]) > 0 for k in range(n)] for i in range(m)])
    e1 = {"queued_s": 0.0, "false_free_s": 0.0, "false_busy_s": 0.0}
    e2 = {"SONAR": 0, "ACOUSTIC": 0, "TIMEOUT": 0}
    e2_free = e3 = e4 = e7 = 0
    stage2_logged = False
    queued = np.zeros((m, n), bool)
    for k in healthy:
        prev, prev_entry = None, None
        for i in range(m):
            g = gs(k, i)
            occ = g.get("occ")
            if _q(g) and not committed[i, k]:
                queued[i, k] = True
                e1["queued_s"] += dt
                e1["false_free_s"] += dt * (occ == "FREE" and other_in[i, k])
                e1["false_busy_s"] += dt * (occ not in (None, "FREE") and not in_dec[i, :].any())
            if prev == "BUSY" and occ in ("EXITING", "FREE") and other_in[i, k]:
                w = g.get("exit_witness") or {}
                e2["TIMEOUT" if occ == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")] += 1
            if prev not in (None, "FREE") and occ == "FREE" and other_in[i, k]:
                e2_free += 1
            prev = occ if g else None
            if i and committed[i, k] and not committed[i - 1, k]:
                e3 += int(other_in[i, k])
            entry = g.get("entry")
            if entry is not None:
                stage2_logged = True
                if entry == "ADMIT" and prev_entry != "ADMIT":
                    e4 += int(other_in[i, k])
                if entry == "HOLD" and prev_entry != "HOLD":
                    e7 += int(other_in[i, k])
            prev_entry = entry
    if not stage2_logged:
        e4 = e3                     # no second check: the commit is the entry decision
    occ_n = inside.sum(axis=1)
    # E8: passages THROUGH the CR cross-section, bypasses, waits, entry gaps, dwell
    passed, bypass, wait, dwell, entries = {}, [], {}, [], []
    for k in healthy:
        before = np.flatnonzero(gf[:, k, 0] < -G.cr_half_len)
        if before.size:
            beyond = np.flatnonzero((gf[:, k, 0] > G.exit_s) & (np.arange(m) > before[0]))
            if beyond.size:
                if inside[before[0]:beyond[0] + 1, k].any():
                    passed[k] = float(t[beyond[0]])
                else:
                    bypass.append(k)
        q = np.flatnonzero(queued[:, k])
        c = np.flatnonzero(committed[:, k])
        if q.size:
            wait[k] = float((t[c[c > q[0]][0]] if c[c > q[0]].size else t[-1]) - t[q[0]])
    for k in range(n):
        for a, b, _open in _intervals(inside[:, k], t):
            entries.append((a, b, k))
            dwell.append(b - a)
    entries.sort()
    gaps = [b2[0] - a1[1] for a1, b2 in zip(entries, entries[1:]) if b2[2] != a1[2]]
    # bounded-response checks (healthy drones only)
    T_release = G.t_occ_max + G.t_clear
    hc = committed[:, healthy].any(axis=1) if healthy else np.zeros(m, bool)
    not_passed = np.array([[k not in passed or t[i] < passed[k] for k in healthy] for i in range(m)]) if healthy \
        else np.zeros((m, 0), bool)
    all_waiting = np.array([bool(np.all(queued[i, healthy][not_passed[i]])) and bool(not_passed[i].any())
                            for i in range(m)]) if healthy else np.zeros(m, bool)
    idle = all_waiting & ~inside.any(axis=1) & ~hc
    idle_viol = [(a, b) for a, b, op in _intervals(idle, t) if b - a > T_release]
    plan_by_idx = {int(p["drone"].split("_")[1]): p for p in plans}
    budget, committed_viol = {}, []
    for k in healthy:
        p = plan_by_idx.get(k)
        if p is None:
            continue
        L = abs(float(p["queue_lateral"])) + G.exit_s - float(p["queue_s"])
        budget[k] = L / G.v_pass + T_release
        for a, b, _op in _intervals(committed[:, k], t):
            if b - a > budget[k] and not (k in passed and a <= passed[k] <= b + 1.0):
                committed_viol.append((k, a, b))
    w = list(wait.values())
    return {"E1": {**{key: round(v, 1) for key, v in e1.items()},
                   "false_free_frac": round(e1["false_free_s"] / e1["queued_s"], 4) if e1["queued_s"] else None,
                   "false_busy_frac": round(e1["false_busy_s"] / e1["queued_s"], 4) if e1["queued_s"] else None},
            "E2": {**e2, "total": sum(e2.values()), "to_free": e2_free}, "E3": e3, "E4": e4,
            "E4_source": "stage2" if stage2_logged else "commit",
            "E5": int((occ_n > 1).sum()), "E7": e7 if stage2_logged else None,
            "E8": {"passed": sorted(passed), "all_passed": all(k in passed for k in healthy), "bypass": bypass,
                   "wait_max_s": round(max(w), 1) if w else None, "wait_mean_s": round(float(np.mean(w)), 1) if w else None,
                   "entry_gap_min_s": round(min(gaps), 1) if gaps else None,
                   "dwell_max_s": round(max(dwell), 1) if dwell else None,
                   "a_cross_ok": all(d <= G.t_occ_max for d in dwell),
                   "idle_stalls": [(round(a, 1), round(b, 1)) for a, b in idle_viol],
                   "committed_stalls": [(k, round(a, 1), round(b, 1)) for k, a, b in committed_viol],
                   "T_release_s": T_release, "crossing_budget_s": {k: round(v, 1) for k, v in budget.items()}},
            "stalled": bool(idle_viol or committed_viol)}
