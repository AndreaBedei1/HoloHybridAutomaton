"""Event- and ordering-based audit of premise C of P2 and of every physical entry into the critical region (CR).

Premise C (docs/GUARANTEES.md, check_fleet F1d): a drone at the gate learns of another drone's commitment (its claim, or
the corridor echo) before it commits itself; two drones that commit together learn of each other's claim before either is
admitted to the CR.  The 32 s of S2 is the horizon of the bounded check S2h (how long a crossing drone may stop before the
CR in the model), not a delivery deadline.  The quantity that does bound the use of a report is the freshness cutoff ``status_fresh_s`` (1.95 s
for st_gate_pair, 2.50 s for the other plans: GatePerception), the age beyond which a report of NOT ENGAGED does not
exclude a drone; a claim is never dropped by age.  No time threshold enters the classes below: only the order of events.

Events read from the logs (onboard state records, referee ground truth, the TDMA schedule of sim/beacon.py):
  * the commit edge of every drone k (``committed`` rises) and the end of its claim (``committed`` falls);
  * k's first transmission of the claim: the first TDMA slot of k (period, slot k) at or after its commit edge at which its
    logged own phase ``tx`` is COMMITTED or ADMITTED; the earliest instant another drone can hold that claim is that
    emission plus the channel latency of the run (none for B0 or when the latency exceeds the acceptance limit
    ``latency_max_s``: the claim is then never delivered);
  * the blocking evidence of another drone j: its latch not FREE (``occ`` BUSY or EXITING) or its stage-2 check closed
    (``entry_ok`` false);
  * j's own commit edge, admission (``entry`` ADMIT) and the arrival at its queue point (``at_queue`` rises);
  * the true position of every drone in the CR box of the gate (referee geometry).

For every claim of k and every other live drone j at that gate (queued at the commit edge, arriving at its queue during the
claim, or committing during it), the pair is:
  SATISFIED     j did not commit while k claimed, or committed only after k's claim ended; or it showed blocking evidence
                before committing and kept it (no commit after a release while k still claimed);
  CONCURRENT    j committed before k's claim could have reached it (j's commit edge precedes the earliest delivery of k's
                first claim emission), and both showed blocking evidence of each other before either was admitted;
  VIOLATED      j committed while k claimed after the claim could have reached it and with no blocking evidence (premise C
                broken: lost packets, an undeliverable claim, a sonar miss), or after a release of its blocking while k still
                claimed, or a concurrent pair in which one drone was admitted before both had evidence;
  INCONCLUSIVE  the logs end, or k gives up, before the order is decided; j dead; B0 runs where a commit without evidence
                cannot be ordered against a claim (B0 has no claim: the sonar corridor premise M3 applies instead).
The physical entries list every entry of a drone's true position into the CR box: ADMITTED, PUSHED (committed, not
admitted), UNCOMMITTED, with the drones inside at that instant and the automaton modes of the last 10 s.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

CLAIM_PHASES = {1, 2}
CLASSES = ("SATISFIED", "CONCURRENT", "VIOLATED", "INCONCLUSIVE")


def _gate(rec) -> Dict:
    return rec.get("gate") or {}


def _blocked(rec) -> bool:
    g = _gate(rec)
    return g.get("occ") in ("BUSY", "EXITING") or g.get("entry_ok") is False


def _claims(rec) -> bool:
    tx = _gate(rec).get("tx") or [None, None]
    try:
        return int(tx[1]) in CLAIM_PHASES
    except (TypeError, ValueError):
        return False


def _episodes(states_k, m: int) -> List[tuple]:
    out, i = [], 1
    while i < m:
        if states_k[i].get("committed") and not states_k[i - 1].get("committed"):
            e = next((x for x in range(i, m) if not states_k[x].get("committed")), m)
            out.append((i, e))
            i = e
        i += 1
    return out


def first_claim_emission(t, states_k, k: int, i0: int, i1: int, period_s: float, n_drones: int) -> Optional[float]:
    """First TDMA emission of drone k at or after index i0 (before i1) that carries a claim.  A ping at tt is sent in a
    tick of the sim.step that follows the control step at T < tt (runner: controllers step at T, then the ticks run at
    T + 1/30 ... T + DT), so it carries the phase of the last record strictly before tt, not the record logged at tt."""
    slot = period_s / max(1, n_drones)
    t0 = float(t[i0])
    base = np.floor(t0 / period_s) * period_s + k * slot
    tt = base if base >= t0 - 1e-9 else base + period_s
    while tt <= float(t[min(i1, len(t) - 1)]) + 1e-9:
        j = min(int(np.searchsorted(t, tt - 1e-9)) - 1, len(states_k) - 1)
        if j >= 0 and _claims(states_k[j]):
            return float(tt)
        tt += period_s
    return None


def classify_pair(t, states, k: int, ep: tuple, j: int, m: int, deliver_s: Optional[float], t_emit: Optional[float],
                  b2: bool, period_s: float = 1.0) -> Optional[Dict]:
    """Class of the pair (claim of k over episode ep, drone j); None if j is not relevant to that claim (a drone committed
    before k is the claimant of the pair the other way round)."""
    i0, i1 = ep
    sj = states[j]
    if sj[min(i0, m - 1)].get("mode") == "DEAD":
        return None
    g0 = _gate(sj[i0])
    queued0 = bool(g0.get("at_queue")) and not sj[i0].get("committed")
    committed0 = bool(sj[i0].get("committed"))
    c_j = next((x for x in range(i0, i1) if sj[x].get("committed") and not sj[x - 1].get("committed")), None)
    if committed0 and c_j is None:
        return None
    arrived = next((x for x in range(i0 + 1, i1) if _gate(sj[x]).get("at_queue") and not _gate(sj[x - 1]).get("at_queue")), None)
    if not (queued0 or committed0 or c_j is not None or arrived is not None):
        return None
    row = {"claimant": k, "drone": j, "t_commit_claimant": round(float(t[i0]), 2),
           "t_claim_end": round(float(t[min(i1, m - 1)]), 2), "queued_at_commit": queued0,
           "arrived_during_claim": arrived is not None, "t_first_emission": None if t_emit is None else round(t_emit, 2),
           "t_earliest_delivery": None if (t_emit is None or deliver_s is None) else round(t_emit + deliver_s, 2)}
    ev = next((x for x in range(i0, i1) if _blocked(sj[x])), None)
    row["t_evidence"] = None if ev is None else round(float(t[ev]), 2)
    if c_j is None:
        released = 0
        if ev is not None:
            released = sum(1 for x in range(ev + 1, i1) if not _blocked(sj[x]) and _blocked(sj[x - 1]))
        row.update({"t_commit": None, "releases_while_claimed": released,
                    "class": "INCONCLUSIVE" if i1 >= m and ev is None and queued0 else "SATISFIED"})
        return row
    row["t_commit"] = round(float(t[c_j]), 2)
    if ev is not None and ev < c_j:
        rel = next((x for x in range(ev + 1, c_j) if not _blocked(sj[x]) and _blocked(sj[x - 1])), None)
        row["releases_while_claimed"] = int(rel is not None)
        row["class"] = "VIOLATED" if rel is not None else "SATISFIED"
        row["why"] = "commit after a release of the blocking while the claim held" if rel is not None else "blocked before commit"
        return row
    if not b2:
        row.update({"class": "INCONCLUSIVE", "why": "B0: no claim; corridor visibility is premise M3"})
        return row
    could = t_emit is not None and deliver_s is not None and float(t[c_j]) >= t_emit + deliver_s - 1e-9
    if not could:
        jc_end = next((y for y in range(c_j, m) if not sj[y].get("committed")), m)
        e_k = next((x for x in range(max(c_j, i0), i1) if _blocked(states[k][x])), None)
        e_j = next((x for x in range(max(c_j, i0), jc_end) if _blocked(sj[x])), None)
        a_k = next((x for x in range(i0, i1) if _gate(states[k][x]).get("entry") == "ADMIT"), None)
        a_j = next((x for x in range(c_j, jc_end) if _gate(sj[x]).get("entry") == "ADMIT"), None)
        known = None if (e_k is None or e_j is None) else max(e_k, e_j)
        blind = [a is not None and (known is None or a < known) for a in (a_k, a_j)]
        row.update({"t_evidence_pair": [None if e is None else round(float(t[e]), 2) for e in (e_k, e_j)],
                    "t_admitted_pair": [None if a is None else round(float(t[a]), 2) for a in (a_k, a_j)]})
        if any(blind):
            row.update({"class": "VIOLATED", "why": "concurrent commitment: admitted before both had evidence"})
        elif known is None and (i1 >= m or jc_end >= m):
            row.update({"class": "INCONCLUSIVE", "why": "concurrent commitment: logs end before both had evidence"})
        else:
            row.update({"class": "CONCURRENT", "why": "committed before the claim could be delivered" if t_emit is not None
                        and deliver_s is not None else "claim never deliverable (latency beyond acceptance)"})
            if deliver_s is None:
                row["class"] = "VIOLATED"
                row["why"] = "claim undeliverable (latency beyond the acceptance limit) and no blocking evidence"
        return row
    row.update({"class": "VIOLATED", "why": "committed after the claim could be delivered, with no blocking evidence",
                "tdma_opportunities": int(np.floor((float(t[c_j]) - t_emit - deliver_s) / period_s)) + 1})
    return row


def physical_entries(t, P, states, gate, cfg, names) -> List[Dict]:
    """Every entry of a drone's true centre into the CR box (referee geometry), with its kind and the occupancy."""
    from holo_fleet.referee.robustness_metrics import cr_entry_kind

    G = cfg.gate
    n = len(names)
    st_t = [np.array([float(r["t"]) for r in s]) for s in states]
    gf = np.array([[gate.to_gate_frame(P[i, k]) for k in range(n)] for i in range(len(t))])
    inside = (np.abs(gf[:, :, 0]) <= G.cr_half_len) & (np.abs(gf[:, :, 1]) <= G.cr_half_width) & \
             (np.abs(gf[:, :, 2]) <= G.cr_half_height)
    out = []
    for k in range(n):
        for i in np.flatnonzero(inside[1:, k] & ~inside[:-1, k]) + 1:
            j = min(int(np.searchsorted(st_t[k], t[i])), len(states[k]) - 1)
            if states[k][j].get("mode") == "DEAD":
                continue
            j0 = int(np.searchsorted(st_t[k], t[i] - 10.0))
            i_out = i
            while i_out < len(t) and inside[i_out, k]:
                i_out += 1
            out.append({"gate": gate.gate_id, "drone": names[k], "t": round(float(t[i]), 2), "kind": cr_entry_kind(states[k], j),
                        "others_inside_at_entry": [names[x] for x in range(n) if x != k and inside[i, x]],
                        "others_inside_while_in": int(inside[i:i_out].sum(axis=1).max() - 1) if i_out > i else 0,
                        "modes_10s": sorted({states[k][y].get("mode") for y in range(j0, j + 1)})})
    return out


def audit(names, t, P, states, gates, cfg, beacon: Optional[Dict] = None) -> Dict:
    """Premise C classes and physical CR entries of one run.  ``beacon``: {"period_s", "latency_s", "enabled"} of the run."""
    n, m = len(names), min(len(s) for s in states)
    tt = np.asarray(t[:m], float) if len(t) >= m else np.asarray(t, float)
    st_t = np.array([float(r["t"]) for r in states[0][:m]])
    b2 = bool(beacon and beacon.get("enabled"))
    lat = None
    if b2:
        lat = float(beacon.get("latency_s") or 0.0)
        if lat > cfg.acoustic.latency_max_s + 1e-9:
            lat = None
    pairs = []
    for k in range(n):
        for ep in _episodes(states[k], m):
            period = float((beacon or {}).get("period_s") or 1.0)
            t_emit = first_claim_emission(st_t, states[k], k, ep[0], ep[1], period, n) if b2 else None
            for j in range(n):
                if j == k:
                    continue
                r = classify_pair(st_t, states, k, ep, j, m, lat, t_emit, b2, period)
                if r is not None:
                    r["claimant"], r["drone"] = names[k], names[j]
                    pairs.append(r)
    entries = []
    for g in gates:
        entries += physical_entries(np.asarray(t, float), P, states, g, cfg, names)
    counts = {c: sum(1 for p in pairs if p["class"] == c) for c in CLASSES}
    return {"pairs": pairs, "counts": counts, "entries": entries,
            "entry_counts": {k: sum(1 for e in entries if e["kind"] == k) for k in ("ADMITTED", "PUSHED", "UNCOMMITTED")},
            "entries_with_another_inside": sum(1 for e in entries if e["others_inside_at_entry"] or e["others_inside_while_in"] > 0),
            "claim_delivery_s": lat, "b2": b2}


def beacon_of_run(rc: Dict) -> Dict:
    """Beacon facts of a run from its run_config: enabled, TDMA period, the channel's processing latency."""
    from holo_fleet.sim.acoustic_channel import channel

    b = ((rc.get("fleet_config") or {}).get("beacon") or {})
    enabled = b.get("mode") == "ACOUSTIC_RB"
    lat = None
    if enabled:
        try:
            lat = float(channel(b.get("channel") or "CLEAN").latency_s)
        except KeyError:
            lat = None
    return {"enabled": enabled, "period_s": float(b.get("period_s") or 1.0), "latency_s": lat}
