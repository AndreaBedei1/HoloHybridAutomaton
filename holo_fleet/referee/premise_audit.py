"""Offline audit of premise C of P2 and of every physical entry into the critical region (CR), from the run logs.

Premise C (check_fleet F0/F1d): a drone at a gate learns of another drone's commitment before it commits itself, and two
drones that commit before either can learn of the other resolve it at the stage-2 check: neither is admitted while the
other's claim could be known to it, and they are never both admitted before either could learn of the other.

Two properties are certified, separately, for every claim episode of a drone k and every other live drone j at the gate.

KNOWLEDGE (claim deliverability at the commit instants of j).  Receptions are not logged (no per-packet log, no payload
acceptance log), and a BUSY latch may come from a corridor echo or a third drone, so actual knowledge of k's claim is never
certified.  The audit decides from the logs and the implemented timing:
  * when k's claim could first be USED at j (``start``) and when j could first use k's report that the claim ended
    (``end``): the integer-tick TDMA schedule of sim/beacon.py (period_ticks, slot_ticks = period_ticks // n, k emits when
    (tick - k slot_ticks) % period_ticks == 0), the phase of the control record strictly before the emission tick (the
    controllers step at T, the pings go out in the ticks after T), the propagation delay of the true distance, the minimum
    processing latency of the channel (jitter only delays further), the delivery tick, the first control step at or after it,
    and the stale rule of perception/acoustic.py (age since arrival on use > latency_max_s: never used).  A BLACKOUT
    channel, a run without payload or a range beyond max_range_m delivers nothing.  This is the earliest opportunity, never a
    reception: random losses, outages and collisions are not excluded;
  * at every commit step j's latch is FREE (commit guard), and a held claim forces the latch BUSY
    (perception/gate_perception.py: a heard claim makes the CR occupied).  So a commit of j shows that j did NOT hold k's
    claim at that step.
Values: NO_COMMIT (j did not commit inside k's usable window; that j held the claim is NOT certified), BEFORE_USE (j
committed before k's claim could be used at j), NOT_HELD (j committed while k's claim could be used at j and did not hold
it: a knowledge failure), UNDELIVERABLE (k's claim was emitted but could never be used at j, or j ignores k's pinger code:
another formation, runner.beacon_identities), NOT_EMITTED (k's commitment ended, by withdrawal or death, before its first
ping: no claim was ever on the air; a commit of j during it is judged as a concurrent commitment), NOT_APPLICABLE (no claim
channel: B0), UNKNOWN (geometry or phase missing, or the logs end before k's first ping).

BLOCKING (observable behaviour of j): j does not commit while k's claim is competing.  Every commit edge of j from k's commit
to ``end`` is examined, recommitments included (a commit at the very sample its latch is released is a commit like any other).
A commit edge before ``start`` is a concurrent commitment, judged by its resolution (the admissions of both drones against
the other's usable window).  A commit edge with no usable claim (B0, or a claim never deliverable) is judged by the corridor
premise M3: explained (basis EXIT_WITNESSED) when j's latch passed through EXITING (a sonar exit witness) after k's commit and
k's true centre had entered a CR box and was outside every box at j's commit.  A pair without commit records the steps j
spent at its queue point inside the window by the logged guard inputs (latch not FREE, no priority, hazard mode;
``unexplained``: every logged input true and no commit; reported, not decisive).

Pair class (premise C; ``knowledge`` and ``blocking`` keep the two verdicts):
  SATISFIED     no commit of j inside the window, or a commit with no usable claim explained by M3 (EXIT_WITNESSED);
  CONCURRENT    j committed before k's claim could be used at j, and the pair was resolved within the logs without an
                admission made while the other's claim could be used (``resolution``: SEQUENTIAL, WITHDRAWAL, BOTH_WITHDRAWN,
                PASSED_WITHOUT_ADMISSION: a crossing without admission, also listed as a PUSHED entry);
  VIOLATED      j committed while k's claim could be used at j (knowledge NOT_HELD), or a drone of a concurrent pair was
                admitted while the other's claim could be used at it, or both drones of a concurrent pair were admitted, each
                before the other's claim could be used at it (whatever the order of the commits and admissions: one of them
                was admitted blind to the other, and the other could not hold its own entry), or (B2) the claim was emitted,
                could never be used and the commit is not explained by M3;
  INCONCLUSIVE  the logs end before the pair is decided; geometry or phase missing; a B0 commit during k's commitment not
                explained by M3 (B0 carries no claim and the timing of the corridor echo is not logged); the gate index
                carried by k's claim differs from the gate index j reports (pairs across gates are not audited).
Commits of j after ``end`` (j could know that k's claim ended) are outside the claim and form no row; two drones committing
at the same control sample form one row (claimant the lower index).
Stated limitations: the gate of a pair is identified only by the gate index modulo 4 of the payload (``tx[0]``), compared
with the index j reports at k's commit; a dead drone k ends its claim at j when j lists k's SLOT in ``gate.dead`` (slot of
k at j from ``identities``, runner.beacon_identities; without it every drone is assumed to fly slot = drone index of a
single formation).
The physical entries list every entry of a drone's true centre into the CR box: ADMITTED, PUSHED (committed, not
admitted), UNCOMMITTED, with the drones inside at that instant.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.config import COMMUNICATION_ACOUSTIC

CLAIM_PHASES = {1, 2}
CLASSES = ("SATISFIED", "CONCURRENT", "VIOLATED", "INCONCLUSIVE")
KNOWLEDGE = ("NO_COMMIT", "BEFORE_USE", "NOT_HELD", "UNDELIVERABLE", "NOT_EMITTED", "NOT_APPLICABLE", "UNKNOWN")
BLOCKING = ("SATISFIED", "CONCURRENT", "VIOLATED", "INCONCLUSIVE")
TICKS_PER_SEC = 30          # sim/holo_env.TICKS_PER_SEC (tests/test_premise_audit.py checks it)
HAZARD_MODES = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT")
UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Link:
    """Claim transport of a run: TDMA of sim/beacon.BeaconTransport, delays of sim/acoustic_channel, acceptance of
    perception/acoustic.py."""

    n: int
    period_s: float = 1.0
    latency_s: float = 0.10
    latency_max_s: float = 0.40
    max_range_m: float = 50.0
    sound_speed: float = 1500.0
    blackout: bool = False
    tps: int = TICKS_PER_SEC
    ids: Optional[Dict[int, Dict[int, int]]] = field(default=None, compare=False, hash=False)

    def slot_at(self, j: int, k: int) -> Optional[int]:
        """Slot under which receiver j knows drone k's pinger code; None: j ignores k (another formation)."""
        return k if self.ids is None else (self.ids.get(j) or {}).get(k)

    @property
    def period_ticks(self) -> int:
        return max(1, int(round(self.period_s * self.tps)))

    @property
    def slot_ticks(self) -> int:
        return max(1, self.period_ticks // max(1, self.n))

    def next_emission(self, k: int, tick: int) -> int:
        """First emission tick of drone k strictly after ``tick`` (ticks start at 1)."""
        nxt = max(tick + 1, 1)
        r = (nxt - k * self.slot_ticks) % self.period_ticks
        return nxt if r == 0 else nxt + self.period_ticks - r


def ticks(times) -> List[int]:
    return [int(round(float(x) * TICKS_PER_SEC)) for x in times]


def earliest_use(e_tick: int, dist_m, link: Link, ctrl: Sequence[int]):
    """Control tick at which another drone could first use the ping emitted at ``e_tick``; None if it can never be used
    (blackout, out of range, stale at the minimum delay); UNKNOWN without the geometry or when the logs end first."""
    if link.blackout:
        return None
    if dist_m is None or not math.isfinite(float(dist_m)):
        return UNKNOWN
    if float(dist_m) > link.max_range_m:
        return None
    t_arr = e_tick / link.tps + float(dist_m) / link.sound_speed
    d_tick = max(e_tick, int(math.ceil((t_arr + link.latency_s) * link.tps - 1e-6)))
    i = bisect_left(ctrl, d_tick)
    if i >= len(ctrl):
        return UNKNOWN
    c = int(ctrl[i])
    return None if c / link.tps - t_arr > link.latency_max_s + 1e-9 else c


def _gate(rec) -> Dict:
    return rec.get("gate") or {}


def _phase(rec) -> Optional[bool]:
    tx = _gate(rec).get("tx")
    if not tx or len(tx) < 2 or tx[1] is None:
        return None
    try:
        return int(tx[1]) in CLAIM_PHASES
    except (TypeError, ValueError):
        return None


def _episodes(states_k, m: int) -> List[tuple]:
    """(commit record, first record after the commitment) of every commitment of a drone."""
    out, i = [], 1
    while i < m:
        if states_k[i].get("committed") and not states_k[i - 1].get("committed"):
            e = next((x for x in range(i, m) if not states_k[x].get("committed")), m)
            out.append((i, e))
            i = e
        i += 1
    return out


def claim_emissions(ctrl, states_k, k: int, i0: int, link: Link, m: int):
    """Claim emission ticks of k for the commitment starting at record i0, and how the claim stops being emitted: the tick
    of the first emission without it, "DEAD" (k stops emitting), None (the logs end) or UNKNOWN (phase not logged)."""
    out, tick, last = [], int(ctrl[i0]), int(ctrl[m - 1])
    while True:
        tick = link.next_emission(k, tick)
        if tick > last:
            return out, None
        j = bisect_left(ctrl, tick) - 1
        if states_k[j].get("mode") == "DEAD":
            return out, "DEAD"
        c = _phase(states_k[j])
        if c is None:
            return out, UNKNOWN
        if c:
            out.append(tick)
        elif out or not states_k[j].get("committed"):
            return out, tick


def first_claim_emission(t, states_k, k: int, i0: int, period_s: float, n_drones: int) -> Optional[float]:
    """Time of the first TDMA emission of drone k after its commit record i0 that carries the claim (None if none)."""
    ctrl = ticks(t)
    em, _ = claim_emissions(ctrl, states_k, k, i0, Link(n=n_drones, period_s=period_s), min(len(ctrl), len(states_k)))
    return em[0] / TICKS_PER_SEC if em else None


class Geometry:
    """Ground truth: distances between drones at a tick, and whether a drone's true centre is inside a CR box."""

    def __init__(self, t_ref, P, inside=None):
        self.t = np.asarray(t_ref, float)
        self.P = None if P is None else np.asarray(P, float)
        self.inside = None if inside is None else np.asarray(inside, bool)

    def _i(self, tick: int) -> int:
        return int(np.clip(np.searchsorted(self.t, tick / TICKS_PER_SEC - 1e-9), 0, max(0, len(self.t) - 1)))

    def dist(self, a: int, b: int, tick: int):
        if self.P is None or len(self.t) == 0:
            return None
        i = self._i(tick)
        if abs(self.t[i] - tick / TICKS_PER_SEC) > 0.2:
            return None
        return float(np.linalg.norm(self.P[i, a] - self.P[i, b]))

    def passed(self, k: int, tick0: int, tick1: int) -> Optional[bool]:
        """k's true centre entered a CR box between the two ticks and is outside every box at tick1 (None: unknown)."""
        if self.inside is None or len(self.t) == 0:
            return None
        a, b = self._i(tick0), self._i(tick1)
        return bool(self.inside[a:b + 1, k].any() and not self.inside[b, k])


def use_window(ctrl, states, geo: Geometry, link: Link, k: int, ep: tuple, j: int, m: int) -> Dict:
    """Ticks at which k's claim of commitment ``ep`` could first be used at j (``start``) and at which j could first use the
    end of that claim (``end``: the first emission without it, or j declaring k dead); None: never; UNKNOWN."""
    em, stop = claim_emissions(ctrl, states[k], k, ep[0], link, m)
    gidx = None
    if em:
        gidx = (_gate(states[k][bisect_left(ctrl, em[0]) - 1]).get("tx") or [None])[0]
    info = {"emissions": len(em), "stop": stop, "gate_idx": gidx}
    if stop == UNKNOWN:
        return {"start": UNKNOWN, "end": UNKNOWN, "uses": [], **info}
    slot = link.slot_at(j, k)
    if slot is None:
        return {"start": None, "end": None, "uses": [], "ignored": True, **info}
    uses = []
    for e in em:
        u = earliest_use(e, geo.dist(k, j, e), link, ctrl)
        if u == UNKNOWN:
            if uses:
                break
            return {"start": UNKNOWN, "end": UNKNOWN, "uses": uses, **info}
        if u is not None:
            uses.append(u)
    start = uses[0] if uses else None
    end = None
    if isinstance(stop, int):
        end = earliest_use(stop, geo.dist(k, j, stop), link, ctrl)
    elif stop == "DEAD":
        d = next((x for x in range(ep[0], m) if slot in (_gate(states[j][x]).get("dead") or [])), None)
        end = None if d is None else int(ctrl[d])
    return {"start": start, "end": end, "uses": uses, **info}


def _usable(w: Dict, tick: int) -> bool:
    return w["start"] not in (None, UNKNOWN) and tick >= w["start"] and (w["end"] in (None, UNKNOWN) or tick < w["end"])


def _t(tick) -> Optional[float]:
    return None if tick is None or tick == UNKNOWN else round(tick / TICKS_PER_SEC, 3)


def _admission(states_x, a: int, b: int) -> Optional[int]:
    return next((y for y in range(a, b) if _gate(states_x[y]).get("entry") == "ADMIT"), None)


def _end_edge(states_x, e: int, m: int) -> Optional[str]:
    """Automaton edge that ended a commitment at record e (pass_done, pass_abort); None if the logs end first."""
    return None if e >= m else (states_x[e].get("transition") or {}).get("edge")


def _step_reason(rec) -> str:
    g = _gate(rec)
    if g.get("occ") in ("BUSY", "EXITING"):
        return "BLOCKED"
    if rec.get("mode") in HAZARD_MODES:
        return "HAZARD"
    if not g.get("has_prio"):
        return "NO_PRIORITY"
    return "UNEXPLAINED"


def _resolution(ctrl, states, geo, link, k, ep_k, j, c, m, w_kj) -> Dict:
    """Concurrent commitment: j committed (record c) before k's claim could be used at j."""
    ep_j = (c, next((y for y in range(c, m) if not states[j][y].get("committed")), m))
    w_jk = use_window(ctrl, states, geo, link, j, ep_j, k, m)
    a_k, a_j = _admission(states[k], ep_k[0], ep_k[1]), _admission(states[j], ep_j[0], ep_j[1])
    out = {"t_admitted_claimant": None if a_k is None else _t(ctrl[a_k]),
           "t_admitted_drone": None if a_j is None else _t(ctrl[a_j]),
           "t_drone_claim_usable_at_claimant": _t(w_jk["start"]),
           "end_edges": [_end_edge(states[k], ep_k[1], m), _end_edge(states[j], ep_j[1], m)]}
    if UNKNOWN in (w_jk["start"], w_jk["end"], w_kj["end"]):
        return {**out, "class": "INCONCLUSIVE", "why": "geometry or phase missing for the concurrent pair"}
    why = []
    if a_k is not None and _usable(w_jk, ctrl[a_k]):
        why.append("claimant admitted while the drone's claim could be used at it")
    if a_j is not None and _usable(w_kj, ctrl[a_j]):
        why.append("drone admitted while the claimant's claim could be used at it")
    early = lambda a, w: w["start"] is None or ctrl[a] < w["start"]
    if not why and a_k is not None and a_j is not None and early(a_k, w_jk) and early(a_j, w_kj):
        why.append("both admitted before either claim could be used")
    if why:
        return {**out, "class": "VIOLATED", "why": "; ".join(why)}
    if ep_k[1] >= m or ep_j[1] >= m:
        return {**out, "class": "INCONCLUSIVE", "why": "logs end before the concurrent pair is resolved"}
    kinds = ["ADMITTED" if a is not None else ("WITHDRAWN" if e == "pass_abort" else "PASSED_WITHOUT_ADMISSION")
             for a, e in ((a_k, out["end_edges"][0]), (a_j, out["end_edges"][1]))]
    res = ("SEQUENTIAL" if kinds == ["ADMITTED", "ADMITTED"] else "BOTH_WITHDRAWN" if kinds == ["WITHDRAWN", "WITHDRAWN"]
           else "PASSED_WITHOUT_ADMISSION" if "PASSED_WITHOUT_ADMISSION" in kinds else "WITHDRAWAL")
    return {**out, "class": "CONCURRENT", "resolution": res, "ends": kinds,
            "why": "committed before the claimant's claim could be used at the drone"}


def _m3(ctrl, states, geo, k, i0, j, c) -> Optional[bool]:
    """A commit of j at record c with no usable claim of k is explained by the corridor premise M3: j's latch passed through
    EXITING (a sonar exit witness) after k's commit and k's true centre had entered a CR box and was outside every box at
    j's commit.  None: geometry missing."""
    seen = geo.passed(k, int(ctrl[i0]), int(ctrl[c]))
    if seen is None:
        return None
    return bool(seen and any(_gate(states[j][y]).get("occ") == "EXITING" for y in range(i0, c)))


def _commit_row(ctrl, states, geo, link, k, ep, j, c, m, w) -> Dict:
    """Row of one commit edge (record c) of j during k's claim."""
    row = {"t_commit": _t(ctrl[c]), "latch_before_commit": _gate(states[j][c - 1]).get("occ")}
    if link is not None and w["start"] == UNKNOWN:
        return {**row, "knowledge": "UNKNOWN", "blocking": "INCONCLUSIVE", "class": "INCONCLUSIVE",
                "why": "geometry or phase missing"}
    if link is not None and w["emissions"] == 0 and not w.get("ignored"):
        if w["stop"] is None:
            return {**row, "knowledge": "UNKNOWN", "blocking": "INCONCLUSIVE", "class": "INCONCLUSIVE",
                    "why": "the logs end before the claimant's first ping"}
        r = _resolution(ctrl, states, geo, link, k, ep, j, c, m, w)
        return {**row, **r, "knowledge": "NOT_EMITTED", "blocking": r["class"]}
    if link is not None and w["start"] is not None:
        if ctrl[c] >= w["start"]:
            n_use = sum(1 for u in w["uses"] if u <= ctrl[c])
            return {**row, "knowledge": "NOT_HELD", "blocking": "VIOLATED", "class": "VIOLATED", "uses_before_commit": n_use,
                    "why": "committed (latch FREE) while the claim could be used at the drone"}
        r = _resolution(ctrl, states, geo, link, k, ep, j, c, m, w)
        return {**row, **r, "knowledge": "BEFORE_USE", "blocking": r["class"]}
    know = "NOT_APPLICABLE" if link is None else "UNDELIVERABLE"
    ok = _m3(ctrl, states, geo, k, ep[0], j, c)
    if ok is None:
        return {**row, "knowledge": know, "blocking": "INCONCLUSIVE", "class": "INCONCLUSIVE", "why": "geometry missing"}
    if ok:
        return {**row, "knowledge": know, "blocking": "SATISFIED", "basis": "EXIT_WITNESSED", "class": "SATISFIED"}
    cls = "INCONCLUSIVE" if link is None else "VIOLATED"
    why = ("B0: no claim; commit during the commitment without an exit witness (corridor premise M3)" if link is None
           else "claim never usable at the drone and the commit is not explained by an exit witness")
    return {**row, "knowledge": know, "blocking": cls, "class": cls, "why": why}


def _quiet_row(states, know: str, j, i_a, i_b, open_end: bool) -> Dict:
    """Row of a pair without commit of j: the steps at its queue point inside the window, by the logged guard inputs."""
    sj = states[j]
    at_q = [x for x in range(i_a, i_b) if _gate(sj[x]).get("at_queue") and not sj[x].get("committed")]
    reasons = [_step_reason(sj[x]) for x in at_q]
    steps = {s: reasons.count(s) for s in sorted(set(reasons))}
    basis = ("BLOCKED" if "BLOCKED" in steps else "PRECEDENCE" if "NO_PRIORITY" in steps
             else "HAZARD" if "HAZARD" in steps else "NOT_AT_QUEUE")
    row = {"t_commit": None, "knowledge": know, "steps_at_queue": len(at_q),
           "steps": steps, "unexplained": steps.get("UNEXPLAINED", 0), "basis": basis}
    if open_end and at_q:
        return {**row, "blocking": "INCONCLUSIVE", "class": "INCONCLUSIVE", "why": "the logs end while the claim holds"}
    return {**row, "blocking": "SATISFIED", "class": "SATISFIED"}


def classify(ctrl, states, geo: Geometry, link: Optional[Link], k: int, ep: tuple, j: int, m: int) -> List[Dict]:
    """Rows of the claim of commitment ``ep`` of k against drone j: one per commit edge of j from k's commit to the end of the
    claim at j, else one row for the window.  ``link`` None: the run carries no claim (B0, an archived run without payload)."""
    i0, i1 = ep
    sj = states[j]
    if sj[min(i0, m - 1)].get("mode") == "DEAD":
        return []
    w = (use_window(ctrl, states, geo, link, k, ep, j, m) if link is not None else
         {"start": None, "end": None, "emissions": 0, "uses": [], "stop": None, "gate_idx": None})
    usable = link is not None and w["start"] not in (None, UNKNOWN)
    if usable and w["end"] not in (None, UNKNOWN):
        i_end = bisect_left(ctrl, w["end"])
    elif usable or w["start"] == UNKNOWN:
        i_end = m
    else:
        i_end = i1
    i_end = max(i_end, i0 + 1)
    base = {"t_commit_claimant": _t(ctrl[i0]), "t_claim_end": None if i1 >= m else _t(ctrl[i1]),
            "t_claim_usable": _t(w["start"]) if w["start"] != UNKNOWN else UNKNOWN,
            "t_claim_end_usable": _t(w["end"]) if w["end"] != UNKNOWN else UNKNOWN, "claim_emissions": w["emissions"],
            "queued_at_commit": bool(_gate(sj[i0]).get("at_queue")) and not sj[i0].get("committed")}
    edges = [x for x in range(i0, min(i_end, m)) if x >= 1 and sj[x].get("committed") and not sj[x - 1].get("committed")]
    edges = [c for c in edges if not (c == i0 and j < k)]
    g_j = (_gate(sj[i0]).get("tx") or [None])[0]
    if link is not None and w["gate_idx"] is not None and g_j is not None and w["gate_idx"] != g_j:
        if not edges and not any(_gate(sj[x]).get("at_queue") for x in range(i0, min(i_end, m))):
            return []
        return [{**base, "t_commit": None if not edges else _t(ctrl[edges[0]]), "knowledge": "UNKNOWN",
                 "blocking": "INCONCLUSIVE", "class": "INCONCLUSIVE",
                 "why": f"claim of gate index {w['gate_idx']}, the drone reports gate index {g_j}"}]
    if edges:
        return [{**base, **_commit_row(ctrl, states, geo, link, k, ep, j, c, m, w)} for c in edges]
    if sj[i0].get("committed"):
        return []
    arrived = any(_gate(sj[x]).get("at_queue") for x in range(i0, min(i_end, m)))
    if not arrived:
        return []
    unsent = link is not None and w["emissions"] == 0 and not w.get("ignored")
    know = ("NOT_APPLICABLE" if link is None else "UNKNOWN" if w["start"] == UNKNOWN or (unsent and w["stop"] is None) else
            "NOT_EMITTED" if unsent else "NO_COMMIT" if usable else "UNDELIVERABLE")
    return [{**base, **_quiet_row(states, know, j, i0, min(i_end, m), open_end=i_end >= m)}]


def physical_entries(t, P, states, gate, cfg, names) -> List[Dict]:
    """Every entry of a drone's true centre into the CR box (referee geometry), with its kind and the occupancy."""
    from holo_fleet.referee.robustness_metrics import cr_entry_kind

    G = cfg.gate
    n = len(names)
    st_t = [np.array([float(r["t"]) for r in s]) for s in states]
    gf = np.array([[gate.to_gate_frame(P[i, k]) for k in range(n)] for i in range(len(t))])
    inside = G.in_cr(gf[:, :, 0], gf[:, :, 1], gf[:, :, 2])
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


def link_of(beacon: Optional[Dict], n: int, cfg, identities: Optional[Dict] = None) -> Optional[Link]:
    """Claim transport of the run, None when no claim is carried (B0, an archived run without payload, latency unknown).
    ``identities``: receiver -> {pinger code (drone index) -> slot} (runner.beacon_identities)."""
    b = beacon or {}
    if not (b.get("enabled") and b.get("payload", True)) or b.get("latency_s") is None:
        return None
    return Link(n=n, period_s=float(b.get("period_s") or 1.0), latency_s=float(b["latency_s"]),
                latency_max_s=float(b.get("latency_max_s") or cfg.acoustic.latency_max_s),
                max_range_m=float(b.get("max_range_m") or 50.0), sound_speed=float(b.get("sound_speed") or 1500.0),
                blackout=bool(b.get("blackout")),
                ids=None if identities is None else {int(j): {int(k): int(v) for k, v in d.items()}
                                                     for j, d in identities.items()})


def _inside_any(t, P, gates, cfg, n) -> Optional[np.ndarray]:
    if P is None or not gates:
        return None
    G = cfg.gate
    inside = np.zeros((len(t), n), bool)
    for g in gates:
        gf = np.array([[g.to_gate_frame(P[i, k]) for k in range(n)] for i in range(len(t))])
        inside |= G.in_cr(gf[..., 0], gf[..., 1], gf[..., 2])
    return inside


def audit(names, t, P, states, gates, cfg, beacon: Optional[Dict] = None, identities: Optional[Dict] = None) -> Dict:
    """Premise C pairs (class, knowledge and blocking verdicts) and physical CR entries of one run.  ``beacon``: as
    beacon_of_run (enabled, payload, period, channel latency, blackout, range, sound speed, acceptance limit)."""
    n, m = len(names), min(len(s) for s in states)
    ctrl = ticks([r["t"] for r in states[0][:m]])
    link = link_of(beacon, n, cfg, identities)
    claims = bool((beacon or {}).get("enabled") and (beacon or {}).get("payload", True))
    geo = Geometry(t, P, _inside_any(np.asarray(t, float), P, gates, cfg, n))
    pairs = []
    for k in range(n):
        for ep in _episodes(states[k], m):
            for j in range(n):
                if j == k:
                    continue
                for r in classify(ctrl, states, geo, link, k, ep, j, m):
                    if claims and link is None:
                        r.update({"class": "INCONCLUSIVE", "knowledge": "UNKNOWN", "why": "channel latency of the run unknown"})
                    pairs.append({"claimant": names[k], "drone": names[j], **r})
    entries = []
    for g in gates:
        entries += physical_entries(np.asarray(t, float), P, states, g, cfg, names)
    count = lambda key, vals: {c: sum(1 for p in pairs if p.get(key) == c) for c in vals}
    return {"pairs": pairs, "counts": count("class", CLASSES), "knowledge_counts": count("knowledge", KNOWLEDGE),
            "blocking_counts": count("blocking", BLOCKING), "entries": entries,
            "entry_counts": {k: sum(1 for e in entries if e["kind"] == k) for k in ("ADMITTED", "PUSHED", "UNCOMMITTED")},
            "entries_with_another_inside": sum(1 for e in entries if e["others_inside_at_entry"] or e["others_inside_while_in"] > 0),
            "claim_delivery_s": None if link is None else link.latency_s, "b2": claims,
            "link": None if link is None else {**{k: v for k, v in asdict(link).items() if k != "ids"},
                                               "period_ticks": link.period_ticks, "slot_ticks": link.slot_ticks}}


def beacon_of_run(rc: Dict) -> Dict:
    """Beacon facts of a run from its run_config: enabled, payload, TDMA period, the channel's minimum processing latency and
    blackout, range, sound speed, acceptance limit.  ``payload`` is read from the run_config of an archived run of the payload-free
    ablation (``false``); it is absent from every run of the single controller, which always sends the packet."""
    from holo_fleet.sim.acoustic_channel import channel

    fc = rc.get("fleet_config") or {}
    b = fc.get("beacon") or {}
    enabled = b.get("mode") == COMMUNICATION_ACOUSTIC
    lat, blackout = None, False
    if enabled:
        try:
            ch = channel(b.get("channel") or "CLEAN")
            lat, blackout = float(ch.latency_s), bool(ch.blackout)
        except (KeyError, StopIteration, ValueError):
            lat = None
    return {"enabled": enabled, "payload": bool(b.get("payload", True)), "period_s": float(b.get("period_s") or 1.0),
            "latency_s": lat, "blackout": blackout, "max_range_m": b.get("max_range_m"), "sound_speed": b.get("sound_speed"),
            "latency_max_s": (fc.get("acoustic") or {}).get("latency_max_s"), "channel": b.get("channel")}
