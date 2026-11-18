"""Offline audit of premise C of P2 and of every physical entry into the critical region (CR), from the run logs.

Premise C: a drone at a gate learns of another drone's commitment before it commits itself; two drones that commit
before either can learn of the other resolve it at the stage-2 check, never by an admission made while the other's
claim could already be known.

Behavioural-blocking contract (what is checked).  The audit uses only what the logs show:
  * a drone j commits only when its latch is FREE (commit guard: calm, at_queue, has_prio, not occ_busy), and a claim
    held by j forces its latch BUSY (perception/gate_perception.py: a heard claim makes the CR occupied).  A commit of j
    therefore shows that j held no claim of k at that step;
  * whether j COULD hold k's claim is decided from the integer-tick TDMA schedule of sim/beacon.py and the channel
    delays (``earliest_use``): the claim emission ticks of k, the propagation delay from the true distance, the minimum
    channel latency (jitter only delays further), the first tick after the due time, the first control step after that
    tick, and the stale rule of perception/acoustic.py (age on use > ``latency_max_s``: never accepted); a BLACKOUT
    channel delivers nothing.  This is the earliest instant at which the claim could have been USED, not a reception:
    random losses, outages and collisions are not excluded, per-packet receptions are not logged and none is assumed;
  * non-commit is explained per control step by the logged guard inputs: latch not FREE, no priority, not at the queue
    point, a hazard mode.  A step with all guard inputs true and no commit is unexplained.
Claimant-specific knowledge (j knows k's claim) is not logged: a BUSY latch may come from a corridor echo or another
drone.  BUSY is therefore reported as behavioural blocking, never as proof that j knew k's claim.

Pair classes, for each claim episode of k and each other live drone j at the gate:
  SATISFIED     j did not commit while k's claim held for j, and every step of j at its queue point in that window is
                explained by the logged guard inputs (``basis``: BLOCKED, PRECEDENCE, BEFORE_USE);
  CONCURRENT    j committed before k's claim could be used at j, and the pair was resolved within the logs without an
                admission made while the other's claim could be used (``resolution``: SEQUENTIAL or WITHDRAWAL);
  VIOLATED      j committed while k's claim could already be used at j (each commit edge, recommitments included), or a
                drone of a concurrent pair was admitted while the other's claim could be used, or both were admitted
                before either could learn of the other, or the claim could never be used (stale beyond the limit);
  INCONCLUSIVE  the logs end before the pair is decided, geometry or phase evidence is missing, a non-commit is not
                explained, or, without a claim payload (B0, B2 without payload), j committed before an exit witness of
                its latch with the claimant truly outside the CR (the corridor premise M3: that case is SATISFIED,
                basis EXIT_WITNESSED).  Every commit row records ``sonar_exit_witnessed``.
The physical entries list every entry of a drone's true centre into the CR box: ADMITTED, PUSHED (committed, not
admitted), UNCOMMITTED, with the drones inside at that instant.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

CLAIM_PHASES = {1, 2}
CLASSES = ("SATISFIED", "CONCURRENT", "VIOLATED", "INCONCLUSIVE")
TICKS_PER_SEC = 30          # sim/holo_env.TICKS_PER_SEC
HAZARD_MODES = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT")
UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Link:
    """Beacon timing of a run: TDMA (sim/beacon.BeaconTransport) and channel delays (sim/acoustic_channel)."""

    n: int
    period_s: float = 1.0
    latency_s: float = 0.0
    latency_max_s: float = 0.4
    max_range_m: float = 50.0
    sound_speed: float = 1500.0
    blackout: bool = False
    tps: int = TICKS_PER_SEC

    @property
    def period_ticks(self) -> int:
        return max(1, int(round(self.period_s * self.tps)))

    @property
    def slot_ticks(self) -> int:
        return max(1, self.period_ticks // max(1, self.n))

    def emits(self, k: int, tick: int) -> bool:
        return tick >= 1 and (tick - k * self.slot_ticks) % self.period_ticks == 0

    def next_emission(self, k: int, tick: int) -> int:
        """First emission tick of drone k strictly after ``tick``."""
        off = k * self.slot_ticks
        n = tick + 1
        r = (n - off) % self.period_ticks
        return n if r == 0 else n + self.period_ticks - r


def ticks(times) -> np.ndarray:
    return np.array([int(round(float(x) * TICKS_PER_SEC)) for x in times], dtype=int)


def earliest_use(e_tick: int, dist_m, link: Link, ctrl_ticks: Sequence[int]):
    """Control tick at which a reception of the ping emitted at ``e_tick`` could first be used by the receiver, None if
    it can never be accepted (out of range, or stale already at the minimum delay), UNKNOWN without the geometry."""
    if dist_m is None or not math.isfinite(float(dist_m)):
        return UNKNOWN
    if link.blackout or float(dist_m) > link.max_range_m:
        return None
    t_arr = e_tick / link.tps + float(dist_m) / link.sound_speed
    d_tick = max(e_tick + 1, int(math.ceil((t_arr + link.latency_s) * link.tps - 1e-6)))
    i = bisect_left(ctrl_ticks, d_tick)
    if i >= len(ctrl_ticks):
        return UNKNOWN
    c = int(ctrl_ticks[i])
    if c / link.tps - t_arr > link.latency_max_s + 1e-9:
        return None
    return c


def _gate(rec) -> Dict:
    return rec.get("gate") or {}


def _claims(rec) -> Optional[bool]:
    tx = _gate(rec).get("tx")
    if not tx or len(tx) < 2 or tx[1] is None:
        return None
    try:
        return int(tx[1]) in CLAIM_PHASES
    except (TypeError, ValueError):
        return None


def _episodes(states_k, m: int) -> List[tuple]:
    out, i = [], 1
    while i < m:
        if states_k[i].get("committed") and not states_k[i - 1].get("committed"):
            e = next((x for x in range(i, m) if not states_k[x].get("committed")), m)
            out.append((i, e))
            i = e
        i += 1
    return out


def _record_before(ctrl, tick: int) -> int:
    """Index of the last control record strictly before ``tick``: a ping in the ticks after the control step at T
    carries the phase set at T (runner: controllers step, then sim.step ticks at T + 1/30 ... T + DT)."""
    return bisect_left(ctrl, tick) - 1


def claim_emissions(ctrl, states_k, k: int, i0: int, link: Link, m: int):
    """Claim emission ticks of drone k for its episode starting at record i0, and the first emission after them that
    carries no claim (None if k stops emitting, e.g. it dies, or the logs end first)."""
    out, tick = [], int(ctrl[i0])
    last = int(ctrl[m - 1])
    while True:
        tick = link.next_emission(k, tick)
        if tick > last:
            return out, None, False
        j = _record_before(ctrl, tick)
        if j < 0 or j >= m or states_k[j].get("mode") == "DEAD":
            return out, None, True
        c = _claims(states_k[j])
        if c is None:
            return out, UNKNOWN, False
        if c:
            out.append(tick)
        elif out or not states_k[j].get("committed"):
            return out, tick, False


def first_claim_emission(t, states_k, k: int, i0: int, i1: int, period_s: float, n_drones: int) -> Optional[float]:
    """Time of the first TDMA emission of drone k after its commit record i0 that carries the claim (None if none)."""
    ctrl = list(ticks(t))
    em, _, _ = claim_emissions(ctrl, states_k, k, i0, Link(n=n_drones, period_s=period_s), min(len(ctrl), len(states_k)))
    return em[0] / TICKS_PER_SEC if em else None


class Geometry:
    """True distances between drones at a tick, and whether a drone's true centre is inside a CR box (referee ground
    truth; ``inside``: [time, drone] booleans, None when unknown)."""

    def __init__(self, t_ref, P, inside=None):
        self.t = np.asarray(t_ref, float)
        self.P = None if P is None else np.asarray(P, float)
        self.inside = None if inside is None else np.asarray(inside, bool)

    def _i(self, tick: int) -> int:
        return int(np.clip(np.searchsorted(self.t, tick / TICKS_PER_SEC - 1e-9), 0, max(0, len(self.t) - 1)))

    def exited(self, k: int, tick0: int, tick1: int) -> Optional[bool]:
        """Drone k's true centre entered a CR box after tick0 and is outside every box at tick1 (None: unknown)."""
        if self.inside is None or len(self.t) == 0:
            return None
        a, b = self._i(tick0), self._i(tick1)
        return bool(self.inside[a:b + 1, k].any() and not self.inside[b, k])

    def dist(self, a: int, b: int, tick: int):
        if self.P is None or len(self.t) == 0:
            return None
        i = int(np.clip(np.searchsorted(self.t, tick / TICKS_PER_SEC - 1e-9), 0, len(self.t) - 1))
        if abs(self.t[i] - tick / TICKS_PER_SEC) > 0.2:
            return None
        return float(np.linalg.norm(self.P[i, a] - self.P[i, b]))


def use_window(ctrl, states, geo: Geometry, link: Link, k: int, ep: tuple, j: int, m: int) -> Dict:
    """When k's claim of episode ``ep`` could first be used at j (``start``), and when j could first learn that it ended
    (``end``: the first non-claim emission, or j's own declaration of k dead).  Ticks, None or UNKNOWN."""
    em, after, died = claim_emissions(ctrl, states[k], k, ep[0], link, m)
    if after == UNKNOWN:
        return {"start": UNKNOWN, "end": UNKNOWN, "emissions": em}
    start = None
    for e in em:
        u = earliest_use(e, geo.dist(k, j, e), link, ctrl)
        if u == UNKNOWN:
            start = UNKNOWN
            break
        if u is not None:
            start = u
            break
    end = None
    if after is not None:
        end = earliest_use(after, geo.dist(k, j, after), link, ctrl)
    elif died:
        d = next((x for x in range(ep[0], m) if k in (_gate(states[j][x]).get("dead") or [])), None)
        end = None if d is None else int(ctrl[d])
    return {"start": start, "end": end, "emissions": em}


def _admission(states_x, a: int, b: int) -> Optional[int]:
    return next((y for y in range(a, b) if _gate(states_x[y]).get("entry") == "ADMIT"), None)


def _episode_end(states_x, c: int, m: int) -> int:
    return next((y for y in range(c, m) if not states_x[y].get("committed")), m)


def _exit_witnessed(states_x, i0: int, c: int) -> bool:
    """The latch of the drone passed through EXITING (an exit witness) between records i0 and c."""
    return any(_gate(states_x[y]).get("occ") == "EXITING" for y in range(i0, c))


def _step_reason(rec) -> str:
    g = _gate(rec)
    if g.get("occ") in ("BUSY", "EXITING"):
        return "BLOCKED"
    if rec.get("mode") in HAZARD_MODES:
        return "HAZARD"
    if not g.get("has_prio"):
        return "NO_PRIORITY"
    return "UNEXPLAINED"


def _t(ctrl, tick) -> Optional[float]:
    return None if tick is None or tick == UNKNOWN else round(tick / TICKS_PER_SEC, 3)


def _concurrent(ctrl, states, geo, link, k, ep_k, j, c, m, w_kj) -> Dict:
    """Resolution of a commitment of j made before k's claim could be used at j."""
    ep_j = (c, _episode_end(states[j], c, m))
    w_jk = use_window(ctrl, states, geo, link, j, ep_j, k, m)
    a_k, a_j = _admission(states[k], ep_k[0], ep_k[1]), _admission(states[j], ep_j[0], ep_j[1])
    out = {"t_admitted_claimant": None if a_k is None else _t(ctrl, ctrl[a_k]),
           "t_admitted_drone": None if a_j is None else _t(ctrl, ctrl[a_j]),
           "t_drone_claim_usable_at_claimant": _t(ctrl, w_jk["start"])}
    if UNKNOWN in (w_jk["start"], w_jk["end"], w_kj["end"]):
        return {**out, "class": "INCONCLUSIVE", "why": "missing geometry or phase evidence for the concurrent pair"}

    def usable(w, tick):
        return w["start"] is not None and tick >= w["start"] and (w["end"] is None or tick < w["end"])

    why = []
    if a_k is not None and ctrl[a_k] >= ctrl[c] and usable(w_jk, ctrl[a_k]):
        why.append("claimant admitted while the drone's claim could be used")
    if a_j is not None and usable(w_kj, ctrl[a_j]):
        why.append("drone admitted while the claimant's claim could be used")
    if (a_k is not None and a_j is not None and ctrl[a_k] >= ctrl[c] and not why
            and (w_jk["start"] is None or ctrl[a_k] < w_jk["start"])
            and (w_kj["start"] is None or ctrl[a_j] < w_kj["start"])):
        why.append("both admitted before either claim could be used")
    if why:
        return {**out, "class": "VIOLATED", "why": "; ".join(why)}
    if ep_k[1] >= m or ep_j[1] >= m:
        return {**out, "class": "INCONCLUSIVE", "why": "logs end before the concurrent pair is resolved"}
    res = "WITHDRAWAL" if (a_k is None or a_j is None) else "SEQUENTIAL"
    return {**out, "class": "CONCURRENT", "resolution": res,
            "why": "committed before the claimant's claim could be used at the drone"}


def classify(ctrl, states, geo: Geometry, link: Optional[Link], k: int, ep: tuple, j: int, m: int) -> List[Dict]:
    """Rows for the claim episode ``ep`` of k and drone j: one per commit edge of j while k's claim held for j, or one
    row for the whole window when j did not commit.  ``link`` None: the run carries no claim (B0, B2 without payload)."""
    i0, i1 = ep
    sj = states[j]
    if sj[min(i0, m - 1)].get("mode") == "DEAD":
        return []
    base = {"t_commit_claimant": _t(ctrl, ctrl[i0]), "t_claim_end": _t(ctrl, ctrl[min(i1, m - 1)])}
    if link is None:
        w = {"start": None, "end": None}
        end_idx = i1
    else:
        w = use_window(ctrl, states, geo, link, k, ep, j, m)
        base.update({"t_claim_usable": _t(ctrl, w["start"]), "t_end_usable": _t(ctrl, w["end"]),
                     "claim_emissions": len(w["emissions"])})
        if w["end"] == UNKNOWN or w["start"] == UNKNOWN:
            return [{**base, "class": "INCONCLUSIVE", "why": "missing geometry or phase evidence"}]
        end_idx = i1 if w["start"] is None else (m if w["end"] is None else bisect_left(ctrl, w["end"]))
    end_idx = min(end_idx, m)
    commits = [x for x in range(i0, end_idx) if sj[x].get("committed") and (x == 0 or not sj[x - 1].get("committed"))
               and (x > i0 or k < j)]
    at_q = [x for x in range(i0, end_idx) if _gate(sj[x]).get("at_queue") and not sj[x].get("committed")
            and sj[x].get("mode") != "DEAD"]
    if not commits and not at_q:
        return []
    rows = []
    for c in commits:
        exit_ok = _exit_witnessed(sj, i0, c) and geo.exited(k, ctrl[i0], ctrl[c])
        r = {**base, "t_commit": _t(ctrl, ctrl[c]), "release_at_commit": _gate(sj[c - 1]).get("occ") in ("BUSY", "EXITING"),
             "sonar_exit_witnessed": bool(exit_ok)}
        if link is None:
            if exit_ok:
                r.update({"class": "SATISFIED", "basis": "EXIT_WITNESSED",
                          "why": "no claim payload: released by a sonar exit witness, the claimant outside the CR (M3)"})
            else:
                r.update({"class": "INCONCLUSIVE", "why": "no claim payload: committed before the claimant's exit was witnessed"})
        elif w["start"] is None or ctrl[c] < w["start"]:
            if w["start"] is None and w["emissions"] and ctrl[c] > w["emissions"][0]:
                r.update({"class": "VIOLATED", "why": ("the claim could never be used (BLACKOUT channel)" if link.blackout else
                                                       "the claim could never be used (stale beyond the limit or out of range)")})
            else:
                r.update(_concurrent(ctrl, states, geo, link, k, ep, j, c, m, w))
        else:
            dead = k in (_gate(sj[c]).get("dead") or [])
            usable = sum(1 for e in w["emissions"] if (lambda u: u not in (None, UNKNOWN) and u <= ctrl[c])(
                earliest_use(e, geo.dist(k, j, e), link, ctrl)))
            r.update({"class": "VIOLATED", "claim_uses_possible_before_commit": usable,
                      "why": ("committed with k declared dead by the drone (premise V)" if dead else
                              "committed with its latch FREE while the claim could be used: it held no claim of k")})
        rows.append(r)
    if commits:
        return rows
    reasons = [_step_reason(sj[x]) for x in at_q]
    after = [rs for x, rs in zip(at_q, reasons) if link is not None and w["start"] is not None and ctrl[x] >= w["start"]]
    releases = sum(1 for x in at_q if x > i0 and _gate(sj[x - 1]).get("occ") in ("BUSY", "EXITING")
                   and _gate(sj[x]).get("occ") == "FREE")
    r = {**base, "t_commit": None, "steps_at_queue": len(at_q), "steps": {s: reasons.count(s) for s in sorted(set(reasons))},
         "releases_while_claimed": releases}
    if "UNEXPLAINED" in reasons:
        r.update({"class": "INCONCLUSIVE", "why": "a step at the queue with every logged guard input true and no commit"})
    elif end_idx >= m:
        r.update({"class": "INCONCLUSIVE", "why": "the logs end while the claim holds"})
    else:
        if link is not None and w["start"] is not None and not after:
            basis = "BEFORE_USE"
        elif all(rs == "BLOCKED" for rs in (after if after else reasons)):
            basis = "BLOCKED"
        else:
            basis = "PRECEDENCE"
        r.update({"class": "SATISFIED", "basis": basis})
    return [r]


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
    """Premise C pairs and physical CR entries of one run.  ``beacon``: beacon_of_run (enabled, payload, timing)."""
    n, m = len(names), min(len(s) for s in states)
    ctrl = [int(x) for x in ticks([r["t"] for r in states[0][:m]])]
    b = beacon or {}
    claims = bool(b.get("enabled") and b.get("payload", True))
    link = None
    if claims:
        if b.get("latency_s") is None:
            link = None
        else:
            link = Link(n=n, period_s=float(b.get("period_s") or 1.0), latency_s=float(b["latency_s"]),
                        latency_max_s=float(b.get("latency_max_s") or cfg.acoustic.latency_max_s),
                        max_range_m=float(b.get("max_range_m") or 50.0), sound_speed=float(b.get("sound_speed") or 1500.0),
                        blackout=bool(b.get("blackout")))
    inside = None
    if P is not None and gates:
        G = cfg.gate
        half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
        inside = np.zeros((len(t), n), bool)
        for g in gates:
            gf = np.array([[g.to_gate_frame(P[i, k]) for k in range(n)] for i in range(len(t))])
            inside |= np.all(np.abs(gf) <= half, axis=2)
    geo = Geometry(t, P, inside)
    pairs = []
    for k in range(n):
        for ep in _episodes(states[k], m):
            for j in range(n):
                if j == k:
                    continue
                for r in classify(ctrl, states, geo, link, k, ep, j, m):
                    if claims and link is None:
                        r.update({"class": "INCONCLUSIVE", "why": "channel latency of the run unknown"})
                    pairs.append({"claimant": names[k], "drone": names[j], **r})
    entries = []
    for g in gates:
        entries += physical_entries(np.asarray(t, float), P, states, g, cfg, names)
    counts = {c: sum(1 for p in pairs if p["class"] == c) for c in CLASSES}
    return {"pairs": pairs, "counts": counts, "entries": entries,
            "entry_counts": {k: sum(1 for e in entries if e["kind"] == k) for k in ("ADMITTED", "PUSHED", "UNCOMMITTED")},
            "entries_with_another_inside": sum(1 for e in entries if e["others_inside_at_entry"] or e["others_inside_while_in"] > 0),
            "claim_delivery_s": None if link is None else link.latency_s, "b2": claims,
            "link": None if link is None else {"n": link.n, "period_ticks": link.period_ticks, "slot_ticks": link.slot_ticks,
                                               "latency_s": link.latency_s, "latency_max_s": link.latency_max_s,
                                               "blackout": link.blackout}}


def beacon_of_run(rc: Dict) -> Dict:
    """Beacon facts of a run from its run_config: enabled, payload, TDMA period, channel latency, range, sound speed,
    acceptance limit."""
    from holo_fleet.sim.acoustic_channel import channel

    fc = rc.get("fleet_config") or {}
    b = fc.get("beacon") or {}
    enabled = b.get("mode") == "ACOUSTIC_RB"
    lat, blackout = None, False
    if enabled:
        try:
            ch = channel(b.get("channel") or "CLEAN")
            lat, blackout = float(ch.latency_s), bool(ch.blackout)
        except (KeyError, StopIteration, ValueError):
            lat = None
    return {"enabled": enabled, "payload": bool(b.get("payload", True)), "period_s": float(b.get("period_s") or 1.0),
            "latency_s": lat, "max_range_m": b.get("max_range_m"), "sound_speed": b.get("sound_speed"),
            "latency_max_s": (fc.get("acoustic") or {}).get("latency_max_s"), "channel": b.get("channel"), "blackout": blackout}
