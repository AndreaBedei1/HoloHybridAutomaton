"""Gate-related perception (P2), onboard information only.

* own gate-frame position and heading error from the navigation estimate and the gate map;
* the own queue point (abreast queue line, lateral order of the formation slots; slots stacked in depth
  share a column and queue one above the other: mission.queue_assignment, no ID involved);
* the relation to every obstacle target inside the queue bracket, from its SECTOR PATTERN only
  (ha/mutex_rule.py), and the resulting decision with a persistence requirement (``t_clear``);
* the critical-region occupancy belief.  From the FRONT sonar position the drone computes the
  smallest and largest range of the CR (gate map + own pose).  A FRONT echo between
  r_near - R_OUT - eps and r_far + eps may come from a drone inside the CR ("corridor"); one
  farther is certainly beyond it; one nearer is a drone between the queue line and the CR (it is
  handled by the WAIT relation).
  A drone in the gate opening is masked by the bars (its echo falls inside the structure window),
  so the belief is a latch: BUSY as soon as an unexplained corridor echo is seen, BUSY while it is
  seen or masked, EXITING on a positive exit witness with the corridor empty (an echo in any sonar
  sector that no hull in the CR can produce and a hull leaving it on the exit side can: exit_witness.py;
  the FRONT echo beyond R_far is the FRONT case), and FREE after ``t_clear`` more without conflicting evidence.
  The wait alone never gives FREE.  After ``t_occ_max`` with no corridor echo, nothing heard and nothing known inside, a BUSY
  set by a corridor echo becomes STALE: no longer an obstacle to committing (the commitment stops at the stage-2 hold,
  outside the CR, control/gate_safety.py).  Every admission that rests on no positive exit witness after the last sighting
  (STALE, BUSY, or FREE without any witness) needs ``entry_after``: the last sighting plus ``GateRule.t_admit_stale``
  (formal/check_stale.py), unless B2 holds fresh reports of every other drone (B2C-E1).  The sighting is a corridor
  echo or any hull within GateRule.near_sight_m (near field), and control/flows.py also counts the bound from the start of the
  current dwell in the admission zone, so an occupant admitted before that dwell needs no sighting at all (premise S is not
  needed).  A corridor echo makes a STALE latch BUSY again; the timed basis is kept until a witness or the next gate.
  B2 adds positive acoustic occupancy to BUSY and vetoes release/admission while a drone is heard possibly inside
  (the complete declared region, perception/acoustic_gate.py). Acoustic exit evidence is evaluated only with the
  corridor empty. Without receptions the sonar outage-mode latch remains.
* the stage-2 entry check ``entry_ok``: no unexplained corridor echo for ENTRY_QUIET_CAPTURES captures WITH the FRONT sector
  healthy over that window (a FRONT outage is never quiet: no echo is evidence only from a working sensor), no claim and no
  drone heard or last known inside;
* the B2 admission wait ``admit_wait_s`` (formal/check_b2_comm.py): the smallest age of the own commitment at which the drone
  may be admitted - ``commit_admit_min_s`` when a FRESH report (age <= ``admit_fresh_s``) of every other drone of the plan is
  held, ``commit_admit_min_s + GateRule.t_cross_max`` otherwise (silence is unknown, never clearance); 0 without the beacon
  (B0) or without other drones.
* a fresh same-gate commitment from a higher-priority queue slot may prompt a pre-admission concession. It still blocks
  admission like any other claim, whatever its age.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha import mutex_rule
from holo_fleet.mission import GateSpec, MissionPlan, queue_assignment, queue_order
from holo_fleet.perception.exit_witness import ExitWitness
from holo_fleet.perception.payload import GP_COMMITTED
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT
from holo_fleet.perception.sonar_processing import DYNAMIC

ENTRY_QUIET_CAPTURES = 3


def stale_release(t: float, last_corridor_t: float, busy_since: float, corridor: bool, heard: bool, known_inside: bool,
                  t_occ_max: float) -> bool:
    """BUSY -> STALE: a corridor-set latch (last echo not before BUSY began) with no corridor echo for more than
    ``t_occ_max``, nothing heard possibly inside and no last known position inside.  The same predicate is encoded in
    formal/check_stale.py."""
    return (not corridor and not heard and not known_inside and last_corridor_t >= busy_since
            and t - last_corridor_t > t_occ_max)


def timed_admission(release: Optional[str], comm_clear: bool = False) -> bool:
    """Whether the admission is bounded in time (formal/check_stale.py ST1): it rests on no positive exit witness after the
    last sighting (release WITNESS: formal M3) and B2 does not hold fresh reports of every other drone (B2C-E1)."""
    return not (release == "WITNESS" or comm_clear)


def entry_after(release: Optional[str], last_sighting_t: float, t_admit_stale: float, comm_clear: bool = False) -> float:
    """Earliest admission time imposed by the occupancy evidence: the last sighting (a corridor echo, or any hull within
    GateRule.near_sight_m) plus ``t_admit_stale`` for every admission that rests on no positive exit witness after it
    (control/flows.py adds the start of the dwell in the admission zone, formal/check_stale.py ST1).  No bound after a witness
    (release WITNESS: the latch argument of formal M3) or with B2 reports of every other drone fresh or of a verified
    passage (``comm_clear``: formal/check_b2_comm.py B2C-E1).  A drone that never saw anything (a lead drone, or one
    whose occupant is hidden, the M4f concurrent commitment of a B0 queue) is bounded too: absence is no evidence."""
    return last_sighting_t + t_admit_stale if timed_admission(release, comm_clear) else -1e9


@dataclass
class GateObs:
    gate_id: Optional[str] = None
    s: float = -1e9
    l: float = 0.0
    dz: float = 0.0
    heading_err_deg: float = 180.0
    in_zone: bool = False
    at_queue: bool = False
    passed: bool = False
    queue_l: float = 0.0
    queue_s: float = -4.6
    queue_dz: float = 0.0
    r_far: float = 0.0
    relations: List[Dict] = field(default_factory=list)
    raw_decision: str = mutex_rule.PRIORITY
    decision: str = mutex_rule.WAIT
    has_prio: bool = False
    occ_state: str = "FREE"
    occ_busy: bool = False
    rank_used: bool = False
    occ_timeouts: int = 0
    entry_after: float = -1e9
    timed_admission: bool = False
    exit_witness: Optional[Dict] = None
    loc_ok: bool = False
    entry_ok: bool = True
    dead: List[int] = field(default_factory=list)
    silent: List[int] = field(default_factory=list)
    admit_wait_s: float = 0.0
    comm_fresh: bool = True
    concede_to: Optional[int] = None


class GatePerception:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, acoustic=None):
        self.plan = plan
        self.cfg = cfg
        self.acoustic_exit = None
        if acoustic is not None and acoustic.enabled:
            from holo_fleet.perception.acoustic_gate import AcousticExit

            self.acoustic_exit = AcousticExit(cfg, acoustic, [k for k in range(len(plan.slots)) if k != plan.slot_index])
            G = cfg.gate
            t_min = (G.cr_s_min - plan.queue_s - R_OUT - G.queue_tol - G.nav_err_m) / self.acoustic_exit.v_bound
            self.acoustic_exit.status_fresh_s = max(0.0, min(cfg.acoustic.status_fresh_s, t_min - cfg.acoustic.latency_max_s))
        self.exit_kind = {"SONAR": 0, "ACOUSTIC": 0}
        self.idx = 0
        self.prio_since: Optional[float] = None
        self.occ_state = "FREE"
        self.last_corridor_t = -1e9
        self.last_blind_t = -1e9
        self.suspects: set = set()
        self.phantom, self.phantoms = False, 0
        self.phantom_events: list = []
        self.busy_since = -1e9
        self.exit_seen_t = -1e9
        self.last_near_t = -1e9
        self.occ_timeouts = 0
        self.release: Optional[str] = None
        self.exit = ExitWitness(cfg)
        self.exit_witness: Optional[Dict] = None
        self.rank_uses = 0
        self._rank_prev = False
        self.prev_targets = []
        G = cfg.gate
        qa = queue_assignment(plan.slots, G)
        self.queue_ranks = queue_order(plan.slots, G)
        me = np.array([0.0, plan.queue_lateral, plan.queue_dz])
        self.bracket_ranks = [self.queue_ranks[k] for k, (lk, dk) in enumerate(qa) if k != plan.slot_index
                              and float(np.linalg.norm(np.array([0.0, lk, dk]) - me)) <= G.queue_bracket_m]

    @property
    def gate(self) -> Optional[GateSpec]:
        return self.plan.gates[self.idx] if self.idx < len(self.plan.gates) else None

    def advance(self) -> None:
        self.idx += 1
        self.prio_since = None
        self.occ_state, self.last_corridor_t, self.exit_seen_t = "FREE", -1e9, -1e9
        self.last_near_t = -1e9
        self.release = None
        self.exit.reset()
        if self.acoustic_exit is not None:
            self.acoustic_exit.reset()

    def cr_ranges(self, g: GateSpec, p: np.ndarray, yaw: float):
        """Smallest and largest distance from the own FRONT sonar to a point of the CR (gate map + own pose)."""
        G = self.cfg.gate
        sonar = p + np.array([math.cos(yaw), math.sin(yaw), 0.0]) * float(MOUNTS["FRONT"][0])
        lo, hi = G.cr_lo(), G.passage_hi()
        c0, half = (lo + hi) / 2, (hi - lo) / 2
        q = g.to_gate_frame(sonar)
        near = float(np.linalg.norm(q - np.clip(q, lo, hi)))
        far = float(max(np.linalg.norm(q - c0 - half * np.array([a, b, c]))
                        for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)))
        return near, far

    def update(self, t: float, p: np.ndarray, yaw: float, targets, readings) -> GateObs:
        G = self.cfg.gate
        g = self.gate
        o = GateObs()
        if g is None:
            return o
        s, l, dz = g.to_gate_frame(p)
        gate_yaw = math.atan2(g.axis[1], g.axis[0])
        herr = math.degrees((yaw - gate_yaw + math.pi) % (2 * math.pi) - math.pi)
        o.gate_id, o.s, o.l, o.dz, o.heading_err_deg = g.gate_id, float(s), float(l), float(dz), herr
        o.queue_l, o.queue_s, o.queue_dz = self.plan.queue_lateral, self.plan.queue_s, self.plan.queue_dz
        half_w = max(G.corridor_half_width, abs(o.queue_l) + 1.0)
        o.in_zone = (o.queue_s - G.approach_len) <= s <= G.exit_s and abs(l) <= half_w
        o.passed = s > G.exit_s
        o.at_queue = (abs(s - o.queue_s) <= G.queue_tol and abs(l - o.queue_l) <= G.queue_tol
                      and abs(dz - o.queue_dz) <= G.queue_tol and abs(herr) <= G.heading_tol_deg)
        r_near, r_far = self.cr_ranges(g, p, yaw)
        r_near = r_near - R_OUT - self.cfg.env.eps_range_far
        r_far = r_far + self.cfg.env.eps_range_near
        o.r_far = r_far
        persistent = [tg for tg in targets if tg.cls == DYNAMIC or any(
            (set(tg.pattern) & set(pp)) and abs(tg.r_min - pr) <= 0.5 for pp, pr in self.prev_targets)]
        self.prev_targets = [(tuple(tg.pattern), tg.r_min) for tg in targets]
        targets = persistent
        # near-field sighting (any sector): a hull within GateRule.near_sight_m; with the corridor sighting and the dwell in
        # the admission zone it bounds the timed admission (formal/check_stale.py premise N)
        if any(tg.d_lower < G.near_sight_m for tg in targets):
            self.last_near_t = t
        ae = self.acoustic_exit
        if ae is not None:
            ae.gate_idx, ae.g = self.idx % 4, g
        r_band = r_near - G.nav_err_m
        corridor = [tg for tg in targets if "FRONT" in tg.pattern
                    and (r_near <= tg.r_min <= r_far or (set(tg.pattern) == {"FRONT"} and r_band <= tg.r_min < r_near))]
        self.phantom = bool(corridor and ae is not None and ae.no_fleet_drone(t, g, p, yaw, [tg.r_min for tg in corridor]))
        rels = []
        for tg in targets:
            if set(tg.pattern) == {"FRONT"} and r_band <= tg.r_min <= r_far:
                continue
            if "FRONT" in tg.pattern:
                if tg.r_min > r_far:
                    continue
            elif tg.r_min > G.queue_bracket_m:
                continue
            rel = mutex_rule.classify_pattern(tg.pattern)
            rels.append({"pattern": "+".join(sorted(tg.pattern)), "range": round(tg.r_min, 2), "relation": rel})
        o.relations = rels
        o.raw_decision = mutex_rule.decide([r["relation"] for r in rels], self.plan.static_rank,
                                           their_ranks=self.bracket_ranks)
        o.rank_used = o.raw_decision == "WAIT_RANK" or (mutex_rule.RANK in [r["relation"] for r in rels])
        if o.rank_used and o.at_queue and not self._rank_prev:
            self.rank_uses += 1
        self._rank_prev = o.rank_used and o.at_queue
        if o.raw_decision == mutex_rule.PRIORITY and o.at_queue:
            self.prio_since = t if self.prio_since is None else self.prio_since
        else:
            self.prio_since = None
        o.decision = o.raw_decision
        o.has_prio = self.prio_since is not None and (t - self.prio_since) >= G.t_clear
        if self.phantom:
            self.phantoms += 1
            self.phantom_events.append({"t": round(t, 2), "fixes": dict(self.acoustic_exit.last_phantom)})
            corridor = []
        claims = self.acoustic_exit.claims(t) if self.acoustic_exit is not None else set()
        heard = self.acoustic_exit is not None and (self.acoustic_exit.heard_inside(t, g) or bool(claims))
        if claims:
            self.acoustic_exit.heard_by |= claims
            o.concede_to = min((j for j in claims if self.queue_ranks[j] < self.queue_ranks[self.plan.slot_index]
                                and self.acoustic_exit.fresh(j, t)
                                and self.acoustic_exit.tracker.status[j].phase == GP_COMMITTED),
                               key=self.queue_ranks.__getitem__, default=None)
        known_inside = self.acoustic_exit is not None and self.acoustic_exit.last_known_inside(t, g)
        if corridor:
            self.last_corridor_t = t
            if self.acoustic_exit is not None:
                self.acoustic_exit.note_corridor(t, g, p, yaw, [tg.r_min for tg in corridor])
            if self.occ_state == "FREE":
                self.busy_since, self.release = t, None
            self.occ_state = "BUSY"
        if heard:
            if self.occ_state == "FREE":
                self.busy_since, self.release = t, None
                self.suspects = set()
            self.suspects |= self.acoustic_exit.heard_by
            self.occ_state, self.exit_witness = "BUSY", None
        witness = None
        if self.occ_state != "FREE":
            healthy = {sec: bool(rd.healthy) for sec, rd in readings.items()}
            if self.last_corridor_t >= self.busy_since:
                witness = self.exit.update(t, g, p, yaw, targets, bool(corridor), self.last_corridor_t, healthy)
            if witness is None and not corridor and self.occ_state in ("BUSY", "STALE") and self.acoustic_exit is not None:
                if self.last_corridor_t >= self.busy_since:
                    witness = self.acoustic_exit.evaluate(t, g)
                else:
                    witness = self.acoustic_exit.evaluate_clearance(t, g, self.busy_since, self.suspects)
        if heard or known_inside:
            witness = None
            self.exit.clear_confirmation()
        o.exit_witness = witness
        if self.occ_state in ("BUSY", "STALE"):
            if witness is not None and not corridor:
                self.occ_state, self.exit_seen_t, self.exit_witness = "EXITING", t, witness
                self.release = "WITNESS"
                self.exit_kind["ACOUSTIC" if witness.get("sector") == "ACOUSTIC" else "SONAR"] += 1
            elif self.occ_state == "BUSY" and stale_release(t, self.last_corridor_t, self.busy_since, bool(corridor),
                                                          heard, known_inside, G.t_occ_max):
                self.occ_state, self.exit_witness, self.release = "STALE", None, "TIMER"
                self.occ_timeouts += 1
        if (self.occ_state == "EXITING" and not corridor and not heard and not known_inside
                and t - max(self.exit_seen_t, self.last_corridor_t) >= G.t_clear):
            self.occ_state = "FREE"
        if self.occ_state == "FREE":
            self.exit.reset()
            self.suspects = set()
            if self.acoustic_exit is not None:
                self.acoustic_exit.reset()
        o.occ_state = self.occ_state
        o.occ_busy = self.occ_state in ("BUSY", "EXITING")
        comm_clear = self.acoustic_exit is not None and bool(self.acoustic_exit.others) and self.acoustic_exit.all_fresh(t)
        o.entry_after = entry_after(self.release, max(self.last_corridor_t, self.last_near_t), G.t_admit_stale, comm_clear)
        o.timed_admission = timed_admission(self.release, comm_clear)
        o.occ_timeouts = self.occ_timeouts
        front = readings.get("FRONT")
        if front is None or not front.healthy:
            self.last_blind_t = t
        quiet = t - max(self.last_corridor_t, self.last_blind_t) > ENTRY_QUIET_CAPTURES / self.cfg.perc.sonar.hz
        o.entry_ok = bool(quiet and not heard and not known_inside)
        o.dead = []
        if self.acoustic_exit is not None:
            ae = self.acoustic_exit
            o.silent = sorted(j for j in ae.others if ae.dead(j, t))
            o.comm_fresh = ae.all_fresh(t)
            if ae.others:
                a = self.cfg.acoustic
                o.admit_wait_s = a.commit_admit_min_s + (0.0 if o.comm_fresh else G.t_cross_max)
        return o
