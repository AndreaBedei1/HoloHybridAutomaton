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
  the FRONT echo beyond R_far is the FRONT case), FREE after ``t_clear`` more - or after ``t_occ_max``
  without corridor echoes (logged as a timeout).  No echo is never evidence of an exit: the timeout is a fallback
  whose safety rests on an explicit assumption, a healthy occupant crosses the CR in less than t_occ_max (formal M3,
  crossing speed >= V_LO); a drone stopped inside the CR by a fault, masked by the bars, is outside that guarantee
  (formal M3f gives the counterexample).  In the gate scenarios the sonar witness from a lateral queue point is rare
  in HoloOcean (the passing drone is lost behind the frame and beyond the detection range), so most releases are this
  fallback (counted as occupancy timeouts in the run metrics).
  B2 adds positive acoustic occupancy to BUSY and vetoes release/admission while a drone is heard possibly inside
  (the complete declared region, perception/acoustic_gate.py). Acoustic exit evidence is evaluated only with the
  corridor empty. Without receptions the sonar outage-mode latch remains.
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
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT
from holo_fleet.perception.sonar_processing import DYNAMIC

ENTRY_QUIET_CAPTURES = 3   # stage 2: captures without a CR-compatible echo (the 3-capture rule of formal M4d)


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
    relations: List[Dict] = field(default_factory=list)       # [{pattern, range, relation}]
    raw_decision: str = mutex_rule.PRIORITY
    decision: str = mutex_rule.WAIT                           # after persistence
    has_prio: bool = False
    occ_state: str = "FREE"                                  # FREE | BUSY | EXITING
    occ_busy: bool = False
    rank_used: bool = False
    occ_timeouts: int = 0
    exit_witness: Optional[Dict] = None                      # {sector, range} of this capture's exit witness
    loc_ok: bool = False                                     # gate localization confirmed (structure_aiding.valid)
    entry_ok: bool = True                                    # stage 2: no hull seen or heard in the CR (update)


class GatePerception:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, acoustic=None):
        self.plan = plan
        self.cfg = cfg
        self.acoustic_exit = None                              # B2 only (perception/acoustic_gate.py)
        if acoustic is not None and acoustic.enabled:
            from holo_fleet.perception.acoustic_gate import AcousticExit

            self.acoustic_exit = AcousticExit(cfg, acoustic, [k for k in range(len(plan.slots)) if k != plan.slot_index])
            G = cfg.gate
            # the shortest time from the queue point to the CR at the speed bound, with the queue tolerance and the declared
            # gate-frame navigation error: a GP_NONE report fresher than that cannot be followed by an unseen entry
            t_min = (abs(plan.queue_s) - (G.cr_half_len + R_OUT) - G.queue_tol - G.nav_err_m) / self.acoustic_exit.v_bound
            self.acoustic_exit.status_fresh_s = max(0.0, min(cfg.acoustic.status_fresh_s, t_min - cfg.acoustic.latency_max_s))
        self.exit_kind = {"SONAR": 0, "ACOUSTIC": 0}           # what moved the latch to EXITING (counts)
        self.idx = 0                                          # index of the current gate in plan.gates
        self.prio_since: Optional[float] = None
        self.occ_state = "FREE"
        self.last_corridor_t = -1e9
        self.suspects: set = set()          # B2: drones heard possibly inside during the current BUSY episode
        self.phantom, self.phantoms = False, 0   # B2: corridor echoes no fleet drone can have produced (count)
        self.phantom_events: list = []
        self.busy_since = -1e9
        self.exit_seen_t = -1e9
        self.occ_timeouts = 0
        self.exit = ExitWitness(cfg)
        self.exit_witness: Optional[Dict] = None              # the witness that moved the latch to EXITING
        self.rank_uses = 0
        self._rank_prev = False
        self.prev_targets = []
        # static-rank fallback (ambiguous patterns only): the queue precedence order of the plan's drones
        # whose queue points lie within the queue bracket of mine - from the shared plan, no identity exchanged
        G = cfg.gate
        qa, order = queue_assignment(plan.slots, G), queue_order(plan.slots, G)
        me = np.array([0.0, plan.queue_lateral, plan.queue_dz])
        self.bracket_ranks = [order[k] for k, (lk, dk) in enumerate(qa) if k != plan.slot_index
                              and float(np.linalg.norm(np.array([0.0, lk, dk]) - me)) <= G.queue_bracket_m]

    @property
    def gate(self) -> Optional[GateSpec]:
        return self.plan.gates[self.idx] if self.idx < len(self.plan.gates) else None

    def advance(self) -> None:
        self.idx += 1
        self.prio_since = None
        self.occ_state, self.last_corridor_t, self.exit_seen_t = "FREE", -1e9, -1e9
        self.exit.reset()
        if self.acoustic_exit is not None:
            self.acoustic_exit.reset()

    def cr_ranges(self, g: GateSpec, p: np.ndarray, yaw: float):
        """Smallest and largest distance from the own FRONT sonar to a point of the CR (gate map + own pose)."""
        G = self.cfg.gate
        sonar = p + np.array([math.cos(yaw), math.sin(yaw), 0.0]) * float(MOUNTS["FRONT"][0])
        half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
        q = g.to_gate_frame(sonar)
        near = float(np.linalg.norm(q - np.clip(q, -half, half)))
        far = float(max(np.linalg.norm(q - half * np.array([a, b, c]))
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
        # the lateral bound always contains the own queue point (for n >= 5 the outer queue points lie
        # beyond corridor_half_width): at_queue -> in_zone holds by construction (invariant I2)
        half_w = max(G.corridor_half_width, abs(o.queue_l) + 1.0)
        o.in_zone = (o.queue_s - G.approach_len) <= s <= G.exit_s and abs(l) <= half_w
        o.passed = s > G.exit_s
        o.at_queue = (abs(s - o.queue_s) <= G.queue_tol and abs(l - o.queue_l) <= G.queue_tol
                      and abs(dz - o.queue_dz) <= G.queue_tol and abs(herr) <= G.heading_tol_deg)
        r_near, r_far = self.cr_ranges(g, p, yaw)
        r_near = r_near - R_OUT - self.cfg.env.eps_range_far       # nearest echo of a hull centred in the CR
        r_far = r_far + self.cfg.env.eps_range_near                 # farther echoes are certainly beyond it
        o.r_far = r_far
        # an echo counts once it persists over two consecutive captures (DYNAMIC is confirmed 2-of-3 by
        # the classifier; an UNKNOWN echo must have been seen at a similar range in the previous update):
        # single-capture noise or grazing echoes of the bars never latch the CR busy
        persistent = [tg for tg in targets if tg.cls == DYNAMIC or any(
            (set(tg.pattern) & set(pp)) and abs(tg.r_min - pr) <= 0.5 for pp, pr in self.prev_targets)]
        self.prev_targets = [(tuple(tg.pattern), tg.r_min) for tg in targets]
        targets = persistent
        ae = self.acoustic_exit
        if ae is not None:
            ae.gate_idx, ae.g = self.idx % 4, g
        corridor = [tg for tg in targets if "FRONT" in tg.pattern and r_near <= tg.r_min <= r_far]
        # B2: an echo that no fleet drone can have produced (every other drone positively elsewhere) is no occupant;
        # under MEDIUM sonar noise such phantom echoes refreshed the latch every few seconds and blocked the queue
        self.phantom = bool(corridor and ae is not None and ae.no_fleet_drone(t, g, p, yaw, [tg.r_min for tg in corridor]))
        # ---------------------------------------------------------------- queue relations (sector patterns)
        rels = []
        for tg in targets:
            # A FRONT-only echo in the corridor band (widened by the declared gate-frame navigation error) is the occupancy
            # latch's business (occ_busy is a guard of the commit), not a queue precedence: drones abreast at their queue
            # points see each other only in LEFT / RIGHT (mutex_rule), and a merged target (FRONT plus another sector)
            # keeps its relation.  With the navigation error at the queue the frame's bar echo is unexplained and fell
            # in this band for ever (the v10 queue deadlock).
            if set(tg.pattern) == {"FRONT"} and r_near - G.nav_err_m <= tg.r_min <= r_far:
                continue
            if "FRONT" in tg.pattern:
                if tg.r_min > r_far:
                    continue                  # certainly beyond the CR: a drone that has passed
            elif tg.r_min > G.queue_bracket_m:
                continue
            rel = mutex_rule.classify_pattern(tg.pattern)
            rels.append({"pattern": "+".join(sorted(tg.pattern)), "range": round(tg.r_min, 2), "relation": rel})
        o.relations = rels
        o.raw_decision = mutex_rule.decide([r["relation"] for r in rels], self.plan.static_rank,
                                           their_ranks=self.bracket_ranks)
        o.rank_used = o.raw_decision == "WAIT_RANK" or (mutex_rule.RANK in [r["relation"] for r in rels])
        if o.rank_used and o.at_queue and not self._rank_prev:
            self.rank_uses += 1                       # counted once per use (rising edge)
        self._rank_prev = o.rank_used and o.at_queue
        if o.raw_decision == mutex_rule.PRIORITY and o.at_queue:
            self.prio_since = t if self.prio_since is None else self.prio_since
        else:
            self.prio_since = None
        o.decision = o.raw_decision
        o.has_prio = self.prio_since is not None and (t - self.prio_since) >= G.t_clear
        # ---------------------------------------------------------------- occupancy belief
        if self.phantom:
            # the echo is no occupant: the latch, the W3 background bookkeeping of the exit witness and the stage-2
            # quiet test all treat this capture as corridor-empty (sound while every other drone is excluded);
            # every such event is logged with the excluding fixes
            self.phantoms += 1
            self.phantom_events.append({"t": round(t, 2), "fixes": dict(self.acoustic_exit.last_phantom)})
            corridor = []
        claims = self.acoustic_exit.claims(t) if self.acoustic_exit is not None else set()   # payload v2: positive occupancy
        heard = self.acoustic_exit is not None and (self.acoustic_exit.heard_inside(t, g) or bool(claims))
        if claims:
            self.acoustic_exit.heard_by |= claims
        # an occupant last known possibly inside and not heard since (shadowed by the bars, a lost ping) is unknown,
        # not gone: no exit witness, no release and no admission while it is (v6 campaign: a MEDIUM clutter echo
        # beyond the exit released the latch over a stalled occupant whose pings were shadowed)
        known_inside = self.acoustic_exit is not None and self.acoustic_exit.last_known_inside(t, g)
        if corridor:
            self.last_corridor_t = t
            if self.acoustic_exit is not None:
                self.acoustic_exit.note_corridor(t, g, p, yaw, [tg.r_min for tg in corridor])
            if self.occ_state == "FREE":
                self.busy_since = t
            self.occ_state = "BUSY"
        if heard:
            if self.occ_state == "FREE":
                self.busy_since = t
                self.suspects = set()
            self.suspects |= self.acoustic_exit.heard_by
            self.occ_state, self.exit_witness = "BUSY", None
        witness = None
        if self.occ_state != "FREE":
            healthy = {sec: bool(rd.healthy) for sec, rd in readings.items()}
            if self.last_corridor_t >= self.busy_since:
                witness = self.exit.update(t, g, p, yaw, targets, bool(corridor), self.last_corridor_t, healthy)
            if witness is None and not corridor and self.occ_state == "BUSY" and self.acoustic_exit is not None:
                if self.last_corridor_t >= self.busy_since:
                    witness = self.acoustic_exit.evaluate(t, g)
                else:
                    witness = self.acoustic_exit.evaluate_clearance(t, g, self.busy_since, self.suspects)
        if heard or known_inside:
            witness = None
            self.exit.clear_confirmation()  # vetoed captures cannot later become an exit witness
        o.exit_witness = witness
        if self.occ_state == "BUSY":
            if witness is not None and not corridor:
                self.occ_state, self.exit_seen_t, self.exit_witness = "EXITING", t, witness
                self.exit_kind["ACOUSTIC" if witness.get("sector") == "ACOUSTIC" else "SONAR"] += 1
            elif (t - max(self.last_corridor_t, self.busy_since) > G.t_occ_max and not heard
                  and self.last_corridor_t >= self.busy_since          # an acoustic-only episode needs clearance
                  and not (self.acoustic_exit is not None and self.acoustic_exit.last_known_inside(t, g))):
                self.occ_state, self.exit_witness = "FREE", None      # fallback: bounded crossing assumed (docstring)
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
        o.occ_busy = self.occ_state != "FREE"
        o.occ_timeouts = self.occ_timeouts
        # stage 2, entry admission (control/flows.py, at the hold point before the CR): from the own pose now, no
        # persistent unexplained FRONT echo compatible with a hull in the CR over the last ENTRY_QUIET_CAPTURES sonar
        # captures, and (B2) no drone heard possibly inside under the declared region. From the hold point only part of the CR
        # is observable apart from the bars (formal/check_stage2.py, M4h): elsewhere hull and bar echoes share
        # ranges and a silent occupant is not seen (a stated limit; the latch argument covers it)
        quiet = t - self.last_corridor_t > ENTRY_QUIET_CAPTURES / self.cfg.perc.sonar.hz
        # an occupant merely unheard (no fresh fix after a fix that placed it possibly inside) also holds the entry
        o.entry_ok = bool(quiet and not heard and not known_inside)
        return o
