"""Gate-related perception (P2), onboard information only.

* own gate-frame position and heading error from the navigation estimate and the gate map;
* the own queue point (abreast queue line, lateral order of the formation slots: no ID involved);
* the relation to every obstacle target inside the queue bracket, from its SECTOR PATTERN only
  (ha/gate_rule.py), and the resulting decision with a persistence requirement (``t_clear``);
* the critical-region occupancy belief.  From the FRONT sonar position the drone computes the
  smallest and largest range of the CR (gate map + own pose).  A FRONT echo between
  r_near - R_OUT - eps and r_far + eps may come from a drone inside the CR ("corridor"); one
  farther is certainly beyond it; one nearer is a drone between the queue line and the CR (it is
  handled by the WAIT relation).
  A drone in the gate opening is masked by the bars (its echo falls inside the structure window),
  so the belief is a latch: BUSY as soon as an unexplained corridor echo is seen, BUSY while it is
  seen or masked, EXITING when an echo is seen beyond R_far with the corridor empty, FREE after
  ``t_clear`` more - or after ``t_occ_max`` without corridor echoes (logged as a timeout).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha import gate_rule
from holo_fleet.mission import GateSpec, MissionPlan
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT
from holo_fleet.perception.sonar_processing import DYNAMIC


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
    r_far: float = 0.0
    relations: List[Dict] = field(default_factory=list)       # [{pattern, range, relation}]
    raw_decision: str = gate_rule.PRIORITY
    decision: str = gate_rule.WAIT                           # after persistence
    has_prio: bool = False
    occ_state: str = "FREE"                                  # FREE | BUSY | EXITING
    occ_busy: bool = False
    rank_used: bool = False
    occ_timeouts: int = 0


class GatePerception:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT):
        self.plan = plan
        self.cfg = cfg
        self.idx = 0                                          # index of the current gate in plan.gates
        self.prio_since: Optional[float] = None
        self.occ_state = "FREE"
        self.last_corridor_t = -1e9
        self.busy_since = -1e9
        self.exit_seen_t = -1e9
        self.occ_timeouts = 0
        self.rank_uses = 0
        self._rank_prev = False
        self.prev_targets = []

    @property
    def gate(self) -> Optional[GateSpec]:
        return self.plan.gates[self.idx] if self.idx < len(self.plan.gates) else None

    def advance(self) -> None:
        self.idx += 1
        self.prio_since = None
        self.occ_state, self.last_corridor_t, self.exit_seen_t = "FREE", -1e9, -1e9

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
        o.queue_l, o.queue_s = self.plan.queue_lateral, self.plan.queue_s
        o.in_zone = (o.queue_s - G.approach_len) <= s <= G.exit_s and abs(l) <= G.corridor_half_width
        o.passed = s > G.exit_s
        o.at_queue = (abs(s - o.queue_s) <= G.queue_tol and abs(l - o.queue_l) <= G.queue_tol
                      and abs(herr) <= G.heading_tol_deg)
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
        # ---------------------------------------------------------------- queue relations (sector patterns)
        rels = []
        for tg in targets:
            if "FRONT" in tg.pattern:
                if tg.r_min > r_far:
                    continue                  # certainly beyond the CR: a drone that has passed
            elif tg.r_min > G.queue_bracket_m:
                continue
            rel = gate_rule.classify_pattern(tg.pattern)
            rels.append({"pattern": "+".join(sorted(tg.pattern)), "range": round(tg.r_min, 2), "relation": rel})
        o.relations = rels
        o.raw_decision = gate_rule.decide([r["relation"] for r in rels], self.plan.static_rank,
                                          their_ranks=[-1] * sum(r["relation"] == gate_rule.RANK for r in rels))
        o.rank_used = o.raw_decision == "WAIT_RANK" or (gate_rule.RANK in [r["relation"] for r in rels])
        if o.rank_used and o.at_queue and not self._rank_prev:
            self.rank_uses += 1                       # counted once per use (rising edge)
        self._rank_prev = o.rank_used and o.at_queue
        if o.raw_decision == gate_rule.PRIORITY and o.at_queue:
            self.prio_since = t if self.prio_since is None else self.prio_since
        else:
            self.prio_since = None
        o.decision = o.raw_decision
        o.has_prio = self.prio_since is not None and (t - self.prio_since) >= G.t_clear
        # ---------------------------------------------------------------- occupancy belief
        corridor = [tg for tg in targets if "FRONT" in tg.pattern and r_near <= tg.r_min <= r_far]
        beyond = [tg for tg in targets if "FRONT" in tg.pattern and tg.r_min > r_far]
        if corridor:
            self.last_corridor_t = t
            if self.occ_state == "FREE":
                self.busy_since = t
            self.occ_state = "BUSY"
        if self.occ_state == "BUSY":
            if beyond and not corridor:
                self.occ_state, self.exit_seen_t = "EXITING", t
            elif t - self.last_corridor_t > G.t_occ_max:
                self.occ_state = "FREE"
                self.occ_timeouts += 1
        if self.occ_state == "EXITING" and not corridor and t - max(self.exit_seen_t, self.last_corridor_t) >= G.t_clear:
            self.occ_state = "FREE"
        o.occ_state = self.occ_state
        o.occ_busy = self.occ_state != "FREE"
        o.occ_timeouts = self.occ_timeouts
        return o
