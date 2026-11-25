"""Sensor-driven 3-D avoidance on sector patterns (no bearing inside a cone is assumed).

Every candidate direction e (26 cube directions + a uniform set, body frame) is rated against every
threat by its *guaranteed* opening rate per unit speed: the minimum of -e.u over all directions u
where a target seen with that sector pattern can be (sonar_geometry.regions; unhealthy sectors may
have missed the target, so their patterns are included in the union).

* SEPARATION_WARNING: the velocity closest to the mission velocity whose guaranteed opening is at
  least ``v_open`` for every threat (a 3-D safety filter on cones);
* COLLISION_AVOIDANCE: the direction that maximises the worst-case guaranteed opening over all
  threats, preferring free sectors, the onboard current estimate and the mission direction.

Ties are broken antisymmetrically (two drones meeting head-on pick opposite sides): laterally the
right-hand side, vertically by the own compass heading (heading in [0, 180) deg prefers up).  A
previous choice is kept while it still satisfies the constraints (warning filter) or still opens
every threat (escape) and scores within HYSTERESIS of the best: no chattering between UP and DOWN.

Gate-aware selection (control/gate_safety.py).  A drone that is not admitted to the critical region passes ``p2``: the
P2 margin of a set of body-frame velocities.  Both filters then choose with the same rules inside the P2-safe
candidates only (speeds 0.15 m/s to the cap and holding still for the warning filter; the escape speed, 0.35 and
0.20 m/s and holding still for the escape), and report the joint-feasibility class (JOINTLY_FEASIBLE: the full P1 share
inside the P2-safe set; UNCERTAIN: only a non-closing P2-safe command, or degraded observations; JOINTLY_INFEASIBLE:
none), the counts of the candidates rejected by each property and the fallback taken.  Without ``p2`` the code path is
the one of the P1 proofs.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Callable, Dict, FrozenSet, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control import gate_safety as gs
from holo_fleet.perception.sonar_geometry import SAMPLING_ERR, SECTORS, escape_candidates, far_field_pattern, regions

LATERAL_TIE = 0.06
VERTICAL_TIE = 0.05
HYSTERESIS = 0.10


@dataclass
class EscapeChoice:
    v_body: np.ndarray
    direction: np.ndarray
    guarantee: float
    feasible: bool
    label: str
    joint: Optional[gs.JointDecision] = None


class EscapePlanner:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.cands = escape_candidates()
        self.regions = regions(cfg.perc.sonar)
        self.G: Dict[FrozenSet[str], np.ndarray] = {
            P: (-(R @ self.cands.T)).min(axis=0) - SAMPLING_ERR for P, R in self.regions.items()}
        self.labels = [direction_label(e) for e in self.cands]
        self.last_sw: Optional[int] = None
        self.last_ca: Optional[int] = None

    def reset(self) -> None:
        self.last_sw = self.last_ca = None

    def guarantee(self, pattern: Iterable[str], unhealthy: Iterable[str] = ()) -> np.ndarray:
        """Per-candidate worst-case opening (per unit speed) for a target seen with ``pattern``."""
        P = frozenset(pattern)
        extra = [s for s in unhealthy if s not in P]
        best = None
        for k in range(len(extra) + 1):
            for add in combinations(extra, k):
                g = self.G.get(P | frozenset(add))
                if g is not None:
                    best = g if best is None else np.minimum(best, g)
        return best if best is not None else np.full(len(self.cands), -1.0)

    def _tie_penalty(self, heading_world: float) -> np.ndarray:
        up_pref = 1.0 if (np.degrees(heading_world) % 360.0) < 180.0 else -1.0
        return LATERAL_TIE * self.cands[:, 1] - VERTICAL_TIE * up_pref * self.cands[:, 2]

    def warning_velocity(self, v_des_body: np.ndarray, threats: Sequence[Tuple[FrozenSet[str], float]],
                         unhealthy: Sequence[str], heading_world: float, v_cap: float,
                         p2: Optional[Callable[[np.ndarray], np.ndarray]] = None, degraded: bool = False,
                         hold: Optional[Tuple[np.ndarray, float]] = None) -> EscapeChoice:
        """threats: (pattern, required opening speed [m/s]) for every target inside the warning band.  ``p2``: P2 margins
        of body-frame velocities (gate-aware selection, module docstring); ``degraded``: the observations cannot certify a
        full share (an unhealthy sector in a threat pattern, stale data); ``hold``: (body velocity, P2 margin) of
        station keeping, the HOLD candidate (default: a zero command rated by ``p2``)."""
        if p2 is not None:
            return self._warning_p2(v_des_body, threats, unhealthy, heading_world, v_cap, p2, degraded, hold)
        env = self.cfg.env
        speeds = np.array([0.15, 0.25, 0.35, v_cap])
        g = np.array([self.guarantee(P, unhealthy) for P, _ in threats]) if threats else np.zeros((0, len(self.cands)))
        need = np.array([req for _, req in threats])
        tie = self._tie_penalty(heading_world)
        best = None
        for sp in speeds:
            V = sp * self.cands
            ok = np.all(sp * g >= need[:, None] - 1e-9, axis=0) if len(threats) else np.ones(len(self.cands), bool)
            if not ok.any():
                continue
            cost = np.sum((V - v_des_body) ** 2, axis=1) + tie
            cost[~ok] = np.inf
            i = int(np.argmin(cost))
            if best is None or cost[i] < best[0]:
                best = (cost[i], sp, i)
            if self.last_sw is not None and np.isfinite(cost[self.last_sw]):
                prev = (cost[self.last_sw] - HYSTERESIS, sp, self.last_sw)
                if best is None or prev[0] <= best[0]:
                    best = prev
        if best is not None:
            _, sp, i = best
            self.last_sw = i
            gmin = float(g[:, i].min()) if len(threats) else 1.0
            return EscapeChoice(sp * self.cands[i], self.cands[i], gmin, True, self.labels[i])
        ch = self.escape_velocity(threats, unhealthy, heading_world, env.v_max_nominal)
        return EscapeChoice(ch.v_body, ch.direction, ch.guarantee, False, ch.label)

    def escape_velocity(self, threats: Sequence[Tuple[FrozenSet[str], float]], unhealthy: Sequence[str],
                        heading_world: float, speed: float, current_body: Optional[np.ndarray] = None,
                        mission_body: Optional[np.ndarray] = None, occupied: Iterable[str] = (),
                        depth_room: Tuple[float, float] = (9.0, 9.0),
                        p2: Optional[Callable[[np.ndarray], np.ndarray]] = None, degraded: bool = False,
                        hold: Optional[Tuple[np.ndarray, float]] = None) -> EscapeChoice:
        if p2 is not None:
            return self._escape_p2(threats, unhealthy, heading_world, speed, current_body, mission_body, occupied,
                                   depth_room, p2, degraded, hold)
        if not threats:
            return EscapeChoice(np.zeros(3), np.zeros(3), 1.0, True, "HOLD")
        g = np.array([self.guarantee(P, unhealthy) for P, _ in threats])
        worst = g.min(axis=0)
        score = worst.copy()
        occ = set(occupied)
        if occ:
            into = np.array([bool(far_field_pattern(e) & occ) for e in self.cands])
            score = score - 0.25 * into
        if current_body is not None and np.linalg.norm(current_body) > 0.03:
            c = current_body / max(np.linalg.norm(current_body), 1e-9)
            score = score + 0.08 * (self.cands @ c)
        if mission_body is not None and np.linalg.norm(mission_body) > 1e-3:
            m = mission_body / np.linalg.norm(mission_body)
            score = score + 0.03 * (self.cands @ m)
        down_room, up_room = depth_room
        score = score - 0.5 * ((self.cands[:, 2] > 0.3) & (up_room < 1.0)) - 0.5 * ((self.cands[:, 2] < -0.3) & (down_room < 1.0))
        score = score - self._tie_penalty(heading_world)
        i = int(np.argmax(score))
        j = self.last_ca
        if j is not None and worst[j] > 0 and score[j] >= score[i] - HYSTERESIS:
            i = j
        self.last_ca = i
        return EscapeChoice(speed * self.cands[i], self.cands[i], float(worst[i]), bool(worst[i] > 0), self.labels[i])

    def _scores(self, threats, unhealthy, heading_world, current_body, mission_body, occupied, depth_room):
        """Per-direction worst-case guarantee and the score of escape_velocity (the same preferences and tie-breaks)."""
        g = np.array([self.guarantee(P, unhealthy) for P, _ in threats])
        worst = g.min(axis=0)
        score = worst.copy()
        occ = set(occupied)
        if occ:
            into = np.array([bool(far_field_pattern(e) & occ) for e in self.cands])
            score = score - 0.25 * into
        if current_body is not None and np.linalg.norm(current_body) > 0.03:
            c = current_body / max(np.linalg.norm(current_body), 1e-9)
            score = score + 0.08 * (self.cands @ c)
        if mission_body is not None and np.linalg.norm(mission_body) > 1e-3:
            m = mission_body / np.linalg.norm(mission_body)
            score = score + 0.03 * (self.cands @ m)
        down_room, up_room = depth_room
        score = score - 0.5 * ((self.cands[:, 2] > 0.3) & (up_room < 1.0)) - 0.5 * ((self.cands[:, 2] < -0.3) & (down_room < 1.0))
        return worst, score - self._tie_penalty(heading_world)

    @staticmethod
    def _comfort(mask: np.ndarray, margins: np.ndarray) -> np.ndarray:
        """Among admissible commands, those that keep at least gs.SLACK_M of P2 margin when there are any: the selection
        does not ride the boundary of the P2-safe set (a creep towards the CR at a margin of millimetres)."""
        keep = mask & (margins >= gs.SLACK_M)
        return keep if keep.any() else mask

    @staticmethod
    def _fallback(opening: np.ndarray, margins: np.ndarray, cost: np.ndarray) -> Tuple[int, str, List[str], str]:
        """Index, class, uncertified properties and reason of a P2-restricted decision without a full P1 share
        (gate_safety module docstring: the CR stays a hard constraint while it is enforceable)."""
        safe = margins > 0.0
        non = EscapePlanner._comfort(safe & (opening >= -1e-9), margins)
        safe = EscapePlanner._comfort(safe, margins)
        if non.any():
            key = np.where(non, opening * 1e3 - cost, -np.inf)
            return (int(np.argmax(key)), gs.UNCERTAIN, ["P1"],
                    "no P2-safe command gives the full P1 share: non-closing, P1 rests on the threat opening (S1g)")
        if safe.any():
            key = np.where(safe, opening * 1e3 - cost, -np.inf)
            return (int(np.argmax(key)), gs.INFEASIBLE, ["P1"],
                    "every P2-safe command may close on a threat: P2 kept, P1 best effort")
        non = opening >= -1e-9
        if non.any():
            key = np.where(non, margins, -np.inf)
            return (int(np.argmax(key)), gs.INFEASIBLE, ["P2"],
                    "no P2-safe command (state outside the invariant): largest P2 margin among the non-closing commands")
        return (int(np.argmax(opening)), gs.INFEASIBLE, ["P1", "P2"],
                "no P2-safe and no non-closing command: largest worst-case opening")

    def _grid(self, speeds):
        nc = len(self.cands)
        V = np.concatenate([sp * self.cands for sp in speeds] + [np.zeros((1, 3))])
        lab = [self.labels[i] for _ in speeds for i in range(nc)] + ["HOLD"]
        idx = np.array([i for _ in speeds for i in range(nc)] + [-1])
        sp_of = np.concatenate([np.full(nc, sp) for sp in speeds] + [np.zeros(1)])
        return V, lab, idx, sp_of

    def _hold_opening(self, v_hold: np.ndarray, g_dir: np.ndarray) -> float:
        """Worst-case opening of the HOLD candidate: 0 (non-closing) while its speed does not exceed the declared residual
        drift it cancels (the w_rel of lemma S1g), else that of its direction, like any candidate (``g_dir``: the
        worst-case guarantee per candidate direction over the threats)."""
        sp = float(np.linalg.norm(v_hold))
        if sp <= self.cfg.env.w_drift_max + 1e-9:
            return 0.0
        return sp * float(g_dir[int(np.argmax(self.cands @ (v_hold / sp)))])

    @staticmethod
    def _rated(V: np.ndarray, p2, hold) -> Tuple[np.ndarray, np.ndarray]:
        """Candidates and their P2 margins.  With ``hold`` the last candidate (HOLD) is station keeping: its velocity is
        the position-loop command and its margin the CERTIFIED station-keeping margin (Flows._hold:
        gate_safety.GateConstraint.certified_hold - the margin from the present state or the latched certificate of that
        anchor, so that a held drone keeps a certified candidate at every step, formal P2R4);
        its worst-case opening stays 0, station keeping being the non-closing behaviour of lemma S1g (the residual drift
        is the w_rel of the pairwise model)."""
        V = V.copy()
        if hold is not None:
            V[-1] = np.asarray(hold[0], float)
        margins = np.asarray(p2(V), float).copy()
        if hold is not None:
            margins[-1] = float(hold[1])
        return V, margins

    def _degraded(self, threats, unhealthy) -> bool:
        """An unhealthy sector changes the certified guarantee of a threat (it may have missed part of the threat's
        region: sonar_geometry.regions with the unhealthy sectors added).  Stale data are covered: a sector older than
        tau_max is unhealthy (perception: EchoClassifier.classify with tau_healthy = tau_max)."""
        if not unhealthy:
            return False
        return any(not np.array_equal(self.guarantee(P, unhealthy), self.guarantee(P, ())) for P, _ in threats)

    def _wrap_unbound(self, ch: EscapeChoice, opening: float, p2, degraded, n_cand: int) -> EscapeChoice:
        """Preserve the reference selection when it already provides a full share and satisfies P2."""
        m = float(np.asarray(p2(ch.v_body[None, :]), float)[0])
        dec = gs.JointDecision(n_candidates=n_cand, n_p2_rejected=0, share="FULL", p2_margin=m, opening=opening,
                               reason="the CR removes no candidate: the selection of the P1 proofs")
        dec.status = gs.FEASIBLE
        if degraded:
            dec.status, dec.uncertified = gs.UNCERTAIN, ["P1"]
            dec.reason = "observations degraded (an unhealthy sector changes a threat's guarantee)"
        return EscapeChoice(ch.v_body, ch.direction, ch.guarantee, dec.status == gs.FEASIBLE, ch.label, dec)

    def _warning_p2(self, v_des_body, threats, unhealthy, heading_world, v_cap, p2, degraded, hold=None) -> EscapeChoice:
        speeds = [0.15, 0.25, 0.35, v_cap]
        V, lab, idx, _sp = self._grid(speeds)
        V, margins = self._rated(V, p2, hold)
        degraded = bool(degraded or self._degraded(threats, unhealthy))
        if bool((margins[:-1] > 0.0).all()):
            last_sw, last_ca = self.last_sw, self.last_ca
            ch = self.warning_velocity(v_des_body, threats, unhealthy, heading_world, v_cap)
            if ch.feasible:
                opening = ch.guarantee * float(np.linalg.norm(ch.v_body)) if threats else 1.0
                rated = self._wrap_unbound(ch, opening, p2, degraded, len(V))
                if rated.joint.p2_margin > 0.0:
                    return rated
            self.last_sw, self.last_ca = last_sw, last_ca
        if threats:
            g = np.array([self.guarantee(P, unhealthy) for P, _ in threats])
            need = np.array([req for _, req in threats])
            op = np.concatenate([sp * g for sp in speeds] + [np.zeros((len(threats), 1))], axis=1)
            if hold is not None:
                op[:, -1] = [self._hold_opening(V[-1], g[t]) for t in range(len(threats))]
            full_ok = np.all(op >= need[:, None] - 1e-9, axis=0)
            opening = op.min(axis=0)
        else:
            full_ok, opening = np.ones(len(V), bool), np.ones(len(V))
        safe = margins > 0.0
        tie = np.concatenate([self._tie_penalty(heading_world)] * len(speeds) + [np.zeros(1)])
        cost = np.sum((V - v_des_body) ** 2, axis=1) + tie
        ok = self._comfort(full_ok & safe, margins)
        dec = gs.JointDecision(n_candidates=len(V), n_p2_rejected=int((~safe).sum()),
                               n_p1_rejected=int((~full_ok).sum()), share="FULL")
        if ok.any():
            c = np.where(ok, cost, np.inf)
            k = int(np.argmin(c))
            if self.last_sw is not None:
                prev = np.flatnonzero((idx == self.last_sw) & ok)
                if prev.size:
                    j = int(prev[np.argmin(c[prev])])
                    if c[j] - HYSTERESIS <= c[k]:
                        k = j
            dec.status = gs.FEASIBLE
            if degraded:
                dec.status, dec.uncertified = gs.UNCERTAIN, ["P1"]
                dec.reason = "observations degraded (an unhealthy sector changes a threat's guarantee)"
        else:
            k, dec.status, dec.uncertified, dec.reason = self._fallback(opening, margins, cost)
            dec.share = "NON_CLOSING" if opening[k] >= -1e-9 else "NONE"
        self.last_sw = int(idx[k]) if idx[k] >= 0 else None
        dec.p2_margin, dec.opening = float(margins[k]), float(opening[k])
        d = self.cands[idx[k]] if idx[k] >= 0 else np.zeros(3)
        g_k = float(opening[k]) / max(float(np.linalg.norm(V[k])), 1e-9) if idx[k] >= 0 else 0.0
        return EscapeChoice(V[k].copy(), d, g_k, dec.status == gs.FEASIBLE, lab[k], dec)

    def _escape_p2(self, threats, unhealthy, heading_world, speed, current_body, mission_body, occupied, depth_room,
                   p2, degraded, hold=None) -> EscapeChoice:
        speeds = [speed, 0.35, 0.20]
        V, lab, idx, sp_of = self._grid(speeds)
        V, margins = self._rated(V, p2, hold)
        nc = len(self.cands)
        if not threats:
            # nothing to escape: station keeping if it is P2-safe, else the largest P2 margin (P1 is vacuous)
            k = len(V) - 1 if margins[-1] > 0.0 else int(np.argmax(margins))
            dec = gs.JointDecision(n_candidates=len(V), n_p2_rejected=int((margins <= 0.0).sum()), share="FULL",
                                   p2_margin=float(margins[k]), opening=1.0)
            dec.status = gs.FEASIBLE if margins[k] > 0.0 else gs.INFEASIBLE
            if margins[k] <= 0.0:
                dec.uncertified, dec.reason = ["P2"], "no threat and no P2-safe command (state outside the invariant)"
            self.last_ca = int(idx[k]) if idx[k] >= 0 else None
            return EscapeChoice(V[k].copy(), self.cands[idx[k]] if idx[k] >= 0 else np.zeros(3), 1.0,
                                dec.status == gs.FEASIBLE, lab[k], dec)
        degraded = bool(degraded or self._degraded(threats, unhealthy))
        if bool((margins[:nc] > 0.0).all()):
            last_ca = self.last_ca
            ch = self.escape_velocity(threats, unhealthy, heading_world, speed, current_body, mission_body, occupied,
                                      depth_room)
            opening = ch.guarantee * speed
            if opening >= self.cfg.env.g_min * speed - 1e-9:
                rated = self._wrap_unbound(ch, opening, p2, degraded, len(V))
                if rated.joint.p2_margin > 0.0:
                    return rated
            self.last_ca = last_ca
        worst, score = self._scores(threats, unhealthy, heading_world, current_body, mission_body, occupied, depth_room)
        sc = np.concatenate([score] * len(speeds) + [np.array([float(score.min()) - 1.0])])
        opening = np.concatenate([sp * worst for sp in speeds] + [np.array([self._hold_opening(V[-1], worst)])])
        safe = margins > 0.0
        full_ok = opening >= self.cfg.env.g_min * speed - 1e-9
        dec = gs.JointDecision(n_candidates=len(V), n_p2_rejected=int((~safe).sum()),
                               n_p1_rejected=int((~full_ok).sum()), share="FULL")
        ok = self._comfort(safe & full_ok, margins)
        if ok.any():
            okx = np.flatnonzero(ok)
            k = int(okx[np.argmax(sc[okx] + 1e-6 * sp_of[okx])])
            if self.last_ca is not None:
                prev = np.flatnonzero((idx == self.last_ca) & ok)
                if prev.size:
                    m = int(prev[np.argmax(sp_of[prev])])
                    if sc[m] >= sc[k] - HYSTERESIS:
                        k = m
            dec.status = gs.FEASIBLE
            if degraded:
                dec.status, dec.uncertified = gs.UNCERTAIN, ["P1"]
                dec.reason = "observations degraded (an unhealthy sector changes a threat's guarantee)"
        else:
            k, dec.status, dec.uncertified, dec.reason = self._fallback(opening, margins, -sc)
            dec.share = "NON_CLOSING" if opening[k] >= -1e-9 else "NONE"
        self.last_ca = int(idx[k]) if idx[k] >= 0 else None
        dec.p2_margin, dec.opening = float(margins[k]), float(opening[k])
        d = self.cands[idx[k]] if idx[k] >= 0 else np.zeros(3)
        g_k = float(worst[idx[k]]) if idx[k] >= 0 else 0.0
        return EscapeChoice(V[k].copy(), d, g_k, dec.status == gs.FEASIBLE, lab[k], dec)

def direction_label(e: np.ndarray) -> str:
    """Readable name of a body-frame direction: e.g. 'REAR+UP' (components above 0.35)."""
    names = []
    for k, (pos, neg) in enumerate((("FWD", "REAR"), ("LEFT", "RIGHT"), ("UP", "DOWN"))):
        if e[k] > 0.35:
            names.append(pos)
        elif e[k] < -0.35:
            names.append(neg)
    return "+".join(names) if names else "HOLD"
