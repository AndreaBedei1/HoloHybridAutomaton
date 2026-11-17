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

Gate guard (``cap`` = (a, u_max), a a unit body-frame vector, flows.gate_guard): a drone near a critical region
that is not admitted to it may only use velocities V with V.a <= u_max.  The warning filter and the escape choose
among those; when none of them opens every threat the warning filter holds (no closing of its
own, moving back along a only when that closes on no threat: guard_hold) and the escape overrides the guard
(collision avoidance first), both reported in ``EscapeChoice.guard``.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
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
    guard: str = ""


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

    def _allowed(self, cap, speed: float) -> np.ndarray:
        if cap is None:
            return np.ones(len(self.cands), bool)
        a, u_max = cap
        return speed * (self.cands @ np.asarray(a, float)) <= float(u_max) + 1e-9

    def opening(self, e: np.ndarray, threats: Sequence[Tuple[FrozenSet[str], float]], unhealthy: Sequence[str]) -> float:
        """Worst-case opening per unit speed of the body direction ``e`` over every threat (as ``guarantee``)."""
        e = np.asarray(e, float)
        worst = np.inf
        for P, _ in threats:
            P = frozenset(P)
            extra = [s for s in unhealthy if s not in P]
            for k in range(len(extra) + 1):
                for add in combinations(extra, k):
                    R = self.regions.get(P | frozenset(add))
                    if R is not None:
                        worst = min(worst, float((-(R @ e)).min()) - SAMPLING_ERR)
        return worst if np.isfinite(worst) else -1.0

    def guard_hold(self, cap, threats, unhealthy) -> EscapeChoice:
        """No allowed velocity opens every threat (warning filter): hold.  Beyond the guard's bound (u_max < 0) the
        drone moves back at -u_max along a only if that direction closes on no threat; else it holds still (its own
        closing component is zero, the other drone of the pair opens: formal/check_gate_guard.py GG2)."""
        a, u_max = np.asarray(cap[0], float), float(cap[1])
        if u_max < 0.0 and self.opening(-a, threats, unhealthy) >= 0.0:
            return EscapeChoice(u_max * a, -a, -1.0, False, "GUARD_RETREAT", "HOLD")
        return EscapeChoice(np.zeros(3), np.zeros(3), -1.0, False, "GUARD_HOLD", "HOLD")

    def warning_velocity(self, v_des_body: np.ndarray, threats: Sequence[Tuple[FrozenSet[str], float]],
                         unhealthy: Sequence[str], heading_world: float, v_cap: float, cap=None) -> EscapeChoice:
        """threats: (pattern, required opening speed [m/s]) for every target inside the warning band; ``cap``: the
        gate guard (module docstring)."""
        env = self.cfg.env
        speeds = np.array([0.15, 0.25, 0.35, v_cap])
        g = np.array([self.guarantee(P, unhealthy) for P, _ in threats]) if threats else np.zeros((0, len(self.cands)))
        need = np.array([req for _, req in threats])
        tie = self._tie_penalty(heading_world)
        best = None
        for sp in speeds:
            V = sp * self.cands
            ok = np.all(sp * g >= need[:, None] - 1e-9, axis=0) if len(threats) else np.ones(len(self.cands), bool)
            ok &= self._allowed(cap, sp)
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
            return EscapeChoice(sp * self.cands[i], self.cands[i], gmin, True, self.labels[i],
                                "" if cap is None else "ACTIVE")
        ch = self.escape_velocity(threats, unhealthy, heading_world, env.v_max_nominal, cap=cap, override=False)
        return EscapeChoice(ch.v_body, ch.direction, ch.guarantee, False, ch.label, ch.guard)

    def escape_velocity(self, threats: Sequence[Tuple[FrozenSet[str], float]], unhealthy: Sequence[str],
                        heading_world: float, speed: float, current_body: Optional[np.ndarray] = None,
                        mission_body: Optional[np.ndarray] = None, occupied: Iterable[str] = (),
                        depth_room: Tuple[float, float] = (9.0, 9.0), cap=None, override: bool = True) -> EscapeChoice:
        """``cap``: the gate guard; with no allowed direction that opens every threat, ``override`` (the escape)
        drops the guard, else (the warning filter) the guard hold is returned."""
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
        guard = ""
        if cap is not None:
            allowed = self._allowed(cap, speed)
            if (allowed & (worst > 0)).any():
                score = np.where(allowed, score, -np.inf)
                guard = "ACTIVE"
            elif override:
                guard = "OVERRIDE"
            else:
                return self.guard_hold(cap, threats, unhealthy)
        i = int(np.argmax(score))
        j = self.last_ca
        if j is not None and worst[j] > 0 and np.isfinite(score[j]) and score[j] >= score[i] - HYSTERESIS:
            i = j
        self.last_ca = i
        return EscapeChoice(speed * self.cands[i], self.cands[i], float(worst[i]), bool(worst[i] > 0), self.labels[i],
                            guard)


def direction_label(e: np.ndarray) -> str:
    """Readable name of a body-frame direction: e.g. 'REAR+UP' (components above 0.35)."""
    names = []
    for k, (pos, neg) in enumerate((("FWD", "REAR"), ("LEFT", "RIGHT"), ("UP", "DOWN"))):
        if e[k] > 0.35:
            names.append(pos)
        elif e[k] < -0.35:
            names.append(neg)
    return "+".join(names) if names else "HOLD"
