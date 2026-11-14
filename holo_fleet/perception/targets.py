"""Sector-level obstacle model built from classified echoes (onboard only).

No 3-D position is reconstructed.  For every sector we keep the nearest obstacle echo (DYNAMIC or
UNKNOWN) with its age and a filtered closing rate; echoes of adjacent sectors at similar ranges
are associated into one *target* whose sector pattern P (e.g. FRONT+LEFT) narrows its direction
to ``sonar_geometry.regions()[P]``.

Conservative distance: perception/distance_bound.py lower-bounds the centre distance of every object from the nearest
obstacle echo of each sector, whatever the grouping of the echoes into targets (two drones merged into one target
are bounded like any other scene); a target is given the bound of the candidate touch sets that include one of its
sectors, so the smallest target bound is the bound of the nearest object.  The staleness allowance c_max * age
makes a stale reading only more cautious.  With an unhealthy sector the per-sector bound B is used (no region
knowledge).

Missed detections (EchoHold).  A capture can miss a vehicle that is there (a faded profile); the bound then rests on
the farther echoes and may exceed the true distance for that capture.  The nearest obstacle echo of every healthy
sector is therefore held for ``tau_max`` (the data age the P1 proof budgets) and used, aged with the staleness
allowance, while no capture shows a nearer one: a single missed capture becomes a stale reading.  Consecutive misses
longer than tau_max are outside the declared envelope.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.distance_bound import conservative_bound, sectors_bound, touch_set_bounds
from holo_fleet.perception.sonar_geometry import MIRROR, SECTORS
from holo_fleet.perception.sonar_processing import DYNAMIC, OBSTACLE_CLASSES, SectorReading

ADJACENT = {s: frozenset(x for x in SECTORS if x != s and x != MIRROR[s]) for s in SECTORS}


@dataclass
class SectorTrack:
    r: float
    t: float
    rate: float = 0.0
    cls: str = DYNAMIC


@dataclass
class Target:
    pattern: FrozenSet[str]
    r_min: float
    d_lower: float
    age: float
    closing_rate: float
    cls: str
    ranges: Dict[str, float] = field(default_factory=dict)
    expected_neighbour: bool = False
    possible_neighbour: bool = False
    ages: Dict[str, float] = field(default_factory=dict)


class EchoHold:
    """Nearest obstacle echo of every healthy sector over the last ``hold_s`` (see the module docstring)."""

    def __init__(self, hold_s: float):
        self.hold_s = float(hold_s)
        self.last: Dict[str, Tuple[float, str, float]] = {}

    def update(self, t: float, readings: Dict[str, SectorReading]) -> Dict[str, Tuple[float, str, float]]:
        """Held echoes to add to this step's echoes: sector -> (range, class, age), only where no capture since the
        held one shows a nearer obstacle echo."""
        held = {}
        for s, rd in readings.items():
            if rd is None or not rd.healthy:
                self.last.pop(s, None)
                continue
            obs = [e for e in rd.echoes if e.cls in OBSTACLE_CLASSES]
            near = min(obs, key=lambda e: e.r0) if obs else None
            prev = self.last.get(s)
            if prev is not None and t - prev[2] <= self.hold_s + 1e-9 and (near is None or near.r0 > prev[0]):
                held[s] = (prev[0], prev[1], t - prev[2])
            if near is not None and (prev is None or s not in held):
                self.last[s] = (float(near.r0), near.cls, t - rd.age)
            elif prev is not None and s not in held:
                self.last.pop(s, None)
        return held


class SectorTracker:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.tracks: Dict[str, Optional[SectorTrack]] = {s: None for s in SECTORS}

    def update(self, t: float, readings: Dict[str, SectorReading]) -> Dict[str, Optional[SectorTrack]]:
        pc = self.cfg.perc
        for s in SECTORS:
            rd = readings.get(s)
            tr = self.tracks[s]
            if rd is None or not rd.healthy:
                if tr is not None and t - tr.t > pc.track_drop_s:
                    self.tracks[s] = None
                continue
            near = rd.nearest(OBSTACLE_CLASSES)
            if near is None:
                self.tracks[s] = None
                continue
            stamp = t - rd.age
            if tr is not None and abs(near.r0 - tr.r) < 1.0 and stamp > tr.t + 1e-6:
                raw_rate = (near.r0 - tr.r) / (stamp - tr.t)
                a = pc.closing_rate_alpha
                rate = a * raw_rate + (1 - a) * tr.rate
            else:
                rate = 0.0
            self.tracks[s] = SectorTrack(r=near.r0, t=stamp, rate=float(rate), cls=near.cls)
        return self.tracks


def build_targets(t: float, readings: Dict[str, SectorReading], tracks: Dict[str, Optional[SectorTrack]],
                  cfg: FleetConfig = DEFAULT, assoc_m: float = 0.6, blind_relevant_m: float = 4.0,
                  held: Optional[Dict[str, Tuple[float, str, float]]] = None) -> List[Target]:
    """Associate obstacle echoes of adjacent sectors at similar ranges into targets (``held``: EchoHold echoes)."""
    env = cfg.env
    echoes = []
    for s in SECTORS:
        rd = readings.get(s)
        if rd is None:
            continue
        for e in rd.echoes:
            if e.cls in OBSTACLE_CLASSES:
                echoes.append((s, e.r0, e.cls, rd.age))
        if rd.blind_from is not None and rd.healthy and rd.blind_from <= blind_relevant_m:
            echoes.append((s, rd.blind_from, "UNKNOWN", rd.age))
        if held and s in held:
            echoes.append((s, held[s][0], held[s][1], held[s][2]))
    echoes.sort(key=lambda x: x[1])
    nearest: Dict[str, float] = {}
    ages: Dict[str, float] = {}
    for s, r, _c, age in echoes:
        if s not in nearest:
            nearest[s], ages[s] = r, age
    healthy = all(rd.healthy for rd in readings.values() if rd is not None)
    vc = cfg.perc.v_close_staleness
    bounds = touch_set_bounds(nearest, ages, env.eps_range_far, vc, cfg.perc.sonar) if healthy else {}
    fallback = conservative_bound(nearest, ages, env.eps_range_far, vc, cfg.perc.sonar)
    targets: List[Target] = []
    used = [False] * len(echoes)
    for i, (s, r, cls, age) in enumerate(echoes):
        if used[i]:
            continue
        members = {s: (r, cls, age)}
        used[i] = True
        for j in range(i + 1, len(echoes)):
            s2, r2, c2, a2 = echoes[j]
            if used[j] or s2 in members or abs(r2 - r) > assoc_m:
                continue
            if all(s2 in ADJACENT[m] for m in members):
                members[s2] = (r2, c2, a2)
                used[j] = True
        d_lower = sectors_bound(members, bounds) if healthy else None
        d_lower = fallback if d_lower is None else d_lower
        rates = [tracks[sec].rate for sec in members if tracks.get(sec) is not None]
        closing = max([-x for x in rates] + [0.0])
        targets.append(Target(pattern=frozenset(members), r_min=min(v[0] for v in members.values()),
                              d_lower=float(d_lower), age=max(v[2] for v in members.values()),
                              closing_rate=float(closing),
                              cls=DYNAMIC if any(v[1] == DYNAMIC for v in members.values()) else "UNKNOWN",
                              ranges={sec: v[0] for sec, v in members.items()},
                              ages={sec: v[2] for sec, v in members.items()}))
    return targets
