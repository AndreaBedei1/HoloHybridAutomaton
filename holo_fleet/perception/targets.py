"""Sector-level obstacle model built from classified echoes (onboard only).

No 3-D position is reconstructed.  For every sector we keep the nearest obstacle echo (DYNAMIC or
UNKNOWN) with its age and a filtered closing rate; echoes of adjacent sectors at similar ranges
are associated into one *target* whose sector pattern P (e.g. FRONT+LEFT) narrows its direction
to ``sonar_geometry.regions()[P]``.

Conservative distance: ``sonar_geometry.centre_distance_lower`` lower-bounds the true centre
distance from the echo range, the mounting and the hull geometry; the guards also subtract
c_max * age, so a stale reading can only make the drone more cautious.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import MIRROR, SECTORS, target_distance_lower
from holo_fleet.perception.sonar_processing import DYNAMIC, OBSTACLE_CLASSES, SectorReading

ADJACENT = {s: frozenset(x for x in SECTORS if x != s and x != MIRROR[s]) for s in SECTORS}


@dataclass
class SectorTrack:
    r: float
    t: float
    rate: float = 0.0                 # d r / d t (negative = closing)
    cls: str = DYNAMIC


@dataclass
class Target:
    pattern: FrozenSet[str]
    r_min: float                      # nearest echo range among its sectors [m]
    d_lower: float                    # conservative centre-distance lower bound (incl. staleness) [m]
    age: float
    closing_rate: float               # >= 0 when closing [m/s]
    cls: str                          # DYNAMIC if any member is confirmed, else UNKNOWN
    ranges: Dict[str, float] = field(default_factory=dict)
    expected_neighbour: bool = False  # matched to a formation neighbour expected by the template
    possible_neighbour: bool = False  # in an expected neighbour's sectors within 3 m of its expected distance


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
                  cfg: FleetConfig = DEFAULT, assoc_m: float = 0.6, blind_relevant_m: float = 4.0) -> List[Target]:
    """Associate obstacle echoes of adjacent sectors at similar ranges into targets."""
    env = cfg.env
    echoes = []                       # (sector, range, cls, age)
    for s in SECTORS:
        rd = readings.get(s)
        if rd is None:
            continue
        for e in rd.echoes:
            if e.cls in OBSTACLE_CLASSES:
                echoes.append((s, e.r0, e.cls, rd.age))
        # seabed clutter: beyond blind_from a vehicle cannot be excluded -> conservative UNKNOWN
        if rd.blind_from is not None and rd.healthy and rd.blind_from <= blind_relevant_m:
            echoes.append((s, rd.blind_from, "UNKNOWN", rd.age))
    echoes.sort(key=lambda x: x[1])
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
        d_lower = (target_distance_lower({sec: rr for sec, (rr, _c, _a) in members.items()}, env.eps_range_far,
                                         cfg.perc.sonar)
                   - cfg.perc.v_close_staleness * max(aa for (_r, _c, aa) in members.values()))
        rates = [tracks[sec].rate for sec in members if tracks.get(sec) is not None]
        closing = max([-x for x in rates] + [0.0])
        targets.append(Target(pattern=frozenset(members), r_min=min(v[0] for v in members.values()),
                              d_lower=float(d_lower), age=max(v[2] for v in members.values()),
                              closing_rate=float(closing),
                              cls=DYNAMIC if any(v[1] == DYNAMIC for v in members.values()) else "UNKNOWN",
                              ranges={sec: v[0] for sec, v in members.items()}))
    return targets
