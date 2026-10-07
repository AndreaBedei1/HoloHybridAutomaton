"""Geometry of the six identical wide-beam single-beam sonars (no simulator state).

Every drone carries six HoloOcean ``SinglebeamSonar`` (a wide-beam directional single-beam sonar
model: one conical beam, echo intensity per range bin, no bearing inside the cone), one on each
face of the BlueROV2 hull, all with the same model; only position and orientation differ.

Conventions (body frame of the agent origin, HoloOcean client axes): x forward, y left, z up.
Sensor rotations are HoloOcean ``[roll, pitch, yaw]`` degrees; pitch is positive *downward*
(probe/probe_singlebeam.py: an agent pitched -90 deg looks up).

What a sector pattern tells about the direction of a target
-----------------------------------------------------------
A sector "sees" a target when any part of its hull lies inside the cone.  For a BlueROV2 at
centre distance rho >= RHO_MIN in body direction u:

* a sector that sees it  ->  the hull's circumscribed ball (R_OUT) touches the cone (necessary);
* a healthy sector that does not see it -> the hull's inscribed ball (R_IN) does not touch the
  cone (otherwise it would have seen it).

``REGIONS[P]`` is the set of unit directions u (sampled) compatible with exactly the pattern P for
some rho in [RHO_MIN, RANGE_MAX]: a conservative superset of where the target can be.  Escape
directions are rated by their *guaranteed* opening rate, min over the region of -e.u.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, FrozenSet, Iterable, List, Tuple

import numpy as np

SECTORS: Tuple[str, ...] = ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")
AXES: Dict[str, np.ndarray] = {
    "FRONT": np.array([1.0, 0.0, 0.0]), "REAR": np.array([-1.0, 0.0, 0.0]),
    "LEFT": np.array([0.0, 1.0, 0.0]), "RIGHT": np.array([0.0, -1.0, 0.0]),
    "UP": np.array([0.0, 0.0, 1.0]), "DOWN": np.array([0.0, 0.0, -1.0]),
}
MIRROR = {"FRONT": "REAR", "REAR": "FRONT", "LEFT": "RIGHT", "RIGHT": "LEFT", "UP": "DOWN", "DOWN": "UP"}
# BlueROV2 (heavy) hull box half extents [m] around the agent origin; sensors sit 1 cm outside.
HULL_HALF = np.array([0.229, 0.288, 0.127])
MOUNTS: Dict[str, np.ndarray] = {
    "FRONT": np.array([0.24, 0.0, 0.0]), "REAR": np.array([-0.24, 0.0, 0.0]),
    "LEFT": np.array([0.0, 0.30, 0.0]), "RIGHT": np.array([0.0, -0.30, 0.0]),
    "UP": np.array([0.0, 0.0, 0.14]), "DOWN": np.array([0.0, 0.0, -0.14]),
}
ROTATIONS: Dict[str, List[float]] = {
    "FRONT": [0.0, 0.0, 0.0], "REAR": [0.0, 0.0, 180.0], "LEFT": [0.0, 0.0, 90.0],
    "RIGHT": [0.0, 0.0, -90.0], "UP": [0.0, -90.0, 0.0], "DOWN": [0.0, 90.0, 0.0],
}
R_IN = float(HULL_HALF.min())                 # inscribed ball of a BlueROV2 hull
R_OUT = float(np.linalg.norm(HULL_HALF))      # circumscribed ball
RHO_MIN = 1.0                                 # regions are built for centre distances >= this (= d_safe)


@dataclass(frozen=True)
class SonarModel:
    """Identical for the six sensors (only mounting differs)."""

    opening_deg: float = 120.0
    range_min: float = 0.3
    range_max: float = 12.0
    bins: int = 234                     # 5 cm bins
    hz: float = 10.0
    add_sigma: float = 0.05             # Rayleigh, on intensity
    mult_sigma: float = 0.10            # normal, on intensity
    range_sigma: float = 0.05           # exponential, per octree leaf [m]
    threshold: float = 0.25             # detection threshold (0 false alarms / 100 % detection, Phase 1)

    @property
    def half_angle_deg(self) -> float:
        return self.opening_deg / 2.0

    @property
    def bin_width(self) -> float:
        return (self.range_max - self.range_min) / self.bins

    def ranges(self) -> np.ndarray:
        return self.range_min + (np.arange(self.bins) + 0.5) * self.bin_width

    def holoocean_configuration(self, view_region: bool = False) -> dict:
        return {"OpeningAngle": self.opening_deg, "RangeMin": self.range_min, "RangeMax": self.range_max,
                "RangeBins": self.bins, "AddSigma": self.add_sigma, "MultSigma": self.mult_sigma,
                "RangeSigma": self.range_sigma, "ShowWarning": False, "InitOctreeRange": 15,
                "ViewRegion": view_region, "ViewOctree": -10}


DEFAULT_SONAR = SonarModel()


def sonar_sensor_name(sector: str) -> str:
    return f"Sonar{sector.capitalize()}"


def sector_of_sensor(name: str) -> str:
    return name[len("Sonar"):].upper()


def _cos(deg: float) -> float:
    return float(np.cos(np.radians(deg)))


def fibonacci_sphere(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    th = np.pi * (1.0 + 5.0 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)


def ball_touches_cone(points: np.ndarray, sector: str, radius: float, model: SonarModel = DEFAULT_SONAR) -> np.ndarray:
    """Does a ball of ``radius`` centred at each body-frame point intersect the sector's cone (within range)?"""
    d = points - MOUNTS[sector]
    r = np.linalg.norm(d, axis=-1)
    cosang = np.clip((d @ AXES[sector]) / np.maximum(r, 1e-9), -1.0, 1.0)
    ang = np.degrees(np.arccos(cosang))
    margin = np.degrees(np.arcsin(np.clip(radius / np.maximum(r, 1e-9), 0.0, 1.0)))
    return (ang <= model.half_angle_deg + margin) & (r - radius <= model.range_max) & (r + radius >= model.range_min)


def pattern_of_point(p_body: np.ndarray, radius: float = R_IN, model: SonarModel = DEFAULT_SONAR) -> FrozenSet[str]:
    """Sectors whose cone a target ball of ``radius`` at body point ``p_body`` touches."""
    p = np.asarray(p_body, dtype=float)[None, :]
    return frozenset(s for s in SECTORS if bool(ball_touches_cone(p, s, radius, model)[0]))


@lru_cache(maxsize=4)
def _region_table(n_dirs: int = 12000, model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], np.ndarray]:
    dirs = fibonacci_sphere(n_dirs)
    radii = (RHO_MIN, 1.25, 1.6, 2.0, 2.6, 3.5, 5.0, 8.0)
    table: Dict[FrozenSet[str], List[int]] = {}
    for rho in radii:
        pts = rho * dirs
        maybe = {s: ball_touches_cone(pts, s, R_OUT, model) for s in SECTORS}
        must = {s: ball_touches_cone(pts, s, R_IN, model) for s in SECTORS}
        # every pattern P with must <= P <= maybe is possible for these directions
        for i in range(len(dirs)):
            m_set = [s for s in SECTORS if must[s][i]]
            o_set = [s for s in SECTORS if maybe[s][i] and not must[s][i]]
            for k in range(1 << len(o_set)):
                P = frozenset(m_set + [o_set[b] for b in range(len(o_set)) if k >> b & 1])
                if P:
                    table.setdefault(P, []).append(i)
    return {P: dirs[np.unique(idx)] for P, idx in table.items()}


def regions(model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], np.ndarray]:
    """Pattern -> array of unit directions where a target seen with exactly that pattern can be."""
    return _region_table(12000, model)


def far_field_pattern(u: np.ndarray, model: SonarModel = DEFAULT_SONAR) -> FrozenSet[str]:
    c = _cos(model.half_angle_deg)
    return frozenset(s for s in SECTORS if float(np.dot(u, AXES[s])) >= c)


@lru_cache(maxsize=2)
def escape_candidates(n_extra: int = 74) -> np.ndarray:
    """26 cube directions (faces, edges, corners) plus a uniform set: unit vectors, deterministic order."""
    cube = [np.array([x, y, z], float) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
            if (x, y, z) != (0, 0, 0)]
    cube = [v / np.linalg.norm(v) for v in cube]
    extra = list(fibonacci_sphere(n_extra)) if n_extra else []
    return np.array(cube + extra)


def guaranteed_opening(e: np.ndarray, pattern: Iterable[str], model: SonarModel = DEFAULT_SONAR) -> float:
    """Worst-case opening rate per unit speed when moving along ``e`` away from a target seen with ``pattern``."""
    R = regions(model).get(frozenset(pattern))
    if R is None or len(R) == 0:
        return -1.0
    return float(np.min(-(R @ np.asarray(e, float))))


def region_axis(pattern: Iterable[str], model: SonarModel = DEFAULT_SONAR) -> np.ndarray:
    R = regions(model)[frozenset(pattern)]
    m = R.mean(axis=0)
    return m / np.linalg.norm(m)


def coverage(rho: float, radius: float, model: SonarModel = DEFAULT_SONAR, n: int = 20000) -> float:
    """Fraction of directions at centre distance rho where a ball of ``radius`` touches at least one cone."""
    pts = rho * fibonacci_sphere(n)
    hit = np.zeros(len(pts), bool)
    for s in SECTORS:
        hit |= ball_touches_cone(pts, s, radius, model)
    return float(hit.mean())


def diagonal_directions() -> Dict[str, np.ndarray]:
    """The eight cube-corner directions, the worst case of an axis-aligned six-cone suite."""
    out = {}
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                v = np.array([sx, sy, sz], float) / np.sqrt(3.0)
                out[f"({'+' if sx > 0 else '-'},{'+' if sy > 0 else '-'},{'+' if sz > 0 else '-'})"] = v
    return out
