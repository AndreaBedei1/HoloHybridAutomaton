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
NEAR_FIELD_WIDEN_M = 0.25                     # near-field beam widening (see sonar_processing, DI-12)


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
    threshold: float = 0.30             # detection threshold: Rayleigh tail 1.5e-8 per bin; 100 % detection (Phase 1)

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


REGION_RADII = (RHO_MIN, 1.1, 1.25, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0, 3.5, 4.2, 5.0, 6.5, 8.0, 12.0)
# A sampled region misses true directions by at most the covering radius of the 12000-point lattice
# (0.023 rad, measured) plus the boundary shift between two consecutive grid radii (<= 0.02 rad):
# -e.u is 1-Lipschitz in u, so every guarantee read from the tables is certified after subtracting this.
SAMPLING_ERR = 0.045


@lru_cache(maxsize=4)
def _region_table_by_radius(n_dirs: int = 12000, model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], Dict[float, np.ndarray]]:
    """Pattern -> {centre distance rho: indices of the unit directions where a target at rho can be seen with it}."""
    dirs = fibonacci_sphere(n_dirs)
    table: Dict[FrozenSet[str], Dict[float, List[int]]] = {}
    for rho in REGION_RADII:
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
                    table.setdefault(P, {}).setdefault(rho, []).append(i)
    return {P: {rho: np.unique(idx) for rho, idx in per.items()} for P, per in table.items()}


@lru_cache(maxsize=4)
def _region_table(n_dirs: int = 12000, model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], np.ndarray]:
    dirs = fibonacci_sphere(n_dirs)
    return {P: dirs[np.unique(np.concatenate(list(per.values())))]
            for P, per in _region_table_by_radius(n_dirs, model).items()}


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


HULL_FUZZ_DEG = 5.0            # cone boundary uncertainty for a hull at >= RHO_MIN (probe: 60 deg, 62.5 at 3 m)


def _parallax_angle(alpha_deg: float, mount: float, D: float = RHO_MIN) -> float:
    """Angle at a sensor mounted ``mount`` ahead of the centre (along its axis) of a point at centre
    distance D whose direction from the centre makes ``alpha_deg`` with the axis."""
    a = np.radians(alpha_deg)
    return float(np.degrees(np.arctan2(D * np.sin(a), D * np.cos(a) - mount)))


@lru_cache(maxsize=4)
def _alpha_table(model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], Dict[str, Dict[float, float]]]:
    """Pattern -> sector -> {rho_grid: largest angle [deg] between the sector axis and a direction of the pattern's
    region at centre distances >= rho_grid}."""
    dirs = fibonacci_sphere(12000)
    out = {}
    for P, per in _region_table_by_radius(12000, model).items():
        out[P] = {}
        for sct in P:
            cos = dirs @ AXES[sct]
            row = {}
            for k, rho in enumerate(REGION_RADII):
                idx = [per[r] for r in REGION_RADII[k:] if r in per]
                if idx:
                    row[rho] = float(np.degrees(np.arccos(np.clip(cos[np.concatenate(idx)].min(), -1.0, 1.0))))
            out[P][sct] = row
    return out


def centre_in_cone(pattern: Iterable[str], sector: str, d_min: float, model: SonarModel = DEFAULT_SONAR) -> bool:
    """True when the target CENTRE is certainly inside ``sector``'s cone as seen from that sensor, for a
    target seen with ``pattern`` at centre distance >= d_min (a sound lower bound)."""
    row = _alpha_table(model).get(frozenset(pattern), {}).get(sector)
    if not row or d_min <= float(np.linalg.norm(MOUNTS[sector])) + 0.05:
        return False
    grid = [r for r in REGION_RADII if r <= d_min and r in row]       # the band that contains d_min
    if not grid:
        return False
    alpha = row[grid[-1]]
    return _parallax_angle(alpha, float(np.linalg.norm(MOUNTS[sector])), d_min) + HULL_FUZZ_DEG <= model.half_angle_deg


def certified_sectors(pattern: Iterable[str], model: SonarModel = DEFAULT_SONAR, d_min: float = RHO_MIN) -> FrozenSet[str]:
    return frozenset(s_ for s_ in frozenset(pattern) if centre_in_cone(pattern, s_, d_min, model))


def target_distance_lower(ranges: Dict[str, float], eps_far: float = 0.12, model: SonarModel = DEFAULT_SONAR) -> float:
    """Sound lower bound of the centre distance of a target seen by the sectors in ``ranges`` (sector ->
    nearest echo range).  Every member sector gives a sound bound (B); where the pattern and that
    bound certify the target centre inside the sector's cone, the tighter bound (A) applies.  The
    minimum over the members is kept: two different objects merged into one target (similar ranges
    in adjacent sectors) are both bounded."""
    P = frozenset(ranges)
    out = []
    for sct, r in ranges.items():
        dB = centre_distance_lower(r, sct, eps_far, model, centre_in_cone=False)
        inside = centre_in_cone(P, sct, max(dB, 0.0), model)
        out.append(centre_distance_lower(r, sct, eps_far, model, centre_in_cone=inside))
    return float(min(out)) if out else 1e9


def centre_distance_lower(r_echo: float, sector: str, eps_far: float = 0.12, model: SonarModel = DEFAULT_SONAR,
                          centre_in_cone: bool = False) -> float:
    """Sound lower bound of the centre-to-centre distance to a BlueROV2 whose nearest echo in ``sector``
    is at range ``r_echo`` (the echo is at most ``eps_far`` beyond the true near surface, so the
    nearest in-cone hull point p is at r' >= r_echo - eps_far from the sensor s = m).

    (B) always: p lies in the cone, so |p|^2 = |m|^2 + r'^2 + 2|m| r' cos(angle(p - m, axis))
        >= |m|^2 + r'^2 + 2|m| r' cos(60 deg + w(r')), w = near-field widening; the target centre is
        within R_OUT of p:  d >= |p| - R_OUT.
    (A) when the pattern certifies that the target CENTRE is inside this sector's cone
        (``certified_sectors``): the inscribed ball's nearest point is in the cone, so the centre
        is at L >= r' + R_IN from the sensor, at an angle phi <= 60 deg + asin(R_OUT / L) from the
        axis (= direction of m):  d^2 = L^2 + |m|^2 + 2 L |m| cos(phi)  ->  d >= L + |m| cos(phi).
    The bound used is the larger of the applicable ones.  Checked on 10^5 random hull poses
    (formal/check_separation.py, S0): never above the true distance.  Without (A) a hull corner
    reaching into the cone while the centre is outside would make r' + R_IN overestimate the
    distance by up to 0.4 m (DI-13).
    """
    m = float(np.linalg.norm(MOUNTS[sector]))
    rp = max(r_echo - eps_far, 0.0)
    w = np.arctan2(NEAR_FIELD_WIDEN_M, max(rp, 1e-6))
    c = max(np.cos(np.radians(model.half_angle_deg) + w), -1.0)
    dB = float(np.sqrt(max(m * m + rp * rp + 2.0 * m * rp * c, 0.0))) - R_OUT
    if not centre_in_cone:
        return dB
    L = rp + R_IN
    phi = np.radians(model.half_angle_deg) + np.arcsin(min(1.0, R_OUT / max(L, 1e-6)))
    return float(max(dB, L + m * max(0.0, np.cos(phi))))
