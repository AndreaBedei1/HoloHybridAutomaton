"""Association-free conservative distance to the nearest object (P1), from the nearest obstacle echo of each sector.

Why.  A sector reports echo ranges, not which object produced them.  targets.build_targets groups echoes of adjacent
sectors at similar ranges into targets (for the escape planner, the gate and the formation logic), and two different
drones can end up in one group: in a square formation the left and the rear neighbour of a drone (3.5 m, 90 deg apart)
become one LEFT+REAR target.  The safety distance must not depend on that grouping, nor be needlessly pessimistic
because of it.

Model, per healthy sector s with mount m_s and axis a_s, for every physical object O with centre c and distance d = |c|:
 (D) detection: when the hull of O reaches into the detection cone K_s (half-angle 60 deg minus the angular size of one
     octree leaf, apex m_s, range limits), the sector reports an obstacle echo at most eps_far beyond O's nearest point
     in K_s.  Hence every point of every object inside K_s lies at range >= r_s - eps_far (r_s: the sector's nearest
     obstacle echo), and no object reaches into the cone of a sector without obstacle echoes.
 (H) hull: every hull point is within R_OUT of c; the ball of radius R_IN around c lies inside the hull, and so does
     the level cylinder of radius r_c and half-height h_c around c, for any heading and any tilt of the hull relative
     to the observer's body frame up to MAX_REL_TILT_DEG = phi: a point at horizontal radius <= r_c and height <= h_c
     has hull-frame components <= r_c + h_c sin(phi) horizontally and <= r_c sin(phi) + h_c cos(phi) vertically, inside
     the box when those stay within min(half length, half width) and the half height (HULL_CYLINDERS).  BlueROV2-class
     vehicles are passively stable in roll and pitch; the bound on the relative tilt is checked on the logs.
For the touch set T(O) (the sectors whose K_s the hull of O reaches) this gives
 (B) s in T(O):              d >= B_s(r_s)   (a hull point inside K_s, within R_OUT of c; sonar_geometry bound B);
 (A) c inside K_s seen from m_s: d >= A_s(r_s) (the ray m_s -> c lies in K_s and meets the hull before the ball);
 (R) T(O) is a nonempty subset of E (the sectors with obstacle echoes) and the direction of c lies in REGION(T, d),
     the directions in which an object at centre distance d can have exactly the touch set T: the R_IN ball reaches
     no cone outside T, the R_OUT ball reaches every cone of T (maybe-cones: half-angle 60 deg plus one leaf).
Estimate.  For every candidate touch set H (nonempty subset of E with a nonempty region) the bound that holds for
EVERY object whose touch set is H:
     b_lo   = max_{s in H} B_s(r_s)                                   (each member constrains the object),
     raised to A_c for every member c in which the centre is certified for all directions of REGION(H, >= b_lo),
     raised to min_{c in H} A_c(r_c) when every direction of REGION(H, >= b_lo) has the centre in SOME member cone,
minus the staleness allowance v_close * (age of the oldest member reading).  Every object has a touch set, so
    d_hat = min_H beta(H) <= min_O d(O)
for any number of objects and any grouping of their echoes.  No region table exists below RHO_MIN: there beta = b_lo.

The regions are sampled on a spherical lattice; a certification holds only if it holds with the margin gamma(rho):
lattice covering radius + region shift to the next tabulated radius, amplified by the mount parallax.  The claims
(D) and (H) are assumptions (measured: results/reference/ASSUMPTIONS.md, calibration/probe_singlebeam.py); the
inequality d_hat <= d is checked numerically on single- and multi-hull scenes by formal/check_separation.py (S0).
"""

from __future__ import annotations

import hashlib
import math
import os
from functools import lru_cache
from pathlib import Path
from itertools import combinations
from typing import Dict, FrozenSet, Iterable, Mapping, Optional, Tuple

import numpy as np

from holo_fleet.perception.sonar_geometry import (AXES, DEFAULT_SONAR, HULL_HALF, MOUNTS, OCTREE_LEAF_M, R_IN, R_OUT,
                                                  RHO_MIN, SECTORS, SonarModel, centre_distance_lower,
                                                  fibonacci_sphere)

N_DIRS = 48000
COVER_RAD = 0.0145
RADII = tuple(float(x) for x in np.unique(np.round(np.append(RHO_MIN * 1.02 ** np.arange(126), 12.0), 4))
              if x <= 12.0)
MAX_REL_TILT_DEG = 10.0


def _hull_cylinders(phi_deg: float = MAX_REL_TILT_DEG):
    """Level cylinders (radius, half-height) inside the hull box for any heading and relative tilt <= phi: the
    mid-plane disk (largest radius) and a thinner, taller one (its rims reach highest)."""
    a, h = float(min(HULL_HALF[0], HULL_HALF[1])), float(HULL_HALF[2])
    sp, cp = math.sin(math.radians(phi_deg)), math.cos(math.radians(phi_deg))
    disk = (min(a, h / sp if sp > 0 else a), 0.0)
    r = 0.875 * a
    hc = min((h - r * sp) / cp, (a - r) / sp if sp > 0 else h)
    return disk, (r, max(hc, 0.0))


HULL_CYLINDERS = _hull_cylinders()
RIM = np.array([[rc * math.cos(t), rc * math.sin(t), z] for rc, hc in HULL_CYLINDERS for z in sorted({-hc, 0.0, hc})
                for t in np.radians(np.arange(0.0, 360.0, 30.0))])
_BIT = {s: 1 << i for i, s in enumerate(SECTORS)}
_M_MAX = float(max(np.linalg.norm(m) for m in MOUNTS.values()))


def _leaf_deg(r: np.ndarray) -> np.ndarray:
    """Angular size of one octree leaf at range r from a sensor [deg]."""
    return np.degrees(np.arctan2(OCTREE_LEAF_M, np.maximum(r, 1e-6)))


def _touch_bits(pts: np.ndarray, radius: float, half_offset: float, model: SonarModel, rim: bool = False) -> np.ndarray:
    """Bitmask of the sectors whose cone (half-angle 60 deg + half_offset * leaf) a ball of ``radius`` at ``pts``
    reaches, within the range limits; ``rim``: or one of the sampled points of the inscribed disk lies inside it
    (a sufficient condition: only used for the cone the hull certainly reaches)."""
    out = np.zeros(len(pts), np.int64)
    for s in SECTORS:
        v = pts - MOUNTS[s]
        r = np.linalg.norm(v, axis=1)
        ang = np.degrees(np.arccos(np.clip((v @ AXES[s]) / np.maximum(r, 1e-9), -1.0, 1.0)))
        widen = np.degrees(np.arcsin(np.clip(radius / np.maximum(r, 1e-9), 0.0, 1.0)))
        near = np.maximum(r - radius, model.range_min)
        hit = (ang <= model.half_angle_deg + half_offset * _leaf_deg(near) + widen) & \
              (r - radius <= model.range_max) & (r + radius >= model.range_min)
        if rim:
            w = pts[:, None, :] + RIM[None, :, :] - MOUNTS[s]
            rw = np.linalg.norm(w, axis=2)
            aw = np.degrees(np.arccos(np.clip((w @ AXES[s]) / np.maximum(rw, 1e-9), -1.0, 1.0)))
            inside = (aw <= model.half_angle_deg + half_offset * _leaf_deg(np.maximum(rw, model.range_min))) & \
                     (rw <= model.range_max) & (rw >= model.range_min)
            hit |= inside.any(axis=1)
        out |= np.where(hit, _BIT[s], 0)
    return out


def _cache_dir() -> Path:
    return Path(os.environ.get("HOLO_FLEET_CACHE", Path.home() / ".cache" / "holo_fleet"))


@lru_cache(maxsize=2)
def _lattice(model: SonarModel = DEFAULT_SONAR):
    """Directions and, per tabulated radius, the must / maybe touch masks of a centre at that distance (cached on
    disk under a key of every geometric input: the masks are a pure function of them)."""
    dirs = fibonacci_sphere(N_DIRS)
    key = hashlib.sha256(repr((N_DIRS, RADII, R_IN, R_OUT, OCTREE_LEAF_M, RIM.round(6).tolist(), model, sorted((k, tuple(v)) for k, v in MOUNTS.items()),
                               sorted((k, tuple(v)) for k, v in AXES.items()))).encode()).hexdigest()[:16]
    f = _cache_dir() / f"touch_lattice_{key}.npz"
    try:
        z = np.load(f)
        return dirs, z["must"], z["maybe"]
    except (OSError, ValueError, KeyError):
        pass
    must = np.array([_touch_bits(rho * dirs, R_IN, -1.0, model, rim=True) for rho in RADII], np.uint8)
    maybe = np.array([_touch_bits(rho * dirs, R_OUT, +1.0, model) for rho in RADII], np.uint8)
    try:
        f.parent.mkdir(parents=True, exist_ok=True)
        tmp = f.with_suffix(f".{os.getpid()}.tmp.npz")
        np.savez_compressed(tmp, must=must, maybe=maybe)
        os.replace(tmp, f)
    except OSError:
        pass
    return dirs, must, maybe


def _gamma_deg(k: int) -> float:
    """Certification margin at tabulated radius k: covering radius plus the shift of the region boundaries to the
    next radius (ball angular radii, leaf angle), amplified by the parallax of the farthest mount."""
    rho = RADII[k]
    nxt = RADII[k + 1] if k + 1 < len(RADII) else rho
    shift = max(math.asin(min(1.0, R / rho)) - math.asin(min(1.0, R / nxt)) for R in (R_IN, R_OUT))
    shift += math.atan2(OCTREE_LEAF_M, rho - R_OUT - _M_MAX) - math.atan2(OCTREE_LEAF_M, nxt - R_OUT - _M_MAX)
    amp = rho / (rho - _M_MAX)
    return math.degrees((COVER_RAD + shift) * amp)


@lru_cache(maxsize=256)
def _pattern_tables(pattern: FrozenSet[str], model: SonarModel = DEFAULT_SONAR):
    """For a touch set: per tabulated radius index k, whether its region is nonempty at some radius >= RADII[k], the
    members certified to contain the centre for every direction at every radius >= RADII[k], and whether every such
    direction has the centre in at least one member cone."""
    dirs, must, maybe = _lattice(model)
    P = np.uint8(sum(_BIT[s] for s in pattern))
    notP = np.uint8(0xFF ^ int(P))
    members = sorted(pattern)
    K = len(RADII)
    exists, cov = np.zeros(K, bool), np.ones(K, bool)
    cert = {c: np.ones(K, bool) for c in members}
    for k in range(K):
        idx = np.flatnonzero(((must[k] & notP) == 0) & ((P & ~maybe[k]) == 0))
        if not idx.size:
            continue
        exists[k] = True
        c_pts = RADII[k] * dirs[idx]
        g = _gamma_deg(k)
        slack = []
        for c in members:
            v = c_pts - MOUNTS[c]
            r = np.linalg.norm(v, axis=1)
            ang = np.degrees(np.arccos(np.clip((v @ AXES[c]) / r, -1.0, 1.0)))
            lim = model.half_angle_deg - _leaf_deg(np.maximum(r - R_OUT, model.range_min))
            sl = ang + g - lim
            cert[c][k] = bool(np.all(sl <= 0.0))
            slack.append(sl)
        cov[k] = bool(np.all(np.min(np.array(slack), axis=0) <= 0.0))
    ex_from = np.logical_or.accumulate(exists[::-1])[::-1]
    cov_from = np.logical_and.accumulate(np.where(exists, cov, True)[::-1])[::-1]
    cert_from = {c: np.logical_and.accumulate(np.where(exists, cert[c], True)[::-1])[::-1] for c in members}
    return ex_from, cov_from, cert_from


def _band(d: float) -> int:
    """Largest tabulated radius index with RADII[k] <= d (d >= RHO_MIN)."""
    return max(0, min(len(RADII) - 1, int(np.searchsorted(RADII, d, side="right")) - 1))


def _bound_in_cone(r_echo: float, sector: str, eps_far: float, model: SonarModel) -> float:
    """Bound (B) for an object that reaches into the detection cone K_s: it has a hull point inside K_s (half-angle
    60 deg minus one leaf at that range) at range >= r_echo - eps_far, within R_OUT of its centre."""
    m = float(np.linalg.norm(MOUNTS[sector]))
    rp = max(r_echo - eps_far, 0.0)
    half = math.radians(model.half_angle_deg) - math.atan2(OCTREE_LEAF_M, max(rp, model.range_min))
    return math.sqrt(max(m * m + rp * rp + 2.0 * m * rp * math.cos(half), 0.0)) - R_OUT


def touch_set_bound(H: FrozenSet[str], ranges: Mapping[str, float], eps_far: float,
                    model: SonarModel = DEFAULT_SONAR) -> Optional[float]:
    """Lower bound of the centre distance of any object whose touch set is exactly ``H`` (None: no such object can
    exist at the distances the members allow)."""
    B = {s: _bound_in_cone(ranges[s], s, eps_far, model) for s in H}
    A = {s: centre_distance_lower(ranges[s], s, eps_far, model, centre_in_cone=True) for s in H}
    b = max(B.values())
    if b < RHO_MIN:
        return b
    for _ in range(2):
        ex_from, cov_from, cert_from = _pattern_tables(H, model)
        k = _band(b)
        if not ex_from[k]:
            return None
        nb = max([b] + [A[c] for c in H if cert_from[c][k]])
        if cov_from[k]:
            nb = max(nb, min(A.values()))
        if nb <= b + 1e-9:
            break
        b = nb
    return float(b)


def touch_set_bounds(ranges: Mapping[str, float], ages: Mapping[str, float], eps_far: float, v_close: float,
                     model: SonarModel = DEFAULT_SONAR) -> Dict[FrozenSet[str], float]:
    """beta(H) - staleness for every candidate touch set H of the sectors with obstacle echoes (``ranges``: nearest
    obstacle echo per sector, healthy sectors only)."""
    E = sorted(ranges)
    out: Dict[FrozenSet[str], float] = {}
    for n in range(1, len(E) + 1):
        for H in combinations(E, n):
            H = frozenset(H)
            if any(MIRRORED(a, b) for a, b in combinations(H, 2)):
                continue
            v = touch_set_bound(H, ranges, eps_far, model)
            if v is not None:
                out[H] = v - v_close * max(ages[s] for s in H)
    return out


def MIRRORED(a: str, b: str) -> bool:
    return bool(np.dot(AXES[a], AXES[b]) < -0.5)


def conservative_bound(ranges: Mapping[str, float], ages: Mapping[str, float], eps_far: float, v_close: float,
                       model: SonarModel = DEFAULT_SONAR) -> float:
    """The B-only bound over every sector with echoes: sound without any region knowledge (unhealthy sectors)."""
    return min((centre_distance_lower(r, s, eps_far, model, centre_in_cone=False) - v_close * ages[s]
                for s, r in ranges.items()), default=1e9)


def nearest_distance_lower(ranges: Mapping[str, float], ages: Mapping[str, float], eps_far: float, v_close: float,
                           model: SonarModel = DEFAULT_SONAR) -> float:
    """d_hat: min over the candidate touch sets (1e9 without echoes)."""
    b = touch_set_bounds(ranges, ages, eps_far, v_close, model)
    return min(b.values(), default=1e9 if not ranges else conservative_bound(ranges, ages, eps_far, v_close, model))


def sectors_bound(pattern: Iterable[str], bounds: Mapping[FrozenSet[str], float]) -> Optional[float]:
    """Bound attributed to a target seen by ``pattern``: the candidate touch sets that include one of its sectors."""
    P = frozenset(pattern)
    vals = [v for H, v in bounds.items() if H & P]
    return min(vals) if vals else None
