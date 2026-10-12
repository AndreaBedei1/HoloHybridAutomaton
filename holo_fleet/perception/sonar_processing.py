"""Echo classification for the six wide-beam single-beam sonars (onboard information only).

An echo is NOT a drone.  Every echo segment of a profile is classified as

* ``STRUCTURE`` - explained by the mission map (arena gates) seen from the own *estimated* pose;
* ``SEABED``    - explained by the seabed seen from the own depth, attitude and DVL altitude;
* ``DYNAMIC``   - unexplained, compact (one hull deep): a possible vehicle;
* ``UNKNOWN``   - anything else that could still be an obstacle: too extended to be one hull, or
                  inside the seabed clutter where a vehicle cannot be told apart from the bottom.
                  The safety layer treats it like DYNAMIC.

M-of-N confirmation: an unexplained echo becomes DYNAMIC/UNKNOWN only once an unexplained echo has
been seen within 0.5 m of it in at least 2 of the last 3 captures; before that it is
``UNCONFIRMED`` (logged, not used by the guards).  Single-capture echoes (noise, grazing returns
of a structure at the cone edge) therefore never trigger a mode change; the confirmation latency
(one capture period) is part of the staleness budget of the P1 derivation.

Inputs: echo profile (234 bins), which sensor produced it, own estimated position/attitude
(dead reckoning), own depth, DVL beam ranges, and the static gate map.  No ground truth.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, NEAR_FIELD_WIDEN_M, SECTORS, SonarModel, fibonacci_sphere

STRUCTURE, SEABED, DYNAMIC, UNKNOWN = "STRUCTURE", "SEABED", "DYNAMIC", "UNKNOWN"
UNCONFIRMED = "UNCONFIRMED"
OBSTACLE_CLASSES = (DYNAMIC, UNKNOWN)


@dataclass
class Echo:
    sector: str
    r0: float                 # range of the first bin above threshold [m]
    r1: float                 # range of the last bin of the segment [m]
    peak: float
    cls: str = UNKNOWN
    why: str = ""

    @property
    def extent(self) -> float:
        return self.r1 - self.r0


@dataclass
class SectorReading:
    sector: str
    age: float                                 # age of the profile [s]
    healthy: bool                              # fresh enough to be trusted (incl. its silence)
    echoes: List[Echo] = field(default_factory=list)
    structure_window: Optional[Tuple[float, float]] = None
    seabed_onset: Optional[float] = None
    # range beyond which this sector is blind to vehicles (seabed clutter): a vehicle there cannot be
    # told apart from the bottom with a range-only sensor, so the safety layer must assume one may be
    blind_from: Optional[float] = None

    def nearest(self, classes: Sequence[str]) -> Optional[Echo]:
        c = [e for e in self.echoes if e.cls in classes]
        return min(c, key=lambda e: e.r0) if c else None


def segment(profile: np.ndarray, model: SonarModel, merge_gap_bins: int = 2) -> List[Tuple[float, float, float]]:
    """Runs of bins above the detection threshold (gaps <= merge_gap_bins merged): (r0, r1, peak)."""
    p = np.asarray(profile, dtype=float).ravel()
    above = np.flatnonzero(p > model.threshold)
    if above.size == 0:
        return []
    ranges = model.ranges()
    out, start, prev = [], above[0], above[0]
    for i in above[1:]:
        if i - prev > merge_gap_bins + 1:
            out.append((float(ranges[start]), float(ranges[prev]), float(p[start:prev + 1].max())))
            start = i
        prev = i
    out.append((float(ranges[start]), float(ranges[prev]), float(p[start:prev + 1].max())))
    return out


def bar_surface_points(bars, spacing: float = 0.04) -> np.ndarray:
    """Points on the surfaces of the mapped gate bars (BarBox: center, axes 3x3, half extents)."""
    pts = []
    for b in bars:
        c, A, h = np.asarray(b.center, float), np.asarray(b.axes, float), np.asarray(b.half, float)
        for ax in range(3):
            u, v = [k for k in range(3) if k != ax]
            nu, nv = max(2, int(2 * h[u] / spacing) + 1), max(2, int(2 * h[v] / spacing) + 1)
            gu, gv = np.meshgrid(np.linspace(-h[u], h[u], nu), np.linspace(-h[v], h[v], nv))
            for sign in (-1.0, 1.0):
                local = np.zeros((gu.size, 3))
                local[:, u], local[:, v], local[:, ax] = gu.ravel(), gv.ravel(), sign * h[ax]
                pts.append(c + local @ A.T)
    return np.concatenate(pts) if pts else np.zeros((0, 3))


class EchoClassifier:
    """Per-drone classifier (keeps the short history needed for confirmation)."""

    def __init__(self, gate_bars=(), cfg: FleetConfig = DEFAULT, cone_samples: int = 700):
        self.cfg = cfg
        self.model = cfg.perc.sonar
        self.struct_pts = bar_surface_points(gate_bars)
        cosh = np.cos(np.radians(self.model.half_angle_deg))
        d = fibonacci_sphere(cone_samples * 4)
        self.cone_dirs = d[d[:, 0] >= cosh]                       # directions inside a cone around +x
        self.history: Dict[str, Deque[List[float]]] = {s: deque(maxlen=3) for s in SECTORS}

    # ------------------------------------------------------------------ predictions
    @staticmethod
    def _sector_frame(sector: str, R_world_body: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """World boresight of the sector and a rotation taking +x to it (for the cone samples)."""
        a = R_world_body @ AXES[sector]
        x = np.array([1.0, 0.0, 0.0])
        v = np.cross(x, a)
        c = float(np.dot(x, a))
        if np.linalg.norm(v) < 1e-9:
            M = np.eye(3) if c > 0 else np.diag([-1.0, -1.0, 1.0])
        else:
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            M = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
        return a, M

    def structure_window(self, sonar_pos: np.ndarray, boresight: np.ndarray) -> Optional[Tuple[float, float]]:
        if len(self.struct_pts) == 0:
            return None
        d = self.struct_pts - sonar_pos
        r = np.linalg.norm(d, axis=1)
        ang = np.arccos(np.clip((d @ boresight) / np.maximum(r, 1e-9), -1.0, 1.0))
        # near-field widening (DI-12): strong reflectors close to the sonar are detected beyond the nominal
        # cone (seen in HoloOcean: a gate post at 0.63 m and 78 deg off-axis; a real wide beam does the
        # same through its side lobes), so the predicted structure window uses a cone widened by
        # atan(NEAR_FIELD_WIDEN_M / r): 22 deg at 0.6 m, 5 deg at 3 m, 2 deg at 7 m
        widen = np.arctan2(NEAR_FIELD_WIDEN_M, np.maximum(r, 1e-6))
        sel = (ang <= np.radians(self.model.half_angle_deg) + widen) & (r >= self.model.range_min) & \
              (r <= self.model.range_max)
        if not sel.any():
            return None
        return float(r[sel].min()), float(r[sel].max())

    def seabed_onset(self, sonar_pos: np.ndarray, M: np.ndarray, seabed_z: Optional[float]) -> Optional[float]:
        """Smallest range at which the cone meets a flat seabed at ``seabed_z`` (None if out of range)."""
        if seabed_z is None:
            return None
        dirs = self.cone_dirs @ M.T
        down = dirs[:, 2] < -1e-6
        if not down.any():
            return None
        r = (sonar_pos[2] - seabed_z) / (-dirs[down, 2])
        r = r[r > 0]
        if r.size == 0:
            return None
        rmin = float(r.min())
        return rmin if rmin <= self.model.range_max else None

    # ------------------------------------------------------------------ classification
    def classify(self, profiles: Dict[str, Optional[np.ndarray]], ages: Dict[str, float], own_pos: np.ndarray,
                 R_world_body: np.ndarray, seabed_z: Optional[float], tau_healthy: float = 0.25) -> Dict[str, SectorReading]:
        pc = self.cfg.perc
        out = {}
        for s in SECTORS:
            prof = profiles.get(s)
            age = float(ages.get(s, 1e9))
            rd = SectorReading(sector=s, age=age, healthy=prof is not None and age <= tau_healthy)
            if prof is None:
                out[s] = rd
                continue
            a, M = self._sector_frame(s, R_world_body)
            spos = own_pos + R_world_body @ MOUNTS[s]
            rd.structure_window = self.structure_window(spos, a)
            rd.seabed_onset = self.seabed_onset(spos, M, seabed_z)
            if rd.seabed_onset is not None:
                rd.blind_from = max(self.model.range_min, rd.seabed_onset - pc.seabed_tol_m)
            segs = segment(prof, self.model, pc.merge_gap_bins)
            seabed_seen = False
            for r0, r1, pk in segs:
                e = Echo(s, r0, r1, pk)
                tol_s, tol_b = pc.structure_tol_m, pc.seabed_tol_m
                sw, sb = rd.structure_window, rd.seabed_onset
                in_struct = sw is not None and (sw[0] - tol_s) <= r0 and r1 <= (sw[1] + pc.structure_tol_far_m)
                if sb is not None and r0 >= sb - tol_b:
                    if r0 <= sb + 0.6 or seabed_seen:
                        e.cls, e.why = SEABED, f"seabed onset {sb:.2f} m"
                        seabed_seen = True
                    else:
                        e.cls, e.why = UNKNOWN, "inside the seabed clutter"
                elif in_struct:
                    e.cls, e.why = STRUCTURE, f"gate map {sw[0]:.2f}-{sw[1]:.2f} m"
                elif e.extent > pc.vehicle_max_extent_m:
                    e.cls, e.why = UNKNOWN, "too extended for one hull"
                else:
                    e.cls, e.why = DYNAMIC, "unexplained compact echo"
                rd.echoes.append(e)
            out[s] = rd
        self._confirm(out)
        return out

    def _confirm(self, readings: Dict[str, SectorReading]) -> None:
        """An unexplained echo (DYNAMIC or UNKNOWN) needs an unexplained echo within 0.5 m in >= confirm_captures
        of the last 3 captures (this one included); otherwise it is UNCONFIRMED."""
        need = self.cfg.perc.confirm_captures
        for s, rd in readings.items():
            if not rd.healthy:
                continue
            cand = [e.r0 for e in rd.echoes if e.cls in OBSTACLE_CLASSES]
            hist = self.history[s]
            hist.append(cand)
            for e in rd.echoes:
                if e.cls not in OBSTACLE_CLASSES:
                    continue
                hits = sum(any(abs(r - e.r0) <= 0.5 for r in past) for past in hist)
                if hits < need:
                    e.cls, e.why = UNCONFIRMED, f"{e.cls.lower()} echo not yet confirmed ({e.why})"


def dvl_altitude(dvl: Optional[np.ndarray], elevation_deg: float = 22.5, max_range: float = 50.0) -> Optional[float]:
    """Altitude from the four DVL beam ranges (None without bottom lock)."""
    if dvl is None:
        return None
    v = np.asarray(dvl, dtype=float).ravel()
    if v.size < 7:
        return None
    r = v[3:7]
    ok = (r > 0.05) & (r < max_range - 0.5)
    if ok.sum() < 3:
        return None
    return float(np.mean(r[ok]) * np.cos(np.radians(elevation_deg)))
