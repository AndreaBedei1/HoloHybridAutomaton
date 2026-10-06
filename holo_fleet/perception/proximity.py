"""Proximity-sonar and forward-looking-sonar processing -> detections of other vehicles.

Pipeline (per sonar frame):
1. ring ranges -> 3D points in the body frame (calibrated beam geometry);
2. body -> local-level map frame using the *estimated* heading/position;
3. returns within ``structure_mask_m`` of a mapped gate bar are labelled as
   structure (map-based clutter rejection) and counted per gate;
4. remaining points are clustered (single linkage); each cluster is a vehicle
   detection whose centre is corrected for the hull radius (rays hit the near
   surface);
5. forward-looking sonar blobs give (range, bearing) detections in the bow
   sector and are fused with ring clusters.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.mission import GateSpec
from holo_fleet.perception.sensor_suite import FLS_GEOMETRY, ring_beam_directions


@dataclass
class Detection:
    t: float
    rel: np.ndarray            # relative position (map-aligned, origin = own estimate) [m]
    range_m: float             # distance to the nearest return (surface) [m]
    bearing_deg: float         # body-frame azimuth, CCW positive (0 = bow)
    elevation_deg: float
    n_points: int
    source: str                # "ring", "fls", "ring+fls"
    confidence: float


@dataclass
class ScanResult:
    detections: List[Detection] = field(default_factory=list)
    structure_hits: Dict[str, int] = field(default_factory=dict)
    structure_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # relative positions
    n_valid: int = 0


class StructureMask:
    """Point-to-oriented-box distances for every mapped gate bar."""

    def __init__(self, structures: Sequence[GateSpec]):
        self.items: List[Tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
        for g in structures:
            for b in g.bars:
                self.items.append((g.gate_id, b.center, b.axes, b.half))

    def classify(self, pts_world: np.ndarray, mask_m: float) -> Tuple[np.ndarray, List[Optional[str]]]:
        n = len(pts_world)
        is_struct = np.zeros(n, dtype=bool)
        owner: List[Optional[str]] = [None] * n
        if n == 0 or not self.items:
            return is_struct, owner
        best = np.full(n, np.inf)
        for gid, c, axes, half in self.items:
            local = (pts_world - c) @ axes          # coordinates in bar frame
            excess = np.maximum(np.abs(local) - half, 0.0)
            d = np.linalg.norm(excess, axis=1)
            closer = d < best
            best = np.where(closer, d, best)
            for idx in np.nonzero(closer & (d < mask_m))[0]:
                owner[idx] = gid
        is_struct = best < mask_m
        return is_struct, owner


def _cluster(points: np.ndarray, link: float) -> List[np.ndarray]:
    n = len(points)
    if n == 0:
        return []
    labels = -np.ones(n, dtype=int)
    cur = 0
    for i in range(n):
        if labels[i] >= 0:
            continue
        stack = [i]
        labels[i] = cur
        while stack:
            j = stack.pop()
            d = np.linalg.norm(points - points[j], axis=1)
            for k in np.nonzero((d < link) & (labels < 0))[0]:
                labels[k] = cur
                stack.append(k)
        cur += 1
    return [np.nonzero(labels == c)[0] for c in range(cur)]


class ProximityProcessor:
    def __init__(self, structures: Sequence[GateSpec], cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.dirs = ring_beam_directions(cfg)
        self.mask = StructureMask(structures)

    def process(self, t: float, rings: Dict[str, np.ndarray], own_p: np.ndarray, R_bw: np.ndarray,
                fls: Optional[np.ndarray] = None) -> ScanResult:
        pc = self.cfg.perc
        pts_body = []
        for name, ranges in rings.items():
            d = self.dirs.get(name)
            if d is None:
                continue
            r = np.asarray(ranges, dtype=float).ravel()
            ok = (r > 0.05) & (r < pc.ring_range_m - 0.05) & np.isfinite(r)
            if ok.any():
                pts_body.append(d[ok] * r[ok, None])
        res = ScanResult()
        if pts_body:
            pb = np.concatenate(pts_body, axis=0)
        else:
            pb = np.zeros((0, 3))
        res.n_valid = len(pb)
        rel = pb @ R_bw.T                         # map-aligned relative positions
        world = rel + own_p
        is_struct, owner = self.mask.classify(world, pc.structure_mask_m)
        for gid in owner:
            if gid is not None:
                res.structure_hits[gid] = res.structure_hits.get(gid, 0) + 1
        res.structure_points = rel[is_struct]
        veh = rel[~is_struct]
        for idx in _cluster(veh, pc.cluster_link_m):
            cpts = veh[idx]
            dist = np.linalg.norm(cpts, axis=1)
            centroid = cpts.mean(axis=0)
            u = centroid / max(np.linalg.norm(centroid), 1e-6)
            centre = centroid + pc.hull_radius_correction_m * u
            b = np.linalg.solve(R_bw, centre)      # back to body for bearing
            res.detections.append(Detection(
                t=t, rel=centre, range_m=float(dist.min()),
                bearing_deg=math.degrees(math.atan2(b[1], b[0])),
                elevation_deg=math.degrees(math.atan2(b[2], math.hypot(b[0], b[1]))),
                n_points=len(idx), source="ring", confidence=min(1.0, 0.35 + 0.15 * len(idx)),
            ))
        if fls is not None:
            self._fuse_fls(t, np.asarray(fls, dtype=float), R_bw, res)
        return res

    def _fuse_fls(self, t: float, img: np.ndarray, R_bw: np.ndarray, res: ScanResult) -> None:
        from scipy import ndimage

        g = FLS_GEOMETRY
        if img.ndim != 2:
            return
        lab, n = ndimage.label(img > self.cfg.perc.fls_threshold)
        for k in range(1, n + 1):
            rr, aa = np.nonzero(lab == k)
            if len(rr) < self.cfg.perc.fls_min_blob_px:
                continue
            rng = g["range_min"] + (rr.min() + 0.5) * (g["range_max"] - g["range_min"]) / g["range_bins"]
            az = -g["azimuth_deg"] / 2 + (aa.mean() + 0.5) * g["azimuth_deg"] / g["azimuth_bins"]
            b = np.array([math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0]) * (
                rng + self.cfg.perc.hull_radius_correction_m)
            rel = R_bw @ b
            fused = False
            for det in res.detections:
                if np.linalg.norm(det.rel[:2] - rel[:2]) < 1.0:
                    det.source = "ring+fls"
                    det.confidence = min(1.0, det.confidence + 0.3)
                    fused = True
                    break
            if not fused:
                # FLS has no elevation resolution: assume co-depth, low confidence.
                res.detections.append(Detection(t=t, rel=rel, range_m=float(rng), bearing_deg=float(az),
                                                elevation_deg=0.0, n_points=len(rr), source="fls",
                                                confidence=0.4))
