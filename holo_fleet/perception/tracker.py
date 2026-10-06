"""Nearest-neighbour tracker of other vehicles in the drone's own navigation frame.

Tracks are anonymous: the drone never learns which teammate a track is.  Track
positions are stored as ``own_estimate + relative_measurement`` at measurement
time; relative quantities are recomputed against the *current* own estimate, so
own motion between sonar frames is accounted for by dead reckoning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.proximity import Detection


@dataclass
class Track:
    tid: int
    p: np.ndarray              # position in own nav frame
    v: np.ndarray              # velocity estimate (nav frame)
    t_last: float
    hits: int = 1
    confidence: float = 0.5
    source: str = "ring"

    def staleness(self, t: float) -> float:
        return max(0.0, t - self.t_last)

    def predicted(self, t: float) -> np.ndarray:
        return self.p + self.v * min(self.staleness(t), 1.0)


class NeighborTracker:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.tracks: List[Track] = []
        self._next = 0

    def update(self, t: float, own_p: np.ndarray, detections: List[Detection]) -> None:
        pc = self.cfg.perc
        used = set()
        for det in sorted(detections, key=lambda d: -d.confidence):
            p_meas = own_p + det.rel
            best, best_d = None, pc.track_gate_m
            for tr in self.tracks:
                if tr.tid in used:
                    continue
                d = float(np.linalg.norm(tr.predicted(t) - p_meas))
                if d < best_d:
                    best, best_d = tr, d
            if best is None:
                self.tracks.append(Track(tid=self._next, p=p_meas.copy(), v=np.zeros(3), t_last=t,
                                         confidence=det.confidence, source=det.source))
                used.add(self._next)
                self._next += 1
            else:
                dt = max(t - best.t_last, 1e-3)
                v_new = (p_meas - best.p) / dt
                best.v = 0.7 * best.v + 0.3 * np.clip(v_new, -1.5, 1.5)
                best.p = p_meas.copy()
                best.t_last = t
                best.hits += 1
                best.confidence = min(1.0, 0.6 * best.confidence + 0.4 * det.confidence + 0.1)
                best.source = det.source
                used.add(best.tid)
        self.tracks = [tr for tr in self.tracks if tr.staleness(t) <= pc.track_drop_s]

    def confirmed(self, t: float) -> List[Track]:
        """Tracks used by the guards (confirmed by >=2 hits or strong confidence)."""
        return [tr for tr in self.tracks if tr.hits >= 2 or tr.confidence >= 0.6]
