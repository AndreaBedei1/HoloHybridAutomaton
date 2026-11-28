"""Gate beacon: a fixed acoustic pinger mounted above every gate at a surveyed position (part of the mission map).

Infrastructure, not inter-drone communication: every drone (B0 and B2 alike) receives it; it carries no payload, only
the gate identity.  The receiver measures range, azimuth and elevation in its body frame (sim/gate_beacon.py adds the
noise of config.GateBeaconConfig).  From a reception and the own attitude the drone gets its position relative to the
gate: p = beacon - R_wb unit(az, el) range, independent of the dead-reckoning drift.  ``GateBeaconFix`` keeps the recent
fixes as offsets from the navigation estimate and gives their median when they agree; perception/perception.py compares
the sonar echoes with the gate map at that pose (the bars then stay STRUCTURE under a drift of the estimate).

Error bound of one fix (formal/check_classification.py CL4): 3 sigma_range + range * tan(3 sigma_ang + heading bound).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np


def beacon_position(g, cfg) -> np.ndarray:
    """Surveyed beacon position of gate ``g``: on the gate plane, centred, ``up_m`` above the gate centre."""
    return np.asarray(g.center, float) + np.array([0.0, 0.0, float(cfg.gate_beacon.up_m)])


def unit(az: float, el: float) -> np.ndarray:
    return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])


def fix_bound(cfg, r: float, n: int = 1) -> float:
    """Horizontal error bound of the median of ``n`` fixes at range ``r``: 3 sigma of the median (1.25 sigma / sqrt(n)) of
    range and angle, plus the heading bound (a bias, not averaged).  formal/check_classification.py CL5."""
    b = cfg.gate_beacon
    k = 3.0 * 1.25 / math.sqrt(max(1, int(n)))
    ang = math.radians(k * b.sigma_ang_deg + b.heading_bound_deg)
    return k * b.sigma_range_m + r * math.tan(ang)


class GateBeaconFix:
    """Recent beacon fixes as offsets from the navigation estimate; the median when they agree."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.fixes: Deque[Tuple[float, np.ndarray, float]] = deque(maxlen=16)
        self.receptions = 0

    def add(self, t: float, gate, r: float, az: float, el: float, p_est: np.ndarray, R_wb: np.ndarray) -> None:
        b = self.cfg.gate_beacon
        if not (0.0 < r <= b.max_range_m):
            return
        p_fix = beacon_position(gate, self.cfg) - (R_wb @ unit(az, el)) * r
        off = p_fix - np.asarray(p_est, float)
        off[2] = 0.0
        self.fixes.append((t, off, r))
        self.receptions += 1

    def shift(self, corr: np.ndarray) -> None:
        """The estimate moved by ``corr``: the stored offsets are relative to it."""
        c = np.array([float(corr[0]), float(corr[1]), 0.0])
        self.fixes = deque(((tt, o - c, r) for tt, o, r in self.fixes), maxlen=self.fixes.maxlen)

    def offset(self, t: float) -> Optional[Tuple[np.ndarray, float]]:
        """(offset, bound) from the fixes of the last ``hold_s``, None when too few or when they disagree."""
        b = self.cfg.gate_beacon
        recent = [(o, r) for tt, o, r in self.fixes if t - tt <= b.hold_s]
        if len(recent) < b.min_fixes:
            return None
        offs = np.array([o for o, _ in recent])
        med = np.median(offs, axis=0)
        r_med = float(np.median([r for _, r in recent]))
        bound = fix_bound(self.cfg, r_med, len(recent))
        if float(np.max(np.linalg.norm(offs - med, axis=1))) > 2.0 * fix_bound(self.cfg, r_med):
            return None
        return med, bound
