"""Simulator side of the gate beacon (perception/gate_beacon.py): who hears which gate pinger, when, and what is measured.

Every gate pinger transmits at ``rate_hz``; every live drone within ``max_range_m`` receives it with probability
1 - dropout and measures range (Gaussian sigma_range_m), azimuth and elevation (Gaussian sigma_ang_deg) in its body frame
from the simulator pose (never onboard).  A synthetic model, as the B2 channel on top of the engine geometry; no
multipath or occlusion (the pinger is above the gate, clear of the frame).
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from holo_fleet.perception.gate_beacon import beacon_position


class GateBeaconSim:
    def __init__(self, names: Sequence[str], gates, cfg, seed: int):
        self.names = list(names)
        self.gates = list(gates)
        self.cfg = cfg
        self.rng = np.random.default_rng([int(seed), 7933])
        self.next_t = 0.0
        self.stats = {"pings": 0, "receptions": 0, "dropped": 0}

    def measure(self, t: float, poses: Dict[str, np.ndarray]) -> Dict[str, List[Tuple[str, float, float, float, float]]]:
        """``poses[name]`` = (position, R_world_body) of the agent origin; returns the receptions of this step."""
        b = self.cfg.gate_beacon
        out: Dict[str, List[Tuple[str, float, float, float, float]]] = {n: [] for n in self.names}
        if t + 1e-9 < self.next_t:
            return out
        self.next_t = t + 1.0 / b.rate_hz
        for g in self.gates:
            bp = beacon_position(g, self.cfg)
            self.stats["pings"] += 1
            for n, (p, R) in poses.items():
                d = R.T @ (bp - np.asarray(p, float))
                r = float(np.linalg.norm(d))
                if r > b.max_range_m or r < 1e-6:
                    continue
                if self.rng.random() < b.dropout:
                    self.stats["dropped"] += 1
                    continue
                az = math.atan2(d[1], d[0]) + math.radians(self.rng.normal(0.0, b.sigma_ang_deg))
                el = math.atan2(d[2], math.hypot(d[0], d[1])) + math.radians(self.rng.normal(0.0, b.sigma_ang_deg))
                rr = max(0.0, r + float(self.rng.normal(0.0, b.sigma_range_m)))
                out[n].append((g.gate_id, round(t, 4), rr, az, el))
                self.stats["receptions"] += 1
        return out
