"""Ocean-current fields expressed as *effective drift* (m/s of an unactuated BlueROV2).

HoloOcean's ``set_ocean_currents`` argument is converted server-side into a force,
and the steady drift it produces is strongly nonlinear (DISCOVERY.md, probe
``probe_current.py``).  We therefore specify experiments in effective drift and
invert the measured calibration table to obtain the command sent to HoloOcean.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT


def drift_to_command(drift: float, table=DEFAULT.plant.current_cmd_to_drift) -> float:
    """Inverse of the measured (command -> drift) table, linear interpolation, extrapolated."""
    drift = abs(float(drift))
    cmds = [c for c, _ in table]
    drifts = [d for _, d in table]
    if drift <= drifts[-1]:
        return float(np.interp(drift, drifts, cmds))
    # beyond the table: drift ~ cmd^1.6 power law fitted on the last two points
    (c1, d1), (c2, d2) = table[-2], table[-1]
    k = math.log(d2 / d1) / math.log(c2 / c1)
    return float(c2 * (drift / d2) ** (1.0 / k))


@dataclass
class CurrentComponent:
    kind: str                                  # uniform | jet | sinusoid | gusts
    drift: Sequence[float] = (0.0, 0.0, 0.0)   # effective drift vector [m/s]
    center: Sequence[float] = (0.0, 0.0)
    radius: float = 0.0
    t_on: float = -1e9
    t_off: float = 1e9
    amplitude: Sequence[float] = (0.0, 0.0, 0.0)
    period: float = 30.0
    seed: int = 0
    ramp: float = 0.0                          # smooth on/off ramp duration [s] (time-windowed gusts)
    _gust_cache: dict = field(default_factory=dict)

    def _window(self, t: float) -> float:
        if not (self.t_on <= t <= self.t_off):
            return 0.0
        if self.ramp <= 0.0:
            return 1.0
        x = min(t - self.t_on, self.t_off - t) / self.ramp
        return 1.0 if x >= 1.0 else math.sin(0.5 * math.pi * x) ** 2

    def at(self, p: np.ndarray, t: float) -> np.ndarray:
        win = self._window(t)
        if win <= 0.0:
            return np.zeros(3)
        d = np.asarray(self.drift, dtype=float) * win
        if self.kind == "uniform":
            return d
        if self.kind == "jet":
            r = float(np.linalg.norm(np.asarray(p[:2]) - np.asarray(self.center)))
            sigma = max(self.radius / 2.0, 1e-6)
            return d * math.exp(-r * r / (2 * sigma * sigma)) if r <= 1.5 * self.radius else np.zeros(3)
        if self.kind == "sinusoid":
            return d + np.asarray(self.amplitude, dtype=float) * math.sin(2 * math.pi * t / self.period)
        if self.kind == "gusts":
            # piecewise-constant random gusts, 6 s cells, seeded
            cell = int(t // 6.0)
            if cell not in self._gust_cache:
                rng = np.random.default_rng(self.seed * 1000 + cell)
                self._gust_cache[cell] = rng.uniform(-1.0, 1.0, size=3) * np.array([1.0, 1.0, 0.0])
            return np.asarray(self.amplitude, dtype=float) * self._gust_cache[cell]
        raise ValueError(self.kind)


@dataclass
class CurrentField:
    components: List[CurrentComponent] = field(default_factory=list)

    def drift_at(self, p: np.ndarray, t: float) -> np.ndarray:
        w = np.zeros(3)
        for c in self.components:
            w = w + c.at(p, t)
        return w

    def command_at(self, p: np.ndarray, t: float) -> np.ndarray:
        w = self.drift_at(p, t)
        n = float(np.linalg.norm(w))
        if n < 1e-9:
            return np.zeros(3)
        return w / n * drift_to_command(n)

    def describe(self) -> list:
        return [{k: (list(v) if isinstance(v, (tuple, list, np.ndarray)) else v)
                 for k, v in c.__dict__.items() if not k.startswith("_")} for c in self.components]
