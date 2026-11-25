"""What a controller receives from the simulator wrapper: onboard sensor data of one drone."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np


@dataclass
class SensorFrame:
    """Onboard sensor data of ONE drone at onboard time ``t`` (latest value + its age)."""

    t: float
    data: Dict[str, Any]
    stamp: Dict[str, float]

    def fresh(self, name: str, max_age: float) -> bool:
        return self.data.get(name) is not None and -1e-9 <= self.age(name) <= max_age + 1e-9

    def sample(self, name: str, max_age: float, min_values: int):
        """A fresh numeric sample, with past validity time and enough finite components."""
        if not self.fresh(name, max_age):
            return None
        try:
            value = np.asarray(self.data[name], float)
        except (TypeError, ValueError):
            return None
        return value if value.size >= min_values and np.isfinite(value).all() else None

    def age(self, name: str) -> float:
        return self.t - self.stamp.get(name, -1e9)
