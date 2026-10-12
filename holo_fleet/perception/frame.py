"""What a controller receives from the simulator wrapper: onboard sensor data of one drone."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class SensorFrame:
    """Onboard sensor data of ONE drone at onboard time ``t`` (latest value + its age)."""

    t: float
    data: Dict[str, Any]
    stamp: Dict[str, float]             # onboard time of the last update of each sensor

    def fresh(self, name: str, max_age: float) -> bool:
        return name in self.data and (self.t - self.stamp.get(name, -1e9)) <= max_age + 1e-9

    def age(self, name: str) -> float:
        return self.t - self.stamp.get(name, -1e9)
