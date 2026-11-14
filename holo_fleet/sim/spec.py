"""Simulation specs shared by benches and scenarios (no simulator state)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.sim.currents import CurrentField


@dataclass(frozen=True)
class StressSpec:
    beam_dropout: float = 0.0
    blackout_every_s: float = 0.0
    blackout_len_s: float = 0.0
    nav_init_err_m: float = 0.0


@dataclass
class SimSpec:
    name: str
    names: List[str]
    spawn_positions: List[Sequence[float]]
    spawn_yaw_deg: List[float]
    gate_ids: Tuple[str, ...] = ()
    props: Tuple[Dict[str, Any], ...] = ()
    current: CurrentField = field(default_factory=CurrentField)
    stress: StressSpec = field(default_factory=StressSpec)
    release_s: Optional[List[float]] = None
    duration_s: float = 60.0
    seed: int = 0
    camera_drone: Optional[int] = None
    env_box: Optional[Tuple[Sequence[float], Sequence[float]]] = None
    chase_offset: Tuple[float, float, float] = (-5.5, 0.0, 1.6)
    side_offset: Tuple[float, float, float] = (1.0, 8.5, 1.2)
    intruders: Tuple[Dict[str, Any], ...] = ()
    faults: Tuple[Dict[str, Any], ...] = ()
    noise: Optional[Any] = None
    noise_drones: Optional[Any] = None

    def fault_active(self, name: str, t: float) -> bool:
        return any(f["drone"] == name and f["t_on"] <= t and (f.get("t_off") is None or t < f["t_off"])
                   for f in self.faults)

    def dead(self, name: str, t: float) -> bool:
        """The drone suffered a TOTAL_FAILURE by time t: no propulsion, no electronics, no transmission."""
        return any(f["drone"] == name and f["type"] == "TOTAL_FAILURE" and f["t_on"] <= t
                   and (f.get("t_off") is None or t < f["t_off"]) for f in self.faults)

    def __post_init__(self) -> None:
        if self.release_s is None:
            self.release_s = [0.0] * len(self.names)

    @property
    def n(self) -> int:
        return len(self.names)


def bench_spec(name: str, positions: Sequence[Sequence[float]], yaws: Optional[Sequence[float]] = None,
               gate_ids: Sequence[str] = (), props: Sequence[Dict[str, Any]] = (), camera_drone: Optional[int] = None,
               env_box=None, stress: StressSpec = StressSpec()) -> SimSpec:
    names = [f"drone_{k}" for k in range(len(positions))]
    return SimSpec(name=name, names=names, spawn_positions=[list(np.asarray(p, float)) for p in positions],
                   spawn_yaw_deg=list(yaws) if yaws is not None else [0.0] * len(positions),
                   gate_ids=tuple(gate_ids), props=tuple(props), camera_drone=camera_drone, env_box=env_box,
                   stress=stress, duration_s=1e9)
