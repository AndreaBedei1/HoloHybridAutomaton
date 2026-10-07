"""Static mission plan: the only a-priori knowledge a drone receives before launch.

A plan contains the survey path, the formation template (relative slot offsets),
the drone's own slot index and launch pose, and the map of fixed structures
(gates taken from the existing marine_race_arena track files).  None of this is
simulator state: it is what an operator uploads to every vehicle before a dive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class BarBox:
    """Oriented box (gate bar) in world/map coordinates."""

    center: np.ndarray
    axes: np.ndarray          # 3x3, columns = local x,y,z axes in world
    half: np.ndarray          # half dimensions


@dataclass(frozen=True)
class GateSpec:
    gate_id: str
    center: np.ndarray        # world (map) coordinates
    axis: np.ndarray          # unit passage direction (horizontal)
    right: np.ndarray         # unit lateral axis (horizontal, left-handed w.r.t. axis: l>0 is LEFT)
    inner_width: float
    inner_height: float
    bars: Tuple[BarBox, ...] = ()

    def to_gate_frame(self, p: np.ndarray) -> np.ndarray:
        """World point -> (s along axis, l lateral (left positive), dz vertical)."""
        d = np.asarray(p, dtype=float) - self.center
        left = np.array([-self.axis[1], self.axis[0], 0.0])
        return np.array([d @ self.axis, d @ left, d[2]])

    def vec_to_gate_frame(self, v: np.ndarray) -> np.ndarray:
        v = np.asarray(v, dtype=float)
        left = np.array([-self.axis[1], self.axis[0], 0.0])
        return np.array([v @ self.axis, v @ left, v[2]])

    def from_gate_frame(self, g: Sequence[float]) -> np.ndarray:
        left = np.array([-self.axis[1], self.axis[0], 0.0])
        return self.center + g[0] * self.axis + g[1] * left + np.array([0.0, 0.0, g[2]])


@dataclass(frozen=True)
class Slot:
    along: float              # path-frame along-track offset [m]
    lateral: float            # path-frame lateral offset, left positive [m]
    dz: float = 0.0           # depth offset (up positive) [m]


@dataclass
class Path:
    """Planar polyline (survey line) with a constant survey depth."""

    waypoints: np.ndarray     # (N,2)
    depth_z: float

    def __post_init__(self) -> None:
        self.waypoints = np.asarray(self.waypoints, dtype=float)
        seg = np.diff(self.waypoints, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.seg_dir = seg / self.seg_len[:, None]
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])

    def project(self, p_xy: np.ndarray) -> Tuple[float, float, int]:
        """Return (s, lateral (left +), segment index) of the closest path point."""
        best = (1e18, 0.0, 0.0, 0)
        for k in range(len(self.seg_len)):
            a = self.waypoints[k]
            d = self.seg_dir[k]
            t = float(np.clip((p_xy - a) @ d, 0.0, self.seg_len[k]))
            q = a + t * d
            dist = float(np.linalg.norm(p_xy - q))
            if dist < best[0] - 1e-9:
                n = np.array([-d[1], d[0]])
                best = (dist, self.cum[k] + t, float((p_xy - q) @ n), k)
        return best[1], best[2], best[3]

    def frame_at(self, s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Point, tangent, left-normal at arclength s (extrapolated beyond the ends)."""
        k = int(np.clip(np.searchsorted(self.cum, s, side="right") - 1, 0, len(self.seg_len) - 1))
        d = self.seg_dir[k]
        p = self.waypoints[k] + (s - self.cum[k]) * d
        return p, d, np.array([-d[1], d[0]])

    def heading_at(self, s: float) -> float:
        _, d, _ = self.frame_at(s)
        return math.atan2(d[1], d[0])


@dataclass(frozen=True)
class FormationClock:
    """Planned along-track progress of the formation reference, identical on every drone.

    s(t) = s0 + v (t - t_start), holding still for ``dwell`` seconds when it reaches each
    ``hold_s`` and jumping from ``s_from`` to ``s_to`` (rendezvous beyond a gate: the reference
    is re-established beyond the gate while the drones pass it one at a time).  Part of the
    mission plan uploaded before the dive: no communication is involved.
    """

    s0: float
    v: float
    t_start: float = 0.0
    holds: Tuple[Tuple[float, float], ...] = ()      # (hold_s, dwell_s)
    s_end: float = 1e9
    jumps: Tuple[Tuple[float, float], ...] = ()      # (s_from, s_to), s_to > s_from

    def s(self, t: float) -> float:
        tau = max(0.0, t - self.t_start)
        events = sorted([(h, 1, d) for h, d in self.holds] + [(a, 0, b) for a, b in self.jumps])
        s, t_cur = self.s0, 0.0
        for at, kind, par in events:
            if at < s - 1e-9:
                continue                                  # behind the reference (skipped by a jump)
            t_reach = t_cur + (at - s) / max(self.v, 1e-9)
            if tau <= t_reach:
                return min(s + self.v * (tau - t_cur), self.s_end)
            if kind == 1:                                 # hold
                if tau <= t_reach + par:
                    return min(at, self.s_end)
                s, t_cur = at, t_reach + par
            else:                                         # jump
                s, t_cur = par, t_reach
        return min(s + self.v * (tau - t_cur), self.s_end)

    def moving(self, t: float, dt: float = 0.1) -> bool:
        return self.s(t + dt) > self.s(t) + 1e-9


@dataclass
class MissionPlan:
    drone_id: str
    slot_index: int
    slots: List[Slot]
    path: Path
    gates: List[GateSpec]             # ordered list of gates this drone must traverse
    structures: List[GateSpec]        # every mapped structure (for clutter rejection)
    launch_position: np.ndarray       # surveyed launch position (world/map)
    launch_yaw_deg: float
    static_rank: int                  # pre-configured, identical-for-all ID rule (last resort)
    s_end: float                      # along-track arclength at which the survey ends
    failsafe_layer_dz: float = 0.0    # pre-assigned vertical layer used when blind
    formation_enabled: bool = True
    mission_name: str = ""
    template_name: str = ""
    queue_lateral: float = 0.0                # own queue point lateral offset (lateral order of the slots)
    queue_s: float = -4.6                     # queue line (gate frame), from GateRule.queue_s(n)
    clock: Optional["FormationClock"] = None
    extra: dict = field(default_factory=dict)

    @property
    def my_slot(self) -> Slot:
        return self.slots[self.slot_index]


def gates_from_arena_track(track_path: str, gate_ids: Optional[Sequence[str]] = None) -> List[GateSpec]:
    """Load gate geometry (and bar boxes) from an existing marine_race_arena track file.

    The arena package is imported from its own repository (re-use, no copy).
    """
    from holo_fleet.arena_bridge import load_arena_gates

    return load_arena_gates(track_path, gate_ids)
