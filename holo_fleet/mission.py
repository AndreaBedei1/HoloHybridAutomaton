"""Static mission plan: the only a-priori knowledge a drone receives before launch.

A plan contains the survey path, the formation template (relative slot offsets),
the drone's own slot index and launch pose, and the map of fixed structures
(gates taken from the existing marine_race_arena track files).  None of this is
simulator state: it is what an operator uploads to every vehicle before a dive.
"""

from __future__ import annotations

import functools
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
    static_rank: int                  # queue precedence order of the plan (queue_order): last-resort tie-break
    s_end: float                      # along-track arclength at which the survey ends
    failsafe_layer_dz: float = 0.0    # pre-assigned vertical layer used when blind
    formation_enabled: bool = True
    mission_name: str = ""
    template_name: str = ""
    queue_lateral: float = 0.0                # own queue point lateral offset (lateral order of the slots)
    queue_s: float = -4.6                     # queue line (gate frame), from GateRule.queue_s(n)
    queue_dz: float = 0.0                     # own queue point vertical offset (stacked slots only)
    clock: Optional["FormationClock"] = None
    extra: dict = field(default_factory=dict)

    @property
    def my_slot(self) -> Slot:
        return self.slots[self.slot_index]


def queue_assignment(slots: Sequence[Slot], G) -> List[Tuple[float, float]]:
    """Queue point (lateral, dz) of every slot of a template, from the shared plan only (no IDs).

    Columns are abreast, assigned in the slots' lateral order (left first, then front first): with one
    slot per column this is the abreast queue, ``G.queue_spacing`` apart.  Slots that share their
    horizontal position (along, lateral) and differ in depth form a vertical stack: they share a column
    and are stacked ``G.stack_spacing`` apart, top first, and the columns are then
    ``G.stack_column_spacing`` apart.  The precedence rule (ha/mutex_rule.py) orders the queue left
    first, then top first."""
    keys = [(round(sl.along, 2), round(sl.lateral, 2)) for sl in slots]
    columns = sorted(set(keys), key=lambda k: (-k[1], -k[0]))
    stacked = len(columns) < len(slots)
    lats = G.queue_laterals(len(columns), G.stack_column_spacing if stacked else 0.0)
    out: List[Tuple[float, float]] = [(0.0, 0.0)] * len(slots)
    for c, key in enumerate(columns):
        members = sorted([i for i in range(len(slots)) if keys[i] == key], key=lambda i: -slots[i].dz)
        m = len(members)
        for r, i in enumerate(members):
            out[i] = (lats[c], ((m - 1) / 2.0 - r) * G.stack_spacing)
    return out


def queue_order(slots: Sequence[Slot], G) -> List[int]:
    """Static rank of every slot = its position in the queue precedence of the shared plan (left first, then
    top first).  It is the same order the sector rule produces geometrically, so the static-rank fallback
    (ambiguous patterns only, ha/mutex_rule.py) can never contradict the geometric precedence."""
    qa = queue_assignment(slots, G)
    order = sorted(range(len(slots)), key=lambda k: (-qa[k][0], -qa[k][1]))
    rank = [0] * len(slots)
    for r, k in enumerate(order):
        rank[k] = r
    return rank


def _gate_frame_points(half: float, depth: float, k: int = 25) -> np.ndarray:
    """Surface samples of a square gate frame (gate frame: s along the axis, l left, z up)."""
    t = np.linspace(-half, half, k)
    pts = [(x, y, z) for x in (-depth, depth) for y in t for z in (-half, half)] + \
          [(x, y, z) for x in (-depth, depth) for y in (-half, half) for z in t]
    return np.array(pts, dtype=float)


def structure_masked_pairs(points: Sequence[Tuple[float, float]], q_s: float, G, perc,
                           frame_half: float = 0.93, frame_depth: float = 0.11, tol: Optional[float] = None,
                           pairs=None) -> List:
    """Queue neighbours that a queued drone would see inside the gate's structure window.

    For every ordered pair of queue points (lateral, dz) within the queue bracket and every sector whose
    cone contains the neighbour, the classifier explains an echo by the gate map when its range falls in
    [min, max] of the frame points inside that (near-field widened) cone, minus / plus the structure
    tolerances (perception/sonar_processing.py): such a neighbour is classified STRUCTURE, its relation is
    lost and the precedence can change (seen in a probe run of the stacked queue: the diagonal neighbour
    hidden in LEFT/RIGHT, seen only in UP/DOWN).  Checked at the corners of a box of +-``tol`` around both
    queue points (default: the measured holding error, as formal M4c; heading along the gate axis).
    ``pairs`` restricts the check to those ordered pairs.  Returns (i, j, sector, echo, window) tuples."""
    from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, NEAR_FIELD_WIDEN_M

    tol = G.hold_err_m if tol is None else tol
    frame = _gate_frame_points(frame_half, frame_depth)
    half = math.radians(perc.sonar.opening_deg / 2.0)
    corners = [np.array([a, b, c]) * tol for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
    out = []
    for i, (li, zi) in enumerate(points):
        for j, (lj, zj) in enumerate(points):
            if i == j or (pairs is not None and (i, j) not in pairs):
                continue
            pi0, pj0 = np.array([q_s, li, zi]), np.array([q_s, lj, zj])
            if np.linalg.norm(pj0 - pi0) - 0.59 > G.queue_bracket_m:
                continue
            for ci in corners:
                for cj in corners:
                    pi, pj = pi0 + ci, pj0 + cj
                    u = (pj - pi) / np.linalg.norm(pj - pi)
                    for sec, ax in AXES.items():
                        if ax @ u < math.cos(half):
                            continue
                        spos = pi + MOUNTS[sec]
                        d = frame - spos
                        r = np.linalg.norm(d, axis=1)
                        ang = np.arccos(np.clip((d @ ax) / r, -1.0, 1.0))
                        sel = ang <= half + np.arctan2(NEAR_FIELD_WIDEN_M, r)
                        if not sel.any():
                            continue
                        w0, w1 = float(r[sel].min()), float(r[sel].max())
                        echo = float(np.linalg.norm(pj - spos)) - 0.29
                        if w0 - perc.structure_tol_m <= echo <= w1 + perc.structure_tol_far_m:
                            out.append((i, j, sec, round(echo, 2), (round(w0, 2), round(w1, 2))))
    return out


def queue_line(slots: Sequence[Slot], G, perc=None) -> float:
    """Queue line of the shared plan: abreast columns and height of the stacks (GateRule.queue_s).

    A queue with a vertical stack is moved back in 0.1 m steps until no queue neighbour within the bracket
    is hidden by the gate frame (``structure_masked_pairs``, formal M4f): its diagonal pairs are needed in
    the nominal order.  An abreast queue keeps the v2 line: its adjacent pairs are never hidden (M4f), and
    moving it back for the pair around a vacant middle point would put the passing drone's exit beyond
    the range at which the outer queued drones detect a hull (~8.5 m, probe runs: occupancy-latch timeouts)."""
    if perc is None:
        from holo_fleet.config import DEFAULT

        perc = DEFAULT.perc
    return _queue_line(tuple(slots), G, perc)


@functools.lru_cache(maxsize=64)
def _queue_line(slots: Tuple[Slot, ...], G, perc) -> float:
    qa = queue_assignment(slots, G)
    lats = tuple(sorted({round(l, 6) for l, _dz in qa}, reverse=True))
    q = G.queue_s(len(lats), max(abs(dz) for _l, dz in qa), lats=lats)
    if len(lats) < len(slots):
        while structure_masked_pairs(qa, q, G, perc) and q > -20.0:
            q = round(q - 0.1, 6)
    return q


def abreast_slots(n: int) -> List[Slot]:
    """n slots abreast (the abreast queue of n drones)."""
    return [Slot(0.0, ((n - 1) / 2.0 - r) * 3.5, 0.0) for r in range(n)]


def gates_from_arena_track(track_path: str, gate_ids: Optional[Sequence[str]] = None) -> List[GateSpec]:
    """Load gate geometry (and bar boxes) from an existing marine_race_arena track file.

    The arena package is imported from its own repository (re-use, no copy).
    """
    from holo_fleet.arena_bridge import load_arena_gates

    return load_arena_gates(track_path, gate_ids)
