"""Bridge to the existing Marine Race Arena project (re-used, not copied).

The arena lives in ``~/Desktop/HoloDroneCompetition`` (override with the
``MARINE_RACE_ARENA_ROOT`` environment variable).  We reuse its track files, its
config loader, its gate geometry/visual factory and its HoloOcean prop spawner.
Only static map geometry crosses this bridge; no simulator state does.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

DEFAULT_ARENA_ROOT = Path.home() / "Desktop" / "HoloDroneCompetition"
HORSESHOE_TRACK = "marine_race_arena/tracks/marine_race_horseshoe_bay.json"
# Inner opening (width, height) [m] of the gates the scenarios spawn, overriding the track (1.5 x 1.5 m).  G06 is
# 2.0 x 2.0 m: the free lateral margin of the hull centre grows from 0.44 to 0.69 m, above the 0.30-0.58 m lateral
# dead-reckoning error measured at the gate under MEDIUM (raw_gate_check), the vertical one from 0.62 to 0.87 m, and
# the opening still ends at the edges of the critical region (cr_half_width = cr_half_height = 1.0 m), so the mutex
# geometry is unchanged.  The perception map and the spawned bars are built from the same overridden config.
GATE_INNER_SIZE_M = {"G06": (2.0, 2.0)}
# Bar section (thickness across the bar, depth along the passage axis) [m] of the gates the scenarios spawn,
# overriding the track (0.18 x 0.22 m).  The arena factory places every bar outside the inner opening (centre at
# inner/2 + thickness/2), so a thicker bar grows outwards and the free opening stays GATE_INNER_SIZE_M.
# G06: 0.25 x 0.30 m, chosen offline (scripts/analyze_gate_bar_section.py, results/gate/bar_section_analysis.json)
# among 0.18x0.22 ... 0.50x0.60.  Closed-loop localization before the critical region (s -2.0..-1.2, 0.3-0.5 m
# lateral error, MEDIUM-like noise) is confirmed in 46 % of the poses instead of 35 %, with no unsafe confirmation and
# no wrong correction; the larger sections add little (47-49 %) while the structure window of the classifier hides a
# drone crossing the gate longer: formal M4e (visible again within cr_half_len + 2 m) holds up to 0.30 x 0.40 (2.74 m
# of 2.80) and fails from 0.35 x 0.45; at 0.25 x 0.30 it is 2.65 m.
GATE_BAR_SECTION_M = {"G06": (0.25, 0.30)}


def arena_root() -> Path:
    root = Path(os.environ.get("MARINE_RACE_ARENA_ROOT", str(DEFAULT_ARENA_ROOT)))
    if not (root / "marine_race_arena").is_dir():
        raise FileNotFoundError(
            f"Marine Race Arena not found at {root}. Set MARINE_RACE_ARENA_ROOT to the HoloDroneCompetition checkout."
        )
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return root


def _rotation_axes(rpy_deg) -> np.ndarray:
    roll, pitch, yaw = [math.radians(float(v)) for v in rpy_deg]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def load_track(track_rel: str = HORSESHOE_TRACK):
    root = arena_root()
    from marine_race_arena.config.loader import load_track_config  # type: ignore

    return load_track_config(str(root / track_rel), benchmark_task="clean_gate", current_profile="none", seed=0)


def _gate_factory(track_rel: str, inner_size: Optional[dict] = None, bar_section: Optional[dict] = None):
    """Arena GateFactory on the track config with the GATE_INNER_SIZE_M openings and GATE_BAR_SECTION_M thicknesses."""
    arena_root()
    from marine_race_arena.arena.gate_factory import GateFactory  # type: ignore

    sizes = GATE_INNER_SIZE_M if inner_size is None else inner_size
    sections = GATE_BAR_SECTION_M if bar_section is None else bar_section
    cfg = load_track(track_rel)
    gates = [dataclasses.replace(g, inner_size_m=tuple(float(v) for v in sizes[g.id])) if g.id in sizes else g
             for g in cfg.gates]
    gates = [dataclasses.replace(g, bar_thickness_m=float(sections[g.id][0])) if g.id in sections else g for g in gates]
    return GateFactory(dataclasses.replace(cfg, gates=gates))


def _visual_gate(factory, g, bar_section: Optional[dict] = None):
    """The arena's four bars of gate g, with the GATE_BAR_SECTION_M depth (a track-wide value in the arena)."""
    sections = GATE_BAR_SECTION_M if bar_section is None else bar_section
    if g.id in sections:
        cfg = factory.config
        track = dataclasses.replace(cfg.track, gate_depth_m=float(sections[g.id][1]))
        factory = type(factory)(dataclasses.replace(cfg, track=track))
    return factory.build_visual_gate(g)


def load_arena_gates(track_rel: str = HORSESHOE_TRACK, gate_ids: Optional[Sequence[str]] = None,
                     inner_size: Optional[dict] = None, bar_section: Optional[dict] = None):
    from holo_fleet.mission import BarBox, GateSpec

    factory = _gate_factory(track_rel, inner_size, bar_section)
    gates = factory.build_gates()
    wanted = list(gate_ids) if gate_ids is not None else [g.id for g in gates]
    by_id = {g.id: g for g in gates}
    out: List[GateSpec] = []
    for gid in wanted:
        g = by_id[gid]
        vis = _visual_gate(factory, g, bar_section)
        bars = []
        for bar in vis.bars:
            axes = _rotation_axes(bar.rotation_rpy_deg)
            bars.append(BarBox(center=np.array(bar.position, dtype=float), axes=axes,
                               half=np.array(bar.dimensions_m, dtype=float) / 2.0))
        axis = np.array([g.passage_direction[0], g.passage_direction[1], 0.0], dtype=float)
        axis /= np.linalg.norm(axis)
        right = np.array([axis[1], -axis[0], 0.0])
        out.append(GateSpec(gate_id=g.id, center=np.array(g.center, dtype=float), axis=axis, right=right,
                            inner_width=float(g.inner_width_m), inner_height=float(g.inner_height_m), bars=tuple(bars)))
    return out


def visual_gates(track_rel: str = HORSESHOE_TRACK, gate_ids: Optional[Sequence[str]] = None):
    """Arena VisualGate objects (bars) for spawning with the arena's HoloOcean spawner."""
    factory = _gate_factory(track_rel)
    gates = [g for g in factory.build_gates() if gate_ids is None or g.id in gate_ids]
    return [_visual_gate(factory, g) for g in gates]


def make_spawner(env):
    arena_root()
    from marine_race_arena.adapters.visual_spawner import HoloOceanVisualSpawner  # type: ignore

    return HoloOceanVisualSpawner(env)


def thruster_command(surge: float, sway: float, heave: float, yaw: float, limit: float = 12.0) -> np.ndarray:
    """Exactly the arena's ``BaseRaceAdapter.command_to_bluerov2_thrusters`` mapping (BlueROV2, scheme 0)."""
    c = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
    surge, sway, heave, yaw = c(surge), c(sway), c(heave), c(yaw)
    vertical = [heave] * 4
    horizontal = [surge + sway + 0.35 * yaw, surge - sway - 0.35 * yaw,
                  surge + sway - 0.35 * yaw, surge - sway + 0.35 * yaw]
    return np.array([max(-limit, min(limit, v * limit)) for v in vertical + horizontal])
