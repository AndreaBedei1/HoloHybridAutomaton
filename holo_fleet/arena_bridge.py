"""Bridge to the existing Marine Race Arena project (re-used, not copied).

The arena lives in ``~/Desktop/HoloDroneCompetition`` (override with the
``MARINE_RACE_ARENA_ROOT`` environment variable).  We reuse its track files, its
config loader, its gate geometry/visual factory and its HoloOcean prop spawner.
Only static map geometry crosses this bridge; no simulator state does.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

DEFAULT_ARENA_ROOT = Path.home() / "Desktop" / "HoloDroneCompetition"
HORSESHOE_TRACK = "marine_race_arena/tracks/marine_race_horseshoe_bay.json"


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


def load_arena_gates(track_rel: str = HORSESHOE_TRACK, gate_ids: Optional[Sequence[str]] = None):
    from holo_fleet.mission import BarBox, GateSpec

    arena_root()
    from marine_race_arena.arena.gate_factory import GateFactory  # type: ignore

    cfg = load_track(track_rel)
    factory = GateFactory(cfg)
    gates = factory.build_gates()
    wanted = list(gate_ids) if gate_ids is not None else [g.id for g in gates]
    by_id = {g.id: g for g in gates}
    out: List[GateSpec] = []
    for gid in wanted:
        g = by_id[gid]
        vis = factory.build_visual_gate(g)
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
    arena_root()
    from marine_race_arena.arena.gate_factory import GateFactory  # type: ignore

    cfg = load_track(track_rel)
    factory = GateFactory(cfg)
    gates = [g for g in factory.build_gates() if gate_ids is None or g.id in gate_ids]
    return factory.build_visual_gates(gates)


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
