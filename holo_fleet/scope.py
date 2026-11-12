"""Operating scope of the permanent-failure model: the exclusion zone of a gate.

A scope decision taken during development (2026-11-12), not pre-registered: permanent failures inside a gate, or close enough to it that
the passive hull can obstruct the passage, are outside the scope.  They are not run and have no special handling.  The zone is defined
geometrically, from the physics of the failure model and the water current of the mission, independently of any result.

A broken drone coasts at most ``coast_m`` and then moves with the water (horizontal speed at most ``w_h``, downward at most ``w_down``) while
it rises at ``v_rise_min_mps`` or more.  The obstruction volume O of a gate is its critical region and frame, grown by the hull's
circumscribed radius R_OUT.  The failure position p is in the exclusion zone E when the hull can reach O horizontally before it has risen
above the top of O:

    d_h(p, O) <= coast_m + w_h * max(z_top(O) - z(p), 0) / (v_rise_min_mps - w_down)

and every position is in E when the hull does not rise (``w_down >= v_rise_min_mps``) or the top of O is not below the surface, where
the hull stops rising.  Outside E the hull rises above O before it can
reach it and never comes back: a dead drone is never in or at the CR (formal/check_status.py, S12), which is what lets AcousticExit.dead
exclude it from the gate decisions.
"""

from __future__ import annotations

import itertools
import math

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import R_OUT


def obstruction_half(g, cfg: FleetConfig = DEFAULT) -> np.ndarray:
    """Half extents (s, l, dz) of the box holding the critical region and the frame of gate g, grown by R_OUT."""
    G = cfg.gate
    half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
    for b in getattr(g, "bars", ()):
        for sg in itertools.product((-1.0, 1.0), repeat=3):
            c = np.abs(np.asarray(g.to_gate_frame(b.center + b.axes @ (np.array(sg) * b.half)), float))
            half = np.maximum(half, c)
    return half + R_OUT


def failure_excluded(g, p, w_h: float, w_down: float = 0.0, cfg: FleetConfig = DEFAULT) -> bool:
    """A permanent failure with the hull centre at p (world) is inside the exclusion zone E of gate g (module docstring)."""
    F = cfg.failure
    h = obstruction_half(g, cfg)
    s, l, dz = (float(v) for v in g.to_gate_frame(np.asarray(p, float)))
    d_h = math.hypot(max(abs(s) - h[0], 0.0), max(abs(l) - h[1], 0.0))
    rise = max(h[2] - dz, 0.0)
    if rise == 0.0:
        return False
    if F.v_rise_min_mps - w_down <= 0.0 or float(g.center[2]) + h[2] >= F.surface_z_m:
        return True
    return d_h <= F.coast_m + w_h * rise / (F.v_rise_min_mps - w_down)
