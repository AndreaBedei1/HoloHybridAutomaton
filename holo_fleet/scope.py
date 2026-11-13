"""Operating scope of the permanent-failure model: the exclusion zone of a gate.

A scope decision taken during development (2026-11-12, after the runs that it concerns), not pre-registered: permanent failures
inside a gate, or close enough to it that the passive hull can obstruct the passage, are outside the scope.  They are not run.
The zone is defined geometrically, from the failure model and the current declared for the mission, independently of any result.

A broken drone coasts at most ``coast_h_m`` horizontally and ``coast_v_m`` downward, and then moves with the water (horizontal
speed at most ``drift_factor`` times the declared bound w_h, downward at most w_down) while it rises, after at most ``rise_lag_s``,
at ``v_rise_min_mps`` or more.  The obstruction volume O of a gate is its critical region and frame, grown by the hull's
circumscribed radius R_OUT.  With h0 the depth of the hull centre below the top of O, the hull is above the top after at most

    t* = max(0, (h0 + coast_v_m + v_rise_min_mps * rise_lag_s) / (v_rise_min_mps - w_down))

and the failure position p is in the exclusion zone E when the hull can reach O horizontally before then:

    d_h(p, O) <= coast_h_m + drift_factor * w_h * t*

Every position is in E when the hull does not rise (``w_down >= v_rise_min_mps``) or the top of O is not below the surface, where
the hull stops rising.  Outside E the hull rises above O before it can reach it and never comes back: it is never in or at the CR
(formal/check_status.py, S12).  Onboard (AcousticExit.dead) a silent drone is declared dead for the gate decisions only when its
last known position, grown by its uncertainty, is outside E for the declared current; otherwise it keeps holding the gate (a safe
stop: the failures inside E are outside the scope, and this is their only treatment).
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


def failure_excluded(g, p, w_h: float, w_down: float = 0.0, cfg: FleetConfig = DEFAULT, margin: float = 0.0) -> bool:
    """A permanent failure with the hull centre at p (world), known within ``margin`` metres horizontally (the depth is measured), is inside the exclusion zone E of
    gate g for a current bounded by w_h (horizontal) and w_down (downward) (module docstring)."""
    S = cfg.scope
    h = obstruction_half(g, cfg)
    if S.v_rise_min_mps - w_down <= 0.0 or float(g.center[2]) + h[2] >= S.surface_z_m:
        return True
    s, l, dz = (float(v) for v in g.to_gate_frame(np.asarray(p, float)))
    d_h = math.hypot(max(abs(s) - h[0], 0.0), max(abs(l) - h[1], 0.0)) - margin
    h0 = h[2] - dz
    t_star = max(0.0, (h0 + S.coast_v_m + S.v_rise_min_mps * S.rise_lag_s) / (S.v_rise_min_mps - w_down))
    return d_h <= S.coast_h_m + S.drift_factor * w_h * t_star


def declared_excluded(g, p, cfg: FleetConfig = DEFAULT, margin: float = 0.0) -> bool:
    """failure_excluded for the current declared for the mission (cfg.scope)."""
    return failure_excluded(g, p, cfg.scope.w_h_max_mps, cfg.scope.w_down_max_mps, cfg, margin)


def current_bound(current) -> tuple:
    """Upper bound (horizontal, downward) of the water speed of a CurrentField: the sum over its components of the steady part
    and of the amplitude of the sinusoids and gusts.  The runner declares it for the mission (cfg.scope)."""
    w_h = w_dn = 0.0
    for c in current.describe():
        for key in ("drift", "amplitude"):
            d = np.asarray(c.get(key, (0.0, 0.0, 0.0)), float)
            w_h += float(np.hypot(d[0], d[1]))
            w_dn += abs(float(d[2]))
    return round(w_h, 4), round(w_dn, 4)
