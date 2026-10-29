"""B2: acoustic exit evidence for the CR occupancy latch (P2), from identified range-bearing fixes.

The sonar latch of gate_perception.py stays as it is: BUSY on an unexplained FRONT corridor echo, EXITING on a
sonar exit witness with the corridor empty, FREE after t_clear more (or the t_occ_max fallback).  B2 adds ONE more
way into EXITING, under the same "corridor empty" condition, and nothing else: an acoustic observation never
moves the latch while a corridor echo is seen, never touches the precedence decision, and its absence means
nothing.

Acoustic exit evidence at time t, for the episode whose last corridor sighting was at t_lc (the latest instant at
which a hull in or at the CR was seen; since then the corridor has been empty):

(A1) accounting at t_lc.  Every other drone j of the plan is either EXCLUDED - an accepted fix of j within
     ``tie_window_s`` of t_lc whose error region, grown by v_bound |t_fix - t_lc|, cannot contain at t_lc a hull
     in the CR (CR box grown by R_OUT and by the own gate-frame navigation error) nor the hull that produced a
     corridor echo of that sighting (the FRONT cone shell of each echo range, grown by R_OUT), and no rejected
     (contradictory) fix of j in that window that could - or a POSSIBLE OCCUPANT.  A drone without fixes is a
     possible occupant (absence is no evidence).
(A2) every possible occupant is positively OUTSIDE the CR now: ``exit_confirm`` consecutive accepted fixes after
     t_lc, on one track, each region clear of the CR (grown by R_OUT and the own gate-frame navigation error) at its
     own time, the latest still clear when grown by v_bound (t - t_fix); no rejected fix since the first of them that
     could place it in the CR.
(A3) at least one possible occupant is positively BEYOND the exit face (the same, with the whole region at s >
     cr_half_len + R_OUT + nav error): the occupant has left.  An episode none of whose possible occupants is a drone
     of the plan is left to the sonar rules.

Soundness argument (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, section 4): under the B0 assumption that every
entry into the CR crosses the FRONT corridor visibly (formal M3/M4), a drone in the CR at t either was in it or
produced the corridor echo at t_lc, or entered later visibly (a new t_lc).  (A1) leaves every such drone among
the possible occupants unless its fixes, inside the declared envelope, place it elsewhere; (A2) places every
possible occupant outside the CR now.  The argument needs the envelope (config.AcousticConfig) to hold for the
accepted fixes: reflected arrivals that pass the kinematic and depth gates twice in a row on the same track are
the residual risk, quantified in scripts/analyze_b2_channel.py.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.acoustic import AcousticTracker, Fix
from holo_fleet.perception.exit_witness import speed_bound
from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, NEAR_FIELD_WIDEN_M, R_OUT


def gate_points(g, P: np.ndarray) -> np.ndarray:
    """World points (n, 3) -> gate frame (s, l, dz)."""
    d = np.atleast_2d(np.asarray(P, dtype=float)) - g.center
    left = np.array([-g.axis[1], g.axis[0], 0.0])
    return np.stack([d @ g.axis, d @ left, d[:, 2]], axis=1)


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class AcousticExit:
    def __init__(self, cfg: FleetConfig, tracker: AcousticTracker, others: Sequence[int]):
        self.cfg = cfg
        self.tracker = tracker
        self.others = list(others)
        G = cfg.gate
        self.half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
        self.v_bound = speed_bound(cfg)
        self.sighting: Optional[Dict] = None       # last corridor sighting: t, FRONT sonar (gate frame), ranges
        self.last: Dict = {}                       # diagnostics of the latest evaluation

    def reset(self) -> None:
        self.sighting = None

    # ------------------------------------------------------------------ inputs
    def note_corridor(self, t: float, g, p: np.ndarray, yaw: float, ranges: List[float]) -> None:
        R = _rot(yaw)
        m = gate_points(g, (p + R @ MOUNTS["FRONT"])[None, :])[0]
        a = g.vec_to_gate_frame(R @ AXES["FRONT"])
        self.sighting = {"t": t, "m": m, "a": a / np.linalg.norm(a), "ranges": [float(r) for r in ranges]}

    # ------------------------------------------------------------------ geometry
    def _cr_possible(self, q: np.ndarray, rho: float) -> bool:
        """Some point within rho of a sample q (gate frame) can be the centre of a hull in the CR."""
        gap = np.linalg.norm(q - np.clip(q, -self.half, self.half), axis=1)
        return bool(np.any(gap <= R_OUT + rho + self.cfg.acoustic.nav_err_gate_m))

    def _echo_possible(self, q: np.ndarray, rho: float) -> bool:
        """Some point within rho of a sample q can be the centre of the hull of a corridor echo of the sighting."""
        sg, env = self.sighting, self.cfg.env
        d = q - sg["m"]
        dist = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
        ang = np.arccos(np.clip((d @ sg["a"]) / dist, -1.0, 1.0))
        for r_e in sg["ranges"]:
            lo, hi = max(r_e - env.eps_range_far, 0.0), r_e + env.eps_range_near
            half = math.radians(self.cfg.perc.sonar.half_angle_deg) + math.atan2(NEAR_FIELD_WIDEN_M, max(lo, 1e-3))
            grow = R_OUT + rho
            in_shell = (dist >= lo - grow) & (dist <= hi + grow)
            in_cone = ang <= half + np.arcsin(np.clip(grow / dist, 0.0, 1.0))
            if np.any(in_shell & in_cone):
                return True
        return False

    def _could_be_occupant(self, g, f: Fix, t_lc: float) -> bool:
        q = gate_points(g, f.pts)
        rho = self.v_bound * abs(f.t - t_lc) + f.slack
        return self._cr_possible(q, rho) or self._echo_possible(q, rho)

    def s_lower(self, g, f: Fix, t: float) -> float:
        """Smallest gate-frame s the drone of fix f can have at t (region, slack, own nav error, motion since)."""
        q = gate_points(g, f.pts)
        return float(q[:, 0].min()) - f.slack - self.cfg.acoustic.nav_err_gate_m - self.v_bound * max(t - f.t, 0.0)

    def outside_now(self, g, j: int, t: float, t_from: float, beyond: bool = False) -> Optional[Fix]:
        """``exit_confirm`` consecutive accepted fixes of j after ``t_from``, on one track, each region outside the CR
        (``beyond``: beyond its exit face) at its own time, the latest still outside when grown by v_bound up to t,
        and no rejected fix since the first of them that could place j in the CR.  Returns the latest fix or None."""
        a = self.cfg.acoustic
        after = self.tracker.all_fixes(j, t_from + 1e-9, t)
        acc = [f for f in after if f.accepted]
        if len(acc) < a.exit_confirm:
            return None
        tail = acc[-a.exit_confirm:]
        if len({f.track for f in tail}) != 1:
            return None
        if beyond:
            s_exit = self.half[0] + R_OUT
            ok = all(self.s_lower(g, f, f.t) > s_exit for f in tail) and self.s_lower(g, tail[-1], t) > s_exit
        else:
            ok = (all(not self._cr_possible(gate_points(g, f.pts), f.slack) for f in tail)
                  and not self._cr_possible(gate_points(g, tail[-1].pts), tail[-1].slack + self.v_bound * (t - tail[-1].t)))
        contra = [f for f in after if not f.accepted and f.t >= tail[0].t
                  and self._cr_possible(gate_points(g, f.pts), f.slack + self.v_bound * max(t - f.t, 0.0))]
        return tail[-1] if ok and not contra else None

    # ------------------------------------------------------------------ decision
    def evaluate(self, t: float, g) -> Optional[Dict]:
        """The acoustic exit evidence (``{sector: ACOUSTIC, slots, range}``) or None; call with the corridor empty.

        (A1) accounting at t_lc (module docstring); (A2') every possible occupant is positively OUTSIDE the CR now
        (``exit_confirm`` accepted fixes); (A3) at least one of them positively BEYOND its exit face: the occupant
        has left.  Drones excluded at t_lc need nothing more."""
        a = self.cfg.acoustic
        self.last = {"possible": [], "not_outside": [], "excluded": [], "exited": []}
        if self.sighting is None:
            return None
        t_lc = self.sighting["t"]
        possible = []
        for j in self.others:
            win = self.tracker.all_fixes(j, t_lc - a.tie_window_s, t_lc + a.tie_window_s)
            acc = [f for f in win if f.accepted and not self._could_be_occupant(g, f, t_lc)]
            contra = [f for f in win if not f.accepted and self._could_be_occupant(g, f, t_lc)]
            if acc and not contra:
                self.last["excluded"].append(j)
                continue
            possible.append(j)
        self.last["possible"] = list(possible)
        if not possible:
            return None
        exited, ranges = [], []
        for j in possible:
            if self.outside_now(g, j, t, t_lc) is None:
                self.last["not_outside"].append(j)
                return None
            f = self.outside_now(g, j, t, t_lc, beyond=True)
            if f is not None:
                exited.append(j)
                ranges.append(round(f.r, 2))
        self.last["exited"] = exited
        if not exited:
            return None
        return {"sector": "ACOUSTIC", "slots": exited, "range": min(ranges)}
