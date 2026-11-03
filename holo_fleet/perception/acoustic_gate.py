"""B2: acoustic exit evidence for the CR occupancy latch (P2), from identified range-bearing fixes.

The sonar latch of gate_perception.py stays as it is: BUSY on an unexplained FRONT corridor echo, EXITING on a
sonar exit witness with the corridor empty, FREE after t_clear more (or the t_occ_max fallback).  B2 adds ONE more
way into EXITING, under the same "corridor empty" condition, and one restriction: the t_occ_max fallback does not
release the CR while two recent accepted fixes place a drone possibly inside it on the clear path (``heard_inside``:
a drone stopped in the gate by a fault that still pings).  An acoustic observation never moves the latch while a corridor echo is seen,
never touches the precedence decision, and its absence means nothing.

Acoustic exit evidence at time t, for the episode whose last corridor sighting was at t_lc (the latest instant at
which a hull in or at the CR was seen; since then the corridor has been empty):

(A1) accounting at t_lc.  Every other drone j of the plan is either EXCLUDED - an accepted fix of j within
     ``tie_window_s`` of t_lc whose error region, grown by v_bound |t_fix - t_lc|, cannot contain at t_lc a hull
     in the CR (CR box grown by R_OUT and by the own gate-frame navigation error) nor the hull that produced a
     corridor echo of that sighting (the FRONT cone shell of each echo range, grown by R_OUT), and no rejected
     (contradictory) fix of j in that window that could - or a POSSIBLE OCCUPANT.  A drone without fixes is a
     possible occupant (absence is no evidence).
(A2) every possible occupant is positively OUTSIDE the CR now: ``exit_confirm`` consecutive accepted fixes after
     t_lc, on one track, each clear of the CR (grown by R_OUT and the own gate-frame navigation error) at its own
     time, the latest still clear when grown by v_bound (t - t_fix); no rejected fix since the first of them that
     could place it in the CR.  A fix is clear of the CR when its whole region is, or - whatever its direction - when
     its smallest possible true range (measured range minus the range envelope and the one-sided multipath excess)
     exceeds the largest distance from the own position to the grown CR: a drone heard far enough away cannot be in
     it, and the direction of arrival, the least precise part of a fix at these ranges, is not needed.
(A3) progress, not safety: at least one possible occupant was heard beyond the exit face (the nominal positions of
     those fixes at s > cr_half_len + R_OUT): the occupant has passed, not backed out.
(A4) an episode none of whose possible occupants is a drone of the plan (every other drone excluded at t_lc) gives
     the exit evidence once every other drone is also positively outside the CR now (A2): no fleet drone is in it.

Soundness argument (mutual exclusion rests on A1 and A2 only): under the B0 assumption that every
entry into the CR crosses the FRONT corridor visibly (formal M3/M4), a drone in the CR at t either was in it or
produced the corridor echo at t_lc, or entered later visibly (a new t_lc).  (A1) leaves every such drone among
the possible occupants unless its fixes, inside the declared envelope, place it elsewhere; (A2) places every
possible occupant outside the CR now.  The argument needs the envelope (config.AcousticConfig) to hold for the
accepted fixes: reflected arrivals that pass the kinematic and depth gates twice in a row on the same track are
the residual risk, quantified in calibration/analyze_b2_channel.py.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.acoustic import AcousticTracker, Fix, box_bounds
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

    def clear_by_range(self, g, f: Fix, t: float) -> bool:
        """The drone of fix f cannot be in the CR (grown by R_OUT) at t, from the range alone: its smallest possible
        true range, less its motion since the fix, exceeds the largest distance from the own position (at the fix
        time, within the own gate-frame navigation error) to the grown CR."""
        a = self.cfg.acoustic
        r_lo = f.r - box_bounds(f.r, self.cfg)[0] - a.los_excess_max_m - self.v_bound * max(t - f.t, 0.0)
        own = gate_points(g, (f.p - f.rel)[None, :])[0]
        corners = (self.half + R_OUT) * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        return r_lo > float(np.max(np.linalg.norm(corners - own, axis=1))) + a.nav_err_gate_m

    def clear_of_cr(self, g, f: Fix, t: float) -> bool:
        """A2 for one fix at time t (module docstring): its region, grown by the motion since the fix, or its range."""
        grow = f.slack + self.v_bound * max(t - f.t, 0.0)
        return not self._cr_possible(gate_points(g, f.pts), grow) or self.clear_by_range(g, f, t)

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
        if beyond:                                  # A3, progress: heard beyond the exit face (nominal positions)
            s_exit = self.half[0] + R_OUT
            ok = all(float(gate_points(g, f.p[None, :])[0][0]) > s_exit for f in tail)
        else:                                       # A2, safety: clear of the CR at its time, the latest also now
            ok = all(self.clear_of_cr(g, f, f.t) for f in tail) and self.clear_of_cr(g, tail[-1], t)
        contra = [f for f in after if not f.accepted and f.t >= tail[0].t
                  and self._cr_possible(gate_points(g, f.pts), f.slack + self.v_bound * max(t - f.t, 0.0))]
        return tail[-1] if ok and not contra else None

    def _inside_clear_path(self, g, f: Fix, t: float) -> bool:
        """The clear-path part of the region of f (ranges no shorter than the measured range minus the range envelope:
        without the one-sided multipath excess), grown by the motion since the fix, can hold a hull in the CR."""
        br = box_bounds(f.r, self.cfg)[0]
        own = f.p - f.rel
        pts = f.pts[np.linalg.norm(f.pts - own, axis=1) >= f.r - br - 1e-6]
        return len(pts) > 0 and self._cr_possible(gate_points(g, pts), f.slack + self.v_bound * max(t - f.t, 0.0))

    def heard_inside(self, t: float, g) -> bool:
        """Positive evidence of occupancy: the two latest accepted fixes of one drone, on one track, the latest no older
        than ``acoustic.fresh_s`` (one lost ping plus the latency: the veto does not drop on a single lost ping), both
        place it possibly inside the CR now on the clear path
        (``_inside_clear_path``).  The t_occ_max fallback of the sonar latch then does not release the CR
        (gate_perception) and a committed drone is not admitted into it (stage 2).

        It is a veto ADDED to the sonar rules: a false negative leaves the sonar-only behaviour, a false positive
        blocks the gate.  So it uses the clear-path region: a reflected arrival is longer, never shorter, and places a
        drone farther than it is; the one-sided multipath excess of the full region (los_excess_max_m, 3 m) made a
        drone waiting beyond the exit look possibly inside and stopped the queue for minutes.  Two fixes on one track
        reject a single outlier.  Without fixes nothing changes: their absence is no evidence."""
        a, period = self.cfg.acoustic, self.cfg.beacon.period_s
        fresh = a.fresh_s
        for j in self.others:
            acc = [f for f in self.tracker.all_fixes(j, t - 2.0 * period - fresh, t) if f.accepted][-2:]
            if (len(acc) == 2 and acc[0].track == acc[1].track and t - acc[-1].t <= fresh
                    and all(self._inside_clear_path(g, f, t) for f in acc)):
                return True
        return False

    # ------------------------------------------------------------------ decision
    def evaluate(self, t: float, g) -> Optional[Dict]:
        """The acoustic exit evidence (``{sector: ACOUSTIC, slots, range}``) or None; call with the corridor empty.

        (A1) accounting at t_lc (module docstring); (A2) every possible occupant is positively OUTSIDE the CR now
        (``exit_confirm`` accepted fixes, by region or by range); (A3, progress) at least one of them heard beyond its
        exit face.  Drones excluded at t_lc need nothing more."""
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
            # (A4) no drone of the plan can be the occupant: each was excluded at t_lc, and each is positively
            # OUTSIDE the CR now (A2), so no fleet drone is in it whether or not a later entry was seen.  P2 concerns
            # the fleet's drones; an unexplained echo that is no drone stays an obstacle for the separation layer.
            # (A phantom corridor echo held the latch for minutes while both other drones were excluded.)
            if all(self.outside_now(g, j, t, t_lc) is not None for j in self.others):
                return {"sector": "ACOUSTIC", "slots": [], "range": None, "rule": "A4"}
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
