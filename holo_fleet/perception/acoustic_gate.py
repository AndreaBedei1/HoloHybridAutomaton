"""B2: acoustic exit evidence for the CR occupancy latch (P2), from identified range-bearing fixes.

The sonar latch of gate_perception.py is BUSY on an unexplained FRONT corridor echo, EXITING on a confirmed sonar
exit witness with the corridor empty, FREE after t_clear more (or the t_occ_max fallback). B2 also marks BUSY when
two recent accepted fixes place a drone possibly inside under the complete declared region (``heard_inside``), and
vetoes releases and admission while this evidence persists. Acoustic exit evidence is evaluated only with the
corridor empty. An acoustic-only BUSY episode can release on A2 for every other drone, without a sonar sighting.
Acoustic observations never touch precedence; their absence is not positive clearance. The timeout fallback keeps
the stated bounded-crossing assumption of the sonar outage mode.

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

Payload (H6).  A fix whose payload is consistent with its measurement (perception/acoustic.py) has a second region:
the ball of radius b_rep + v_bound |t - t_tx| around the reported position, in the mission frame of the gate map
(no own navigation error enters: both are mission-frame quantities).  The true position lies in BOTH regions under
the two envelopes, so their intersection is used: a fix is clear of the CR when either region is (A2), can be the
occupant at t_lc only when both regions allow it (A1), and is beyond the exit face when either places it there
(A3).  ``heard_inside`` therefore vetoes only when both regions allow the CR: the indistinguishable multipath
histories of the range-bearing measurement alone (a drone beyond the exit with a +3 m excess) are separated by the
independent navigation estimate.  An inconsistent or missing payload leaves the measurement-only rules.

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
        self.veto: set = set()                     # drones whose heard-inside veto is on (hysteresis)
        self.last_phantom: Dict = {}               # the excluding fixes of the latest phantom echo (audit log)
        self.heard_by: set = set()                 # ... at the latest heard_inside evaluation
        self.last: Dict = {}                       # diagnostics of the latest evaluation

    def reset(self) -> None:
        self.sighting = None

    # ------------------------------------------------------------------ inputs
    def note_corridor(self, t: float, g, p: np.ndarray, yaw: float, ranges: List[float]) -> None:
        R = _rot(yaw)
        m = gate_points(g, (p + R @ MOUNTS["FRONT"])[None, :])[0]
        a = g.vec_to_gate_frame(R @ AXES["FRONT"])
        self.sighting = {"t": t, "m": m, "a": a / np.linalg.norm(a), "ranges": [float(r) for r in ranges]}

    def no_fleet_drone(self, t: float, g, p: np.ndarray, yaw: float, ranges: List[float]) -> bool:
        """The corridor echo of this capture cannot be a fleet drone: every other drone of the plan is EXCLUDED at t
        (as A1 at t_lc: an accepted fix within tie_window_s whose region, grown by its motion, can neither be a hull in
        the CR nor the hull of one of these echoes, and no contradictory rejected fix that could).  A4 applied at the
        sighting instant: P2 concerns the fleet's drones, and the echo stays an obstacle for the separation layer.
        Without a fix of every other drone (a silent channel) nothing is excluded: the sonar latch rules apply."""
        if not self.others:
            return False
        a = self.cfg.acoustic
        saved = self.sighting
        self.note_corridor(t, g, p, yaw, ranges)
        try:
            self.last_phantom = {}
            for j in self.others:
                win = self.tracker.all_fixes(j, t - a.tie_window_s, t)
                acc = [f for f in win if f.accepted]
                # every accepted fix of the window must exclude j (one accepted outlier never suffices), the latest no
                # older than fresh_s, and no rejected fix of the window may place j in the CR or at the echo
                if (not acc or t - acc[-1].t > a.fresh_s or any(self._could_be_occupant(g, f, t) for f in acc)
                        or any(self._could_be_occupant(g, f, t) for f in win if not f.accepted)):
                    return False
                self.last_phantom[j] = [round(f.t, 2) for f in acc]
            return True
        finally:
            self.sighting = saved

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

    def rep_cr_possible(self, g, f: Fix, t: float) -> bool:
        """The payload ball of f at t (mission frame) can hold the centre of a hull in the CR (True without a usable
        payload: no evidence)."""
        if not f.rep_ok:
            return True
        q = gate_points(g, f.rep.p[None, :])
        gap = np.linalg.norm(q - np.clip(q, -self.half, self.half), axis=1)
        return bool(gap[0] <= R_OUT + f.rep_radius(t, self.v_bound))

    def _could_be_occupant(self, g, f: Fix, t_lc: float) -> bool:
        q = gate_points(g, f.pts)
        rho = self.v_bound * abs(f.t - t_lc) + f.slack
        meas = self._cr_possible(q, rho) or self._echo_possible(q, rho)   # own-frame errors are in the region
        if not meas or not f.rep_ok:
            return meas
        b_own = self.tracker.pose.bound_at(t_lc)
        if b_own is None:
            return True
        qr = gate_points(g, f.rep.p[None, :])
        rr = f.rep_radius(t_lc, self.v_bound)
        # the echo shell is expressed in the own estimated frame: the reported position is compared with it within
        # the own navigation bound as well
        # a mission-frame point against the own-frame echo cone: the own attitude error rotates the cone as well
        a = self.cfg.acoustic
        att = float(np.linalg.norm(qr[0] - self.sighting["m"])) * math.radians(a.heading_err_deg + a.tilt_err_deg)
        return self.rep_cr_possible(g, f, t_lc) or self._echo_possible(qr, rr + b_own + att)

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
        return (not self._cr_possible(gate_points(g, f.pts), grow) or self.clear_by_range(g, f, t)
                or not self.rep_cr_possible(g, f, t))

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
            ok = all(float(gate_points(g, f.p[None, :])[0][0]) > s_exit
                     or (f.rep_ok and float(gate_points(g, f.rep.p[None, :])[0][0]) - f.rep_radius(f.t, self.v_bound)
                         > s_exit) for f in tail)
        else:                                       # A2, safety: clear of the CR at its time, the latest also now
            ok = all(self.clear_of_cr(g, f, f.t) for f in tail) and self.clear_of_cr(g, tail[-1], t)
        contra = [f for f in after if not f.accepted and f.t >= tail[0].t
                  and self._cr_possible(gate_points(g, f.pts), f.slack + self.v_bound * max(t - f.t, 0.0))]
        return tail[-1] if ok and not contra else None

    def heard_inside(self, t: float, g) -> bool:
        """Positive evidence of occupancy: the two latest accepted fixes of one drone, on one track, the latest no older
        than ``acoustic.fresh_s`` (one lost ping plus the latency: the veto does not drop on a single lost ping), both
        place it possibly inside the CR now under the complete declared region, including one-sided multipath.
        Gate perception blocks commitment, pending releases and admission while this evidence persists.

        Two fixes reject a single outlier; hysteresis: once on, the veto of a drone drops only when its two latest
        fixes are BOTH clear of the CR (a single cleared fix, e.g. a payload that passed the CRC by chance, does not
        end it), or when it has no fresh fix (then last_known_inside keeps the latch from timing out).  A nearby
        outside drone can remain ambiguous indefinitely: expiration is bounded after the last compatible reception,
        not merely after its true centre leaves the CR.  Missing fixes provide no positive clearance."""
        a, period = self.cfg.acoustic, self.cfg.beacon.period_s
        fresh = a.fresh_s
        self.heard_by = set()
        for j in self.others:
            acc = [f for f in self.tracker.all_fixes(j, t - 2.0 * period - fresh, t) if f.accepted][-2:]
            if not acc or t - acc[-1].t > fresh:
                self.veto.discard(j)
                continue
            poss = [not self.clear_of_cr(g, f, t) for f in acc]
            pair = len(acc) == 2 and acc[0].track == acc[1].track
            if pair and all(poss):
                self.veto.add(j)
            elif j in self.veto and pair and not any(poss):
                self.veto.discard(j)
            if j in self.veto:
                self.heard_by.add(j)
        return bool(self.heard_by)

    def last_known_inside(self, t: float, g) -> bool:
        """Some other drone's LATEST accepted fix (within the tracker memory) placed it possibly in the CR at its own
        time, and no later accepted fix has cleared it: its whereabouts since are unknown, so it is a possible
        occupant and the CR cannot become FREE by a timeout (an unknown occupant never silently becomes FREE).
        Without any fix of a drone (a silent channel) the sonar outage-mode rules apply."""
        n = self.cfg.acoustic.exit_confirm
        for j in self.others:
            last = self.tracker.last_accepted.get(j)
            if last is None or last.t > t:
                continue
            gone = max(self.cfg.gate.t_occ_max + self.cfg.gate.t_clear, self.cfg.acoustic.silent_gone_s)
            if t - last.t > gone and not self.tracker.all_fixes(j, last.t + 1e-9, t):
                continue                   # silent that long: risen out of the CR (deployment assumption, config)
            # cleared only by exit_confirm consecutive accepted fixes of one track, each clear (as A2): one cleared
            # fix followed by silence does not release a possible occupant
            tail = [f for f in self.tracker.all_fixes(j, -1e9, t) if f.accepted][-n:] or [last]
            if any(not self.clear_of_cr(g, f, f.t) for f in tail):
                return True
        return False

    # ------------------------------------------------------------------ decision
    def evaluate_clearance(self, t: float, g, since: float, suspects=None) -> Optional[Dict]:
        """A2 for the drones heard possibly inside during an acoustic-only BUSY episode (``suspects``; every other
        drone when not given).  Without a corridor sighting, an entry by another drone would have crossed the FRONT
        corridor visibly (the sonar latch premise, formal M3/M4) and started a sonar episode instead."""
        need = self.others if suspects is None else [j for j in self.others if j in suspects]
        if need and all(self.outside_now(g, j, t, since) is not None for j in need):
            return {"sector": "ACOUSTIC", "slots": sorted(need), "range": None, "rule": "A2"}
        return None

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
                # diagnostics: the latest fixes of j since t_lc (time, accepted, payload, clear of the CR at t)
                self.last["fixes"] = [(round(f.t, 1), f.accepted, f.rep_reason, self.clear_of_cr(g, f, t) if f.accepted
                                       else None) for f in self.tracker.all_fixes(j, t_lc + 1e-9, t)[-4:]]
                return None
            f = self.outside_now(g, j, t, t_lc, beyond=True)
            if f is not None:
                exited.append(j)
                ranges.append(round(f.r, 2))
        self.last["exited"] = exited
        if not exited:
            return None
        return {"sector": "ACOUSTIC", "slots": exited, "range": min(ranges)}
