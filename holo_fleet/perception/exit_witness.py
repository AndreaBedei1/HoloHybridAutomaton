"""Positive exit evidence for the critical-region occupancy latch (P2), onboard information only.

A queued drone holds the CR BUSY from the moment it sees an unexplained echo in its FRONT corridor.  It may
release it only on POSITIVE evidence that the occupant has left: an obstacle echo, in ANY healthy sonar
sector, that no hull inside the CR can produce and that a hull leaving the CR on the exit side could.  The
FRONT-only rule (an echo beyond the far corner of the CR) is the special case of the FRONT sector; from a
lateral queue point the bars of a wide gate can hide the exiting hull from the FRONT sonar exactly where that
rule needs it, while another sector, or the FRONT sonar a little later, sees it.

Witness conditions for an echo at range r in sector k (gate map, estimated own pose, no identity):

(W1) not from the CR: r > r_far(k) + eps_range_far (an echo is at most eps_range_far beyond the true near surface).
     When the whole CR lies inside the sector's cone (the FRONT cone of every queue point, formal M4a), a hull
     centred in the CR is in the cone and its near surface is nearer than its centre: r_far(k) = the farthest
     CR corner (the FRONT corridor bound of v2).  Otherwise a hull centred in the CR just outside the cone can
     reach into it: r_far(k) = the largest range of a point of the CR grown by the hull radius R_OUT inside
     the (near-field widened) cone, -inf if there is none - every echo of that sector satisfies W1, W2-W3 decide.
(W2) ahead and on the exit side, within reach: every hull centre compatible with the echo (shell of the cone
     between r - eps_far and r + eps_near, grown by the hull radius) lies ahead of the observer (s > its own s:
     the exit region is downstream of the CR, the observer upstream of it; a drone level with or behind it - a
     queue neighbour, a lost or a following drone - is never evidence), and one of them lies beyond the CR
     (s > cr_half_len) within v_bound * (t - t_last_corridor) of it, v_bound the speed bound of one drone.
(W3) new: no obstacle echo of that sector already satisfied W1 while the occupant was still seen in the corridor
     at a range it could have reached since (tracked per sector, gate 0.5 m + v_bound * dt): a drone that had
     left before is not taken for the occupant.

The same sector must supply three distinct consecutive captures whose centre-range intervals share a motion-
compatible history, accounting for observer displacement. Held echoes do not add captures. This confirms range
evolution, not identity. No echo is never evidence: without a witness the latch retains its timeout fallback.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, NEAR_FIELD_WIDEN_M, R_OUT, SECTORS, fibonacci_sphere

ASSOC_M = 0.5            # range gate of a tracked background echo (per capture, plus v_bound * dt)
_GRID = 9                # CR samples per axis (W1)
_DIRS = 900              # sphere directions; those inside a cone sample it (W2)
CONFIRM_CAPTURES = 3     # [PROT] G.3(b): three distinct consecutive sonar captures, as in M4d


def speed_bound(cfg: FleetConfig = DEFAULT) -> float:
    """Upper bound of the speed of one drone: the largest commanded speed plus the unrejected drift."""
    return max(cfg.env.v_escape, cfg.env.v_max_nominal, cfg.gate.v_pass) + cfg.env.w_drift_max


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class ExitWitness:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        G = cfg.gate
        self.half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
        ax = [np.linspace(-h, h, _GRID) for h in self.half]
        self.cr_pts = np.array(np.meshgrid(*ax, indexing="ij")).reshape(3, -1).T           # gate frame
        self.slack = float(np.linalg.norm(2.0 * self.half / (_GRID - 1))) / 2.0            # grid half-diagonal
        self.dirs = fibonacci_sphere(_DIRS)
        self.v_bound = speed_bound(cfg)
        self.background: Dict[str, List[Tuple[float, float]]] = {s: [] for s in SECTORS}  # sector -> [(r, t)]
        self.confirmations = {}  # sector -> (capture time, centre-range lo/hi, sonar origin, count)

    def reset(self) -> None:
        self.background = {s: [] for s in SECTORS}
        self.clear_confirmation()

    def clear_confirmation(self) -> None:
        self.confirmations.clear()

    # ------------------------------------------------------------------ geometry
    def _cone(self, g, p: np.ndarray, yaw: float, sector: str):
        R = _rot(yaw)
        m = g.to_gate_frame(p + R @ MOUNTS[sector])
        a = g.vec_to_gate_frame(R @ AXES[sector])
        return m, a / np.linalg.norm(a)

    def _half_angle(self, r: float) -> float:
        return math.radians(self.cfg.perc.sonar.half_angle_deg) + math.atan2(NEAR_FIELD_WIDEN_M, max(r, 1e-6))

    def r_far(self, g, p: np.ndarray, yaw: float, sector: str) -> float:
        """Largest range from the sector's sonar of a CR point inside its (widened) cone; -inf if none."""
        m, a = self._cone(g, p, yaw, sector)
        nominal = math.radians(self.cfg.perc.sonar.half_angle_deg)
        corners = self.half * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        dc = corners - m
        rc = np.linalg.norm(dc, axis=1)
        if np.all(np.arccos(np.clip((dc @ a) / rc, -1.0, 1.0)) <= nominal):
            return float(rc.max())         # the whole CR is in the nominal cone: exact (v2 FRONT corridor)
        d = self.cr_pts - m
        r = np.linalg.norm(d, axis=1)
        ang = np.arccos(np.clip((d @ a) / np.maximum(r, 1e-9), -1.0, 1.0))
        # a hull centred at a grid point (or between grid points: slack) reaches R_OUT around it
        grow = R_OUT + self.slack
        reach_ang = np.arcsin(np.clip(grow / np.maximum(r, grow), 0.0, 1.0))
        inside = ang <= np.array([self._half_angle(max(x - grow, 1e-3)) for x in r]) + reach_ang
        return float(r[inside].max() + grow) if inside.any() else -math.inf

    def ahead(self, g, p: np.ndarray, yaw: float, sector: str, r: float) -> bool:
        """W2 (first part): every hull centre compatible with the echo is ahead of the observer (exact: the
        smallest s of the cone shell is reached on its edge nearest the -s direction)."""
        m, a = self._cone(g, p, yaw, sector)
        env = self.cfg.env
        lo, hi = max(r - env.eps_range_far, 0.0), r + env.eps_range_near
        c = math.cos(min(math.pi, math.acos(float(np.clip(a[0], -1.0, 1.0))) + self._half_angle(lo)))
        s_min = m[0] + (lo if c >= 0.0 else hi) * c - R_OUT
        return s_min > float(g.to_gate_frame(p)[0])

    def exit_side_reachable(self, g, p: np.ndarray, yaw: float, sector: str, r: float, reach: float) -> bool:
        """W2 (second part): a hull centre compatible with the echo lies beyond the CR, within ``reach`` of it."""
        m, a = self._cone(g, p, yaw, sector)
        env = self.cfg.env
        lo, hi = max(r - env.eps_range_far, 0.0), r + env.eps_range_near
        u = self.dirs[np.arccos(np.clip(self.dirs @ a, -1.0, 1.0)) <= self._half_angle(lo)]
        for rho in (lo, 0.5 * (lo + hi), hi):
            x = m + rho * u
            # the centre is within R_OUT of the echoing surface point x
            beyond = x[:, 0] + R_OUT > self.half[0]
            gap = np.linalg.norm(x - np.clip(x, -self.half, self.half), axis=1) - R_OUT
            if np.any(beyond & (gap <= reach)):
                return True
        return False

    # ------------------------------------------------------------------ per capture
    def update(self, t: float, g, p: np.ndarray, yaw: float, targets, occupant_seen: bool,
               t_last_corridor: float, healthy: Dict[str, bool]) -> Optional[Dict]:
        """The exit witness of this capture (``{sector, range}``), or None.  ``targets`` are the persistent
        obstacle targets (targets.Target, per-sector ranges); ``occupant_seen``: a FRONT corridor echo now;
        ``healthy``: per-sector health of this capture (a sector without a reading counts as healthy: its echo
        is the target itself)."""
        reach = self.v_bound * max(t - t_last_corridor, 0.0)
        fresh = []
        for tg in targets:
            for sec, r in (tg.ranges or {sec: tg.r_min for sec in tg.pattern}).items():
                if not healthy.get(sec, True):
                    continue
                if r <= self.r_far(g, p, yaw, sec) + self.cfg.env.eps_range_far:
                    continue                                                       # W1
                # W3: echoes outside the CR seen while the occupant was in the corridor are background contacts,
                # followed per sector; none of them is the occupant
                bg = self.background[sec]
                k = next((i for i, (rb, tb) in enumerate(bg) if abs(r - rb) <= ASSOC_M + self.v_bound * (t - tb)), None)
                if k is not None:
                    bg[k] = (float(r), t)
                    continue
                if occupant_seen:
                    bg.append((float(r), t))
                    continue
                if not (self.ahead(g, p, yaw, sec, r)
                        and self.exit_side_reachable(g, p, yaw, sec, r, reach + self.cfg.env.eps_range_near)):
                    continue                                                       # W2
                fresh.append((sec, float(r), t - tg.ages.get(sec, tg.age)))
        if occupant_seen or not fresh:
            self.clear_confirmation()
            return None
        candidates = {}
        for sec, r, stamp in fresh:
            current = candidates.get(sec)
            if current is None or stamp > current[1] or (stamp == current[1] and r < current[0]):
                candidates[sec] = (r, stamp)
        confirmed, next_tracks = [], {}
        for sec, (r, stamp) in candidates.items():
            previous = self.confirmations.get(sec)
            if previous is not None and stamp <= previous[0] + 1e-6:
                next_tracks[sec] = previous  # a held/cached echo is not another capture
                continue
            origin, _ = self._cone(g, p, yaw, sec)
            lo, hi = r - self.cfg.env.eps_range_far - R_OUT, r + self.cfg.env.eps_range_near + R_OUT
            count = 1
            if previous is not None:
                pt, pl, ph, po, pc = previous
                dt = stamp - pt
                motion = self.v_bound * dt + float(np.linalg.norm(origin - po))
                lower, upper = max(lo, pl - motion), min(hi, ph + motion)
                if dt <= 1.0 / self.cfg.perc.sonar.hz + 1e-6 and lower <= upper:
                    lo, hi, count = lower, upper, pc + 1
            next_tracks[sec] = (stamp, lo, hi, origin, count)
            if count >= CONFIRM_CAPTURES:
                confirmed.append((sec, r))
        self.confirmations = next_tracks
        if not confirmed:
            return None
        sec, r = min(confirmed, key=lambda x: x[1])
        return {"sector": sec, "range": round(r, 2), "captures": CONFIRM_CAPTURES}
