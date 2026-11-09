"""Onboard verification of the drone's own passage through a gate (payload v2, gate phase GP_PASSED).

The gate phase a drone reports must not rest on the long-term dead reckoning: its drift is exactly what breaks the rest
of the gate logic under MEDIUM sensors.  Since the drone was admitted (stage 2), a passage is verified by what its own
sensors observe of the gate itself:

* the CROSSING signature: while passing the gate plane the LEFT and RIGHT sonars both see the frame posts at a sum of ranges
  that matches the opening (L + R = inner width - the sonar mounts, within ``CROSS_TOL_M``), for ``CROSS_N`` captures: a
  direct measurement of being inside the opening, independent of the navigation estimate (in the MEDIUM logs of v10 both
  sides are seen in 94 % of the captures at |s| <= 0.2 m).  It also gives the lateral position in the gate (R - L) / 2
  and the vertical one (D - U) / 2, measured, not estimated.  Then the along-axis DVL displacement carries the drone
  beyond the exit face: verified.
* otherwise (the signature was missed: a fade): the DVL displacement from the map-validated hold point beyond the exit face, the
  navigation estimate agreeing, and a REAR sonar echo of the frame at the range the displacement predicts (97 % of the captures
  in the MEDIUM logs), for ``REAR_N`` captures;
* failing that too (a sonar fault), the displacement and the estimate alone after ``T_ODO_MAX_S``, so that a sonar fault
  does not stall the fleet behind a drone that has passed: only the odometry over the last few metres from a map-validated
  start is then relied on.

A drone that never reaches this point (stopped by a bar, or holding, or it dies) keeps reporting GP_ADMITTED and the gate stays
closed behind it.  ``verified`` is monotone; each gate is verified at most once.
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import numpy as np

from holo_fleet.perception.payload import GP_ADMITTED, GP_COMMITTED, GP_NONE, GP_PASSED
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT

CROSS_TOL_M = 0.25                # L + R agrees with the opening within this
CROSS_N = 3                       # consecutive captures with the signature
CROSS_MAX_RANGE_M = 1.6           # the posts are seen this close
CROSS_S_WINDOW_M = 1.2            # only near the gate plane, by the odometry from the hold point
REAR_RANGE_OFFSET_M = 0.50        # the REAR echo of the frame is this much shorter than the own distance to the gate plane
REAR_TOL_M = 0.50                 # the echo agrees within this (DVL scale and heading error over the last metres, sonar jitter)
REAR_N = 3                        # consecutive captures with a matching REAR echo
T_ODO_MAX_S = 10.0                # without any sonar evidence, the odometry and the estimate suffice after this long
CROSSING_RESTART_M = 0.6          # horizontal error bound at a detected crossing: along the axis, the signature lasts
                                  # |s| <~ 0.4 m; laterally the measured (R - L) / 2 has sd 0.03 m in the MEDIUM logs


def inside_keepout(s: float, l: float, dz: float, G) -> bool:
    """The own gate-frame estimate is near enough to the CR that the drone may really be in it (hull ball plus the declared
    gate-frame navigation error)."""
    m = R_OUT + G.nav_err_m
    return abs(s) <= G.cr_half_len + m and abs(l) <= G.cr_half_width + m and abs(dz) <= G.cr_half_height + m


def report_phase(verified: bool, in_progress: bool, committed: bool, in_keepout: bool) -> int:
    """The gate phase a drone reports about itself.  A drone inside the keep-out box that is not committed (carried there by a
    current or an escape, or back after a passage) cannot say "not engaged" or "passed": the others would exclude it as an
    occupant.  It reports a claim, which can only hold the gate BUSY (formal B2-PE and S9)."""
    if in_progress:
        return GP_ADMITTED
    if in_keepout:
        return GP_COMMITTED
    if verified:
        return GP_PASSED
    return GP_COMMITTED if committed else GP_NONE


class PassageVerifier:
    def __init__(self, cfg):
        self.cfg = cfg
        self.idx = 0                       # index of the gate tracked
        self.gate = None                   # its GateSpec
        self.admitted = False
        self.verified = False
        self.by = ""                       # "crossing" | "rear" | "odometry"
        self.s0 = 0.0
        self.d = 0.0
        self.t_odo: Optional[float] = None
        self.rear_hits = 0
        self.cross_hits = 0
        self.crossing: Optional[tuple] = None   # (l, dz or None) measured inside the opening; None until detected

    @property
    def in_progress(self) -> bool:
        return self.admitted and not self.verified

    def next_gate(self, idx: int) -> None:
        self.__init__(self.cfg)
        self.idx = idx

    def admit(self, idx: int, gate, p_est) -> None:
        """The drone is admitted to the CR now: the gate-frame position is validated against the map."""
        if self.admitted:
            return
        self.idx, self.gate, self.admitted = idx, gate, True
        self.s0 = float(gate.to_gate_frame(np.asarray(p_est, float))[0])
        self.d = 0.0

    def _crossing_now(self, sectors: Dict[str, Sequence[float]]) -> Optional[tuple]:
        G = self.gate
        near = {s: [float(r) for r in sectors.get(s, ()) if float(r) <= CROSS_MAX_RANGE_M] for s in ("LEFT", "RIGHT", "UP", "DOWN")}
        if not near["LEFT"] or not near["RIGHT"]:
            return None
        L, R = min(near["LEFT"]), min(near["RIGHT"])
        width = float(G.inner_width) - abs(float(MOUNTS["LEFT"][1])) - abs(float(MOUNTS["RIGHT"][1]))
        if abs(L + R - width) > CROSS_TOL_M:
            return None
        dz = (min(near["DOWN"]) - min(near["UP"])) / 2.0 if near["UP"] and near["DOWN"] else None
        return (R - L) / 2.0, dz

    def update(self, t: float, dt: float, p_est, v_world, sectors: Dict[str, Sequence[float]], dvl_fresh: bool = True) -> bool:
        """``sectors``: the echo ranges of this step for the sonars LEFT, RIGHT, UP, DOWN and REAR.  Only DVL measurements
        integrate the displacement (``dvl_fresh``: during a DVL outage it is not advanced, never using the model velocity)."""
        if not self.in_progress or self.gate is None:
            return self.verified
        G = self.cfg.gate
        axis = np.asarray(self.gate.axis, float)
        if dvl_fresh:
            self.d += float(np.asarray(v_world, float)[:2] @ axis[:2]) * dt
        s_odo = self.s0 + self.d
        s_nav = float(self.gate.to_gate_frame(np.asarray(p_est, float))[0])
        if self.crossing is None and abs(s_odo) <= CROSS_S_WINDOW_M:
            c = self._crossing_now(sectors)
            self.cross_hits = self.cross_hits + 1 if c is not None else 0
            if self.cross_hits >= CROSS_N:
                self.crossing = c                  # measured position in the opening; from now on s_odo restarts at the plane
                self.s0 = -self.d
                s_odo = 0.0
        if self.crossing is not None and s_odo >= G.exit_s:
            self.verified, self.by = True, "crossing"
            return True
        if s_odo < G.exit_s or s_nav < G.exit_s:
            return False
        self.t_odo = t if self.t_odo is None else self.t_odo
        expected = s_odo - REAR_RANGE_OFFSET_M
        self.rear_hits = self.rear_hits + 1 if any(abs(float(r) - expected) <= REAR_TOL_M for r in sectors.get("REAR", ())) else 0
        if self.rear_hits >= REAR_N:
            self.verified, self.by = True, "rear"
        elif t - self.t_odo >= T_ODO_MAX_S:
            self.verified, self.by = True, "odometry"
        return self.verified
