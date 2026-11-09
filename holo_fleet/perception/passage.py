"""Onboard verification of the drone's own passage through a gate (payload v2, gate phase GP_PASSED).

The gate phase a drone reports must not rest on the long-term dead reckoning: its drift is exactly what breaks the rest
of the gate logic under MEDIUM sensors.  A passage is VERIFIED when, since the drone was admitted (stage 2, a gate
localization validated against the map at the hold point, Flows.gate):

* its along-axis DVL displacement carries it beyond the exit face (``exit_s``), and its navigation estimate agrees, and
* its REAR sonar shows an echo of the gate frame at the range that displacement predicts: a physical observation of the
  frame BEHIND it (in the MEDIUM logs of v10 such an echo is in 97 % of the captures past the exit), counted over
  ``REAR_N`` captures.

If the REAR echo does not come (a sonar fault, a fade), the displacement and the navigation estimate alone verify the
passage after ``T_ODO_MAX_S``, so that a sonar fault cannot stall the fleet behind a drone that has passed: only the odometry
over the last few metres, from a map-validated start, is then relied on.  A drone that never reaches this point (it is
stopped by a bar, avoidance holds it, or it dies) keeps reporting GP_ADMITTED and the gate stays closed behind it.

Each gate is verified at most once; ``verified`` is monotone.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

REAR_RANGE_OFFSET_M = 0.50        # the REAR echo of the frame is this much shorter than the own distance to the gate plane
                                  # (the sonar mount and the bar half-depth, measured in the MEDIUM logs: 0.4 - 0.6 m)
REAR_TOL_M = 0.80                 # the echo agrees within this (DVL scale and heading error over the last metres, sonar jitter)
REAR_N = 3                        # captures with a matching REAR echo
T_ODO_MAX_S = 10.0                # without a REAR echo, the odometry and the estimate suffice after this long


class PassageVerifier:
    def __init__(self, cfg):
        self.cfg = cfg
        self.idx = 0                       # index of the gate tracked
        self.gate = None                   # its GateSpec
        self.admitted = False
        self.verified = False
        self.by = ""                       # "rear" | "odometry"
        self.s0 = 0.0
        self.d = 0.0
        self.t_odo: Optional[float] = None
        self.hits = 0

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

    def update(self, t: float, dt: float, p_est, v_world, rear_ranges: Sequence[float]) -> bool:
        if not self.in_progress or self.gate is None:
            return self.verified
        G = self.cfg.gate
        axis = np.asarray(self.gate.axis, float)
        self.d += float(np.asarray(v_world, float)[:2] @ axis[:2]) * dt
        s_odo = self.s0 + self.d
        s_nav = float(self.gate.to_gate_frame(np.asarray(p_est, float))[0])
        if s_odo < G.exit_s or s_nav < G.exit_s:
            return False
        self.t_odo = t if self.t_odo is None else self.t_odo
        expected = s_odo - REAR_RANGE_OFFSET_M
        if any(abs(float(r) - expected) <= REAR_TOL_M for r in rear_ranges):
            self.hits += 1
        if self.hits >= REAR_N:
            self.verified, self.by = True, "rear"
        elif t - self.t_odo >= T_ODO_MAX_S:
            self.verified, self.by = True, "odometry"
        return self.verified
