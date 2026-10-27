"""Terminal criterion of a run (REFEREE side: ground truth and the controllers' logs, never fed to a controller).

A run ends with one of three outcomes instead of at a fixed duration:

* FAILURE  - a safety violation is demonstrated: two drones closer than d_safe (P1), two drones in a critical region
             (P2), a contact (drones, gate, the non-cooperative vehicle) or that vehicle closer than d_safe;
* SUCCESS  - every required outcome of the scenario is reached, judged against its committed CLEAN reference run:
             every healthy drone has come within PROGRESS_TOL_M of the position its slot had at the end of the
             reference (the mission progress the reference shows), every healthy drone has passed the gate, the
             formation is recovered (or, with a permanent loss, the remaining drones have declared the slot vacant and
             re-formed), and no healthy drone is still waiting for a missing neighbour;
* DEADLINE - none of the above by the deadline.  The deadline comes from the CLEAN completion time: the CLEAN runs
             of the campaign run until SUCCESS within a long exploratory horizon (CLEAN_HORIZON_FACTOR x T_ref +
             CLEAN_HORIZON_MARGIN_S) and measure T_clean, the time at which every required outcome is reached (the
             slowest seed); a degraded run gets deadline = DEADLINE_FACTOR x T_clean + DEADLINE_MARGIN_S, the room
             left for the delays a degraded sensor introduces (FAILSAFE stops, reduced progress while a neighbour is
             missing, a longer gate queue).  T_ref, the duration of the committed reference run, is often shorter than
             T_clean: e.g. a lost drone's slot is declared vacant only after t_rejoin of moving formation time, which
             the reference does not reach when its plan holds the clock at the gate.

SUCCESS is never declared before T_ref, so a degraded run is never judged on a shorter horizon than its reference;
a run that has not completed by the deadline is a liveness failure only if the deadline is reached (an infrastructure
interruption is INVALID, see robustness_metrics).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np

PROGRESS_TOL_M = 3.0
DEADLINE_FACTOR = 1.5
DEADLINE_MARGIN_S = 20.0
CLEAN_HORIZON_FACTOR = 3.0
CLEAN_HORIZON_MARGIN_S = 30.0
WAITING = ("FORMATION_WAIT_REJOIN",)


def deadline_s(t_clean: float) -> float:
    """Deadline of a degraded run from the CLEAN completion time."""
    return DEADLINE_FACTOR * float(t_clean) + DEADLINE_MARGIN_S


def clean_horizon_s(t_ref: float) -> float:
    """Horizon of a CLEAN run (it ends at SUCCESS, normally long before)."""
    return CLEAN_HORIZON_FACTOR * float(t_ref) + CLEAN_HORIZON_MARGIN_S


class TerminalMonitor:
    def __init__(self, scenario, cfg, names: Sequence[str], reference: Dict, t_ref: float, deadline: float):
        self.sc, self.cfg, self.names = scenario, cfg, list(names)
        self.ref = reference
        self.t_ref = float(t_ref)
        self.deadline = float(deadline)
        faults = getattr(scenario.sim, "faults", ()) or ()
        self.permanent = {f["drone"] for f in faults if f.get("t_off") is None}
        self.healthy = [k for k, n in enumerate(self.names) if n not in self.permanent]
        self.end = [np.asarray(e, float) for e in reference["end"]] if reference and reference.get("end") else None
        self.reached: Dict[int, Optional[float]] = {k: None for k in self.healthy}
        self.gate = scenario.judged_gates[0] if scenario.judged_gates else None
        self.before = {k: False for k in self.healthy}
        self.passed: Dict[int, Optional[float]] = {k: None for k in self.healthy}
        self.outcome: Optional[str] = None
        self.t_end: Optional[float] = None
        self.reason = ""

    def _failure(self, referee) -> Optional[str]:
        if referee.p1_violations:
            return "P1 violated"
        if referee.p2_violations:
            return "P2 violated"
        if referee.contacts or referee.collision_edges or referee.intruder_contacts:
            return "contact"
        if referee.intruder_min_t is not None and referee.intruder_min_d < self.cfg.sep.d_safe:
            return "non-cooperative vehicle closer than d_safe"
        return None

    def update(self, t: float, P: np.ndarray, referee, records: List[Dict]) -> Optional[str]:
        """Return SUCCESS / FAILURE / DEADLINE once, then keep returning it."""
        if self.outcome is not None:
            return self.outcome
        why = self._failure(referee)
        if why:
            return self._finish("FAILURE", t, why)
        for k in self.healthy:
            if self.end is not None and self.reached[k] is None and np.linalg.norm(P[k] - self.end[k]) <= PROGRESS_TOL_M:
                self.reached[k] = t
            if self.gate is not None and self.passed[k] is None:
                s = self.gate.to_gate_frame(P[k])[0]
                G = self.cfg.gate
                self.before[k] = self.before[k] or s < -G.cr_half_len
                if self.before[k] and s > G.exit_s:
                    self.passed[k] = t
        if t >= self.t_ref - 1e-9 and self._success(referee, records):
            return self._finish("SUCCESS", t, "every required outcome reached")
        if t >= self.deadline - 1e-9:
            return self._finish("DEADLINE", t, "required outcomes not all reached: " + ", ".join(self.missing(referee, records)))
        return None

    def finalize(self, t: float, referee, records: List[Dict]) -> str:
        """End of the horizon without an outcome: the deadline (the loop's last step falls one step short of it)."""
        if self.outcome is None:
            self._finish("DEADLINE", t, "required outcomes not all reached: " + ", ".join(self.missing(referee, records)))
        return self.outcome

    def missing(self, referee, records) -> List[str]:
        out = []
        if self.end is not None and any(v is None for v in self.reached.values()):
            out.append("progress")
        if self.gate is not None and any(v is None for v in self.passed.values()):
            out.append("gate")
        live = referee.live()
        if self.ref.get("P3_recovered") and live.get("formation") != "OK":
            out.append("P3 recovered")
        if self.ref.get("P3_degraded_holds"):
            if live.get("formation_present") != "OK":
                out.append("P3-deg re-formed")
            slots = {pl.slot_index for pl in self.sc.plans if pl.drone_id in self.permanent}
            for k in self.healthy:
                vac = set(((records[k] or {}).get("form") or {}).get("vacant") or [])
                if not slots <= vac:
                    out.append("P3-deg vacancy")
                    break
        if any((records[k] or {}).get("mode") in WAITING for k in self.healthy):
            out.append("still waiting for a missing neighbour")
        return out

    def _success(self, referee, records) -> bool:
        return not self.missing(referee, records)

    def _finish(self, outcome: str, t: float, reason: str) -> str:
        self.outcome, self.t_end, self.reason = outcome, round(float(t), 2), reason
        return outcome

    def summary(self) -> Dict:
        return {"outcome": self.outcome or "INCOMPLETE", "t_end": self.t_end, "reason": self.reason,
                "t_ref": self.t_ref, "deadline_s": self.deadline, "progress_tol_m": PROGRESS_TOL_M,
                "reached_reference_end_t": {self.names[k]: v for k, v in self.reached.items()},
                "gate_passed_t": {self.names[k]: v for k, v in self.passed.items()} if self.gate is not None else None}
