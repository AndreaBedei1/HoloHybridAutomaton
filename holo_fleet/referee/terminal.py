"""Terminal criterion of a run (REFEREE side: ground truth and the controllers' logs, never fed to a controller).

A run ends with one of three outcomes instead of at a fixed duration:

* FAILURE  - a safety violation is demonstrated: two drones closer than d_safe (P1), two drones in a critical region
             (P2), a contact (drones, gate, the non-cooperative vehicle) or that vehicle closer than d_safe;
* SUCCESS  - every requirement of criteria v3 (referee/criteria.py) holds at the same time, not before the
             scenario's nominal end T_nom:
             - progress: every healthy drone is within progress_tol_m of its PLANNED end position (now, not once);
             - gate: every healthy drone has passed the judged gate THROUGH the critical region (before it, inside
               the CR at some instant, beyond the exit); a drone that went around or over the frame has not;
             - P3: the formation (with a permanent loss: that of the healthy drones on their original slots) has no
               open loss episode, the error being inside the E_TOL_M band for t_ok_hold;
             - P3-deg vacancy: with a permanent loss, every healthy drone that expects the lost drone as a neighbour
               has declared its slot vacant;
             - no false vacancy: no healthy drone holds vacant the slot of a drone that is not permanently faulted;
             - no healthy drone is still waiting for a missing neighbour;
* DEADLINE - none of the above by the deadline (criteria.deadline_v3: T_nom plus the protocol's worst-case delays).

The same monitor is replayed offline on logged runs (robustness_metrics.replay_terminal), so the online stop and the
offline verdict use one requirement set (v2 checked the false vacancy only offline).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.referee.criteria import (CRITERIA_VERSION, E_TOL_M, expects, formation_judged, nominal_end_s,
                                         permanent_faults, plan_targets, progress_tol_m)

WAITING = ("FORMATION_WAIT_REJOIN",)
REQUIREMENTS = ("progress", "gate", "P3 recovered", "P3-deg re-formed", "P3-deg vacancy", "no false vacancy",
                "not waiting")


class TerminalMonitor:
    def __init__(self, scenario, cfg, names: Sequence[str], deadline: float, t_nom: Optional[float] = None):
        self.sc, self.cfg, self.names = scenario, cfg, list(names)
        self.t_nom = nominal_end_s(scenario) if t_nom is None else float(t_nom)
        self.deadline = float(deadline)
        self.permanent = permanent_faults(scenario)
        self.healthy = [k for k, n in enumerate(self.names) if n not in self.permanent]
        self.targets = plan_targets(scenario, self.t_nom)
        self.tol = progress_tol_m(cfg)
        self.reached: Dict[int, Optional[float]] = {k: None for k in self.healthy}      # first time (report only)
        self.near: Dict[int, bool] = {k: False for k in self.healthy}                   # within tol now
        self.gate = scenario.judged_gates[0] if scenario.judged_gates else None
        self.before = {k: False for k in self.healthy}
        self.crossed = {k: False for k in self.healthy}
        self.passed: Dict[int, Optional[float]] = {k: None for k in self.healthy}
        self.bypassed: List[str] = []
        lost_idx = [k for k, n in enumerate(self.names) if n in self.permanent]
        self.lost_slots = {pl.slot_index for pl in scenario.plans if pl.drone_id in self.permanent}
        # the drones that must declare a lost slot vacant: those that expect its drone as a neighbour
        self.must_vacate = {k: {scenario.plans[j].slot_index for j in lost_idx if expects(scenario, cfg, k, j)}
                            for k in self.healthy}
        self.slot_drone = {pl.slot_index: self.names.index(pl.drone_id) for pl in scenario.plans}
        self.judge_formation = formation_judged(scenario)
        self.outcome: Optional[str] = None
        self.t_end: Optional[float] = None
        self.reason = ""
        self.last_missing: List[str] = []
        self.met_since: Dict[str, Optional[float]] = {r: None for r in REQUIREMENTS}
        # a scenario whose mission is physically infeasible by design (a drone stalled in the gate) is judged on
        # safety over the whole horizon only: no requirement can be asked of the drones it blocks
        self.safety_only = bool(getattr(scenario, "safety_only", False))

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
        G = self.cfg.gate
        for k in self.healthy:
            self.near[k] = bool(np.linalg.norm(P[k] - self.targets[k]) <= self.tol)
            if self.reached[k] is None and self.near[k]:
                self.reached[k] = t
            if self.gate is not None and self.passed[k] is None:
                s, l, dz = self.gate.to_gate_frame(P[k])
                self.before[k] = self.before[k] or s < -G.cr_half_len
                if (self.before[k] and abs(s) <= G.cr_half_len and abs(l) <= G.cr_half_width
                        and abs(dz) <= G.cr_half_height):
                    self.crossed[k] = True
                if self.before[k] and s > G.exit_s:
                    if self.crossed[k]:
                        self.passed[k] = t
                    elif self.names[k] not in self.bypassed:
                        self.bypassed.append(self.names[k])          # around or over the frame: not a passage
        miss = self.missing(referee, records, P)
        for r in REQUIREMENTS:
            if r in miss:
                self.met_since[r] = None
            elif self.met_since[r] is None:
                self.met_since[r] = round(float(t), 2)
        self.last_missing = miss
        if self.safety_only:
            self.last_missing = []                  # nothing is required of drones the stalled one blocks
            if t >= self.deadline - 1e-9:
                return self._finish("SUCCESS", t, "safety held over the whole horizon (mission infeasible by design)")
            return None
        if t >= self.t_nom - 1e-9 and not miss:
            return self._finish("SUCCESS", t, "every required outcome reached")
        if t >= self.deadline - 1e-9:
            return self._finish("DEADLINE", t, "required outcomes not all reached: " + ", ".join(miss))
        return None

    def finalize(self, t: float, referee, records: List[Dict]) -> str:
        """End of the horizon without an outcome: the deadline (the loop's last step falls one step short of it)."""
        if self.outcome is None and self.safety_only:
            self._finish("SUCCESS", t, "safety held over the whole horizon (mission infeasible by design)")
        if self.outcome is None:
            self._finish("DEADLINE", t, "required outcomes not all reached: " + ", ".join(self.last_missing))
        return self.outcome

    def missing(self, referee, records, P: Optional[np.ndarray] = None) -> List[str]:
        out = []
        if not all(self.near.values()):
            out.append("progress")
        if self.gate is not None and any(v is None for v in self.passed.values()):
            out.append("gate")
        live = referee.live()
        if self.judge_formation:
            if not self.permanent:
                if live.get("formation") != "OK":
                    out.append("P3 recovered")
            else:
                if live.get("formation_present") != "OK":
                    out.append("P3-deg re-formed")
                for k in self.healthy:
                    vac = set(((records[k] or {}).get("form") or {}).get("vacant") or [])
                    if not self.must_vacate[k] <= vac:
                        out.append("P3-deg vacancy")
                        break
            for k in self.healthy:                          # ground truth: only a permanently faulted drone is lost
                vac = set(((records[k] or {}).get("form") or {}).get("vacant") or [])
                if any(self.slot_drone.get(sl) in self.healthy for sl in vac):
                    out.append("no false vacancy")
                    break
        if any((records[k] or {}).get("mode") in WAITING for k in self.healthy):
            out.append("not waiting")
        return out

    def _finish(self, outcome: str, t: float, reason: str) -> str:
        self.outcome, self.t_end, self.reason = outcome, round(float(t), 2), reason
        return outcome

    def summary(self) -> Dict:
        return {"criteria": CRITERIA_VERSION, "outcome": self.outcome or "INCOMPLETE", "t_end": self.t_end,
                "reason": self.reason, "missing": self.last_missing, "t_nom": self.t_nom, "deadline_s": self.deadline,
                "progress_tol_m": round(self.tol, 2), "e_tol_m": E_TOL_M, "gate_bypass": list(self.bypassed),
                "reached_plan_target_t": {self.names[k]: v for k, v in self.reached.items()},
                "gate_passed_t": {self.names[k]: v for k, v in self.passed.items()} if self.gate is not None else None,
                "requirement_met_since": dict(self.met_since)}
