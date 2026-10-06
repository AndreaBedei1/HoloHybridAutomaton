"""Ground-truth referee / validator.

The referee is the ONLY component that reads simulator ground truth.  It is
fed by the simulation runner and never returns anything to the controllers.
It computes the empirical verdicts for P1 (separation), P2 (critical-region
mutual exclusion) and P3 (bounded formation recovery).
"""

from __future__ import annotations

import itertools
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.mission import GateSpec, MissionPlan


@dataclass
class Episode:
    t_lost: float
    t_recovered: Optional[float] = None
    cause: str = ""
    t_calm: Optional[float] = None          # start of the last perturbation-free interval

    @property
    def duration(self) -> Optional[float]:
        return None if self.t_recovered is None else self.t_recovered - self.t_lost

    @property
    def recovery_after_perturbation(self) -> Optional[float]:
        if self.t_recovered is None or self.t_calm is None:
            return None
        return self.t_recovered - self.t_calm


class Referee:
    def __init__(self, plans: List[MissionPlan], gates: List[GateSpec], cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.plans = plans
        self.names = [p.drone_id for p in plans]
        self.gates = gates
        self.rows: List[Dict] = []
        self.pairs = list(itertools.combinations(range(len(plans)), 2))
        self.min_d = {p: 1e9 for p in self.pairs}
        self.min_d_t = {p: None for p in self.pairs}
        self.p1_violations: List[Dict] = []
        self.p1_collisions: List[Dict] = []
        self.p2_violations: List[Dict] = []
        self.occupancy_log: Dict[str, List[str]] = {g.gate_id: [] for g in gates}   # entry order
        self._inside_prev: Dict[str, set] = {g.gate_id: set() for g in gates}
        self.collision_sensor_events: List[Dict] = []
        self._coll_prev = np.zeros(len(plans), dtype=bool)
        self.episodes: List[Episode] = []
        self._lost = False
        self._ok_since: Optional[float] = None
        self.envelope_flags: List[Dict] = []
        self.max_drift = 0.0
        self.formation_enabled = all(p.formation_enabled for p in plans) and len(plans) > 1
        self._prev_pos = None
        self.max_speed = 0.0

    # ------------------------------------------------------------------
    def formation_error(self, P: np.ndarray) -> float:
        """Max deviation of each drone from the best-translated formation template (path frame)."""
        plan = self.plans[0]
        slots = plan.slots
        centroid = P.mean(axis=0)
        s_c, _, _ = plan.path.project(centroid[:2])
        _, tan, nrm = plan.path.frame_at(s_c)
        rel = P - centroid
        rel_pf = np.stack([rel[:, :2] @ tan, rel[:, :2] @ nrm, rel[:, 2]], axis=1)
        tmpl = np.array([[s.along, s.lateral, s.dz] for s in slots])
        tmpl = tmpl - tmpl.mean(axis=0)
        return float(np.max(np.linalg.norm(rel_pf - tmpl, axis=1)))

    def update(self, truth, modes: Optional[List[str]] = None, perturbation: bool = False) -> Dict:
        """``perturbation`` = some drone is in a gate/avoidance/failsafe mode or inside a localized
        current disturbance.  P3 is a bounded-recovery property *after* the perturbation ends."""
        sep, F = self.cfg.sep, self.cfg.form
        t = truth.t
        P = truth.positions
        row: Dict = {"t": round(t, 3)}
        for (i, j) in self.pairs:
            d = float(np.linalg.norm(P[i] - P[j]))
            row[f"d_{i}{j}"] = round(d, 4)
            if d < self.min_d[(i, j)]:
                self.min_d[(i, j)], self.min_d_t[(i, j)] = d, t
            if truth.released[i] and truth.released[j]:
                if d < sep.d_safe:
                    self.p1_violations.append({"t": t, "pair": [self.names[i], self.names[j]], "d": d})
                if d < sep.d_collision:
                    self.p1_collisions.append({"t": t, "pair": [self.names[i], self.names[j]], "d": d})
        # collision sensor rising edges
        for k in range(len(self.names)):
            if truth.collision[k] and not self._coll_prev[k]:
                self.collision_sensor_events.append({"t": t, "drone": self.names[k],
                                                     "nearest_drone_d": float(min(
                                                         [np.linalg.norm(P[k] - P[m]) for m in range(len(P)) if m != k] or [1e9]))})
        self._coll_prev = truth.collision.copy()
        # critical-region occupancy (true positions)
        for g in self.gates:
            G = self.cfg.gate
            inside = set()
            for k in range(len(self.names)):
                s, l, dz = g.to_gate_frame(P[k])
                if abs(s) <= G.cr_half_len and abs(l) <= G.cr_half_width and abs(dz) <= G.cr_half_height:
                    inside.add(k)
            row[f"occ_{g.gate_id}"] = len(inside)
            for k in sorted(inside - self._inside_prev[g.gate_id]):
                self.occupancy_log[g.gate_id].append(self.names[k])
            if len(inside) > 1:
                self.p2_violations.append({"t": t, "gate": g.gate_id, "inside": [self.names[k] for k in sorted(inside)]})
            self._inside_prev[g.gate_id] = inside
        # formation (P3)
        if self.formation_enabled and all(truth.released):
            fe = self.formation_error(P)
            row["form_err"] = round(fe, 4)
            row["perturbation"] = int(perturbation)
            if not self._lost and fe > F.e_lost:
                self._lost = True
                self._ok_since = None
                self.episodes.append(Episode(t_lost=t, cause=",".join(sorted(set(modes or [])))))
            if self._lost:
                ep = self.episodes[-1]
                if perturbation:
                    ep.t_calm = None
                elif ep.t_calm is None:
                    ep.t_calm = t
                if fe < F.e_ok:
                    self._ok_since = t if self._ok_since is None else self._ok_since
                    if t - self._ok_since >= F.t_ok_hold:
                        self.episodes[-1].t_recovered = t
                        self._lost = False
                else:
                    self._ok_since = None
        # currents / envelope
        for k in range(len(self.names)):
            w = float(np.linalg.norm(truth.current_drift[k]))
            row[f"drift_{k}"] = round(w, 3)
            self.max_drift = max(self.max_drift, w)
            if w > self.cfg.env.current_drift_max + 1e-9:
                if not self.envelope_flags or t - self.envelope_flags[-1]["t"] > 5.0:
                    self.envelope_flags.append({"t": t, "drone": self.names[k], "drift": w,
                                                "limit": self.cfg.env.current_drift_max})
        for k in range(len(self.names)):
            row[f"x_{k}"], row[f"y_{k}"], row[f"z_{k}"] = (round(float(v), 4) for v in P[k])
            row[f"yaw_{k}"] = round(float(truth.yaw_deg[k]), 2)
        if self._prev_pos is not None:
            sp = np.linalg.norm(P - self._prev_pos, axis=1) / max(t - self._prev_t, 1e-6)
            self.max_speed = max(self.max_speed, float(sp.max()))
        self._prev_pos, self._prev_t = P.copy(), t
        self.rows.append(row)
        return row

    # ------------------------------------------------------------------
    def metrics(self) -> Dict:
        sep, F = self.cfg.sep, self.cfg.form
        rec_times = [e.recovery_after_perturbation for e in self.episodes if e.recovery_after_perturbation is not None]
        unrecovered = [e for e in self.episodes if e.t_recovered is None]
        p3_late = [e for e in self.episodes
                   if e.recovery_after_perturbation is not None and e.recovery_after_perturbation > F.t_recovery_max]
        min_overall = min(self.min_d.values()) if self.min_d else None
        out = {
            "P1_separation": {
                "formula": "G( forall i!=j : d_ij >= d_safe )",
                "d_safe": sep.d_safe, "d_warning": sep.d_warning, "d_collision": sep.d_collision,
                "min_distance_overall": None if min_overall is None else round(min_overall, 4),
                "min_distance_per_pair": {f"{self.names[i]}-{self.names[j]}": {"d_min": round(v, 4), "t": self.min_d_t[(i, j)]}
                                          for (i, j), v in self.min_d.items()},
                "violations_d_lt_d_safe": len(self.p1_violations),
                "first_violation": self.p1_violations[0] if self.p1_violations else None,
                "physical_contacts_d_lt_d_collision": len(self.p1_collisions),
                "collision_sensor_rising_edges": self.collision_sensor_events,
                "holds": len(self.p1_violations) == 0,
            },
            "P2_mutual_exclusion": {
                "formula": "G( sum_i inside_CR_i <= 1 )",
                "gates": [g.gate_id for g in self.gates],
                "max_occupancy": {g.gate_id: int(max([r.get(f"occ_{g.gate_id}", 0) for r in self.rows] or [0]))
                                  for g in self.gates},
                "entry_order": self.occupancy_log,
                "violations": len(self.p2_violations),
                "first_violation": self.p2_violations[0] if self.p2_violations else None,
                "all_drones_traversed": {g.gate_id: sorted(set(self.occupancy_log[g.gate_id])) == sorted(self.names)
                                         for g in self.gates},
                "holds": len(self.p2_violations) == 0 if self.gates else None,
            },
            "P3_formation_recovery": {
                "formula": "G( formation_lost -> F_[0,T] formation_recovered )",
                "T_s": F.t_recovery_max, "e_lost": F.e_lost, "e_ok": F.e_ok, "t_ok_hold": F.t_ok_hold,
                "enabled": self.formation_enabled,
                "episodes": [{"t_lost": e.t_lost, "t_perturbation_end": e.t_calm, "t_recovered": e.t_recovered,
                              "episode_duration": e.duration, "recovery_after_perturbation": e.recovery_after_perturbation,
                              "modes_at_loss": e.cause}
                             for e in self.episodes],
                "n_episodes": len(self.episodes),
                "max_recovery_time_s": max(rec_times) if rec_times else None,
                "max_episode_duration_s": max([e.duration for e in self.episodes if e.duration is not None], default=None),
                "unrecovered_at_end": len(unrecovered),
                "late_recoveries": len(p3_late),
                "final_form_err": self.rows[-1].get("form_err") if self.rows else None,
                "holds": (len(p3_late) == 0 and len(unrecovered) == 0) if self.formation_enabled else None,
            },
            "envelope": {
                "current_drift_max_claimed": self.cfg.env.current_drift_max,
                "max_effective_drift_applied": round(self.max_drift, 3),
                "out_of_envelope_flags": self.envelope_flags,
                "inside_envelope": len(self.envelope_flags) == 0,
            },
            "kinematics": {"max_true_speed_m_s": round(self.max_speed, 3)},
        }
        return out
