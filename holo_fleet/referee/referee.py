"""Ground-truth referee / validator (v2, generic N).

The referee is the ONLY component that reads simulator ground truth.  It is fed by the runner and
never returns anything to a controller.  It judges, on agent-origin (hull-centre) positions:

* P1  G(forall i != j: d_ij >= d_safe)          - minimum distance per pair, violations, contacts;
* P2  G(sum_i inside_CR_i <= 1)                  - occupancy of every critical region, entry order;
* P3  G(formation_lost -> F formation_recovered) - episodes of formation loss with their recovery
  times.  A finite run cannot falsify a liveness property: an episode still open at the end of a
  run is reported as "not recovered within the run", never as a counterexample.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.mission import GateSpec

PERTURBING_MODES = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT",
                    "MUTEX_APPROACH", "MUTEX_YIELD", "MUTEX_PASS")


@dataclass
class Episode:
    t_lost: float
    t_recovered: Optional[float] = None
    t_perturbation_end: Optional[float] = None
    modes_at_loss: str = ""

    @property
    def recovery_time(self) -> Optional[float]:
        return None if self.t_recovered is None else self.t_recovered - self.t_lost

    @property
    def recovery_after_perturbation(self) -> Optional[float]:
        if self.t_recovered is None or self.t_perturbation_end is None:
            return None
        return max(0.0, self.t_recovered - self.t_perturbation_end)


class Referee:
    def __init__(self, names: Sequence[str], template_offsets: Optional[np.ndarray], path, gates: Sequence[GateSpec],
                 cfg: FleetConfig = DEFAULT, formation_enabled: bool = True):
        self.cfg = cfg
        self.names = list(names)
        self.n = len(self.names)
        self.tmpl = None if template_offsets is None else np.asarray(template_offsets, float)
        self.path = path
        self.gates = list(gates)
        self.formation_enabled = formation_enabled and self.tmpl is not None and self.n > 1
        self.pairs = list(itertools.combinations(range(self.n), 2))
        self.min_d = {p: 1e9 for p in self.pairs}
        self.min_d_t = {p: None for p in self.pairs}
        self.p1_violations: List[Dict] = []
        self.contacts: List[Dict] = []
        self.collision_edges: List[Dict] = []
        self._coll_prev = np.zeros(self.n, dtype=bool)
        self.p2_violations: List[Dict] = []
        self.entry_order: Dict[str, List[str]] = {g.gate_id: [] for g in self.gates}
        self.max_occ: Dict[str, int] = {g.gate_id: 0 for g in self.gates}
        self._inside_prev: Dict[str, set] = {g.gate_id: set() for g in self.gates}
        self.episodes: List[Episode] = []
        self._lost = False
        self._ok_since: Optional[float] = None
        self._last_perturbation = -1e9
        self.max_drift = 0.0
        self.max_vertical_drift = 0.0
        self.max_speed = 0.0
        self.rows: List[Dict] = []
        self.intruder_min_d = 1e9                  # fleet drone <-> scripted non-fleet vehicle
        self.intruder_min_t: Optional[float] = None
        self.intruder_contacts: List[Dict] = []

    # ------------------------------------------------------------------ formation error
    def formation_error(self, P: np.ndarray) -> float:
        """Max deviation of a drone from the template after the best translation (formation frame)."""
        if self.tmpl is None:
            return 0.0
        c = P.mean(axis=0)
        s_c, _lat, _k = self.path.project(c[:2])
        _p, tan, nrm = self.path.frame_at(s_c)
        rel = P - c
        rel_f = np.stack([rel[:, :2] @ tan, rel[:, :2] @ nrm, rel[:, 2]], axis=1)
        tm = self.tmpl - self.tmpl.mean(axis=0)
        return float(np.max(np.linalg.norm(rel_f - tm, axis=1)))

    def cr_inside(self, g: GateSpec, p: np.ndarray) -> bool:
        G = self.cfg.gate
        s, l, dz = g.to_gate_frame(p)
        return abs(s) <= G.cr_half_len and abs(l) <= G.cr_half_width and abs(dz) <= G.cr_half_height

    # ------------------------------------------------------------------ update
    def update(self, truth, modes: Sequence[str], disturbance_active: bool = False) -> Dict:
        t, P = truth.t, truth.positions
        sep = self.cfg.sep
        row = {"t": round(t, 3)}
        for (i, j) in self.pairs:
            d = float(np.linalg.norm(P[i] - P[j]))
            row[f"d_{i}_{j}"] = round(d, 4)
            if d < self.min_d[(i, j)]:
                self.min_d[(i, j)], self.min_d_t[(i, j)] = d, t
            if d < sep.d_safe:
                self.p1_violations.append({"t": t, "pair": [self.names[i], self.names[j]], "d": round(d, 3)})
            if d < sep.d_collision:
                self.contacts.append({"t": t, "pair": [self.names[i], self.names[j]], "d": round(d, 3)})
        intr = getattr(truth, "intruders", None)
        if intr is not None and len(intr):
            for m, q in enumerate(np.asarray(intr, float)):
                row[f"ix{m}"], row[f"iy{m}"], row[f"iz{m}"] = (round(float(v), 3) for v in q)
                for k in range(self.n):
                    d = float(np.linalg.norm(P[k] - q))
                    if d < self.intruder_min_d:
                        self.intruder_min_d, self.intruder_min_t = d, t
                    if d < sep.d_collision:
                        self.intruder_contacts.append({"t": t, "drone": self.names[k], "d": round(d, 3)})
            row["d_intruder"] = round(float(min(np.linalg.norm(P[k] - q) for k in range(self.n) for q in intr)), 4)
        col = np.asarray(truth.collision, dtype=bool)
        for k in np.flatnonzero(col & ~self._coll_prev):
            self.collision_edges.append({"t": t, "drone": self.names[int(k)]})
        self._coll_prev = col.copy()
        for g in self.gates:
            inside = {self.names[k] for k in range(self.n) if self.cr_inside(g, P[k])}
            occ = len(inside)
            row[f"occ_{g.gate_id}"] = occ
            self.max_occ[g.gate_id] = max(self.max_occ[g.gate_id], occ)
            for nm in sorted(inside - self._inside_prev[g.gate_id]):
                if nm not in self.entry_order[g.gate_id]:
                    self.entry_order[g.gate_id].append(nm)
            if occ > 1:
                self.p2_violations.append({"t": t, "gate": g.gate_id, "inside": sorted(inside)})
            self._inside_prev[g.gate_id] = inside
        if any(m in PERTURBING_MODES for m in modes) or disturbance_active:
            self._last_perturbation = t
        if self.formation_enabled:
            e = self.formation_error(P)
            row["form_err"] = round(e, 4)
            rc = self.cfg.ref
            if not self._lost and e > rc.e_lost:
                self._lost = True
                self.episodes.append(Episode(t_lost=t, modes_at_loss=",".join(sorted(set(modes)))))
                self._ok_since = None
            elif self._lost:
                if e < rc.e_ok:
                    self._ok_since = t if self._ok_since is None else self._ok_since
                    if t - self._ok_since >= rc.t_ok_hold:
                        ep = self.episodes[-1]
                        ep.t_recovered = self._ok_since
                        ep.t_perturbation_end = min(self._last_perturbation, self._ok_since)
                        self._lost = False
                else:
                    self._ok_since = None
        drift = np.asarray(truth.current_drift, float)
        self.max_drift = max(self.max_drift, float(np.max(np.linalg.norm(drift[:, :2], axis=1))))
        self.max_vertical_drift = max(self.max_vertical_drift, float(np.max(np.abs(drift[:, 2]))))
        self.max_speed = max(self.max_speed, float(np.max(np.linalg.norm(truth.velocities, axis=1))))
        row["max_drift"] = round(float(np.max(np.linalg.norm(drift, axis=1))), 3)
        for k in range(self.n):
            row[f"x{k}"], row[f"y{k}"], row[f"z{k}"] = (round(float(v), 3) for v in P[k])
        self.rows.append(row)
        return row

    def live(self) -> Dict:
        """Compact status for the UI (REFEREE / ground truth)."""
        last = self.rows[-1] if self.rows else {}
        dmin = min((last.get(f"d_{i}_{j}", 1e9) for i, j in self.pairs), default=None)
        return {"d_min_true": dmin, "p1_ok": not self.p1_violations,
                "occupancy": {g.gate_id: last.get(f"occ_{g.gate_id}", 0) for g in self.gates},
                "p2_ok": not self.p2_violations, "form_err": last.get("form_err"),
                "formation": ("LOST" if self._lost and self._ok_since is None else
                              "RECOVERING" if self._lost else "OK") if self.formation_enabled else None,
                "episodes": len(self.episodes)}

    # ------------------------------------------------------------------ verdicts
    def metrics(self) -> Dict:
        sep = self.cfg.sep
        dmin = min(self.min_d.values()) if self.min_d else None
        eps = [{"t_lost": round(e.t_lost, 2),
                "t_recovered": None if e.t_recovered is None else round(e.t_recovered, 2),
                "recovery_time_s": None if e.recovery_time is None else round(e.recovery_time, 2),
                "t_perturbation_end": None if e.t_perturbation_end is None else round(e.t_perturbation_end, 2),
                "recovery_after_perturbation_s": None if e.recovery_after_perturbation is None
                else round(e.recovery_after_perturbation, 2),
                "modes_at_loss": e.modes_at_loss} for e in self.episodes]
        return {
            "P1_separation": {"formula": "G(forall i!=j: d_ij >= d_safe)", "d_safe": sep.d_safe,
                              "min_distance": None if dmin is None else round(dmin, 3),
                              "min_per_pair": {f"{self.names[i]}-{self.names[j]}": {"d": round(d, 3), "t": self.min_d_t[(i, j)]}
                                               for (i, j), d in self.min_d.items()},
                              "violations": len(self.p1_violations), "first_violation": self.p1_violations[:1],
                              "physical_contacts": len(self.contacts), "collision_sensor_edges": self.collision_edges,
                              "holds": not self.p1_violations,
                              "intruder": None if self.intruder_min_t is None else {
                                  "note": "scripted vehicle outside the fleet (no controller); not part of P1 among drones",
                                  "min_distance": round(self.intruder_min_d, 3), "t": self.intruder_min_t,
                                  "below_d_safe": self.intruder_min_d < sep.d_safe,
                                  "contacts": len(self.intruder_contacts)}},
            "P2_mutual_exclusion": {"formula": "G(sum_i inside_CR_i <= 1)", "gates": [g.gate_id for g in self.gates],
                                    "max_occupancy": self.max_occ, "entry_order": self.entry_order,
                                    "violations": len(self.p2_violations), "first_violation": self.p2_violations[:1],
                                    "holds": not self.p2_violations},
            "P3_formation_recovery": {"formula": "G(formation_lost -> F formation_recovered)",
                                      "enabled": self.formation_enabled, "e_lost": self.cfg.ref.e_lost,
                                      "e_ok": self.cfg.ref.e_ok, "t_ok_hold": self.cfg.ref.t_ok_hold,
                                      "episodes": eps, "n_episodes": len(eps),
                                      "open_at_end": int(self._lost),
                                      "all_recovered_within_run": not self._lost,
                                      "final_form_err": self.rows[-1].get("form_err") if self.rows else None},
            # raw current statistics; the envelope verdict (control-feasibility of the requested velocities,
            # DI-27) needs the controllers' logs and is added by referee/envelope.evaluate_run at the end of a run
            "envelope": {"max_horizontal_drift": round(self.max_drift, 3),
                         "max_vertical_drift": round(self.max_vertical_drift, 3)},
            "kinematics": {"max_true_speed_m_s": round(self.max_speed, 3)},
        }
