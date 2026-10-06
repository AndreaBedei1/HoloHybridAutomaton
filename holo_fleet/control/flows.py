"""Continuous flows of the local hybrid automaton: one desired-velocity law per mode.

All laws use only the drone's own estimate and its local observation.  Output is
a desired velocity in the local-level map frame plus a desired heading.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.spec import Mode
from holo_fleet.mission import GateSpec, MissionPlan
from holo_fleet.perception.perception import LocalObservation


@dataclass
class FlowOutput:
    v: np.ndarray                 # desired velocity (map frame)
    yaw_d: float                  # desired heading [rad]
    authority: str                # "nominal" or "escape"
    note: str = ""


def _unit_dirs() -> np.ndarray:
    dirs = []
    for d in itertools.product((-1, 0, 1), repeat=3):
        if d == (0, 0, 0):
            continue
        v = np.array(d, dtype=float)
        dirs.append(v / np.linalg.norm(v))
    return np.array(dirs)


ESCAPE_DIRS = _unit_dirs()   # 26 candidate directions (cube faces, edges, corners)


def project_halfspaces(v_t: np.ndarray, A: np.ndarray, b: np.ndarray) -> Optional[np.ndarray]:
    """Exact solution of  min ||v - v_t||^2  s.t.  A v <= b  by active-set enumeration (m <= 4).

    Returns None when the constraint set is empty.  Deterministic (fixed subset order).
    """
    m = len(b)
    best, best_cost = None, float("inf")
    for r in range(0, m + 1):
        for S in itertools.combinations(range(m), r):
            if r == 0:
                v = v_t.copy()
                lam_ok = True
            else:
                As, bs = A[list(S)], b[list(S)]
                G = As @ As.T
                if abs(np.linalg.det(G)) < 1e-9:
                    continue
                lam = np.linalg.solve(G, As @ v_t - bs)
                lam_ok = bool(np.all(lam >= -1e-9))
                v = v_t - As.T @ lam
            if not lam_ok or np.any(A @ v > b + 1e-7):
                continue
            cost = float(np.sum((v - v_t) ** 2))
            if cost < best_cost - 1e-12:
                best, best_cost = v, cost
    return best


def bisector_escape(threats: List[np.ndarray], rel_z: List[float], z: float, cfg: FleetConfig) -> np.ndarray:
    """CA escape direction: away along the bisector of the (<= 2) threat directions, plus a vertical
    component (3D escape) used only when it points away from EVERY threat (all threats above, or all
    below, by more than 0.15 m).  Lemmas S2b/S2c in formal/check_separation.py cover exactly this
    construction."""
    env = cfg.env
    if len(threats) == 1:
        b = -threats[0]
    else:
        s = threats[0] + threats[1]
        n = float(np.linalg.norm(s))
        b = -s / n if n > 1e-3 else np.array([0.0, 0.0, 1.0])
    vz = 0.0
    if rel_z and all(rz > 0.15 for rz in rel_z):
        vz = -1.0                                   # everybody above me: go down
    elif rel_z and all(rz < -0.15 for rz in rel_z):
        vz = 1.0                                    # everybody below me: go up
    if (vz > 0 and z > env.z_max - 0.5) or (vz < 0 and z < env.z_min + 0.5):
        vz = 0.0
    e = b + env.escape_vertical_weight * np.array([0.0, 0.0, vz])
    if z > env.z_max - 0.5 and e[2] > 0:
        e[2] = 0.0
    if z < env.z_min + 0.5 and e[2] < 0:
        e[2] = 0.0
    n = float(np.linalg.norm(e))
    return e / n if n > 1e-6 else np.array([-1.0, 0.0, 0.0])


def choose_escape(threats: List[np.ndarray], z: float, cfg: FleetConfig, rel_z: Optional[List[float]] = None,
                  structures: Optional[List[np.ndarray]] = None) -> Tuple[np.ndarray, float]:
    """Pick the candidate direction maximising the minimum opening rate over all threats.

    Deterministic tie-breaking: prefer vertical motion away from the threat's
    relative depth (antisymmetric, so two drones pick opposite vertical senses),
    then lateral-right (COLREG-like, also antisymmetric in each drone's frame).
    """
    best, best_score = ESCAPE_DIRS[0], -1e9
    for e in ESCAPE_DIRS:
        if z > cfg.env.z_max - 0.5 and e[2] > 0:
            continue
        if z < cfg.env.z_min + 0.5 and e[2] < 0:
            continue
        score = min((-float(e @ u) for u in threats), default=1.0)
        if structures:
            score = min(score, min(-float(e @ u) + 0.3 for u in structures))
        bias = 0.0
        if rel_z:
            mean_rz = float(np.mean(rel_z))
            if abs(mean_rz) > 0.15:
                bias += 0.06 * (-np.sign(mean_rz) * e[2])
        if threats:
            u0 = threats[0]                                   # callers pass nearest threat first
            right = np.array([u0[1], -u0[0], 0.0])
            if np.linalg.norm(right) > 1e-6:
                bias += 0.03 * float(e @ (right / np.linalg.norm(right)))
        if score + bias > best_score:
            best, best_score = e, score + bias
    score = min((-float(best @ u) for u in threats), default=1.0)
    return best, score


def structure_filter(v: np.ndarray, structure_close: List[List[float]], cfg: FleetConfig):
    """Safety filter against fixed structures seen by the proximity sonar (gate bars, frames).

    Minimum change of v such that the drone never closes on a structure return nearer than
    struct_keepout and opens (0.1 m/s) from one nearer than struct_push.  Passing through the middle
    of a gate is unaffected: the bars are lateral to the motion (v.u ~ 0).  Returns (v, active).
    """
    env = cfg.env
    A, b = [], []
    for p in structure_close:
        p = np.asarray(p, dtype=float)
        d = float(np.linalg.norm(p))
        if d >= env.struct_keepout or d < 1e-3:
            continue
        A.append(p / d)
        b.append(-0.10 if d < env.struct_push else 0.0)
    if not A:
        return v, False
    A_, b_ = np.array(A), np.array(b)
    if np.all(A_ @ v <= b_ + 1e-9):
        return v, False
    out = project_halfspaces(v, A_, b_)
    if out is None:
        out = project_halfspaces(v, A_, np.zeros_like(b_))
    return (out if out is not None else np.zeros(3)), True


class Flows:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT):
        self.plan = plan
        self.cfg = cfg
        self._rally_s = {}

    # ---------------------------------------------------------------- helpers
    def _gate_to_world(self, gate: GateSpec, vs: float, vl: float, vz: float) -> np.ndarray:
        left = np.array([-gate.axis[1], gate.axis[0], 0.0])
        return vs * gate.axis + vl * left + np.array([0.0, 0.0, vz])

    def _rally_s_for(self, gate: GateSpec) -> float:
        if gate.gate_id not in self._rally_s:
            G = self.cfg.gate
            p = gate.from_gate_frame([G.cr_half_len + G.occ_exit_margin + G.rally_after_exit, 0.0, 0.0])
            s, _, _ = self.plan.path.project(p[:2])
            self._rally_s[gate.gate_id] = s
        return self._rally_s[gate.gate_id]

    # ---------------------------------------------------------------- mission flows
    def formation(self, obs: LocalObservation, mode: Mode, p: np.ndarray, gate_index: int) -> FlowOutput:
        F, plan = self.cfg.form, self.plan
        me = plan.my_slot
        s_i, l_i, _ = plan.path.project(p[:2])
        _, tan, nrm = plan.path.frame_at(s_i)
        fo = obs.formation
        has_neighbors = bool(fo.assigned)
        if not plan.formation_enabled:
            v_s = F.v_nominal
        elif has_neighbors:
            v_s = F.v_nominal + F.k_along * fo.along_consensus
        else:
            v_s = 0.0 if mode == Mode.FORMATION_RECOVERY else F.v_nominal * 0.5
        # passed drones clear the gate exit before waiting for the others
        if gate_index > 0 and gate_index - 1 < len(plan.gates):
            if s_i < self._rally_s_for(plan.gates[gate_index - 1]):
                v_s = max(v_s, 0.8 * F.v_nominal)
        if s_i - me.along >= plan.s_end:
            v_s = 0.0                                          # survey line completed: station keep
        v_s = float(np.clip(v_s, 0.0, self.cfg.env.v_max_nominal))
        v_l = -F.k_lat_lane * (l_i - me.lateral)
        if has_neighbors:
            v_l += F.k_lat_rel * fo.lat_consensus
        v_l = float(np.clip(v_l, -F.v_lat_max, F.v_lat_max))
        v_z = float(np.clip(-F.k_depth * (p[2] - (plan.path.depth_z + me.dz)), -F.v_z_max, F.v_z_max))
        v = np.array([v_s * tan[0] + v_l * nrm[0], v_s * tan[1] + v_l * nrm[1], v_z])
        return FlowOutput(v=v, yaw_d=math.atan2(tan[1], tan[0]), authority="nominal",
                          note=f"v_s={v_s:.2f} v_l={v_l:.2f}")

    def gate_queue(self, obs: LocalObservation, gate: GateSpec, p: np.ndarray, yielding: bool) -> FlowOutput:
        G, F = self.cfg.gate, self.cfg.form
        s_i, l_i, dz = gate.to_gate_frame(p)
        lat_slot = self.plan.my_slot.lateral
        l_q = math.copysign(G.queue_lateral, lat_slot) if abs(lat_slot) > 0.5 else 0.0
        s_target = G.s_queue
        if abs(lat_slot) <= 0.5:
            # centre slot waits on the axis, behind the queue line while side drones are queued,
            # so that their merge onto the axis never brings them inside d_warning of it
            if any(q["gate_frame"][0] > s_i - 0.5 for q in obs.gate.queue_neighbors):
                s_target = G.s_queue - G.queue_center_back
        if obs.gate.lane_blocker_ds is not None:
            s_target = min(s_target, s_i + obs.gate.lane_blocker_ds - 3.0)
        bo = obs.gate.backoff
        if bo != "none" and obs.gate.backoff_target_s is not None:
            s_target = min(s_target, obs.gate.backoff_target_s)   # retreat until the along rule decides
        v_s = float(np.clip(0.4 * (s_target - s_i), -G.v_approach, G.v_approach))
        v_l = float(np.clip(0.5 * (l_q - l_i), -F.v_lat_max, F.v_lat_max))
        v_z = float(np.clip(-0.6 * dz, -F.v_z_max, F.v_z_max))
        return FlowOutput(v=self._gate_to_world(gate, v_s, v_l, v_z), yaw_d=math.atan2(gate.axis[1], gate.axis[0]),
                          authority="nominal", note=f"queue s*={s_target:.2f} l*={l_q:.2f} backoff={bo}")

    def gate_pass(self, obs: LocalObservation, gate: GateSpec, p: np.ndarray) -> FlowOutput:
        G = self.cfg.gate
        s_i, l_i, dz = gate.to_gate_frame(p)
        straight_until = G.s_queue + G.occ_gamma + self.cfg.env.eps_rel + 0.3
        v_z = float(np.clip(-0.8 * dz, -0.3, 0.3))
        if s_i < straight_until:
            # phase 1: leave the queue line straight ahead (keeps priority monotone)
            v_s, v_l, phase = G.v_pass, 0.0, "leave_queue"
        elif s_i < -G.cr_half_len - 0.2 and abs(l_i) > 0.25:
            # phase 2: merge onto the gate axis before entering the critical region
            v_l = float(np.clip(-0.6 * l_i, -0.35, 0.35))
            v_s = max(0.05, G.v_pass * float(np.clip(1.0 - (abs(l_i) - 0.25) / 0.9, 0.0, 1.0)))
            phase = "merge"
        else:
            v_s, v_l, phase = G.v_pass, float(np.clip(-0.8 * l_i, -0.3, 0.3)), "traverse"
        return FlowOutput(v=self._gate_to_world(gate, v_s, v_l, v_z), yaw_d=math.atan2(gate.axis[1], gate.axis[0]),
                          authority="nominal", note=f"pass:{phase}")

    # ---------------------------------------------------------------- safety flows
    def separation_warning(self, base: FlowOutput, obs: LocalObservation) -> FlowOutput:
        """Soft avoidance: never close on a threat; convert closing intent into a right-hand sidestep.

        The sidestep is to the RIGHT of the line of sight in each drone's own view.  The rule is
        antisymmetric (u_ji = -u_ij), so two drones meeting head-on move to opposite sides and
        pass port-to-port instead of stopping nose to nose.  When the threat is at a different
        depth the vertical gap is also widened (again antisymmetric through sign(rel_z)).
        """
        sep, env = self.cfg.sep, self.cfg.env
        v_t = base.v.copy()
        A, b = [], []
        for u, d in zip(obs.threats, obs.threat_dists):
            if d >= sep.d_warning_exit:
                continue
            u = np.asarray(u, dtype=float)
            closing = float(v_t @ u)
            if closing > 0:
                uh = np.array([u[0], u[1], 0.0])
                if np.linalg.norm(uh) > 1e-3:
                    right = np.array([uh[1], -uh[0], 0.0]) / np.linalg.norm(uh)
                    v_t += (closing + 0.08) * right                      # sidestep intent (right-hand rule)
            if abs(u[2]) * d > 0.3:
                v_t[2] -= 0.08 * np.sign(u[2])                            # widen an existing depth gap
            A.append(u)
            b.append(-env.v_open if d < sep.d_warning else 0.0)        # open inside the band, never close
        note = "warning:" + base.note
        if A:
            A_, b_ = np.array(A), np.array(b)
            v = project_halfspaces(v_t, A_, b_)
            if v is None:                                              # opposite threats: only non-closing
                v = project_halfspaces(v_t, A_, np.zeros_like(b_))
                note += " [filter relaxed to non-closing]"
            if v is None:
                v = np.zeros(3)
        else:
            v = v_t
        n = np.linalg.norm(v)
        if n > env.v_filter_cap:
            v *= env.v_filter_cap / n
        return FlowOutput(v=v, yaw_d=base.yaw_d, authority="brake", note=note)

    def collision_avoidance(self, obs: LocalObservation, p: np.ndarray, yaw: float) -> FlowOutput:
        sep = self.cfg.sep
        thr, rz = [], []
        for nb in sorted(obs.neighbors, key=lambda n: n.distance):   # nearest threat first
            if nb.distance < sep.d_warning:
                thr.append(np.asarray(nb.rel) / max(nb.distance, 1e-6))
                rz.append(nb.rel_z)
        structs = [np.asarray(s) / max(np.linalg.norm(s), 1e-6) for s in obs.structure_close]
        if 1 <= len(thr) <= 2 and not structs:
            e = bisector_escape(thr, rz, p[2], self.cfg)
            score = min(-float(e @ u) for u in thr)
            how = "bisector"
        else:
            e, score = choose_escape(thr, p[2], self.cfg, rel_z=rz, structures=structs)
            how = "candidates"
        return FlowOutput(v=self.cfg.env.v_escape * e, yaw_d=yaw, authority="escape",
                          note=f"escape[{how}] dir={np.round(e, 2).tolist()} min_opening={score:.2f}")

    def failsafe(self, obs: LocalObservation, p: np.ndarray, yaw: float) -> FlowOutput:
        env = self.cfg.env
        if not obs.sense_ok:
            z_t = self.plan.path.depth_z + self.plan.failsafe_layer_dz
            note = "blind: hold xy, move to pre-assigned layer"
        else:
            z_t = float(np.clip(p[2], env.z_min + 1.0, env.z_max - 1.0))
            note = "envelope: hold xy, return inside depth band"
        v_z = float(np.clip(-0.6 * (p[2] - z_t), -0.35, 0.35))
        return FlowOutput(v=np.array([0.0, 0.0, v_z]), yaw_d=yaw, authority="escape", note=note)
