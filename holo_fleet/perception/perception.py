"""Per-drone perception facade: onboard sensor frame -> local observation + abstract observation.

Inputs are exactly the onboard sensor streams of one drone (``SensorFrame``).
Outputs:

* :class:`LocalObservation` (rich, logged as ``drone_i_observations.jsonl``);
* :class:`holo_fleet.ha.automaton.AbstractObservation` (guard variables).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha import gate_rule
from holo_fleet.ha.automaton import AbstractObservation
from holo_fleet.ha.spec import PY_LOGIC
from holo_fleet.mission import GateSpec, MissionPlan
from holo_fleet.perception.nav import DeadReckoning, NavState
from holo_fleet.perception.proximity import ProximityProcessor, ScanResult
from holo_fleet.perception.sensor_suite import ring_name
from holo_fleet.perception.tracker import NeighborTracker, Track


@dataclass
class SensorFrame:
    """Onboard sensor data of ONE drone at onboard time ``t`` (latest value + its age)."""

    t: float
    data: Dict[str, Any]
    stamp: Dict[str, float]             # onboard time of the last update of each sensor

    def fresh(self, name: str, max_age: float) -> bool:
        return name in self.data and (self.t - self.stamp.get(name, -1e9)) <= max_age + 1e-9


@dataclass
class NeighborObs:
    tid: int
    rel: List[float]
    distance: float
    bearing_deg: float
    rel_z: float
    confidence: float
    staleness: float
    source: str


@dataclass
class GateObs:
    gate_id: Optional[str] = None
    index: int = -1
    own_gate_frame: Optional[List[float]] = None     # (s, l, dz)
    distance_to_gate: Optional[float] = None
    alignment_deg: Optional[float] = None            # heading error w.r.t. gate axis
    inside_approach_corridor: bool = False
    inside_cr_estimate: bool = False
    structure_returns: int = 0
    confidence: float = 0.0
    occ_busy: bool = False
    has_prio: bool = False
    at_queue: bool = False
    passed: bool = False
    queue_neighbors: List[Dict[str, Any]] = field(default_factory=list)
    occupant_neighbors: List[Dict[str, Any]] = field(default_factory=list)
    backoff: str = "none"                            # none | BACKOFF_REAR | BACKOFF_RIGHT | BACKOFF_RANK
    backoff_target_s: Optional[float] = None         # gate-frame along position to retreat to
    lane_blocker_ds: Optional[float] = None


@dataclass
class FormationObs:
    s_path: float = 0.0
    lat_err: float = 0.0
    z_err: float = 0.0
    form_err: float = 0.0
    neighbors_ok: bool = True
    assigned: Dict[int, int] = field(default_factory=dict)     # slot index -> track id
    residuals: Dict[int, List[float]] = field(default_factory=dict)
    along_consensus: float = 0.0                               # mean along-track slot error (m)
    lat_consensus: float = 0.0
    lost: bool = False
    recovered: bool = False
    t_ok: float = 0.0


@dataclass
class LocalObservation:
    t: float
    nav: Dict[str, Any]
    neighbors: List[NeighborObs]
    nearest_neighbor_distance_est: float
    hazard_level: str
    gate: GateObs
    formation: FormationObs
    sense_ok: bool
    env_ok: bool
    sensor_age: Dict[str, float]
    n_ring_returns: int
    threats: List[List[float]] = field(default_factory=list)  # unit vectors to threats (for flows)
    threat_dists: List[float] = field(default_factory=list)
    structure_close: List[List[float]] = field(default_factory=list)


class Perception:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, nav_init_offset: Optional[np.ndarray] = None):
        self.cfg = cfg
        self.plan = plan
        p0 = np.array(plan.launch_position, dtype=float)
        if nav_init_offset is not None:
            p0 = p0 + np.asarray(nav_init_offset, dtype=float)
        self.nav = DeadReckoning(p0, plan.launch_yaw_deg)
        self.prox = ProximityProcessor(plan.structures, cfg)
        self.tracker = NeighborTracker(cfg)
        self.gate_index = 0
        self.t_ok = 0.0
        self.t_neighbors_missing = 0.0
        self.last_scan_t = -1e9
        self.last_scan: Optional[ScanResult] = None
        self.ring_names = [ring_name(e) for e in cfg.perc.ring_elevations_deg]
        self.env_violation_timer = 0.0
        self.env_ok_flag = True

    # ------------------------------------------------------------------
    def current_gate(self) -> Optional[GateSpec]:
        if 0 <= self.gate_index < len(self.plan.gates):
            return self.plan.gates[self.gate_index]
        return None

    def advance_gate(self) -> None:
        self.gate_index += 1

    def report_tracking_error(self, err: float, saturated: bool, dt: float) -> None:
        """Operational-envelope monitor fed by the controller.

        The vehicle declares itself OUT OF ENVELOPE when its horizontal command is saturated and it
        still cannot follow the commanded velocity (error > 0.3 m/s) for 4 s: the disturbance exceeds
        the available authority.  The declaration is latched until the condition has been absent for
        as long as it was present (hysteresis), so FAILSAFE is not left after a single good sample.
        """
        if saturated and err > 0.3:
            self.env_violation_timer = min(self.env_violation_timer + dt, 10.0)
        else:
            self.env_violation_timer = max(0.0, self.env_violation_timer - dt)
        if self.env_ok_flag and self.env_violation_timer >= 4.0:
            self.env_ok_flag = False
        elif not self.env_ok_flag and self.env_violation_timer <= 0.0:
            self.env_ok_flag = True

    # ------------------------------------------------------------------
    def update(self, frame: SensorFrame, dt: float) -> Tuple[LocalObservation, AbstractObservation]:
        cfg = self.cfg
        t = frame.t
        d = frame.data
        nav = self.nav.update(
            t, dt,
            compass=d.get("Compass") if frame.fresh("Compass", 0.2) else None,
            dvl=d.get("DVLSensor") if frame.fresh("DVLSensor", 0.2) else None,
            depth=d.get("DepthSensor") if frame.fresh("DepthSensor", 0.2) else None,
            imu=d.get("IMUSensor") if frame.fresh("IMUSensor", 0.2) else None,
        )
        R_bw = self.nav.body_to_world_rot()

        # --- proximity sonar (process each new frame once) ---------------
        ring_stamp = min((frame.stamp.get(n, -1e9) for n in self.ring_names), default=-1e9)
        rings_present = all(n in d for n in self.ring_names)
        if rings_present and ring_stamp > self.last_scan_t:
            fls = d.get("FrontSonar") if frame.fresh("FrontSonar", 0.25) else None
            scan = self.prox.process(t, {n: d[n] for n in self.ring_names}, nav.p.copy(), R_bw, fls)
            self.tracker.update(t, nav.p.copy(), scan.detections)
            self.last_scan_t = ring_stamp
            self.last_scan = scan
        scan = self.last_scan or ScanResult()
        ring_age = t - ring_stamp if rings_present else 1e9

        sense_ok = (ring_age <= cfg.env.tau_max and frame.fresh("DVLSensor", 0.5) and frame.fresh("Compass", 0.5)
                    and frame.fresh("DepthSensor", 0.5))
        depth_ok = cfg.env.z_min + 0.3 <= nav.p[2] <= cfg.env.z_max - 0.0
        env_ok = depth_ok and self.env_ok_flag

        # --- neighbours ----------------------------------------------------
        v_close = 2 * cfg.env.v_max_nominal + cfg.env.w_rel_max
        neighbors: List[NeighborObs] = []
        threats, threat_d = [], []
        d_min = 1e9
        for tr in self.tracker.tracks:
            rel = tr.predicted(t) - nav.p
            dist = float(np.linalg.norm(rel))
            st = tr.staleness(t)
            d_eff = dist - v_close * st
            d_min = min(d_min, d_eff)
            b = np.linalg.solve(R_bw, rel)
            neighbors.append(NeighborObs(tid=tr.tid, rel=rel.round(3).tolist(), distance=round(dist, 3),
                                         bearing_deg=round(math.degrees(math.atan2(b[1], b[0])), 1),
                                         rel_z=round(float(rel[2]), 3), confidence=round(tr.confidence, 2),
                                         staleness=round(st, 3), source=tr.source))
            if d_eff < cfg.sep.d_warning_exit + 0.3:
                threats.append((rel / max(dist, 1e-6)).tolist())
                threat_d.append(d_eff)
        structure_close = []
        if len(scan.structure_points):
            sd = np.linalg.norm(scan.structure_points, axis=1)
            near = scan.structure_points[sd < 1.0]
            near = near[np.argsort(np.linalg.norm(near, axis=1))]
            # keep the nearest return per distinct direction (>= 30 deg apart), at most 4
            for p in near:
                u = p / max(np.linalg.norm(p), 1e-6)
                if all(float(u @ (np.asarray(q) / max(np.linalg.norm(q), 1e-6))) < math.cos(math.radians(30))
                       for q in structure_close):
                    structure_close.append(p.round(3).tolist())
                if len(structure_close) >= 4:
                    break
        hazard = "NORMAL"
        if d_min < cfg.sep.d_ca:
            hazard = "CRITICAL"
        elif d_min < cfg.sep.d_warning:
            hazard = "WARNING"

        gate_obs = self._gate_obs(t, nav, scan)
        form_obs = self._formation_obs(t, dt, nav)

        local = LocalObservation(
            t=t,
            nav={"p": nav.p.round(3).tolist(), "yaw_deg": round(math.degrees(nav.yaw), 2),
                 "v_body": np.asarray(nav.v_body).round(3).tolist(), "yaw_rate": round(nav.yaw_rate, 3)},
            neighbors=neighbors,
            nearest_neighbor_distance_est=round(d_min, 3) if d_min < 1e8 else None,
            hazard_level=hazard,
            gate=gate_obs,
            formation=form_obs,
            sense_ok=bool(sense_ok),
            env_ok=bool(env_ok),
            sensor_age={k: round(t - v, 3) for k, v in frame.stamp.items()
                        if not k.startswith("ProxSonar_")} | {"ProxSonar": round(ring_age, 3)},
            n_ring_returns=scan.n_valid,
            threats=threats,
            threat_dists=threat_d,
            structure_close=structure_close,
        )
        abstract = AbstractObservation(
            d_min=float(d_min),
            form_err=float(form_obs.form_err),
            t_ok=float(form_obs.t_ok),
            sense_ok=bool(sense_ok),
            env_ok=bool(env_ok),
            gate_zone=bool(gate_obs.inside_approach_corridor),
            at_queue=bool(gate_obs.at_queue),
            occ_busy=bool(gate_obs.occ_busy),
            has_prio=bool(gate_obs.has_prio),
            passed=bool(gate_obs.passed),
            neighbors_ok=bool(form_obs.neighbors_ok),
        )
        return local, abstract

    # ------------------------------------------------------------------
    def _gate_obs(self, t: float, nav: NavState, scan: ScanResult) -> GateObs:
        G = self.cfg.gate
        gate = self.current_gate()
        obs = GateObs()
        if gate is None:
            return obs
        own = gate.to_gate_frame(nav.p)
        s_i, l_i, z_i = own
        obs.gate_id = gate.gate_id
        obs.index = self.gate_index
        obs.own_gate_frame = own.round(3).tolist()
        obs.distance_to_gate = round(float(np.linalg.norm(nav.p - gate.center)), 3)
        gate_yaw = math.atan2(gate.axis[1], gate.axis[0])
        obs.alignment_deg = round(math.degrees((nav.yaw - gate_yaw + math.pi) % (2 * math.pi) - math.pi), 2)
        exit_s = G.cr_half_len + G.occ_exit_margin
        obs.passed = bool(s_i > exit_s)
        obs.inside_approach_corridor = bool(G.s_queue - G.approach_len <= s_i <= exit_s and abs(l_i) <= G.corridor_half_width + 3.0)
        obs.inside_cr_estimate = bool(abs(s_i) <= G.cr_half_len and abs(l_i) <= G.cr_half_width and abs(z_i) <= G.cr_half_height)
        obs.at_queue = bool(s_i >= G.s_queue - G.commit_window)
        obs.structure_returns = int(scan.structure_hits.get(gate.gate_id, 0))
        obs.confidence = round(min(1.0, obs.structure_returns / 40.0), 2)

        occ_lo = G.s_queue + G.occ_gamma
        occ_hi = exit_s
        has_prio = True
        backoff = "none"
        backoff_target = None
        lane_blocker = None
        cfg_eps = self.cfg.env.eps_rel
        for tr in self.tracker.confirmed(t):
            rel = tr.predicted(t) - nav.p
            gj = gate.vec_to_gate_frame(rel) + own              # neighbour in gate frame
            s_j, l_j, z_j = gj
            info = {"tid": tr.tid, "gate_frame": gj.round(3).tolist(), "staleness": round(tr.staleness(t), 3)}
            if abs(l_j) > G.corridor_half_width or abs(z_j) > 3.0:
                continue
            if occ_lo <= s_j <= occ_hi:
                obs.occupant_neighbors.append(info)
                continue
            if G.s_queue - G.approach_len <= s_j < occ_lo:
                obs.queue_neighbors.append(info)
                ds, dl, dz = float(s_i - s_j), float(l_i - l_j), float(z_i - z_j)
                dec = gate_rule.decide(ds, dl, dz, G, cfg_eps)
                info["decision"] = dec
                if dec != "COMMIT":
                    has_prio = False
                if dec in ("BACKOFF_REAR", "BACKOFF_RIGHT"):
                    target = s_j - gate_rule.backoff_clearance(G, cfg_eps)
                    backoff_target = target if backoff_target is None else min(backoff_target, target)
                    backoff = dec if backoff in ("none", "BACKOFF_RANK") else backoff
                elif dec == "BACKOFF_RANK":
                    target = G.s_queue - 1.0 - 0.8 * self.plan.static_rank
                    backoff_target = target if backoff_target is None else min(backoff_target, target)
                    backoff = dec if backoff == "none" else backoff
                # same-lane car following (queue spacing)
                if abs(dl) < 1.5 and 0 < s_j - s_i < 3.0:
                    lane_blocker = float(s_j - s_i) if lane_blocker is None else min(lane_blocker, float(s_j - s_i))
        obs.occ_busy = bool(obs.occupant_neighbors)
        obs.has_prio = bool(has_prio)
        obs.backoff = backoff
        obs.backoff_target_s = None if backoff_target is None else round(float(backoff_target), 3)
        obs.lane_blocker_ds = lane_blocker
        return obs

    # ------------------------------------------------------------------
    def _formation_obs(self, t: float, dt: float, nav: NavState) -> FormationObs:
        F = self.cfg.form
        plan = self.plan
        fo = FormationObs()
        s_i, l_i, _ = plan.path.project(nav.p[:2])
        me = plan.my_slot
        fo.s_path = round(s_i - me.along, 3)       # progress of the formation reference implied by me
        fo.lat_err = round(l_i - me.lateral, 3)
        fo.z_err = round(nav.p[2] - (plan.path.depth_z + me.dz), 3)
        others = [k for k in range(len(plan.slots)) if k != plan.slot_index]
        if not plan.formation_enabled or not others:
            fo.neighbors_ok = True
            fo.t_ok = self.t_ok = self.t_ok + dt
            fo.recovered = True
            return fo
        tracks = [tr for tr in self.tracker.confirmed(t) if tr.staleness(t) < F.neighbor_lost_s]
        # relative coordinates of each track in the path frame at my position
        _, tan, nrm = plan.path.frame_at(s_i)
        rels = []
        for tr in tracks:
            r = tr.predicted(t) - nav.p
            rels.append(np.array([r[:2] @ tan, r[:2] @ nrm, r[2]]))
        best_cost, best_assign = 1e18, {}
        k_slots = len(others)
        for perm in itertools.permutations(range(len(tracks)), min(k_slots, len(tracks))):
            cost, assign = 0.0, {}
            for slot_k, tr_idx in zip(others, perm):
                exp = np.array([plan.slots[slot_k].along - me.along, plan.slots[slot_k].lateral - me.lateral,
                                plan.slots[slot_k].dz - me.dz])
                res = rels[tr_idx] - exp
                cost += float(res @ res)
                assign[slot_k] = tr_idx
            if cost < best_cost:
                best_cost, best_assign = cost, assign
        errs, along_terms, lat_terms = [], [], []
        for slot_k, tr_idx in best_assign.items():
            exp = np.array([plan.slots[slot_k].along - me.along, plan.slots[slot_k].lateral - me.lateral,
                            plan.slots[slot_k].dz - me.dz])
            res = rels[tr_idx] - exp
            if float(np.linalg.norm(res)) > 6.0:
                continue                        # implausible association -> treat slot as unseen
            fo.assigned[slot_k] = tracks[tr_idx].tid
            fo.residuals[slot_k] = res.round(3).tolist()
            errs.append(float(np.linalg.norm(res)))
            along_terms.append(res[0])
            lat_terms.append(res[1])
        all_seen = len(fo.assigned) == k_slots
        if all_seen:
            self.t_neighbors_missing = 0.0
        else:
            self.t_neighbors_missing += dt
        fo.neighbors_ok = bool(all_seen or self.t_neighbors_missing < F.neighbor_lost_s)
        fo.form_err = round(max(errs) if errs else 99.0, 3)
        fo.along_consensus = float(np.mean(along_terms)) if along_terms else 0.0
        fo.lat_consensus = float(np.mean(lat_terms)) if lat_terms else 0.0
        if fo.form_err < F.e_ok and all_seen:
            self.t_ok += dt
        else:
            self.t_ok = 0.0
        fo.t_ok = round(self.t_ok, 3)
        fo.lost = bool(fo.form_err > F.e_lost or not fo.neighbors_ok)
        fo.recovered = bool(fo.form_err < F.e_ok and self.t_ok >= F.t_ok_hold and fo.neighbors_ok)
        return fo
