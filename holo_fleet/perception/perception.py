"""Onboard perception of one drone (v2): six sonar profiles + DVL + IMU + compass + depth -> observation.

Only data a real vehicle has is used: its own sensors and the mission plan (survey line, formation
template, gate map).  No simulator state, no neighbour identity, no communication.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.automaton import AbstractObservation
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.formation_perception import FormationObs, check_formation
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.gate_perception import GateObs, GatePerception
from holo_fleet.perception.nav import DeadReckoning
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
from holo_fleet.perception.sonar_processing import EchoClassifier, SectorReading, dvl_altitude
from holo_fleet.perception.targets import SectorTracker, Target, build_targets


@dataclass
class LocalObservation:
    t: float
    p: np.ndarray                      # own estimated position (map frame)
    yaw: float
    R_wb: np.ndarray                   # body -> world (estimated attitude)
    v_world: np.ndarray
    readings: Dict[str, SectorReading]
    targets: List[Target]
    d_min: float
    sense_ok: bool
    sonar_ages: Dict[str, float]
    altitude: Optional[float]
    seabed_z: Optional[float]
    gate: GateObs
    form: FormationObs
    slot_pos: np.ndarray
    ab: AbstractObservation
    unhealthy: List[str] = field(default_factory=list)
    gate_index: int = 0


class AttitudeFilter:
    """Roll/pitch from the accelerometer's gravity direction (low-passed); heading from the compass."""

    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, imu) -> None:
        if imu is None:
            return
        arr = np.asarray(imu, dtype=float)
        if arr.ndim != 2 or arr.shape[0] < 1:
            return
        ax, ay, az = arr[0][:3]
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if not 8.0 < g < 11.6:                  # accelerating: keep the previous estimate
            return
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        lim = math.radians(20.0)
        self.roll += self.alpha * (max(-lim, min(lim, roll)) - self.roll)
        self.pitch += self.alpha * (max(-lim, min(lim, pitch)) - self.pitch)


def rot_world_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Perception:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, nav_init_err: Optional[np.ndarray] = None,
                 use_attitude: bool = False):
        self.plan = plan
        self.cfg = cfg
        launch = np.asarray(plan.launch_position, dtype=float) + (np.zeros(3) if nav_init_err is None else nav_init_err)
        self.nav = DeadReckoning(launch, plan.launch_yaw_deg)
        self.att = AttitudeFilter()
        self.use_attitude = use_attitude
        bars = [b for g in plan.structures for b in g.bars]
        self.clf = EchoClassifier(gate_bars=bars, cfg=cfg)
        self.tracker = SectorTracker(cfg)
        self.gp = GatePerception(plan, cfg)
        self.ok_since: Optional[float] = None
        self.neighbour_seen: Dict[int, float] = {}     # slot -> last time it was matched by sonar

    # ------------------------------------------------------------------ formation reference
    def slot_reference(self, t: float, sigma: float = 0.0):
        """World position of the own slot and the formation frame (tangent, left normal, up)."""
        plan = self.plan
        clock = plan.clock
        s_ref = (clock.s(t) if clock is not None else 0.0) + sigma
        me = plan.slots[plan.slot_index]
        p_path, d, n = plan.path.frame_at(s_ref + me.along)
        slot = np.array([p_path[0] + me.lateral * n[0], p_path[1] + me.lateral * n[1], plan.path.depth_z + me.dz])
        _p0, d0, n0 = plan.path.frame_at(s_ref)
        R_form = np.array([[d0[0], n0[0], 0.0], [d0[1], n0[1], 0.0], [0.0, 0.0, 1.0]])
        return slot, R_form, s_ref

    # ------------------------------------------------------------------ main update
    def update(self, frame: SensorFrame, dt: float, sigma: float = 0.0, env_ok: bool = True,
               offset: Optional[np.ndarray] = None) -> LocalObservation:
        t = frame.t
        d = frame.data
        env = self.cfg.env
        nav = self.nav.update(t, dt, d.get("Compass"), d.get("DVLSensor"), d.get("DepthSensor"), d.get("IMUSensor"))
        self.att.update(d.get("IMUSensor"))
        roll, pitch = (self.att.roll, self.att.pitch) if self.use_attitude else (0.0, 0.0)
        R_wb = rot_world_body(roll, pitch, nav.yaw)
        altitude = dvl_altitude(d.get("DVLSensor"))
        seabed_z = None if altitude is None else float(nav.p[2] - 0.15 - altitude)
        profiles, ages = {}, {}
        for s in SECTORS:
            key = sonar_sensor_name(s)
            profiles[s] = d.get(key)
            ages[s] = frame.age(key) if key in d else 1e9
        readings = self.clf.classify(profiles, ages, nav.p, R_wb, seabed_z, tau_healthy=env.tau_max)
        tracks = self.tracker.update(t, readings)
        targets = build_targets(t, readings, tracks, self.cfg)
        d_min = min([tg.d_lower for tg in targets], default=1e9)
        unhealthy = [s for s in SECTORS if not readings[s].healthy]
        sense_ok = (not unhealthy and frame.fresh("DVLSensor", 0.35) and frame.fresh("Compass", 0.2)
                    and frame.fresh("DepthSensor", 0.2))
        gate = self.gp.update(t, nav.p, nav.yaw, targets, readings)
        slot, R_form, _s_ref = self.slot_reference(t, sigma)
        if offset is not None:
            slot = slot + offset                  # intentional give-way deviation is not a formation error
        healthy = {s: readings[s].healthy for s in SECTORS}
        form = check_formation(self.plan, self.cfg, slot - nav.p, R_form, R_wb, targets, healthy, t, self.neighbour_seen,
                               readings)
        fr = self.cfg.form
        if form.form_err < fr.e_ok and form.neighbors_ok:
            self.ok_since = t if self.ok_since is None else self.ok_since
        else:
            self.ok_since = None
        t_ok = 0.0 if self.ok_since is None else t - self.ok_since
        ab = AbstractObservation(
            d_min=float(d_min), form_err=float(form.form_err), t_ok=float(t_ok), sense_ok=bool(sense_ok),
            env_ok=bool(env_ok), mutex_zone=bool(gate.in_zone), at_queue=bool(gate.at_queue),
            occ_busy=bool(gate.occ_busy), has_prio=bool(gate.has_prio), passed=bool(gate.passed),
            neighbors_ok=bool(form.neighbors_ok))
        return LocalObservation(t=t, p=nav.p.copy(), yaw=nav.yaw, R_wb=R_wb, v_world=nav.v_world.copy(),
                                readings=readings, targets=targets, d_min=float(d_min), sense_ok=bool(sense_ok),
                                sonar_ages=ages, altitude=altitude, seabed_z=seabed_z, gate=gate, form=form,
                                slot_pos=slot, ab=ab, unhealthy=unhealthy, gate_index=self.gp.idx)
