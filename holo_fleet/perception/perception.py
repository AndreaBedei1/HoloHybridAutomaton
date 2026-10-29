"""Onboard perception of one drone (v2): six sonar profiles + DVL + IMU + compass + depth -> observation.

Only data a real vehicle has is used: its own sensors and the mission plan (survey line, formation
template, gate map).  No simulator state, no neighbour identity, no communication.  Near a mapped gate the
dead-reckoning position is corrected against the gate map (perception/structure_aiding.py).

Missing neighbours (lost-drone handling, DI-29).  Every expected neighbour that should be visible and
is not seen is MISSING; its timer runs only while the drone itself holds its slot (FORMATION_FOLLOW,
FORMATION_WAIT_REJOIN, DEGRADED_FORMATION: a drone that is itself lost or avoiding cannot judge the
others) and while the shared clock moves at the drone's own formation time (planned holds such as the
rendezvous beyond a gate are not absences; control/formation_time.py).  After ``t_rejoin`` of missing
time the slot is declared VACANT (a latch: ``degraded``); a vacant slot is no longer required, so the
drone continues in DEGRADED_FORMATION on its own original slot.  A neighbour becomes MISSING when it is
not seen for NEIGHBOUR_MEMORY_S and counts as back only when CONFIRMED near its slot (at least ``CONFIRM_N``
associations within the last ``CONFIRM_WINDOW_S``, each with a range residual within half the range
gate): an echo of another neighbour that happens to fit the missing one's (wide) range gate does not reset
the timer.  A vacant slot is re-included after its drone has been confirmed there for ``t_reinclude``.
At launch the formation is complete by plan (every neighbour counts as seen at the first update).

Variant B2 (range-bearing beacon, perception/acoustic.py).  Identified fixes enter the perception only, and never
``d_min``, ``sense_ok`` or ``has_prio``:
* gate: a second way to an exit witness of the CR latch (perception/acoustic_gate.py);
* association: a sonar echo is associated with a neighbour where its fix places it (formation_perception);
* presence (rule A-P): while the latest accepted fix is fresh (``acoustic.fresh_s``: one lost ping plus the
  latency) and within ``e_lost`` of the neighbour's expected position, the neighbour counts as sighted now for the
  NEIGHBOUR_MEMORY_S timer (a drone occluded for the sonar but heard where it should be is not missing); it never
  confirms a MISSING neighbour back (only the sonar confirmation does) and never clears a
  vacancy;
* displacement (rule A-D): ``A_CONFIRM`` consecutive accepted fixes whose whole error region lies farther than
  ``e_lost`` + ``A_DISPLACED_MARGIN`` from the neighbour's expected position declare it MISSING at once (and
  keep it so: no sonar confirmation while the fixes say otherwise);
* relative formation error: for a neighbour with a fresh fix, the full relative-position residual (fix minus
  template, map axes) replaces the range residual in the along-track / lateral consensus (``along_corr``,
  ``lateral_corr``): the along-track offset of an abreast neighbour, unobservable from ranges, becomes observable.
Without receptions (B0, a silent channel) every one of these is inactive and the perception is the sonar one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.automaton import AbstractObservation
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.formation_perception import (NEIGHBOUR_MEMORY_S, FormationObs, check_formation,
                                                       expected_relative)

CONFIRM_N = 3              # associations ...
CONFIRM_WINDOW_S = 1.0     # ... within this window confirm that a missing neighbour is back,
CONFIRM_GATE = 0.5         # ... each within this fraction of the range gate (near its slot)
A_CONFIRM = 2              # B2 rule A-D: consecutive accepted fixes ...
A_DISPLACED_MARGIN = 1.0   # ... whose region lies beyond e_lost + this from the expected position [m]
A_CORR_AGE_S = 1.5         # a fix this recent feeds the relative formation residual
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.gate_perception import GateObs, GatePerception
from holo_fleet.perception.nav import DeadReckoning
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
from holo_fleet.perception.sonar_processing import EchoClassifier, SectorReading, dvl_altitude
from holo_fleet.perception.structure_aiding import StructureAiding
from holo_fleet.perception.targets import EchoHold, SectorTracker, Target, build_targets


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
        self.aiding = StructureAiding(self.clf, cfg, gates=plan.structures)     # map-aided position correction near the gate
        self.tracker = SectorTracker(cfg)
        self.echo_hold = EchoHold(cfg.env.tau_max)     # missed detections become stale readings (targets.py)
        self.acoustic = AcousticTracker(cfg)           # B2 range-bearing beacon (inactive in B0)
        self.gp = GatePerception(plan, cfg, acoustic=self.acoustic)
        self.ok_since: Optional[float] = None
        self.neighbour_seen: Dict[int, float] = {}     # slot -> last time it was matched by sonar
        self.sightings: Dict[int, List[float]] = {}     # slot -> association times within CONFIRM_WINDOW_S
        self.confirmed_since: Dict[int, float] = {}     # slot -> start of the current confirmed presence
        self.missing_state: Dict[int, bool] = {}        # slot -> MISSING (hysteresis, see the module docstring)
        self.missing_t: Dict[int, float] = {}          # slot -> missing time (own slot held, clock moving)
        self.vacant: Dict[int, float] = {}             # slot -> time it was declared vacant (latch)
        self.events: List[Dict] = []                   # vacancy / re-inclusion events (drained by the controller)
        self.acoustic_present: List[int] = []          # B2 rule A-P: sightings from fixes this step
        self.acoustic_displaced: List[int] = []        # B2 rule A-D: neighbours positively away from their slot
        self.acoustic_corr: List[int] = []             # neighbours whose fix fed the relative residual

    # ------------------------------------------------------------------ formation reference
    def slot_reference(self, t_form: float, sigma: float = 0.0):
        """World position of the own slot and the formation frame (tangent, left normal, up) at the own formation
        time ``t_form`` (= mission time minus the local delay of the reduced progress, DI-31)."""
        plan = self.plan
        clock = plan.clock
        s_ref = (clock.s(t_form) if clock is not None else 0.0) + sigma
        me = plan.slots[plan.slot_index]
        p_path, d, n = plan.path.frame_at(s_ref + me.along)
        slot = np.array([p_path[0] + me.lateral * n[0], p_path[1] + me.lateral * n[1], plan.path.depth_z + me.dz])
        _p0, d0, n0 = plan.path.frame_at(s_ref)
        R_form = np.array([[d0[0], n0[0], 0.0], [d0[1], n0[1], 0.0], [0.0, 0.0, 1.0]])
        return slot, R_form, s_ref

    # ------------------------------------------------------------------ missing neighbours
    def track_missing(self, t: float, dt: float, form: FormationObs, in_formation: bool,
                      t_form: Optional[float] = None) -> None:
        """Missing state (hysteresis), missing timers and the vacancy latch (see the module docstring)."""
        fr = self.cfg.form
        clock = self.plan.clock
        moving = clock is None or clock.moving(t if t_form is None else t_form)
        for k in self.acoustic_displaced:                  # rule A-D: positively away from its slot
            if k not in form.missing:
                form.missing.append(k)
            if k not in form.required:
                form.required.append(k)
        for chk in form.checks:
            k = chk.slot
            sight = [x for x in self.sightings.get(k, []) if t - x <= CONFIRM_WINDOW_S]
            if k in self.acoustic_displaced:
                sight = []                                 # the fixes contradict a sonar confirmation near the slot
            elif chk.seen and abs(chk.residual) <= CONFIRM_GATE * fr.range_gate_m:
                sight.append(t)                            # near its slot: counts toward the confirmation
            self.sightings[k] = sight
            if len(sight) >= CONFIRM_N:
                self.confirmed_since.setdefault(k, t)
            else:
                self.confirmed_since.pop(k, None)
            confirmed = k in self.confirmed_since
            if k in form.missing:
                self.missing_state[k] = True               # not seen for NEIGHBOUR_MEMORY_S
            elif self.missing_state.get(k) and confirmed:
                self.missing_state[k] = False              # back, confirmed
            if not self.missing_state.get(k):
                self.missing_t[k] = 0.0
            if k in self.vacant:
                if confirmed and t - self.confirmed_since[k] >= fr.t_reinclude - 1e-9:
                    del self.vacant[k]
                    self.missing_state[k], self.missing_t[k] = False, 0.0
                    self.events.append({"t": t, "type": "SLOT_REOCCUPIED", "slot": k})
            elif self.missing_state.get(k) and k in form.required and in_formation and moving:
                self.missing_t[k] = self.missing_t.get(k, 0.0) + dt
                if self.missing_t[k] >= fr.t_rejoin - 1e-9:
                    self.vacant[k] = t
                    self.events.append({"t": t, "type": "SLOT_DECLARED_VACANT", "slot": k,
                                        "missing_s": round(self.missing_t[k], 1)})
        form.missing = [k for k in form.required if self.missing_state.get(k)]
        form.neighbors_ok = not [k for k in form.missing if k not in self.vacant]

    # ------------------------------------------------------------------ B2: identified neighbours
    def acoustic_neighbours(self, t: float, slot_err: np.ndarray, R_form: np.ndarray) -> Optional[Dict[int, np.ndarray]]:
        """Rules A-P and A-D (module docstring); returns the fresh relative positions used for the association."""
        self.acoustic_present, self.acoustic_displaced = [], []
        if not self.acoustic.enabled:
            return None
        a, fr = self.cfg.acoustic, self.cfg.form
        out = {}
        for k, d_world in expected_relative(self.plan, R_form).items():
            f = self.acoustic.latest(k, t, a.fresh_s)
            if f is None:
                continue
            out[k] = f.rel
            exp = d_world + np.asarray(slot_err, float)          # where k should be, relative to me
            if float(np.linalg.norm(f.rel - exp)) <= fr.e_lost:
                self.acoustic_present.append(k)
                self.neighbour_seen[k] = t                       # sighted while the fix is fresh (rule A-P)
            acc = [g for g in self.acoustic.all_fixes(k, t - a.fresh_s, t) if g.accepted][-A_CONFIRM:]
            if (len(acc) == A_CONFIRM and len({g.track for g in acc}) == 1
                    and all(float(np.linalg.norm(g.rel - exp)) - g.extent > fr.e_lost + A_DISPLACED_MARGIN for g in acc)):
                self.acoustic_displaced.append(k)
        return out

    def acoustic_residuals(self, t: float, form: FormationObs, R_form: np.ndarray) -> None:
        """Relative formation residual from the fresh fixes of neighbours near their expected position (B2)."""
        self.acoustic_corr = []
        if not self.acoustic.enabled:
            return
        t_hat, n_hat = R_form[:, 0], R_form[:, 1]
        exp_rel = expected_relative(self.plan, R_form)
        along, lat = [], []
        for chk in form.checks:
            k = chk.slot
            f = self.acoustic.latest(k, t, A_CORR_AGE_S)
            if f is not None and k in self.acoustic_present and k not in self.acoustic_displaced:
                e = f.rel - exp_rel[k]
                along.append(float(e @ t_hat))
                lat.append(float(e @ n_hat))
                self.acoustic_corr.append(k)
            elif chk.seen:
                along.append(chk.residual * chk.along_w)
                lat.append(chk.residual * chk.lateral_w)
        if self.acoustic_corr:
            form.along_corr = float(np.mean(along))
            form.lateral_corr = float(np.mean(lat))

    # ------------------------------------------------------------------ main update
    def update(self, frame: SensorFrame, dt: float, sigma: float = 0.0, env_ok: bool = True,
               offset: Optional[np.ndarray] = None, in_formation: bool = True,
               t_form: Optional[float] = None) -> LocalObservation:
        t = frame.t
        t_form = t if t_form is None else t_form
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
        corr = self.aiding.update(nav.p, R_wb, profiles, ages, env.tau_max, t=t)
        if corr is not None:                      # dead-reckoning drift bounded by the mapped gate (structure_aiding)
            self.nav.state.p += corr
        self.acoustic.update(t, d.get("AcousticRx"), nav.p, nav.yaw)     # B2 fixes (own pose after the aiding)
        readings = self.clf.classify(profiles, ages, nav.p, R_wb, seabed_z, tau_healthy=env.tau_max)
        tracks = self.tracker.update(t, readings)
        targets = build_targets(t, readings, tracks, self.cfg, held=self.echo_hold.update(t, readings))
        d_min = min([tg.d_lower for tg in targets], default=1e9)
        unhealthy = [s for s in SECTORS if not readings[s].healthy]
        sense_ok = (not unhealthy and frame.fresh("DVLSensor", 0.35) and frame.fresh("Compass", 0.2)
                    and frame.fresh("DepthSensor", 0.2))
        gate = self.gp.update(t, nav.p, nav.yaw, targets, readings)
        gate.loc_ok = self.aiding.valid(t)
        slot, R_form, _s_ref = self.slot_reference(t_form, sigma)
        if offset is not None:
            slot = slot + offset                  # intentional give-way deviation is not a formation error
        healthy = {s: readings[s].healthy for s in SECTORS}
        if not self.missing_state:                 # first update: the formation is complete by plan (launch)
            for k in range(len(self.plan.slots)):
                if k != self.plan.slot_index:
                    self.neighbour_seen[k], self.missing_state[k] = t, False
        ac_rel = self.acoustic_neighbours(t, slot - nav.p, R_form)
        form = check_formation(self.plan, self.cfg, slot - nav.p, R_form, R_wb, targets, healthy, t, self.neighbour_seen,
                               readings, vacant=self.vacant, acoustic=ac_rel)
        self.acoustic_residuals(t, form, R_form)
        self.track_missing(t, dt, form, in_formation, t_form)
        fr = self.cfg.form
        if form.form_err < fr.e_ok:
            self.ok_since = t if self.ok_since is None else self.ok_since
        else:
            self.ok_since = None
        t_ok = 0.0 if self.ok_since is None else t - self.ok_since
        ab = AbstractObservation(
            d_min=float(d_min), form_err=float(form.form_err), t_ok=float(t_ok), sense_ok=bool(sense_ok),
            env_ok=bool(env_ok), mutex_zone=bool(gate.in_zone), at_queue=bool(gate.at_queue),
            occ_busy=bool(gate.occ_busy), has_prio=bool(gate.has_prio), passed=bool(gate.passed),
            neighbors_ok=bool(form.neighbors_ok), degraded=bool(self.vacant))
        return LocalObservation(t=t, p=nav.p.copy(), yaw=nav.yaw, R_wb=R_wb, v_world=nav.v_world.copy(),
                                readings=readings, targets=targets, d_min=float(d_min), sense_ok=bool(sense_ok),
                                sonar_ages=ages, altitude=altitude, seabed_z=seabed_z, gate=gate, form=form,
                                slot_pos=slot, ab=ab, unhealthy=unhealthy, gate_index=self.gp.idx)
