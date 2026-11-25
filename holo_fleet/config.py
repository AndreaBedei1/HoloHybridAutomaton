"""Single source of truth for thresholds, envelope assumptions and controller gains.

Every number used by the formal models in ``formal/`` is imported from here, so the verified
abstraction and the deployed controller cannot silently diverge.  Sensor numbers come from the
sonar probe (calibration/analyze_sonar_probe.py), plant numbers from calibration/calibrate_plant.py;
provenance of each value in config/parameter_sources.json.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import asdict, dataclass, field
from typing import Dict, Tuple

import numpy as np

from holo_fleet.perception.sonar_geometry import DEFAULT_SONAR, R_IN, R_OUT, SonarModel


@dataclass(frozen=True)
class SeparationThresholds:
    """P1 thresholds.  ``d_safe``/``d_collision`` are ground-truth centre distances (referee only);
    the guards compare the *conservative onboard* distance (a lower bound of the centre distance,
    see Envelope.d_lower_offset) with ``d_ca`` / ``d_warning``.

    d_nominal > d_warning_exit > d_warning > d_ca_exit > d_ca > d_safe > d_collision.
    The values of d_warning and d_ca are the smallest ones for which formal/check_separation.py
    proves G(d >= d_safe) on the pairwise model, plus a margin (see README.md).
    """

    d_nominal: float = 3.5
    d_warning: float = 2.4
    d_warning_exit: float = 2.7
    d_ca: float = 1.7
    d_ca_exit: float = 2.0
    d_safe: float = 1.0
    d_collision: float = 0.6


@dataclass(frozen=True)
class Envelope:
    """Assumptions of the formal proofs (each one measured on the logs of the CLEAN reference runs, external archive)."""

    dt: float = 0.1
    tau_max: float = 0.25
    eps_range_far: float = 0.12
    eps_range_near: float = 0.08
    v_max_nominal: float = 0.50
    v_escape: float = 0.55
    a_brake: float = 0.50
    a_brake_measured: float = 0.743
    w_rel_max: float = 0.15
    w_drift_max: float = 0.15
    g_min: float = 0.50
    v_open: float = 0.20
    current_validated_max: float = 0.6
    current_vertical_max: float = 0.25
    struct_keepout: float = 0.70
    z_min: float = -9.0
    z_max: float = -1.5

    @property
    def c_max(self) -> float:
        """Worst-case closing speed of a pair: one escaping, one cruising, plus differential drift."""
        return self.v_escape + self.v_max_nominal + self.w_rel_max

    @staticmethod
    def d_lower_offset(mount_norm: float, eps_far: float = 0.12) -> float:
        """centre distance >= echo range + this (sensor at |mount| from the centre, target hull >= R_IN)."""
        return R_IN - mount_norm - eps_far

    @staticmethod
    def d_upper_offset(mount_norm: float, eps_near: float = 0.08) -> float:
        """centre distance <= echo range + this."""
        return mount_norm + R_OUT + eps_near


@dataclass(frozen=True)
class PerceptionConfig:
    sonar: SonarModel = DEFAULT_SONAR
    merge_gap_bins: int = 2
    vehicle_max_extent_m: float = 0.9
    structure_tol_m: float = 0.30
    structure_tol_far_m: float = 0.70
    seabed_tol_m: float = 0.70
    confirm_captures: int = 2
    track_drop_s: float = 1.2
    closing_rate_alpha: float = 0.5
    v_close_staleness: float = 1.20
    beam_dropout: float = 0.0
    blackout_every_s: float = 0.0
    blackout_len_s: float = 0.0
    camera_hz: int = 5


@dataclass(frozen=True)
class GateRule:
    """P2 critical-region protocol (gate frame: s along the axis towards the exit, l left, z up).

    The monitored CR covers the approach from -0.8 m through the gate plane at 0 m.
    The independent physical clearance plane is at +0.55 m: 0.15 m bar half depth plus
    the 0.389 m hull circumscribed radius, rounded up. Occupancy sensing and admission
    reserve the approach and passage through that clearance plane.
    Queue points are abreast on the queue line s = queue_s(n), ``queue_spacing`` apart, so that two
    queued drones see each other in a pure LEFT/RIGHT relation; the line is far enough back that the
    whole CR lies inside every queued drone's FRONT cone (``queue_cone_deg`` + heading tolerance <
    60 deg).  Decisions use only sector patterns (ha/mutex_rule.py), the own navigation estimate and
    the gate map.  A committed drone leaves the queue line on a path that keeps ``merge_clearance``
    from the queue points on its right (``pass_path``), joins the axis ``merge_ahead`` beyond the
    queue line, crosses the gate and veers towards its own formation lane after ``veer_s``.
    """

    cr_s_min: float = -0.8
    cr_s_max: float = 0.0
    passage_clear_s: float = 0.55
    cr_half_width: float = 1.0
    cr_half_height: float = 1.0
    queue_spacing: float = 3.5
    queue_cone_deg: float = 49.0
    queue_tol: float = 0.35
    heading_tol_deg: float = 6.0
    approach_len: float = 7.0
    corridor_half_width: float = 6.5
    t_clear: float = 1.0
    t_occ_max: float = 25.0
    nav_err_m: float = 1.0
    merge_angle_deg: float = 55.0
    merge_before_cr: float = 0.7
    merge_clearance: float = 3.2
    veer_s: float = 1.0
    exit_s: float = 3.5
    rally_s: float = 6.0
    v_approach: float = 0.50
    v_pass: float = 0.50
    k_track: float = 0.6
    queue_bracket_m: float = 7.5
    stack_column_spacing: float = 5.0
    stack_spacing: float = 3.5
    queue_lateral_max: float = 3.5
    hold_err_m: float = 0.10
    stage2_hold_s: float = -2.5525
    hold_lateral_allow: float = 0.27
    """Stage-2 hold of a committed drone before admission (gate frame s, on the axis): control/gate_safety.hold_s, the
    nearest point where holding is P2-safe under the declared navigation error (cr_s_min - (nav_err_m + reach of a
    hold + slack) = -2.2025 m), minus the measured holding error and a 0.25 m reserve preserving the calibrated acoustic
    acquisition position."""

    v_cross_min: float = 0.12
    """Premise (formal M3, formal/check_b2_comm.py): an admitted drone progresses along the gate axis at >= v_cross_min from
    its admission until it has left the CR, and never stops inside it (failures inside the CR are outside the scope,
    holo_fleet/scope.py).  A declared assumption of the P2 argument, not a measurement (M3f / B2C-Lv give the
    counterexamples without it)."""
    admit_zone_m: float = 0.4
    """Admission zone of the stage-2 hold: a committed drone may be admitted once its estimated s >= stage2_hold_s -
    admit_zone_m (control/flows.py RELOC_ZONE_M, kept equal by tests/test_b2_comm_fixes.py)."""

    @property
    def t_cross_max(self) -> float:
        """Progress-based time from the earliest admission to physical clearance."""
        return (self.passage_clear_s - self.stage2_hold_s + self.admit_zone_m + self.nav_err_m) / self.v_cross_min

    def passage_hi(self) -> np.ndarray:
        """Upper corner of the physical occupancy envelope used by admission sensing."""
        return np.array([self.passage_clear_s, self.cr_half_width, self.cr_half_height])

    @property
    def cr_half_len(self) -> float:
        """Half length of the CR along the axis (derived): (cr_s_max - cr_s_min) / 2."""
        return 0.5 * (self.cr_s_max - self.cr_s_min)

    @property
    def cr_center_s(self) -> float:
        """Gate-frame s of the CR centre (derived): (cr_s_min + cr_s_max) / 2."""
        return 0.5 * (self.cr_s_min + self.cr_s_max)

    def cr_lo(self) -> np.ndarray:
        """Lower corner of the CR box in the gate frame (s, l, dz)."""
        return np.array([self.cr_s_min, -self.cr_half_width, -self.cr_half_height])

    def cr_hi(self) -> np.ndarray:
        """Upper corner of the CR box in the gate frame (s, l, dz)."""
        return np.array([self.cr_s_max, self.cr_half_width, self.cr_half_height])

    def cr_center(self) -> np.ndarray:
        """Centre of the CR box in the gate frame (s, l, dz)."""
        return np.array([self.cr_center_s, 0.0, 0.0])

    def cr_half(self) -> np.ndarray:
        """Half extents of the CR box (s, l, dz)."""
        return np.array([self.cr_half_len, self.cr_half_width, self.cr_half_height])

    def in_cr(self, s, l, dz, margin: float = 0.0):
        """Is the gate-frame point (s, l, dz) inside the CR grown by ``margin`` on every side?  Vectorised (numpy
        broadcasting); a Python bool for scalar input.  The P2 occupancy predicate of the referee."""
        s, l, dz = np.asarray(s, float), np.asarray(l, float), np.asarray(dz, float)
        out = ((s >= self.cr_s_min - margin) & (s <= self.cr_s_max + margin)
               & (np.abs(l) <= self.cr_half_width + margin) & (np.abs(dz) <= self.cr_half_height + margin))
        return bool(out) if np.ndim(out) == 0 else out

    @property
    def merge_s(self) -> float:
        return self.cr_s_min - self.merge_before_cr

    def queue_s(self, n: int, dz_max: float = 0.0, lats: Tuple[float, ...] = ()) -> float:
        """Queue line, the smaller of two bounds:
        (a) the CR lies inside the FRONT cone (``queue_cone_deg``) of every queue point (with a vertical
            stack of queue points, ``dz_max`` > 0, the off-axis distance includes the vertical offset);
        (b) a drone left of the axis that descends to the merge point at ``merge_angle_deg`` keeps
            ``merge_clearance`` from its right neighbour's queue point (the next in the order);
        (c) with two or more columns, a drone that holds at the stage-2 hold (``stage2_hold_s``, within
            ``hold_lateral_allow`` of the axis: the steady deviation w_drift_max / k_track = 0.25 m of the hold loop
            against the declared lateral drift, plus 0.02 m; the acquisition offsets are vertical) keeps ``merge_clearance`` from
            every queue point: a queued drone and a drone held before the CR are not in each other's warning band (the
            criterion of (b); for n = 2 the line moves from -4.43 m to -5.39 m, for n = 3 from -5.66 m to -5.75 m; n = 1
            has no other queue point and n = 4 is unchanged);
        (d) the queue line is never ahead of the stage-2 hold: a queued drone holds where holding is P2-safe under the
            declared navigation error (control/gate_safety.py; n = 1 moves from -2.16 m to -2.55 m).
        Bound (a) is measured from the CR entry plane (-cr_s_min); every bound follows the CR entry plane.
        ``lats``: the column laterals (default: the abreast queue of n columns)."""
        lats = tuple(lats) or self.queue_laterals(n)
        l_max = max(abs(x) for x in lats)
        need = -self.cr_s_min + 0.24 + (l_max + self.cr_half_width) / math.tan(math.radians(self.queue_cone_deg))
        if dz_max > 0.0:
            off = math.hypot(l_max + self.cr_half_width, dz_max + self.cr_half_height)
            need = max(need, -self.cr_s_min + 0.24 + off / math.tan(math.radians(self.queue_cone_deg)))
        a = math.radians(self.merge_angle_deg)
        for k in range(len(lats) - 1):
            lq, ln = lats[k], lats[k + 1]
            if lq <= 0.3:
                continue
            d = (self.merge_clearance + ln * math.cos(a)) / math.sin(a)
            need = max(need, -self.merge_s + d)
        if len(lats) >= 2:
            for lq in lats:
                dl = max(abs(lq) - self.hold_lateral_allow, 0.0)
                if dl < self.merge_clearance:
                    need = max(need, -self.stage2_hold_s + math.sqrt(self.merge_clearance ** 2 - dl ** 2))
        need = max(need, -self.stage2_hold_s)
        return -need

    def queue_laterals(self, n: int, spacing: float = 0.0) -> Tuple[float, ...]:
        """Abreast queue points, left (positive) first."""
        return tuple(((n - 1) / 2.0 - r) * (spacing or self.queue_spacing) for r in range(n))


@dataclass(frozen=True)
class FormationRule:
    """P3: slot tracking with the own navigation + sonar range-based relative correction."""

    e_lost: float = 1.2
    e_ok: float = 0.5
    t_ok_hold: float = 2.0
    k_slot: float = 0.45
    k_sonar: float = 0.20
    v_corr_max: float = 0.10
    k_progress: float = 0.08
    progress_max: float = 2.0
    k_lateral: float = 0.08
    v_nominal: float = 0.30
    v_slot_max: float = 0.40
    v_recovery_max: float = 0.50
    range_gate_m: float = 1.2
    neighbour_range_m: float = 7.5
    t_reinclude: float = 1.0
    rejoin_lane_tol_m: float = 1.0
    rejoin_back_m: float = 2.0
    pass_dz_m: float = 3.5
    t_silent_s: float = 45.0
    v_wait: float = 0.20
    a_wait: float = 0.02
    t_rejoin_margin: float = 5.0
    t_rejoin_max: float = 60.0
    follow_on_m: float = 0.08
    follow_off_m: float = 0.04
    a_follow: float = 0.05
    v_catch_min: float = 0.10

    def keeping_cap(self, v_plan: float) -> float:
        """Horizontal speed cap of the formation-keeping modes at the planned reference speed ``v_plan`` [m/s].

        The slot moves at up to v_plan, so a keeping cap at or below v_plan leaves no along-track catch-up authority:
        a drone behind its slot by less than e_lost stays behind for ever (formal C1m).  The cap is v_slot_max, raised
        to v_plan + v_catch_min when the plan is faster, never above the recovery cap (the vehicle authority)."""
        return min(self.v_recovery_max, max(self.v_slot_max, v_plan + self.v_catch_min))

    def check_clock_speed(self, v_plan: float) -> None:
        """Operating-domain precondition of P3 (formal C4), checked when a mission plan is loaded: the planned speed
        of the formation reference leaves the catch-up margin v_catch_min below the recovery cap (hence below the
        keeping cap).  Raises ValueError on violation."""
        if not v_plan + self.v_catch_min <= self.v_recovery_max + 1e-9:
            raise ValueError(f"formation clock speed {v_plan:.3f} m/s leaves less than the catch-up margin "
                             f"v_catch_min = {self.v_catch_min:.2f} m/s below v_recovery_max = {self.v_recovery_max:.2f} m/s "
                             "(P3 operating domain, formal C4)")

    @property
    def catch_up(self) -> float:
        """Relative speed at which a recovering drone closes on a waiting formation [m/s]."""
        return self.v_recovery_max - self.v_wait

    @property
    def t_rejoin(self) -> float:
        """Missing time after which an expected neighbour's slot is declared vacant [s].

        The time a lost drone that is still at the edge of the sensing range (``neighbour_range_m`` behind its
        slot) needs to close the gap at the catch-up speed of the waiting formation, plus a margin, capped:
        min(t_rejoin_max, 7.5 / (0.50 - 0.20) + 5.0) = 30.0 s.  The gap is a configured bound, not an onboard
        estimate: after its last association the missing drone is not observed at all, and the last residual
        (within the range gate) says nothing about how far it drifted since.  A drone still missing after
        t_rejoin is either unable to move or beyond the sensing range; the others stop waiting and keep a
        degraded formation (no reconfiguration).  The slot is re-included if the drone comes back later."""
        if self.catch_up <= 0.0:
            raise ValueError("v_recovery_max must exceed v_wait (no catch-up)")
        return min(self.t_rejoin_max, self.neighbour_range_m / self.catch_up + self.t_rejoin_margin)


@dataclass(frozen=True)
class PlantCalibration:
    """Measured in calibration/probe_motion.py, calibration/probe_current.py and calibration/calibrate_plant.py."""

    surge_speed_per_cmd: float = 2.4
    sway_speed_per_cmd: float = 2.4
    heave_speed_per_cmd: float = 3.4
    thruster_limit: float = 12.0
    cmd_quadratic: float = 0.203
    cmd_model_tolerance: float = 0.06
    current_cmd_to_drift: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))
    current_cmd_to_drift_z: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))


@dataclass(frozen=True)
class RefereeConfig:
    """Ground-truth judging (never visible to a controller, referee/criteria.py): one tolerance band of 1.0 m opens
    an episode (error above it) and closes it (inside it for t_ok_hold [s]).  t_ok_hold = 4 / k_slot: the 2 % settling
    time of the first-order slot-tracking loop (criteria.t_hold_s; a test keeps them equal)."""

    e_lost: float = 1.0
    e_ok: float = 1.0
    t_ok_hold: float = 8.9


COMMUNICATION_ACOUSTIC = "ACOUSTIC_RB"
"""``BeaconConfig.mode`` of the controller: the passive one-way acoustic beacons (B2)."""
COMMUNICATION_DISABLED = "NONE"
"""``BeaconConfig.mode`` of the communication-disabled baseline (B0): the SAME controller, no beacon is emitted or heard."""


@dataclass(frozen=True)
class BeaconConfig:
    """The passive one-way acoustic beacons of the controller (B2); ``mode = COMMUNICATION_DISABLED`` switches them off.

    Every drone emits, on a pre-assigned TDMA schedule of period ``period_s``, a coded ping carrying its static
    identity and a short data packet with its ONBOARD position estimate, a conservative bound of that estimate's error
    and its gate phase (sim/beacon.py, perception/payload.py).  Strictly
    one-way broadcast: no handshake, no reply, no acknowledgement, no control decision; receivers measure range
    (one-way travel time, synchronised clocks) and direction (receiving array) themselves.  Transport: HoloOcean's AcousticBeaconSensor driven by
    sim/beacon.py, measurement model sim/acoustic_channel.py (level ``channel``, simulator side only), onboard use
    perception/acoustic.py with the declared envelope AcousticConfig.  With the communication disabled nothing is heard:
    silence is never evidence that a passage is safe (perception/acoustic_gate.py, formal/check_b2_comm.py)."""

    mode: str = COMMUNICATION_ACOUSTIC
    period_s: float = 1.0
    max_range_m: float = 50.0
    check_visible: bool = True
    sound_speed: float = 1500.0
    channel: str = "CLEAN"
    payload_bitrate_bps: float = 1000.0

    @property
    def enabled(self) -> bool:
        return self.mode == COMMUNICATION_ACOUSTIC


@dataclass(frozen=True)
class AcousticConfig:
    """Onboard side of the range-bearing beacon (perception/acoustic.py): the DECLARED measurement envelope.

    The perception does not know the channel level of a run: it uses these bounds everywhere
    (config/parameter_sources.json).  A reception inside the envelope has
    |range error| <= k_sigma * sigma_range_m + sound_speed_err * r + clock_offset_m and direction errors within
    k_sigma * sigma + the array calibration bias + the own attitude error bound; anything else (a reflected
    arrival, a clear-path multipath episode) is an outlier that the
    kinematic gate and the depth check must reject or the fix must not be used.  The envelope is the MEDIUM channel
    (sim/acoustic_channel.py) at k_sigma = 3, the heading bound the MEDIUM compass bias (sim/sensor_noise.py) plus
    its noise, the pitch / roll bound the unestimated attitude of a BlueROV2 holding depth."""

    k_sigma: float = 3.0
    sigma_range_m: float = 0.15
    sound_speed_err: float = 0.03
    clock_offset_m: float = 0.21
    sigma_az_deg: float = 2.5
    sigma_el_deg: float = 2.5
    doa_bias_deg: float = 3.0
    los_excess_max_m: float = 3.0
    heading_err_deg: float = 3.0
    tilt_err_deg: float = 3.0
    latency_max_s: float = 0.40
    nav_err_gate_m: float = 1.0
    track_gate_m: float = 0.5
    exit_confirm: int = 3
    tie_window_s: float = 3.0
    fresh_s: float = 2.5
    depth_tol_m: float = 1.5
    t_silent_gate_s: float = 70.0
    status_fresh_s: float = 2.5
    nav_b0_m: float = 0.30
    dvl_scale_err: float = 0.01
    dvl_sigma_body_mps: float = 0.0285
    nav_depth_err_m: float = 0.45
    dr_model_tau_s: float = 0.5
    admit_fresh_s: float = 3.5
    """B2 admission (formal/check_b2_comm.py): a status report of another drone is FRESH for the admission when its emission
    (TDMA slot, own clock) is at most admit_fresh_s old.  Derived: with at most k = 2 consecutive losses on a link the latest
    report is at most (k + 1) period_s + latency_max_s = 3.4 s old, so a channel with that loss pattern never degrades the
    admission."""
    commit_admit_min_s: float = 4.0
    """B2 admission: no admission earlier than commit_admit_min_s after the own commitment (own clock).  Derived: it must exceed
    admit_fresh_s + 2 clock_skew_s + one control step (0.1 s, the age of the phase a report carries) = 3.7 s, so that a fresh
    report of a drone committed before me (or admitted before me) is emitted after its commitment, i.e. is a claim.  Below the
    natural time from a queue point to the admission zone (4.2 s along the axis at v_pass for n = 2, more along the pass path): no
    slowdown in nominal runs."""
    clock_skew_s: float = 0.05
    """Bound of the offset between two drones' clocks used to date a TDMA slot (synchronised clocks; hardware assumption H6:
    the range-equivalent clock_offset_m = 0.21 m is 0.14 ms, this bound is a generous allowance for drift over a mission)."""


@dataclass(frozen=True)
class FailureModel:
    """Simulator side only (never read onboard): the failure model of a drone, a deployment assumption.

    A broken drone is completely broken: propulsion, electronics and acoustic transmission stop together, and, being
    ballasted positively buoyant (a deployment requirement of the vehicle, ~ +0.2 kg net), it rises.  The engine's
    BlueROV2 is exactly neutral, so the ascent is applied as a vertical current command on that drone only, added to the
    current drift vector (holo_env.truth; the terminal speed is calibrated by calibration/probe_buoyant_ascent.py).
    ``v_rise_mps``: steady ascent of net buoyancy F against the vertical drag: sqrt(2 F / (rho C_d A)) for F = 1.96 N
    and C_d A of the BlueROV2 Heavy plan area (0.458 x 0.576 m x 1.1) gives about 0.12 m/s; with an identified heave damping of
    the vehicle (linear 33 N s/m, quadratic 190 N s^2/m^2) it is 0.047 m/s, 0.102 m/s with the quadratic term
    alone.  Credible range 0.05 - 0.13 m/s; the nominal 0.10 m/s is in its upper part and is an assumption: the +0.2 kg is not
    measured; the sensitivity runs use the ends of the range (*_rise_low, *_rise_high).  ``surface_z_m``: the centre stops rising
    here (just awash).  It is an emulation: the engine's drag acts on the whole relative velocity."""

    v_rise_mps: float = 0.10
    surface_z_m: float = -0.25


@dataclass(frozen=True)
class ScopeConfig:
    """The operating scope of the permanent-failure model (holo_fleet/scope.py), known onboard: what a broken drone can do at
    most, and the current declared for the mission.  ``v_rise_min_mps`` below the model value of FailureModel (0.047 m/s with the
    identified heave damping): a hull that does not rise (a leak) is outside the model.  ``coast_h_m``: measured in the simulator,
    at most 0.39 m at 0.39 m/s through the water and at most 1.22 s times the speed at the failure, about 0.9 m
    at 0.75 m/s; not validated on a vehicle.  ``coast_v_m``: the vertical speed is at most 0.25 m/s.  ``rise_lag_s``: the heave
    time constant is about 0.35 s (mass over linear damping).  ``drift_factor``: a hull drifted up to 4 % faster than the effective
    drift in the runs.  ``t_lost_s``: a drone fails within this time of its last reception heard (one TDMA period): the margin
    of its last fix.  ``w_h_max_mps``, ``w_down_max_mps``: the current declared for the mission (the runner declares the bound of
    the scenario's field)."""

    v_rise_min_mps: float = 0.045
    coast_h_m: float = 1.0
    coast_v_m: float = 0.3
    rise_lag_s: float = 2.0
    drift_factor: float = 1.1
    surface_z_m: float = -0.25
    t_lost_s: float = 1.0
    w_h_max_mps: float = 0.0
    w_down_max_mps: float = 0.0


@dataclass(frozen=True)
class FleetConfig:
    sep: SeparationThresholds = field(default_factory=SeparationThresholds)
    env: Envelope = field(default_factory=Envelope)
    gate: GateRule = field(default_factory=GateRule)
    form: FormationRule = field(default_factory=FormationRule)
    perc: PerceptionConfig = field(default_factory=PerceptionConfig)
    plant: PlantCalibration = field(default_factory=PlantCalibration)
    ref: RefereeConfig = field(default_factory=RefereeConfig)
    beacon: BeaconConfig = field(default_factory=BeaconConfig)
    acoustic: AcousticConfig = field(default_factory=AcousticConfig)
    failure: FailureModel = field(default_factory=FailureModel)
    scope: ScopeConfig = field(default_factory=ScopeConfig)
    traffic_rule: bool = True

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT = FleetConfig()


def communication_disabled(cfg: FleetConfig = DEFAULT) -> FleetConfig:
    """The communication-disabled baseline (B0) of ``cfg``: the same controller with the acoustic beacons switched off.
    Nothing else differs (tests/test_single_controller.py)."""
    return dataclasses.replace(cfg, beacon=dataclasses.replace(cfg.beacon, mode=COMMUNICATION_DISABLED))
