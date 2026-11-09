"""Single source of truth for thresholds, envelope assumptions and controller gains (v2).

Every number used by the formal models in ``formal/`` is imported from here, so the verified
abstraction and the deployed controller cannot silently diverge.  Sensor numbers come from the
Phase-1 probe (docs/v2/SONAR_PROBE.md), plant numbers from calibration/calibrate_plant.py.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Dict, Tuple

from holo_fleet.perception.sonar_geometry import DEFAULT_SONAR, R_IN, R_OUT, SonarModel


@dataclass(frozen=True)
class SeparationThresholds:
    """P1 thresholds.  ``d_safe``/``d_collision`` are ground-truth centre distances (referee only);
    the guards compare the *conservative onboard* distance (a lower bound of the centre distance,
    see Envelope.d_lower_offset) with ``d_ca`` / ``d_warning``.

    d_nominal > d_warning_exit > d_warning > d_ca_exit > d_ca > d_safe > d_collision.
    The values of d_warning and d_ca are the smallest ones for which formal/check_separation.py
    proves G(d >= d_safe) on the pairwise model, plus a margin (see REPORT.md).
    """

    d_nominal: float = 3.5           # smallest inter-slot distance of every formation template
    d_warning: float = 2.4
    d_warning_exit: float = 2.7
    d_ca: float = 1.7
    d_ca_exit: float = 2.0
    d_safe: float = 1.0
    d_collision: float = 0.6         # BlueROV2 hulls touch around this centre distance


@dataclass(frozen=True)
class Envelope:
    """Assumptions of the formal proofs (each one measured back on the logs, results/reference/ASSUMPTIONS.md)."""

    dt: float = 0.1                  # controller period [s]
    tau_max: float = 0.25            # max age of the sonar data used by a guard [s] (one dropped 10 Hz capture)
    eps_range_far: float = 0.12      # echo may be this much FARTHER than the true near surface [m]: measured in
                                     # HoloOcean (largest +0.054 m, Phase 1 probe) plus margin; the MEDIUM profile's
                                     # +-2-bin range jitter exceeds it in ~0.6 % of captures per side (docs/
                                     # PARAMETER_PROVENANCE.md): outside the proved P1 envelope
    eps_range_near: float = 0.08     # ... or this much nearer [m]
    v_max_nominal: float = 0.50      # speed cap outside avoidance (formation recovery) [m/s]
    v_escape: float = 0.55           # escape speed in COLLISION_AVOIDANCE [m/s]
    a_brake: float = 0.50            # closing-speed reduction per drone [m/s^2] (calibrated >= 0.74)
    a_brake_measured: float = 0.743  # closed-loop stop in the simulator (plant_calibration.json): formal S1d only
    w_rel_max: float = 0.15          # unrejected differential drift between two drones [m/s]
    w_drift_max: float = 0.15        # unrejected absolute drift of one drone [m/s]
                                     # (residuals after compensation: they hold while the requested velocity is
                                     #  control-feasible, see control/current_envelope.py)
    g_min: float = 0.50              # certified opening per unit speed of the escape table (single threat), S2a
    v_open: float = 0.20             # opening speed demanded by the warning filter [m/s]
    # current envelope: a current is acceptable when the requested velocity stays control-feasible
    # against it (control/current_envelope.py: steady command within the authority, direction-dependent)
    # AND it lies in the exercised range below.  The range is NOT a controllability bound: against the
    # motion at 0.30 m/s the authority already stops at about 0.41 m/s.
    current_validated_max: float = 0.6     # largest horizontal current magnitude exercised (calibration, demos) [m/s]
    current_vertical_max: float = 0.25     # declared vertical range (separate heave channel, not binding) [m/s]
    struct_keepout: float = 0.70     # never close on mapped structure nearer than this [m]
    z_min: float = -9.0              # operating depth band (world z, up positive)
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
    merge_gap_bins: int = 2           # echo segments closer than this are merged
    vehicle_max_extent_m: float = 0.9 # a single BlueROV2 echo spans less than this in range
    structure_tol_m: float = 0.30     # echo explained by the gate map if within this of a predicted return
    structure_tol_far_m: float = 0.70 # ... on the far side: the per-leaf exponential range noise only lengthens
                                      # echoes (max of ~10^3 leaves ~ 0.05 ln N ~ 0.4 m)
    seabed_tol_m: float = 0.70     # rough, vegetated seabed (bench: kelp up to ~0.6 m above the DVL bottom)
    confirm_captures: int = 2         # a dynamic echo is confirmed after this many of the last 3 captures
    track_drop_s: float = 1.2         # a sector track without echoes for this long is dropped
    closing_rate_alpha: float = 0.5   # alpha-beta filter of the range rate
    v_close_staleness: float = 1.20   # = Envelope.c_max: staleness inflation of the conservative distance
    beam_dropout: float = 0.0         # per-capture dropout probability of a sonar (stress demos only)
    blackout_every_s: float = 0.0     # stress demos only
    blackout_len_s: float = 0.0
    camera_hz: int = 5


@dataclass(frozen=True)
class GateRule:
    """P2 critical-region protocol (gate frame: s along the axis towards the exit, l left, z up).

    The CR is |s| <= cr_half_len, |l| <= cr_half_width, |z| <= cr_half_height around the gate centre.
    Queue points are abreast on the queue line s = queue_s(n), ``queue_spacing`` apart, so that two
    queued drones see each other in a pure LEFT/RIGHT relation; the line is far enough back that the
    whole CR lies inside every queued drone's FRONT cone (``queue_cone_deg`` + heading tolerance <
    60 deg).  Decisions use only sector patterns (ha/mutex_rule.py), the own navigation estimate and
    the gate map.  A committed drone leaves the queue line on a path that keeps ``merge_clearance``
    from the queue points on its right (``pass_path``), joins the axis ``merge_ahead`` beyond the
    queue line, crosses the gate and veers towards its own formation lane after ``veer_s``.
    """

    cr_half_len: float = 0.8
    cr_half_width: float = 1.0
    cr_half_height: float = 1.0
    queue_spacing: float = 3.5        # > d_warning_exit + estimate margin: queued neighbours never trigger the warning
    queue_cone_deg: float = 49.0      # CR bearing from every queue sonar <= this (+ heading_tol 6 + hull fuzz 5 = 60)
    queue_tol: float = 0.35           # |position - queue point| to count as queued
    heading_tol_deg: float = 6.0      # queued drones face the gate within this
    approach_len: float = 7.0         # approach zone: s in [queue_s - approach_len, exit_s]
    corridor_half_width: float = 6.5
    t_clear: float = 1.0              # PRIORITY / a free CR must persist this long before committing [s]
    t_occ_max: float = 25.0           # CR seen busy, no exit seen: belief returns FREE after this [s] (logged)
    nav_err_m: float = 1.0            # declared own gate-frame navigation error (= AcousticConfig.nav_err_gate_m):
                                      # the give-up plane of a stage-2 hold lies this far beyond R_OUT before the CR
    merge_angle_deg: float = 55.0     # committed drones descend to the axis on this heading (from the axis)
    merge_before_cr: float = 0.7      # ... and are on the axis this far before the CR
    merge_clearance: float = 3.2      # ... keeping at least this from every queue point still occupied
    veer_s: float = 1.0               # beyond the CR the passing drone veers towards its own lane
    exit_s: float = 3.5               # a passing drone has passed beyond this s (FORMATION_RECOVERY)
    rally_s: float = 6.0              # formation reference beyond the gate (rendezvous)
    v_approach: float = 0.50
    v_pass: float = 0.50
    k_track: float = 0.6              # proportional gain of the gate-path tracking [1/s]
    queue_bracket_m: float = 7.5      # LEFT/RIGHT/REAR neighbours closer than this count for the queue decision
    # vertically stacked queue (slots that differ only in depth): columns farther apart than the stack spacing,
    # so that a diagonal neighbour is always seen in LEFT/RIGHT (left first decides) and a neighbour of the same
    # column only in UP/DOWN (top first decides), within the position / heading tolerances (formal M2v); every
    # pair stays inside the queue bracket
    stack_column_spacing: float = 5.0
    stack_spacing: float = 3.5
    # validated domain of the gate passage: every queued drone gets a sonar exit witness of the drone that passes
    # (formal M4g) only for queue points within this lateral offset of the axis - the abreast queue up to 3 drones,
    # the 2 x 2 and 3-drone stacks; beyond it (the abreast queue of 4-6, a 3-column stack) the outer drones see no
    # exit and the planner refuses the passage (mission.check_gate_domain)
    queue_lateral_max: float = 3.5
    hold_err_m: float = 0.10          # measured position-holding error of a queued drone (results/reference/ASSUMPTIONS.md)

    @property
    def merge_s(self) -> float:
        return -(self.cr_half_len + self.merge_before_cr)

    def queue_s(self, n: int, dz_max: float = 0.0, lats: Tuple[float, ...] = ()) -> float:
        """Queue line, the smaller of two bounds:
        (a) the CR lies inside the FRONT cone (``queue_cone_deg``) of every queue point (with a vertical
            stack of queue points, ``dz_max`` > 0, the off-axis distance includes the vertical offset);
        (b) a drone left of the axis that descends to the merge point at ``merge_angle_deg`` keeps
            ``merge_clearance`` from its right neighbour's queue point (the next in the order).
        ``lats``: the column laterals (default: the abreast queue of n columns)."""
        lats = tuple(lats) or self.queue_laterals(n)
        l_max = max(abs(x) for x in lats)
        # bearing of the far CR corners seen from the FRONT sonar (0.24 m ahead of the centre)
        need = self.cr_half_len + 0.24 + (l_max + self.cr_half_width) / math.tan(math.radians(self.queue_cone_deg))
        if dz_max > 0.0:
            off = math.hypot(l_max + self.cr_half_width, dz_max + self.cr_half_height)
            need = max(need, self.cr_half_len + 0.24 + off / math.tan(math.radians(self.queue_cone_deg)))
        a = math.radians(self.merge_angle_deg)
        for k in range(len(lats) - 1):
            lq, ln = lats[k], lats[k + 1]
            if lq <= 0.3:
                continue                          # right of the axis: it descends away from its right neighbour
            # distance from (s_q, ln) to the line through (merge_s, 0) with direction (-cos a, sin a):
            # |(s_q - merge_s) sin a + ln cos a| >= merge_clearance, with s_q < merge_s
            d = (self.merge_clearance + ln * math.cos(a)) / math.sin(a)
            need = max(need, -self.merge_s + d)
        return -need

    def queue_laterals(self, n: int, spacing: float = 0.0) -> Tuple[float, ...]:
        """Abreast queue points, left (positive) first."""
        return tuple(((n - 1) / 2.0 - r) * (spacing or self.queue_spacing) for r in range(n))


@dataclass(frozen=True)
class FormationRule:
    """P3: slot tracking with the own navigation + sonar range-based relative correction."""

    e_lost: float = 1.2               # formation error above which formation is lost [m]
    e_ok: float = 0.5                 # below this (for t_ok_hold) the formation is recovered [m]
    t_ok_hold: float = 2.0
    k_slot: float = 0.45              # slot-tracking gain [1/s]
    k_sonar: float = 0.20             # gain of the sonar range correction [1/s]
    v_corr_max: float = 0.10          # cap of the sonar correction [m/s]
    k_progress: float = 0.08          # along-track progress correction from sonar ranges [1/s]
    progress_max: float = 2.0         # |local progress offset| cap [m]
    k_lateral: float = 0.08           # lateral counterpart of k_progress: integrated lateral slot offset [1/s]
    v_nominal: float = 0.30           # survey speed [m/s]
    v_slot_max: float = 0.40          # speed cap while following
    v_recovery_max: float = 0.50      # speed cap while recovering (catch-up margin 0.2 m/s; = Envelope.v_max_nominal)
    range_gate_m: float = 1.2         # an echo is associated to an expected neighbour within this
    neighbour_range_m: float = 7.5    # expected neighbours nearer than this must be seen (neighbors_ok)
    t_reinclude: float = 1.0          # a vacant slot is re-included after its drone is seen there this long [s]
    rejoin_lane_tol_m: float = 1.0    # a recovering drone farther than this from its own lane rejoins from behind
    rejoin_back_m: float = 2.0        # ... through a point on its lane this far behind the rear-most slot [m]
    # a slot with another slot behind it on its lane (lane and level nearer than pass_dz_m: square front drones,
    # the tee centre drone) is reached from behind on a level pass_dz_m below (above when the depth band has no
    # room), the drone rising only past that slot: the vertical give-way of the traffic rule, at the queue spacing
    # (> d_warning_exit + estimate margin), so the drone of the rear slot never stands in the way
    pass_dz_m: float = 3.5
    # reduced progress while an expected neighbour is missing (control/formation_time.py): the formation
    # reference of a drone that misses a neighbour advances at v_wait instead of the planned speed, reached and
    # left with a ramp of a_wait.  v_wait = 0.20: the catch-up of a rejoining drone grows from 0.20 to 0.30 m/s
    # (v_recovery_max - v_wait); the along-track offset between two drones that notice the absence at different
    # times grows at v_nominal - v_wait = 0.10 m/s, so a detection spread of 7 s (lost-drone runs) stays below
    # e_ok after the ramp (0.10 * (7 - T_ramp / 2) = 0.45 m); slower than that would trade formation keeping
    # for catch-up.  Never 0: the fleet keeps surveying and keeps steerage.
    # silence of a drone: not heard for this long = dead (TOTAL_FAILURE: nothing is emitted any more), whatever the
    # observer's own state.  Justified against the channel: in MEDIUM-ALL the longest silence between two drones of the
    # 72 v10 runs was 31 s, median over the runs 9 s, reception rate ~45-70 %.  Silence never clears a possibly occupied
    # gate (perception/acoustic_gate.py)
    t_silent_s: float = 45.0
    v_wait: float = 0.20
    a_wait: float = 0.02              # [m/s^2]: 0.30 -> 0.20 m/s in 5 s, far below the braking capability (a_brake)
    t_rejoin_margin: float = 5.0      # [s] on top of the catch-up time: ramp-in (~0.8 s), confirmation (<= 1 s) and
                                      # the proportional final approach (~1.5 s), rounded up
    t_rejoin_max: float = 60.0        # upper limit of the wait, whatever the configured speeds [s]
    # keeping the spacing to the drone ahead on the same lane (control/formation_time.py): a drone whose front
    # neighbour is measured closer than its template distance by more than follow_on slows its reference too, until
    # the compression is below follow_off; on its slot, a slowing drone keeps slowing until the drone ahead is
    # farther than planned by follow_on (and does not start while it is).  Drones notice an absence at different
    # times, or not at all (range-only sensing: an echo of another neighbour can stand in for the missing one);
    # without this a rear drone at the survey speed behind a slowed one would close on it - the warning band of a
    # same-lane pair at 3.5 m starts about 0.23 m closer.  follow_on: above the calm-flight residual (one 5 cm range
    # bin); a_follow = 2.5 a_wait keeps the compression below ~0.14 m when the drone ahead slows at a_wait.
    follow_on_m: float = 0.08
    follow_off_m: float = 0.04
    a_follow: float = 0.05            # [m/s^2]

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

    surge_speed_per_cmd: float = 2.4      # small-signal slope [m/s per unit command]: the low level's linear model
    sway_speed_per_cmd: float = 2.4
    heave_speed_per_cmd: float = 3.4
    thruster_limit: float = 12.0
    # steady command needed per horizontal axis for a through-water speed r [m/s]:
    #   g(r) = |r| / surge_speed_per_cmd + cmd_quadratic * r^2
    # fitted on results/calibration/head_current_authority.json (calibration/probe_head_current_authority.py);
    # largest residual on the calibration points: cmd_model_tolerance (relative)
    cmd_quadratic: float = 0.203
    cmd_model_tolerance: float = 0.06
    # commanded set_ocean_currents magnitude -> steady horizontal drift of an unactuated BlueROV2
    current_cmd_to_drift: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))
    # vertical component (calibration/probe_current.py --vertical)
    current_cmd_to_drift_z: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))


@dataclass(frozen=True)
class RefereeConfig:
    """Ground-truth judging (never visible to a controller).  Criteria v3 (referee/criteria.py): one tolerance band of
    1.0 m opens an episode (error above it) and closes it (inside it for t_ok_hold).  v2 used e_lost 1.2 / e_ok 0.5,
    whose band in between never opened an episode, and a 2 s hold.  t_ok_hold = 4 / k_slot: the 2 % settling time of
    the first-order slot-tracking loop (criteria.t_hold_s; a test keeps them equal)."""

    e_lost: float = 1.0
    e_ok: float = 1.0
    t_ok_hold: float = 8.9


@dataclass(frozen=True)
class BeaconConfig:
    """Variant B2 one-way acoustic beacon (OFF in the communication-free baseline B0).

    Every drone emits, on a pre-assigned TDMA schedule of period ``period_s``, a coded ping carrying its static
    identity and (``payload``) a short data packet with its ONBOARD position estimate and a conservative bound of
    that estimate's error (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, H6; sim/beacon.py, encode_payload).  Strictly
    one-way broadcast: no handshake, no reply, no acknowledgement, no control decision; receivers measure range
    (one-way travel time, synchronised clocks) and direction (receiving array) themselves.  Transport: HoloOcean's AcousticBeaconSensor driven by
    sim/beacon.py, measurement model sim/acoustic_channel.py (level ``channel``, simulator side only), onboard use
    perception/acoustic.py with the declared envelope AcousticConfig."""

    mode: str = "NONE"               # NONE (B0) | ACOUSTIC_RB (B2)
    period_s: float = 1.0
    max_range_m: float = 50.0
    check_visible: bool = True
    sound_speed: float = 1500.0
    channel: str = "CLEAN"           # simulator-side channel level (sim/acoustic_channel.py), never onboard
    payload: bool = True             # H6: the ping carries the sender's onboard position estimate and error bound
    payload_bitrate_bps: float = 1000.0   # H6: coherent short-packet rate assumed (PAYLOAD_BITS / rate per slot)

    @property
    def enabled(self) -> bool:
        return self.mode == "ACOUSTIC_RB"


@dataclass(frozen=True)
class AcousticConfig:
    """Onboard side of the B2 range-bearing beacon (perception/acoustic.py): the DECLARED measurement envelope.

    The perception does not know the channel level of a run: it uses these bounds everywhere, fixed before any B2
    result (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, section 3).  A reception inside the envelope has
    |range error| <= k_sigma * sigma_range_m + sound_speed_err * r + clock_offset_m and direction errors within
    k_sigma * sigma + the array calibration bias + the own attitude error bound; anything else (a reflected
    arrival, a clear-path multipath episode) is an outlier that the
    kinematic gate and the depth check must reject or the fix must not be used.  The envelope is the MEDIUM channel
    (sim/acoustic_channel.py) at k_sigma = 3, the heading bound the MEDIUM compass bias (sim/sensor_noise.py) plus
    its noise, the pitch / roll bound the unestimated attitude of a BlueROV2 holding depth."""

    k_sigma: float = 3.0
    sigma_range_m: float = 0.15
    sound_speed_err: float = 0.03        # 3 x 1 % (one scale factor per run)
    clock_offset_m: float = 0.21         # 3 x sqrt(2) x 0.05 m: the bias between two drones' clocks
    sigma_az_deg: float = 2.5
    sigma_el_deg: float = 2.5
    doa_bias_deg: float = 3.0            # calibration residual of the receiving array (MEDIUM amplitude)
    los_excess_max_m: float = 3.0        # a clear-path multipath episode can make a range this much too long (one-sided:
                                         # the region reaches this much nearer; channel reflect_extra_m upper bound)
    heading_err_deg: float = 3.0         # own compass bias + noise (MEDIUM 2.0 deg bias)
    tilt_err_deg: float = 3.0            # own unestimated roll / pitch
    latency_max_s: float = 0.40          # a delivery later than this after its arrival is not used
    nav_err_gate_m: float = 1.0          # own position error in the gate frame (a queued drone, bounded by aiding)
    track_gate_m: float = 0.5            # extra slack of the kinematic consistency gate of a track
    exit_confirm: int = 3                # consecutive accepted fixes (outside / beyond the CR) for an acoustic exit;
                                         # 3 not 2: calibration/analyze_b2_channel.py, persistent reflections (results/b2)
    tie_window_s: float = 3.0            # fixes this close to the last corridor sighting account for a drone
    fresh_s: float = 2.5                 # a fix this recent counts for the neighbours (formation)
    depth_tol_m: float = 1.5             # a fix this far outside the operating depth band is a reflection
    # a drone fallen silent is excluded as a possible occupant of a gate only after this longer interval (and only if
    # it was not engaged with the gate when it fell silent: never a drone that reported committed or admitted): the
    # argument is that a live drone cannot stay unheard this long on a link of the queue geometry (model of the MEDIUM
    # channel: P(silence >= 70 s) on a shadowed queue link < 1e-3 per mission, docs/ACOUSTIC_CHANNEL_MODEL.md), while a
    # dead drone has risen several metres above the gate by then (config.FailureModel)
    t_silent_gate_s: float = 70.0        # a silent drone leaves the identity premise of the map fit only (Perception._other_regions); never a gate decision
    # a gate phase NOT_ENGAGED excludes its sender as an occupant while fresher than this: not above the time a queued
    # drone needs to reach a position where the corridor echo of a real entry would stop being visible (formal/check_status.py,
    # M3-status BMC: 2.5 s holds, 4.5 s does not), and per plan not above the time from the queue point to the CR at the
    # speed bound with the queue tolerance and the declared navigation error (GatePerception.__init__)
    status_fresh_s: float = 2.5
    # onboard bound of the own dead-reckoning error (perception/nav.py, NavBound), sent in the payload (H6).  Every
    # term is the declared envelope of a sensor, never the profile of a run:
    nav_b0_m: float = 0.30               # [REQ] launch survey of the shared mission frame (horizontal)
    dvl_scale_err: float = 0.01          # [LIT] DVL velocity scale error (1 %: low-cost DVL class; Nortek <= 0.1 %)
    dvl_sigma_body_mps: float = 0.0285   # [PROF] white DVL body-velocity noise, MEDIUM: 0.015 m/s per beam x 1.9
    nav_depth_err_m: float = 0.45        # [PROF] depth sensor, 3 x the MEDIUM 0.15 m noise
    # DVL outage: model-aided dead reckoning (perception/nav.py).  The low level tracks the requested velocity as a
    # first-order lag; calibrated on 64 MEDIUM-ONE runs of the v5 campaign (analysis/scripts/cmd_velocity_model.py):
    dr_model_tau_s: float = 0.5          # [CAL] time constant of the commanded-velocity model (smallest p99 error)
    # the error bound of that model per outage length is the calibration table results/calibration/
    # dr_model_calibration.json (max integrated position error), read by perception/nav.NavBound


@dataclass(frozen=True)
class FailureModel:
    """Simulator side only (never read onboard): the failure model of a drone, the deployment's (the owner's decision).

    A broken drone is completely broken: propulsion, electronics and acoustic transmission stop together, and, being
    ballasted positively buoyant (a deployment requirement of the vehicle, ~ +0.2 kg net), it rises.  The engine's
    BlueROV2 is exactly neutral, so the ascent is applied as a vertical current command on that drone only, added to the
    current drift vector (holo_env.truth; the terminal speed is calibrated by calibration/probe_buoyant_ascent.py).
    ``v_rise_mps``: steady ascent of net buoyancy F against the vertical drag: sqrt(2 F / (rho C_d A)) for F = 1.96 N
    and C_d A of the BlueROV2 Heavy plan area (0.458 x 0.576 m x 1.1) gives about 0.12 m/s; with an identified heave damping of
    the vehicle (linear 33 N s/m, quadratic 190 N s^2/m^2, reviewer round 2) it is 0.047 m/s, 0.102 m/s with the quadratic term
    alone.  Credible range 0.05 - 0.13 m/s; the nominal 0.10 m/s is in its upper part and is an assumption: the +0.2 kg is not
    measured; the sensitivity runs use the ends of the range (*_rise_low, *_rise_high).  ``surface_z_m``: the centre stops rising
    here (just awash).  It is an emulation: the engine's drag acts on the whole relative velocity."""

    v_rise_mps: float = 0.10
    surface_z_m: float = -0.25


@dataclass(frozen=True)
class FleetConfig:
    sep: SeparationThresholds = field(default_factory=SeparationThresholds)
    env: Envelope = field(default_factory=Envelope)
    gate: GateRule = field(default_factory=GateRule)
    form: FormationRule = field(default_factory=FormationRule)
    perc: PerceptionConfig = field(default_factory=PerceptionConfig)
    plant: PlantCalibration = field(default_factory=PlantCalibration)
    ref: RefereeConfig = field(default_factory=RefereeConfig)
    beacon: BeaconConfig = field(default_factory=BeaconConfig)   # B2 one-way acoustic beacon (OFF in B0)
    acoustic: AcousticConfig = field(default_factory=AcousticConfig)   # B2 onboard envelope (used only when beacon.enabled)
    failure: FailureModel = field(default_factory=FailureModel)        # simulator side: what a broken drone does
    comms_enabled: bool = False       # inter-agent communication is OFF by default
    traffic_rule: bool = True         # head-on give-way rule (liveness aid); off only to exercise the safety layer alone

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT = FleetConfig()
