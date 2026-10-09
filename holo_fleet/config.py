"""Single source of truth for thresholds, envelope assumptions and controller gains (v2).

Every number used by the formal models in ``formal/`` is imported from here, so the verified
abstraction and the deployed controller cannot silently diverge.  Sensor numbers come from the
Phase-1 probe (docs/v2/SONAR_PROBE.md), plant numbers from scripts/calibrate_plant.py.
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
    """Assumptions of the formal proofs (each one measured back on the logs, results/v2/ASSUMPTIONS.md)."""

    dt: float = 0.1                  # controller period [s]
    tau_max: float = 0.25            # max age of the sonar data used by a guard [s] (one dropped 10 Hz capture)
    eps_range_far: float = 0.12      # echo may be this much FARTHER than the true near surface [m]
                                     # (one 5 cm bin + 6 cm octree leaf + the +5 cm threshold bias, Phase 1)
    eps_range_near: float = 0.08     # ... or this much nearer [m]
    v_max_nominal: float = 0.50      # speed cap outside avoidance (formation recovery) [m/s]
    v_escape: float = 0.55           # escape speed in COLLISION_AVOIDANCE [m/s]
    a_brake: float = 0.50            # closing-speed reduction per drone [m/s^2] (calibrated >= 0.74)
    w_rel_max: float = 0.15          # unrejected differential drift between two drones [m/s]
    w_drift_max: float = 0.15        # unrejected absolute drift of one drone [m/s]
                                     # (residuals after compensation: they hold while the requested velocity is
                                     #  control-feasible, see control/current_envelope.py)
    g_min: float = 0.50              # certified opening per unit speed of the escape table (single threat), S2a
    v_open: float = 0.20             # opening speed demanded by the warning filter [m/s]
    # current envelope (DI-27): a current is acceptable when the requested velocity stays control-feasible
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
                                      # echoes (max of ~10^3 leaves ~ 0.05 ln N ~ 0.4 m), DI-14
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
    hold_err_m: float = 0.10          # measured position-holding error of a queued drone (results/v2/ASSUMPTIONS.md)

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
    v_nominal: float = 0.30           # survey speed [m/s]
    v_slot_max: float = 0.40          # speed cap while following
    v_recovery_max: float = 0.50      # speed cap while recovering (catch-up margin 0.2 m/s; = Envelope.v_max_nominal)
    range_gate_m: float = 1.2         # an echo is associated to an expected neighbour within this
    neighbour_range_m: float = 7.5    # expected neighbours nearer than this must be seen (neighbors_ok)


@dataclass(frozen=True)
class PlantCalibration:
    """Measured in probe/probe_motion.py, probe/probe_current.py and scripts/calibrate_plant.py."""

    surge_speed_per_cmd: float = 2.4      # small-signal slope [m/s per unit command]: the low level's linear model
    sway_speed_per_cmd: float = 2.4
    heave_speed_per_cmd: float = 3.4
    thruster_limit: float = 12.0
    # steady command needed per horizontal axis for a through-water speed r [m/s]:
    #   g(r) = |r| / surge_speed_per_cmd + cmd_quadratic * r^2
    # fitted on results/calibration/head_current_authority.json (probe/probe_head_current_authority.py);
    # largest residual on the calibration points: cmd_model_tolerance (relative)
    cmd_quadratic: float = 0.203
    cmd_model_tolerance: float = 0.06
    # commanded set_ocean_currents magnitude -> steady horizontal drift of an unactuated BlueROV2
    current_cmd_to_drift: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))
    # vertical component (probe/probe_current.py --vertical)
    current_cmd_to_drift_z: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))


@dataclass(frozen=True)
class RefereeConfig:
    """Ground-truth judging (never visible to a controller)."""

    e_lost: float = 1.2
    e_ok: float = 0.5
    t_ok_hold: float = 2.0


@dataclass(frozen=True)
class FleetConfig:
    sep: SeparationThresholds = field(default_factory=SeparationThresholds)
    env: Envelope = field(default_factory=Envelope)
    gate: GateRule = field(default_factory=GateRule)
    form: FormationRule = field(default_factory=FormationRule)
    perc: PerceptionConfig = field(default_factory=PerceptionConfig)
    plant: PlantCalibration = field(default_factory=PlantCalibration)
    ref: RefereeConfig = field(default_factory=RefereeConfig)
    comms_enabled: bool = False       # inter-agent communication is OFF by default
    traffic_rule: bool = True         # head-on give-way rule (liveness aid); off only to exercise the safety layer alone

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT = FleetConfig()
