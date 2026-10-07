"""Single source of truth for thresholds, envelope assumptions and controller gains (v2).

Every number used by the formal models in ``formal/`` is imported from here, so the verified
abstraction and the deployed controller cannot silently diverge.  Sensor numbers come from the
Phase-1 probe (docs/v2/SONAR_PROBE.md), plant numbers from scripts/calibrate_plant.py.
"""

from __future__ import annotations

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

    d_nominal: float = 3.0           # smallest inter-slot distance of every formation template
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
    tau_max: float = 0.2             # max age of the sonar data used by a guard [s] (10 Hz + one period)
    eps_range_far: float = 0.12      # echo may be this much FARTHER than the true near surface [m]
                                     # (one 5 cm bin + 6 cm octree leaf + the +5 cm threshold bias, Phase 1)
    eps_range_near: float = 0.08     # ... or this much nearer [m]
    v_max_nominal: float = 0.40      # speed cap outside avoidance [m/s]
    v_escape: float = 0.50           # escape speed in COLLISION_AVOIDANCE [m/s]
    a_brake: float = 0.50            # closing-speed reduction per drone [m/s^2] (calibrated >= 0.74)
    w_rel_max: float = 0.15          # unrejected differential drift between two drones [m/s]
    w_drift_max: float = 0.15        # unrejected absolute drift of one drone [m/s]
    g_min: float = 0.55              # guaranteed opening per unit speed of the escape table (single threat)
    v_open: float = 0.20             # opening speed demanded by the warning filter [m/s]
    current_drift_max: float = 0.6   # largest effective horizontal current drift we claim to handle [m/s]
    current_vertical_max: float = 0.25
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
    seabed_tol_m: float = 0.70     # rough, vegetated seabed (bench: kelp up to ~0.6 m above the DVL bottom)
    confirm_captures: int = 2         # a dynamic echo is confirmed after this many of the last 3 captures
    track_drop_s: float = 1.2         # a sector track without echoes for this long is dropped
    closing_rate_alpha: float = 0.5   # alpha-beta filter of the range rate
    v_close_staleness: float = 1.05   # = Envelope.c_max: staleness inflation of the conservative distance
    beam_dropout: float = 0.0         # per-capture dropout probability of a sonar (stress demos only)
    blackout_every_s: float = 0.0     # stress demos only
    blackout_len_s: float = 0.0
    camera_hz: int = 5


@dataclass(frozen=True)
class GateRule:
    """P2 critical-region protocol (gate frame: s along the axis towards the exit, l left, z up).

    The CR is |s| <= cr_half_len, |l| <= cr_half_width, |z| <= cr_half_height around the gate centre.
    Queue points are abreast on the queue line s = s_queue, ``queue_spacing`` apart, so that two
    queued drones see each other in a pure LEFT/RIGHT relation.  Decisions use only sector
    patterns (gate_rule.py), the own navigation estimate and the gate map.
    """

    cr_half_len: float = 0.8
    cr_half_width: float = 1.0
    cr_half_height: float = 1.0
    s_queue: float = -4.0
    queue_spacing: float = 2.6        # > d_warning_exit: queued neighbours never trigger the warning
    queue_tol: float = 0.45           # |position - queue point| to count as queued
    heading_tol_deg: float = 12.0     # queued drones face the gate within this (formal tolerance 15 deg)
    approach_len: float = 6.0         # approach zone: s in [s_queue - approach_len, s_queue]
    corridor_half_width: float = 4.5
    t_clear: float = 1.0              # sectors / corridor must be clear this long before committing [s]
    t_occ_max: float = 25.0           # a departure seen but no exit seen: CR held busy this long [s]
    exit_s: float = 3.0               # a passing drone counts as exited beyond this s
    rally_s: float = 9.0              # passed drones gather this far beyond the gate (outside the corridor view)
    v_approach: float = 0.25
    v_pass: float = 0.30
    queue_bracket_m: float = 6.0      # neighbours closer than this count for the queue decision


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
    v_slot_max: float = 0.40          # = v_max_nominal
    range_gate_m: float = 1.2         # an echo is associated to an expected neighbour within this


@dataclass(frozen=True)
class PlantCalibration:
    """Measured in probe/probe_motion.py, probe/probe_current.py and scripts/calibrate_plant.py."""

    surge_speed_per_cmd: float = 2.4
    sway_speed_per_cmd: float = 2.4
    heave_speed_per_cmd: float = 3.4
    thruster_limit: float = 12.0
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

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT = FleetConfig()
