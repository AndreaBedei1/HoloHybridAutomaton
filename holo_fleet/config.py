"""Single source of truth for thresholds, envelope assumptions and controller gains.

Every number used by the formal models in ``formal/`` is imported from here, so
the verified abstraction and the deployed controller cannot silently diverge.
Plant-capability numbers come from the HoloOcean calibration probes documented
in DISCOVERY.md (BlueROV2, control scheme 0, marine_race_arena thruster map).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, Tuple


@dataclass(frozen=True)
class SeparationThresholds:
    """P1 thresholds, centre-to-centre distances in metres.

    d_nominal > d_warning > d_ca > d_safe > d_collision.
    ``d_ca`` is the hard collision-avoidance entry; the soft warning band
    [d_ca, d_warning) already removes every closing velocity component.
    """

    d_nominal: float = 3.0
    d_warning: float = 2.3
    d_warning_exit: float = 2.6      # hysteresis for leaving SEPARATION_WARNING
    d_ca: float = 1.6
    d_ca_exit: float = 1.9           # hysteresis for leaving COLLISION_AVOIDANCE
    d_safe: float = 1.0
    d_collision: float = 0.6         # BlueROV2 hull ~0.58 m long: centre distance at contact
    r_detect: float = 10.0           # proximity-sonar max range


@dataclass(frozen=True)
class Envelope:
    """Operational envelope assumed by the formal proofs (checked at runtime by the referee)."""

    dt: float = 0.1                  # controller period [s]
    tau_max: float = 0.2             # max perception staleness accepted as "fresh" [s] (sonar 10 Hz)
    eps_rel: float = 0.30            # bound on per-component relative-position error [m]
                                     # (measured in HoloOcean: 99.7% <= 0.25 m, max 0.28 m; see REPORT.md)
    v_max_nominal: float = 0.40      # speed cap in every non-avoidance mode [m/s]
    v_escape: float = 0.50           # escape speed in COLLISION_AVOIDANCE [m/s]
    a_brake: float = 0.50            # closing-speed reduction per drone [m/s^2] (calibrated: 0.51)
    w_rel_max: float = 0.15          # unrejected differential drift between two drones [m/s]
    v_open: float = 0.20             # guaranteed opening speed of the SW safety filter [m/s]
    v_filter_cap: float = 0.60       # speed cap of the SW safety-filter solution [m/s]
    escape_vertical_weight: float = 0.35  # vertical component of the CA escape (3D escape)
    struct_keepout: float = 0.70     # never close on structure returns nearer than this [m] (sensor origin)
    struct_push: float = 0.45        # actively open away from structure returns nearer than this [m]
    w_drift_max: float = 0.15        # unrejected absolute drift of one drone [m/s]
    current_drift_max: float = 0.6   # largest effective current drift we claim to handle [m/s]
    z_min: float = -8.0              # operating depth band (world z, up positive)
    z_max: float = -1.5

    @property
    def c_max(self) -> float:
        """Worst-case closing speed of a pair: one escaping, one cruising, plus differential drift."""
        return self.v_escape + self.v_max_nominal + self.w_rel_max


@dataclass(frozen=True)
class GateRule:
    """P2 critical-region protocol parameters, gate frame (s along axis, l lateral, z up).

    The critical region (CR) is s in [-cr_half_len, +cr_half_len], |l|<=cr_half_width.
    Drones that are not committed hold before ``s_queue``.  A drone may commit only
    when it is at the queue line, sees nobody in the occupied zone and has robust
    priority over every perceived queued drone.
    """

    cr_half_len: float = 1.5
    cr_half_width: float = 1.5
    cr_half_height: float = 1.5
    s_queue: float = -4.0            # queue line (gate-frame along coordinate)
    commit_window: float = 0.6       # may commit only if s >= s_queue - commit_window
    occ_gamma: float = 0.6           # occupied zone starts at s_queue + occ_gamma
    occ_exit_margin: float = 1.0     # occupied zone ends at cr_half_len + occ_exit_margin
    corridor_half_width: float = 4.5 # lateral extent of approach/occupied corridor
    approach_len: float = 4.0        # approach zone starts at s_queue - approach_len (kept clear of the
                                     # arena's previous gate G05, which sits 9 m before G06 on its axis)
    # robust lexicographic priority (along, then lateral, then vertical) with gap bands;
    # margins derived in formal/check_mutex.py from eps_rel and the hold tolerances below
    mu_s_hi: float = 1.5             # > mu_s_lo + 2 eps + mono_tol; > commit_window + hold_tol_s + mono_tol + eps
    mu_s_lo: float = 0.7             # > 2 eps (robust sign of the along gap)
    mu_l_hi: float = 2.3             # > mu_l_lo + 2 eps + 4 hold_tol_lat
    mu_l_lo: float = 0.35            # > eps
    mu_z: float = 0.6                # > eps
    # assumptions on position holding (validated empirically by scripts/validate_assumptions.py)
    hold_tol_lat: float = 0.3        # |lateral - setpoint| while queued or leaving the queue line
    hold_tol_z: float = 0.15         # |depth - setpoint| idem
    hold_tol_s: float = 0.3          # queued drones stay at s <= s_queue + hold_tol_s
    mono_tol: float = 0.15           # committed drone's along progress >= queued drone's - mono_tol
    queue_lateral: float = 2.6       # queue points of side slots are at |l| = queue_lateral (> d_warning)
    queue_center_back: float = 2.3   # centre-slot drone holds this far behind s_queue while others queue
                                     # (> mu_s_hi + 2 eps: it then waits robustly, no needless backoff)
    v_approach: float = 0.22         # along-axis speed cap while approaching/yielding
    v_pass: float = 0.30             # along-axis speed while passing (>= v_approach)
    rally_after_exit: float = 4.0    # passed drones advance at least this far beyond the CR
    tie_backoff_s: float = 3.0       # last-resort ID-scaled backoff period


@dataclass(frozen=True)
class FormationRule:
    """P3 formation parameters."""

    e_lost: float = 1.1              # formation error above which formation is lost [m]
    e_ok: float = 0.55               # formation error below which it is recovered [m]
                                     # (>= the formally guaranteed steady tolerance, formal/check_formation.py)
    t_ok_hold: float = 2.0           # e < e_ok must hold this long to declare recovery [s]
    t_recovery_max: float = 60.0     # P3 deadline T used by the referee [s] (>= formal worst case)
    k_along: float = 0.18            # along-track consensus gain [1/s]
    k_lat_lane: float = 0.5          # lane-keeping gain [1/s]
    k_lat_rel: float = 0.25          # relative lateral consensus gain [1/s]
    k_depth: float = 0.6             # depth-keeping gain [1/s]
    v_nominal: float = 0.30          # survey speed [m/s]
    v_lat_max: float = 0.35
    v_z_max: float = 0.35
    neighbor_lost_s: float = 4.0     # missing expected neighbour this long => formation lost


@dataclass(frozen=True)
class PerceptionConfig:
    ring_elevations_deg: Tuple[int, ...] = tuple(range(-60, 61, 5))
    ring_beams: int = 72                 # 5 deg azimuth resolution
    ring_range_m: float = 10.0
    ring_hz: int = 10
    range_noise_std: float = 0.04
    beam_dropout: float = 0.0            # per-beam dropout probability (stress: >0)
    cluster_link_m: float = 0.7
    hull_radius_correction_m: float = 0.25
    track_gate_m: float = 1.0
    track_drop_s: float = 2.0
    structure_mask_m: float = 0.45       # returns closer than this to a mapped gate bar are structure
    fls_threshold: float = 0.15          # FLS speckle max ~0.09, vehicle echoes ~0.4 (measured)
    fls_min_blob_px: int = 6
    fls_hz: int = 5
    camera_hz: int = 5


@dataclass(frozen=True)
class PlantCalibration:
    """Measured in probe/probe_motion.py and probe/probe_current.py (see DISCOVERY.md)."""

    surge_speed_per_cmd: float = 2.4     # ~0.77 m/s @0.3, 2.15 m/s @1.0 (feed-forward slope)
    sway_speed_per_cmd: float = 2.4
    heave_speed_per_cmd: float = 3.4     # ~1.04 m/s @0.3
    thruster_limit: float = 12.0
    # commanded set_ocean_currents magnitude -> steady drift of an unactuated BlueROV2
    current_cmd_to_drift: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.3, 0.012), (1.0, 0.117), (2.0, 0.391), (4.0, 1.191))


@dataclass(frozen=True)
class FleetConfig:
    sep: SeparationThresholds = field(default_factory=SeparationThresholds)
    env: Envelope = field(default_factory=Envelope)
    gate: GateRule = field(default_factory=GateRule)
    form: FormationRule = field(default_factory=FormationRule)
    perc: PerceptionConfig = field(default_factory=PerceptionConfig)
    plant: PlantCalibration = field(default_factory=PlantCalibration)
    comms_enabled: bool = False          # inter-agent communication is OFF by default

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT = FleetConfig()
