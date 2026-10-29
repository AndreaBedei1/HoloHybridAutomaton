"""Reproducible sensor-degradation profiles: a realistic operating domain (LOW, MEDIUM) and exploratory stress.

Every disturbance acts on the SENSOR, before a controller receives the data; ground truth (PoseSensor,
VelocitySensor, CollisionSensor) stays clean and goes to the referee only.

Two mechanisms:

* HoloOcean-native noise (engine side, set in the sensor ``configuration``): SinglebeamSonar AddSigma (Rayleigh
  intensity noise, its scale: calibration/probe_sensor_noise.py), MultSigma, RangeSigma (exponential per octree leaf);
  DVL VelSigma (per beam), RangeSigma; IMU AccelSigma, AngVelSigma, AccelBiasSigma, AngVelBiasSigma (random walk);
  Magnetometer Sigma and MagneticVector (a rotated reference vector = a constant heading bias, hard-iron like);
  DepthSensor Sigma.  The engine draws these with its own unseeded generator: runs are not bit-reproducible.
* wrapper stress (holo_env.HoloFleetSim, seeded from the scenario seed): what HoloOcean does not model, or not in
  the way we need: sonar capture dropout, burst blackouts, whole-profile range jitter (timing / sound-speed
  fluctuation; HoloOcean's RangeSigma only lengthens echoes, it barely moves their leading edge), deep fades
  (missed detections: the capture arrives, the target echo does not), DVL dropout and outages (bottom-lock loss),
  latency of every onboard stream (data delivered late with its true capture time stamp).

The four levels are anchored to the project's declared assumptions (config.Envelope: sonar 10 Hz, tau_max 0.25 s,
eps_range_far 0.12 m / eps_range_near 0.08 m; FormationRule e_ok 0.5 / e_lost 1.2 m; GateRule queue_tol 0.35 m):
LOW and MEDIUM are the realistic operating domain (datasheet / literature anchored, see below); HIGH is a stress level
beyond it (out-of-envelope checks).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from typing import Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class NoiseProfile:
    name: str = "CLEAN"
    # --- HoloOcean-native (engine) -------------------------------------------------------------- deployed values
    sonar_add_sigma: float = 0.05        # Rayleigh scale of the additive intensity noise (threshold 0.30)
    sonar_mult_sigma: float = 0.10
    sonar_range_sigma: float = 0.05      # exponential, per octree leaf [m]
    dvl_vel_sigma: float = 0.01          # per beam [m/s]; body x/y std ~ 1.9x (probe)
    dvl_range_sigma: float = 0.02        # per beam [m]
    imu_accel_sigma: float = 0.01
    imu_gyro_sigma: float = 0.005        # [rad/s]
    imu_accel_bias_sigma: float = 0.0    # random-walk step [m/s^2]
    imu_gyro_bias_sigma: float = 0.0     # random-walk step [rad/s]
    mag_sigma: float = 0.005             # per axis of a unit field: heading std ~ 57 x sigma deg (probe)
    heading_bias_deg: float = 0.0        # constant per-drone compass bias, sign drawn from the seed
    depth_sigma: float = 0.02            # [m]
    # --- wrapper (seeded) ------------------------------------------------------------------------------------
    sonar_dropout: float = 0.0           # per capture, per sonar
    sonar_blackout_every_s: float = 0.0  # mean period of whole-drone sonar blackouts (0: none)
    sonar_blackout_len_s: float = 0.0
    sonar_range_jitter_m: float = 0.0    # per capture whole-profile shift, normal, quantised to 5 cm bins
    sonar_fade_prob: float = 0.0         # per capture: the profile is attenuated (missed detection)
    sonar_fade_gain: float = 0.2         # peak echo ~0.9-1.2 (probe) x 0.2 < threshold 0.30
    dvl_dropout: float = 0.0             # per DVL sample
    dvl_outage_every_s: float = 0.0      # mean period of DVL outages (bottom-lock loss)
    dvl_outage_len_s: float = 0.0
    latency_s: float = 0.0               # every onboard stream delivered this late (stamped with its capture time)

    @property
    def wrapper_active(self) -> bool:
        return any((self.sonar_dropout, self.sonar_blackout_every_s, self.sonar_range_jitter_m, self.sonar_fade_prob,
                    self.dvl_dropout, self.dvl_outage_every_s, self.latency_s))

    def as_dict(self) -> Dict:
        return asdict(self)


CLEAN = NoiseProfile()

# Realistic operating domain (the validation campaign).  Every value is tied to a datasheet or a paper in
# docs/SENSOR_ROBUSTNESS_LITERATURE.md (table in docs/SENSOR_ROBUSTNESS_AND_BEACONING.md, section 1).  CLEAN (the
# deployed suite) is already at or above the datasheet white noise of the IMU (ICM-20602, A6), the depth sensor
# (Bar30, A10) and the DVL (Nortek single ping ~0.8 cm/s, A2), so those channels stay at CLEAN in LOW and grow
# only moderately in MEDIUM; the degradations that matter are heading (calibration, A8/A9), DVL bottom-lock loss
# (A1/A3/A4), sonar range resolution and spurious / missed echoes (A11/A12/A16).  Sonar dropout and blackouts,
# DVL outages and latency have no measured statistics for this hardware: they are conservative operational
# perturbations, not measured sensor accuracy.
LOW = NoiseProfile(
    name="LOW",
    sonar_add_sigma=0.06, sonar_mult_sigma=0.15, sonar_range_sigma=0.05,
    dvl_vel_sigma=0.01, dvl_range_sigma=0.02,
    imu_accel_sigma=0.01, imu_gyro_sigma=0.005, imu_accel_bias_sigma=0.0002, imu_gyro_bias_sigma=0.0001,
    mag_sigma=0.01, heading_bias_deg=0.5, depth_sigma=0.02,
    sonar_dropout=0.005, sonar_range_jitter_m=0.015, sonar_fade_prob=0.01,
    dvl_dropout=0.01, latency_s=0.02)

MEDIUM = NoiseProfile(
    name="MEDIUM",
    sonar_add_sigma=0.075, sonar_mult_sigma=0.25, sonar_range_sigma=0.10,
    dvl_vel_sigma=0.015, dvl_range_sigma=0.05,
    imu_accel_sigma=0.02, imu_gyro_sigma=0.01, imu_accel_bias_sigma=0.0005, imu_gyro_bias_sigma=0.0003,
    mag_sigma=0.02, heading_bias_deg=2.0, depth_sigma=0.03,
    sonar_dropout=0.01, sonar_blackout_every_s=60.0, sonar_blackout_len_s=0.15, sonar_range_jitter_m=0.03,
    sonar_fade_prob=0.03, dvl_dropout=0.03, dvl_outage_every_s=60.0, dvl_outage_len_s=1.0, latency_s=0.05)

# Stress level beyond the declared operating domain (out-of-envelope checks only, not a validation level).
HIGH = NoiseProfile(
    name="HIGH",
    sonar_add_sigma=0.09, sonar_mult_sigma=0.40, sonar_range_sigma=0.20,
    dvl_vel_sigma=0.08, dvl_range_sigma=0.40,
    imu_accel_sigma=0.10, imu_gyro_sigma=0.05, imu_accel_bias_sigma=0.005, imu_gyro_bias_sigma=0.002,
    mag_sigma=0.07, heading_bias_deg=4.0, depth_sigma=0.15,
    sonar_dropout=0.12, sonar_blackout_every_s=20.0, sonar_blackout_len_s=0.5, sonar_range_jitter_m=0.07,
    sonar_fade_prob=0.10, dvl_dropout=0.08, dvl_outage_every_s=30.0, dvl_outage_len_s=1.0, latency_s=0.12)

PROFILES: Dict[str, NoiseProfile] = {p.name: p for p in (CLEAN, LOW, MEDIUM, HIGH)}
VALIDATION_LEVELS = ("CLEAN", "LOW", "MEDIUM")          # the realistic operating domain
SCOPES = ("ONE", "SUBSET", "ALL")                         # which drones receive the degraded profile

# Isolation: CLEAN plus ONE phenomenon at its HIGH level (the combined case is the HIGH profile itself).
FACTORS: Dict[str, Tuple[str, ...]] = {
    "sonar_intensity": ("sonar_add_sigma", "sonar_mult_sigma", "sonar_range_sigma"),
    "sonar_range": ("sonar_range_jitter_m",),
    "sonar_fading": ("sonar_fade_prob",),
    "sonar_dropout": ("sonar_dropout",),
    "sonar_blackout": ("sonar_blackout_every_s", "sonar_blackout_len_s"),
    "dvl_velocity": ("dvl_vel_sigma",),
    "dvl_dropout": ("dvl_dropout", "dvl_outage_every_s", "dvl_outage_len_s"),
    "dvl_range": ("dvl_range_sigma",),
    "imu_white": ("imu_accel_sigma", "imu_gyro_sigma"),
    "imu_bias": ("imu_accel_bias_sigma", "imu_gyro_bias_sigma"),
    "heading": ("mag_sigma", "heading_bias_deg"),
    "depth": ("depth_sigma",),
    "latency": ("latency_s",),
}


def isolated(factor: str, level: NoiseProfile = HIGH) -> NoiseProfile:
    """CLEAN with only the fields of ``factor`` taken from ``level``."""
    return replace(CLEAN, name=f"ISO_{factor}", **{f: getattr(level, f) for f in FACTORS[factor]})


def profile(name: str) -> NoiseProfile:
    if name in PROFILES:
        return PROFILES[name]
    if name.startswith("ISO_") and name[4:] in FACTORS:
        return isolated(name[4:])
    raise KeyError(f"unknown noise profile {name!r}")


def heading_biases(p: NoiseProfile, n: int, seed: int) -> np.ndarray:
    """Per-drone constant compass bias [deg]: magnitude heading_bias_deg, sign drawn from the seed."""
    if p.heading_bias_deg == 0.0:
        return np.zeros(n)
    rng = np.random.default_rng([int(seed), 101])
    return p.heading_bias_deg * rng.choice([-1.0, 1.0], size=n)


def plan_windows(rng: np.random.Generator, every_s: float, len_s: float, duration_s: float, t0: float = 4.0):
    """Burst windows: first start uniform in [t0, every_s], then exponential gaps (mean every_s)."""
    wins = []
    if every_s > 0 and len_s > 0:
        t = float(rng.uniform(t0, max(t0, every_s)))
        while t < duration_s:
            wins.append((t, t + len_s))
            t += float(rng.exponential(every_s)) + len_s
    return wins
