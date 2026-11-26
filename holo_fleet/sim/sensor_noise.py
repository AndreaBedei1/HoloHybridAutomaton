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

Levels.  LOW (good conditions) and MEDIUM (typical operational conditions) are realistic nominal values derived from the
sensor datasheets and the literature (Ping2-class echo sounder, A50-class DVL, MMC5983MA / ICM-20602 compass and IMU,
MS5837 depth; config/parameter_sources.json, with references.bib): small white noise, per-run scale factors (DVL scale,
residual sound speed of a temperature-compensated sonar) and the heading bias of a calibrated compass, with the
whole-profile sonar shift truncated at one 0.05 m bin (the sample grid of the sounder bounds the leading-edge phase).  They
stay inside the deterministic envelopes of the proofs (config.Envelope eps_range_far 0.12 m / eps_range_near 0.08 m,
tau_max 0.25 s, NavBound) up to small, quantified per-sample exceedances (Gaussian tails, single missed echoes) listed in
the registry.  HIGH is a stress level, not a hardware characterisation: it exceeds the envelopes on purpose.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from typing import Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class NoiseProfile:
    name: str = "CLEAN"
    sonar_add_sigma: float = 0.05
    sonar_mult_sigma: float = 0.10
    sonar_range_sigma: float = 0.05
    dvl_vel_sigma: float = 0.01
    dvl_range_sigma: float = 0.02
    imu_accel_sigma: float = 0.01
    imu_gyro_sigma: float = 0.005
    imu_accel_bias_sigma: float = 0.0
    imu_gyro_bias_sigma: float = 0.0
    mag_sigma: float = 0.005
    heading_bias_deg: float = 0.0
    depth_sigma: float = 0.02
    sonar_dropout: float = 0.0
    sonar_blackout_every_s: float = 0.0
    sonar_blackout_len_s: float = 0.0
    sonar_range_jitter_m: float = 0.0
    sonar_fade_prob: float = 0.0
    sonar_fade_gain: float = 0.2
    dvl_dropout: float = 0.0
    dvl_outage_every_s: float = 0.0
    dvl_outage_len_s: float = 0.0
    latency_s: float = 0.0
    sonar_jitter_max_bins: int = 0
    """Truncation of the whole-profile shift at this many 0.05 m bins (the sample grid of the sounder bounds the
    leading-edge phase); a large value is untruncated (HIGH), 0 with no jitter at all."""
    sonar_scale_max: float = 0.0
    """Residual sound-speed range scale of each drone's sonars, uniform in [-max, +max], constant per run: a sounder whose
    sound speed is set from a temperature sensor and the known salinity (|scale| <= 0.13 % keeps eps_range valid to 12 m)."""
    dvl_scale_max: float = 0.0
    """DVL velocity scale factor of each drone, uniform in [-max, +max], constant per run (A50-class long-term accuracy;
    NavBound's k_lin carries 1 % of it)."""

    @property
    def wrapper_active(self) -> bool:
        return any((self.sonar_dropout, self.sonar_blackout_every_s, self.sonar_range_jitter_m, self.sonar_fade_prob,
                    self.dvl_dropout, self.dvl_outage_every_s, self.latency_s, self.sonar_scale_max, self.dvl_scale_max))

    def as_dict(self) -> Dict:
        return asdict(self)


CLEAN = NoiseProfile()

LOW = NoiseProfile(
    name="LOW",
    sonar_add_sigma=0.05, sonar_mult_sigma=0.10, sonar_range_sigma=0.05,
    dvl_vel_sigma=0.01, dvl_range_sigma=0.02,
    imu_accel_sigma=0.01, imu_gyro_sigma=0.005, imu_accel_bias_sigma=3e-6, imu_gyro_bias_sigma=2e-7,
    mag_sigma=0.005, heading_bias_deg=1.0, depth_sigma=0.02,
    sonar_dropout=0.001, sonar_range_jitter_m=0.01, sonar_fade_prob=0.005,
    dvl_dropout=0.005, latency_s=0.02, sonar_jitter_max_bins=1, sonar_scale_max=0.001, dvl_scale_max=0.005)

MEDIUM = NoiseProfile(
    name="MEDIUM",
    sonar_add_sigma=0.06, sonar_mult_sigma=0.15, sonar_range_sigma=0.05,
    dvl_vel_sigma=0.0125, dvl_range_sigma=0.03,
    imu_accel_sigma=0.015, imu_gyro_sigma=0.0075, imu_accel_bias_sigma=3e-5, imu_gyro_bias_sigma=7e-7,
    mag_sigma=0.005, heading_bias_deg=2.0, depth_sigma=0.03,
    sonar_dropout=0.005, sonar_range_jitter_m=0.015, sonar_fade_prob=0.01,
    dvl_dropout=0.01, dvl_outage_every_s=60.0, dvl_outage_len_s=1.0, latency_s=0.05,
    sonar_jitter_max_bins=1, sonar_scale_max=0.001, dvl_scale_max=0.01)

# HIGH: stress (not a hardware characterisation).  latency 0.10 s = 3 ticks of 30 Hz: 0.12 s was quantised to 4 ticks and
# left the DVL stale at every step in 2 of 3 tick phases (a test of navigation without DVL rather than of late data).
HIGH = NoiseProfile(
    name="HIGH",
    sonar_add_sigma=0.09, sonar_mult_sigma=0.40, sonar_range_sigma=0.20,
    dvl_vel_sigma=0.08, dvl_range_sigma=0.40,
    imu_accel_sigma=0.10, imu_gyro_sigma=0.05, imu_accel_bias_sigma=0.005, imu_gyro_bias_sigma=0.002,
    mag_sigma=0.07, heading_bias_deg=4.0, depth_sigma=0.15,
    sonar_dropout=0.12, sonar_blackout_every_s=20.0, sonar_blackout_len_s=0.5, sonar_range_jitter_m=0.07,
    sonar_fade_prob=0.10, dvl_dropout=0.08, dvl_outage_every_s=30.0, dvl_outage_len_s=1.0, latency_s=0.10,
    sonar_jitter_max_bins=99, sonar_scale_max=0.005, dvl_scale_max=0.02)

MEDIUM_HB3 = replace(MEDIUM, name="MEDIUM_HB3", heading_bias_deg=3.0)
MEDIUM_HB4 = replace(MEDIUM, name="MEDIUM_HB4", heading_bias_deg=4.0)

PROFILES: Dict[str, NoiseProfile] = {p.name: p for p in (CLEAN, LOW, MEDIUM, HIGH, MEDIUM_HB3, MEDIUM_HB4)}
VALIDATION_LEVELS = ("CLEAN", "LOW", "MEDIUM")
SCOPES = ("ONE", "SUBSET", "ALL")

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
    "scale": ("sonar_scale_max", "dvl_scale_max"),
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


def scale_factors(p: NoiseProfile, n: int, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    """Per-drone (sonar range scale, DVL velocity scale), uniform in +-max, constant per run, drawn from the seed."""
    rng = np.random.default_rng([int(seed), 7927])
    sonar = rng.uniform(-p.sonar_scale_max, p.sonar_scale_max, size=n) if p.sonar_scale_max > 0 else np.zeros(n)
    dvl = rng.uniform(-p.dvl_scale_max, p.dvl_scale_max, size=n) if p.dvl_scale_max > 0 else np.zeros(n)
    return sonar, dvl


def plan_windows(rng: np.random.Generator, every_s: float, len_s: float, duration_s: float, t0: float = 4.0):
    """Burst windows: first start uniform in [t0, every_s], then exponential gaps (mean every_s)."""
    wins = []
    if every_s > 0 and len_s > 0:
        t = float(rng.uniform(t0, max(t0, every_s)))
        while t < duration_s:
            wins.append((t, t + len_s))
            t += float(rng.exponential(every_s)) + len_s
    return wins
