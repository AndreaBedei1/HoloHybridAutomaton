"""Measurement model of the one-way acoustic range-bearing beacon (variant B2), simulator side.

What the HoloOcean 2.3 engine computes (engine/Source/Holodeck/Sensors/Private/AcousticBeaconSensor.cpp, read
in the patched engine source, and results/calibration/beacon_api_probe.json): for a (sender, receiver)
command, the sender's sensor position in the RECEIVER's sensor frame (``InverseTransformLocation`` of the receiver
component transform: body frame, roll and pitch included, client axes x forward, y left, z up) as
[azimuth, elevation, distance, receiver depth]; exact values, no noise; -1 in every field when the distance exceeds
``MaxDistance`` or the straight segment between the two sensors first hits another actor (``CheckVisible``: a hull,
a prop such as a gate bar); the result is written after round(d / (dt * 1500)) ticks (0 or 1 tick below 50 m).
The fourth field is the RECEIVER's own depth, not the sender's: nothing of the sender's state is carried.

Those engine values are treated as the geometric truth of the direct path, never handed to a controller as they
are.  This module turns them into what a real receiver would deliver, under explicit hardware assumptions
(docs/B2_ACOUSTIC_AIDED_COORDINATION.md, section 2):

* H1 static identity, fixed schedule: drone k emits a coded ping (its identity, no payload) at the pre-assigned
  instants t = m * period + k * period / n (TDMA: the schedule is mission-plan knowledge, like the identity);
* H2 synchronised clocks (one-way travel-time ranging, OWTT, e.g. a chip-scale atomic clock disciplined before
  the dive, with a PPS-triggered coded ping): range = c_assumed * (t_arrival - t_scheduled), ``c_assumed`` computed
  ONBOARD from the measured temperature and depth and a known salinity (Mackenzie / UNESCO): the engine's 1500 m/s
  is not the onboard value (real water spans ~1426-1540 m/s; assuming 1500 m/s would err by 1-5 %).  Errors: a
  per-drone clock offset (constant over a run, a range bias between two drones), the residual sound-speed error of
  that estimate (a range scale factor, one per run: the water mass is shared), detection jitter (white);
* H3 a small, calibrated 3-D receiving array (inverted USBL) gives the direction of arrival in the receiver's body
  frame: white azimuth / elevation noise (array SNR) plus the residual of the array calibration, a smooth bias that
  depends on the direction of arrival and is fixed per receiver (``doa_bias_deg``; the dominant error of field
  inverted-USBL systems); the receiver's own attitude error is applied onboard, not here;
* H4 the receiver detects the FIRST arrival.  The engine's line of sight is a single ray, blocked by anything it
  touches.  At the 20-30 kHz of micro-USBL modems (wavelength 5-7.5 cm) the first Fresnel zone at mid-path has a
  radius of 0.19-0.24 m at 3 m and 0.35-0.43 m at 10 m, comparable to the cross-section of a BlueROV2 hull or of a
  gate bar: such a body SHADOWS the direct path (diffraction), it does not cut it.  So when the ray is
  blocked and the simulator finds a drone hull or a gate bar on it (``shadowed``), the ping is lost with
  probability ``shadow_loss``, otherwise the diffracted direct arrival is detected, ``shadow_extra_m`` longer at
  most and with ``shadow_dir_deg`` more direction noise.  Blocked by anything else (a large prop, the terrain):
  usually lost; with probability ``nlos_rx`` a reflected path (surface or bottom bounce, a structure) is detected
  instead, longer by the extra path length and from the direction of the reflection: a contradictory measurement.
  A clear direct path does not exclude outliers either (multipath false maxima, cancellation with a boundary
  reflection, field reports): with probability ``los_outlier`` per reception a persistent multipath episode starts
  on that link and lasts ``los_outlier_pings`` receptions on average; during it the time of arrival is that of a
  boundary bounce, longer by a fixed excess (``reflect_extra_m``, one-sided), while the array still resolves the
  direct path: slowly varying ranges that mimic valid data, the case the onboard envelope must cover
  (config.AcousticConfig.los_excess_max_m);
* H5 processing latency (detection, beamforming): the measurement is delivered ``latency_s`` after the arrival,
  stamped with its arrival time;
* channel: per-reception dropout growing with range (absorption, SNR) and burst outages (interference: thruster or
  vessel noise).  Two arrivals never collide at a receiver: the TDMA slots of H1 are far wider than the propagation
  spread of the ranges here.

Levels (docs/ACOUSTIC_CHANNEL_MODEL.md): CLEAN, LOW, MEDIUM and BLACKOUT (the channel delivers nothing).  Anchors:
range resolution +-0.1 m of a commercial micro-USBL modem (Blueprint SeaTrac X150 datasheet, "dependant on provided
VOS accuracy"); bearing 1 deg RMS (Tritech MicronNav 200 datasheet) to +-3 deg (older micro-USBL); a low-cost
one-way inverted-USBL beacon (Rypkema, Schmidt, Fischell, Field Robotics 2022) reported, in a static test at about
30 and 57 m in a river, 68 % of the azimuth errors below 7 deg, with a direction-dependent bias.  Our ranges are
2-20 m in open water: CLEAN / LOW sit near the datasheet values, MEDIUM between them and the field figures.  The
calibration bias and the clear-path multipath episodes were added after the provenance audit of the model (stress
values: no source gives their probability), before the final campaign; they only make the channel harsher.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class AcousticChannel:
    name: str = "CLEAN"
    blackout: bool = False               # nothing is ever received
    sigma_range_m: float = 0.05          # detection jitter (white, per reception)
    sound_speed_err: float = 0.003       # std of the relative sound-speed error (one scale factor per run)
    clock_offset_m: float = 0.01         # std of a drone's clock offset, as a range [m] (0.01 m = 7 us)
    sigma_az_deg: float = 1.0            # direction of arrival, white
    sigma_el_deg: float = 1.0
    dropout: float = 0.02                # per reception, at short range
    dropout_far: float = 0.10            # ... growing linearly to this at max_range_m
    outage_every_s: float = 0.0          # mean period of burst outages (0: none)
    outage_len_s: float = 0.0
    nlos_rx: float = 0.10                # blocked direct path: probability that a reflected path is detected
    reflect_extra_m: Tuple[float, float] = (0.3, 3.0)    # its extra path length (uniform)
    reflect_dir_deg: float = 25.0        # its direction error (normal, per axis)
    shadow_loss: float = 0.30            # direct path shadowed by a hull or a bar: probability that the ping is lost
    shadow_extra_m: float = 0.10         # ... otherwise the diffracted arrival is up to this longer (uniform)
    shadow_dir_deg: float = 0.5          # ... with this much more direction noise (normal, per axis)
    latency_s: float = 0.10              # processing delay (delivery after the arrival)
    doa_bias_deg: float = 0.5            # calibration residual of the array: smooth direction-dependent bias amplitude
    los_outlier: float = 0.0             # clear path: per-reception probability that a multipath episode starts
    los_outlier_pings: float = 5.0       # ... its mean length in receptions (geometric)
    payload_loss: float = 0.02           # H6: the ping is detected but its data packet is not decoded (STRESS)

    def as_dict(self) -> Dict:
        return asdict(self)


CLEAN = AcousticChannel()
LOW = AcousticChannel(name="LOW", sigma_range_m=0.10, sound_speed_err=0.005, clock_offset_m=0.03, sigma_az_deg=1.5,
                      sigma_el_deg=1.5, dropout=0.08, dropout_far=0.25, nlos_rx=0.20, latency_s=0.15,
                      shadow_loss=0.50, shadow_dir_deg=1.0, doa_bias_deg=1.5, los_outlier=0.01,
                      payload_loss=0.05)
MEDIUM = AcousticChannel(name="MEDIUM", sigma_range_m=0.15, sound_speed_err=0.010, clock_offset_m=0.05,
                         sigma_az_deg=2.5, sigma_el_deg=2.5, dropout=0.20, dropout_far=0.50, outage_every_s=40.0,
                         outage_len_s=4.0, nlos_rx=0.35, latency_s=0.25, shadow_loss=0.70, shadow_dir_deg=1.5,
                         doa_bias_deg=3.0, los_outlier=0.03, payload_loss=0.15)
BLACKOUT = AcousticChannel(name="BLACKOUT", blackout=True)
CHANNELS: Dict[str, AcousticChannel] = {c.name: c for c in (CLEAN, LOW, MEDIUM, BLACKOUT)}


def channel(name: str) -> AcousticChannel:
    if name not in CHANNELS:
        raise KeyError(f"unknown acoustic channel level {name!r}")
    return CHANNELS[name]


def unit(az: float, el: float) -> np.ndarray:
    return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])


class ChannelModel:
    """Seeded per-run channel: turns an engine result into a delivered measurement, or a loss."""

    def __init__(self, ch: AcousticChannel, n: int, seed: int, max_range_m: float, duration_s: float = 3600.0):
        self.ch = ch
        self.max_range = float(max_range_m)
        self.rng = np.random.default_rng([int(seed), 5101])
        self.scale = 1.0 + float(self.rng.normal(0.0, ch.sound_speed_err))
        self.offset = self.rng.normal(0.0, ch.clock_offset_m, size=n)          # per drone [m]
        self.outages: List[Tuple[float, float]] = []
        if ch.outage_every_s > 0 and ch.outage_len_s > 0:
            t = float(self.rng.uniform(4.0, max(4.0, ch.outage_every_s)))
            while t < duration_s:
                self.outages.append((t, t + ch.outage_len_s))
                t += float(self.rng.exponential(ch.outage_every_s)) + ch.outage_len_s
        # array calibration residual of every receiver: b(az) = A (sin(az + p1) + sin(2 az + p2)) / 2 per axis
        self.bias_phase = self.rng.uniform(0.0, 2.0 * math.pi, size=(n, 4))
        self.episode: Dict[Tuple[int, int], Tuple[int, float]] = {}      # link -> (receptions left, range excess)
        self.stats = {"lost_blackout": 0, "lost_outage": 0, "lost_dropout": 0, "lost_blocked": 0, "nlos_delivered": 0,
                      "los_outlier_delivered": 0}

    def doa_bias(self, receiver: int, az: float) -> Tuple[float, float]:
        """Calibration bias of the receiver's array (azimuth, elevation) [rad] for an arrival from azimuth ``az``."""
        A = math.radians(self.ch.doa_bias_deg)
        p = self.bias_phase[receiver]
        return (A * 0.5 * (math.sin(az + p[0]) + math.sin(2.0 * az + p[1])),
                A * 0.5 * (math.sin(az + p[2]) + math.sin(2.0 * az + p[3])))

    def _los_episode(self, sender: int, receiver: int) -> Optional[float]:
        """Clear-path multipath episode on the link (H4): its range excess while one lasts, else None."""
        ch, rng = self.ch, self.rng
        key = (sender, receiver)
        left, extra = self.episode.get(key, (0, 0.0))
        if left <= 0:
            if ch.los_outlier <= 0.0 or rng.random() >= ch.los_outlier:
                return None
            left = int(rng.geometric(1.0 / max(ch.los_outlier_pings, 1.0)))
            extra = float(rng.uniform(*ch.reflect_extra_m))
        self.episode[key] = (left - 1, extra)
        return extra

    def in_outage(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.outages)

    def measure(self, t: float, sender: int, receiver: int, buf: np.ndarray) -> Optional[Tuple[float, float, float, bool]]:
        """Engine buffer [az, el, d, depth] (or -1s: direct path blocked / out of range) -> (range, az, el, nlos)."""
        ch, rng = self.ch, self.rng
        if ch.blackout:
            self.stats["lost_blackout"] += 1
            return None
        if self.in_outage(t):
            self.stats["lost_outage"] += 1
            return None
        blocked = bool(np.all(buf == -1.0))
        if blocked:
            # the engine gives no geometry for a blocked path; a reflected arrival needs the direct geometry, which
            # the transport passes as the last clear one (see BeaconTransport) - here only whether one is detected
            self.stats["lost_blocked"] += 1
            return None
        az, el, d = float(buf[0]), float(buf[1]), float(buf[2])
        p_drop = ch.dropout + (ch.dropout_far - ch.dropout) * min(1.0, d / self.max_range)
        if rng.random() < p_drop:
            self.stats["lost_dropout"] += 1
            return None
        ep = self._los_episode(sender, receiver)
        b_az, b_el = self.doa_bias(receiver, az)
        r = (d + (ep or 0.0)) * self.scale + (self.offset[sender] - self.offset[receiver]) \
            + float(rng.normal(0.0, ch.sigma_range_m))
        az_m = az + b_az + math.radians(float(rng.normal(0.0, ch.sigma_az_deg)))
        el_m = el + b_el + math.radians(float(rng.normal(0.0, ch.sigma_el_deg)))
        if ep is not None:
            self.stats["los_outlier_delivered"] += 1
        return max(r, 0.0), az_m, el_m, ep is not None

    def shadowed(self, t: float, sender: int, receiver: int, geom: Tuple[float, float, float]):
        """Direct path shadowed by a drone hull or a gate bar (H4): lost, or the diffracted direct arrival."""
        ch, rng = self.ch, self.rng
        if ch.blackout or self.in_outage(t) or rng.random() < ch.shadow_loss:
            return None
        az, el, d = geom
        p_drop = ch.dropout + (ch.dropout_far - ch.dropout) * min(1.0, d / self.max_range)
        if rng.random() < p_drop:
            return None
        r = (d + float(rng.uniform(0.0, ch.shadow_extra_m))) * self.scale + (self.offset[sender] - self.offset[receiver]) \
            + float(rng.normal(0.0, ch.sigma_range_m))
        sd_az = math.hypot(ch.sigma_az_deg, ch.shadow_dir_deg)
        sd_el = math.hypot(ch.sigma_el_deg, ch.shadow_dir_deg)
        b_az, b_el = self.doa_bias(receiver, az)
        az_m = az + b_az + math.radians(float(rng.normal(0.0, sd_az)))
        el_m = el + b_el + math.radians(float(rng.normal(0.0, sd_el)))
        self.stats["lost_blocked"] -= 1
        self.stats["shadow_delivered"] = self.stats.get("shadow_delivered", 0) + 1
        return max(r, 0.0), az_m, el_m, False

    def reflected(self, t: float, sender: int, receiver: int, geom: Tuple[float, float, float]):
        """Blocked direct path with a known (truth) geometry: a reflected arrival, or None (lost)."""
        ch, rng = self.ch, self.rng
        if ch.blackout or self.in_outage(t) or rng.random() >= ch.nlos_rx:
            return None
        az, el, d = geom
        lo, hi = ch.reflect_extra_m
        r = (d + float(rng.uniform(lo, hi))) * self.scale + (self.offset[sender] - self.offset[receiver]) \
            + float(rng.normal(0.0, ch.sigma_range_m))
        az_m = az + math.radians(float(rng.normal(0.0, 0.2 * ch.reflect_dir_deg)))     # boundary bounces err mainly
        el_m = el + math.radians(float(rng.normal(0.0, ch.reflect_dir_deg)))           # in elevation
        self.stats["lost_blocked"] -= 1
        self.stats["nlos_delivered"] += 1
        return r, az_m, el_m, True


def segment_hits_box(a: np.ndarray, b: np.ndarray, center: np.ndarray, axes: np.ndarray, half: np.ndarray) -> bool:
    """Does the segment a-b touch the oriented box (centre, axes as columns, half extents)?  (slab test)"""
    A, B = axes.T @ (np.asarray(a, float) - center), axes.T @ (np.asarray(b, float) - center)
    d, t0, t1 = B - A, 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(A[k]) > half[k]:
                return False
            continue
        u0, u1 = (-half[k] - A[k]) / d[k], (half[k] - A[k]) / d[k]
        if u0 > u1:
            u0, u1 = u1, u0
        t0, t1 = max(t0, u0), min(t1, u1)
        if t0 > t1:
            return False
    return True
