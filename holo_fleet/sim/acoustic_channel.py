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
  the dive): range = c_assumed * (t_arrival - t_scheduled).  Errors: a per-drone clock offset (constant over a run,
  a range bias between two drones), the sound-speed error (a range scale factor, one per run: the water mass is
  shared), detection jitter (white);
* H3 a small receiving array (inverted USBL) gives the direction of arrival in the receiver's body frame: white
  azimuth / elevation noise (array SNR; the receiver's own attitude error is applied onboard, not here);
* H4 the receiver detects the FIRST arrival.  With a clear direct path the first arrival is the direct one (no
  range outlier).  The engine's line of sight is a single ray, blocked by anything it touches.  At the 20-30 kHz
  of micro-USBL modems (wavelength ~6 cm) the first Fresnel zone of a 3-10 m path has a radius of 0.3-0.5 m, larger
  than the cross-section of a BlueROV2 hull (0.25 x 0.46 m seen from the side) or of a gate bar (0.25 x 0.30 m):
  such a body SHADOWS the direct path (diffraction loss of a few dB), it does not cut it.  So when the ray is
  blocked and the simulator finds a drone hull or a gate bar on it (``shadowed``), the ping is lost with
  probability ``shadow_loss``, otherwise the diffracted direct arrival is detected, ``shadow_extra_m`` longer at
  most and with ``shadow_dir_deg`` more direction noise.  Blocked by anything else (a large prop, the terrain):
  usually lost; with probability ``nlos_rx`` a reflected path (surface or bottom bounce, a structure) is detected
  instead, longer by the extra path length and from the direction of the reflection: a contradictory measurement;
* H5 processing latency (detection, beamforming): the measurement is delivered ``latency_s`` after the arrival,
  stamped with its arrival time;
* channel: per-reception dropout growing with range (absorption, SNR), burst outages (interference: thruster or
  vessel noise), and the collision of two arrivals within ``guard_s`` at one receiver (both lost; with the TDMA
  slots of H1 it only happens for slots spaced below the propagation spread, never in the scenarios here).

Levels (fixed before any B2 result, docs/B2_ACOUSTIC_AIDED_COORDINATION.md, table 2): CLEAN, LOW, MEDIUM and
BLACKOUT (the channel delivers nothing).  Anchors: range resolution +-0.1 m of a commercial micro-USBL modem
(Blueprint SeaTrac X150 datasheet, "dependant on provided VOS accuracy"); bearing 1 deg RMS (Tritech MicronNav
200 datasheet) to +-3 deg (older micro-USBL); a low-cost OWTT + inverted-USBL fleet at 1 Hz reported 68 % of
range errors < 0.7 m and of azimuth errors < 7 deg at 30-300 m in a river (Rypkema, Schmidt, Fischell, Field
Robotics 2022).  Our ranges are 2-20 m in open water: CLEAN / LOW sit near the datasheet values, MEDIUM between
them and the low-cost field figures.
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
    guard_s: float = 0.010               # two arrivals closer than this at one receiver are both lost

    def as_dict(self) -> Dict:
        return asdict(self)


CLEAN = AcousticChannel()
LOW = AcousticChannel(name="LOW", sigma_range_m=0.10, sound_speed_err=0.005, clock_offset_m=0.03, sigma_az_deg=1.5,
                      sigma_el_deg=1.5, dropout=0.08, dropout_far=0.25, nlos_rx=0.20, latency_s=0.15,
                      shadow_loss=0.50, shadow_dir_deg=1.0)
MEDIUM = AcousticChannel(name="MEDIUM", sigma_range_m=0.15, sound_speed_err=0.010, clock_offset_m=0.05,
                         sigma_az_deg=2.5, sigma_el_deg=2.5, dropout=0.20, dropout_far=0.50, outage_every_s=40.0,
                         outage_len_s=4.0, nlos_rx=0.35, latency_s=0.25, shadow_loss=0.70, shadow_dir_deg=1.5)
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
        self.stats = {"lost_blackout": 0, "lost_outage": 0, "lost_dropout": 0, "lost_blocked": 0, "nlos_delivered": 0}

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
        r = d * self.scale + (self.offset[sender] - self.offset[receiver]) + float(rng.normal(0.0, ch.sigma_range_m))
        az_m = az + math.radians(float(rng.normal(0.0, ch.sigma_az_deg)))
        el_m = el + math.radians(float(rng.normal(0.0, ch.sigma_el_deg)))
        return max(r, 0.0), az_m, el_m, False

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
        az_m = az + math.radians(float(rng.normal(0.0, sd_az)))
        el_m = el + math.radians(float(rng.normal(0.0, sd_el)))
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
        az_m = az + math.radians(float(rng.normal(0.0, ch.reflect_dir_deg)))
        el_m = el + math.radians(float(rng.normal(0.0, ch.reflect_dir_deg)))
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
