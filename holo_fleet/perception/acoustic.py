"""Onboard side of the B2 range-bearing beacon: acoustic fixes of identified neighbours, with error regions.

A reception (sim/beacon.py, delivered as ``AcousticRx``) is (identity, arrival time, range, azimuth, elevation):
the identity is the static pinger code of the plan (which drone holds which slot), range and direction come from
the receiver's own OWTT clock and array (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, H1-H5).  Nothing else of the
sender is known.  This module turns receptions into FIXES:

* the nominal relative position r * u(az, el) in the own body frame at the arrival time, rotated with the own
  heading estimate of that instant (pose history) into the map frame;
* an ERROR REGION from the declared envelope (config.AcousticConfig, never from the channel level of the run):
  range within +-(k sigma_r + c_err r + clock offset), azimuth within +-(k sigma_az + own heading error),
  elevation within +-(k sigma_el + own tilt error).  The region is sampled on a 7 x 7 x 7 grid of that box plus a
  slack covering the space between samples (half the diagonal of a grid cell), so that every point of the box is
  within ``slack`` of a sample;
* a reception delivered later than ``latency_max_s`` after its arrival is not used (stale);
* a fix whose nominal depth lies outside the operating depth band by more than ``depth_tol_m`` is a reflection
  (a surface or bottom bounce arrives from above or below) and is rejected;
* a track per identity: a fix is ACCEPTED when it is kinematically consistent with the previous accepted fix of
  that identity (displacement <= v_bound dt + k sqrt(2) sigma_white + ``track_gate_m``).  An inconsistent fix is
  REJECTED but kept (the gate uses the white part of the errors only: the biases are common to two fixes); it is contradictory evidence, and whatever a rejected fix could mean is never ignored by a
  decision that relies on the identity's absence (gate_perception).  Two mutually consistent rejected fixes, while
  the track has had no accepted fix for ``REINIT_S``, start a new track (a new track id: whatever was established
  on the old one is not carried over).

Absence of fixes is never evidence of anything.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.exit_witness import speed_bound

_N = 7                 # samples per axis of the error box
REINIT_S = 3.0         # a track with no accepted fix for this long may be restarted by consistent rejected fixes
_KEEP_S = 30.0         # fixes older than this are forgotten


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_bounds(r: float, cfg: FleetConfig = DEFAULT) -> Tuple[float, float, float]:
    """Half-widths of the error box (range [m], azimuth [rad], elevation [rad]) of a fix at range r."""
    a = cfg.acoustic
    br = a.k_sigma * a.sigma_range_m + a.sound_speed_err * r + a.clock_offset_m
    ba = math.radians(a.k_sigma * a.sigma_az_deg + a.heading_err_deg)
    be = math.radians(a.k_sigma * a.sigma_el_deg + a.tilt_err_deg)
    return br, ba, be


def region_samples(r: float, az: float, el: float, cfg: FleetConfig = DEFAULT) -> Tuple[np.ndarray, float]:
    """Samples of the error box of a measurement (body frame, relative) and the slack that covers the box."""
    br, ba, be = box_bounds(r, cfg)
    rr = np.linspace(max(r - br, 0.0), r + br, _N)
    aa = np.linspace(az - ba, az + ba, _N)
    ee = np.linspace(el - be, el + be, _N)
    R, A, E = np.meshgrid(rr, aa, ee, indexing="ij")
    pts = np.stack([R * np.cos(E) * np.cos(A), R * np.cos(E) * np.sin(A), R * np.sin(E)], axis=-1).reshape(-1, 3)
    r_max = r + br
    steps = np.array([rr[1] - rr[0], r_max * (aa[1] - aa[0]), r_max * (ee[1] - ee[0])])
    return pts, 0.5 * float(np.linalg.norm(steps))


@dataclass
class Fix:
    slot: int
    t: float                         # arrival time (onboard clock)
    r: float
    az: float
    el: float
    p: np.ndarray                    # nominal position (map frame, own pose at t)
    pts: np.ndarray                  # error-region samples (map frame)
    slack: float
    rel: np.ndarray                  # nominal relative position, map-frame axes (own heading at t)
    accepted: bool = True
    track: int = 0
    reason: str = ""

    @property
    def extent(self) -> float:
        """Radius of a ball around ``p`` containing the whole region."""
        return float(np.max(np.linalg.norm(self.pts - self.p, axis=1))) + self.slack


@dataclass
class PoseHistory:
    keep_s: float = 3.0
    buf: Deque[Tuple[float, np.ndarray, float]] = field(default_factory=deque)

    def push(self, t: float, p: np.ndarray, yaw: float) -> None:
        self.buf.append((t, np.array(p, dtype=float), float(yaw)))
        while self.buf and t - self.buf[0][0] > self.keep_s:
            self.buf.popleft()

    def at(self, t: float) -> Optional[Tuple[np.ndarray, float]]:
        """Own pose estimate at time t (nearest stored instant; None if t is not covered)."""
        if not self.buf or t < self.buf[0][0] - 0.11 or t > self.buf[-1][0] + 0.11:
            return None
        k = min(range(len(self.buf)), key=lambda i: abs(self.buf[i][0] - t))
        return self.buf[k][1], self.buf[k][2]


class AcousticTracker:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.enabled = cfg.beacon.range_bearing
        self.v_bound = speed_bound(cfg)
        self.slot_of: Dict[int, int] = {}
        self.fixes: Dict[int, List[Fix]] = {}             # slot -> fixes (accepted and rejected), oldest first
        self.track_id: Dict[int, int] = {}
        self.pose = PoseHistory()
        self.stats = {"received": 0, "stale": 0, "no_pose": 0, "implausible_depth": 0, "accepted": 0, "rejected": 0,
                      "track_restarts": 0}

    def set_identities(self, ids: Dict[int, int]) -> None:
        self.slot_of = {int(k): int(v) for k, v in ids.items()}

    # ------------------------------------------------------------------ ingestion
    def update(self, t: float, rx, p_now: np.ndarray, yaw_now: float) -> List[Fix]:
        """Store the own pose of this step and turn the receptions of this frame into fixes."""
        self.pose.push(t, p_now, yaw_now)
        new = []
        if not self.enabled:
            return new
        a, env = self.cfg.acoustic, self.cfg.env
        for item in rx or ():
            ident, t_arr, r, az, el = int(item[0]), float(item[1]), float(item[2]), float(item[3]), float(item[4])
            slot = self.slot_of.get(ident)
            if slot is None:
                continue
            self.stats["received"] += 1
            if t - t_arr > a.latency_max_s + 1e-9:
                self.stats["stale"] += 1
                continue
            pose = self.pose.at(t_arr)
            if pose is None:
                self.stats["no_pose"] += 1
                continue
            p_own, yaw = pose
            R = _rot(yaw)
            body, slack = region_samples(r, az, el, self.cfg)
            u = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
            rel = R @ (r * u)
            f = Fix(slot=slot, t=t_arr, r=r, az=az, el=el, p=p_own + rel, pts=p_own + body @ R.T, slack=slack, rel=rel)
            if not (env.z_min - a.depth_tol_m <= f.p[2] <= env.z_max + a.depth_tol_m):
                f.accepted, f.reason = False, "depth"
                self.stats["implausible_depth"] += 1
            else:
                self._gate(f)
            self.fixes.setdefault(slot, []).append(f)
            new.append(f)
        for slot, lst in self.fixes.items():
            self.fixes[slot] = [f for f in lst if t - f.t <= _KEEP_S]
        return new

    def consistent(self, f: Fix, g: Fix) -> bool:
        """Kinematic consistency of two fixes of one identity.  The biases of the envelope (clock offset, sound speed,
        own heading bias) are constant over a run and cancel between two fixes: their difference is the motion
        (<= v_bound dt) plus the white part of the errors, k sqrt(2) sigma_w(r), sigma_w the white position std."""
        a = self.cfg.acoustic
        r = max(f.r, g.r)
        sw = math.sqrt(a.sigma_range_m ** 2 + (r * math.radians(a.sigma_az_deg)) ** 2
                       + (r * math.radians(a.sigma_el_deg)) ** 2)
        return float(np.linalg.norm(f.p - g.p)) <= (self.v_bound * abs(f.t - g.t) + a.k_sigma * math.sqrt(2.0) * sw
                                                    + a.track_gate_m)

    def _gate(self, f: Fix) -> None:
        lst = self.fixes.get(f.slot, [])
        last = next((g for g in reversed(lst) if g.accepted), None)
        tid = self.track_id.get(f.slot, 0)
        if last is None or self.consistent(f, last):
            f.accepted, f.track = True, tid
            self.stats["accepted"] += 1
            return
        f.accepted, f.reason, f.track = False, "kinematic", tid
        self.stats["rejected"] += 1
        prev = next((g for g in reversed(lst) if not g.accepted and g.reason == "kinematic"), None)
        if prev is not None and f.t - last.t >= REINIT_S and self.consistent(f, prev):
            tid += 1                                      # a new track: nothing of the old one carries over
            self.track_id[f.slot] = tid
            prev.track, f.track = tid, tid
            prev.accepted = f.accepted = True
            prev.reason = f.reason = "restart"
            self.stats["track_restarts"] += 1

    # ------------------------------------------------------------------ queries
    def all_fixes(self, slot: int, t0: float = -1e9, t1: float = 1e9) -> List[Fix]:
        return [f for f in self.fixes.get(slot, []) if t0 <= f.t <= t1]

    def latest(self, slot: int, t: float, max_age: float) -> Optional[Fix]:
        """Latest accepted fix of ``slot`` not older than ``max_age`` (None: no evidence, never absence)."""
        for f in reversed(self.fixes.get(slot, [])):
            if f.accepted and t - f.t <= max_age:
                return f
            if t - f.t > max_age:
                break
        return None
