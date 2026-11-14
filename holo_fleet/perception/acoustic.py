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
  that identity (displacement <= v_bound dt + k sqrt(2) sigma_white + ``track_gate_m``, and range change <= the
  sender's motion + the own displacement + k sqrt(2) sigma_range + RANGE_SLACK_M: a reflection is always longer).
  An inconsistent fix is
  REJECTED but kept (the gate uses the white part of the errors only: the biases are common to two fixes); it is contradictory evidence, and whatever a rejected fix could mean is never ignored by a
  decision that relies on the identity's absence (gate_perception).  Two mutually consistent rejected fixes, while
  the track has had no accepted fix for ``REINIT_S``, start a new track (a new track id: whatever was established
  on the old one is not carried over).

Payload (H6, perception/payload.py).  A reception may carry the sender's decoded onboard position estimate p_rep
and its error bound b_rep (a ball that contains the sender's true position at the emission instant, under the
sender's declared navigation envelope).  It is attached to the fix only if it is CONSISTENT with the measurement:
some point of the fix's error region, which is expressed in the receiver's own estimated frame, lies within
b_rep + v_bound age_tx + the receiver's own bound b_own + slack of p_rep.  An inconsistent payload means that one of
the two navigation estimates, or the measurement, is outside its envelope: it is not used (``rep_ok`` False) and the
fix keeps only its measurement region.  Nothing is inferred from a missing payload.

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

_N = 7
REINIT_S = 3.0
_KEEP_S = 30.0
RANGE_SLACK_M = 0.15


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_bounds(r: float, cfg: FleetConfig = DEFAULT) -> Tuple[float, float, float]:
    """Half-widths of the error box (range [m], azimuth [rad], elevation [rad]) of a fix at range r."""
    a = cfg.acoustic
    br = a.k_sigma * a.sigma_range_m + a.sound_speed_err * r + a.clock_offset_m
    ba = math.radians(a.k_sigma * a.sigma_az_deg + a.doa_bias_deg + a.heading_err_deg)
    be = math.radians(a.k_sigma * a.sigma_el_deg + a.doa_bias_deg + a.tilt_err_deg)
    return br, ba, be


def region_samples(r: float, az: float, el: float, cfg: FleetConfig = DEFAULT) -> Tuple[np.ndarray, float]:
    """Samples of the error box of a measurement (body frame, relative) and the slack that covers the box."""
    br, ba, be = box_bounds(r, cfg)
    rr = np.linspace(max(r - br - cfg.acoustic.los_excess_max_m, 0.0), r + br, _N)
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
    t: float
    r: float
    az: float
    el: float
    p: np.ndarray
    pts: np.ndarray
    slack: float
    rel: np.ndarray
    accepted: bool = True
    track: int = 0
    reason: str = ""
    rep: Optional[object] = None
    t_tx: float = 0.0
    rep_reason: str = ""

    @property
    def rep_ok(self) -> bool:
        return self.rep is not None

    def rep_radius(self, t: float, v_bound: float) -> float:
        """Radius of the ball around the reported position that contains the sender at time t."""
        return float(self.rep.bound) + v_bound * abs(t - self.t_tx)

    @property
    def extent(self) -> float:
        """Radius of a ball around ``p`` containing the whole region."""
        return float(np.max(np.linalg.norm(self.pts - self.p, axis=1))) + self.slack


@dataclass(frozen=True)
class GateStatus:
    """Latest gate phase a drone reported about itself (payload v2, perception/payload.py)."""
    t_tx: float
    gate_idx: int
    phase: int


@dataclass
class PoseHistory:
    keep_s: float = 3.0
    buf: Deque[Tuple[float, np.ndarray, float]] = field(default_factory=deque)

    def push(self, t: float, p: np.ndarray, yaw: float, bound: Optional[float] = None) -> None:
        self.buf.append((t, np.array(p, dtype=float), float(yaw), bound))
        while self.buf and t - self.buf[0][0] > self.keep_s:
            self.buf.popleft()

    def _nearest(self, t: float):
        if not self.buf or t < self.buf[0][0] - 0.11 or t > self.buf[-1][0] + 0.11:
            return None
        return self.buf[min(range(len(self.buf)), key=lambda i: abs(self.buf[i][0] - t))]

    def at(self, t: float) -> Optional[Tuple[np.ndarray, float]]:
        """Own pose estimate at time t (nearest stored instant; None if t is not covered)."""
        e = self._nearest(t)
        return None if e is None else (e[1], e[2])

    def bound_at(self, t: float) -> Optional[float]:
        """Own navigation error bound at time t (None if unknown).  The bound never decreases (nav.NavBound), so for
        an instant older than the stored history the latest bound is a valid upper bound."""
        e = self._nearest(t)
        if e is None and self.buf and t < self.buf[0][0]:
            e = self.buf[-1]
        return None if e is None or e[3] is None else float(e[3])


class AcousticTracker:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg
        self.enabled = cfg.beacon.enabled
        self.v_bound = speed_bound(cfg)
        self.slot_of: Dict[int, int] = {}
        self.fixes: Dict[int, List[Fix]] = {}
        self.track_id: Dict[int, int] = {}
        self.last_accepted: Dict[int, Fix] = {}
        self.heard: Dict[int, float] = {}
        self.t_start: Optional[float] = None
        self.status: Dict[int, GateStatus] = {}
        self.pose = PoseHistory()
        self.stats = {"received": 0, "stale": 0, "no_pose": 0, "implausible_depth": 0, "accepted": 0, "rejected": 0,
                      "track_restarts": 0}

    def set_identities(self, ids: Dict[int, int]) -> None:
        self.slot_of = {int(k): int(v) for k, v in ids.items()}

    def update(self, t: float, rx, p_now: np.ndarray, yaw_now: float, bound_now: Optional[float] = None) -> List[Fix]:
        """Store the own pose (and own navigation bound) of this step and turn the receptions of this frame into
        fixes."""
        if self.t_start is None:
            self.t_start = t
        self.pose.push(t, p_now, yaw_now, bound_now)
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
            self.heard[slot] = max(self.heard.get(slot, -1e9), t_arr)
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
            f.t_tx = t_arr - r / self.cfg.beacon.sound_speed
            self._attach(f, item[5] if len(item) > 5 else None)
            self.fixes.setdefault(slot, []).append(f)
            new.append(f)
            if f.accepted:
                self.last_accepted[slot] = f
        for slot, lst in self.fixes.items():
            self.fixes[slot] = [f for f in lst if t - f.t <= _KEEP_S]
        return new

    def _attach(self, f: Fix, packet) -> None:
        """Decode the payload: the gate status of the sender is kept when the packet is intact and of this frame (even if
        the fix itself is rejected: a reflected arrival carries the same packet); the position payload is kept only if
        the fix is accepted and consistent with the measurement (module docstring)."""
        from holo_fleet.perception.payload import decode, frame_index

        rep = decode(packet) if packet is not None else None
        if rep is None:
            f.rep_reason = "missing"
            return
        if rep.frame != frame_index(f.t_tx, self.cfg.beacon.period_s):
            f.rep_reason = "frame"
            self.stats["payload_frame"] = self.stats.get("payload_frame", 0) + 1
            return
        old = self.status.get(f.slot)
        if old is None or f.t_tx >= old.t_tx:
            self.status[f.slot] = GateStatus(t_tx=f.t_tx, gate_idx=rep.gate_idx, phase=rep.phase)
        if not rep.valid_position:
            f.rep_reason = "invalid_bound"
            return
        if not f.accepted:
            f.rep_reason = "rejected_fix"
            return
        b_own = self.pose.bound_at(f.t)
        if b_own is None or not math.isfinite(b_own):
            f.rep_reason = "no_own_bound"
            return
        reach = rep.bound + self.v_bound * max(f.t - f.t_tx, 0.0) + b_own + self.v_bound * 0.11 + f.slack
        if float(np.min(np.linalg.norm(f.pts - rep.p, axis=1))) > reach:
            f.rep_reason = "inconsistent"
            self.stats["payload_inconsistent"] = self.stats.get("payload_inconsistent", 0) + 1
            return
        f.rep, f.rep_reason = rep, "ok"
        self.stats["payload_ok"] = self.stats.get("payload_ok", 0) + 1

    def consistent(self, f: Fix, g: Fix) -> bool:
        """Kinematic consistency of two fixes of one identity.  The biases of the envelope (clock offset, sound speed,
        own heading bias) are constant over a run and cancel between two fixes: their difference is the motion
        (<= v_bound dt) plus the white part of the errors, k sqrt(2) sigma_w(r), sigma_w the white position std."""
        a = self.cfg.acoustic
        r = max(f.r, g.r)
        dt = abs(f.t - g.t)
        sw = math.sqrt(a.sigma_range_m ** 2 + (r * math.radians(a.sigma_az_deg)) ** 2
                       + (r * math.radians(a.sigma_el_deg)) ** 2)
        if float(np.linalg.norm(f.p - g.p)) > self.v_bound * dt + a.k_sigma * math.sqrt(2.0) * sw + a.track_gate_m:
            return False
        own = float(np.linalg.norm((f.p - f.rel) - (g.p - g.rel)))
        return abs(f.r - g.r) <= self.v_bound * dt + own + a.k_sigma * math.sqrt(2.0) * a.sigma_range_m + RANGE_SLACK_M

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
            tid += 1
            self.track_id[f.slot] = tid
            prev.track, f.track = tid, tid
            prev.accepted = f.accepted = True
            prev.reason = f.reason = "restart"
            self.stats["track_restarts"] += 1

    def shift(self, corr: np.ndarray) -> None:
        """A correction of the own position estimate moves the frame the stored fixes and poses are expressed in with it (a
        map fit, a measured gate crossing): they are kept in the frame of the estimate."""
        done = set()
        for lst in list(self.fixes.values()) + [list(self.last_accepted.values())]:
            for f in lst:
                if id(f) not in done:
                    done.add(id(f))
                    f.p = f.p + corr
                    f.pts = f.pts + corr
        self.pose.buf = deque((t, p + corr, y, b) for t, p, y, b in self.pose.buf)

    def deaf(self, t: float) -> bool:
        """Nobody at all has been heard for the silence interval (a deaf receiver, a silent channel): the absence of every
        other drone is no evidence about any of them."""
        return bool(self.heard) and all(t - h >= self.cfg.form.t_silent_s for h in self.heard.values())

    def silent(self, slot: int, t: float, after: Optional[float] = None) -> bool:
        """Not heard for the declared silence interval (``after``, default t_silent_s): a dead drone, or lost pings that
        cannot be told from one.  A receiver that hears nobody at all (a deaf receiver or a silent channel) declares
        no one silent: the absence of every other drone is no evidence about any of them.  A drone of the plan never heard is
        silent since the receiver started (a failure before its first reception)."""
        lim = self.cfg.form.t_silent_s if after is None else after
        if t - self.heard.get(slot, t if self.t_start is None else self.t_start) < lim:
            return False
        return any(t - h < lim for k, h in self.heard.items() if k != slot)

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
