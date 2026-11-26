"""Mutual exclusion (P2) as a constraint on the final velocity command, and its joint feasibility with separation (P1).

Why.  The stage-2 hold before the critical region (CR) used to sit at s = -(0.8 + hull half-length + 0.15) = -1.18 m
(with the former symmetric CR |s| <= 0.8), 0.38 m from the CR, while the declared gate-frame navigation error is
``nav_err_m`` = 1.0 m: a drone holding there on an estimate 1 m short of the truth has its centre inside the CR.  And no gate rule edited the avoidance output of a drone
that is not admitted: the separation warning filter of a drone held before the CR answers a drone behind it (a REAR
threat) by moving FORWARD, the only directions with a guaranteed opening from a REAR sector pattern (formal F1p, F1pa).

What.  A drone that is not authorized to be in the CR (not committed and admitted, ``Flows.cr_cleared``) treats the CR
COLUMN as a forbidden set for its TRUE centre: the CR of property P2 extended to every depth, so that a drone that is not
admitted neither enters the CR nor passes under or over it (a REAR threat and a forward drift otherwise let a drone
descend below the CR and pass forward under the frame, P2-safe by the centre definition but an unauthorized passage).
The CR itself is the exact occupancy of property P2 (cr_s_min <= s <= cr_s_max, |l| <= cr_half_width,
|dz| <= cr_half_height: GateRule.in_cr; the approach up to the gate plane, s in [-0.8, 0.0]; the bar plane beyond it, up to
passage_clear_s, is a P1 / physical-clearance matter).  A velocity command v is
P2-SAFE when the backup that follows it cannot reach the CR under the declared bounds:

* the command is applied for ``tau_c`` = one control period + the largest data age (dt + tau_max), the true velocity
  reaching an increase of the command at once (no acceleration bound is assumed towards the command) and losing at
  least ``a_brake`` per second towards a decrease;
* then the drone brakes to rest at ``a_brake`` (the declared braking capability of the P1 proofs);
* then it holds position with the steady error ``e_track`` = w_drift_max / k_hold of a first-order position loop
  against the declared residual drift (k_hold = the smallest hold gain of the flows, 0.5);
* the unrejected residual drift ``w_drift_max`` acts in any direction throughout;
* the true position differs from the estimate by at most ``nav_err_m`` on each horizontal gate-frame axis (the declared
  premise of P2, |e_h| <= nav_err_m, monitored per run); the same bound is taken vertically, more than the depth
  sensor's envelope (the control layer reads only the gate and envelope settings: tests/test_b2_isolation.py).

Per gate-frame axis the reach of the backup (``axis_reach``) is the running maximum of the displacement under the upper
(lower) velocity envelope plus the drift, plus the hold error at the end; the swept set is contained in the box hull of
the position grown by these reaches and the navigation error, and v is P2-safe when that box and the CR are disjoint
(separated along s or l: the column has no vertical bound).  This ONE-STEP margin certifies the control interval: the
command of a step is applied for dt <= tau_c, the first phase of its own backup (formal P2B1).  Its terminal hold is
stated around the TRUE rest point, which a position loop on the estimate does not realise (during the braking the estimate
moves by up to rho T more than the truth): the recursive argument below uses the one-step margin for the control interval
only, and certifies what follows by station keeping.

Station keeping (SK) and the recursive argument.  The terminal hold of a backup is a position loop on the ESTIMATE, so its
anchor must be fixed in estimate coordinates: station keeping on an anchor a (``GateConstraint.hold_velocity``: gain
K_HOLD, capped at HOLD_V_MAX on each gate-frame axis, so that every axis stays restoring against the drift) is rated by
``hold_margin`` from the band (``sk_band``) of each side of each axis: the closed-form braking from the upper true
velocity (``brake_reach``: X = a0+^2 / (2 a_brake) + w T, at rest at T = max(tau_c, a0+ / a_brake)), plus the drift of the
estimate relative to the truth over T (``rho`` T: the estimate integrates the measured velocity, rho >= its error bound),
where the loop starts; then the loop keeps the deviation of the estimate within max(the deviation where it starts,
e_track + w tau_c) - premise H: after the braking the velocity the dead reckoning integrates follows the loop command up
to the residual drift, the decision latency adding w tau_c (formal P2R3a; for this per-axis capped law the band also holds
when instead the TRUE motion follows the command, P2R3c) - plus the navigation error and the slack.  A positive SK
margin computed at the time the maneuver is engaged (latched, ``P2Certificate``) is a statement about the whole future of
the true trajectory under the declared bounds: continuing that maneuver needs no re-certification (a margin recomputed later
adds a fresh measurement inflation and may drop: it is only logged).  The planners and the mission filter use the ROBUST
margin (cr_margin with ``lookahead``): the one-step margin and the SK margin of station keeping on the NEXT estimate, from
every state one control period later that is consistent with the bounds - the true displacement and velocity on the envelope
after dt, the next estimate within ``rho`` dt of the true displacement - the smaller of the two.  A command with a positive
robust margin therefore hands the next step a certificate (kind NEXT) for engaging station keeping where the drone then
is, without the next measurement; that maneuver, once engaged, is latched.  Induction (formal P2R1-P2R4): a certified
command at step k (robust margin > 0, or a latched SK) leaves a certified action at k + 1 (SK on the next estimate, or the
continuation of the latch), so the certified set is never empty while the declared bounds hold; base case: on entering the
active range (ACTIVE_RANGE_M) SK is certified for every admissible velocity.  Switching from a latched maneuver to any other
command requires that command's robust margin > 0.  A certificate is dropped when its premises are no longer the ones it
was computed with: another gate, a position correction of the estimate (an estimate jump: DroneController), a larger
estimate drift bound rho during its braking phase (a DVL outage).  The applied command of every step is certified
(``GateConstraint.certify``); if none is, the controller logs P2_UNCERTIFIED.  A drone that keeps commanding zero velocity is
not holding: each step re-anchors the backup where the residual drift has carried it (formal/check_f1p_kinematic.py found
such a drone carried into the CR while every single step looked P2-safe).

Mission commands (``filter_mission``).  A mission command whose robust margin is positive is kept; the position loop of
a hold (the stage-2 hold, the standoff, the failsafe hold and its probe) is kept when it IS that loop, unsaturated, station
keeping on its target is certified (hold_margin, or the latched certificate of that anchor) and the loop cannot saturate
inside its band (``latchable``: its horizontal cap is a 2-norm one), else station keeping on the target replaces it; any
other command that is not P2-safe becomes station keeping on the point where the cut began (latched by Flows.p2_filter; a
valid certificate's anchor when the cut begins) - a scaled-down velocity command would let the drift accumulate - then a
scaled command, then (holding not P2-safe) a horizontal retreat of the largest margin.  The measured velocity carries an
error bound ``v_err`` (DVL noise while fresh, the speed bound during an outage: DroneController).

Hold point.  The stage-2 hold of a drone whose gate localization is not validated is the nearest point on the axis that
is P2-safe when holding: ``hold_s`` = cr_s_min - (nav_err_m + reach of a hold + ``SLACK_M``) = -2.45 m, near the
keep-out plane of an uncommitted drone (Flows.keepout_filter: s = cr_s_min - (R_OUT + nav_err_m) = -2.44 m).  No new
premise on the error after a validated fit is introduced (the fit uncertainty is not an error bound,
perception/nav.py), so the same point serves a localized drone that waits for the stage-2 check.

Joint feasibility (``classify``).  For a drone in an avoidance mode the escape planner evaluates every candidate
(direction x speed, and holding still) for P1 and for P2 and reports, besides the chosen command,

* JOINTLY_FEASIBLE: a P2-safe candidate gives the drone its full P1 share - an opening speed of at least ``v_open``
  from every threat of the warning band (SEPARATION_WARNING, lemma S2b), at least g_min * v_escape from every threat of
  the avoidance band (COLLISION_AVOIDANCE, lemma S2a);
* UNCERTAIN: no P2-safe candidate gives the full share, but one does not close on any threat in the worst case
  (holding still always qualifies when it is P2-safe).  Separation then rests on the threat opening (formal S1g: the
  pairwise radial model with one drone non-closing and the other reacting keeps d >= d_safe); a drone cannot observe
  whether its threat is free to do so, so the decision cannot be certified onboard.  Also UNCERTAIN: a threat seen
  only in part because a sector of its pattern is unhealthy, or sonar data older than tau_max;
* JOINTLY_INFEASIBLE: no P2-safe candidate is non-closing for every threat, or no candidate is P2-safe at all (the
  state is outside the invariant: a navigation premise or a drift bound was exceeded, an estimate jumped).

Fallback, explicitly documented.  While a P2-safe candidate exists the CR stays a hard constraint (the user brief: P2
"as a control constraint wherever physically enforceable") and P1 is best effort inside it: the P2-safe candidate with
the largest worst-case opening speed, then the existing deterministic selection rules.  With no P2-safe candidate the
command that opens the largest P2 margin among the non-closing ones is taken (moving away from the CR also moves away
from an occupant inside it), else the one with the largest worst-case opening.  Neither is labelled safe: the decision
is logged with the constraint that could not be certified and the reason.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig

FREE = "NOT_CONSTRAINED"
FEASIBLE = "JOINTLY_FEASIBLE"
UNCERTAIN = "UNCERTAIN"
INFEASIBLE = "JOINTLY_INFEASIBLE"
RECOVERY = "P2_RECOVERY"
STATUSES = (FREE, FEASIBLE, UNCERTAIN, INFEASIBLE)

K_HOLD = 0.5
HOLD_V_MAX = 0.27  # above sqrt(3) * w_drift_max = 0.26 m/s: the loop never saturates against the declared drift (formal P2B4)
SLACK_M = 0.05
COLUMN = True
REACH_STEPS = 48
ACTIVE_RANGE_M = 8.0


@dataclass(frozen=True)
class P2Bounds:
    """Declared bounds of the backup (module docstring); every number comes from FleetConfig."""

    e_h: float
    e_z: float
    w: float
    e_track: float
    tau_c: float
    a_brake: float
    slack: float = SLACK_M
    dt: float = 0.1

    @classmethod
    def from_cfg(cls, cfg: FleetConfig = DEFAULT, e_h: Optional[float] = None) -> "P2Bounds":
        env = cfg.env
        k = min(K_HOLD, cfg.gate.k_track)
        return cls(e_h=float(cfg.gate.nav_err_m if e_h is None else e_h), e_z=float(cfg.gate.nav_err_m if e_h is None else e_h),
                   w=float(env.w_drift_max), e_track=float(env.w_drift_max / k), tau_c=float(env.dt + env.tau_max),
                   a_brake=float(env.a_brake), dt=float(env.dt))


def axis_reach(u0: np.ndarray, u: np.ndarray, b: P2Bounds, hold_error: bool = True) -> np.ndarray:
    """Reach of the backup along one axis, (..., 2) = (towards +, towards -) in metres, for current velocity u0 and
    command u (arrays of the same shape).  Upper envelope of the velocity towards each side: an increase of the command
    is reached at once, a decrease at no less than a_brake; after tau_c the drone brakes to rest at a_brake (an increase
    towards rest is immediate).  The drift adds w t; the reach is the running maximum of the displacement, and the hold
    error e_track is added at rest (trapezoidal integration of the piecewise-linear envelope on REACH_STEPS points).
    ``hold_error`` False: the running maximum of the displacement alone (the braking part of a station-keeping hold)."""
    u0, u = np.broadcast_arrays(np.asarray(u0, float), np.asarray(u, float))
    out = np.empty(u.shape + (2,))
    tau, a = b.tau_c, b.a_brake
    s = np.linspace(0.0, 1.0, REACH_STEPS + 1).reshape((-1,) + (1,) * u.ndim)
    for k, sgn in enumerate((1.0, -1.0)):
        a0, a1 = sgn * u0, sgn * u
        up = a1 >= a0
        ua = np.where(up, a1, np.maximum(a1, a0 - a * tau))
        T = tau + np.maximum(ua, 0.0) / a
        t = s * T
        in_a = np.where(up, a1, np.maximum(a1, a0 - a * np.minimum(t, tau)))
        in_b = np.maximum(ua - a * np.maximum(t - tau, 0.0), 0.0)
        vel = np.where(t <= tau, in_a, in_b) + b.w
        x = np.concatenate([np.zeros((1,) + u.shape), np.cumsum(0.5 * (vel[1:] + vel[:-1]) * np.diff(t, axis=0), axis=0)])
        out[..., k] = np.maximum(x.max(axis=0), x[-1] + b.e_track) if hold_error else x.max(axis=0)
    return out


def hold_reach(b: P2Bounds) -> float:
    """Reach of holding still (u0 = u = 0): drift over tau_c plus the hold error."""
    return float(axis_reach(np.zeros(1), np.zeros(1), b)[0, 0])


def brake_reach(a0, b: P2Bounds):
    """Closed form of braking along one side of one axis (zero command): ``a0`` the upper bound of the true velocity
    towards that side (<= 0: away from it).  The envelope is max(0, a0 - a_brake t) (an increase towards rest at once),
    at rest at T = max(tau_c, a0+ / a_brake) (the latency phase is never shorter than tau_c), the drift w towards the side
    throughout.  Returns (X, T): X = a0+^2 / (2 a_brake) + w T, the displacement at T and its running maximum (the
    integrand is >= w > 0); both are non-decreasing in a0 (formal P2R1)."""
    a0p = np.maximum(np.asarray(a0, float), 0.0)
    T = np.maximum(b.tau_c, a0p / b.a_brake)
    return a0p ** 2 / (2.0 * b.a_brake) + b.w * T, T


def sk_hold(b: P2Bounds) -> float:
    """Deviation band of the station-keeping loop at rest: e_track + w tau_c (the steady error against the drift and the
    decision latency; premise H)."""
    return b.e_track + b.w * b.tau_c


def sk_band(q, a0p, a0m, anchor, b: P2Bounds, rho: float, amp: float = 0.0):
    """Band of station keeping on ``anchor`` (module docstring), per axis, before the navigation error and the slack:
    (lo, hi, T).  ``a0p`` / ``a0m``: upper bounds of the true velocity towards + / towards - (speeds); ``rho``: bound of
    the drift of the estimate relative to the truth; ``amp``: a vertical probe around the anchor.  Towards each side the
    estimate is at most q + X + rho T where the loop starts (the true centre during the braking at most q + X + the
    navigation error), then within max(that, anchor + e_track + w tau_c) (premise H).  T: the end of the braking phase."""
    Xp, Tp = brake_reach(a0p, b)
    Xm, Tm = brake_reach(a0m, b)
    hold = sk_hold(b)
    probe = np.array([0.0, 0.0, amp])
    hi = np.maximum(q + Xp + rho * Tp, anchor + probe + hold)
    lo = np.minimum(q - Xm - rho * Tm, anchor - probe - hold)
    return lo, hi, np.maximum(Tp, Tm)


def hold_s(cfg: FleetConfig = DEFAULT) -> float:
    """Stage-2 hold of a drone that is not admitted: the nearest P2-safe point on the axis when holding, under the
    declared navigation error (module docstring)."""
    b = P2Bounds.from_cfg(cfg)
    return cfg.gate.cr_s_min - (b.e_h + hold_reach(b) + b.slack)


def _after(u0: np.ndarray, u: np.ndarray, b: P2Bounds) -> tuple:
    """Worst-case state one control period dt after command u from velocity u0, per side of one axis: (displacement
    towards +, velocity towards +, displacement towards -, velocity towards -), on the envelope of axis_reach (dt <= tau_c:
    the command is still the one applied) plus the drift."""
    out = []
    for sgn in (1.0, -1.0):
        a0, a1 = sgn * u0, sgn * u
        up = a1 >= a0
        t_star = np.where(up, 0.0, np.minimum((a0 - a1) / b.a_brake, b.dt))
        x = np.where(up, a1 * b.dt,
                     a0 * t_star - 0.5 * b.a_brake * t_star ** 2 + np.maximum(a1, a0 - b.a_brake * b.dt) * (b.dt - t_star))
        vel = np.where(up, a1, np.maximum(a1, a0 - b.a_brake * b.dt))
        out += [x + b.w * b.dt, vel]
    return tuple(out)


def box_gap(lo: np.ndarray, hi: np.ndarray, cr_lo: np.ndarray, cr_hi: np.ndarray) -> np.ndarray:
    """Per-axis signed separation of the boxes [lo, hi] and [cr_lo, cr_hi] (positive = disjoint along that axis)."""
    return np.maximum(cr_lo - hi, lo - cr_hi)


def cr_margin(q: np.ndarray, v0: np.ndarray, V: np.ndarray, b: P2Bounds, cr_lo: np.ndarray, cr_hi: np.ndarray,
              lookahead: bool = False, column: bool = COLUMN, v_err: float = 0.0, rho: Optional[float] = None) -> np.ndarray:
    """Signed separation [m] between the forbidden set and the box swept by the backup of each command in V (K, 3), all in
    the gate frame; positive = disjoint (P2-safe).  ``cr_lo``, ``cr_hi``: the corners of the CR box (GateRule.cr_lo /
    cr_hi).  Navigation error: e_h on s and l, e_z on dz.  ``column``: the
    forbidden set is the CR COLUMN (cr_s_min <= s <= cr_s_max, |l| <= cr_half_width, every depth), so that a drone that is not
    admitted can neither enter the CR nor pass under or over it; separation is then possible along s and l only.
    ``v_err``: the error bound of the measured velocity v0 on each axis (the side towards which the reach is taken uses
    v0 + v_err, the other v0 - v_err).  ``lookahead``: the robust margin, the smaller of this one-step margin and the SK
    margin of station keeping on the NEXT estimate (module docstring), onboard-consistent: from the true worst-case state
    after dt (displacement and velocity on the envelope of the command from v0 +- v_err) and the next estimate within
    ``rho`` dt of the true displacement (``rho``: the bound of the drift of the estimate relative to the truth, v_err when
    None); the SK band is that of sk_band with the next estimate as anchor and the next TRUE velocity (no new
    measurement: the certificate is handed to the next step, P2Certificate kind NEXT)."""
    V = np.atleast_2d(np.asarray(V, float))
    q, v0 = np.asarray(q, float), np.asarray(v0, float)
    U0 = np.broadcast_to(v0, V.shape)
    r = np.stack([axis_reach(U0 + v_err, V, b)[..., 0], axis_reach(U0 - v_err, V, b)[..., 1]], axis=-1)
    err = np.array([b.e_h, b.e_h, b.e_z])
    n_ax = 2 if column else 3
    hi = q + r[..., 0] + err + b.slack
    lo = q - r[..., 1] - err - b.slack
    m = box_gap(lo, hi, cr_lo, cr_hi)[:, :n_ax].max(axis=1)
    if not lookahead:
        return m
    dp, dm, _T = next_band(U0, V, b, v_err, rho)
    d = float(v_err if rho is None else rho) * b.dt
    xp, _, _, _ = _after(U0 + v_err, V, b)
    _, _, xm, _ = _after(U0 - v_err, V, b)
    hi2 = q + xp + d + dp + err + b.slack
    lo2 = q - xm - d - dm - err - b.slack
    m2 = box_gap(lo2, hi2, cr_lo, cr_hi)[:, :n_ax].max(axis=1)
    return np.minimum(m, m2)


def next_band(U0, V, b: P2Bounds, v_err: float, rho: Optional[float] = None):
    """Half widths (towards +, towards -) of the band of station keeping on the NEXT estimate, around that estimate, over
    every true velocity after dt on the envelope of command V from U0 +- v_err (cr_margin lookahead), and the end of its
    braking phase counted from the next step: max(X(v') + rho T(v'), e_track + w tau_c) on each side."""
    rho = float(v_err if rho is None else rho)
    _, vp, _, _ = _after(U0 + v_err, V, b)
    _, _, _, vm = _after(U0 - v_err, V, b)
    Xp, Tp = brake_reach(vp, b)
    Xm, Tm = brake_reach(vm, b)
    hold = sk_hold(b)
    return np.maximum(Xp + rho * Tp, hold), np.maximum(Xm + rho * Tm, hold), np.maximum(Tp, Tm)


NEXT = "NEXT"
SK = "SK"


@dataclass(frozen=True)
class P2Certificate:
    """What the applied command of a step certifies for the next steps (module docstring).  ``kind`` NEXT: station keeping
    engaged at the next step on the estimate it then has (``t``: the step that issued it); SK: station keeping on
    ``anchor`` (gate frame) engaged at ``t``, valid for the whole future under the declared bounds.  ``margin``: the margin
    of the certificate (a lower bound of the CR-column separation of the true centre); ``dev``: the half width of the
    deviation band of the estimate around the anchor on each axis (the larger side); ``rho``: the drift bound of the
    estimate it assumed until ``t_end`` (the end of its braking phase); ``gate``: the gate it refers to."""

    kind: str
    margin: float
    t: float
    rho: float
    t_end: float
    gate: object
    anchor: Optional[np.ndarray] = None
    dev: Optional[np.ndarray] = None

    def record(self) -> Dict:
        return {"kind": self.kind, "margin": round(float(self.margin), 3), "t": round(float(self.t), 2),
                "anchor": None if self.anchor is None else np.round(self.anchor, 3).tolist()}


def latchable(dev: Optional[np.ndarray], gain: float, cap_h: Optional[float]) -> bool:
    """A position loop of gain ``gain`` whose horizontal command is capped in 2-norm at ``cap_h`` (Flows: the stage-2 hold,
    the standoff, the failsafe hold) stays unsaturated inside a deviation band of half widths ``dev`` (gate frame s, l)
    when gain |dev_h| <= cap_h: then every horizontal axis is restoring at gain >= K_HOLD and the band is invariant (formal
    P2R3).  The band of the column axes (s, l) is independent of the vertical loop (clipped per axis).  ``cap_h`` None: the
    per-axis capped law of GateConstraint.hold_velocity, latchable for every band."""
    if cap_h is None:
        return True
    if dev is None:
        return False
    return bool(gain * float(np.linalg.norm(np.asarray(dev, float)[:2])) <= cap_h + 1e-9)


@dataclass
class GateConstraint:
    """P2 constraint of one control step: the gate, the own gate-frame estimate and velocity, the bounds.  ``v_err``: the
    error bound of the measured velocity; ``rho``: the bound of the drift of the estimate relative to the truth (v_err when
    None; DroneController adds the compass and DVL scale term of NavBound); ``t``: the onboard time of the step (the
    validity of the certificates, P2Certificate)."""

    gate: object
    q: np.ndarray
    v0: np.ndarray
    bounds: P2Bounds
    cr_lo: np.ndarray
    cr_hi: np.ndarray
    v_err: float = 0.0
    rho: Optional[float] = None
    t: float = 0.0

    @property
    def rho_eff(self) -> float:
        return float(self.v_err if self.rho is None else self.rho)

    def margins_world(self, V_world: np.ndarray, robust: bool = True) -> np.ndarray:
        """P2 margins of world velocities: robust (onboard-consistent lookahead: SK on the next estimate) and against the CR
        column by default (cr_margin)."""
        V = np.atleast_2d(np.asarray(V_world, float))
        a = np.asarray(self.gate.axis, float)
        left = np.array([-a[1], a[0], 0.0])
        Vg = np.stack([V @ a, V @ left, V[:, 2]], axis=1)
        return cr_margin(self.q, self.v0, Vg, self.bounds, self.cr_lo, self.cr_hi, lookahead=robust, v_err=self.v_err,
                         rho=self.rho_eff)

    def margins_body(self, V_body: np.ndarray, R_wb: np.ndarray, robust: bool = True) -> np.ndarray:
        return self.margins_world(np.atleast_2d(np.asarray(V_body, float)) @ np.asarray(R_wb, float).T, robust)

    def hold_velocity(self, anchor: np.ndarray) -> np.ndarray:
        """World velocity of station keeping on a gate-frame ``anchor``: the position loop of gain K_HOLD, capped at
        HOLD_V_MAX on each gate-frame axis (> w_drift_max: every axis stays restoring against the declared drift, whatever
        the deviation of the others; formal P2R3).  A zero-velocity command is not a hold: against the declared residual
        drift it moves the drone by up to w_drift_max per second, step after step."""
        d = np.clip(K_HOLD * (np.asarray(anchor, float) - self.q), -HOLD_V_MAX, HOLD_V_MAX)
        a = np.asarray(self.gate.axis, float)
        left = np.array([-a[1], a[0], 0.0])
        return d[0] * a + d[1] * left + np.array([0.0, 0.0, d[2]])

    def _sk(self, anchor: np.ndarray, amp: float = 0.0):
        """(margin, deviation band half widths, end of the braking phase) of station keeping on ``anchor`` engaged now."""
        b = self.bounds
        a = np.asarray(anchor, float)
        lo, hi, T = sk_band(self.q, self.v0 + self.v_err, self.v_err - self.v0, a, b, self.rho_eff, amp)
        err = np.array([b.e_h, b.e_h, b.e_z]) + b.slack
        n_ax = 2 if COLUMN else 3
        m = float(box_gap(lo - err, hi + err, self.cr_lo, self.cr_hi)[:n_ax].max())
        return m, np.maximum(hi - a, a - lo), float(np.max(T[:n_ax]))

    def hold_margin(self, anchor: np.ndarray, amp: float = 0.0) -> float:
        """P2 margin of station keeping on ``anchor`` (gate frame) engaged now, from the measured velocity within v_err:
        the band of sk_band (braking in closed form, the drift of the estimate rho T until the loop starts, then the loop
        band max(the deviation where it starts, e_track + w tau_c)), plus the navigation error and the slack, against the
        CR column.  Positive = P2-safe for as long as that hold lasts (premises of the module docstring): the certificate
        of the maneuver, latched when it is engaged (P2Certificate).  Recomputed at a later step it adds a fresh
        measurement inflation and may be smaller (formal P2B4): that does not invalidate the latched certificate.
        ``amp``: a vertical probe of that amplitude around the anchor (the failsafe probe) widens the vertical band."""
        return self._sk(anchor, amp)[0]

    def cert_valid(self, cert: Optional[P2Certificate]) -> bool:
        """The premises a certificate was computed with still hold: the same gate; for NEXT, the step right after the one
        that issued it; during its braking phase (t <= t_end) a drift bound of the estimate not above the one it assumed
        (a DVL outage that begins raises it).  The controller drops every certificate on a position correction."""
        if cert is None or cert.gate != getattr(self.gate, "gate_id", None):
            return False
        dt = self.bounds.dt
        if cert.kind == NEXT:
            return bool(1e-9 < self.t - cert.t <= 1.5 * dt and self.rho_eff <= cert.rho + 1e-12)
        return bool(self.t >= cert.t - 1e-9 and (self.t > cert.t_end + 1e-9 or self.rho_eff <= cert.rho + 1e-12))

    def cert_anchor(self, cert: Optional[P2Certificate]) -> Optional[np.ndarray]:
        """The anchor of station keeping a valid certificate covers: its own (SK) or the present estimate (NEXT)."""
        if not self.cert_valid(cert):
            return None
        return np.asarray(cert.anchor, float).copy() if cert.kind == SK else self.q.copy()

    def _applies(self, cert: Optional[P2Certificate], anchor: np.ndarray) -> bool:
        ref = self.cert_anchor(cert)
        n = 2 if COLUMN else 3
        return ref is not None and bool(np.allclose(np.asarray(anchor, float)[:n], ref[:n], atol=1e-9))

    def certified_hold(self, anchor: np.ndarray, cert: Optional[P2Certificate] = None, amp: float = 0.0) -> float:
        """Certified margin of station keeping on ``anchor``: the margin from the present state (hold_margin) or that of a
        valid certificate covering this anchor, the larger (both are lower bounds of the true separation)."""
        m = self.hold_margin(anchor, amp)
        if self._applies(cert, anchor):
            m = max(m, float(cert.margin))
        return m

    def sk_certificate(self, anchor: np.ndarray, cert: Optional[P2Certificate] = None,
                       amp: float = 0.0, *, gain: float = K_HOLD,
                       cap: Optional[float] = None) -> Optional[P2Certificate]:
        """Certificate of station keeping on ``anchor`` from this step: the valid certificate covering it (latched: a
        NEXT one becomes an SK one on the present estimate) or one from the present state. Keep an applicable old proof
        when its braking ends no later, even if a fresh proof gives a larger margin. None when neither margin is positive."""
        a = np.asarray(anchor, float)
        m, dev, T = self._sk(a, amp)
        old = None
        if self._applies(cert, a) and cert.margin > 0.0:
            old = cert if cert.kind == SK else P2Certificate(SK, cert.margin, self.t, cert.rho, cert.t_end, cert.gate,
                                                               self.q.copy(), cert.dev)
        if m <= 0.0:
            return old
        if old is not None:
            earlier = old.t_end <= self.t + T and (self.t > old.t_end + 1e-9 or old.rho >= self.rho_eff)
            keep_old = earlier
            if not latchable(dev, gain, cap) or (keep_old and latchable(old.dev, gain, cap)):
                return old
        return P2Certificate(SK, m, self.t, self.rho_eff, self.t + T, getattr(self.gate, "gate_id", None), a.copy(), dev)

    def next_certificate(self, v_world: np.ndarray) -> Optional[P2Certificate]:
        """Certificate (kind NEXT) a command with a positive robust margin hands to the next step; None otherwise."""
        v = np.atleast_2d(np.asarray(v_world, float))
        m = float(self.margins_world(v)[0])
        if m <= 0.0:
            return None
        Vg = self.to_gate(v[0])[None, :]
        dp, dm, T = next_band(self.v0[None, :], Vg, self.bounds, self.v_err, self.rho_eff)
        n_ax = 2 if COLUMN else 3
        return P2Certificate(NEXT, m, self.t, self.rho_eff, self.t + self.bounds.dt + float(np.max(T[0, :n_ax])),
                             getattr(self.gate, "gate_id", None), None, np.maximum(dp, dm)[0])

    def certify(self, v: np.ndarray, sk_anchors=(), loops=(), cert: Optional[P2Certificate] = None
                ) -> Optional[P2Certificate]:
        """The certificate the applied command ``v`` (world) carries to the next step: station keeping on one of
        ``sk_anchors`` (the per-axis capped law of hold_velocity), the position loop of a hold described by one of the
        ``loops`` (mission_anchor dicts: anchor, gain, amp, cap) when it cannot saturate in its band, else a command with a
        positive robust margin (NEXT).  None: the command is not certified (the controller logs P2_UNCERTIFIED)."""
        v = np.asarray(v, float)
        for a in sk_anchors:
            if a is not None and np.allclose(v, self.hold_velocity(a), atol=1e-6):
                c = self.sk_certificate(a, cert)
                if c is not None:
                    return c
        for kw in loops:
            a = kw.get("anchor") if kw else None
            if a is None:
                continue
            gain, amp = float(kw.get("gain", K_HOLD)), float(kw.get("amp", 0.0))
            if self.is_hold_loop(v, a, gain, amp):
                cap = kw.get("cap")
                c = self.sk_certificate(a, cert, amp, gain=gain, cap=cap)
                if c is not None and latchable(c.dev, gain, cap):
                    return c
        return self.next_certificate(v)

    def to_gate(self, v_world: np.ndarray) -> np.ndarray:
        return np.asarray(self.gate.vec_to_gate_frame(np.asarray(v_world, float)), float)

    def from_gate(self, v_gate: np.ndarray) -> np.ndarray:
        a = np.asarray(self.gate.axis, float)
        left = np.array([-a[1], a[0], 0.0])
        return v_gate[0] * a + v_gate[1] * left + np.array([0.0, 0.0, v_gate[2]])

    def is_hold_loop(self, v: np.ndarray, anchor: np.ndarray, gain: float, amp: float = 0.0) -> bool:
        """``v`` is the unsaturated position loop of gain ``gain`` (>= K_HOLD) on ``anchor``, up to a vertical probe of
        amplitude ``amp`` around it: the deviation bound of hold_margin applies to it."""
        if gain < K_HOLD - 1e-9:
            return False
        expect = gain * (np.asarray(anchor, float) - self.q)
        tol = np.array([1e-6, 1e-6, 1e-6 + gain * amp])
        return bool(np.all(np.abs(self.to_gate(v) - expect) <= tol))

    def filter_mission(self, v: np.ndarray, v_max: float, anchor: Optional[np.ndarray] = None, gain: float = K_HOLD,
                       amp: float = 0.0, station: Optional[np.ndarray] = None, cap: Optional[float] = None,
                       cert: Optional[P2Certificate] = None) -> "MissionFilter":
        """The P2-safe velocity closest to a mission velocity along its own direction (largest safe scale); when even
        holding is not P2-safe, the velocity of the largest margin among a direction set at the cruise speed.
        ``anchor`` (gate frame), ``gain``, ``amp``, ``cap``: the mission command may be the position loop of a hold on that
        point (the stage-2 hold, the failsafe hold and its probe; ``cap`` the 2-norm cap of its horizontal command); it is
        kept when it IS that loop, unsaturated (is_hold_loop), station keeping there is certified (sk_certificate: the
        margin from the present state or the latched certificate ``cert`` of that anchor) and the loop cannot saturate in
        its band (latchable); certified but able to saturate, station keeping on that point replaces it.  A mission command
        that is not P2-safe is replaced by station keeping on ``station`` (gate frame: where the cut began, latched by the
        caller), else on the anchor of a valid certificate, else on the current estimate, the first that is certified: a
        scaled-down velocity command
        would let the residual drift accumulate step after step, the position loop does not.  Only when that is not
        certified the command is scaled along its own direction; recovery (holding not P2-safe): a horizontal retreat -
        the CR column cannot be left vertically - of the largest margin, ties towards the reverse of the gate axis."""
        v = np.asarray(v, float)
        m = float(self.margins_world(v)[0])
        if m > 0.0:
            return MissionFilter(v, m, FEASIBLE, False, "")
        if anchor is not None and self.is_hold_loop(v, anchor, gain, amp):
            c = self.sk_certificate(anchor, cert, amp, gain=gain, cap=cap)
            if c is not None:
                if latchable(c.dev, gain, cap):
                    return MissionFilter(v, c.margin, FEASIBLE, False, "station keeping on the hold point")
                return MissionFilter(self.hold_velocity(anchor), c.margin, FEASIBLE, True,
                                     "station keeping on the hold point (per-axis capped: the loop of the flow could "
                                     "saturate in its band)", np.asarray(anchor, float).copy())
        ca = self.cert_anchor(cert)
        for st in ([] if station is None else [np.asarray(station, float)]) + ([] if ca is None else [ca]) + [self.q.copy()]:
            ms = self.certified_hold(st, cert)
            if ms > 0.0:
                return MissionFilter(self.hold_velocity(st), ms, FEASIBLE, True,
                                     "station keeping where the mission command was cut (its backup reaches the CR)", st)
        m0 = float(self.margins_world(np.zeros(3))[0])
        if m0 > 0.0:
            lo, hi = 0.0, 1.0
            for _ in range(14):
                mid = 0.5 * (lo + hi)
                if float(self.margins_world(mid * v)[0]) > 0.0:
                    lo = mid
                else:
                    hi = mid
            return MissionFilter(lo * v, float(self.margins_world(lo * v)[0]), FEASIBLE, True,
                                 f"mission command scaled by {lo:.2f} (backup reaches the CR)")
        ang = np.radians(np.arange(0.0, 360.0, 15.0))
        dirs_g = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], axis=1)
        Vg = np.concatenate([sp * dirs_g for sp in (v_max, 0.5 * v_max, 0.25 * v_max)])
        V = np.array([self.from_gate(x) for x in Vg])
        M = self.margins_world(V)
        key = M - 0.02 * np.concatenate([dirs_g[:, 0]] * 3)
        i = int(np.argmax(key))
        return MissionFilter(V[i], float(M[i]), RECOVERY, True,
                             "holding is not P2-safe (state outside the invariant): horizontal retreat of the largest margin")


@dataclass
class MissionFilter:
    v: np.ndarray
    margin: float
    status: str
    modified: bool
    reason: str
    station: Optional[np.ndarray] = None


@dataclass
class JointDecision:
    """Outcome of a gate-aware avoidance decision (logged by the controller)."""

    status: str = FREE
    share: str = ""
    p2_margin: Optional[float] = None
    opening: Optional[float] = None
    n_candidates: int = 0
    n_p2_rejected: int = 0
    n_p1_rejected: int = 0
    reason: str = ""
    uncertified: List[str] = field(default_factory=list)

    def record(self) -> Dict:
        return {"status": self.status, "share": self.share,
                "p2_margin": None if self.p2_margin is None else round(self.p2_margin, 3),
                "opening": None if self.opening is None else round(self.opening, 3), "n": self.n_candidates,
                "p2_rejected": self.n_p2_rejected, "p1_rejected": self.n_p1_rejected, "reason": self.reason,
                "uncertified": list(self.uncertified)}


def classify(openings: np.ndarray, margins: np.ndarray, full: float, degraded: bool = False) -> str:
    """Joint-feasibility class of a candidate set (module docstring): ``openings`` the worst-case opening speed of each
    candidate over every threat [m/s], ``margins`` its P2 margin, ``full`` the opening of a full P1 share."""
    safe = margins > 0.0
    if not safe.any() or not (safe & (openings >= -1e-9)).any():
        return INFEASIBLE
    if degraded or not (safe & (openings >= full - 1e-9)).any():
        return UNCERTAIN
    return FEASIBLE


def constraint_for(cfg: FleetConfig, gate, p: np.ndarray, v_world: np.ndarray, authorized: bool,
                   e_h: Optional[float] = None, v_err: float = 0.0, rho: Optional[float] = None,
                   t: float = 0.0) -> Optional[GateConstraint]:
    """The constraint of this step, or None: no gate, the drone authorized in the CR, or the CR out of reach.  ``v_err``:
    the error bound of the measured velocity (the DVL noise while it is fresh, the speed bound during an outage); ``rho``:
    the bound of the drift of the estimate relative to the truth (GateConstraint); ``t``: the onboard time."""
    if gate is None or authorized:
        return None
    G = cfg.gate
    q = np.asarray(gate.to_gate_frame(np.asarray(p, float)), float)
    lo, hi = G.cr_lo(), G.cr_hi()
    if float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0))) > ACTIVE_RANGE_M:
        return None
    return GateConstraint(gate=gate, q=q, v0=np.asarray(gate.vec_to_gate_frame(np.asarray(v_world, float)), float),
                          bounds=P2Bounds.from_cfg(cfg, e_h), cr_lo=lo, cr_hi=hi, v_err=float(v_err),
                          rho=None if rho is None else float(rho), t=float(t))
