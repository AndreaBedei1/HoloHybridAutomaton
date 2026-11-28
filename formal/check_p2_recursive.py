"""P2 recursive safety argument: certified station keeping, the onboard-consistent lookahead and the induction over the
certificates (control/gate_safety.py, module docstring).  Part of the gate-aware suite (formal/check_gate_safety.py run()).

Model, per gate-frame axis (the CR column is separated along s or l; each side of an axis is the + side of the mirrored
axis, so every lemma is stated for the + side).  q the estimate, p = q + e the truth, |e| <= E (= nav_err_m, premise N);
the measured velocity within v_err of the true one (premise V); the estimate drifts from the truth by at most rho per
second (rho = v_err + k_lin v_bound, NavBound; premise D: no position correction - the controller drops every certificate
on one); the true velocity inside the response envelope of the command (gate_safety.axis_reach), the residual drift at most
w.  Premise H (hold loop): after the braking from the engagement velocity, the velocity the dead reckoning integrates
follows the loop command up to the residual drift w, with the decision latency covered by the allowance w tau_c of the
band (``q-loop`` reading); for the per-axis capped law of hold_velocity the band is also proved when instead the TRUE motion
follows the command up to w (``p-loop`` reading: the loop then tracks the moving target a + e(t)).

P2R1   (proved, Z3 QF_NRA) the closed-form braking reach (gate_safety.brake_reach) X(a0) = a0+^2 / (2 a) + w T,
       T = max(tau_c, a0+ / a): non-decreasing in a0, and X is the maximum over [0, T] of the displacement under the
       zero-command envelope plus the drift (P2R1x).  P2R1n (numeric): the deployed function equals the closed form and a
       dense integration of the envelope.
P2R2   (proved, Z3 QF_NRA) onboard-consistent lookahead: from every state after dt consistent with the bounds - true
       displacement <= xp, true velocity <= vp (the envelope after dt), next estimate within rho dt of the true
       displacement - the band of station keeping engaged at the next step on the next estimate, from the true velocity, is
       inside the band the deployed lookahead (cr_margin, next_band) evaluated at step k: a positive robust margin hands
       the next step a certificate (NEXT) without the next measurement.  P2R2n (numeric): the deployed lookahead equals that
       formula computed independently (the envelope after dt by the exact pieces of check_gate_safety.upper_at).
P2R3   (proved, Z3) the station-keeping band is invariant: P2R3a q-loop, any per-axis restoring law of gain in
       [K_HOLD, 1/dt] capped at >= HOLD_V_MAX per axis, any band half widths >= e_track (sampled loop, residual |w|);
       P2R3b the 2-norm capped laws of the flows stay unsaturated inside a latchable band (gate_safety.latchable); P2R3c
       p-loop, the per-axis capped law of hold_velocity on the truth with |e| <= E: band half widths >= E + e_track; P2R3d
       the true centre stays out of the column during the braking and the loop when the SK margin is positive. P2R3e
       retaining a compatible SK proof whose braking ends no later and whose drift coverage dominates preserves validity.
P2R3n  (numeric) sampled closed loop of station keeping with the latency (data age tau_max, period dt), the velocity lag of
       the envelope, adversarial drift and estimate error inside the premises, from the next states of P2R4n and from the
       stage-2 hold: the true centre never enters the CR column (checks the latency allowance of premise H).
P2R4   (proved, Z3, induction) recursive availability: if the step where the constraint becomes active is certified and
       the premises hold at every step, every applied command is certified (P2_UNCERTIFIED never logged) and the true centre
       stays out of the column; the lemmas P2R2, P2R3, P2R5 and the deployed selection rule are its axioms (the selection
       rule - a certified command is applied whenever one exists - is checked numerically by J1 and P2R4n).
P2R4n  (numeric, grid of P2B5) every command returned with a positive robust margin by the deployed warning filter, escape
       and filter_mission, at every worst-case next state (side, estimate drift and measurement error at the corners):
       the deployed next-step GateConstraint finds a certified station keeping (sk_certificate with the handed NEXT
       certificate); reported, how many of those next states rely on the handed certificate (the margin recomputed with the
       fresh measurement is not positive) and how many have no onboard-certified braking.
P2R2m  (mutation) the braking lookahead (braking from the true worst-case state, re-certified at the next step with the
       fresh measurement) on the same grid: next states where braking is not certified onboard - the 744 / 1124
       counterexample of that lookahead, kept as a witness.
P2R4l  (declared limit) the induction step needs its premises at the next step: a DVL outage that begins before a
       handed certificate is used, or a position correction, can leave no certified action (P2_UNCERTIFIED, retreat).
P2R6   (proved, Z3 QF_LRA, induction over the steps of a DVL outage) the outage hull of gate_safety.outage_hull: with the true
       velocity within k_rw of the last fresh measurement (premise V), a residual drift |d| <= w at every step (it may reverse),
       the response part of the true velocity between its previous value and each command (the response envelope of
       axis_reach) and the dead reckoning integrating the held sample or a model blend towards the command, the response part
       and the dead-reckoning velocity stay in [lo, up] (lo / up: the measurement -/+ (k_rw + w) and the commands so far), so the
       true velocity is in [lo - w, up + w] and the dead reckoning drifts from the truth by at most the deployed rho of the
       outage.  P2R2o (proved): the reserve handed before an onset, the hull width + w, dominates that rho for every
       dead-reckoning velocity inside the hull.  P2R6m / P2R6m2 (mutations): the measured side with w instead of 2 w (a drift
       that reverses), the hull without the commands.  P2R6n (numeric): the deployed outage_hull equals the closed form.
       Premise of the outage (declared, evidenced by the replay of archive v27-final-paired-campaign-fixed-2026-11-25, not proved): the response envelope holds without DVL
       feedback (the low-level loop then runs feedforward with its integral frozen).
P2R5   (proved, Z3 QF_NRA) base case: station keeping on the estimate is certified at every state whose column separation
       along s or l is at least D_base, for every measured speed per axis up to v_bound (fresh DVL); reported, the smallest
       column separation at activation (ACTIVE_RANGE_M, the depth envelope of the arena).
"""

from __future__ import annotations

import itertools
import math
import time

import numpy as np
import z3

from common import CheckResult, check, exact  # noqa: E402

from holo_fleet.config import FleetConfig  # noqa: E402
from holo_fleet.control import gate_safety as gs  # noqa: E402

ENC = "formal/check_p2_recursive.py"


def _rho(cfg: FleetConfig, fresh: bool = True) -> float:
    from holo_fleet.perception.nav import NavBound

    nb = NavBound(cfg)
    return (float(nb.k_rw) if fresh else 2 * float(nb.v_bound)) + float(nb.k_lin) * float(nb.v_bound)


def _ve(cfg: FleetConfig, fresh: bool = True) -> float:
    from holo_fleet.perception.nav import NavBound

    nb = NavBound(cfg)
    return float(nb.k_rw) if fresh else float(nb.v_bound)


def _gc(cfg, q, v0, ve, rho, t=0.0):
    import check_gate_safety as C

    lo, hi = C._box(cfg)
    return gs.GateConstraint(C.GATE, np.asarray(q, float), np.asarray(v0, float), gs.P2Bounds.from_cfg(cfg), lo, hi,
                             v_err=float(ve), rho=float(rho), t=float(t))


class _P:
    def __init__(self, cfg: FleetConfig):
        b = gs.P2Bounds.from_cfg(cfg)
        self.b = b
        self.a, self.w, self.tau, self.dt = exact(b.a_brake), exact(b.w), exact(b.tau_c), exact(b.dt)
        self.E, self.sl, self.et = exact(b.e_h), exact(b.slack), exact(b.e_track)
        self.H = self.et + self.w * self.tau
        self.K, self.Vmax = exact(gs.K_HOLD), exact(gs.HOLD_V_MAX)


def zX(a0, P: _P):
    """Closed form of brake_reach in Z3: (X, T)."""
    pos = z3.If(a0 > 0, a0, z3.RealVal(0))
    T = z3.If(pos <= P.a * P.tau, P.tau, pos / P.a)
    return pos * pos / (2 * P.a) + P.w * T, T


def _zmax(x, y):
    return z3.If(x >= y, x, y)


# ---------------------------------------------------------------------------------------------------------------- P2R1
def reach_lemmas(cfg: FleetConfig):
    P = _P(cfg)
    a0, a1, t = z3.Reals("a0 a1 t")
    X0, T0 = zX(a0, P)
    X1, T1 = zX(a1, P)
    r1 = check("P2R1 the closed-form braking reach X(a0) = a0+^2 / (2 a_brake) + w T, T = max(tau_c, a0+ / a_brake), is "
               "non-decreasing in the upper velocity a0 (for all reals)",
               "a0 <= a1 -> X(a0) <= X(a1) & T(a0) <= T(a1)", ENC, [a0 <= a1, z3.Or(X0 > X1, T0 > T1)], "unsat",
               note=f"a_brake {P.b.a_brake}, w {P.b.w}, tau_c {P.b.tau_c}: the onboard upper bounds of the next step (a "
                    f"larger velocity) never give a smaller reach (used by P2R2, P2R5)")
    pos = z3.If(a0 > 0, a0, z3.RealVal(0))
    xt = z3.If(t <= pos / P.a, pos * t - P.a * t * t / 2, pos * pos / (2 * P.a)) + P.w * t
    r2 = check("P2R1x the closed-form X is the maximum over [0, T] of the displacement under the zero-command envelope "
               "max(0, a0 - a_brake t) plus the drift w, and is reached at T (the envelope is at rest from T on)",
               "0 <= t <= T(a0) -> x(t) <= X(a0); x(T(a0)) = X(a0)", ENC,
               [z3.Or(z3.And(t >= 0, t <= T0, xt > X0),
                      z3.substitute(xt, (t, T0)) != X0)], "unsat",
               note="x(t) the integral of the envelope in closed form (piecewise quadratic): the running maximum is the "
                    "end value (integrand >= w > 0)")
    b = P.b
    g = np.linspace(-1.6, 1.6, 641)
    X, T = gs.brake_reach(g, b)
    worst = 0.0
    for a0v, Xv, Tv in zip(g, X, T):
        ts = np.linspace(0.0, Tv, 4001)
        f = np.maximum(a0v - b.a_brake * ts, 0.0) + b.w
        xi = float(np.sum(0.5 * (f[1:] + f[:-1]) * np.diff(ts)))
        closed = max(a0v, 0.0) ** 2 / (2 * b.a_brake) + b.w * max(b.tau_c, max(a0v, 0.0) / b.a_brake)
        worst = max(worst, abs(xi - Xv), abs(closed - Xv))
    ok = worst < 1e-6
    r3 = CheckResult("P2R1n the deployed gate_safety.brake_reach equals the closed form and a dense integration of the "
                     "envelope (numeric)", "forall a0 in grid: |brake_reach - closed| , |brake_reach - integral| < 1e-6",
                     ENC, "holds", verdict="holds" if ok else "violated", passed=ok,
                     note=f"{g.size} velocities in [-1.6, 1.6] m/s, 4000 trapezoids each; largest difference {worst:.1e} m")
    return [r1, r2, r3]


# ---------------------------------------------------------------------------------------------------------------- P2R2
def lookahead_lemma(cfg: FleetConfig):
    P = _P(cfg)
    q, x, xp, v1, vp, dq, rho = z3.Reals("q x xp v1 vp dq rho")
    q1 = q + x + dq
    Xv, Tv = zX(v1, P)
    Xp, Tp = zX(vp, P)
    band_next = _zmax(q1 + Xv + rho * Tv, q1 + P.H)
    deployed = q + xp + rho * P.dt + _zmax(Xp + rho * Tp, P.H)
    cons = [x <= xp, v1 <= vp, rho >= 0, dq <= rho * P.dt, dq >= -rho * P.dt, band_next > deployed]
    r = check("P2R2 onboard-consistent lookahead: from every state after dt consistent with the bounds (true displacement "
              "<= xp, true velocity <= vp, next estimate within rho dt of the true displacement), the band of station keeping "
              "engaged at the next step on the next estimate is inside the band of the deployed lookahead",
              "x <= xp & v1 <= vp & |dq| <= rho dt -> max(q1 + X(v1) + rho T(v1), q1 + H) <= q + xp + rho dt + "
              "max(X(vp) + rho T(vp), H), q1 = q + x + dq", ENC, cons, "unsat",
              note=f"for all reals q, x, xp, v1, vp and every rho >= 0; H = e_track + w tau_c = {P.b.e_track + P.b.w * P.b.tau_c:.4f} m; "
                   f"the - side is the + side of the mirrored axis.  With the CR separation of P2R3d it gives: robust "
                   f"margin > 0 at step k -> station keeping on the next estimate certified at k + 1 (kind NEXT), with no "
                   f"use of the next measurement (the 2 v_err inflation of a re-certification is not needed)")
    wit = check("P2R2w witness: the premises of P2R2 are satisfiable with the deployed band attained (non-vacuous, tight "
                "at x = xp, v1 = vp, dq = rho dt)", "exists a state with the premises and band_next = deployed", ENC,
                cons[:-1] + [band_next == deployed, rho > 0, vp > P.a * P.tau], "sat")
    return [r, wit, lookahead_numeric(cfg)]


def lookahead_numeric(cfg: FleetConfig, n: int = 4000, seed: int = 5) -> CheckResult:
    import check_gate_safety as C

    b = gs.P2Bounds.from_cfg(cfg)
    lo, hi = C._box(cfg)
    rng = np.random.default_rng(seed)
    ve, rho = _ve(cfg), _rho(cfg)
    err = np.array([b.e_h, b.e_h, b.e_z]) + b.slack
    worst = 0.0
    for _ in range(n):
        q = np.array([rng.uniform(-6.0, 3.0), rng.uniform(-4.0, 4.0), rng.uniform(-2.0, 2.0)])
        v0 = rng.uniform(-0.6, 0.6, 3)
        u = rng.uniform(-0.6, 0.6, 3)
        dep = float(gs.cr_margin(q, v0, u[None, :], b, lo, hi, lookahead=True, v_err=ve, rho=rho)[0])
        one = float(gs.cr_margin(q, v0, u[None, :], b, lo, hi, lookahead=False, v_err=ve)[0])
        hi2, lo2 = np.zeros(3), np.zeros(3)
        for k in range(3):
            vp, xp = C.upper_at(v0[k] + ve, u[k], b, b.dt)
            vm, xm = C.upper_at(-(v0[k] - ve), -u[k], b, b.dt)
            band = []
            for vv in (vp, vm):
                pos = max(vv, 0.0)
                T = max(b.tau_c, pos / b.a_brake)
                band.append(max(pos * pos / (2 * b.a_brake) + b.w * T + rho * T, b.e_track + b.w * b.tau_c))
            hi2[k] = q[k] + xp + rho * b.dt + band[0]
            lo2[k] = q[k] - xm - rho * b.dt - band[1]
        m2 = float(np.max(np.maximum(lo - (hi2 + err), (lo2 - err) - hi)[:2]))
        worst = max(worst, abs(dep - min(one, m2)))
    ok = worst < 1e-6
    return CheckResult("P2R2n the deployed robust margin (cr_margin lookahead, next_band) is min(one-step margin, the "
                       "lookahead band of P2R2) computed independently (numeric)",
                       "forall sampled (q, v0, u): |deployed - min(m1, m2_independent)| < 1e-6", ENC, "holds",
                       verdict="holds" if ok else "violated", passed=ok,
                       note=f"{n} random states and commands (|v| <= 0.6 m/s), v_err {ve:.3f}, rho {rho:.3f} m/s; the state "
                            f"after dt by the exact pieces of check_gate_safety.upper_at; largest difference {worst:.1e} m")


# ---------------------------------------------------------------------------------------------------------------- P2R3
def loop_lemmas(cfg: FleetConfig):
    P = _P(cfg)
    out = []
    d, om, g, Dp, Dm, Vc = z3.Reals("d om g Dp Dm Vc")
    gd = g * d
    u = -z3.If(gd > Vc, Vc, z3.If(gd < -Vc, -Vc, gd))
    d1 = d + P.dt * (u + om)
    cons = [g >= P.K, g * P.dt <= 1, Vc >= P.Vmax, Dp >= P.w / P.K, Dm >= P.w / P.K, d <= Dp, d >= -Dm,
            om <= P.w, om >= -P.w, z3.Or(d1 > Dp, d1 < -Dm)]
    out.append(check("P2R3a station-keeping band, q-loop (premise H on the estimate): every per-axis restoring loop of gain "
                     "in [K_HOLD, 1/dt], capped per axis at >= HOLD_V_MAX, keeps the deviation of the estimate inside any "
                     "band [-Dm, Dp] with Dm, Dp >= e_track against the residual drift (sampled loop; inductive invariant)",
                     "-Dm <= d <= Dp & |w_k| <= w -> -Dm <= d + dt (-sat(g d) + w_k) <= Dp", ENC, cons, "unsat",
                     note=f"K_HOLD {gs.K_HOLD}, HOLD_V_MAX {gs.HOLD_V_MAX} > w {P.b.w}, dt {P.b.dt}; covers hold_velocity "
                          f"(gain K_HOLD, per-axis cap) and the unsaturated loops of the flows (gain k_track {cfg.gate.k_track}); "
                          f"the band of sk_band has half widths >= e_track + w tau_c (the latency allowance: premise H, "
                          f"P2R3n); base case: the loop starts inside the band by construction (q + X + rho T, P2R1)"))
    ds, dl, Ds, Dl, ch = z3.Reals("ds dl Ds Dl ch")
    cons = [g >= 0, ch >= 0, g * g * (Ds * Ds + Dl * Dl) <= ch * ch, ds <= Ds, ds >= -Ds, dl <= Dl, dl >= -Dl,
            g * g * (ds * ds + dl * dl) > ch * ch]
    out.append(check("P2R3b a horizontal loop capped in 2-norm at c_h stays unsaturated inside a latchable band "
                     "(gate_safety.latchable: g |D_h| <= c_h), so P2R3a applies to it on s and l",
                     "g^2 (Ds^2 + Dl^2) <= c_h^2 & |ds| <= Ds & |dl| <= Dl -> g^2 (ds^2 + dl^2) <= c_h^2", ENC, cons, "unsat",
                     note="the stage-2 hold / standoff (k_track, v_pass) and the failsafe hold (0.5, 0.25 m/s) of Flows; a "
                          "loop that is not latchable is replaced by hold_velocity on its anchor (filter_mission)"))
    p, e, Dq = z3.Reals("p e Dq")
    gk = P.K * (e - p)
    u = z3.If(gk > P.Vmax, P.Vmax, z3.If(gk < -P.Vmax, -P.Vmax, gk))
    p1 = p + P.dt * (u + om)
    cons = [Dq >= P.E + P.w / P.K, p <= Dq, p >= -Dq, e <= P.E, e >= -P.E, om <= P.w, om >= -P.w, z3.Or(p1 > Dq, p1 < -Dq)]
    out.append(check("P2R3c station-keeping band, p-loop (the TRUE motion follows the command up to the drift): "
                     "hold_velocity on the estimate q = p - e, |e| <= E, keeps the true deviation p - a inside any band "
                     "|p - a| <= D with D >= E + e_track (per-axis cap: every axis restoring whatever the others)",
                     "|p| <= D & |e| <= E & |w_k| <= w -> |p + dt (clip(K (e - p), +-HOLD_V_MAX) + w_k)| <= D", ENC, cons,
                     "unsat", note="the band of hold_margin in true coordinates is max(p_s - a, E + e_track) + the latency "
                                   "allowance: the deployed band (+ rho T) contains it, so the per-axis capped law is "
                                   "certified under either reading of premise H; a 2-norm cap is not (an axis with a small "
                                   "deviation may lose its restoring component when the estimate deviation is large)"))
    q0, e0, x, X, rho, T, dqs, a, qt, et = z3.Reals("q0 e0 x X rho T dqs a qt et")
    hi = _zmax(q0 + X + rho * T, a + P.H)
    clo = z3.Real("cr_lo")
    margin_pos = clo - (hi + P.E + P.sl) > 0
    braking = z3.And(x <= X, q0 + e0 + x >= clo)
    loop = z3.And(qt <= hi, qt + et >= clo)
    cons = [rho >= 0, T >= 0, e0 <= P.E, e0 >= -P.E, et <= P.E, et >= -P.E, dqs <= rho * T, dqs >= -rho * T,
            margin_pos, z3.Or(braking, loop)]
    out.append(check("P2R3d a positive station-keeping margin keeps the true centre out of the CR column: during the braking "
                     "(true displacement <= X, P2R1x) and in the loop (estimate inside the band, P2R3a / P2R3b, truth within "
                     "E of it)", "cr_lo - (max(q0 + X + rho T, a + H) + E + slack) > 0 -> p(t) < cr_lo", ENC, cons, "unsat",
                     note="separation along one axis towards its + side (s or l, either side by mirroring), as P2B1"))
    now, use_t, old_end, new_end, old_rho, new_rho, use_rho = z3.Reals("now use_t old_end new_end old_rho new_rho use_rho")
    time_tol, rho_tol = exact(1e-9), exact(1e-12)
    cons = [use_t >= now, old_end <= new_end, z3.Or(now > old_end + time_tol, old_rho >= new_rho),
            z3.Or(use_t > new_end + time_tol, use_rho <= new_rho + rho_tol),
            use_t <= old_end + time_tol, use_rho > old_rho + rho_tol]
    out.append(check("P2R3e retaining an eligible station-keeping proof with an earlier end of braking preserves its "
                     "drift-regime validity when the older reserve dominates or its braking has ended",
                     "old_brake_ends_no_later & (old_finished | old_rho >= new_rho) & new_valid -> old_valid",
                     ENC, cons, "unsat", note="Same gate and hold law on the same horizontal anchor, positive margins, "
                     "and no position correction. The formula uses the tolerances of GateConstraint.cert_valid."))
    out.append(loop_numeric(cfg))
    return out


def _sim_sk(cfg, a, p0, v0, e0, modes, horizon: float = 40.0, h: float = 0.01):
    """Sampled closed loop of hold_velocity on the estimate anchor ``a`` (N, 2: s, l), true start ``p0``, true velocity
    ``v0``, estimate error ``e0``; the command of each control period from the estimate tau_max old, the true velocity on
    the envelope (an increase at once, a decrease at a_brake), the drift and the estimate error moving adversarially
    (``modes``: drift 'out' = away from the anchor, '+', '-'; estimate error 'out' (truth pushed outwards), '+', '-'
    at rate rho, clipped at E).  Returns the true trajectory extremes per axis (min, max)."""
    b = gs.P2Bounds.from_cfg(cfg)
    rho = _rho(cfg)
    env = cfg.env
    n_ctl = int(round(b.dt / h))
    lag = int(round(env.tau_max / h))
    p, v, e = p0.copy(), v0.copy(), e0.copy()
    hist = [p - e] * (lag + 1)
    pmin, pmax = p.copy(), p.copy()
    u = np.zeros_like(p)
    for k in range(int(horizon / h)):
        if k % n_ctl == 0:
            qd = hist[0]
            u = np.clip(gs.K_HOLD * (a - qd), -gs.HOLD_V_MAX, gs.HOLD_V_MAX)
        v = np.where(u >= v, u, np.maximum(u, v - b.a_brake * h))
        side = np.sign(p - a)
        side[side == 0] = 1.0
        dm, em = modes
        w = b.w * (side if dm == "out" else (1.0 if dm == "+" else -1.0))
        de = rho * h * (side if em == "out" else (1.0 if em == "+" else -1.0))
        p = p + (v + w) * h
        e = np.clip(e + de, -b.e_h, b.e_h)
        hist = hist[1:] + [p - e]
        pmin, pmax = np.minimum(pmin, p), np.maximum(pmax, p)
    return pmin, pmax


def loop_numeric(cfg: FleetConfig) -> CheckResult:
    """P2R3n on states engaging station keeping: next states of commands near the CR (P2R4n grid, sampled) and the
    stage-2 hold at rest."""
    t0 = time.time()
    b = gs.P2Bounds.from_cfg(cfg)
    G = cfg.gate
    ve, rho = _ve(cfg), _rho(cfg)
    lo_cr, hi_cr = G.cr_lo(), G.cr_hi()
    rng = np.random.default_rng(3)
    starts = []
    for _ in range(600):
        q = np.array([rng.uniform(G.cr_s_min - 3.4, G.cr_s_min - 1.5), rng.uniform(-3.0, 3.0), 0.0])
        vm = rng.uniform(-0.6, 0.6, 3)
        gc = _gc(cfg, q, vm, ve, rho)
        if gc.hold_margin(q) > 0.0:
            starts.append((q, vm, gc.hold_margin(q)))
    q_h = np.array([G.stage2_hold_s, 0.0, 0.0])
    for vm in itertools.product((-ve, 0.0, ve), repeat=2):
        starts.append((q_h, np.array([vm[0], vm[1], 0.0]), _gc(cfg, q_h, np.array([vm[0], vm[1], 0.0]), ve, rho).hold_margin(q_h)))
    A = np.array([s[0][:2] for s in starts])
    VM = np.array([s[1][:2] for s in starts])
    worst_in = 0
    worst_gap = np.inf
    n = 0
    rest = slice(len(starts) - 9, len(starts))
    excess = -np.inf
    band_t = b.e_h + b.e_track
    for vs in (1.0, -1.0):
        for es in (1.0, -1.0):
            V0 = VM + vs * ve
            E0 = np.full_like(A, es * b.e_h)
            for modes in itertools.product(("out", "+", "-"), repeat=2):
                pmin, pmax = _sim_sk(cfg, A, A + E0, V0, E0, modes)
                n += len(A)
                # the CR footprint is crossed only if both axes overlap it at the same time; the extremes give a
                # conservative test: separated along s (max s < cr_s_min) or along l
                sep_s = lo_cr[0] - pmax[:, 0]
                sep_l = np.maximum(lo_cr[1] - pmax[:, 1], pmin[:, 1] - hi_cr[1])
                gap = np.maximum(sep_s, sep_l)
                worst_gap = min(worst_gap, float(gap.min()))
                worst_in += int((gap <= 0.0).sum())
                dev = np.maximum(pmax[rest] - A[rest], A[rest] - pmin[rest]) - band_t
                excess = max(excess, float(dev.max()))
    ok = worst_in == 0
    return CheckResult("P2R3n sampled closed loop of station keeping (hold_velocity) with the latency, the velocity lag, "
                       "adversarial drift and estimate error inside the premises: no start certified by hold_margin lets the "
                       "true centre reach the CR column (numeric)",
                       "forall sampled certified starts and adversarial modes: true (s, l) extremes separated from the CR",
                       ENC, "holds", verdict="holds" if ok else "violated", passed=ok, seconds=round(time.time() - t0, 1),
                       note=f"{len(starts)} certified starts (random states before the CR with |v| <= 0.6 m/s, the stage-2 "
                            f"hold with the measured velocity at +-v_err), true velocity at the measured one +- v_err, the "
                            f"estimate error at +-E, 9 drift / error modes, {n} closed loops of 40 s at 10 ms, command every "
                            f"{b.dt} s from data {cfg.env.tau_max} s old; {worst_in} reached the column; smallest separation "
                            f"of the true extremes {worst_gap:.3f} m (conservative: extremes of s and l taken independently).  Latency allowance "
                            f"of premise H: from the stage-2 hold at rest the true deviation from the anchor exceeded E + "
                            f"e_track by at most {excess:.4f} m (allowance w tau_c = {b.w * b.tau_c:.4f} m)")


# ---------------------------------------------------------------------------------------------------------------- P2R4
def theorem(cfg: FleetConfig) -> CheckResult:
    """Induction over the certificate state of two consecutive steps (abstract: the lemmas as axioms)."""
    nxt = [z3.Bool(f"next_{i}") for i in range(2)]
    sk = [z3.Bool(f"sk_{i}") for i in range(2)]
    prem = [z3.Bool(f"prem_{i}") for i in range(2)]
    robust = [z3.Bool(f"robust_{i}") for i in range(2)]
    skq = [z3.Bool(f"skq_{i}") for i in range(2)]
    cont = [z3.Bool(f"cont_{i}") for i in range(2)]
    fresh = [z3.Bool(f"fresh_{i}") for i in range(2)]
    safe = [z3.Bool(f"safe_{i}") for i in range(2)]
    avail = [z3.Or(robust[i], skq[i], cont[i], fresh[i]) for i in range(2)]
    ax = []
    for i in range(2):
        ax += [z3.Implies(avail[i], z3.Or(nxt[i], sk[i])),           # selection rule (J1, P2R4n)
               z3.Implies(nxt[i], robust[i]), z3.Implies(sk[i], z3.Or(skq[i], cont[i], fresh[i])),
               z3.Implies(z3.And(prem[i], z3.Or(nxt[i], sk[i])), safe[i])]  # P2B1 (interval), P2R3d
    ax += [z3.Implies(z3.And(nxt[0], prem[1]), skq[1]),                # P2R2
           z3.Implies(z3.And(sk[0], prem[1]), cont[1])]                # P2R3a-c (latched band invariant)
    inv = [z3.And(z3.Or(nxt[i], sk[i]), z3.Implies(prem[i], safe[i])) for i in range(2)]
    base = check("P2R4 base: the step where the constraint becomes active is certified (P2R5: SK on the estimate)",
                 "prem_0 & fresh_0 -> certified_0 & safe_0", ENC,
                 ax + [prem[0], fresh[0], z3.Not(inv[0])], "unsat")
    step = check("P2R4 recursive availability (induction step): a certified command at step k and the premises at k + 1 "
                 "leave a certified command applied at k + 1 and the true centre out of the column: the certified set is "
                 "never empty and P2_UNCERTIFIED is never logged while the declared bounds hold",
                 "(next_k | sk_k) & prem_{k+1} & axioms -> (next_{k+1} | sk_{k+1}) & safe_{k+1}", ENC,
                 ax + [inv[0], prem[1], z3.Not(inv[1])], "unsat",
                 note="axioms: the selection rule (a certified command is applied whenever one exists: checked numerically "
                      "by J1 and P2R4n, not proved), P2R2 (NEXT -> SK on the next estimate certified), P2R3a-d (a latched "
                      "SK stays certified, the true centre out of the column), P2B1 (a robust command keeps the control "
                      "interval out of the column), P2R5 (base).  Premises per step: N, V, D (no position correction: the "
                      "controller drops the certificates), the envelope, H, the same gate, rho not raised during a braking "
                      "phase (a DVL outage onset can break the step: declared limit)")
    step.note = (step.note or "") + f"; base case: {base.verdict}"
    step.passed = step.passed and base.passed
    return step


def availability(cfg: FleetConfig):
    """P2R4n and P2R2m on the grid of P2B5 (check_gate_safety.robust_selection)."""
    import check_gate_safety as C

    b = gs.P2Bounds.from_cfg(cfg)
    ve, rho = _ve(cfg), _rho(cfg)
    t0 = time.time()
    n_cmd = n_next = fail = need_cert = brake_off = 0
    worst_inh = np.inf
    first = None
    for q, v0, gc, who, u in C.returned_commands(cfg, lambda q, v0: _gc(cfg, q, v0, ve, rho, t=1.0)):
        cert = gc.certify(u, (), (), None)
        n_cmd += 1
        if cert is None or cert.kind != gs.NEXT:
            fail += 1
            continue
        ug = gc.to_gate(u)
        Cn = C._corners_after(v0, ug, b, ve)
        for sel in itertools.product((0, 1), repeat=3):
            x = np.array([Cn[k][sel[k]][0] for k in range(3)])
            v = np.array([Cn[k][sel[k]][1] for k in range(3)])
            sgn = np.array([1.0 if s == 0 else -1.0 for s in sel])
            for dsg, msg in ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0)):
                q1 = q + x + dsg * sgn * rho * b.dt
                vm1 = v + msg * sgn * ve
                g1 = _gc(cfg, q1, vm1, ve, rho, t=1.0 + b.dt)
                n_next += 1
                c1 = g1.sk_certificate(q1, cert)
                if c1 is None:
                    fail += 1
                    if first is None:
                        first = {"q": q.tolist(), "v0": v0.tolist(), "u": np.round(ug, 3).tolist(), "who": who,
                                 "q1": np.round(q1, 3).tolist(), "vm1": np.round(vm1, 3).tolist()}
                    continue
                worst_inh = min(worst_inh, float(cert.margin))
                need_cert += g1.hold_margin(q1) <= 0.0
                brake_off += float(gs.cr_margin(q1, vm1, np.zeros((1, 3)), b, *C._box(cfg), v_err=ve)[0]) <= 0.0
    ok = fail == 0 and n_cmd > 0
    r = CheckResult("P2R4n every command returned with a positive robust margin (warning filter, escape with the HOLD, "
                    "filter_mission) leaves, at every worst-case next state, a certified station keeping to the deployed "
                    "next-step constraint (numeric, grid of P2B5)",
                    "forall returned u, corners (side, estimate drift +-rho dt, measurement +-v_err): sk_certificate(q1, "
                    "NEXT) is not None", ENC, "holds", verdict="holds" if ok else "violated", passed=ok,
                    seconds=round(time.time() - t0, 1), counterexample=first,
                    note=f"{n_cmd} returned commands, {n_next} next states (8 side corners x 3 drift / measurement corners); "
                         f"{fail} without a certified action; {need_cert} of the next states rely on the handed certificate "
                         f"(the SK margin recomputed with the fresh measurement is not positive: the R-c drop), {brake_off} "
                         f"have no onboard-certified braking (one-step margin of a zero command, the braking lookahead); "
                         f"smallest handed margin {worst_inh:.3f} m")
    return r


def braking_lookahead_mutation(cfg: FleetConfig) -> CheckResult:
    """P2R2m: the braking lookahead on the same grid (mutation)."""
    import check_gate_safety as C

    b = gs.P2Bounds.from_cfg(cfg)
    ve = _ve(cfg)
    box = C._box(cfg)
    t0 = time.time()
    n = bad_on = bad_phys = 0
    deficit = []
    for q, v0, gc, who, u in C.returned_commands(cfg, lambda q, v0: _BrakingLookaheadGC.make(cfg, q, v0, ve)):
        n += 1
        phys, onb, _w = C._braking_ok(cfg, q, v0, gc.to_gate(u), b, ve, box)
        bad_phys += not phys
        if not onb:
            bad_on += 1
            Cn = C._corners_after(v0, gc.to_gate(u), b, ve)
            m = np.inf
            for sel in itertools.product((0, 1), repeat=3):
                x = np.array([Cn[k][sel[k]][0] for k in range(3)])
                v = np.array([Cn[k][sel[k]][1] for k in range(3)])
                sgn = np.array([1.0 if s == 0 else -1.0 for s in sel])
                m = min(m, float(gs.cr_margin(q + x, v + sgn * ve, np.zeros((1, 3)), b, *box, v_err=ve)[0]))
            deficit.append(m)
    d = np.array(deficit) if deficit else np.zeros(1)
    caught = bad_on > 0
    return CheckResult("P2R2m mutation: the braking lookahead (braking from the true worst-case state after dt, "
                       "re-certified at the next step with the fresh measurement) leaves next states where no braking is "
                       "certified onboard although braking is physically safe: the counterexample of that recursive "
                       "argument, kept as a witness", "expect returned commands with an onboard braking margin <= 0 at a "
                       "next-state corner", ENC, "counterexample", verdict="counterexample" if caught else "none",
                       passed=caught, seconds=round(time.time() - t0, 1),
                       note=f"{n} commands returned by the planners with the robust margin and the hold_margin of the braking lookahead; "
                            f"{bad_on} with a next-state corner whose onboard braking (velocity measured within v_err and "
                            f"filtered with it) is not certified, {bad_phys} "
                            f"physically unsafe; onboard deficit min {d.min():.3f} m, median {np.median(d):.3f} m.  Root cause "
                            f"(R-a): the braking lookahead took the next velocity as the true worst case vp, the onboard "
                            f"re-certification uses up to vp + 2 v_err and the next estimate moves by up to rho dt more")


class _BrakingLookaheadGC(gs.GateConstraint):
    """GateConstraint with the robust margin and hold_margin of the braking lookahead (the mutation of P2R2m)."""

    @classmethod
    def make(cls, cfg, q, v0, ve):
        import check_gate_safety as C

        lo, hi = C._box(cfg)
        return cls(C.GATE, np.asarray(q, float), np.asarray(v0, float), gs.P2Bounds.from_cfg(cfg), lo, hi, v_err=float(ve))

    def margins_world(self, V_world, robust=True):
        V = np.atleast_2d(np.asarray(V_world, float))
        a = np.asarray(self.gate.axis, float)
        left = np.array([-a[1], a[0], 0.0])
        Vg = np.stack([V @ a, V @ left, V[:, 2]], axis=1)
        b, ve = self.bounds, self.v_err
        m = gs.cr_margin(self.q, self.v0, Vg, b, self.cr_lo, self.cr_hi, v_err=ve)
        if not robust:
            return m
        U0 = np.broadcast_to(self.v0, Vg.shape)
        xp, vp, _, _ = gs._after(U0 + ve, Vg, b)
        _, _, xm, vm = gs._after(U0 - ve, Vg, b)
        rp = gs.axis_reach(vp, np.zeros_like(vp), b)[..., 0]
        rm = gs.axis_reach(-vm, np.zeros_like(vm), b)[..., 1]
        err = np.array([b.e_h, b.e_h, b.e_z]) + b.slack
        m2 = gs.box_gap(self.q - xm - rm - err, self.q + xp + rp + err, self.cr_lo, self.cr_hi)[:, :2].max(axis=1)
        return np.minimum(m, m2)

    def hold_margin(self, anchor, amp=0.0):
        b = self.bounds
        a = np.asarray(anchor, float)
        r = np.stack([gs.axis_reach(self.v0 + self.v_err, np.zeros(3), b, hold_error=False)[:, 0],
                      gs.axis_reach(self.v0 - self.v_err, np.zeros(3), b, hold_error=False)[:, 1]], axis=-1)
        err = np.array([b.e_h, b.e_h, b.e_z]) + b.slack
        lat = b.w * b.tau_c
        hi = np.maximum(self.q + r[:, 0], a + b.e_track + lat) + err
        lo = np.minimum(self.q - r[:, 1], a - b.e_track - lat) - err
        return float(gs.box_gap(lo, hi, self.cr_lo, self.cr_hi)[:2].max())


# ---------------------------------------------------------------------------------------------------------------- P2R5
def _d_base(cfg: FleetConfig, fresh: bool) -> float:
    """Smallest column separation at which station keeping on the estimate is certified for every measured speed per axis
    up to v_bound (the closed form of sk_band at the largest velocity: P2R1 monotonicity)."""
    from holo_fleet.perception.nav import NavBound

    b = gs.P2Bounds.from_cfg(cfg)
    ve, rho = _ve(cfg, fresh), _rho(cfg, fresh)
    a0 = (float(NavBound(cfg).v_bound) if fresh else 0.0) + ve
    T = max(b.tau_c, a0 / b.a_brake)
    X = a0 * a0 / (2 * b.a_brake) + b.w * T
    return b.e_h + b.slack + max(X + rho * T, b.e_track + b.w * b.tau_c)


def base_case(cfg: FleetConfig) -> CheckResult:
    from holo_fleet.perception.nav import NavBound

    P = _P(cfg)
    b = P.b
    Vb = float(NavBound(cfg).v_bound)
    ve, rho = _ve(cfg), _rho(cfg)
    d_base = round(_d_base(cfg, True) + 1e-3, 4)
    d_out = _d_base(cfg, False)
    q, vm, clo = z3.Reals("q vm cr_lo")
    Xz, Tz = zX(vm + exact(ve), P)
    hi = q + _zmax(Xz + exact(rho) * Tz, P.H)
    cons = [q <= clo - exact(d_base), vm <= exact(Vb), vm >= -exact(Vb), hi + P.E + P.sl >= clo]
    G, env = cfg.gate, cfg.env
    vz = (env.z_max - env.z_min) - 2 * G.cr_half_height
    d3 = gs.ACTIVE_RANGE_M - (Vb + rho) * b.dt
    per_axis = math.sqrt(max(d3 * d3 - vz * vz, 0.0)) / math.sqrt(2.0)
    return check(f"P2R5 base case: station keeping on the estimate is certified at every state whose column separation along "
                 f"s or l is at least D_base = {d_base:.3f} m, for every measured speed per axis up to v_bound {Vb:.2f} m/s "
                 f"(fresh DVL)", "q <= cr_lo - D_base & |vm| <= v_bound -> hi_SK(q, vm) + E + slack < cr_lo", ENC, cons,
                 "unsat",
                 note=f"v_err {ve:.3f} m/s, rho {rho:.3f} m/s.  Activation (reported): the constraint becomes active within "
                      f"ACTIVE_RANGE_M = {gs.ACTIVE_RANGE_M} m of the CR box (3-D) and was inactive one step earlier, so at "
                      f"activation the distance is >= {d3:.2f} m; the depth envelope of the arena ({env.z_min} to "
                      f"{env.z_max} m) bounds the vertical offset from the CR box by {vz:.1f} m, so the larger of the s / l "
                      f"separations is >= {per_axis:.2f} m ({'>=' if per_axis >= d_base else '<'} D_base): the base case "
                      f"{'holds' if per_axis >= d_base else 'does NOT hold'} on entering the range.  During a DVL outage "
                      f"(v_err = v_bound) D_base = {d_out:.2f} m ({'covered' if per_axis >= d_out else 'not covered'} at "
                      f"activation).  A gate that becomes the current one while the drone is already inside the range "
                      f"(consecutive gates nearer than the range) is outside this base case")


def outage_limit(cfg: FleetConfig) -> CheckResult:
    """P2R4l: the premises of the induction step that the controller cannot keep: a DVL outage (or a position correction)
    between a command and the next step."""
    env = cfg.env
    b = gs.P2Bounds.from_cfg(cfg)
    ve, rho = _ve(cfg), _rho(cfg)
    ve_o, rho_o = _ve(cfg, False), _rho(cfg, False)
    q, v0, u = np.array([cfg.gate.cr_s_min - 2.0, 0.0, 0.0]), np.zeros(3), np.array([0.5, 0.0, 0.0])
    gc = _gc(cfg, q, v0, ve, rho, t=1.0)
    reserved = _gc(cfg, q, v0, ve, rho_o, t=1.0).certify(u, (), (), None)
    cert = reserved or gc.certify(u, (), (), None)
    import check_gate_safety as C

    x, v = C._corners_after(v0, u, b, ve)[0][0]
    q1 = q + np.array([x + rho * b.dt, 0.0, 0.0])
    out = _gc(cfg, q1, np.zeros(3), ve_o, rho_o, t=1.0 + b.dt)
    fresh = _gc(cfg, q1, np.array([v + ve, 0.0, 0.0]), ve, rho, t=1.0 + b.dt)
    f_out = out.filter_mission(np.zeros(3), env.v_max_nominal, cert=cert)
    f_corr = fresh.filter_mission(np.zeros(3), env.v_max_nominal, cert=None)
    f_ok = fresh.filter_mission(np.zeros(3), env.v_max_nominal, cert=cert)
    ce = cert is not None and f_out.status == gs.RECOVERY and f_corr.status == gs.RECOVERY and f_ok.status == gs.FEASIBLE
    return CheckResult("P2R4l declared limit: the induction step needs its premises at the next step; a DVL outage that begins "
                       "before a handed certificate is used (v_err and rho raised to the outage bounds) or a position "
                       "correction (the certificates are dropped) can leave no certified action: the controller then logs "
                       "P2_UNCERTIFIED and retreats (P2_RECOVERY)", "expect a state with a NEXT certificate whose next step is "
                       "uncertified in an outage and after a correction, and certified otherwise", ENC, "counterexample",
                       verdict="counterexample" if ce else "none", passed=ce,
                       counterexample={"q": q.tolist(), "v0": v0.tolist(), "u": u.tolist(),
                                       "handed_margin": None if cert is None else round(cert.margin, 3),
                                       "q1": np.round(q1, 3).tolist(), "vm1": round(float(v + ve), 3),
                                       "next_outage": f_out.status, "next_after_correction": f_corr.status,
                                       "next_nominal": f_ok.status},
                       note=f"fresh v_err {ve:.3f} / rho {rho:.3f} m/s, outage {ve_o:.3f} / {rho_o:.3f} m/s; robust command "
                            f"(0.5 m/s along the axis, 2.0 m before the CR entry plane), worst next state: with the handed "
                            f"certificate station keeping is certified ({f_ok.status}), without it (correction) or with the "
                            f"outage bounds the filter retreats ({f_corr.status}, {f_out.status}).  Not resolved: the next "
                            f"measurement regime cannot be anticipated when the stronger reserve is unavailable for the applied command; "
                            f"the test preserves the counterexample at the certificate boundary")



def _zmin(a, b):
    return z3.If(a <= b, a, b)


def _zmax2(a, b):
    return z3.If(a >= b, a, b)


def outage_hull_lemmas(cfg: FleetConfig) -> list:
    """P2R6, P2R2o, P2R6m, P2R6m2, P2R6n: the DVL outage hull (gate_safety.outage_hull), one gate-frame axis."""
    from holo_fleet.perception.nav import NavBound

    nb = NavBound(cfg)
    b = gs.P2Bounds.from_cfg(cfg)
    kr, w, sc = exact(float(nb.k_rw)), exact(float(b.w)), exact(float(nb.k_lin) * float(nb.v_bound))
    out = []
    vm, vt0, d0, e, vdr, lo, up, c, alpha, ep, d = z3.Reals("vm vt0 d0 e vdr lo up c alpha ep d")
    # base: the response part e0 = vt0 - d0 and the held sample vm lie in the initial hull
    base = [z3.Abs(vt0 - vm) <= kr, z3.Abs(d0) <= w, e == vt0 - d0, vdr == vm, lo == vm - kr - w, up == vm + kr + w,
            z3.Not(z3.And(lo <= e, e <= up, lo <= vdr, vdr <= up))]
    out.append(check("P2R6 base: the response part of the true velocity and the held sample start inside the outage hull",
                     "|vt - vm| <= k_rw, |d| <= w -> vm - k_rw - w <= vt - d <= vm + k_rw + w", ENC, base))
    lo2, up2, vdr2 = z3.Reals("lo2 up2 vdr2")
    inv = z3.And(lo <= e, e <= up, lo <= vdr, vdr <= up)
    step = [inv, lo2 == _zmin(lo, c), up2 == _zmax2(up, c), ep >= _zmin(e, c), ep <= _zmax2(e, c),
            alpha >= 0, alpha <= 1, vdr2 == vdr + alpha * (c - vdr),
            z3.Not(z3.And(lo2 <= ep, ep <= up2, lo2 <= vdr2, vdr2 <= up2))]
    out.append(check("P2R6 step: the hull with the new command is invariant (response envelope, dead-reckoning blend or held "
                     "sample)", "inv(lo, up) and e' in [min(e, c), max(e, c)] and vdr' = vdr + a (c - vdr) -> inv(min(lo, c), "
                     "max(up, c))", ENC, step))
    rho_dep = _zmax2(up - vdr, vdr - lo) + w + sc
    prop = [inv, z3.Abs(d) <= w,
            z3.Not(z3.And(e + d >= lo - w, e + d <= up + w, z3.Abs(vdr - (e + d)) <= rho_dep))]
    out.append(check("P2R6 property: the true velocity is in [lo - w, up + w] and the dead reckoning drifts from it by at "
                     "most the deployed outage rho", "inv and |d| <= w -> lo - w <= e + d <= up + w and |vdr - (e + d)| <= "
                     "max(up - vdr, vdr - lo) + w + scale", ENC, prop))
    res = [lo <= vdr, vdr <= up, z3.Not(rho_dep <= (up - lo) + w + sc)]
    out.append(check("P2R2o the reserve handed before an onset (hull width + w + scale) dominates the outage rho for every "
                     "dead-reckoning velocity inside the hull", "lo <= vdr <= up -> max(up - vdr, vdr - lo) <= up - lo", ENC,
                     res))
    m1 = [z3.Abs(vt0 - vm) <= kr, z3.Abs(d0) <= w, e == vt0 - d0, lo == vm - kr, up == vm + kr,
          z3.Not(z3.And(lo <= e, e <= up))]
    out.append(check("P2R6m mutation: the measured side with w instead of 2 w (a residual drift that reverses after the last "
                     "sample) leaves the hull", "expect counterexample", ENC, m1, expect="sat"))
    m2 = [inv, ep >= _zmin(e, c), ep <= _zmax2(e, c), z3.Not(z3.And(lo <= ep, ep <= up))]
    out.append(check("P2R6m2 mutation: a hull without the commands applied since the measurement leaves the true response",
                     "expect counterexample", ENC, m2, expect="sat"))
    rng = np.random.default_rng(13)
    bad, n = [], 0
    kr_f, w_f, sc_f = float(nb.k_rw), float(b.w), float(nb.k_lin) * float(nb.v_bound)
    for _ in range(400):
        vmv = rng.uniform(-0.7, 0.7, 3)
        U = rng.uniform(-0.7, 0.7, (int(rng.integers(0, 6)), 3))
        n += 1
        upv = np.maximum(vmv + kr_f + w_f, U.max(axis=0)) if len(U) else vmv + kr_f + w_f
        lov = np.minimum(vmv - kr_f - w_f, U.min(axis=0)) if len(U) else vmv - kr_f - w_f
        vd = lov + rng.uniform(0.0, 1.0, 3) * (upv - lov)  # inside the hull (P2R6)
        cand = np.vstack([vd[None, :], U]) if len(U) else vd[None, :]
        rho_cf = float(np.max(np.maximum(upv - cand.min(axis=0), cand.max(axis=0) - lov)[:2])) + w_f + sc_f
        err_cf = float(np.max(0.5 * (upv - lov)[:2])) + w_f
        res_cf = float(np.max((upv - lov)[:2])) + w_f + sc_f
        v0, ve, r = gs.outage_hull(vmv, list(U), kr_f, w_f, sc_f, v_dr=vd)
        _v0, _ve, rr = gs.outage_hull(vmv, list(U), kr_f, w_f, sc_f)
        if (not np.allclose(v0, 0.5 * (upv + lov)) or abs(ve - err_cf) > 1e-12 or abs(r - rho_cf) > 1e-12
                or abs(rr - res_cf) > 1e-12 or r > rr + 1e-12):
            bad.append((vmv.round(3).tolist(), len(U)))
    out.append(CheckResult(f"P2R6n deployed gate_safety.outage_hull equals the closed form and its reserve dominates the "
                           f"onset rho ({n} random states)", "outage_hull(x) == closed form(x); rho(v_dr) <= reserve",
                           "numeric", "holds", verdict="holds" if not bad else "violated", passed=not bad,
                           note="" if not bad else str(bad[:3])))
    return out

def run_checks(cfg: FleetConfig):
    out = []
    out += reach_lemmas(cfg)
    out += lookahead_lemma(cfg)
    out += loop_lemmas(cfg)
    out.append(base_case(cfg))
    out.append(theorem(cfg))
    out += outage_hull_lemmas(cfg)
    out.append(outage_limit(cfg))
    out.append(availability(cfg))
    out.append(braking_lookahead_mutation(cfg))
    return out
