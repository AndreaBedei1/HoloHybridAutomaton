"""Gate-aware joint safety: P2 as a constraint on the final command (control/gate_safety.py) and its feasibility with P1.

Every number comes from holo_fleet.config; the deployed code is evaluated, nothing is copied.  A synthetic gate (axis +x,
centre at -5 m depth) is used where the arena map is not needed: the constraint is defined in the gate frame.

P2B1  (proved, Z3 LRA) per-step soundness of the P2 test: with the navigation error inside (e_h, e_h, e_z) and the true
      displacement of the control interval and of the backup inside the per-axis reach [-R-, R+], a positive cr_margin puts
      the true centre outside the CR at every instant (the command of a step is applied for dt <= tau_c, the first phase of
      its own backup, so the trajectory between two decisions is covered by the backup of the earlier one).  The recursive
      argument uses it for the control interval only: the terminal hold of the backup is stated around the TRUE rest point,
      which a loop on the estimate does not realise; what follows is certified by station keeping (P2R).
P2B1r (numeric) the deployed axis_reach is not below the exact maximum of the continuous envelope integral (piecewise
      quadratic, evaluated in closed form on a dense grid of current velocity and command); with the analytic bound of the
      trapezoidal and sampling error of REACH_STEPS points it stays inside SLACK_M.
P2B2  (numeric) of the ONE-STEP margin (cr_margin without lookahead): if a command is P2-safe with one-step margin above
      Delta, braking is P2-safe at the next step, Delta = max over the grid of the displacement over dt + the reach of
      braking from the velocity reached - the reach of the command + the change of the navigation error over one step
      (NavBound coefficients).  The stage-2 hold target keeps hold_err_m of margin while holding, above Delta of a hold.
P2B2s (scope) strict invariance of the one-step P2-safe set fails: the latency phase and its drift are applied again at every
      step, so a command with a one-step margin below Delta may leave no P2-safe command at the next step.  This is why the
      deployed selection does not use the one-step margin: it uses the ROBUST margin (P2B5) and the anchored hold (P2B4).
P2B3  (numeric) adversarial trajectories: states near the CR, current velocities and the planner's candidate commands; for
      every command the deployed filter calls P2-safe, extreme and random true velocity profiles inside the declared
      response envelope, drift +-w_drift_max, the navigation error at the corners and the hold error at rest: the true
      centre never enters the CR.
P2B4  (numeric) station keeping (GateConstraint.hold_margin on a latched anchor): over every state the position loop can
      produce (deviation from the anchor within e_track on each axis, the velocity of the loop at that deviation - gain 0.5
      or 0.6, capped at HOLD_V_MAX per axis - plus the drift), the margin equals that of the anchor itself (time-invariant:
      no fresh drift accumulates), and it is positive at the stage-2 target; with the measured velocity within v_err the
      RECOMPUTED margin can drop (R-c), hence the latched certificate (formal/check_p2_recursive.py).
P2B5  (numeric, grid) the robust margin: every command the deployed warning filter, escape (with the station-keeping HOLD) or
      filter_mission returns with a positive robust margin leaves station keeping on the next estimate P2-safe from every
      worst-case state after dt (position and velocity at the corners of an independently computed envelope, the next
      estimate at +-rho dt, the true velocity as the initial state of the braking).
P2R*  (formal/check_p2_recursive.py) the recursive argument: closed-form braking (P2R1), the onboard-consistent lookahead
      (P2R2), the station-keeping band (P2R3), the induction over the certificates (P2R4), the base case (P2R5); the
      braking lookahead (from the true worst-case state, re-certified at the next step) as a mutation (P2R2m).
P2B6  (numeric) the CR column: no command called P2-safe lets a true trajectory inside the declared envelope pass under or
      over the CR (cr_s_min <= s <= cr_s_max and |l| <= cr_half_width at any depth); P2B6m mutation: with the box (column=False)
      commands called safe pass under or over it.
P2Bm  (mutation) the near hold rule NEAR_HOLD (0.379 m before the CR entry plane: -1.179 m with the former symmetric CR, -1.429 m with
      the present one) with the declared navigation error leaves the true centre in the CR, and the
      filter that ignores the navigation error (e_h = 0) calls safe a state whose truth is inside the CR.
S1g   pairwise radial model (as S1 of formal/check_separation.py) with a gate-constrained drone B that only removes its
      closing component (c_B' <= max(c_B - a_brake dt, 0)) and a drone C that reacts as in S1 (c_C' <= max(c_C - a_brake dt,
      -v_open)), each reacting on its own conservative distance, drift +-w_rel_max: S1ga BMC, S1gb k-induction of
      G(d >= d_safe) from d_hat >= d_warning.  S1gm (declared limit): two drones that can only stop (both constrained) close
      by the drift: the jointly infeasible pair.
J1    (numeric, exhaustive over the sector patterns, one and two threats, x positions around the stage-2 hold x current
      velocities) the deployed P2-aware warning filter and escape choose a P2-safe command whenever one exists, the full P1
      share whenever a P2-safe candidate has it, a non-closing command whenever a P2-safe one is non-closing, and report the
      class of an independent reference on the same candidates (also gate_safety.classify when the threats ask the same
      opening).  J1m mutation: the planner without p2 is caught.
J2    (reported) the class of every single-threat pattern at the stage-2 hold.
Q1    (numeric) queue geometry, abreast n = 2, 3, 4 and the stacked templates stack_square (gate_four) and stack_pair
      (mutex_deadlock_resolution) as mission.queue_line / queue_assignment lay them out: centre distance from every point of
      the stage-2 hold (within hold_lateral_allow of the axis, at every acquisition offset of flows.ACQ_OFFSETS) to every
      queue point >= merge_clearance, the deployed conservative distance >= d_warning both ways.  Q1m mutation: the queue
      line of the NEAR_HOLD rule for n = 2.
M4h   (reported, formal/check_stage2.py) observability of the CR from the new stage-2 hold.

Premises (declared, not proved here): the true velocity of each gate-frame axis stays inside the response envelope of
gate_safety.axis_reach (an increase of the command at most at once, a decrease at no less than a_brake, braking to rest at
a_brake, no overshoot of the command), the residual drift is at most w_drift_max per axis, the hold error at rest is at most
w_drift_max / k_hold from the time the envelope of that side is at rest (a first-order position loop against the drift: the
drift during the residual braking of a motion away from that side is rejected by it), the navigation error at most e_h = nav_err_m
horizontally per axis and e_z (gate_safety.P2Bounds) vertically, and changes by at most the NavBound rate over one step.
"""

from __future__ import annotations

import itertools
import math
import sys
import time
from pathlib import Path

import numpy as np
import z3

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CheckResult, Report, check, exact  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.control import gate_safety as gs  # noqa: E402
from holo_fleet.control.escape import EscapePlanner  # noqa: E402
from holo_fleet.mission import GateSpec  # noqa: E402
from holo_fleet.perception.sonar_geometry import MOUNTS, R_IN, centre_distance_lower, pattern_of_point  # noqa: E402

ENC = "formal/check_gate_safety.py"
V_CAP = 0.6
GATE = GateSpec("SYNTH", np.array([0.0, 0.0, -5.0]), np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]), 2.0, 2.0)


def _box(cfg):
    """Corners (lo, hi) of the CR box in the gate frame (GateRule.cr_lo / cr_hi)."""
    return cfg.gate.cr_lo(), cfg.gate.cr_hi()


def _in_box(P, box, axes=3):
    """Rows of P (k, 3) inside the CR box on the first ``axes`` axes (2: the column footprint)."""
    lo, hi = box
    return np.all((P[:, :axes] >= lo[:axes]) & (P[:, :axes] <= hi[:axes]), axis=1)


def _gc(cfg, q, v0=np.zeros(3), e_h=None, v_err=0.0):
    lo, hi = _box(cfg)
    return gs.GateConstraint(GATE, np.asarray(q, float), np.asarray(v0, float), gs.P2Bounds.from_cfg(cfg, e_h), lo, hi,
                             v_err=float(v_err))


def _v_err_fresh(cfg):
    from holo_fleet.perception.nav import NavBound

    return float(NavBound(cfg).k_rw)


def envelope_pieces(a0: float, a1: float, b: gs.P2Bounds):
    """Pieces (t0, t1, f0, slope) of f = upper velocity envelope + drift on the + side (gate_safety.axis_reach), exact."""
    tau, a, w = b.tau_c, b.a_brake, b.w
    out = []
    if a1 >= a0:
        out.append((0.0, tau, a1 + w, 0.0))
        ua = a1
    else:
        t1 = (a0 - a1) / a
        if t1 >= tau:
            out.append((0.0, tau, a0 + w, -a))
        else:
            out += [(0.0, t1, a0 + w, -a), (t1, tau, a1 + w, 0.0)]
        ua = max(a1, a0 - a * tau)
    if ua > 0.0:
        out.append((tau, tau + ua / a, ua + w, -a))
    return out


def exact_reach(a0: float, a1: float, b: gs.P2Bounds) -> float:
    """Exact running maximum of the integral of the envelope pieces, and the hold error at rest."""
    x, best = 0.0, 0.0
    for t0, t1, f0, sl in envelope_pieces(a0, a1, b):
        L = t1 - t0
        if sl != 0.0:
            ts = -f0 / sl
            if 0.0 < ts < L:
                best = max(best, x + f0 * ts + 0.5 * sl * ts * ts)
        x = x + f0 * L + 0.5 * sl * L * L
        best = max(best, x)
    return max(best, x + b.e_track)


def upper_at(a0: float, a1: float, b: gs.P2Bounds, t: float):
    """Upper envelope velocity (without drift) and the integral of envelope + drift up to t (t <= the whole backup)."""
    x, v = 0.0, None
    for t0, t1, f0, sl in envelope_pieces(a0, a1, b):
        if t >= t0:
            L = min(t, t1) - t0
            x += f0 * L + 0.5 * sl * L * L
            v = f0 + sl * L - b.w
    return (0.0 if v is None else max(v, 0.0) if t > b.tau_c else v), x


# ---------------------------------------------------------------------------------------------------------------- P2B1
def separation_lemma(cfg: FleetConfig) -> CheckResult:
    b = gs.P2Bounds.from_cfg(cfg)
    blo, bhi = _box(cfg)
    lo, hi = [exact(x) for x in blo], [exact(x) for x in bhi]
    err = [exact(b.e_h), exact(b.e_h), exact(b.e_z)]
    sl = exact(b.slack)
    q = [z3.Real(f"q{k}") for k in range(3)]
    e = [z3.Real(f"e{k}") for k in range(3)]
    x = [z3.Real(f"x{k}") for k in range(3)]
    rp = [z3.Real(f"rp{k}") for k in range(3)]
    rm = [z3.Real(f"rm{k}") for k in range(3)]
    cons = []
    for k in range(3):
        cons += [e[k] <= err[k], e[k] >= -err[k], rp[k] >= 0, rm[k] >= 0, x[k] <= rp[k], x[k] >= -rm[k]]
    sep = z3.Or([z3.Or(q[k] + rp[k] + err[k] + sl < lo[k], q[k] - rm[k] - err[k] - sl > hi[k]) for k in range(3)])
    inside = z3.And([z3.And(q[k] + e[k] + x[k] <= hi[k], q[k] + e[k] + x[k] >= lo[k]) for k in range(3)])
    r = check("P2B1 per-step soundness of the P2 test: a positive cr_margin, the navigation error inside (e_h, e_h, e_z) and "
              "the true displacement inside the per-axis reach keep the true centre out of the CR",
              "sep_k(q, R, err, slack) & |e| <= err & -R- <= x <= R+ -> not inside_CR(q + e + x)", ENC, cons + [sep, inside],
              "unsat", note=f"e_h {b.e_h} m, e_z {b.e_z} m, slack {b.slack} m; the command of a step is applied for dt = "
                            f"{cfg.env.dt} s <= tau_c = {b.tau_c} s, inside the first phase of its own backup")
    return r


def reach_numeric(cfg: FleetConfig) -> CheckResult:
    b = gs.P2Bounds.from_cfg(cfg)
    t0 = time.time()
    g = np.round(np.arange(-V_CAP, V_CAP + 1e-9, 0.01), 3)
    U0, U = np.meshgrid(g, g, indexing="ij")
    dep = gs.axis_reach(U0, U, b)
    worst, arg = 0.0, None
    for i in range(U0.shape[0]):
        for j in range(U0.shape[1]):
            for k, sgn in enumerate((1.0, -1.0)):
                ex = exact_reach(sgn * U0[i, j], sgn * U[i, j], b)
                d = float(dep[i, j, k]) - ex
                if d < worst:
                    worst, arg = d, (float(U0[i, j]), float(U[i, j]), "+-"[k])
    T_max = b.tau_c + V_CAP / b.a_brake
    h = T_max / gs.REACH_STEPS
    err = 4 * b.a_brake * h * h / 8.0 + 1e-12
    ok = -worst <= err + 1e-9 and err < b.slack
    return CheckResult("P2B1r the deployed axis_reach does not under-estimate the exact maximum of the envelope integral "
                       "(numeric, dense grid)", "forall (u0, u) in grid: axis_reach >= exact - err_kinks, err_kinks < SLACK_M",
                       "control/gate_safety.py axis_reach vs closed form", "holds", verdict="holds" if ok else "violated",
                       passed=ok, seconds=round(time.time() - t0, 2),
                       note=f"{U0.size} pairs per side, |u| <= {V_CAP} m/s; largest under-estimate {-worst:.2e} m at {arg}; "
                            f"analytic bound of the trapezoidal/sampling error (4 kinks, a h^2 / 8, h = {h:.4f} s) "
                            f"{err:.2e} m; SLACK_M {b.slack} m")


# ---------------------------------------------------------------------------------------------------------------- P2B2
def recursive(cfg: FleetConfig):
    from holo_fleet.perception.nav import NavBound

    b = gs.P2Bounds.from_cfg(cfg)
    nb = NavBound(cfg)
    dt = cfg.env.dt
    de = (nb.k_lin * V_CAP + nb.k_rw) * dt
    t0 = time.time()
    g = np.round(np.arange(-V_CAP, V_CAP + 1e-9, 0.02), 3)
    worst, arg, strict = -1e9, None, 0
    worst_to, worst_slow = -1e9, -1e9
    for u0 in g:
        for u in g:
            for sgn in (1.0, -1.0):
                a0, a1 = sgn * u0, sgn * u
                r_old = exact_reach(a0, a1, b)
                v_dt, x_dt = upper_at(a0, a1, b, dt)
                lo_v = -upper_at(-a0, -a1, b, dt)[0]
                r_new = max(exact_reach(v, 0.0, b) for v in np.linspace(min(lo_v, v_dt), v_dt, 5))
                d = x_dt + r_new + de - r_old
                if d > worst:
                    worst, arg = d, (float(u0), float(u), "+" if sgn > 0 else "-")
                if a1 >= 0.0 and a0 >= 0.0:
                    worst_to = max(worst_to, d)
                if abs(u0) <= 0.3 and abs(u) <= 0.3:
                    worst_slow = max(worst_slow, d)
                strict += d > 1e-12
    _v, xh = upper_at(0.0, 0.0, b, dt)
    d_hold = xh + exact_reach(0.0, 0.0, b) + de - exact_reach(0.0, 0.0, b)
    hold_margin = gs.hold_s(cfg) - cfg.gate.stage2_hold_s
    ok = hold_margin > d_hold
    r1 = CheckResult(f"P2B2 recursive feasibility of the one-step margin: a command P2-safe with one-step margin above Delta = {worst:.3f} m leaves "
                     f"braking P2-safe at the next step; holding at the stage-2 target (margin {hold_margin:.3f} m) stays "
                     f"P2-safe while the realised drift of a step is within its bound (numeric, grid)",
                     "forall grid (u0, u): x(dt) + R(v(dt), 0) + delta_e - R(u0, u) <= Delta; hold margin > Delta_hold",
                     ENC, "holds", verdict="holds" if ok else "violated", passed=ok, seconds=round(time.time() - t0, 2),
                     note=f"{len(g) ** 2 * 2} cases, |u| <= {V_CAP} m/s, dt {dt} s; worst Delta {worst:.3f} m at (u0, u, side) "
                          f"{arg} (a fast motion away from that side: the envelope lets it stop at once at the next step); worst "
                          f"Delta when neither the velocity nor the command points away {worst_to:.3f} m, with |u0|, |u| <= 0.3 "
                          f"m/s {worst_slow:.3f} m; Delta of a hold {d_hold:.3f} m (drift w dt + change of the navigation "
                          f"error {de:.4f} m per step from the NavBound coefficients).  This describes the ONE-STEP margin "
                          f"(cr_margin without lookahead); the deployed selection uses the robust margin (P2B5) and the "
                          f"anchored station keeping (P2B4)")
    r2 = CheckResult("P2B2s scope: strict invariance of the one-step P2-safe set fails (one-step margin > 0 does not imply a "
                     "P2-safe braking at the next step): the latency phase and its drift are applied again at every step",
                     "expect a grid case with x(dt) + R(v(dt), 0) > R(u0, u)", ENC, "counterexample",
                     verdict="counterexample" if strict else "none", passed=bool(strict),
                     note=f"{strict} of {len(g) ** 2 * 2} cases with a positive deficit, worst {worst:.3f} m at {arg}: a command "
                          f"chosen on its one-step margin below it may leave no P2-safe command at the next step.  The "
                          f"deployed selection therefore uses the robust margin (P2B5: braking stays P2-safe from every "
                          f"worst-case state after dt) and the anchored station keeping (P2B4)")
    return r1, r2


# ---------------------------------------------------------------------------------------------------------------- P2B3
def env_vel(a0: float, a1: float, b: gs.P2Bounds, ts: np.ndarray) -> np.ndarray:
    """Upper velocity envelope (no drift) on the + side at times ts, vectorized (gate_safety.axis_reach)."""
    tau, a = b.tau_c, b.a_brake
    if a1 >= a0:
        va, ua = np.full(ts.shape, a1), a1
    else:
        va, ua = np.maximum(a1, a0 - a * np.minimum(ts, tau)), max(a1, a0 - a * tau)
    vb = np.maximum(ua - a * np.maximum(ts - tau, 0.0), 0.0)
    return np.where(ts <= tau, va, vb)


def side_end(a0: float, a1: float, b: gs.P2Bounds) -> float:
    ua = a1 if a1 >= a0 else max(a1, a0 - b.a_brake * b.tau_c)
    return b.tau_c + max(ua, 0.0) / b.a_brake


def _profile(a0, a1, b, ts, mode, rng):
    """True displacement along one axis at times ts for one admissible profile: 'up' (upper envelope + drift +w, then the
    hold error +e_track), 'down' (the mirror), 'rand' (random inside the envelopes and the drift, random hold error).  Hold
    premise: once the envelope of a side is at rest (its end time), the displacement towards that side stays within e_track
    of where it was (first-order position loop against the drift)."""
    vu = env_vel(a0, a1, b, ts)
    vl = -env_vel(-a0, -a1, b, ts)
    Tu, Tl = side_end(a0, a1, b), side_end(-a0, -a1, b)
    if mode == "up":
        v, w, dev = vu, np.full(len(ts), b.w), b.e_track
    elif mode == "down":
        v, w, dev = vl, np.full(len(ts), -b.w), -b.e_track
    else:
        r = np.repeat(rng.uniform(0.0, 1.0, len(ts) // 10 + 1), 10)[:len(ts)]
        v = vl + (vu - vl) * r
        w = np.repeat(rng.uniform(-b.w, b.w, len(ts) // 10 + 1), 10)[:len(ts)]
        dev = rng.uniform(-b.e_track, b.e_track)
    f = v + w
    x = np.concatenate([[0.0], np.cumsum(0.5 * (f[1:] + f[:-1]) * np.diff(ts))])
    ramp = lambda T: np.clip((ts - T) / 0.5, 0.0, 1.0)
    if mode == "up":
        iu = int(np.searchsorted(ts, Tu))
        x[iu:] = x[max(iu - 1, 0)] + dev * ramp(ts[max(iu - 1, 0)])[iu:]
    elif mode == "down":
        il = int(np.searchsorted(ts, Tl))
        x[il:] = x[max(il - 1, 0)] + dev * ramp(ts[max(il - 1, 0)])[il:]
    else:
        T = max(Tu, Tl)
        i = int(np.searchsorted(ts, T))
        x[i:] = x[max(i - 1, 0)] + dev * ramp(ts[max(i - 1, 0)])[i:]
        iu, il = int(np.searchsorted(ts, Tu)), int(np.searchsorted(ts, Tl))
        x[iu:] = np.minimum(x[iu:], x[max(iu - 1, 0)] + b.e_track)
        x[il:] = np.maximum(x[il:], x[max(il - 1, 0)] - b.e_track)
    return x


def adversarial(cfg: FleetConfig, n_states: int = 1500, seed: int = 7) -> CheckResult:
    b = gs.P2Bounds.from_cfg(cfg)
    box = _box(cfg)
    c0 = cfg.gate.cr_center_s
    pl = EscapePlanner(cfg)
    rng = np.random.default_rng(seed)
    t0 = time.time()
    dirs = pl.cands
    C = np.concatenate([sp * dirs for sp in (0.2, 0.35, cfg.env.v_escape)] + [np.zeros((1, 3))])
    ts = np.arange(0.0, b.tau_c + V_CAP / b.a_brake + 1.2, 0.01)
    tested = viol = 0
    first = None
    for _ in range(n_states):
        q = np.array([c0 + rng.uniform(-3.6, 1.6), rng.uniform(-2.6, 2.6), rng.uniform(-2.6, 2.6)])
        v0 = rng.uniform(-0.5, 0.5, 3) * np.array([1.0, 1.0, 0.5])
        gc = _gc(cfg, q, v0)
        M = gc.margins_world(C)
        idx = np.flatnonzero(M > 0)
        if idx.size == 0:
            continue
        for k in rng.choice(idx, size=min(5, idx.size), replace=False):
            u = C[k]
            for modes in list(itertools.product(("up", "down"), repeat=3)) + [("rand",) * 3, ("rand",) * 3]:
                X = np.stack([_profile(v0[a], u[a], b, ts, modes[a], rng) for a in range(3)], axis=1)
                e = np.array([(b.e_h if a < 2 else b.e_z) * (1.0 if modes[a] == "up" else -1.0 if modes[a] == "down"
                                                           else rng.uniform(-1, 1)) for a in range(3)])
                P = q + e + X
                tested += 1
                bad = _in_box(P, box)
                if bad.any():
                    viol += 1
                    if first is None:
                        first = {"q": np.round(q, 3).tolist(), "v0": np.round(v0, 3).tolist(), "u": np.round(u, 3).tolist(),
                                 "modes": modes, "margin": round(float(M[k]), 4), "t": float(ts[int(np.argmax(bad))])}
    ok = viol == 0 and tested > 0
    return CheckResult("P2B3 adversarial trajectories: no command called P2-safe by the deployed filter lets a true trajectory "
                       "inside the declared envelope into the CR (numeric)",
                       "forall sampled (q, v0, u) with margin > 0 and admissible profiles: true centre not in CR", ENC, "holds",
                       verdict="holds" if ok else "violated", passed=ok, seconds=round(time.time() - t0, 1),
                       counterexample=first,
                       note=f"{tested} trajectories ({n_states} states near the CR, up to 5 P2-safe candidate commands each, "
                            f"8 extreme per-axis combinations at the error corners + 2 random), sampled every 10 ms; "
                            f"{viol} entered the CR")


def mutations(cfg: FleetConfig) -> CheckResult:
    b = gs.P2Bounds.from_cfg(cfg)
    G = cfg.gate
    q_old = np.array([G.cr_s_min - (0.229 + 0.15), 0.0, 0.0])
    truth_old = q_old + np.array([b.e_h, 0.0, 0.0])
    a = bool(G.in_cr(*truth_old))
    m_old = float(_gc(cfg, q_old).margins_world(np.zeros(3))[0])
    q = np.array([G.cr_s_min - 0.5, 0.0, 0.0])
    m_e0 = float(_gc(cfg, q, e_h=0.0).margins_world(np.zeros(3))[0])
    truth = q + np.array([b.e_h, 0.0, 0.0])
    c = m_e0 > 0 and bool(G.in_cr(*truth))
    ok = a and m_old <= 0.0 and c
    return CheckResult("P2Bm mutation: the near hold point (NEAR_HOLD) with the declared navigation error, and the filter without the "
                       "navigation error, let the true centre into the CR",
                       "expect: truth(hold_near) in CR; margin_{e_h=0}(q) > 0 with truth(q) in CR", ENC, "counterexample",
                       verdict="counterexample" if ok else "none", passed=ok,
                       note=f"NEAR_HOLD rule with the present CR s = {q_old[0]:.3f} m: an estimate {b.e_h} m short puts the centre at s = "
                            f"{truth_old[0]:.3f} m (inside: {a}); the deployed filter gives holding there margin "
                            f"{m_old:.3f} m (rejected); with e_h = 0 holding at s = {q[0]} m has margin {m_e0:.3f} m while the "
                            f"truth s = {truth[0]:.2f} m is inside")


# ---------------------------------------------------------------------------------------------------------------- S1g
def _s1g_params(cfg):
    env, sep = cfg.env, cfg.sep
    return dict(dt=exact(env.dt), a=exact(env.a_brake), w=exact(env.w_rel_max), vo=exact(env.v_open), v=exact(env.v_escape),
                csum=exact(env.v_escape) + exact(env.v_max_nominal), dw=exact(sep.d_warning), ds=exact(sep.d_safe))


def _zmax(a, b):
    return z3.If(a >= b, a, b)


def _s1g_vars(k):
    return [[z3.Real(f"{nm}_{i}") for i in range(k + 1)] for nm in ("d", "cb", "cc", "hb", "hc", "w")]


def _s1g_caps(p, cb, cc, w, i):
    return [cb[i] <= p["v"], cb[i] >= -p["v"], cc[i] <= p["v"], cc[i] >= -p["v"], cb[i] + cc[i] <= p["csum"],
            w[i] <= p["w"], w[i] >= -p["w"]]


def _s1g_trans(p, V, i, c_opens):
    d, cb, cc, hb, hc, w = V
    floor = -p["vo"] if c_opens else z3.RealVal(0)
    return z3.And(*_s1g_caps(p, cb, cc, w, i + 1), hb[i] <= d[i], hc[i] <= d[i],
                  z3.Implies(hb[i] < p["dw"], cb[i + 1] <= _zmax(cb[i] - p["a"] * p["dt"], z3.RealVal(0))),
                  z3.Implies(hc[i] < p["dw"], cc[i + 1] <= _zmax(cc[i] - p["a"] * p["dt"], floor)),
                  d[i + 1] == d[i] - ((cb[i] + cc[i] + w[i]) + (cb[i + 1] + cc[i + 1] + w[i + 1])) / 2 * p["dt"])


def s1g_bmc(p, K, c_opens=True):
    V = _s1g_vars(K)
    d, cb, cc, _hb, _hc, w = V
    cons = [d[0] >= p["dw"]] + _s1g_caps(p, cb, cc, w, 0)
    cons += [_s1g_trans(p, V, i, c_opens) for i in range(K)]
    cons.append(z3.Or(*[d[i] < p["ds"] for i in range(K + 1)]))
    return cons


def s1g_step(p, k):
    V = _s1g_vars(k)
    d, cb, cc, _hb, _hc, w = V
    cons = _s1g_caps(p, cb, cc, w, 0)
    for i in range(k):
        cons += [d[i] >= p["ds"], _s1g_trans(p, V, i, True)]
    cons.append(d[k] < p["ds"])
    return cons


def s1g(cfg: FleetConfig, verbose: bool):
    p = _s1g_params(cfg)
    env, sep = cfg.env, cfg.sep
    out = []
    K = 60
    tag = (f"d_warning {sep.d_warning} m, d_safe {sep.d_safe} m, a_brake {env.a_brake} m/s^2 each, v_open {env.v_open}, "
           f"w_rel {env.w_rel_max} m/s, per-drone speed <= v_escape {env.v_escape}, sum <= v_escape + v_max_nominal, dt {env.dt} s")
    out.append(check(f"S1ga BMC: a gate-constrained drone that only stops and a reacting drone keep d >= d_safe for {K * env.dt:.0f} s "
                     "from any start with d_hat >= d_warning", "Init & T_g^K -> G_[0,K] d >= d_safe", ENC, s1g_bmc(p, K),
                     "unsat", timeout_ms=600000, note=tag, fresh_context=True))
    proved = None
    for k in range(1, 41):
        r = check(f"S1gb k-induction step k={k}", "P^k & T_g^k -> P'", ENC, s1g_step(p, k), "unsat", timeout_ms=120000)
        if r.verdict == "unsat":
            proved = k
            r.prop = (f"S1gb k-induction: G(d >= d_safe) for all time with one drone non-closing and the other opening (k={k})")
            r.formula = "P(s_0..s_{k-1}) & T_g -> P(s_k), base case covered by S1ga (K >= k)"
            r.note = tag
            out.append(r)
            break
    if proved is None:
        out.append(CheckResult("S1gb k-induction (no k <= 40 found)", "", ENC, "unsat", verdict="unknown", passed=False))
    Km = 150
    r = check(f"S1gm declared limit (jointly infeasible pair): two drones that can both only stop (each gate-constrained, "
              f"neither opens) close by the drift below d_safe within {Km * env.dt:.0f} s",
              "expect a trace with d < d_safe", ENC, s1g_bmc(p, Km, c_opens=False), "sat", timeout_ms=600000, note=tag,
              fresh_context=True)
    out.append(r)
    return out


# ---------------------------------------------------------------------------------------------------------------- J1, J2
def _ref(pl, threats, V, idx, margins, planner, speed):
    """Reference sets of a decision on the planner's own candidate grid: safe, full share, non-closing, opening."""
    if planner == "SW":
        speeds = [0.15, 0.25, 0.35, None]
        g = np.array([pl.guarantee(P, []) for P, _ in threats])
        need = np.array([req for _, req in threats])
        nc = len(pl.cands)
        sp = np.array([np.linalg.norm(V[i]) for i in range(len(V))])
        op = np.where(idx[None, :] >= 0, g[:, np.maximum(idx, 0)] * sp[None, :], 0.0)
        full = np.all(op >= need[:, None] - 1e-9, axis=0)
        opening = op.min(axis=0)
        del speeds, nc
    else:
        worst = np.array([pl.guarantee(P, []) for P, _ in threats]).min(axis=0)
        sp = np.array([np.linalg.norm(V[i]) for i in range(len(V))])
        opening = np.where(idx >= 0, worst[np.maximum(idx, 0)] * sp, 0.0)
        full = opening >= pl.cfg.env.g_min * speed - 1e-9
    safe = margins > 0.0
    return safe, full, opening >= -1e-9, opening


def _decide(pl, planner, threats, gc, cfg, p2=True, cache=None):
    env = cfg.env
    I3 = np.eye(3)

    def f(V):
        if cache is not None and V.shape[0] == len(cache[1]) and np.array_equal(V, cache[0]):
            return cache[1]
        return gc.margins_body(V, I3)
    f = f if p2 else None
    pl.reset()
    if planner == "SW":
        return pl.warning_velocity(np.zeros(3), threats, [], 0.0, env.v_max_nominal, p2=f)
    occ = {s for P, _ in threats for s in P}
    return pl.escape_velocity(threats, [], 0.0, env.v_escape, mission_body=None, occupied=occ, depth_room=(4.0, 3.5), p2=f)


def joint_exhaustive(cfg: FleetConfig):
    env, G = cfg.env, cfg.gate
    pl = EscapePlanner(cfg)
    pats = sorted(pl.regions, key=lambda P: "+".join(sorted(P)))
    sets = [[(P, env.v_open)] for P in pats] + [[(P, 0.0)] for P in pats]
    sets += [[(P, env.v_open), (Q, env.v_open)] for P, Q in itertools.combinations(pats, 2)]
    sets += [[(P, env.v_open), (Q, 0.0)] for P, Q in itertools.combinations(pats, 2)]
    c0 = G.cr_center_s
    poss = [(s, l, dz) for s in (c0 - 2.6, G.stage2_hold_s, c0 - 1.9) for l in (0.0, 1.2) for dz in (0.0, 1.3)]
    vels = [np.zeros(3), np.array([0.3, 0.0, 0.0])]
    t0 = time.time()
    n = bad = mbad = cls_bad = 0
    first = None
    counts = {s: 0 for s in gs.STATUSES}
    grids = {"SW": pl._grid([0.15, 0.25, 0.35, env.v_max_nominal]), "CA": pl._grid([env.v_escape, 0.35, 0.20])}
    for q in poss:
        for v0 in vels:
            gc = _gc(cfg, q, v0)
            for planner in ("SW", "CA"):
                V, _lab, idx, _sp = grids[planner]
                margins = gc.margins_body(V, np.eye(3))
                safe = margins > 0.0
                for th in sets:
                    if planner == "CA" and any(r != env.v_open for _, r in th):
                        continue
                    uniform = len({r for _, r in th}) == 1
                    _s, full, non, opening = _ref(pl, th, V, idx, margins, planner, env.v_escape)
                    ch = _decide(pl, planner, th, gc, cfg, cache=(V, margins))
                    k = int(np.argmin(np.linalg.norm(V - ch.v_body, axis=1)))
                    m_ch = float(margins[k])
                    ref = (gs.FEASIBLE if (safe & full).any() else gs.UNCERTAIN if (safe & non).any() else gs.INFEASIBLE)
                    st = ch.joint.status
                    counts[st] = counts.get(st, 0) + 1
                    errs = []
                    if safe.any() and m_ch <= 0.0:
                        errs.append("unsafe chosen while a P2-safe command exists")
                    if (safe & full).any() and not (full[k] and st == gs.FEASIBLE):
                        errs.append("full share available but not chosen")
                    if (safe & non).any() and not non[k]:
                        errs.append("non-closing available but a closing command chosen")
                    if st != ref:
                        errs.append(f"class {st} != reference {ref}")
                    if st == gs.FEASIBLE and not full[k]:
                        errs.append("FEASIBLE without the full share")
                    if uniform and planner == "SW" and gs.classify(opening, margins, float(th[0][1])) != ref:
                        cls_bad += 1
                    n += 1
                    if errs:
                        bad += 1
                        if first is None:
                            first = {"threats": [("+".join(sorted(P)), r) for P, r in th], "q": list(q), "v0": v0.tolist(),
                                     "planner": planner, "chosen": ch.label, "status": st, "errors": errs}
                    chm = _decide(pl, planner, th, gc, cfg, p2=False)
                    km = int(np.argmin(np.linalg.norm(V - chm.v_body, axis=1)))
                    if safe.any() and (np.linalg.norm(V[km] - chm.v_body) > 1e-9
                                       and float(gc.margins_body(chm.v_body[None, :], np.eye(3))[0]) <= 0.0
                                       or np.linalg.norm(V[km] - chm.v_body) <= 1e-9 and margins[km] <= 0.0):
                        mbad += 1
    sec = round(time.time() - t0, 1)
    ok = bad == 0 and cls_bad == 0
    r = CheckResult("J1 gate-aware selection (numeric, exhaustive over sector patterns): a P2-safe command whenever one exists, "
                    "the full P1 share whenever a P2-safe candidate has it, non-closing whenever possible, the class of the "
                    "reference", "forall threat sets x positions x velocities x {warning, escape}: properties (i)-(v)", ENC,
                    "holds", verdict="holds" if ok else "violated", passed=ok, seconds=sec, counterexample=first,
                    note=f"{n} decisions ({len(sets)} threat sets: 26 patterns alone and every pair, with opening v_open or 0; "
                         f"{len(poss)} gate-frame positions around the stage-2 hold; 2 current velocities; the escape with opening v_open "
                         f"only); {bad} with an error; "
                         f"gate_safety.classify disagreed with the reference {cls_bad} times (uniform requirements); classes "
                         f"{ {k: v for k, v in counts.items() if v} }")
    rm = CheckResult("J1m mutation: the planner without p2 (the unconstrained selection) is caught choosing a command that is not P2-safe while "
                     "a P2-safe one exists", "expect violations of (i)", ENC, "caught", verdict="caught" if mbad else "missed",
                     passed=mbad > 0, note=f"{mbad} of {n} decisions")
    return r, rm


def hold_table(cfg: FleetConfig) -> CheckResult:
    env, G = cfg.env, cfg.gate
    pl = EscapePlanner(cfg)
    gc = _gc(cfg, (G.stage2_hold_s, 0.0, 0.0))
    rows = {}
    for P in sorted(pl.regions, key=lambda P: "+".join(sorted(P))):
        sw = _decide(pl, "SW", [(P, env.v_open)], gc, cfg)
        ca = _decide(pl, "CA", [(P, env.v_open)], gc, cfg)
        rows["+".join(sorted(P))] = (sw.joint.status, sw.label, ca.joint.status, ca.label)
    by = {}
    for k, (s1, l1, s2, l2) in rows.items():
        by.setdefault(f"SW {s1}", []).append(f"{k}->{l1}")
        by.setdefault(f"CA {s2}", []).append(f"{k}->{l2}")
    note = "; ".join(f"{k} ({len(v)}): {', '.join(v)}" for k, v in sorted(by.items()))
    return CheckResult("J2 reported: joint-feasibility class of every single-threat pattern at the stage-2 hold (warning filter, "
                       "escape)", "table", ENC, "reported", verdict="reported", passed=True,
                       note=f"hold s = {G.stage2_hold_s} m on the axis, at rest, heading along the axis; {note}")


# ---------------------------------------------------------------------------------------------------------------- Q1
def _dlow(pi, pj):
    rel = np.asarray(pj, float) - np.asarray(pi, float)
    pat = pattern_of_point(rel, R_IN)
    if not pat:
        return math.inf
    return max(centre_distance_lower(max(float(np.linalg.norm(rel - MOUNTS[s])) - R_IN, 0.0), s) for s in pat)


def _hold_points(cfg: FleetConfig):
    """Where a drone holds at the stage-2 hold: on the axis within hold_lateral_allow, at the vertical (and lateral)
    acquisition offsets of flows.ACQ_OFFSETS."""
    from holo_fleet.control.flows import ACQ_OFFSETS

    G = cfg.gate
    dls = sorted({0.0, G.hold_lateral_allow, -G.hold_lateral_allow} | {float(dl) for dl, _dz in ACQ_OFFSETS})
    dzs = sorted({0.0} | {float(dz) for _dl, dz in ACQ_OFFSETS})
    return [np.array([G.stage2_hold_s, dl, dz]) for dl in dls for dz in dzs]


def _queue_templates(cfg: FleetConfig, override=None):
    """(name, queue line, [(lateral, dz)]) of the abreast queues n = 2..4 (GateRule.queue_s) and of the stacked templates the
    scenarios fly (mission.queue_line / queue_assignment: stack_square of gate_four, stack_pair of
    mutex_deadlock_resolution)."""
    from holo_fleet.formations import TEMPLATES
    from holo_fleet.mission import queue_assignment, queue_line

    G = cfg.gate
    out = [(f"abreast n={n}", G.queue_s(n), [(lq, 0.0) for lq in G.queue_laterals(n)]) for n in (2, 3, 4)]
    for nm in ("stack_square", "stack_pair"):
        sl = TEMPLATES[nm].slots
        out.append((nm, queue_line(sl, G), list(queue_assignment(sl, G))))
    if override:
        out = [(nm, override.get(nm, qs), pts) for nm, qs, pts in out]
    return out


def _geometry_rows(cfg: FleetConfig, templates):
    G, sep = cfg.gate, cfg.sep
    holds = _hold_points(cfg)
    rows, ok = [], True
    for nm, qs, pts in templates:
        dc_min = dl_min = np.inf
        for lq, dzq in pts:
            qp = np.array([qs, lq, dzq])
            for h in holds:
                dc_min = min(dc_min, float(np.linalg.norm(qp - h)))
                dl_min = min(dl_min, _dlow(h, qp), _dlow(qp, h))
        good = dc_min >= G.merge_clearance - 1e-9 and dl_min >= sep.d_warning
        ok &= good
        rows.append(f"{nm} (line {qs:.3f}, {len(pts)} points): centre min {dc_min:.2f}, d_lower min {dl_min:.2f}"
                    f"{'' if good else ' FAIL'}")
    return ok, rows


def queue_geometry(cfg: FleetConfig, queue_s=None):
    return _geometry_rows(cfg, _queue_templates(cfg, queue_s))


def queue_checks(cfg: FleetConfig):
    G = cfg.gate
    ok, rows = queue_geometry(cfg)
    holds = _hold_points(cfg)
    extra = ""
    try:
        from holo_fleet.mission import queue_assignment, queue_line
        from holo_fleet.sim.scenarios import STACK_AXIS

        _ok, r_axis = _geometry_rows(cfg, [("stack_axis", queue_line(STACK_AXIS.slots, G), list(queue_assignment(STACK_AXIS.slots, G)))])
        extra = (f"; not in the check, reported: {r_axis[0]} (scenario gate_rear_approach: one stacked column on the axis, the "
                 f"lower drone queues behind the drone held before the CR by design, the rear-threat test)")
    except Exception as e:  # report, never hide
        extra = f"; stack_axis not evaluated ({type(e).__name__}: {e})"
    r = CheckResult("Q1 queue geometry: the stage-2 hold (on the axis within hold_lateral_allow, at every acquisition offset) keeps "
                    "merge_clearance (centre) and the warning band (deployed conservative distance, both ways) from every queue "
                    "point of the abreast queues n = 2, 3, 4 and of the stacked templates stack_square and stack_pair (numeric)",
                    "forall templates, queue points q, hold points h: |h - q| >= merge_clearance & d_lower >= d_warning", ENC,
                    "holds", verdict="holds" if ok else "violated", passed=ok,
                    note=f"hold s = {G.stage2_hold_s} m, {len(holds)} hold points (lateral within {G.hold_lateral_allow} m, "
                         f"flows.ACQ_OFFSETS); merge_clearance {G.merge_clearance} m, d_warning {cfg.sep.d_warning} m; "
                         + "; ".join(rows) + extra)
    q_near = -4.181 + (G.cr_s_min + 0.8)  # the queue rule of the NEAR_HOLD hold for n = 2 from the present CR entry plane (-4.181 m with the former CR)
    okm, rowsm = queue_geometry(cfg, queue_s={"abreast n=2": q_near})
    rm = CheckResult(f"Q1m mutation: the NEAR_HOLD queue line for n = 2 ({q_near:.3f} m with the present CR entry plane) puts the queue "
                     "points inside the clearance of the hold",
                     "expect a violation of Q1", ENC, "caught", verdict="caught" if not okm else "missed", passed=not okm,
                     note="; ".join(x for x in rowsm if x.startswith("abreast n=2")))
    return r, rm


# ---------------------------------------------------------------------------------------------------------------- P2B4-6
def station_keeping(cfg: FleetConfig) -> CheckResult:
    b = gs.P2Bounds.from_cfg(cfg)
    ve = _v_err_fresh(cfg)
    t0 = time.time()
    c0 = cfg.gate.cr_center_s
    anchors = [np.array([cfg.gate.stage2_hold_s, 0.0, 0.0]), np.array([c0 - 2.6, 0.9, 0.0]), np.array([c0 - 2.4, 0.0, 1.8]),
               np.array([c0 - 3.0, -1.4, -2.5]), np.array([c0, 2.6, 0.0])]
    devs = np.linspace(-b.e_track, b.e_track, 5)
    drop_true = drop_meas = 0.0
    worst_target, n, n_sat = np.inf, 0, 0
    arg = arg_m = None
    for an in anchors:
        ref_true = float(_gc(cfg, an, np.zeros(3)).hold_margin(an))
        ref_meas = float(_gc(cfg, an, np.zeros(3), v_err=ve).hold_margin(an))
        for e in itertools.product(devs, repeat=3):
            e = np.array(e)
            for gain in (gs.K_HOLD, cfg.gate.k_track):
                cmd = gain * (-e)
                sat = bool(np.any(np.abs(cmd) > gs.HOLD_V_MAX))
                cmd = np.clip(cmd, -gs.HOLD_V_MAX, gs.HOLD_V_MAX)
                for d in itertools.product((-b.w, 0.0, b.w), repeat=3):
                    v_true = cmd + np.array(d)
                    m = float(_gc(cfg, an + e, v_true).hold_margin(an))
                    if ref_true - m > drop_true + 1e-12:
                        drop_true, arg = ref_true - m, (an.tolist(), np.round(e, 3).tolist(), np.round(v_true, 3).tolist())
                    n_sat += sat
                    for me in (-ve, 0.0, ve):
                        mm = float(_gc(cfg, an + e, v_true + me, v_err=ve).hold_margin(an))
                        n += 1
                        if ref_meas - mm > drop_meas + 1e-12:
                            drop_meas, arg_m = ref_meas - mm, (an.tolist(), np.round(e, 3).tolist(), np.round(v_true + me, 3).tolist())
                        if np.allclose(an, anchors[0]):
                            worst_target = min(worst_target, mm)
    box_min = np.inf
    an = anchors[0]
    for e in itertools.product(devs, repeat=3):
        for v in itertools.product(np.linspace(-gs.HOLD_V_MAX, gs.HOLD_V_MAX, 5), repeat=3):
            if np.linalg.norm(v) <= gs.HOLD_V_MAX + 1e-9:
                box_min = min(box_min, float(_gc(cfg, an + np.array(e), np.array(v), v_err=ve).hold_margin(an)))
    ok = drop_true <= 1e-9 and worst_target > 0.0
    return CheckResult("P2B4 station keeping on a latched anchor: over the states of the unsaturated position loop (deviation within "
                       "e_track, the loop velocity at that deviation plus the drift) the margin equals that of the anchor "
                       "(time-invariant), and with the measured velocity within v_err it stays positive at the stage-2 target "
                       "(numeric)", "forall anchors, |e_k| <= e_track, v = k (a - q) + d: hold_margin(q, v) = hold_margin(a, 0); "
                       "with v_err: hold_margin > 0 at the stage-2 target", ENC, "holds", verdict="holds" if ok else "violated",
                       passed=ok, seconds=round(time.time() - t0, 1),
                       note=f"{n} loop states, 5 anchors (stage-2 target, beside, above, below, beyond the gate), gains "
                            f"{gs.K_HOLD} / {cfg.gate.k_track}, cap {gs.HOLD_V_MAX} m/s, drift +-{b.w} per axis; exact invariance "
                            f"with the true velocity: largest drop {drop_true:.2e} m{'' if arg is None else ' at ' + str(arg)}.  "
                            f"With the measured velocity within v_err {ve:.3f} m/s (DVL 3 sigma) the braking term of hold_margin "
                            f"can exceed the anchor term at the edge of the deviation band: largest drop {drop_meas:.3f} m at "
                            f"{arg_m}: the margin RECOMPUTED onboard at a later step is time-invariant only up to that amount "
                            f"(R-c: a fresh measurement inflation), which is why the certificate of station keeping is the one "
                            f"latched when the maneuver is engaged (gate_safety.P2Certificate; P2R3, P2R4): continuing it needs "
                            f"no re-certification; stage-2 target margin min {worst_target:.3f} m.  {n_sat // 27} "
                            f"deviation/gain cases saturate the per-axis cap HOLD_V_MAX (> w: still restoring on every axis, "
                            f"P2R3a; the former 2-norm cap needed the drift vector below HOLD_V_MAX).  Deviation and velocity "
                            f"taken independently (states the loop does not produce): min {box_min:.3f} m")


def _corners_after(u0, u, b, ve):
    """Worst-case (displacement, velocity) after dt on each side of each axis, from the independent envelope (upper_at) with
    the measured velocity error: [(x+, v+), (-x-, -v-)] per axis."""
    out = []
    for k in range(3):
        vp, xp = upper_at(u0[k] + ve, u[k], b, b.dt)
        vm, xm = upper_at(-(u0[k] - ve), -u[k], b, b.dt)
        out.append([(xp, vp), (-xm, -vm)])
    return out


def _braking_ok(cfg, q, u0, u, b, ve, box):
    """Braking one-step P2-safe from every corner of the worst-case state after dt (v_err 0: the corner velocity is the
    true one), and as computed onboard at the next step (measured within v_err, filtered with v_err)."""
    C = _corners_after(u0, u, b, ve)
    phys = onboard = True
    worst = np.inf
    for sel in itertools.product((0, 1), repeat=3):
        x = np.array([C[k][sel[k]][0] for k in range(3)])
        v = np.array([C[k][sel[k]][1] for k in range(3)])
        m = float(gs.cr_margin(q + x, v, np.zeros((1, 3)), b, *box)[0])
        worst = min(worst, m)
        phys &= m > 0.0
        sgn = np.array([1.0 if sel[k] == 0 else -1.0 for k in range(3)])
        m2 = float(gs.cr_margin(q + x, v + sgn * ve, np.zeros((1, 3)), b, *box, v_err=ve)[0])
        onboard &= m2 > 0.0
    return phys, onboard, worst


def returned_commands(cfg: FleetConfig, factory):
    """Commands with a positive robust margin returned by the deployed warning filter, escape (station-keeping HOLD) and
    filter_mission on the grid of P2B5: (q, v0, gc, who, u) with gc = factory(q, v0)."""
    env, G = cfg.env, cfg.gate
    c0 = G.cr_center_s
    pl = EscapePlanner(cfg)
    pats = sorted(pl.regions, key=lambda P: "+".join(sorted(P)))
    sets = [[(P, env.v_open)] for P in pats] + [[(P, env.v_open), (Q, env.v_open)] for P, Q in itertools.combinations(pats, 2)][::9]
    poss = [(sv, lv, dz) for sv in (c0 - 3.2, c0 - 2.7, G.stage2_hold_s, c0 - 2.0) for lv in (0.0, 1.5) for dz in (0.0, 1.6)]
    vels = [np.zeros(3), np.array([0.3, 0.0, 0.0]), np.array([-0.3, 0.0, 0.0]), np.array([0.1, 0.25, 0.1])]
    I3 = np.eye(3)
    mission = [np.array(v, float) for v in ((0.5, 0, 0), (0.25, 0, 0), (0.1, 0, 0), (0.35, 0.35, 0), (0.0, 0.5, 0),
                                              (0.0, 0.0, 0.25), (0.2, 0.0, -0.25), (-0.3, 0.0, 0.0), (0.0, 0.0, 0.0))]
    for q in poss:
        q = np.array(q)
        for v0 in vels:
            gc = factory(q, v0)
            hold = (gc.hold_velocity(q), gc.hold_margin(q))
            outs = []
            for th in sets:
                for planner in ("SW", "CA"):
                    pl.reset()
                    f = lambda V: gc.margins_body(V, I3)
                    if planner == "SW":
                        ch = pl.warning_velocity(np.zeros(3), th, [], 0.0, env.v_max_nominal, p2=f, hold=hold)
                    else:
                        ch = pl.escape_velocity(th, [], 0.0, env.v_escape, occupied={x for P, _ in th for x in P},
                                                depth_room=(4.0, 3.5), p2=f, hold=hold)
                    if ch.label != "HOLD":
                        outs.append((planner, ch.v_body))
            for vm in mission:
                mf = gc.filter_mission(vm, env.v_max_nominal)
                if mf.station is None and mf.status == gs.FEASIBLE:
                    outs.append(("MISSION", mf.v))
            for who, u in outs:
                if float(gc.margins_world(u)[0]) > 0.0:
                    yield q, v0, gc, who, u


def _sk_next_margin(cfg, q, u0, u, b, ve, rho, box):
    """Margin of station keeping engaged one step later on the next estimate, from every corner of the true state after dt
    (independent: the exact pieces of upper_at, the braking to rest integrated by upper_at), the next estimate at +-rho dt
    towards the side of the corner."""
    Cn = _corners_after(u0, u, b, ve)
    H = b.e_track + b.w * b.tau_c
    err = np.array([b.e_h, b.e_h, b.e_z]) + b.slack
    lo_cr, hi_cr = box
    worst = np.inf
    for sel in itertools.product((0, 1), repeat=3):
        hi, lo = np.zeros(3), np.zeros(3)
        for k in range(3):
            x, v = Cn[k][sel[k]]
            sgn = 1.0 if sel[k] == 0 else -1.0
            q1 = q[k] + x + sgn * rho * b.dt
            a0 = sgn * v
            T = side_end(a0, 0.0, b)
            X = upper_at(a0, 0.0, b, T)[1]
            ext = max(X + rho * T, H)
            hi[k], lo[k] = (q1 + ext, q1 - H) if sgn > 0 else (q1 + H, q1 - ext)
        worst = min(worst, float(np.max(np.maximum(lo_cr - (hi + err), (lo - err) - hi_cr)[:2])))
    return worst


def robust_selection(cfg: FleetConfig) -> CheckResult:
    from check_p2_recursive import _gc as gc2, _rho, _ve

    b = gs.P2Bounds.from_cfg(cfg)
    ve, rho = _ve(cfg), _rho(cfg)
    box = _box(cfg)
    t0 = time.time()
    n = bad = 0
    first = None
    worst = np.inf
    for q, v0, gc, who, u in returned_commands(cfg, lambda q, v0: gc2(cfg, q, v0, ve, rho)):
        n += 1
        w = _sk_next_margin(cfg, q, v0, gc.to_gate(u), b, ve, rho, box)
        worst = min(worst, w)
        if w <= 0.0:
            bad += 1
            if first is None:
                first = {"q": q.tolist(), "v0": v0.tolist(), "who": who, "u": np.round(u, 3).tolist(),
                         "worst_next_margin": round(w, 4)}
    ok = bad == 0 and n > 0
    return CheckResult("P2B5 robust selection: every command returned by the deployed warning filter, escape (station-keeping "
                       "HOLD) or filter_mission with a positive robust margin leaves station keeping on the next estimate "
                       "P2-safe from every worst-case state after dt (numeric, grid)", "forall returned u with robust margin "
                       "> 0, corners of the state after dt and of the next estimate: SK margin from the true velocity > 0",
                       ENC, "holds", verdict="holds" if ok else "violated", passed=ok, seconds=round(time.time() - t0, 1),
                       counterexample=first,
                       note=f"{n} returned commands (63 threat sets x 2 planners, 9 mission commands; 16 positions x 4 "
                            f"velocities; HOLD answers are station keeping, P2B4 / P2R3); v_err {ve:.3f} m/s, rho {rho:.3f} "
                            f"m/s; worst next-step SK margin {worst:.3f} m, computed independently of next_band (upper_at, "
                            f"side_end); {bad} failures.  The certificate is handed to the next step, not recomputed there "
                            f"(P2R2, P2R4n); the old target of this check - braking re-certified onboard at the next step - "
                            f"is the counterexample P2R2m")


def column(cfg: FleetConfig, n_states: int = 900, seed: int = 11):
    b = gs.P2Bounds.from_cfg(cfg)
    box = _box(cfg)
    c0 = cfg.gate.cr_center_s
    pl = EscapePlanner(cfg)
    rng = np.random.default_rng(seed)
    t0 = time.time()
    C = np.concatenate([sp * pl.cands for sp in (0.2, 0.35, cfg.env.v_escape)] + [np.zeros((1, 3))])
    ts = np.arange(0.0, b.tau_c + V_CAP / b.a_brake + 1.2, 0.01)
    tested = viol = box_pass = 0
    first = None
    for _ in range(n_states):
        q = np.array([c0 + rng.uniform(-3.4, 3.4), rng.uniform(-2.6, 2.6), rng.choice([-1, 1]) * rng.uniform(1.0, 4.0)])
        v0 = rng.uniform(-0.5, 0.5, 3) * np.array([1.0, 1.0, 0.5])
        gc = _gc(cfg, q, v0)
        M = gc.margins_world(C)
        Mbox = gs.cr_margin(q, v0, C, b, *box, lookahead=True, column=False)
        for k in np.flatnonzero((Mbox > 0) & (M <= 0))[:3]:
            u = C[k]
            X = np.stack([_profile(v0[a], u[a], b, ts, "rand", rng) for a in range(3)], axis=1)
            P = q + X
            box_pass += bool(np.any(_in_box(P, box, axes=2)))
        idx = np.flatnonzero(M > 0)
        for k in rng.choice(idx, size=min(5, idx.size), replace=False) if idx.size else []:
            u = C[k]
            for modes in list(itertools.product(("up", "down"), repeat=2)) + [("rand", "rand")] * 2:
                mz = rng.choice(["up", "down", "rand"])
                X = np.stack([_profile(v0[0], u[0], b, ts, modes[0], rng), _profile(v0[1], u[1], b, ts, modes[1], rng),
                              _profile(v0[2], u[2], b, ts, mz, rng)], axis=1)
                e = np.array([b.e_h * (1.0 if m == "up" else -1.0 if m == "down" else rng.uniform(-1, 1)) for m in modes] +
                             [rng.uniform(-b.e_z, b.e_z)])
                P = q + e + X
                tested += 1
                inside = _in_box(P, box, axes=2)
                if inside.any():
                    viol += 1
                    if first is None:
                        first = {"q": np.round(q, 3).tolist(), "v0": np.round(v0, 3).tolist(), "u": np.round(u, 3).tolist(),
                                 "modes": list(modes) + [mz], "margin": round(float(M[k]), 4)}
    ok = viol == 0 and tested > 0
    r = CheckResult("P2B6 the CR column: no command called P2-safe lets a true trajectory inside the declared envelope pass under or "
                    "over the CR (cr_s_min <= s <= cr_s_max, |l| <= cr_half_width at any depth; numeric)",
                    "forall sampled (q above or below the CR, v0, u) with robust margin > 0: true (s, l) never in the CR footprint",
                    ENC, "holds", verdict="holds" if ok else "violated", passed=ok, seconds=round(time.time() - t0, 1),
                    counterexample=first,
                    note=f"{tested} trajectories from {n_states} states 1-4 m above or below the CR centre (extreme s, l "
                         f"profiles at the error corners and random ones, any vertical profile); {viol} reached the column")
    rm = CheckResult("P2B6m mutation: with the forbidden set the CR box (column=False) commands called safe let the drone pass "
                     "under or over the CR", "expect a sampled trajectory inside the CR footprint", ENC, "caught",
                     verdict="caught" if box_pass else "missed", passed=box_pass > 0,
                     note=f"{box_pass} random trajectories of box-safe, column-unsafe commands crossed the footprint")
    return r, rm


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("gate_safety")
    rep.add(separation_lemma(cfg), verbose)
    rep.add(reach_numeric(cfg), verbose)
    for r in recursive(cfg):
        rep.add(r, verbose)
    rep.add(adversarial(cfg), verbose)
    rep.add(station_keeping(cfg), verbose)
    rep.add(robust_selection(cfg), verbose)
    import check_p2_recursive

    for r in check_p2_recursive.run_checks(cfg):
        rep.add(r, verbose)
    for r in column(cfg):
        rep.add(r, verbose)
    rep.add(mutations(cfg), verbose)
    for r in s1g(cfg, verbose):
        rep.add(r, verbose)
    for r in joint_exhaustive(cfg):
        rep.add(r, verbose)
    rep.add(hold_table(cfg), verbose)
    for r in queue_checks(cfg):
        rep.add(r, verbose)
    import check_stage2

    rep.add(check_stage2.run(cfg, verbose=False).results[-1], verbose)
    return rep


if __name__ == "__main__":
    r = run()
    r.save()
    for x in r.results:
        if x.note:
            print("   ", x.prop.split(" ")[0], x.note[:600])
    print("all as expected" if r.ok else "SOME CHECK NOT AS EXPECTED")
    sys.exit(0 if r.ok else 1)
