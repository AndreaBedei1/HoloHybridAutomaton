"""P3 - eventual formation recovery for the DEPLOYED law: coupled 2-norm cap, private formation clocks, rendezvous,
operating domain.  It complements formal/check_formation.py (F1-F6), whose per-axis model is kept unchanged.

No deadline is imposed: every liveness claim is a ranking function with a positive decrease (reported as a rate, m/s or
m per step); where no ranking is found the gap is recorded as a declared limit with its counterexample.

Deployed law (holo_fleet/control/flows.py Flows.formation, per drone, horizontal plane of the formation frame t, n):
    u = v_ref t + k_slot e + c n,   |c| <= v_corr_max         (sat2 of the lateral sonar correction, along n only)
    v = u min(1, V / |u|)                                     (_cap: ONE 2-norm cap, vertical clipped separately)
    e' = e + dt (v_ref t + sigma_dot t + lambda_dot n - v - w),   |w|_2 <= w_res
with V = v_recovery_max in FORMATION_RECOVERY and V = keeping_cap(v_plan) = min(v_recovery_max, max(v_slot_max,
v_plan + v_catch_min)) in the keeping modes (FOLLOW, WAIT_REJOIN, DEGRADED); v_ref in [v_wait, v_plan] (reduced progress,
B2 sync).  sigma (progress) and lambda (lateral) consensus states move the slot; they are bounded disturbances here
(|sigma_dot| <= S, |lambda_dot| <= L) and feedback states in C2.

C1  per-drone ranking ||e||_2, certified by CELL DECOMPOSITION in exact rational interval arithmetic: (r, theta, v_ref, c)
    cells, theta on rational points of the unit circle (t = tan(theta / 2)), interval bounds of the saturation factor
    min(1, V / |u|) with rational square-root bounds, and the second-order term of the 0.1 s step:
        ||e'|| <= ||e|| - dt eps   with   eps = rho - dt E^2 / (2 r_lo),  rho >= -(e_hat . e_dot),  E >= ||e_dot||.
    Symmetry l -> -l (c, lambda_dot, w mirrored) halves theta to [0, pi].  Far band [e_lost, 8 m] (sigma frozen: Flows
    updates it only nearer than e_lost; lambda driven by lateral_far in B2), near band [r_in, e_lost] with the residual
    consensus drift S = L = s_res (premise A6), and the invariant ball r_in + dt E_in.  C1x: the per-axis along ranking of
    F1 is NOT a ranking of the coupled law (Z3 NRA counterexample): the per-axis model is an abstraction gap, not a proof
    of the deployed law.  C1l: the near band at the transient consensus bound and the B2 far band at the domain edge with
    the adversarial lambda_dot bound have counterexamples (declared limits: sigma, lambda are feedback states, C2).
    C1m: the constant keeping cap v_slot_max = v_plan (integrated_short, 0.40 m/s) has no along catch-up authority (defect,
    fixed by keeping_cap; tests/test_p3_recovery_fixes.py).
C2  pairwise consensus of the private formation clocks (control/formation_time.py, perception.py _rep_offset): state
    D = v (tau_j - tau_i) (the drone behind is j), sigma_i, sigma_j, the slowdown rho_i = 1 - rate_i; X = D + sigma_i -
    sigma_j the effective along offset of the two references, B = band_max + eta.  Ranking max(0, X - B - delta) + (rho_d
    - rho_i)^+ with a positive decrease while the reported-position lag is available (premise: a fresh fix of the pair in
    every step; C2s is the scope check).  C2m: the controller with the sync off in FORMATION_RECOVERY, keeps an
    abreast pair in RECOVERY for ever (counterexample; fixed by behind_rep).
C3  growth of tau: one reduced-progress episode adds at most d_max t_rejoin + v d_max^2 / (2 a_wait) (inductive
    invariant); the rendezvous hold re-synchronises (C3h; without it the difference is kept: C3m).
C4  operating domain (FormationRule.check_clock_speed, raising at plan time): every admitted clock speed leaves the
    catch-up margin v_catch_min in the keeping and recovery modes, above w_res + s_res.

Premises (eventual recovery holds under them, and only under them): A1-A5 of check_formation.py; A6 the consensus drift of
sigma and lambda is eventually below s_res (not proved: C1l); A7 (C2) both drones on or near their slots (|e_j along| +
relative navigation error + extrapolation error beyond the band <= eta = e_ok), a fresh accepted fix of the pair with a
valid payload in every step, no reduced-progress episode and no same-lane following during the convergence; A8 (C1, B2)
|lateral_far| <= LAT_FAR_MAX (the acceptance gate e_lost + band at A_CORR_AGE_S).
"""

from __future__ import annotations

import math
import sys
import time
from fractions import Fraction as Fr
from math import isqrt

import z3

from common import CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.perception import A_CORR_AGE_S

ENC = ("formal/check_formation_coupled.py (Flows.formation 2-norm cap + FormationRule.keeping_cap, FormationTime, "
       "Perception._rep_offset)")
DT = Fr(1, 10)
W_RES = Fr(1, 20)
S_RES = Fr(1, 100)
E_MAX = Fr(8)
SC = 1 << 40


def q(x) -> Fr:
    return Fr(repr(float(x)))


def z(x) -> z3.ArithRef:
    return z3.RealVal(str(Fr(x) if not isinstance(x, Fr) else x))


# ----------------------------------------------------------------------------------------------------------- C1 cells
def _dn(x: Fr) -> Fr:
    return Fr(math.floor(x * SC), SC)


def _up(x: Fr) -> Fr:
    return Fr(math.ceil(x * SC), SC)


def _sqrt_lo(x: Fr) -> Fr:
    return Fr(0) if x <= 0 else Fr(isqrt((x.numerator * SC * SC) // x.denominator), SC)


def _sqrt_hi(x: Fr) -> Fr:
    if x <= 0:
        return Fr(0)
    n = -((-x.numerator * SC * SC) // x.denominator)
    r = isqrt(n)
    return Fr(r + (r * r < n), SC)


def _mul(a, b):
    p = (a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1])
    return _dn(min(p)), _up(max(p))


def _add(a, b):
    return a[0] + b[0], a[1] + b[1]


def _sq(a):
    lo, hi = a
    if lo >= 0:
        return _dn(lo * lo), _up(hi * hi)
    if hi <= 0:
        return _dn(hi * hi), _up(lo * lo)
    return Fr(0), _up(max(lo * lo, hi * hi))


def _circle(t: Fr):
    """Exact rational point of the unit circle, t = tan(theta / 2)."""
    d = 1 + t * t
    return (1 - t * t) / d, 2 * t / d


def _cell(P, r0, r1, quad, t0, t1, v0, v1, c0, c1):
    """(certified decrease eps of ||e|| in m/s on the cell, bound E of ||e_dot||): sound for every point of the cell."""
    ca, sa = _circle(t0)
    cb, sb = _circle(t1)
    cos = (cb, ca) if quad == 0 else (-ca, -cb)
    sin = (sa, sb)
    kr = (P["k"] * r0, P["k"] * r1)
    ux = _add((v0, v1), _mul(kr, cos))
    uy = _add((c0, c1), _mul(kr, sin))
    u2 = _add(_sq(ux), _sq(uy))
    n_lo, n_hi = _sqrt_lo(u2[0]), _sqrt_hi(u2[1])
    V = P["V"]
    f_lo = min(Fr(1), _dn(V / n_hi)) if n_hi > 0 else Fr(1)
    f_hi = Fr(1) if n_lo <= V else _up(V / n_lo)
    A = _mul(cos, (v0, v1))
    B = _add(_mul((c0, c1), sin), kr)
    # e_hat . e_dot = (1 - f) cos v_ref - f (c sin + k r) + cos sigma_dot + sin lambda_dot - e_hat . w
    h = max((1 - f) * A[1] - f * B[0] for f in (f_lo, f_hi))
    h += max(abs(cos[0]), abs(cos[1])) * P["S"] + max(abs(sin[0]), abs(sin[1])) * P["L"] + P["W"]
    fi = (f_lo, f_hi)
    ex = _add(_add((v0, v1), (-P["S"], P["S"])), _mul(fi, (-ux[1], -ux[0])))
    ey = _add((-P["L"], P["L"]), _mul(fi, (-uy[1], -uy[0])))
    E = _sqrt_hi(_up(max(ex[0] ** 2, ex[1] ** 2) + max(ey[0] ** 2, ey[1] ** 2))) + P["W"]
    rho = -h
    eps = rho - DT * E * E / (2 * r0) if (rho > 0 and r0 > 0) else rho
    return eps, E


def _cells(P, lo, hi, n_r=8, n_t=16):
    rs = [lo + (hi - lo) * Fr(i, n_r) for i in range(n_r + 1)]
    return [(0, rs[i], rs[i + 1], quad, Fr(j, n_t), Fr(j + 1, n_t), P["vr"][0], P["vr"][1], -P["C"], P["C"])
            for quad in (0, 1) for i in range(n_r) for j in range(n_t)]


def _split(cell):
    d, r0, r1, quad, a, b, v0, v1, c0, c1 = cell
    ax = d % 4
    if ax == 0:
        m = (r0 + r1) / 2
        return [(d + 1, r0, m, quad, a, b, v0, v1, c0, c1), (d + 1, m, r1, quad, a, b, v0, v1, c0, c1)]
    if ax == 1:
        m = (a + b) / 2
        return [(d + 1, r0, r1, quad, a, m, v0, v1, c0, c1), (d + 1, r0, r1, quad, m, b, v0, v1, c0, c1)]
    if ax == 2:
        m = (v0 + v1) / 2
        return [(d + 1, r0, r1, quad, a, b, v0, m, c0, c1), (d + 1, r0, r1, quad, a, b, m, v1, c0, c1)]
    m = (c0 + c1) / 2
    return [(d + 1, r0, r1, quad, a, b, v0, v1, c0, m), (d + 1, r0, r1, quad, a, b, v0, v1, m, c1)]


def certify(P, lo, hi, eps, depth=16):
    """(True, cells) iff every cell of [lo, hi] x [0, pi] x v_ref x c certifies a decrease >= eps (adaptive bisection);
    else (False, the first cell that cannot be certified at the depth limit)."""
    stack, n = _cells(P, lo, hi), 0
    while stack:
        cell = stack.pop()
        n += 1
        if _cell(P, *cell[1:])[0] >= eps:
            continue
        if cell[0] >= depth:
            return False, cell
        stack += _split(cell)
    return True, n


def best_eps(P, lo, hi, iters=7):
    """Largest decrease (m/s, resolution 0.25 / 2^iters) certified on the band, 0 when none."""
    if not certify(P, lo, hi, Fr(1, 10 ** 6))[0]:
        return Fr(0)
    a, b = Fr(1, 10 ** 6), Fr(1, 4)
    for _ in range(iters):
        m = (a + b) / 2
        a, b = (m, b) if certify(P, lo, hi, m)[0] else (a, m)
    return a


def inner_radius(P, hi, eps=Fr(1, 500), step=Fr(1, 40)):
    """Smallest r_in on the grid ``step`` with a certified decrease >= eps on [r_in, hi], and the bound E_in of ||e_dot||
    on the disk r <= r_in (so the ball r_in + dt E_in is invariant)."""
    r = step * 4
    while r < hi:
        if certify(P, r, hi, eps)[0]:
            E_in = max(_cell(P, *c[1:])[1] for c in _cells(P, Fr(0), r, n_r=8, n_t=32))
            return r, E_in
        r += step
    return None, None


def params(cfg: FleetConfig, V, v_plan, S=Fr(0), L=Fr(0)):
    F = cfg.form
    return {"k": q(F.k_slot), "C": q(F.v_corr_max), "W": W_RES, "V": q(V), "vr": (q(F.v_wait), q(v_plan)), "S": S, "L": L}


def lat_far_max(cfg: FleetConfig, v_plan) -> Fr:
    F = cfg.form
    return q(F.e_lost) + q(F.e_ok) + (q(F.keeping_cap(v_plan)) - q(F.v_wait)) * q(A_CORR_AGE_S)


def c1_checks(cfg: FleetConfig) -> list:
    F = cfg.form
    out = []
    e_lost, v_nom, v_edge = q(F.e_lost), q(F.v_nominal), q(F.v_recovery_max) - q(F.v_catch_min)
    for tag, v_plan in (("nominal clock", v_nom), ("domain edge", v_edge)):
        lam = q(F.k_lateral) * lat_far_max(cfg, v_plan) + q(0.02) * q(F.progress_max)
        for var, L in (("B0", Fr(0)), ("B2", lam)):
            P = params(cfg, F.v_recovery_max, v_plan, L=L)
            t0 = time.time()
            eps = best_eps(P, e_lost, E_MAX)
            if var == "B2" and eps == 0:
                continue  # recorded below as the declared limit C1l2
            out.append(CheckResult(
                f"C1f far band {var}, {tag} v_plan {float(v_plan):.2f}: coupled 2-norm cap {F.v_recovery_max} "
                f"(RECOVERY), ||w||2 <= {float(W_RES)}, |lambda_dot| <= {float(L):.3f}: ||e|| decreases on "
                f"[{float(e_lost)}, {float(E_MAX)}] m", "e_lost <= ||e|| <= 8 -> ||e'|| <= ||e|| - dt eps (all cells)",
                ENC, "unsat", verdict="unsat" if eps > 0 else "sat", passed=eps > 0, seconds=round(time.time() - t0, 1),
                note=f"certified decrease eps = {float(eps):.4f} m/s (exact rational cells, sigma frozen beyond e_lost)"))
    for tag, V, v_plan in (("keeping, nominal clock", F.keeping_cap(F.v_nominal), v_nom),
                           ("keeping, domain edge", F.keeping_cap(float(v_edge)), v_edge),
                           ("RECOVERY, nominal clock", F.v_recovery_max, v_nom),
                           ("RECOVERY, domain edge", F.v_recovery_max, v_edge)):
        P = params(cfg, V, v_plan, S=S_RES, L=S_RES)
        t0 = time.time()
        r_in, E_in = inner_radius(P, e_lost)
        ball = None if r_in is None else r_in + DT * E_in
        ok = ball is not None and ball < q(F.e_ok)
        out.append(CheckResult(
            f"C1n near band, {tag} (cap {float(q(V)):.2f}, v_plan {float(v_plan):.2f}), residual consensus drift "
            f"|sigma_dot|, |lambda_dot| <= {float(S_RES)} (A6), ||w||2 <= {float(W_RES)}: ||e|| decreases on [r_in, e_lost] "
            "and the ball r_in + dt E_in < e_ok is invariant",
            "r_in <= ||e|| <= e_lost -> ||e'|| <= ||e|| - dt eps;  ||e|| <= r_in -> ||e'|| <= r_in + dt E_in", ENC, "unsat",
            verdict="unsat" if ok else "sat", passed=ok, seconds=round(time.time() - t0, 1),
            note=("no inner radius certified" if r_in is None else
                  f"r_in = {float(r_in):.3f} m, ball {float(ball):.3f} m (e_ok = {F.e_ok}), eps >= 0.002 m/s on the band")))
    out += c1_witnesses(cfg, v_edge)
    out.append(vertical_check(cfg))
    return out


def vertical_check(cfg: FleetConfig) -> CheckResult:
    """Vertical channel: e_z' = e_z + dt (-clip(k e_z, -0.3, 0.3) - w_z), |w_z| <= w_res: the ball w_res / k * 1.05 + 0.005
    is invariant and reached (one-dimensional, exact LRA)."""
    k, ez, wz = z(q(cfg.form.k_slot)), z3.Real("e_z"), z3.Real("w_z")
    b = W_RES / q(cfg.form.k_slot) * Fr(21, 20) + Fr(1, 200)
    cl = z3.If(k * ez > z(q(0.3)), z(q(0.3)), z3.If(k * ez < -z(q(0.3)), -z(q(0.3)), k * ez))
    e1 = ez + z(DT) * (-cl - wz)
    a = lambda x: z3.If(x >= 0, x, -x)  # noqa: E731
    return check(f"C1z vertical channel: |e_z| decreases outside and stays inside the ball {float(b):.3f} m",
                 "|e_z| >= b -> |e_z'| < |e_z|;  |e_z| <= b -> |e_z'| <= b", ENC,
                 [wz >= -z(W_RES), wz <= z(W_RES), a(ez) <= 8,
                  z3.Or(z3.And(a(ez) >= z(b), a(e1) >= a(ez)), z3.And(a(ez) <= z(b), a(e1) > z(b)))],
                 note="the 3-D slot error of the keeping modes is within sqrt(ball_h^2 + ball_z^2) (C1n, C1z)")


def _on_axis(V, v_ref_lo, v_ref_hi, S, W, lo, hi):
    """Restriction of the coupled law to e = (e_a, 0), c = 0, lambda_dot = 0, w = (w_a, 0) (an invariant line of the
    model, so a witness here is a witness of the full law): u = (v_ref + k e_a, 0), |u| = v_ref + k e_a."""
    ea, vr, sd, wa = z3.Reals("e_a v_ref sigma_dot w_a")
    ua = vr + z(q(DEFAULT.form.k_slot)) * ea
    va = z3.If(ua > z(V), z(V), ua)
    e1 = ea + z(DT) * (vr + sd - va - wa)
    return [ea >= z(lo), ea <= z(hi), vr >= z(v_ref_lo), vr <= z(v_ref_hi), sd >= -z(S), sd <= z(S),
            wa >= -z(W), wa <= z(W), e1 >= ea]


def c1_witnesses(cfg: FleetConfig, v_edge) -> list:
    F = cfg.form
    out = []
    ea, el, vr, c, wa, wl, n, va, vl = z3.Reals("ea el vr c wa wl n va vl")
    K, V = z(q(F.k_slot)), z(q(F.v_recovery_max))
    ua, ul = vr + K * ea, K * el + c
    cons = [vr >= z(q(F.v_wait)), vr <= z(q(F.v_nominal)), c >= -z(q(F.v_corr_max)), c <= z(q(F.v_corr_max)),
            wa * wa + wl * wl <= z(W_RES * W_RES), n >= 0, n * n == ua * ua + ul * ul,
            z3.Or(z3.And(n <= V, va == ua, vl == ul), z3.And(n > V, va * n == V * ua, vl * n == V * ul)),
            ea >= z(q(F.e_lost)), ea <= z(E_MAX)]
    ea1 = ea + z(DT) * (vr - va - wa)
    out.append(check("C1x scope: the per-axis along ranking |e_a| of F1 (check_formation.py) is not a ranking of the deployed "
                     "coupled 2-norm cap (far band, RECOVERY, nominal clock): the per-axis model is an abstraction gap",
                     "exists e, w: |e_a'| > |e_a| - dt 0.001", ENC,
                     cons + [z3.Or(ea1 > ea - z(DT) * z(Fr(1, 1000)), ea1 < -(ea - z(DT) * z(Fr(1, 1000))))],
                     expect="sat", note="a large lateral error takes the 2-norm budget from the along axis; ||e|| (C1f) "
                                        "is the ranking of the coupled law"))
    sig = q(F.k_progress) * q(F.range_gate_m) + q(0.02) * q(F.progress_max)
    out.append(check(f"C1l1 declared limit: at the transient consensus bound |sigma_dot| <= {float(sig):.3f} (the bound of "
                     "check_formation.py) the keeping-mode near band has no decrease: a drone behind its slot with sigma "
                     "moving its slot ahead stays behind (sigma is a feedback state, C2; premise A6)",
                     "exists e_a in [e_ok, e_lost]: e_a' >= e_a", ENC,
                     _on_axis(q(F.keeping_cap(F.v_nominal)), q(F.v_wait), q(F.v_nominal), sig, W_RES, q(F.e_ok),
                              q(F.e_lost)), expect="sat",
                     note="B0: the sonar residual also contains the own error (sigma_dot ~ +k_progress e), a genuine "
                          "positive feedback limited by the leak and the clip"))
    lam = q(F.k_lateral) * lat_far_max(cfg, v_edge) + q(0.02) * q(F.progress_max)
    vr_e = z(v_edge)
    ua2, ul2 = vr_e + K * ea, K * el + c
    lad = z3.Real("lambda_dot")
    m, m1 = z3.Reals("m m1")
    ea2 = ea + z(DT) * (vr_e - va - wa)
    el2 = el + z(DT) * (lad - vl - wl)
    out.append(check(f"C1l2 declared limit: B2 far band at the domain edge (v_plan {float(v_edge):.2f}, RECOVERY cap "
                     f"{F.v_recovery_max}) with the adversarial lateral consensus |lambda_dot| <= {float(lam):.3f} (A8): "
                     "||e|| need not decrease (lambda is a feedback state)",
                     "exists e, w, lambda_dot: ||e'|| >= ||e||", ENC,
                     [ea >= 3.3, ea <= 3.9, el >= 1.4, el <= 2.0, c == -z(q(F.v_corr_max)), lad == z(lam),
                      wa * wa + wl * wl <= z(W_RES * W_RES), n > V, n * n == ua2 * ua2 + ul2 * ul2,
                      va * n == V * ua2, vl * n == V * ul2, m > 0, m * m == ea * ea + el * el, m1 >= 0,
                      m1 * m1 == ea2 * ea2 + el2 * el2, m1 >= m],
                     expect="sat", note="at the nominal clock the same bound is certified (C1f B2); B0 is certified at both"))
    out.append(check(f"C1m mutation (constant keeping cap, integrated_short, the defect): keeping cap v_slot_max = {F.v_slot_max} at a clock "
                     f"of {F.v_slot_max} m/s leaves no along catch-up authority: a drone behind its slot (e_ok <= e_a <= "
                     "e_lost) never closes, even without disturbance",
                     "exists e_a in [e_ok, e_lost]: e_a' >= e_a", ENC,
                     _on_axis(q(F.v_slot_max), q(F.v_slot_max), q(F.v_slot_max), Fr(0), Fr(0), q(F.e_ok), q(F.e_lost)),
                     expect="sat", note="fixed by FormationRule.keeping_cap (C1n keeping, domain edge) and C4"))
    return out


# ---------------------------------------------------------------------------------------------------------- C2 clocks
def _clip(x, lo, hi):
    return z3.If(x < lo, lo, z3.If(x > hi, hi, x))


def _pos(x):
    return z3.If(x > 0, x, z(0))


def c2_model(cfg: FleetConfig, v_plan, sync_off_slot: bool = True, sigma_mode: str = "consensus",
             fresh: bool = True):
    """One 0.1 s step of the pair (drone i ahead, j behind).  Returns (constants, state, next, constraints)."""
    F, A = cfg.form, cfg.acoustic
    v, kp, leak, P = q(v_plan), q(F.k_progress), q(0.02), q(F.progress_max)
    vw, a = q(F.v_wait), q(F.a_wait)
    eta = q(F.e_ok)
    b_lo, b_hi = q(F.e_ok), q(F.e_ok) + (q(F.keeping_cap(float(v))) - vw) * q(A.fresh_s)
    h = a * DT / v
    D, si, sj, ri = z3.Reals("D sigma_i sigma_j rho_i")
    ni, nai, naj, band = z3.Reals("n_i n_sig_i n_sig_j band")
    upd_i, upd_j = z3.Bools("near_i near_j")
    X = D + si - sj
    lag = X - ni - band
    tgt = z3.If(z3.And(lag > 0, z3.BoolVal(bool(fresh and sync_off_slot))),
                z3.If(z(kp) * lag < z(v - vw), z(kp) * lag, z(v - vw)) / z(v), z(0))
    ri1 = ri + _clip(tgt - ri, -z(h), z(h))
    if sigma_mode == "consensus":
        si_u = _clip(si + z(DT) * (z(kp) * (-X + nai) - z(leak) * si), -z(P), z(P))
        sj_u = _clip(sj + z(DT) * (z(kp) * (X + naj) - z(leak) * sj), -z(P), z(P))
    else:  # leak only: the neighbour is not 'present' and the sonar residual of an abreast pair has no along part
        si_u, sj_u = si * z(1 - DT * leak), sj * z(1 - DT * leak)
    si1, sj1 = z3.If(upd_i, si_u, si), z3.If(upd_j, sj_u, sj)
    D1 = D - z(DT * v) * ri1
    X1 = D1 + si1 - sj1
    cons = [si >= -z(P), si <= z(P), sj >= -z(P), sj <= z(P), ri >= 0, ri <= z(1 - vw / v),
            ni >= -z(eta), ni <= z(eta), nai >= -z(eta), nai <= z(eta), naj >= -z(eta), naj <= z(eta),
            band >= z(b_lo), band <= z(b_hi)]
    k = {"v": v, "kp": kp, "vw": vw, "h": h, "B": b_hi + eta, "b_lo": b_lo, "b_hi": b_hi, "eta": eta, "P": P,
         "leak": leak}
    return k, {"X": X, "ri": ri, "si": si, "sj": sj}, {"X": X1, "ri": ri1, "si": si1, "sj": sj1}, cons


def c2_checks(cfg: FleetConfig) -> list:
    F = cfg.form
    out = []
    delta = Fr(1, 20)
    L = Fr(7, 2)
    d_lock = (L + q(F.e_ok)) ** 2 - L ** 2
    for tag, v_plan in (("nominal clock", q(F.v_nominal)), ("domain edge", q(F.v_recovery_max) - q(F.v_catch_min))):
        k, s, s1, cons = c2_model(cfg, v_plan)
        rho_d = min(k["kp"] * delta, k["v"] - k["vw"]) / k["v"]
        eps = min(DT * k["v"] * rho_d, k["h"])
        R = lambda st: _pos(st["X"] - z(k["B"] + delta)) + _pos(z(rho_d) - st["ri"])  # noqa: E731
        out.append(check(
            f"C2a clock consensus, {tag} (v_plan {float(v_plan):.2f}): max(0, X - B - delta) + (rho_d - rho_i)^+ "
            f"decreases by >= {float(eps):.5f} m per step while X > B + delta (B = band_max + eta = {float(k['B']):.2f} m, "
            f"delta = {float(delta)}), sigma by consensus or frozen",
            "X > B + delta -> X' <= B + delta or R' <= R - eps", ENC,
            cons + [s["X"] > z(k["B"] + delta), z3.Not(z3.Or(s1["X"] <= z(k["B"] + delta), R(s1) <= R(s) - z(eps)))],
            note=f"rate {float(eps / DT):.4f} m/s (k_progress delta at most, after the rate ramp); the drone behind "
                 "keeps its speed (C2j); eventual agreement |X| <= B + delta, by the symmetry i <-> j"))
        bound = k["B"] + 2 * delta
        out.append(check(
            f"C2b ultimate band, {tag}: |X| <= B + 2 delta = {float(bound):.2f} m is invariant (sigma by consensus or frozen)",
            "-(B + 2 delta) <= X <= B + 2 delta -> X' within the same band", ENC,
            cons + [s["X"] >= 0, s["X"] <= z(bound), z3.Or(s1["X"] > z(bound), s1["X"] < -z(bound))],
            note="one step moves X by less than delta; above B + delta it decreases (C2a)"))
        out.append(check(
            f"C2c lock exclusion, {tag}: an abreast pair 3.5 m apart within the band has a range residual below e_ok "
            f"(B + 2 delta = {float(bound):.2f} m < sqrt((L + e_ok)^2 - L^2) = {math.sqrt(d_lock):.2f} m)",
            "X^2 <= (B + 2 delta)^2 -> sqrt(L^2 + X^2) - L < e_ok", ENC,
            [z3.BoolVal(not bound * bound < d_lock)], note="the guard self_recovered is then reachable (with C1n)"))
    k, s, s1, cons = c2_model(cfg, q(F.v_nominal))
    nj, bj = z3.Reals("n_j band_j")
    out.append(check("C2j the drone behind sees no lag in the region X >= 0: it does not slow down (its rate ramps back to "
                     "1 at a_wait), so only the drone ahead re-synchronises",
                     "X >= 0 -> -X - n_j - band_j <= 0", ENC,
                     cons + [s["X"] >= 0, nj >= -z(k["eta"]), nj <= z(k["eta"]), bj >= z(k["b_lo"]),
                             -s["X"] - nj - bj > 0]))
    for mode, label in (("leak", "leak-only"),):
        kl, sl, sl1, consl = c2_model(cfg, q(F.v_nominal), sigma_mode=mode)
        rho_d = min(kl["kp"] * delta, kl["v"] - kl["vw"]) / kl["v"]
        Rl = lambda st: _pos(st["X"] - z(kl["B"] + delta)) + _pos(z(rho_d) - st["ri"])  # noqa: E731
        epsl = min(DT * kl["v"] * rho_d, kl["h"])
        out.append(check(
            f"C2l declared limit: the ranking of C2a is not a ranking when sigma only leaks ({label}: an abreast B2 pair "
            "beyond the presence gate, its sonar residual without along part): the leak can push X up faster than the "
            "re-synchronisation pulls it down; a common ranking across the sigma regimes is open",
            "exists a leak step with X > B + delta, R' > R - eps", ENC,
            consl + [sl["X"] > z(kl["B"] + delta), sl1["X"] > z(kl["B"] + delta), Rl(sl1) > Rl(sl) - z(epsl)],
            expect="sat",
            note="X + |sigma_i| + |sigma_j| is non-increasing in the leak / frozen regime (C2d); the closed loop with "
                 "the switching converges offline (tests/test_p3_recovery_fixes.py), not proved"))
        a_ = lambda x: z3.If(x >= 0, x, -x)  # noqa: E731
        out.append(check(
            "C2d leak / frozen regime: X + |sigma_i| + |sigma_j| never increases, and decreases by dt v rho_i'",
            "Phi' <= Phi - dt v rho_i'", ENC,
            consl + [z3.Not(sl1["X"] + a_(sl1["si"]) + a_(sl1["sj"]) <=
                            sl["X"] + a_(sl["si"]) + a_(sl["sj"]) - z(DT * kl["v"]) * sl1["ri"])]))
    _km, sm, sm1, consm = c2_model(cfg, q(F.v_nominal), sync_off_slot=False, sigma_mode="leak")
    out.append(check(
        "C2m mutation (no re-synchronisation in FORMATION_RECOVERY, the defect): without the re-synchronisation in FORMATION_RECOVERY a pair of drones both "
        f"recovering, sigma relaxed, X >= {math.sqrt(d_lock):.2f} m stays there (the residual of the abreast pair stays above "
        "e_ok: FORMATION_RECOVERY for ever)",
        "exists a state of the lock set whose successor is in it with X' = X", ENC,
        consm + [sm["si"] == 0, sm["sj"] == 0, sm["ri"] == 0, sm["X"] * sm["X"] >= z(d_lock), sm1["X"] == sm["X"],
                 sm1["ri"] == 0], expect="sat",
        note="the set {rho_i = 0, sigma = 0, X >= 1.94 m} is closed under the step without the re-synchronisation; offline: delays of 8 s and more"))
    for fresh in (False, True):
        kw, w0, w1, cw = c2_model(cfg, q(F.v_nominal), fresh=fresh)
        D = lambda w: w["X"] - w["si"] + w["sj"]  # noqa: E731
        decrement = DT * kw["v"] * min(kw["h"], kw["kp"] * delta / kw["v"],
                                       (kw["v"] - kw["vw"]) / kw["v"]) if fresh else Fr(0)
        out.append(check(
            f"C2w pair clock gap with {'a usable report' if fresh else 'a missing report'} outside the lag band",
            "X > B + delta -> D' <= D - decrement, D = X - sigma_i + sigma_j", ENC,
            cw + [w0["X"] > z(kw["B"] + delta), D(w1) > D(w0) - z(decrement)],
            note="fixed ahead/behind role in the pair model; repeated usable reports force entrance into the lag band "
                 "because D stays bounded below while X stays outside it. This does not prove the switched in-band dynamics."))

    kf, sf, sf1, consf = c2_model(cfg, q(F.v_nominal), fresh=False)
    rho_d = min(kf["kp"] * delta, kf["v"] - kf["vw"]) / kf["v"]
    Rf = lambda st: _pos(st["X"] - z(kf["B"] + delta)) + _pos(z(rho_d) - st["ri"])  # noqa: E731
    out.append(check("C2s scope: a step without a fresh payload fix (no lag) lets the slowdown ramp back, the ranking "
                     "increases: the premise of a fresh fix in every step (A7) is where C2a ends",
                     "exists a step without a fix: R' > R", ENC,
                     consf + [sf["X"] > z(kf["B"] + delta), Rf(sf1) > Rf(sf)], expect="sat"))
    return out


# ------------------------------------------------------------------------------------------------------------ C3 tau
def c3_checks(cfg: FleetConfig) -> list:
    F = cfg.form
    out = []
    v, vw, a = q(F.v_nominal), q(F.v_wait), q(F.a_wait)
    dmax, h = 1 - vw / v, a * DT / v
    T = q(F.t_rejoin)
    tau, d, E, tau1, d1, E1 = z3.Reals("tau d E tau1 d1 E1")
    run, run1 = z3.Bools("running running1")
    K = z(v / (2 * a))
    inv = lambda ta, dd, EE, rr: z3.And(dd >= 0, dd <= z(dmax), EE >= 0, EE <= z(T), ta >= 0,  # noqa: E731
                                        z3.If(rr, ta <= z(dmax) * EE, ta + K * dd * dd <= z(dmax) * EE + K * z(dmax * dmax)))
    up = z3.If(d + z(h) < z(dmax), d + z(h), z(dmax))
    down = z3.If(d - z(h) > 0, d - z(h), z(0))
    step = z3.And(z3.If(run, z3.And(d1 == up, E1 == E + z(DT)), z3.And(d1 == down, E1 == E)),
                  tau1 == tau + d1 * z(DT), z3.Implies(z3.Not(run), z3.Not(run1)),
                  z3.Implies(E1 >= z(T), z3.Not(run1)))
    bound = dmax * T + v * dmax * dmax / (2 * a)
    out.append(check(f"C3e one reduced-progress episode adds at most d_max t_rejoin + v d_max^2 / (2 a_wait) = "
                     f"{float(bound):.2f} s to tau (v_plan {float(v):.2f}: {float(v * bound):.2f} m of reference offset)",
                     "inductive invariant: running: tau <= d_max E;  ended: tau + v d^2 / (2 a) <= d_max E + v d_max^2 / (2 a)",
                     ENC, [inv(tau, d, E, run), step, E1 <= z(T), z3.Not(inv(tau1, d1, E1, run1))],
                     note="control/formation_time.py: rate ramps at a_wait / v_plan, d = 1 - rate, E the moving time of "
                          "the episode (<= t_rejoin); same-lane following and the sync follow another drone's tau"))
    tau_i, tau_j, t, end, H = z3.Reals("tau_i tau_j t end H")
    rs = lambda x: z3.If(t - end > 0, t - end, z(0))  # noqa: E731
    inh = lambda x: z3.And(t - x >= end - H, t - x <= end, x >= 0)  # noqa: E731
    a_ = lambda x: z3.If(x >= 0, x, -x)  # noqa: E731
    out.append(check("C3h rendezvous hold (FormationTime._hold_resync): inside a planned hold tau' = max(0, t - end) keeps the "
                     "slot still, never raises tau, contracts the difference of two drones and zeroes it when both "
                     "are within the dwell",
                     "in hold -> 0 <= tau' <= tau, t - tau' in hold, |tau_i' - tau_j'| <= |tau_i - tau_j|, "
                     "(t <= end -> tau_i' = tau_j' = 0)", ENC,
                     [H > 0, inh(tau_i), inh(tau_j),
                      z3.Not(z3.And(rs(tau_i) <= tau_i, inh(rs(tau_i)), a_(rs(tau_i) - rs(tau_j)) <= a_(tau_i - tau_j),
                                    z3.Implies(t <= end, z3.And(rs(tau_i) == 0, rs(tau_j) == 0))))]))
    out.append(check("C3m mutation (no re-synchronisation in the hold): without the hold re-synchronisation two drones leave the rendezvous with the "
                     "delay difference they brought", "exists tau_i != tau_j in the hold: tau' = tau", ENC,
                     [H > 0, inh(tau_i), inh(tau_j), a_(tau_i - tau_j) > 1, t <= end], expect="sat"))
    return out


# ---------------------------------------------------------------------------------------------------------- C4 domain
def c4_checks(cfg: FleetConfig) -> list:
    F, env = cfg.form, cfg.env
    vp = z3.Real("v_plan")
    vrec, vcat, vsl = z(q(F.v_recovery_max)), z(q(F.v_catch_min)), z(q(F.v_slot_max))
    cap = z3.If(vsl > vp + vcat, vsl, vp + vcat)
    cap = z3.If(cap < vrec, cap, vrec)
    dom = [vp >= 0, vp + vcat <= vrec]
    out = [check("C4 operating domain (FormationRule.check_clock_speed raises outside it): every admitted clock speed leaves "
                 f"the catch-up margin v_catch_min = {F.v_catch_min} in the keeping and the recovery modes, the caps within "
                 f"the vehicle authority v_max_nominal = {env.v_max_nominal}, and v_catch_min > w_res + s_res",
                 "v_plan + v_catch_min <= v_rec -> keeping_cap - v_plan >= v_catch_min, v_rec - v_plan >= v_catch_min, "
                 "keeping_cap <= v_rec <= v_max_nominal, v_catch_min > w_res + s_res", ENC,
                 dom + [z3.Not(z3.And(cap - vp >= vcat, vrec - vp >= vcat, cap <= vrec, vrec <= z(q(env.v_max_nominal)),
                                      vcat > z(W_RES + S_RES)))],
                 note="keeping_cap = min(v_recovery_max, max(v_slot_max, v_plan + v_catch_min)), the code of Flows.formation"),
           check("C4m mutation (constant keeping cap): the constant keeping cap v_slot_max leaves no margin at an admitted clock "
                 "(integrated_short 0.40 m/s)", "exists v_plan in the domain: v_slot_max - v_plan < v_catch_min", ENC,
                 dom + [vsl - vp < vcat], expect="sat")]
    return out


def c5_checks(cfg: FleetConfig) -> list:
    """Composition rule for a common nonnegative recovery rank during a switched episode."""
    rank, next_rank, initial, progress, loss, gain, rise = z3.Reals(
        "rank next_rank initial progress loss gain rise")
    invariant = rank + progress <= initial + loss
    common = [rank >= 0, next_rank >= 0, initial >= 0, progress >= 0, loss >= 0]
    return [
        check("C5i recovery rank budget is inductive across tracking and interruption",
              "R + progress <= R0 + loss  and  R' <= R - gain + rise  ->  R' + progress + gain <= R0 + loss + rise",
              ENC, common + [invariant, gain >= 0, rise >= 0, next_rank <= rank - gain + rise,
                             next_rank + progress + gain > initial + loss + rise],
              note="Requires the same rank on both modes and certified rise bounds. Usable reports and physically feasible "
                   "tracking can supply gain; time spent in a safety mode is not treated as gain."),
        check("C5p eventual recovery when cumulative progress exceeds initial rank and interruption cost",
              "R >= 0 and R + progress <= R0 + loss and progress > R0 + loss is impossible", ENC,
              common + [invariant, progress > initial + loss],
              note="Conditional liveness by contradiction if still outside tolerance. A bound on loss and recurring steps "
                   "with a uniform positive gain suffice. Existing pair and slot ranks have different domains; no fleet "
                   "bound is inferred across gate and avoidance transitions."),
        check("C5s scope: regularly available recovery steps do not imply progress against repeated interruptions",
              "A positive rank admits a progress step followed by a bounded interruption returning to the same rank",
              ENC, [rank > 0, next_rank > 0, gain > 0, rise > 0, gain <= 1, rise <= 1,
                    next_rank == rank - gain, rank == next_rank + rise], expect="sat",
              note="The two transitions can repeat forever while remaining outside tolerance, despite infinitely many "
                   "recovery steps. Positive communication fairness alone cannot exclude this execution."),
    ]


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("formation_coupled")
    for part in (c4_checks, c3_checks, c2_checks, c1_checks, c5_checks):
        for r in part(cfg):
            rep.add(r, verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"formation_coupled: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
