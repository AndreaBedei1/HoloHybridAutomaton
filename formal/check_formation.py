"""P3 - bounded formation recovery:  G( formation_lost -> F_[0,T] formation_recovered ).

Abstraction of the deployed formation law (holo_fleet.control.flows.Flows.formation) for 3 drones,
one step of dt_f = 0.5 s (controller at 10 Hz; gain * dt_f << 1 keeps Euler faithful).  Along-track
and lateral laws are decoupled, so each axis is analysed separately:

  along  : x_i' = x_i + (clip(v_nom + k_along*(mean_j (x_j - x_i) + eta_i), 0, v_max) + omega_i) dt_f
  lateral: y_i' = y_i + (clip(-k_lane*y_i + k_rel*(mean_j (y_j - y_i) + eta_i), -v_lat, v_lat) + omega_i) dt_f
  |eta| <= eps_rel   (relative-measurement error, adversarial, re-chosen at every step)
  |omega| <= w_res   (residual velocity-tracking error, adversarial)

Error measure (as the referee): E = max_i |x_i - mean x| (resp. y).  Proof by RANKING FUNCTION,
every lemma is a one-step, universally quantified Z3 query (hence valid for unbounded time):

  R_k   for every band [lo_k, hi_k] of E down to the tolerance: E' <= E - delta_k  (delta_k > 0
        found by bisection, the largest value Z3 can certify);
  B     the tolerance box {E <= tol} is invariant (once recovered, stays recovered);
  L     lateral: the lane-error box |y_i| <= L0 is invariant (keeps the saturation analysis valid);
  F3    tol_along, tol_lateral and the depth allowance fit inside e_ok (the referee's threshold);
  F4    recovery bound T = sum_k (hi_k - lo_k)/delta_k * dt_f  <=  referee deadline;
  Fm    mutation: without along-track consensus the ranking lemma cannot be certified.

The bound T is a worst case over adversarial noise; the measured recoveries are much faster.
Assumptions (validated empirically in scripts/validate_assumptions.py): no new perturbation during
recovery, neighbours perceived and correctly associated, |eta| <= eps_rel, |omega| <= w_res.
"""

from __future__ import annotations

import math
import sys
import time

import z3

from common import CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig

ENC = "formal/check_formation.py (abstraction of Flows.formation)"
DT_F = 0.5
N = 3
A0 = 5.5          # max initial |x_i - mean| [m] (an ~8 m along spread, e.g. the column after a gate)
L0 = 2.0          # max initial lateral lane error [m]
A1 = 0.275        # lateral phase-2 set |y_i| <= A1: no lateral saturation possible inside it
W_RES = 0.03      # residual tracking error [m/s] (measured p95 ~0.027 m/s over 0.5 s windows)
Z_ERR = 0.03      # depth deviation allowance [m] (measured depth hold within +-0.02 m)


def clip(v, lo, hi):
    return z3.If(v < lo, lo, z3.If(v > hi, hi, v))


def zabs(x):
    return z3.If(x >= 0, x, -x)


def equilibrium_bounds(cfg: FleetConfig):
    """Linear-regime steady bounds of max |e_i| under constant adversarial noise (3-drone complete graph)."""
    F, eps = cfg.form, cfg.env.eps_rel
    e_a = (4.0 / 3.0) * (F.k_along * eps + W_RES) / (1.5 * F.k_along)
    e_l = (4.0 / 3.0) * (F.k_lat_rel * eps + W_RES) / (F.k_lat_lane + 1.5 * F.k_lat_rel)
    return e_a, e_l


def tolerances(cfg: FleetConfig):
    e_a, e_l = equilibrium_bounds(cfg)
    return round(e_a + 0.06, 3), round(e_l + 0.05, 3)


def one_step(cfg, kind: str, k_a=None, lat_box: float = L0):
    F, env, eps = cfg.form, cfg.env, cfg.env.eps_rel
    x = [z3.Real(f"{kind}{i}") for i in range(N)]
    xn = [z3.Real(f"{kind}{i}n") for i in range(N)]
    cons = []
    for i in range(N):
        eta, om = z3.Reals(f"eta{i} om{i}")
        cons += [eta >= -eps, eta <= eps, om >= -W_RES, om <= W_RES]
        res = sum(x[j] - x[i] for j in range(N) if j != i) / (N - 1) + eta
        if kind == "x":
            ka = F.k_along if k_a is None else k_a
            v = clip(F.v_nominal + ka * res, 0.0, env.v_max_nominal)
        else:
            v = clip(-F.k_lat_lane * x[i] + F.k_lat_rel * res, -F.v_lat_max, F.v_lat_max)
        cons.append(xn[i] == x[i] + (v + om) * DT_F)
    if kind == "x":
        cons.append(sum(x) == 0)                       # translation invariance: centroid at 0 (WLOG)
    else:
        cons += [zabs(xi) <= lat_box for xi in x]      # lane-error box (invariant, lemmas L/L1)
    return x, xn, cons


def dev(v):
    m = sum(v) / N
    return [vi - m for vi in v]


def E_constraints(E, e):
    return [z3.And(E >= ei, E >= -ei) for ei in e] + [z3.Or(*[z3.Or(E == ei, E == -ei) for ei in e])]


def band_query(cfg, kind, lo, hi, delta, k_a=None, measure="dev", lat_box=L0):
    """measure = 'dev': E = max |v_i - mean v| (formation error);  'abs': E = max |v_i| (lane error)."""
    x, xn, cons = one_step(cfg, kind, k_a, lat_box)
    E = z3.Real("E")
    e, en = (dev(x), dev(xn)) if measure == "dev" else (x, xn)
    q = cons + E_constraints(E, e) + [E >= lo, E <= hi]
    q.append(z3.Or(*[z3.Or(ei > E - delta, -ei > E - delta) for ei in en]))
    return q


def certify_band(cfg, kind, lo, hi, k_a=None, measure="dev", lat_box=L0):
    """Largest delta (bisection) such that E' <= E - delta for every state with E in [lo, hi]."""
    kw = dict(k_a=k_a, measure=measure, lat_box=lat_box)
    if check("", "", ENC, band_query(cfg, kind, lo, hi, 1e-5, **kw), timeout_ms=60000).verdict != "unsat":
        return None
    a, b = 1e-5, 0.3
    for _ in range(18):
        mid = (a + b) / 2
        if check("", "", ENC, band_query(cfg, kind, lo, hi, mid, **kw), timeout_ms=60000).verdict == "unsat":
            a = mid
        else:
            b = mid
    return a


def bands(top: float, tol: float):
    edges = [top]
    while edges[-1] > tol + 1e-9:
        nxt = max(tol, edges[-1] * 0.8 if edges[-1] > 1.0 else edges[-1] - 0.08)
        edges.append(round(nxt, 4))
    return list(zip(edges[1:], edges[:-1]))


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("formation")
    F = cfg.form
    e_a, e_l = equilibrium_bounds(cfg)
    tol_a, tol_l = tolerances(cfg)
    if verbose:
        print(f"linear-regime steady bounds: along {e_a:.3f} m, lateral {e_l:.3f} m; tolerances {tol_a} / {tol_l} m; "
              f"norm {math.hypot(tol_a, tol_l):.3f} + depth {Z_ERR} vs e_ok {F.e_ok}")
    # B: invariance of the tolerance boxes (lateral box inside the no-saturation set |y| <= A1)
    x, xn, cons = one_step(cfg, "x")
    e, en = dev(x), dev(xn)
    rep.add(check(f"B invariance of the along tolerance box (E <= {tol_a} m)", "E <= tol -> E' <= tol", ENC,
                  cons + [zabs(ei) <= tol_a for ei in e] + [z3.Or(*[zabs(ei) > tol_a for ei in en])]), verbose)
    x, xn, cons = one_step(cfg, "y", lat_box=A1)
    e, en = dev(x), dev(xn)
    rep.add(check(f"B invariance of the lateral tolerance box (E <= {tol_l} m, within |y| <= {A1} m)",
                  "|y| <= A1 & E <= tol -> E' <= tol", ENC,
                  cons + [zabs(ei) <= tol_l for ei in e] + [z3.Or(*[zabs(ei) > tol_l for ei in en])]), verbose)
    # L / L1: lane-error boxes are invariant
    for box, name in ((L0, "L"), (A1, "L1")):
        x, xn, cons = one_step(cfg, "y", lat_box=box)
        rep.add(check(f"{name} lane-error box |y_i| <= {box} m is invariant", "|y| <= box -> |y'| <= box", ENC,
                      cons + [z3.Or(*[zabs(v) > box for v in xn])]), verbose)
    # F3
    rep.add(check("F3 tolerances fit the recovered threshold e_ok",
                  f"sqrt({tol_a}^2 + {tol_l}^2) + {Z_ERR} <= e_ok = {F.e_ok}", ENC,
                  [z3.BoolVal(math.hypot(tol_a, tol_l) + Z_ERR > F.e_ok)]), verbose)
    # R: ranking lemmas per band
    phases = [("along", "x", "dev", A0, tol_a, L0),
              ("lateral phase 1 (lane error, saturated regime)", "y", "abs", L0, A1, L0),
              ("lateral phase 2 (formation error inside |y| <= A1)", "y", "dev", round(4 * A1 / 3, 4), tol_l, A1)]
    T_phase = {}
    for name, kind, measure, top, tol, box in phases:
        total, ok, t0 = 0.0, True, time.time()
        details = []
        for lo, hi in bands(top, tol):
            d = certify_band(cfg, kind, lo, hi, measure=measure, lat_box=box)
            if d is None:
                ok = False
                details.append(f"[{lo},{hi}]: not certified")
                break
            steps = math.ceil((hi - lo) / d)
            total += steps * DT_F
            details.append(f"[{lo:.2f},{hi:.2f}] delta={d * 1000:.1f} mm/step -> {steps * DT_F:.1f}s")
        res = CheckResult(prop=f"R ranking lemmas - {name}: decrease >= delta_k per step in every band down to {tol} m",
                          formula="E in [lo_k, hi_k] -> E' <= E - delta_k  (one step, all noise, all saturations)",
                          encoding=ENC, expect="unsat", verdict="unsat" if ok else "unknown", passed=ok,
                          seconds=round(time.time() - t0, 1), note="; ".join(details) + f" | total {total:.1f}s")
        rep.add(res, verbose)
        if verbose:
            for d_ in details:
                print("       " + d_)
        if ok:
            T_phase[name] = total
    if len(T_phase) == 3:
        t_along = T_phase[phases[0][0]]
        t_lat = T_phase[phases[1][0]] + T_phase[phases[2][0]]
        T = max(t_along, t_lat)
        rep.add(check(f"F4 worst-case recovery bound T={T:.1f} s <= referee deadline {F.t_recovery_max} s",
                      "sum_k ceil(width_k/delta_k) dt_f <= T_deadline (axes recover concurrently)", ENC,
                      [z3.BoolVal(T > F.t_recovery_max)],
                      note=f"along {t_along:.1f}s from {A0} m, lateral {t_lat:.1f}s from {L0} m"), verbose)
    # mutation: no along-track consensus -> the first band cannot be certified
    lo, hi = bands(A0, tol_a)[0]
    q = band_query(cfg, "x", lo, hi, 1e-5, k_a=0.0)
    rep.add(check("Fm mutation: without along-track consensus the error does not decrease",
                  "expect counterexample", ENC, q, expect="sat"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"formation: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
