"""P3 - bounded formation recovery:  G( formation_lost -> F_[0,T] formation_recovered ).

Abstraction of the deployed formation law (holo_fleet.control.flows.Flows.formation), 3 drones,
discrete time dt_f = 0.5 s (controller runs at 10 Hz; gain * dt_f << 1 keeps Euler faithful).
Along-track and lateral dynamics are decoupled in the law, so they are checked separately:

  along  : x_i = s_i - a_i (slot-corrected progress)
           x_i' = x_i + (clip(v_nom + k_along * (mean_j (x_j - x_i) + eta_i), 0, v_max) + omega_i) dt_f
  lateral: y_i = l_i - o_i (deviation from own slot offset)
           y_i' = y_i + (clip(-k_lane y_i + k_rel (mean_j (y_j - y_i) + eta'_i), +-v_lat_max) + omega'_i) dt_f
  |eta|, |eta'| <= eps_rel   relative-measurement error (adversarial, time varying, per drone)
  |omega|       <= w_res     residual velocity-tracking error after current rejection

Formation error (as the referee computes it) = max_i ||(x_i - mean x, y_i - mean y)||.

  F1a/F1b  incremental BMC: from ANY lost formation (along spread <= A0, lateral deviation <= L0,
           all neighbours visible) every relative error is inside its tolerance after N steps;
           the minimal N gives the formal recovery bound T = N dt_f;
  F2a/F2b  the tolerance boxes are invariant (once recovered, stays recovered);
  F3       the tolerance box (plus depth error) fits inside e_ok, the recovered threshold;
  F4       T_formal <= referee deadline;
  Fm       mutation: without along-track consensus (k_along = 0) no recovery -> counterexample.

Tolerances follow from the worst-case noise gain of the consensus (3-drone complete graph,
contraction 1.5 k): |eta_i - mean eta| <= 4 eps / 3.
"""

from __future__ import annotations

import math
import sys

import z3

from common import CheckResult, Report, check, model_to_dict  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig

ENC = "formal/check_formation.py (abstraction of Flows.formation)"
DT_F = 0.5
N_DRONES = 3
A0 = 6.0          # initial along spread [m] (gate column, gust)
L0 = 2.0          # initial lateral deviation [m]
W_RES = 0.03      # residual tracking error [m/s] (calibration: ~0.001 m/s steady up to 0.40 m/s drift)
Z_ERR = 0.05      # depth error allowance [m]


def clip(v, lo, hi):
    return z3.If(v < lo, lo, z3.If(v > hi, hi, v))


def tolerances(cfg: FleetConfig):
    F, eps = cfg.form, cfg.env.eps_rel
    tol_a = round((4 * eps / 3) / 1.5 + 0.04, 3)
    tol_l = round(F.k_lat_rel * (4 * eps / 3) / (F.k_lat_lane + 1.5 * F.k_lat_rel) + 0.04, 3)
    return tol_a, tol_l


def step_along(cfg, x, t, k_a):
    F, env, eps = cfg.form, cfg.env, cfg.env.eps_rel
    nxt, cons = [], []
    for i in range(N_DRONES):
        eta, om = z3.Reals(f"eta{i}_{t} om{i}_{t}")
        cons += [eta >= -eps, eta <= eps, om >= -W_RES, om <= W_RES]
        res = sum(x[j] - x[i] for j in range(N_DRONES) if j != i) / (N_DRONES - 1) + eta
        v = clip(F.v_nominal + k_a * res, 0.0, env.v_max_nominal)
        xn = z3.Real(f"x{i}_{t + 1}")
        cons.append(xn == x[i] + (v + om) * DT_F)
        nxt.append(xn)
    return nxt, cons


def step_lat(cfg, y, t):
    F, eps = cfg.form, cfg.env.eps_rel
    nxt, cons = [], []
    for i in range(N_DRONES):
        eta, om = z3.Reals(f"etl{i}_{t} oml{i}_{t}")
        cons += [eta >= -eps, eta <= eps, om >= -W_RES, om <= W_RES]
        res = sum(y[j] - y[i] for j in range(N_DRONES) if j != i) / (N_DRONES - 1) + eta
        u = clip(-F.k_lat_lane * y[i] + F.k_lat_rel * res, -F.v_lat_max, F.v_lat_max)
        yn = z3.Real(f"y{i}_{t + 1}")
        cons.append(yn == y[i] + (u + om) * DT_F)
        nxt.append(yn)
    return nxt, cons


def outside(v, tol):
    m = sum(v) / N_DRONES
    return z3.Or(*[z3.Or(vi - m > tol, m - vi > tol) for vi in v])


def bmc_min_horizon(cfg, kind: str, tol: float, k_a=None, n_max: int = 160, expect_found=True):
    """Incremental BMC: smallest N such that no trajectory is outside tol at step N."""
    s = z3.Solver()
    s.set("timeout", 300000)
    v = [z3.Real(f"{'x' if kind == 'along' else 'y'}{i}_0") for i in range(N_DRONES)]
    if kind == "along":
        s.add(*[v[i] - v[j] <= A0 for i in range(N_DRONES) for j in range(N_DRONES)])
    else:
        s.add(*[z3.And(vi <= L0, vi >= -L0) for vi in v])
    last_model = None
    for t in range(n_max):
        if kind == "along":
            v, cons = step_along(cfg, v, t, cfg.form.k_along if k_a is None else k_a)
        else:
            v, cons = step_lat(cfg, v, t)
        s.add(*cons)
        if t + 1 < 4:
            continue
        s.push()
        s.add(outside(v, tol))
        r = s.check()
        if r == z3.sat:
            last_model = s.model()
        s.pop()
        if r == z3.unsat:
            return t + 1, None
        if r == z3.unknown:
            return None, "unknown"
    return None, last_model


def invariance(cfg, kind: str, tol: float):
    v = [z3.Real(f"{'x' if kind == 'along' else 'y'}{i}_0") for i in range(N_DRONES)]
    if kind == "along":
        nxt, cons = step_along(cfg, v, 0, cfg.form.k_along)
    else:
        nxt, cons = step_lat(cfg, v, 0)
    return cons + [z3.Not(outside(v, tol)), outside(nxt, tol)]


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    import time

    rep = Report("formation")
    F = cfg.form
    tol_a, tol_l = tolerances(cfg)
    if verbose:
        print(f"tolerances: along {tol_a} m, lateral {tol_l} m, norm {math.hypot(tol_a, tol_l):.3f} m "
              f"(+{Z_ERR} depth) vs e_ok={F.e_ok} m")
    rep.add(check("F2a along tolerance box is invariant", "inside_a(t) & T -> inside_a(t+1)", ENC,
                  invariance(cfg, "along", tol_a)), verbose)
    rep.add(check("F2b lateral tolerance box is invariant", "inside_l(t) & T -> inside_l(t+1)", ENC,
                  invariance(cfg, "lat", tol_l)), verbose)
    rep.add(check("F3 tolerance box fits the recovered threshold e_ok",
                  f"sqrt({tol_a}^2 + {tol_l}^2) + {Z_ERR} <= e_ok={F.e_ok}", ENC,
                  [z3.BoolVal(math.hypot(tol_a, tol_l) + Z_ERR > F.e_ok)]), verbose)
    T = {}
    for kind, tol, label, init in (("along", tol_a, "F1a", f"spread <= {A0} m"), ("lat", tol_l, "F1b", f"|y| <= {L0} m")):
        t0 = time.time()
        N, info = bmc_min_horizon(cfg, kind, tol)
        res = CheckResult(prop=f"{label} {kind} recovery from {init}", formula="Init_lost & T^N -> inside(N)",
                          encoding=ENC, expect="unsat")
        res.seconds = round(time.time() - t0, 2)
        if N is not None:
            T[kind] = N * DT_F
            res.verdict, res.passed = "unsat", True
            res.prop += f": inside tolerance after N={N} steps = {N * DT_F:.1f} s"
        else:
            res.verdict, res.passed = ("unknown" if info == "unknown" else "sat"), False
            if info not in (None, "unknown"):
                res.counterexample = model_to_dict(info)
        rep.add(res, verbose)
    if len(T) == 2:
        t_formal = max(T.values())
        rep.add(check(f"F4 formal recovery bound T={t_formal:.1f} s within referee deadline {F.t_recovery_max} s",
                      "T_formal <= T_deadline (with F2: G(lost -> F_[0,T] recovered))", ENC,
                      [z3.BoolVal(t_formal > F.t_recovery_max)],
                      note=f"along {T['along']:.1f}s, lateral {T['lat']:.1f}s"), verbose)
        # mutation: no along consensus -> still outside the box at the same horizon
        s = z3.Solver()
        v = [z3.Real(f"x{i}_0") for i in range(N_DRONES)]
        s.add(*[v[i] - v[j] <= A0 for i in range(N_DRONES) for j in range(N_DRONES)])
        for t in range(int(T["along"] / DT_F)):
            v, cons = step_along(cfg, v, t, 0.0)
            s.add(*cons)
        t0 = time.time()
        s.add(outside(v, tol_a))
        r = s.check()
        mres = CheckResult(prop="Fm mutation: without along-track consensus the formation does not recover",
                           formula="expect counterexample", encoding=ENC, expect="sat", verdict=str(r),
                           passed=(r == z3.sat), seconds=round(time.time() - t0, 2))
        if r == z3.sat:
            mres.counterexample = {k: v_ for k, v_ in model_to_dict(s.model()).items() if k.startswith("x")}
        rep.add(mres, verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"formation: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
