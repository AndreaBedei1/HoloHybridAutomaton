"""P3 - formation recovery (liveness):  G( formation_lost -> F formation_recovered ).

No time bound is claimed.  Eventual recovery is proved by RANKING FUNCTIONS on an abstraction of the
deployed formation law (holo_fleet.control.flows.Flows.formation), under explicit fairness and
environment assumptions; every lemma is a one-step, universally quantified Z3 query, hence valid for
unbounded time.

Abstraction (per drone, per axis, dt = 0.1 s).  The slot moves with the shared FormationClock at
v_clock plus the drift of the local progress offset sigma (|sigma_dot| <= s_rate: sigma is driven by
the along-track sonar residual, capped by range_gate, plus a leak); the drone commands
v = v_clock t + k_slot e (+ the capped lateral sonar correction), saturated at v_recovery (along)
and sqrt(v_recovery^2 - v_clock^2) (lateral); the unrejected disturbance after the low-level
integrator has converged is |w| <= w_res.

  along   e' = e + dt (v_clock + sigma_dot - clip(v_clock + k_slot e, -V, V) - w)
  lateral e' = e + dt (- clip(k_slot e + corr, -V_lat, V_lat) - w),   |corr| <= v_corr_max

F1  far band (saturated): |e| decreases by >= eps_far per step whenever |e| >= e1  (ranking R1);
F2  near band (linear):   |e| decreases by >= eps_near per step whenever e* <= |e| <= e1 (ranking R2);
F3  the ball |e| <= e* is invariant (once recovered, the slot error stays recovered);
F4  automaton: in FORMATION_RECOVERY, calm, no gate, the guard `recovered` enables exactly the edge to
    FORMATION_FOLLOW; in FORMATION_FOLLOW `lost` enables exactly the edge to FORMATION_RECOVERY
    (the same spec functions as the runtime);
Fm  mutations: no catch-up margin (v_recovery = v_clock) must break F1; k_slot = 0 must break F2.

Fairness / environment assumptions (eventual recovery holds under them, and only under them):
  A1  every perturbation ends: currents return inside the envelope, encounters (SEPARATION_WARNING /
      COLLISION_AVOIDANCE) and gate passages are finitely many;
  A2  after a perturbation the residual disturbance is |w| <= w_res (the integrator has converged);
  A3  the sensors stay healthy (no FAILSAFE) and the expected neighbours become visible
      (neighbors_ok), so the onboard estimate form_err follows the slot errors;
  A4  slot errors inside the ball give form_err < e_ok (sonar residual = own + neighbour slot error +
      range/hull ambiguity): checked on the logs, not proved.
"""

from __future__ import annotations

import math
import sys

import z3

from common import CheckResult, Obs, Report, Z3L, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.spec import Mode, Predicates, build_edges

ENC = "formal/check_formation.py (abstraction of Flows.formation) + holo_fleet/ha/spec.py"
DT = 0.1
W_RES = 0.05


def clip(v, lo, hi):
    return z3.If(v < lo, lo, z3.If(v > hi, hi, v))


def zabs(x):
    return z3.If(x >= 0, x, -x)


def model(cfg: FleetConfig, v_rec=None, k=None):
    F = cfg.form
    V = F.v_recovery_max if v_rec is None else v_rec
    kk = F.k_slot if k is None else k
    s_rate = F.k_progress * F.range_gate_m + 0.02 * F.progress_max       # |sigma_dot| bound
    return {"V": V, "V_lat": math.sqrt(max(V * V - F.v_nominal ** 2, 0.0)), "k": kk, "v_clock": F.v_nominal,
            "s_rate": s_rate, "w": W_RES, "corr": F.v_corr_max}


def along_step(p, e):
    sd, w = z3.Reals("sigma_dot w")
    e1 = e + DT * (p["v_clock"] + sd - clip(p["v_clock"] + p["k"] * e, -p["V"], p["V"]) - w)
    return e1, [zabs(sd) <= p["s_rate"], zabs(w) <= p["w"]]


def lateral_step(p, e):
    c, w = z3.Reals("corr w")
    e1 = e + DT * (-clip(p["k"] * e + c, -p["V_lat"], p["V_lat"]) - w)
    return e1, [zabs(c) <= p["corr"], zabs(w) <= p["w"]]


def certify(step, p, lo, hi, eps):
    """UNSAT  <=>  for every e with lo <= |e| <= hi and every disturbance: |e'| <= |e| - eps."""
    e = z3.Real("e")
    e1, dist = step(p, e)
    return [zabs(e) >= lo, zabs(e) <= hi] + dist + [zabs(e1) > zabs(e) - eps]


def best_eps(step, p, lo, hi):
    """Largest decrease per step Z3 certifies on the band (bisection), or 0."""
    a, b = 0.0, 0.2
    if check("", "", ENC, certify(step, p, lo, hi, 1e-6)).verdict != "unsat":
        return 0.0
    for _ in range(18):
        m = (a + b) / 2
        if check("", "", ENC, certify(step, p, lo, hi, m)).verdict == "unsat":
            a = m
        else:
            b = m
    return a


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("formation")
    p = model(cfg)
    if verbose:
        print("P3 abstraction:", {k: round(v, 4) for k, v in p.items()})
    E_MAX = 8.0
    for axis, step, sat_rate in (("along", along_step, p["V"] - p["v_clock"] - p["s_rate"] - p["w"]),
                                 ("lateral", lateral_step, p["V_lat"] - p["corr"] - p["w"])):
        noise = (p["s_rate"] + p["w"]) if axis == "along" else (p["corr"] + p["w"])
        e_star = noise / p["k"] * 1.05 + 0.005                       # linear-regime ball (with a small margin)
        e1 = max((p["V"] if axis == "along" else p["V_lat"]) / p["k"], e_star + 0.05)
        eps_far = best_eps(step, p, e1, E_MAX)
        rep.add(CheckResult(f"F1 {axis}: ranking decrease in the saturated band [{e1:.2f}, {E_MAX}] m",
                            "lo <= |e| <= hi -> |e'| <= |e| - eps_far", ENC, "unsat",
                            verdict="unsat" if eps_far > 0 else "sat", passed=eps_far > 0,
                            note=f"eps_far = {eps_far * 10:.4f} m/s certified (analytic margin {sat_rate:.3f} m/s)"), verbose)
        eps_near = best_eps(step, p, e_star, e1)
        rep.add(CheckResult(f"F2 {axis}: ranking decrease in the linear band [{e_star:.2f}, {e1:.2f}] m",
                            "lo <= |e| <= hi -> |e'| <= |e| - eps_near", ENC, "unsat",
                            verdict="unsat" if eps_near > 0 else "sat", passed=eps_near > 0,
                            note=f"eps_near = {eps_near * 10:.4f} m/s; ball e* = {e_star:.2f} m"), verbose)
        e = z3.Real("e")
        e_n, dist = step(p, e)
        rep.add(check(f"F3 {axis}: the ball |e| <= {e_star:.2f} m is invariant", "|e| <= e* -> |e'| <= e*", ENC,
                      [zabs(e) <= e_star] + dist + [zabs(e_n) > e_star]), verbose)
        rep.results[-1].note = f"e* = {e_star:.3f} m (A4 needs the onboard form_err below e_ok = {cfg.form.e_ok} m)"
    # ---------------------------------------------------------------- F4 automaton
    edges = build_edges(cfg)
    P = Predicates(cfg)
    o = Obs()
    gs = [(e, e.guard(o, Z3L)) for e in edges[Mode.FORMATION_RECOVERY]]
    calm = P.calm(o, Z3L, Mode.FORMATION_RECOVERY)
    rep.add(check("F4a RECOVERY & calm & no gate & recovered -> only the edge to FORMATION_FOLLOW",
                  "guard(e) & target(e) != FOLLOW is unsatisfiable", ENC,
                  [o.legal(cfg), calm, z3.Not(o.gate_zone), z3.Not(o.committed), P.recovered(o, Z3L),
                   z3.Or(*[g for e, g in gs if e.target != Mode.FORMATION_FOLLOW])]), verbose)
    rep.add(check("F4b ... and that edge is enabled", "recovered -> guard(formation_recovered)", ENC,
                  [o.legal(cfg), calm, z3.Not(o.gate_zone), z3.Not(o.committed), P.recovered(o, Z3L),
                   z3.Not(z3.Or(*[g for e, g in gs if e.target == Mode.FORMATION_FOLLOW]))]), verbose)
    gs = [(e, e.guard(o, Z3L)) for e in edges[Mode.FORMATION_FOLLOW]]
    calm_f = P.calm(o, Z3L, Mode.FORMATION_FOLLOW)
    rep.add(check("F4c FOLLOW & calm & no gate & lost -> FORMATION_RECOVERY", "lost -> target = RECOVERY", ENC,
                  [o.legal(cfg), calm_f, z3.Not(o.gate_zone), z3.Not(o.committed), P.lost(o, Z3L),
                   z3.Or(*[g for e, g in gs if e.target != Mode.FORMATION_RECOVERY])]), verbose)
    # ---------------------------------------------------------------- mutations
    pm = model(cfg, v_rec=cfg.form.v_nominal)
    e1m = max(pm["V"] / pm["k"], 0.5)
    rep.add(check("Fm1 mutation: no catch-up margin (v_recovery = v_clock) must break F1 (along)",
                  "expect counterexample", ENC, certify(along_step, pm, e1m, E_MAX, 1e-6), expect="sat"), verbose)
    pk = model(cfg, k=0.0)
    rep.add(check("Fm2 mutation: k_slot = 0 must break F2 (lateral)", "expect counterexample", ENC,
                  certify(lateral_step, pk, 0.4, 2.0, 1e-6), expect="sat"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"formation: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
