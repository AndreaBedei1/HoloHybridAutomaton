"""P2 - critical-region mutual exclusion:  G( sum_i inside_CR_i <= 1 ), no leader, no messages.

The decision rule is NOT re-implemented: holo_fleet.ha.gate_rule is instantiated with Z3
terms.  Each drone measures the pair's relative gate-frame position with an independent
error bounded by eps_rel per component (the two measurements are NOT shared).

Lemmas (UNSAT = holds):
  X0  the six pairwise decision classes are exhaustive and mutually exclusive;
  M1  no simultaneous commit: i and j never both decide COMMIT on the same configuration;
  M2  no mutual waiting (deadlock freedom of the pairwise rule): never both WAIT;
      and WAIT_ROBUST_i implies COMMIT_j (the awaited drone does go);
  M3  priority persistence while the committed drone leaves the queue line (transit window,
      before it is visible in the occupied zone): if j committed over i at time t0 and, up to a
      later time t, (a) queued drones stay at s <= s_queue + hold_tol_s, (b) lateral/vertical
      positions stay within hold tolerances of their t0 values, (c) the committed drone's
      along progress is >= the queued drone's progress - mono_tol, then i cannot COMMIT over j;
  M4  occupancy visibility: a committed drone that is >= occ_gamma + eps past the queue line
      (and not yet past the exit margin) is perceived inside the occupied zone -> occ_busy;
  M5  queued drones (s <= s_queue + hold_tol_s) are never inside the critical region, and
      exited drones (s > exit + eps) are not either;
  M6  composition: with M1, M3, M4, M5 the invariant "at most one drone in PASS" is inductive,
      hence G(occupancy <= 1) (propositional abstraction of the protocol, checked by Z3);
  Mm  mutations: shrinking the gap band (mu_s_hi - mu_s_lo <= 2 eps) breaks M1/M3, removing
      the robust-wait class breaks M2 -> counterexamples expected.

Assumptions (not proved here, validated empirically): detection of every drone inside the
approach corridor with per-component error <= eps_rel (A_det), position holding tolerances
(A_hold), monotone progress of the committed drone (A_mono), nav error included in eps_rel.
"""

from __future__ import annotations

import sys
from dataclasses import replace

import z3

from common import ExactNamespace, Report, Z3L, check, exact  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig, GateRule
from holo_fleet.ha import gate_rule

ENC = "holo_fleet/ha/gate_rule.py (shared rule) + formal/check_mutex.py"


def zabs(x):
    return z3.If(x >= 0, x, -x)


def decisions(G: GateRule, eps: float, meas):
    return gate_rule.decision_predicates(meas[0], meas[1], meas[2], G, eps, Z3L, zabs)


def pair_vars(tag: str, eps: float):
    """True relative position (i - j) and the two independent measurements (i's and j's)."""
    D = z3.Reals(f"ds_{tag} dl_{tag} dz_{tag}")
    ei = z3.Reals(f"eis_{tag} eil_{tag} eiz_{tag}")
    ej = z3.Reals(f"ejs_{tag} ejl_{tag} ejz_{tag}")
    bounds = [z3.And(e >= -eps, e <= eps) for e in ei + ej]
    meas_i = [D[k] + ei[k] for k in range(3)]
    meas_j = [-D[k] + ej[k] for k in range(3)]
    return D, meas_i, meas_j, bounds


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("mutex")
    # exact rationals for every threshold (see common.exact)
    G, eps = ExactNamespace(cfg.gate), exact(cfg.env.eps_rel)

    # ---------------------------------------------------------------- X0
    m = z3.Reals("ms ml mz")
    P = decisions(G, eps, m)
    keys = list(P)
    exclusive = z3.Or(*[z3.And(P[a], P[b]) for ai, a in enumerate(keys) for b in keys[ai + 1:]])
    rep.add(check("X0a decision classes mutually exclusive", "no measurement satisfies two classes", ENC,
                  [exclusive]), verbose)
    rep.add(check("X0b decision classes exhaustive", "every measurement satisfies some class", ENC,
                  [z3.Not(z3.Or(*P.values()))]), verbose)

    # ---------------------------------------------------------------- M1
    D, mi, mj, bnd = pair_vars("m1", eps)
    Pi, Pj = decisions(G, eps, mi), decisions(G, eps, mj)
    rep.add(check("M1 no simultaneous commit (independent errors <= eps)",
                  "COMMIT_i(meas_i) & COMMIT_j(meas_j) is impossible", ENC,
                  bnd + [Pi["COMMIT"], Pj["COMMIT"]]), verbose)

    # ---------------------------------------------------------------- M2
    waits_i = z3.Or(Pi["WAIT_ROBUST"], Pi["WAIT_FRONT"])
    waits_j = z3.Or(Pj["WAIT_ROBUST"], Pj["WAIT_FRONT"])
    rep.add(check("M2a no mutual waiting (deadlock freedom of the pairwise rule)",
                  "never (WAIT_i & WAIT_j): at least one commits or retreats", ENC,
                  bnd + [waits_i, waits_j]), verbose)
    rep.add(check("M2b robust wait is justified", "WAIT_ROBUST_i -> COMMIT_j", ENC,
                  bnd + [Pi["WAIT_ROBUST"], z3.Not(Pj["COMMIT"])]), verbose)
    rep.add(check("M2c retreat targets are consistent (never both BACKOFF_REAR)",
                  "BACKOFF_REAR_i & BACKOFF_REAR_j impossible (sign of ds is robust)", ENC,
                  bnd + [Pi["BACKOFF_REAR"], Pj["BACKOFF_REAR"]]), verbose)

    # ---------------------------------------------------------------- M3 transit window
    # absolute gate-frame positions at commit time t0 and at a later time t (both drones)
    si0, li0, zi0, sj0, lj0, zj0 = z3.Reals("si0 li0 zi0 sj0 lj0 zj0")
    si1, li1, zi1, sj1, lj1, zj1 = z3.Reals("si1 li1 zi1 sj1 lj1 zj1")
    e0 = z3.Reals("e0s e0l e0z")       # j's measurement error at t0 (j commits)
    e1 = z3.Reals("e1s e1l e1z")       # i's measurement error at t  (i tries to commit)
    ebnd = [z3.And(x >= -eps, x <= eps) for x in e0 + e1]
    j_meas_t0 = [sj0 - si0 + e0[0], lj0 - li0 + e0[1], zj0 - zi0 + e0[2]]
    i_meas_t1 = [si1 - sj1 + e1[0], li1 - lj1 + e1[1], zi1 - zj1 + e1[2]]
    # A_hold is needed only for a queued drone that was ALONG-TIED with the committing drone at t0
    # (i.e. both at the queue line): a drone arriving from behind is decided by the along level, where
    # A_mono alone preserves the priority whatever its lateral/vertical motion.
    tied = zabs(sj0 - si0) <= G.mu_s_lo + eps
    hyp = [
        decisions(G, eps, j_meas_t0)["COMMIT"],            # j committed over i at t0 ...
        sj0 >= G.s_queue - G.commit_window,                 # ... from the queue line (commit window)
        si0 <= G.s_queue + G.hold_tol_s, si1 <= G.s_queue + G.hold_tol_s,   # i queued (A_hold)
        sj1 - sj0 >= si1 - si0 - G.mono_tol,                # A_mono: j progresses at least as i
        z3.Implies(tied, z3.And(zabs(li1 - li0) <= 2 * G.hold_tol_lat, zabs(lj1 - lj0) <= 2 * G.hold_tol_lat,
                                zabs(zi1 - zi0) <= 2 * G.hold_tol_z, zabs(zj1 - zj0) <= 2 * G.hold_tol_z)),
    ]
    rep.add(check("M3 priority persistence during the transit window",
                  "COMMIT_j(t0) & A_mono & (along-tied at t0 -> A_hold) -> not COMMIT_i(t)", ENC,
                  ebnd + hyp + [decisions(G, eps, i_meas_t1)["COMMIT"]]), verbose)

    # ---------------------------------------------------------------- M3b late arrivals (timing)
    # A drone that was outside (behind) the approach zone when j committed was not compared with j.
    # Inside the zone its along speed is capped at v_approach, so it needs at least
    # (approach_len - commit_window) / v_approach to reach the commit window; j, moving at
    # >= v_pass_min = v_pass - w_drift, becomes visible in the occupied zone (M4) after at most
    # (commit_window + occ_gamma + eps + hold_tol_s) / v_pass_min.
    t_arr, t_vis, d_i, d_j = z3.Reals("t_arrival t_visible dist_i dist_j")
    v_pass_min = G.v_pass - exact(cfg.env.w_drift_max)
    gf, ef = cfg.gate, cfg.env
    rep.add(check("M3b late arrivals cannot commit before the committed drone is visible",
                  "t_arrival(i) >= (approach_len - commit_window)/v_approach > t_visible(j)", ENC,
                  [d_i >= G.approach_len - G.commit_window, t_arr * G.v_approach >= d_i,
                   d_j <= G.commit_window + G.occ_gamma + eps + G.hold_tol_s, t_vis * v_pass_min <= d_j,
                   t_arr <= t_vis],
                  note=f"t_arrival >= {(gf.approach_len - gf.commit_window) / gf.v_approach:.1f}s, "
                       f"t_visible <= {(gf.commit_window + gf.occ_gamma + ef.eps_rel + gf.hold_tol_s) / (gf.v_pass - ef.w_drift_max):.1f}s"), verbose)

    # ---------------------------------------------------------------- M4 occupancy visibility
    sj, lj, zj, es, el, ez = z3.Reals("sj lj zj es el ez")
    occ_lo = G.s_queue + G.occ_gamma
    occ_hi = G.cr_half_len + G.occ_exit_margin
    seen = z3.And(sj + es >= occ_lo, sj + es <= occ_hi, zabs(lj + el) <= G.corridor_half_width, zabs(zj + ez) <= 3.0)
    rep.add(check("M4 committed drone past occ_gamma+eps is perceived in the occupied zone",
                  "s_j in [s_q+gamma+eps, exit-eps], |l_j| <= W-eps -> occ_busy_i", ENC,
                  [z3.And(x >= -eps, x <= eps) for x in (es, el, ez)] +
                  [sj >= occ_lo + eps, sj <= occ_hi - eps, zabs(lj) <= G.corridor_half_width - eps,
                   zabs(zj) <= 3.0 - eps, z3.Not(seen)]), verbose)

    # ---------------------------------------------------------------- M5 queued/exited drones outside CR
    s = z3.Real("s")
    in_cr = z3.And(s >= -G.cr_half_len, s <= G.cr_half_len)
    rep.add(check("M5a queued drones are outside the critical region", "s <= s_q + hold_tol_s -> not in CR", ENC,
                  [s <= G.s_queue + G.hold_tol_s, in_cr]), verbose)
    rep.add(check("M5b exited drones are outside the critical region",
                  "s > cr_half_len + occ_exit_margin - eps -> not in CR", ENC,
                  [s > G.cr_half_len + G.occ_exit_margin - eps, in_cr]), verbose)

    # ---------------------------------------------------------------- M6 composition (3 drones)
    # phases per drone: Q (queued), T (committed, transit), V (committed, visible in occ zone), X (exited)
    ph = {}
    for k in range(3):
        for p in ("Q", "T", "V", "X"):
            ph[(k, p, 0)] = z3.Bool(f"{p}{k}")
            ph[(k, p, 1)] = z3.Bool(f"{p}{k}n")
    one_phase = []
    for k in range(3):
        for t in (0, 1):
            vs = [ph[(k, p, t)] for p in ("Q", "T", "V", "X")]
            one_phase.append(z3.PbEq([(v, 1) for v in vs], 1))
    passing = lambda k, t: z3.Or(ph[(k, "T", t)], ph[(k, "V", t)])  # noqa: E731
    inv = lambda t: z3.PbLe([(passing(k, t), 1) for k in range(3)], 1)  # noqa: E731
    # "commits[k]" = Q -> T transition; lemma conclusions as constraints:
    trans = []
    for k in range(3):
        commit_k = z3.And(ph[(k, "Q", 0)], ph[(k, "T", 1)])
        others = [m_ for m_ in range(3) if m_ != k]
        for m_ in others:
            trans.append(z3.Implies(z3.And(commit_k, ph[(m_, "V", 0)]), z3.BoolVal(False)))   # M4 -> occ_busy
            trans.append(z3.Implies(z3.And(commit_k, ph[(m_, "T", 0)]), z3.BoolVal(False)))   # M3 persistence
            trans.append(z3.Implies(z3.And(commit_k, z3.And(ph[(m_, "Q", 0)], ph[(m_, "T", 1)])),
                                    z3.BoolVal(False)))                                        # M1
        # legal phase progressions: Q->Q|T, T->T|V, V->V|X, X->X
        trans.append(z3.Implies(ph[(k, "Q", 0)], z3.Or(ph[(k, "Q", 1)], ph[(k, "T", 1)])))
        trans.append(z3.Implies(ph[(k, "T", 0)], z3.Or(ph[(k, "T", 1)], ph[(k, "V", 1)])))
        trans.append(z3.Implies(ph[(k, "V", 0)], z3.Or(ph[(k, "V", 1)], ph[(k, "X", 1)])))
        trans.append(z3.Implies(ph[(k, "X", 0)], ph[(k, "X", 1)]))
    occupancy_ok = lambda t: z3.PbLe([(passing(k, t), 1) for k in range(3)], 1)  # noqa: E731
    rep.add(check("M6a invariant 'at most one drone committed' is inductive",
                  "Inv & T -> Inv'  (T constrained by M1, M3, M4)", ENC,
                  one_phase + trans + [inv(0), z3.Not(inv(1))]), verbose)
    init = [ph[(k, "Q", 0)] for k in range(3)]
    rep.add(check("M6b initial state satisfies the invariant", "all queued -> Inv", ENC,
                  one_phase + init + [z3.Not(inv(0))]), verbose)
    rep.add(check("M6c invariant + M5 implies G(occupancy_CR <= 1)",
                  "only committed drones can be inside the CR (M5), at most one is committed", ENC,
                  one_phase + [inv(0), z3.Not(occupancy_ok(0))]), verbose)

    # ---------------------------------------------------------------- mutations
    Gbad = ExactNamespace(replace(cfg.gate, mu_s_hi=cfg.gate.mu_s_lo + 2 * cfg.env.eps_rel - 0.05))
    D, mi, mj, bnd = pair_vars("mm", eps)
    rep.add(check(f"Mm1 mutation: gap band mu_s_hi-mu_s_lo={2 * cfg.env.eps_rel - 0.05:.2f} <= 2eps breaks M3",
                  "expect counterexample", ENC,
                  ebnd + [decisions(Gbad, eps, j_meas_t0)["COMMIT"], sj0 >= Gbad.s_queue - Gbad.commit_window,
                          si0 <= Gbad.s_queue + Gbad.hold_tol_s, si1 <= Gbad.s_queue + Gbad.hold_tol_s,
                          sj1 - sj0 >= si1 - si0 - Gbad.mono_tol,
                          z3.Implies(zabs(sj0 - si0) <= Gbad.mu_s_lo + eps,
                                     z3.And(zabs(li1 - li0) <= 2 * Gbad.hold_tol_lat, zabs(lj1 - lj0) <= 2 * Gbad.hold_tol_lat,
                                            zabs(zi1 - zi0) <= 2 * Gbad.hold_tol_z, zabs(zj1 - zj0) <= 2 * Gbad.hold_tol_z)),
                          decisions(Gbad, eps, i_meas_t1)["COMMIT"]], expect="sat"), verbose)
    Gbad2 = ExactNamespace(replace(cfg.gate, mu_s_hi=0.4, mu_s_lo=0.2))
    Pi2, Pj2 = decisions(Gbad2, eps, mi), decisions(Gbad2, eps, mj)
    rep.add(check("Mm2 mutation: margins below eps (mu_s_hi=0.4) allow a double commit",
                  "expect counterexample", ENC, bnd + [Pi2["COMMIT"], Pj2["COMMIT"]], expect="sat"), verbose)
    # without the retreat classes the rule degenerates to "commit or wait": mutual waiting exists
    Pi3, Pj3 = decisions(G, eps, mi), decisions(G, eps, mj)
    rep.add(check("Mm3 mutation: 'commit or wait' (no retreat classes) allows mutual waiting",
                  "expect counterexample", ENC, bnd + [z3.Not(Pi3["COMMIT"]), z3.Not(Pj3["COMMIT"])],
                  expect="sat"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"mutex: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
