"""P1 - inter-vehicle separation:  G( forall i != j : d_ij >= d_safe ).

Abstraction (all numbers imported from holo_fleet.config; plant numbers come from
scripts/calibrate_plant.py):

S1  Pairwise radial model (linear real arithmetic, discrete time dt).
    State (d, c): true centre distance and closing speed of a pair.
    * velocity caps            c <= c_max = v_escape + v_max_nominal + w_rel_max
    * perception               measured distance dh in [d - eps', d + eps'],
                               eps' = eps_rel + c_max * tau_max   (error + staleness)
    * reaction (SW/CA)         dh < d_warning  ->  c' <= max(c - a_pair*dt, w_rel - v_open)
                               (both drones remove their closing component; at least one opens
                               at >= v_open: lemmas S2/S3), a_pair = 2 * a_brake
    * no guarantee otherwise   dh >= d_warning ->  c' <= c_max (adversarial)
    * kinematics               d' = d - (c + c')/2 * dt
    Verified by bounded model checking from Init (d >= d_warning + eps' + 0.1) and by
    k-induction (unbounded horizon).  A mutated parameter set (d_warning too small) must fail.

S2  Local opening lemma (nonlinear real arithmetic).  For a drone whose (<= 2) threat
    directions u1, u2 are at most 90 deg apart:
    (a) the SW safety filter is feasible with opening v_open: the witness
        v = -s (u1+u2)/|u1+u2| (s = 0.4 <= v_filter_cap) satisfies v.u_j <= -v_open;
    (b) the CA escape e = normalise(b + w*vz*z_hat), b = -(u1+u2)/|u1+u2|, with the vertical
        term only used when it points away from every threat, opens every threat at
        >= v_open / v_escape.
S3  Triangle lemma: with all pairwise distances >= d_safe > 0, at most one of the three
    vertices has an angle >= 90 deg; hence in every threatened pair at least one member
    satisfies the hypothesis of S2 (and the other is at least non-closing).
S4  Blackout lemma: a sensing blackout of length <= T_b that starts with d >= d_blackout
    cannot bring the pair below d_safe (drones hold position in FAILSAFE).

What is NOT proved: perception completeness/accuracy (eps_rel, tau_max, symmetric
detection) and the plant numbers are assumptions, validated empirically by
scripts/validate_assumptions.py on the HoloOcean logs.
"""

from __future__ import annotations

import sys
from dataclasses import replace

import z3

from common import Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig, SeparationThresholds

ENC = "formal/check_separation.py"


def params(cfg: FleetConfig):
    env, sep = cfg.env, cfg.sep
    c_max = env.c_max
    return {
        "dt": env.dt, "c_max": c_max, "eps_p": env.eps_rel + c_max * env.tau_max,
        "a_pair": 2.0 * env.a_brake, "w_rel": env.w_rel_max, "v_open": env.v_open,
        "d_warning": sep.d_warning, "d_safe": sep.d_safe,
    }


def zmax(a, b):
    return z3.If(a >= b, a, b)


def radial_vars(k: int):
    d = [z3.Real(f"d_{i}") for i in range(k + 1)]
    c = [z3.Real(f"c_{i}") for i in range(k + 1)]
    dh = [z3.Real(f"dh_{i}") for i in range(k + 1)]
    return d, c, dh


def trans(p, d, c, dh, i):
    react = dh[i] < p["d_warning"]
    return z3.And(
        c[i + 1] <= p["c_max"], c[i + 1] >= -p["c_max"],
        z3.Implies(react, c[i + 1] <= zmax(c[i] - p["a_pair"] * p["dt"], p["w_rel"] - p["v_open"])),
        d[i + 1] == d[i] - (c[i] + c[i + 1]) / 2 * p["dt"],
    )


def meas(p, d, dh, i):
    return z3.And(dh[i] >= d[i] - p["eps_p"], dh[i] <= d[i] + p["eps_p"])


def bmc(p, K: int):
    d, c, dh = radial_vars(K)
    cons = [d[0] >= p["d_warning"] + p["eps_p"] + 0.1, c[0] <= p["c_max"], c[0] >= -p["c_max"]]
    for i in range(K):
        cons += [meas(p, d, dh, i), trans(p, d, c, dh, i)]
    cons.append(z3.Or(*[d[i] < p["d_safe"] for i in range(K + 1)]))
    return cons


def kinduction_step(p, k: int):
    d, c, dh = radial_vars(k)
    cons = [c[0] <= p["c_max"], c[0] >= -p["c_max"]]
    for i in range(k):
        cons += [d[i] >= p["d_safe"], meas(p, d, dh, i), trans(p, d, c, dh, i)]
    cons.append(d[k] < p["d_safe"])
    return cons


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("separation")
    p = params(cfg)
    if verbose:
        print("S1 parameters:", {k: round(v, 4) for k, v in p.items()})
    K = 120
    rep.add(check("S1a BMC: no violation within 12 s from any admissible start",
                  "Init & T^K -> G_[0,K] d >= d_safe", ENC, bmc(p, K)), verbose)
    proved_k = None
    for k in range(1, 61):
        r = check(f"S1b k-induction step k={k}", "P^k & T^k -> P'", ENC, kinduction_step(p, k))
        if r.verdict == "unsat":
            proved_k = k
            r.prop = f"S1b k-induction: G(d >= d_safe) for all time (k={k})"
            r.formula = "P(s_0..s_{k-1}) & T -> P(s_k), with base case covered by S1a (K >= k)"
            rep.add(r, verbose)
            break
    if proved_k is None:
        rep.add(check("S1b k-induction (no k <= 60 found)", "", ENC, [z3.BoolVal(True)], expect="unsat",
                      note="induction did not close"), verbose)
    # minimal warning distance that still verifies (sensitivity, informative)
    lo, hi = p["d_safe"], p["d_warning"]
    for _ in range(14):
        mid = (lo + hi) / 2
        pm = dict(p, d_warning=mid)
        if check("", "", ENC, bmc(pm, K)).verdict == "unsat":
            hi = mid
        else:
            lo = mid
    if verbose:
        print(f"    sensitivity: smallest d_warning verified by BMC = {hi:.3f} m (configured {p['d_warning']})")
    rep.results[-1].note = (rep.results[-1].note or "") + f" min verified d_warning ~ {hi:.3f} m"
    # mutation: too small a warning distance must produce a counterexample trace
    pm = dict(p, d_warning=max(p["d_safe"] + 0.2, hi - 0.25))
    rep.add(check(f"S1m mutation: d_warning={pm['d_warning']:.2f} must violate P1",
                  "expect counterexample (trace of d_i, c_i)", ENC, bmc(pm, K), expect="sat"), verbose)

    # ---------------------------------------------------------------- S2 opening lemma
    env = cfg.env
    u1 = [z3.Real(f"u1{a}") for a in "xyz"]
    u2 = [z3.Real(f"u2{a}") for a in "xyz"]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))  # noqa: E731
    unit = [dot(u1, u1) == 1, dot(u2, u2) == 1]
    cth = dot(u1, u2)
    n = z3.Real("n")                      # n = |u1 + u2|
    s_sum = [x + y for x, y in zip(u1, u2)]
    norm_n = [n > 0, n * n == dot(s_sum, s_sum)]
    s_w = 0.4
    # (a) SW filter witness v = -s_w * (u1+u2)/n ; v.u_j = -s_w * (1 + c)/n
    rep.add(check("S2a SW filter feasible with v_open when threats <= 90 deg apart",
                  "u1.u2 >= 0 -> exists v, |v| <= cap: v.u_j <= -v_open (witness -0.4 (u1+u2)/|u1+u2|)", ENC,
                  unit + norm_n + [cth >= 0, z3.Or(-s_w * (1 + cth) / n > -env.v_open)]), verbose)
    # (b) CA escape opening (vertical term only when it points away from every threat).
    # Square-root-free encoding in coordinates aligned with the bisector b (no loss of generality:
    # the construction and the "away" condition are invariant under rotations about z):
    #   b = (-bh, 0, bz), q = cos(phi) (bz, 0, bh) + sin(phi) (0, 1, 0)  (unit, orthogonal to b)
    #   u_{1,2} = -alpha b +/- beta q, alpha = cos(theta/2) >= cos(45 deg), beta = sin(theta/2)
    #   e = b + w vz z_hat (unnormalised);  claim  e.u_j <= -kappa |e|  <=>  e.u_j <= 0 & (e.u_j)^2 >= kappa^2 |e|^2
    w = env.escape_vertical_weight
    kappa = env.v_open / env.v_escape
    bz, bh, cp, sp, al, be, vz = z3.Reals("bz bh cphi sphi alpha beta vz")
    b_vec = [-bh, 0, bz]
    q_vec = [cp * bz, sp, cp * bh]
    U = [[-al * b_vec[k] + sgn * be * q_vec[k] for k in range(3)] for sgn in (1, -1)]
    e_vec = [-bh, 0, bz + w * vz]
    geo = [bh >= 0, bh * bh + bz * bz == 1, cp * cp + sp * sp == 1, al >= 0, be >= 0, al * al + be * be == 1,
           2 * al * al >= 1]                       # theta <= 90 deg
    vz_dom = z3.Or(vz == 0, vz == 1, vz == -1)
    away = z3.And(vz * U[0][2] <= 0, vz * U[1][2] <= 0)
    viol = z3.Or(*[z3.Or(dot(e_vec, u) > 0, dot(e_vec, u) * dot(e_vec, u) < kappa * kappa * dot(e_vec, e_vec))
                   for u in U])
    rep.add(check("S2b CA escape opens every threat at >= v_open/v_escape when <= 90 deg apart",
                  "theta <= 90 & vertical term away from threats -> e.u_j <= -(v_open/v_escape)|e|", ENC,
                  geo + [vz_dom, away, viol]), verbose)
    # single threat (trivial but explicit): e = normalise(-u + w vz z), vz away
    ne = z3.Real("ne")
    rep.add(check("S2c CA escape opens a single threat at >= v_open/v_escape",
                  "e = normalise(-u + w vz z), vz.u_z <= 0 -> e.u <= -v_open/v_escape", ENC,
                  [dot(u1, u1) == 1, z3.Or(vz == 0, vz == 1, vz == -1), vz * u1[2] <= 0, ne > 0,
                   ne * ne == dot([-u1[0], -u1[1], -u1[2] + w * vz], [-u1[0], -u1[1], -u1[2] + w * vz]),
                   dot([-u1[0], -u1[1], -u1[2] + w * vz], u1) > -kappa * ne]), verbose)
    # mutation: vertical term regardless of threat geometry must break S2b for some configuration
    w_bad = 1.5
    e_bad = [-bh, 0, bz + w_bad * vz]
    viol_bad = z3.Or(*[z3.Or(dot(e_bad, u) > 0, dot(e_bad, u) * dot(e_bad, u) < kappa * kappa * dot(e_bad, e_bad))
                       for u in U])
    rep.add(check("S2m mutation: strong vertical escape regardless of threat geometry must break S2b",
                  "expect counterexample", ENC, geo + [z3.Or(vz == 1, vz == -1), viol_bad], expect="sat",
                  note="vertical weight 1.5 without the 'away from every threat' condition"), verbose)

    # ---------------------------------------------------------------- S3 triangle lemma
    P = [[z3.Real(f"p{i}{a}") for a in "xyz"] for i in range(3)]
    sub = lambda a, b: [x - y for x, y in zip(a, b)]  # noqa: E731
    dsafe2 = cfg.sep.d_safe ** 2
    sep_c = [dot(sub(P[i], P[j]), sub(P[i], P[j])) >= dsafe2 for i, j in ((0, 1), (0, 2), (1, 2))]
    obtuse = lambda i, j, k: dot(sub(P[j], P[i]), sub(P[k], P[i])) <= 0  # noqa: E731
    rep.add(check("S3 at most one vertex angle >= 90 deg (pairwise d >= d_safe)",
                  "d_ij >= d_safe -> not(angle_i >= 90 and angle_j >= 90)", ENC,
                  sep_c + [z3.Or(z3.And(obtuse(0, 1, 2), obtuse(1, 0, 2)), z3.And(obtuse(0, 1, 2), obtuse(2, 0, 1)),
                                 z3.And(obtuse(1, 0, 2), obtuse(2, 0, 1)))]), verbose)

    # ---------------------------------------------------------------- S4 blackout lemma
    T_b = 0.8
    pb = dict(p)
    Kb = int(round((T_b + cfg.env.tau_max) / pb["dt"])) + 10
    d, c, dh = radial_vars(Kb)
    d_blackout = round(pb["d_safe"] + pb["c_max"] * cfg.env.tau_max + pb["c_max"] ** 2 / (2 * pb["a_pair"])
                       + pb["w_rel"] * (T_b + 1.0) + 0.05, 2)
    cons = [d[0] >= d_blackout, c[0] <= pb["c_max"], c[0] >= -pb["c_max"]]
    n_react = int(round(cfg.env.tau_max / pb["dt"]))
    for i in range(Kb):
        hold = z3.BoolVal(i >= n_react)              # FAILSAFE engaged after tau_max: both hold position
        cons += [c[i + 1] <= pb["c_max"], c[i + 1] >= -pb["c_max"],
                 z3.Implies(hold, c[i + 1] <= zmax(c[i] - pb["a_pair"] * pb["dt"], pb["w_rel"])),
                 d[i + 1] == d[i] - (c[i] + c[i + 1]) / 2 * pb["dt"]]
    cons.append(z3.Or(*[d[i] < pb["d_safe"] for i in range(Kb + 1)]))
    rep.add(check(f"S4 blackout <= {T_b}s starting at d >= {d_blackout} m keeps d >= d_safe",
                  "FAILSAFE hold with unrejected drift w_rel", ENC, cons,
                  note=f"d_blackout = {d_blackout} m (blackouts closer than this are covered empirically only)"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"separation: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
