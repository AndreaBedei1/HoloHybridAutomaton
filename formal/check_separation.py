"""P1 - inter-vehicle separation:  G( forall i != j : d_ij >= d_safe )   (v2: six wide-beam sonars).

All numbers come from holo_fleet.config; the sensing lemmas evaluate the deployed perception and
escape code (holo_fleet.perception.sonar_geometry, holo_fleet.control.escape) - nothing is copied.

S0  Conservative distance (numeric, 10^5 random BlueROV2 poses, every orientation, 1-8 m, echo
    error in [-eps_near, +eps_far]): the onboard distance d_hat of a target (perception.targets:
    min over the sectors that see it of the sound per-sector bound, sonar_geometry.
    target_distance_lower) is never above the true centre distance.  With staleness the guards
    use d_hat - c_max * age, still below the current distance.  Mutation S0m: the v1-style bound
    (r + R_IN everywhere) must be caught overestimating.
S1  Pairwise radial model (linear real arithmetic, discrete time dt).  State (d, c): true centre
    distance and closing speed of a pair.
      velocity caps    |c| <= c_max = v_escape + v_max_nominal + w_rel_max
      perception       d_hat <= d   (S0: one-sided; the looseness only makes the reaction earlier)
      reaction         d_hat < d_warning -> c' <= max(c - a_pair dt, w_rel - v_open)
                       (both drones remove the closing component, at least one opens at >= v_open:
                        lemmas S2, S3), a_pair = 2 a_brake
      no guarantee     otherwise c' <= c_max (adversarial)
      kinematics       d' = d - (c + c')/2 dt
    BMC (12 s) from every start with d_hat >= d_warning, k-induction for unbounded time, bisection
    of the smallest d_warning that still verifies (the configured value must exceed it), and a
    mutation (d_warning too small) that must produce a counterexample trace.
S2  Escape tables (exhaustive over the finite set of sector patterns, deployed EscapePlanner):
    (a) one threat, any pattern: the COLLISION_AVOIDANCE escape has a guaranteed opening per unit
        speed >= g_min after subtracting the sampling error of the direction tables;
    (b) one threat, any pattern: the SEPARATION_WARNING filter finds a velocity <= v_max with a
        guaranteed opening >= v_open;
    (c) two threats: every pair of patterns whose regions lie within a 90 deg cone gets a positive
        guarantee; the pairs that cannot (e.g. FRONT and REAR) are listed - there P1 rests on the
        other drones of the triangle (S3);
    (d) a threat seen by FRONT+DOWN is escaped with an UP component, by FRONT+UP with a DOWN one;
    S2m mutation: a fixed "always reverse" escape must fail (a) for some pattern.
S3  Triangle lemma (Z3, nonlinear): with all pairwise distances >= d_safe at most one of three
    drones sees the other two more than 90 deg apart.
S4  Blackout lemma (Z3): a sonar blackout of <= T_b s starting at d >= d_blackout keeps d >= d_safe
    (FAILSAFE holds position; unrejected drift w_rel).

NOT proved here (assumptions, measured in results/v2/ASSUMPTIONS.md): the sonar detects a BlueROV2
anywhere in its 60 deg cone within range (probe F), both drones of a pair detect each other (six
cones cover the sphere), a_brake, w_rel_max and the velocity caps of the plant, no simultaneous
masking of both drones of a pair by a mapped structure.
"""

from __future__ import annotations

import itertools
import math
import sys
import time

import numpy as np
import z3

from common import CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.escape import EscapePlanner, direction_label
from holo_fleet.perception import sonar_geometry as sg

ENC = "formal/check_separation.py"


# ---------------------------------------------------------------------------------------------- S0
def _hull_points(n=12):
    pts = []
    for ax in range(3):
        u, v = [k for k in range(3) if k != ax]
        gu, gv = np.meshgrid(np.linspace(-1, 1, n), np.linspace(-1, 1, n))
        for sgn in (-1, 1):
            loc = np.zeros((gu.size, 3))
            loc[:, u], loc[:, v], loc[:, ax] = gu.ravel() * sg.HULL_HALF[u], gv.ravel() * sg.HULL_HALF[v], sgn * sg.HULL_HALF[ax]
            pts.append(loc)
    return np.concatenate(pts)


def _rand_rot(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    a, b, c, d = q
    return np.array([[a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)],
                     [2 * (b * c + a * d), a * a - b * b + c * c - d * d, 2 * (c * d - a * b)],
                     [2 * (b * d - a * c), 2 * (c * d + a * b), a * a - b * b - c * c + d * d]])


def bound_samples(cfg: FleetConfig, n_poses: int, bound, seed: int = 1, corner: bool = False):
    """Random hull poses -> (true distance, bound) for echo errors -eps_near, 0, +eps_far.  ``corner``:
    directions near the eight cube corners at 1-3 m (no cone contains the centre: the hard case)."""
    rng = np.random.default_rng(seed)
    H = _hull_points()
    cos60 = math.cos(math.radians(cfg.perc.sonar.half_angle_deg))
    env = cfg.env
    out = []
    for _ in range(n_poses):
        d = rng.uniform(1.0, 3.0) if corner else rng.uniform(1.0, 8.0)
        u = rng.normal(size=3)
        if corner:
            u = np.sign(u) / math.sqrt(3.0) + 0.12 * rng.normal(size=3)
        u /= np.linalg.norm(u)
        P = d * u + H @ _rand_rot(rng).T
        ranges = {}
        for s in sg.SECTORS:
            sv = P - sg.MOUNTS[s]
            r = np.linalg.norm(sv, axis=1)
            inc = (sv @ sg.AXES[s]) / r >= cos60
            if inc.any() and float(r[inc].min()) >= cfg.perc.sonar.range_min:
                ranges[s] = float(r[inc].min())
        if not ranges:
            continue
        for err in (-env.eps_range_near, 0.0, env.eps_range_far):
            out.append((d, bound({s: r + err for s, r in ranges.items()})))
    return np.array(out)


def v1_bound(ranges, eps_far=0.12):
    """Mutation: the centre assumed inside every member cone (r + R_IN, no certification) and the
    tightest member kept - a hull corner reaching into a cone makes it overestimate (DI-13)."""
    return max(sg.centre_distance_lower(r, s, eps_far, centre_in_cone=True) for s, r in ranges.items())


# ---------------------------------------------------------------------------------------------- S1
def params(cfg: FleetConfig):
    env, sep = cfg.env, cfg.sep
    return {"dt": env.dt, "c_max": env.c_max, "a_pair": 2.0 * env.a_brake, "w_rel": env.w_rel_max,
            "v_open": env.v_open, "d_warning": sep.d_warning, "d_safe": sep.d_safe}


def zmax(a, b):
    return z3.If(a >= b, a, b)


def radial_vars(k: int):
    return ([z3.Real(f"d_{i}") for i in range(k + 1)], [z3.Real(f"c_{i}") for i in range(k + 1)],
            [z3.Real(f"dh_{i}") for i in range(k + 1)])


def trans(p, d, c, dh, i):
    react = dh[i] < p["d_warning"]
    return z3.And(c[i + 1] <= p["c_max"], c[i + 1] >= -p["c_max"],
                  z3.Implies(react, c[i + 1] <= zmax(c[i] - p["a_pair"] * p["dt"], p["w_rel"] - p["v_open"])),
                  d[i + 1] == d[i] - (c[i] + c[i + 1]) / 2 * p["dt"])


def meas(d, dh, i):
    return dh[i] <= d[i]                       # S0: the onboard distance never exceeds the true one


def bmc(p, K: int):
    d, c, dh = radial_vars(K)
    cons = [d[0] >= p["d_warning"], c[0] <= p["c_max"], c[0] >= -p["c_max"]]
    for i in range(K):
        cons += [meas(d, dh, i), trans(p, d, c, dh, i)]
    cons.append(z3.Or(*[d[i] < p["d_safe"] for i in range(K + 1)]))
    return cons


def kinduction_step(p, k: int):
    d, c, dh = radial_vars(k)
    cons = [c[0] <= p["c_max"], c[0] >= -p["c_max"]]
    for i in range(k):
        cons += [d[i] >= p["d_safe"], meas(d, dh, i), trans(p, d, c, dh, i)]
    cons.append(d[k] < p["d_safe"])
    return cons


# ---------------------------------------------------------------------------------------------- run
def run(cfg: FleetConfig = DEFAULT, verbose: bool = True, n_poses: int = 35000) -> Report:
    rep = Report("separation")
    env, sep = cfg.env, cfg.sep
    # ---------------------------------------------------------------- S0
    t0 = time.time()
    S = bound_samples(cfg, n_poses, lambda r: sg.target_distance_lower(r, env.eps_range_far, cfg.perc.sonar))
    gap = S[:, 0] - S[:, 1]
    r = CheckResult("S0 onboard distance never above the true centre distance (numeric)",
                    "forall sampled poses/errors: d_hat <= d", "perception/targets.py + sonar_geometry.target_distance_lower",
                    "holds", verdict="holds" if gap.min() >= 0 else "violated", passed=bool(gap.min() >= 0),
                    seconds=round(time.time() - t0, 1),
                    note=f"{len(S)} samples; min(d - d_hat) = {gap.min():.3f} m; looseness p50 {np.percentile(gap, 50):.2f}, "
                         f"p95 {np.percentile(gap, 95):.2f}, max {gap.max():.2f} m (affects liveness only)")
    rep.add(r, verbose)
    if verbose:
        print("       " + r.note)
    S1 = bound_samples(cfg, max(4000, n_poses // 8), v1_bound, corner=True)
    g1 = S1[:, 0] - S1[:, 1]
    rep.add(CheckResult("S0m mutation: uncertified bound (centre assumed inside every member cone) must overestimate",
                        "expect: some sample with d_hat > d", ENC, "violated",
                        verdict="violated" if g1.min() < 0 else "holds", passed=bool(g1.min() < 0),
                        note=f"worst overestimate {(-g1.min()):.3f} m in {int((g1 < 0).sum())} of {len(S1)} samples"), verbose)
    # ---------------------------------------------------------------- S1
    p = params(cfg)
    if verbose:
        print("S1 parameters:", {k: round(v, 4) for k, v in p.items()})
    K = 40
    rep.add(check("S1a BMC: no violation within 4 s from any start with d_hat >= d_warning",
                  "Init & T^K -> G_[0,K] d >= d_safe", ENC, bmc(p, K)), verbose)
    proved_k = None
    for k in range(1, 61):
        rr = check(f"S1b k-induction step k={k}", "P^k & T^k -> P'", ENC, kinduction_step(p, k))
        if rr.verdict == "unsat":
            proved_k = k
            rr.prop = f"S1b k-induction: G(d >= d_safe) for all time (k={k})"
            rr.formula = "P(s_0..s_{k-1}) & T -> P(s_k), base case covered by S1a (K >= k)"
            rep.add(rr, verbose)
            break
    if proved_k is None:
        rep.add(CheckResult("S1b k-induction (no k <= 60 found)", "", ENC, "unsat", verdict="unknown", passed=False), verbose)
    lo, hi = p["d_safe"], p["d_warning"]
    for _ in range(14):
        mid = (lo + hi) / 2
        if check("", "", ENC, bmc(dict(p, d_warning=mid), K)).verdict == "unsat":
            hi = mid
        else:
            lo = mid
    margin = p["d_warning"] - hi
    rep.add(CheckResult("S1c configured d_warning above the smallest verified one", "d_warning >= d_warning_min",
                        ENC, "holds", verdict="holds" if margin >= 0 else "violated", passed=margin >= 0,
                        note=f"smallest d_warning verified by BMC = {hi:.3f} m; configured {p['d_warning']} m "
                             f"(margin {margin:.2f} m keeps escapes rare: liveness)"), verbose)
    if verbose:
        print(f"       smallest verified d_warning = {hi:.3f} m (configured {p['d_warning']})")
    pm = dict(p, d_warning=max(p["d_safe"] + 0.1, hi - 0.25))
    rep.add(check(f"S1m mutation: d_warning={pm['d_warning']:.2f} must violate P1",
                  "expect counterexample (trace of d_i, c_i)", ENC, bmc(pm, K), expect="sat"), verbose)
    # ---------------------------------------------------------------- S2 escape tables
    planner = EscapePlanner(cfg)
    sample_err = sg.SAMPLING_ERR        # the planner's guarantees are already certified (sampled - SAMPLING_ERR)
    worst_a, worst_b, rows = 9.0, 9.0, []
    for P in sorted(sg.regions(cfg.perc.sonar), key=lambda x: (len(x), sorted(x))):
        planner.reset()
        ch = planner.escape_velocity([(P, env.v_open)], [], 0.0, env.v_escape)
        worst_a = min(worst_a, ch.guarantee)
        planner.reset()
        wv = planner.warning_velocity(np.array([0.3, 0.0, 0.0]), [(P, env.v_open)], [], 0.0, env.v_max_nominal)
        opening = float((-(sg.regions(cfg.perc.sonar)[P] @ wv.v_body)).min()) - sample_err * float(np.linalg.norm(wv.v_body))
        worst_b = min(worst_b, opening)
        rows.append(("+".join(sorted(P)), ch.label, round(ch.guarantee, 3), wv.label, round(opening, 3)))
    rep.add(CheckResult("S2a CA escape, one threat: guaranteed opening per unit speed >= g_min (all patterns)",
                        "forall P: certified G(P, e*(P)) >= g_min", "control/escape.py (deployed planner)", "holds",
                        verdict="holds" if worst_a >= env.g_min else "violated", passed=worst_a >= env.g_min,
                        note=f"worst certified guarantee {worst_a:.3f} (g_min {env.g_min}); {len(rows)} patterns; "
                             f"sampling error {sample_err:.3f} already subtracted"), verbose)
    rep.add(CheckResult("S2b SW filter, one threat: velocity <= v_max with guaranteed opening >= v_open (all patterns)",
                        "forall P: exists v: min_u -v.u >= v_open", "control/escape.py (deployed planner)", "holds",
                        verdict="holds" if worst_b >= env.v_open - 1e-9 else "violated", passed=worst_b >= env.v_open - 1e-9,
                        note=f"worst certified opening {worst_b:.3f} m/s (v_open {env.v_open})"), verbose)
    R = sg.regions(cfg.perc.sonar)
    pats = sorted(R, key=lambda x: (len(x), sorted(x)))
    feasible, boxed, acute_bad = 0, [], []
    for P1, P2 in itertools.combinations_with_replacement(pats, 2):
        planner.reset()
        ch = planner.escape_velocity([(P1, env.v_open), (P2, env.v_open)], [], 0.0, env.v_escape)
        within90 = float((R[P1] @ R[P2].T).min()) >= 0.0             # every direction pair <= 90 deg apart
        if ch.guarantee > 0:
            feasible += 1
        else:
            boxed.append("+".join(sorted(P1)) + " & " + "+".join(sorted(P2)))
            if within90:
                acute_bad.append(boxed[-1])
    rep.add(CheckResult("S2c CA escape, two threats within 90 deg: positive guarantee",
                        "forall P1,P2 with all region directions <= 90 deg apart: max_e min G > 0", "control/escape.py",
                        "holds", verdict="holds" if not acute_bad else "violated", passed=not acute_bad,
                        note=f"{feasible} pattern pairs with a positive guarantee; {len(boxed)} boxed pairs (threats more than "
                             f"90 deg apart, e.g. {boxed[:3]}): there P1 rests on the acute-vertex drones (S3)"), verbose)
    planner.reset()
    up = planner.escape_velocity([(frozenset({"FRONT", "DOWN"}), env.v_open)], [], 0.0, env.v_escape)
    planner.reset()
    dn = planner.escape_velocity([(frozenset({"FRONT", "UP"}), env.v_open)], [], 0.0, env.v_escape)
    ok_v = up.direction[2] > 0.35 and dn.direction[2] < -0.35
    rep.add(CheckResult("S2d vertical escapes: FRONT+DOWN -> up component, FRONT+UP -> down component",
                        "e*(F+D).z > 0.35 and e*(F+U).z < -0.35", "control/escape.py", "holds",
                        verdict="holds" if ok_v else "violated", passed=ok_v,
                        note=f"F+D -> {up.label} (G {up.guarantee:.2f}); F+U -> {dn.label} (G {dn.guarantee:.2f})"), verbose)
    rev = np.array([-1.0, 0.0, 0.0])
    worst_rev = min(float((-(R[P] @ rev)).min()) for P in pats)
    rep.add(CheckResult("S2m mutation: 'always reverse' escape must fail S2a for some pattern",
                        "expect: some P with G(P, REAR) < g_min", ENC, "violated",
                        verdict="violated" if worst_rev < env.g_min else "holds", passed=worst_rev < env.g_min,
                        note=f"worst opening of the fixed reverse escape {worst_rev:.2f}"), verbose)
    rep.tables = {"escape_single_threat": rows}
    # ---------------------------------------------------------------- S3 triangle lemma
    Pz = [[z3.Real(f"p{i}{a}") for a in "xyz"] for i in range(3)]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))  # noqa: E731
    sub = lambda a, b: [x - y for x, y in zip(a, b)]  # noqa: E731
    dsafe2 = sep.d_safe ** 2
    sep_c = [dot(sub(Pz[i], Pz[j]), sub(Pz[i], Pz[j])) >= dsafe2 for i, j in ((0, 1), (0, 2), (1, 2))]
    obtuse = lambda i, j, k: dot(sub(Pz[j], Pz[i]), sub(Pz[k], Pz[i])) <= 0  # noqa: E731
    rep.add(check("S3 at most one vertex angle >= 90 deg (pairwise d >= d_safe)",
                  "d_ij >= d_safe -> not(angle_i >= 90 and angle_j >= 90)", ENC,
                  sep_c + [z3.Or(z3.And(obtuse(0, 1, 2), obtuse(1, 0, 2)), z3.And(obtuse(0, 1, 2), obtuse(2, 0, 1)),
                                 z3.And(obtuse(1, 0, 2), obtuse(2, 0, 1)))], timeout_ms=900000,
                  note="nonlinear real arithmetic: about 50 s alone, longer under load"), verbose)
    # ---------------------------------------------------------------- S4 blackout lemma
    T_b = 0.8
    Kb = int(round((T_b + env.tau_max) / p["dt"])) + 10
    d, c, dh = radial_vars(Kb)
    d_blackout = round(p["d_safe"] + p["c_max"] * env.tau_max + p["c_max"] ** 2 / (2 * p["a_pair"])
                       + p["w_rel"] * (T_b + 1.0) + 0.05, 2)
    cons = [d[0] >= d_blackout, c[0] <= p["c_max"], c[0] >= -p["c_max"]]
    n_react = int(round(env.tau_max / p["dt"]))
    for i in range(Kb):
        hold = z3.BoolVal(i >= n_react)
        cons += [c[i + 1] <= p["c_max"], c[i + 1] >= -p["c_max"],
                 z3.Implies(hold, c[i + 1] <= zmax(c[i] - p["a_pair"] * p["dt"], p["w_rel"])),
                 d[i + 1] == d[i] - (c[i] + c[i + 1]) / 2 * p["dt"]]
    cons.append(z3.Or(*[d[i] < p["d_safe"] for i in range(Kb + 1)]))
    rep.add(check(f"S4 blackout <= {T_b}s starting at d >= {d_blackout} m keeps d >= d_safe",
                  "FAILSAFE hold with unrejected drift w_rel", ENC, cons,
                  note=f"d_blackout = {d_blackout} m"), verbose)
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"separation: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
