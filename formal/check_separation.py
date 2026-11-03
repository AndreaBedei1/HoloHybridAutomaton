"""P1 - inter-vehicle separation:  G( forall i != j : d_ij >= d_safe )   (v2: six wide-beam sonars).

All numbers come from holo_fleet.config; the sensing lemmas evaluate the deployed perception and
escape code (holo_fleet.perception.sonar_geometry, holo_fleet.control.escape) - nothing is copied.

S0  Conservative distance (numeric): scenes of one to three BlueROV2 hulls (any heading, roll / pitch within the
    tilt assumption of perception/distance_bound.py; uniform, cube-corner, cube-edge, near-field and square-formation
    placements), the detection boundary of every sector drawn inside its uncertainty band (60 deg +- one octree leaf)
    and an independent echo error in [-eps_near, +eps_far] per sector: the deployed association-free estimate (the
    smallest target bound of perception/targets.py) is never above the distance of the nearest hull.  With staleness
    the guards use d_hat - c_max * age, still below the current distance.  Mutations that must be caught
    overestimating: S0m (centre assumed inside every member cone).  Reported for information: bound A on every echo
    without touch-set reasoning (not found above the truth in the sampled scenes, but without a certificate), and the
    deployed estimate on scenes with an arbitrary hull attitude (the role of the tilt assumption).
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

NOT proved here (assumptions, measured in results/reference/ASSUMPTIONS.md): the sonar detects a BlueROV2
anywhere in its 60 deg cone within range (probe F), both drones of a pair detect each other (six
cones cover the sphere), a_brake, w_rel_max (a RESIDUAL drift after the low-level compensation: it holds
while the commanded velocities are control-feasible against the raw current, control/current_envelope.py, and
the onboard monitor declares ENVELOPE_VIOLATION otherwise) and the velocity caps of the plant, no simultaneous
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


def _level_rot(rng, tilt_deg):
    """Any heading and a tilt (angle between the hull's z axis and the observer's) up to tilt_deg about a random
    horizontal axis: the hull assumption of perception/distance_bound.py."""
    yaw = rng.uniform(-math.pi, math.pi)
    t = math.radians(rng.uniform(0.0, tilt_deg))
    ax = rng.uniform(-math.pi, math.pi)
    k = np.array([math.cos(ax), math.sin(ax), 0.0])
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    tilt = np.eye(3) + math.sin(t) * K + (1.0 - math.cos(t)) * (K @ K)           # Rodrigues
    cy, sy = math.cos(yaw), math.sin(yaw)
    return tilt @ np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])


def scene_ranges(hulls, rng, cfg: FleetConfig, errors: str = "random", boundary: str = "random"):
    """Nearest echo per sector of a scene of hull point clouds (body frame of the observer).  The sector's detection
    boundary is drawn per scene inside the uncertainty band of the model, 60 deg +- one octree leaf (points beyond it
    are never seen, points inside 60 deg - leaf always), and every echo gets its own error in [-eps_near, +eps_far]
    ("extreme": one of the two limits, "far": always +eps_far); ``boundary`` "narrow" puts every detection boundary at
    60 deg - leaf (fewest detections: the worst case for the soundness of the estimate)."""
    model, env = cfg.perc.sonar, cfg.env
    ranges = {}
    for s in sg.SECTORS:
        k = -1.0 if boundary == "narrow" else rng.uniform(-1.0, 1.0)
        best = math.inf
        for P in hulls:
            sv = P - sg.MOUNTS[s]
            r = np.linalg.norm(sv, axis=1)
            ang = np.degrees(np.arccos(np.clip((sv @ sg.AXES[s]) / r, -1.0, 1.0)))
            lim = model.half_angle_deg + k * np.degrees(np.arctan2(sg.OCTREE_LEAF_M, np.maximum(r, model.range_min)))
            inc = (ang <= lim) & (r >= model.range_min) & (r <= model.range_max)
            if inc.any():
                best = min(best, float(r[inc].min()))
        if best < math.inf:
            e = (env.eps_range_far if errors == "far" else
                 rng.choice([-env.eps_range_near, env.eps_range_far]) if errors == "extreme"
                 else rng.uniform(-env.eps_range_near, env.eps_range_far))
            ranges[s] = best + float(e)
    return ranges


def _direction(rng, kind):
    u = rng.normal(size=3)
    if kind == "corner":                          # near the eight cube corners: no cone contains the centre for sure
        u = np.sign(u) / math.sqrt(3.0) + 0.12 * rng.normal(size=3)
    elif kind == "edge":                          # near the twelve cube edges: two adjacent cones overlap
        ax = rng.choice(3)
        u = np.sign(u)
        u[ax] = 0.0
        u = u / np.linalg.norm(u) + 0.15 * rng.normal(size=3)
    return u / np.linalg.norm(u)


def bound_scenes(cfg: FleetConfig, n_scenes: int, bound, seed: int = 1, kind: str = "single", tilt_deg: float = None,
                 n_hulls=(1, 1), dist=(1.0, 8.0), errors: str = "random", boundary: str = "random"):
    """Scenes of 1..3 hulls -> (true distance of the nearest hull, bound evaluated on the nearest echo of every
    sector).  ``kind`` places the hull centres: single (uniform directions), corner, edge, or square (two neighbours
    90 deg apart in the horizontal plane, the formation case that motivated the association-free bound)."""
    from holo_fleet.perception.distance_bound import MAX_REL_TILT_DEG

    tilt = MAX_REL_TILT_DEG if tilt_deg is None else tilt_deg
    rng = np.random.default_rng(seed)
    H = _hull_points(16)
    out = []
    for _ in range(n_scenes):
        m = int(rng.integers(n_hulls[0], n_hulls[1] + 1))
        centres = []
        if kind == "square":
            base = rng.uniform(0.0, 2.0 * math.pi)
            for k in range(m):
                a = base + k * math.pi / 2.0 + np.radians(rng.normal(0.0, 6.0))
                r = rng.uniform(*dist)
                centres.append(r * np.array([math.cos(a), math.sin(a), rng.normal(0.0, 0.1)]))
        else:
            for _k in range(m):
                centres.append(rng.uniform(*dist) * _direction(rng, kind))
        rot = _rand_rot if tilt >= 90.0 else (lambda g: _level_rot(g, tilt))
        hulls = [c + H @ rot(rng).T for c in centres]
        ranges = scene_ranges(hulls, rng, cfg, errors, boundary)
        if not ranges:
            continue
        out.append((min(float(np.linalg.norm(c)) for c in centres), bound(ranges)))
    return np.array(out)


def deployed_bound(cfg: FleetConfig):
    """The deployed estimate (perception/distance_bound.py, as used by perception/targets.py), fresh readings."""
    from holo_fleet.perception.distance_bound import nearest_distance_lower

    return lambda r: nearest_distance_lower(r, {s: 0.0 for s in r}, cfg.env.eps_range_far, 0.0, cfg.perc.sonar)


def v1_bound(ranges, eps_far=0.12):
    """Mutation: the centre assumed inside every member cone (r + R_IN, no certification) and the
    tightest member kept - a hull corner reaching into a cone makes it overestimate."""
    return max(sg.centre_distance_lower(r, s, eps_far, centre_in_cone=True) for s, r in ranges.items())


def naive_union_bound(ranges, eps_far=0.12):
    """Mutation: bound A for every echo, no touch-set reasoning (the centre assumed inside the cone of the sector with
    the nearest echo) - wrong in the near field and at the cube corners."""
    return min(sg.centre_distance_lower(r, s, eps_far, centre_in_cone=True) for s, r in ranges.items())


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
    est = deployed_bound(cfg)
    sets = [("single hull, 1-8 m", dict(kind="single", n_hulls=(1, 1), dist=(1.0, 8.0))),
            ("single hull near the cube corners, 1-3 m", dict(kind="corner", n_hulls=(1, 1), dist=(1.0, 3.0))),
            ("single hull near the cube edges, 1-4 m", dict(kind="edge", n_hulls=(1, 1), dist=(1.0, 4.0))),
            ("near field, 0.7-1.6 m", dict(kind="single", n_hulls=(1, 2), dist=(0.7, 1.6))),
            ("2-3 hulls, 1-6 m", dict(kind="single", n_hulls=(2, 3), dist=(1.0, 6.0))),
            ("2-3 hulls at cube edges, 1.5-4 m", dict(kind="edge", n_hulls=(2, 3), dist=(1.5, 4.0))),
            ("square-formation neighbours 90 deg apart, 2.5-4.5 m", dict(kind="square", n_hulls=(2, 2), dist=(2.5, 4.5))),
            ("square-formation neighbours, extreme range errors", dict(kind="square", n_hulls=(2, 3), dist=(2.5, 4.5),
                                                                      errors="extreme")),
            ("worst case (narrowest boundary, +eps_far): corners 0.7-3 m", dict(kind="corner", n_hulls=(1, 2),
                                                                               dist=(0.7, 3.0), errors="far",
                                                                               boundary="narrow")),
            ("worst case: edges 0.7-4 m", dict(kind="edge", n_hulls=(1, 3), dist=(0.7, 4.0), errors="far",
                                                boundary="narrow")),
            ("worst case: square neighbours", dict(kind="square", n_hulls=(2, 3), dist=(2.5, 4.5), errors="far",
                                                    boundary="narrow"))]
    worst, total, notes = math.inf, 0, []
    per = max(400, n_poses // len(sets))
    for k, (label, kw) in enumerate(sets):
        S = bound_scenes(cfg, per, est, seed=100 + k, **kw)
        g = S[:, 0] - S[:, 1]
        worst, total = min(worst, float(g.min())), total + len(S)
        notes.append(f"{label}: min(d - d_hat) {g.min():.3f}, p50 {np.percentile(g, 50):.2f}")
    r = CheckResult("S0 onboard distance never above the true distance of the nearest hull (numeric, multi-hull scenes)",
                    "forall sampled scenes/errors/detection boundaries: d_hat <= min_O d(O)",
                    "perception/distance_bound.py (perception/targets.py)",
                    "holds", verdict="holds" if worst >= 0 else "violated", passed=bool(worst >= 0),
                    seconds=round(time.time() - t0, 1),
                    note=f"{total} scenes, hull tilt <= distance_bound.MAX_REL_TILT_DEG; " + "; ".join(notes))
    rep.add(r, verbose)
    if verbose:
        print("       " + r.note)
    S1 = bound_scenes(cfg, max(3000, n_poses // 10), lambda r: v1_bound(r, cfg.env.eps_range_far), seed=9, kind="corner",
                      dist=(1.0, 3.0))
    g1 = S1[:, 0] - S1[:, 1]
    rep.add(CheckResult("S0m mutation: uncertified bound (centre assumed inside every member cone) must overestimate",
                        "expect: some scene with d_hat > d", ENC, "violated",
                        verdict="violated" if g1.min() < 0 else "holds", passed=bool(g1.min() < 0),
                        note=f"worst overestimate {(-g1.min()):.3f} m in {int((g1 < 0).sum())} of {len(S1)} scenes"), verbose)
    S2 = bound_scenes(cfg, max(3000, n_poses // 10), lambda r: naive_union_bound(r, cfg.env.eps_range_far), seed=10,
                      kind="corner", n_hulls=(1, 2), dist=(0.6, 3.0), errors="far", boundary="narrow")
    g2 = S2[:, 0] - S2[:, 1]
    if verbose:                                   # informational: bound A on every echo has no certificate
        print(f"       S0 (info) bound A on every echo, worst-case corner scenes: min(d - d_hat) {g2.min():.3f} m, "
              f"{int((g2 < 0).sum())} of {len(S2)} above the true distance (not certified: no touch-set reasoning)")
    S3 = bound_scenes(cfg, max(2000, n_poses // 15), est, seed=11, kind="corner", n_hulls=(1, 1), dist=(1.0, 3.0),
                      tilt_deg=90.0)
    g3 = S3[:, 0] - S3[:, 1]
    if verbose:                                   # informational: the role of the hull-tilt assumption
        print(f"       S0 outside the tilt assumption (any attitude, cube corners): min(d - d_hat) {g3.min():.3f} m, "
              f"{int((g3 < 0).sum())} of {len(S3)} scenes above the true distance")
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
    # one-sided reaction (one drone of the pair does not react: a missed detection, a drone in FAILSAFE holding its
    # own position, a non-cooperative vehicle): only one drone removes the closing component.  With the configured,
    # conservative a_brake (0.5 m/s^2) the BMC is SAT (reported: P1 is then NOT guaranteed by this model); with the
    # per-drone deceleration calibrated in closed loop in the simulator (Envelope.a_brake_measured, 0.743 m/s^2 from the
    # stop time, results/calibration/plant_calibration.json) it is UNSAT.  The one-sided claim therefore rests on
    # the simulator calibration, not on the configured margin; it is to be re-measured on hardware.
    p1s = dict(p, a_pair=env.a_brake)
    rep.add(check("S1o one-sided reaction at the configured a_brake: reported (not guaranteed)",
                  "Init & T^K -> G d >= d_safe, a_pair = a_brake", ENC, bmc(p1s, K), expect="sat"), verbose)
    p1c = dict(p, a_pair=env.a_brake_measured)
    rep.add(check(f"S1d one-sided reaction at the calibrated deceleration {env.a_brake_measured} m/s^2: "
                  "no violation within 4 s", "Init & T^K -> G d >= d_safe, a_pair = a_brake_measured", ENC,
                  bmc(p1c, K)), verbose)
    lo1, hi1 = p["d_safe"], 4.0
    for _ in range(14):
        mid = (lo1 + hi1) / 2
        if check("", "", ENC, bmc(dict(p1s, d_warning=mid), K)).verdict == "unsat":
            hi1 = mid
        else:
            lo1 = mid
    if verbose:
        print(f"       one-sided at the configured a_brake: smallest verified d_warning = {hi1:.3f} m "
              f"(configured {p['d_warning']})")
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
