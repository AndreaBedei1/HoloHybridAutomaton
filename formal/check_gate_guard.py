"""The gate guard of the avoidance layer and the composition of the gate with separation and collision avoidance.

st_lost_middle s5201 (b2-freeze-v11): two drones committed 0.5 s apart; drone 2 held before the critical region (CR) by
the stage-2 check, drone 0 behind it; the sonar placed drone 0 within the warning band (lower bound 1.96 m, true 3.05 m)
and the separation filter, finding no velocity that opens the REAR patterns at v_open, fell back to the escape and drove
drone 2 forward at 0.49 m/s into the CR without admission (53.7 s and 57.2 s).  The separate proofs (S1-S2 of the
separation, the latch M3, the stage-2 freshness S2, the fleet model F0-F2) each held; their composition did not exclude
the entry, because the avoidance layer was free to move a held drone along the gate axis.

The guard (holo_fleet/control/flows.gate_guard, control/escape.py): a drone not admitted to the CR (not cleared by the
stage-2 check, not past the centre) inside the keep-out box may only use avoidance velocities with v.a <= u_max =
GUARD_GAIN (s_hold - s) along the gate axis a, s_hold the stage-2 hold point.  When no allowed velocity opens every threat,
the separation filter holds (moving back along a only when that closes on no threat) and the collision avoidance
overrides the guard: collision first, reported as GATE_GUARD_OVERRIDE.

Checks:
  GG1  exhaustive numeric: the deployed planner, every sector pattern (also with one unhealthy sector), headings of the
       gate axis within +-15 deg of the body x axis, bounds u_max from -0.5 to 1 m/s, mission velocities: the separation
       filter never advances beyond the bound (v.a <= max(u_max, 0)) and the escape respects it unless it overrides, and
       overrides only when no allowed direction opens every threat.  Mutation GG1m: the v11 planner (no guard) advances.
  GG2  exhaustive numeric, pair lemma: two drones both guarded at their hold points (u_max = 0), every relative direction
       (2000 directions), every sector pattern consistent with it at each drone, headings +-10 deg: the better of the two
       has an allowed velocity with a guaranteed opening of at least g_pair (0.153 m/s, below v_open = 0.20 in about 3 % of
       the cases: the guard weakens the reaction premise of S1).  GG2b reports g_pair with both 0.1 m beyond the bound.
  GG5  the separation model of S1 with v_open replaced by g_pair and a one-sided reaction at the calibrated deceleration:
       BMC and k-induction (unbounded for the abstraction).  GG5o at the configured a_brake: counterexample (as S1o).
       GG5b both drones beyond the bound: bounded only (declared limit).
  GG3  along the gate axis (Z3, dt 0.1 s), unbounded: a braking-curve set is inductive for the guarded loop (the calibrated
       deceleration, the residual drift w_drift_max) and contains the mission hold of the stage-2 check, so the estimated
       centre stays within s_hold + 0.30 m, 0.079 m outside the CR face (GG3, GG3i); at the configured a_brake within
       s_hold + 0.32 m (GG3a, GG3ai).  Estimated position: the gate-frame navigation error is premise nav_err of the hold.
       GG3m mutation: without the guard (the v11 avoidance at v_escape) the bound is violated (BMC).
  GG3h declared limit (BMC, SAT): a guard hold that cannot move back (a threat behind) drifts at w_drift_max and crosses
       the bound in the worst case; logged at run time (GATE_GUARD_HOLD, the referee's CR entries).
  GG4  composition, n = 2, 3, 4 (check_fleet.Fleet, from the launch): two drones may commit together (no sonar precedence,
       the claims learnt one step late: the conditions of s5201), separation and collision avoidance may stop any drone at
       any step, and with the guard no avoidance moves a drone that is not admitted into the CR: never two drones in X.
       GG4o declared limit (SAT): a collision-avoidance override from the hold or the queue (the guard's infeasible case)
       can put a second drone into the CR; logged at run time.
The guard does not change the admitted drone's passage, the latch, the stage-2 check, the precedence or the formation.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import z3

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from common import CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.control.escape import EscapePlanner  # noqa: E402
from holo_fleet.control.flows import GUARD_GAIN, RELOC_MARGIN_M  # noqa: E402
from holo_fleet.perception.sonar_geometry import HULL_HALF, R_OUT, SECTORS, fibonacci_sphere  # noqa: E402

ENC = "formal/check_gate_guard.py"
U_MAX = (-0.5, -0.2, -0.05, 0.0, 0.05, 0.2, 0.5, 1.0)
HEADINGS = np.radians([-15.0, -7.5, 0.0, 7.5, 15.0])
V_DES = [np.array(v, float) for v in ([0, 0, 0], [0.3, 0, 0], [-0.2, 0, 0], [0.1, 0.2, 0], [0.1, -0.2, 0], [0, 0, 0.1])]


def _axis(psi: float) -> np.ndarray:
    return np.array([np.cos(psi), np.sin(psi), 0.0])


def _unhealthy_sets():
    return [()] + [(s,) for s in SECTORS]


def guard_bound_violations(cfg: FleetConfig = DEFAULT, guarded: bool = True, limit: int = 5):
    """GG1: (pattern, unhealthy, heading, u_max, layer) where the planner breaks the guard (or overrides wrongly)."""
    pl, env = EscapePlanner(cfg), cfg.env
    bad, n = [], 0
    for P in pl.regions:
        for un in _unhealthy_sets():
            T = [(P, env.v_open)]
            for psi in HEADINGS:
                a = _axis(psi)
                for u in U_MAX:
                    cap = (a, u) if guarded else None
                    for vd in V_DES:
                        pl.reset()
                        n += 1
                        w = pl.warning_velocity(vd, T, list(un), 0.0, env.v_max_nominal, cap=cap)
                        if float(w.v_body @ a) > max(u, 0.0) + 1e-9:
                            bad.append((sorted(P), un, round(float(np.degrees(psi)), 1), u, "SW", w.label))
                    pl.reset()
                    e = pl.escape_velocity(T, list(un), 0.0, env.v_escape, cap=cap)
                    adv = float(e.v_body @ a) > (u if guarded else max(u, 0.0)) + 1e-9
                    if adv and e.guard != "OVERRIDE":
                        bad.append((sorted(P), un, round(float(np.degrees(psi)), 1), u, "CA", e.label))
                    if e.guard == "OVERRIDE":
                        g = pl.guarantee(P, un)
                        if (pl._allowed((a, u), env.v_escape) & (g > 0)).any():
                            bad.append((sorted(P), un, round(float(np.degrees(psi)), 1), u, "CA override with an allowed opening", e.label))
                    if len(bad) >= limit:
                        return bad, n
    return bad, n


def _consistent(pl: EscapePlanner, u_body: np.ndarray, tol_deg: float = 4.0):
    c = np.cos(np.radians(tol_deg))
    return [P for P, R in pl.regions.items() if float((R @ u_body).max()) >= c]


def pair_lemma(cfg: FleetConfig = DEFAULT, n_dirs: int = 2000, u_max: float = 0.0):
    """GG2: for two guarded drones (bound u_max at both), every relative direction, every sector pattern consistent with it
    at each drone and headings +-10 deg, the larger of the two guaranteed opening speeds the better drone can reach with an
    allowed velocity (speeds of the separation filter, worst pattern).  Returns (minimum over the cases, fraction below
    v_open, cases)."""
    pl, env = EscapePlanner(cfg), cfg.env
    speeds = np.array([0.15, 0.25, 0.35, env.v_max_nominal])
    yaws = np.radians([-10.0, 0.0, 10.0])
    cache = {}

    def best(P, psi):
        key = (P, round(float(psi), 4))
        if key not in cache:
            gP, proj = pl.guarantee(P, []), pl.cands @ _axis(-psi)
            vals = [sp * gP[sp * proj <= u_max + 1e-9].max() for sp in speeds if (sp * proj <= u_max + 1e-9).any()]
            cache[key] = max(vals) if vals else -1.0
        return cache[key]

    def rot(psi):
        c, s_ = np.cos(psi), np.sin(psi)
        return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

    vals = []
    for uw in fibonacci_sphere(n_dirs):
        for pa in yaws:
            for pb in yaws:
                ba = min(best(P, pa) for P in _consistent(pl, rot(pa).T @ uw))
                bb = min(best(P, pb) for P in _consistent(pl, rot(pb).T @ (-uw)))
                vals.append(max(ba, bb))
    vals = np.array(vals)
    return float(vals.min()), float((vals < env.v_open - 1e-9).mean()), len(vals)


def pair_separation(cfg: FleetConfig, v_open_pair: float, a_pair: float, k_max: int = 40):
    """GG5: the separation model of check_separation (S1) with the opening of the guarded pair and a one-sided reaction:
    (BMC 4 s verdict, the smallest k of a k-induction proof or None)."""
    import check_separation as cs

    q = dict(cs.params(cfg), v_open=v_open_pair, a_pair=a_pair)
    S = z3.Solver()
    S.set("timeout", 300000)
    S.add(*cs.bmc(q, 40))
    r = S.check()
    for k in range(1, k_max + 1):
        S2 = z3.Solver()
        S2.set("timeout", 60000)
        S2.add(*cs.kinduction_step(q, k))
        if S2.check() == z3.unsat:
            return r, k
    return r, None


def axis_bmc(cfg: FleetConfig = DEFAULT, a_dec: float = None, guarded: bool = True, hold_still: bool = False,
             B: float = 0.30, horizon_s: float = 30.0):
    """GG3: Z3 BMC of the along-axis loop.  State (s, v) in the estimated gate frame, dt = env.dt.  Command vc <= v_escape,
    with the guard vc <= GUARD_GAIN (s_hold - s); hold_still: vc = 0 (a guard hold that cannot move back).  The plant
    reaches the command from below at once and slows towards it at a_dec at least; residual drift |w| <= w_drift_max.
    Initial set: the mission hold of the stage-2 check (s <= s_hold + w / k_track, v <= max(0, k_track (s_hold - s)) +
    0.1, v <= v_pass).  Returns the constraints whose satisfiability is a run beyond s_hold + B."""
    env, G = cfg.env, cfg.gate
    a_dec = env.a_brake_measured if a_dec is None else a_dec
    dt, W, kt = env.dt, env.w_drift_max, G.k_track
    s_hold = -(G.cr_half_len + float(HULL_HALF[0]) + RELOC_MARGIN_M)
    s_keep = -(G.cr_half_len + float(R_OUT) + G.nav_err_m)
    N = int(round(horizon_s / dt))
    s = [z3.Real(f"s_{i}") for i in range(N + 1)]
    v = [z3.Real(f"v_{i}") for i in range(N + 1)]
    vc = [z3.Real(f"vc_{i}") for i in range(N)]
    w = [z3.Real(f"w_{i}") for i in range(N)]
    q = lambda x: z3.RealVal(repr(float(x)))  # noqa: E731
    zmax = lambda x, y: z3.If(x >= y, x, y)  # noqa: E731
    cons = [s[0] >= q(s_keep), s[0] <= q(s_hold + W / kt), v[0] >= -q(env.v_escape), v[0] <= q(G.v_pass),
            v[0] <= zmax(q(0.0), q(kt) * (q(s_hold) - s[0])) + q(0.1)]
    for i in range(N):
        cons += [w[i] >= -q(W), w[i] <= q(W), vc[i] <= q(env.v_escape), vc[i] >= -q(env.v_escape)]
        if hold_still:
            cons.append(vc[i] == 0)
        elif guarded:
            cons.append(vc[i] <= q(GUARD_GAIN) * (q(s_hold) - s[i]))
        cons += [v[i + 1] <= zmax(vc[i], v[i] - q(a_dec * dt)), v[i + 1] >= -q(env.v_escape),
                 s[i + 1] == s[i] + q(dt) * (v[i] + v[i + 1]) / 2 + q(dt) * w[i]]
    cons.append(z3.Or([s[i] > q(s_hold + B) for i in range(N + 1)]))
    return cons, s_hold


def axis_invariant(cfg: FleetConfig = DEFAULT, a_dec: float = None, a_inv: float = 0.7, B: float = 0.30):
    """GG3 (unbounded): the set I = {s <= s_hold + B, |v| <= v_escape, v + w_max <= 0 or (v + w_max)^2 <= 2 a_inv (s_hold + B - s)}
    is inductive for the guarded along-axis loop of axis_bmc and contains its initial set (the mission hold).  Returns the
    constraints of the two obligations (each UNSAT when it holds): the inductive step, the initial containment."""
    env, G = cfg.env, cfg.gate
    a_dec = env.a_brake_measured if a_dec is None else a_dec
    dt, W, kt = env.dt, env.w_drift_max, G.k_track
    s_hold = -(G.cr_half_len + float(HULL_HALF[0]) + RELOC_MARGIN_M)
    s_keep = -(G.cr_half_len + float(R_OUT) + G.nav_err_m)
    q = lambda x: z3.RealVal(repr(float(x)))  # noqa: E731

    def inv(s_, v_):
        u = v_ + q(W)
        return z3.And(s_ <= q(s_hold + B), v_ >= -q(env.v_escape), v_ <= q(env.v_escape),
                      z3.Or(u <= 0, u * u <= q(2.0 * a_inv) * (q(s_hold + B) - s_)))
    s, v, vc, w, s2, v2 = z3.Reals("s v vc w s2 v2")
    step = [inv(s, v), w >= -q(W), w <= q(W), vc <= q(env.v_escape), vc >= -q(env.v_escape),
            vc <= q(GUARD_GAIN) * (q(s_hold) - s), z3.Or(v2 <= vc, v2 <= v - q(a_dec * dt)), v2 >= -q(env.v_escape),
            s2 == s + q(dt) * (v + v2) / 2 + q(dt) * w, z3.Not(inv(s2, v2))]
    init = [s >= q(s_keep), s <= q(s_hold + W / kt), v >= -q(env.v_escape), v <= q(G.v_pass),
            v <= z3.If(q(kt) * (q(s_hold) - s) >= 0, q(kt) * (q(s_hold) - s), q(0.0)) + q(0.1), z3.Not(inv(s, v))]
    return step, init, s_hold


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("gate_guard")
    env, G = cfg.env, cfg.gate
    t0 = time.time()
    bad, n = guard_bound_violations(cfg)
    rep.add(CheckResult("GG1 the guarded planner never advances beyond the bound along the gate axis (separation filter), the escape "
                        "only by an override, and an override only when no allowed direction opens every threat",
                        "forall P, unhealthy, heading, u_max, v_des: SW v.a <= max(u_max, 0); CA v.a <= u_max or (OVERRIDE and no "
                        "allowed opening)", "holo_fleet/control/escape.py (exhaustive over the sector patterns)", "holds",
                        "holds" if not bad else "violated", not bad, round(time.time() - t0, 3),
                        note=f"{n} cases; {bad[:2]}"), verbose)
    t0 = time.time()
    badm, nm = guard_bound_violations(cfg, guarded=False, limit=1)
    rep.add(CheckResult("GG1m mutation: the v11 planner (no guard) advances along the axis beyond the bound (s5201)",
                        "expect a case with v.a > max(u_max, 0)", ENC, "counterexample", "counterexample" if badm else "none",
                        bool(badm), round(time.time() - t0, 3), note=f"{badm[:1]}"), verbose)
    t0 = time.time()
    g0, frac0, n0 = pair_lemma(cfg, u_max=0.0)
    rep.add(CheckResult("GG2 pair lemma: of two guarded drones at their hold points (u_max = 0), for every relative direction, every "
                        "consistent sector pattern and headings +-10 deg, the better one has an allowed velocity with a guaranteed "
                        f"opening of at least {g0:.3f} m/s", "min over cases of max(best_A, best_B) > w_rel_max", ENC, "holds",
                        "holds" if g0 > env.w_rel_max else "violated", g0 > env.w_rel_max, round(time.time() - t0, 3),
                        note=f"{n0} cases; below v_open = {env.v_open} m/s in {100 * frac0:.1f} % of them (unguarded, every "
                             "single pattern reaches v_open: S2b)"), verbose)
    t0 = time.time()
    g1, frac1, n1 = pair_lemma(cfg, u_max=-0.1)
    rep.add(CheckResult("GG2b reported: both drones 0.1 m beyond the bound (u_max = -0.1 m/s): the better opening of the pair",
                        "min over cases of max(best_A, best_B)", ENC, "reported", f"{g1:.3f} m/s", True, round(time.time() - t0, 3),
                        note=f"{n1} cases; below v_open in {100 * frac1:.1f} %; at or below w_rel_max = {env.w_rel_max}: "
                             f"{'yes' if g1 <= env.w_rel_max else 'no'} (GG5b)"), verbose)
    t0 = time.time()
    r, k = pair_separation(cfg, g0, env.a_brake_measured)
    ok = r == z3.unsat and k is not None
    rep.add(CheckResult(f"GG5 P1 for a guarded pair at the hold points: the separation model S1 with the pair opening {g0:.3f} m/s and "
                        f"a one-sided reaction at the calibrated deceleration {env.a_brake_measured} m/s^2 keeps d >= d_safe for all "
                        "time", "BMC 4 s and k-induction of G(d >= d_safe)", ENC + " + formal/check_separation.py", "unsat",
                        f"BMC {r}, k-induction k={k}", ok, round(time.time() - t0, 3),
                        note="the held drone's own closing component is zero (guard_hold); one-sided as S1d, on the simulator's "
                             "calibration"), verbose)
    t0 = time.time()
    r, k = pair_separation(cfg, g0, env.a_brake, k_max=20)
    rep.add(CheckResult(f"GG5o reported: the same at the configured a_brake = {env.a_brake} m/s^2 (as S1o)", "BMC 4 s", ENC, "sat",
                        f"BMC {r}", r == z3.sat, round(time.time() - t0, 3)), verbose)
    t0 = time.time()
    r, k = pair_separation(cfg, g1, env.a_brake_measured, k_max=40)
    rep.add(CheckResult(f"GG5b declared limit: both drones beyond the bound (pair opening {g1:.3f} m/s <= w_rel_max): bounded only",
                        "BMC 4 s holds, no k-induction up to k = 40", ENC, "bounded", f"BMC {r}, k-induction k={k}",
                        r == z3.unsat and k is None, round(time.time() - t0, 3)), verbose)
    for name, a_dec, a_inv, B in (("GG3", env.a_brake_measured, 0.70, 0.30), ("GG3a", env.a_brake, 0.46, 0.32)):
        step, init, s_hold = axis_invariant(cfg, a_dec=a_dec, a_inv=a_inv, B=B)
        margin = -G.cr_half_len - (s_hold + B)
        what = "calibrated" if name == "GG3" else "configured"
        rep.add(check(f"{name} along-axis reachability at the {what} deceleration {a_dec} m/s^2, unbounded: the guarded loop keeps the "
                      f"estimated centre within s_hold + {B} m = {s_hold + B:.3f} m (CR face {-G.cr_half_len} m, margin {margin:.3f} m): "
                      "inductive step", f"I & T -> I' (I: braking curve 2 x {a_inv} m/s^2 to s_hold + B, shifted by w_drift_max)",
                      ENC, step, "unsat", timeout_ms=600000,
                      note=f"dt {env.dt} s, w_drift_max {env.w_drift_max} m/s, GUARD_GAIN {GUARD_GAIN} 1/s"), verbose)
        rep.add(check(f"{name}i the initial set (the mission hold of the stage-2 check: s <= s_hold + w_drift_max / k_track, "
                      "v <= max(0, k_track (s_hold - s)) + 0.1, v <= v_pass) lies in I", "Init -> I", ENC, init, "unsat"), verbose)
    for name, kw, what in (("GG3m", {"guarded": False}, "mutation: without the guard (v11 avoidance at v_escape) the estimated centre "
                                                        "passes s_hold + 0.3 m"),
                           ("GG3h", {"hold_still": True}, "declared limit: a guard hold that cannot move back (a threat behind) "
                                                          "drifts at w_drift_max past s_hold + 0.3 m")):
        cons, s_hold = axis_bmc(cfg, B=0.30, horizon_s=8.0, **kw)
        rep.add(check(f"{name} {what} (BMC 8 s)", "expect a run beyond s_hold + B", ENC, cons, "sat", timeout_ms=600000), verbose)
    import check_fleet
    from check_fleet import PREMISES, Fleet, _run, timers
    T = timers(cfg)
    L = 12
    for n_, depth in ((2, 40), (3, 52), (4, 52)):
        fl = Fleet(n_, L + depth, L, T, latch_delay=1, precedence=False, **PREMISES[n_])
        rep.add(check(f"GG4 n={n_} composition with the guard, from the launch: concurrent commitments (no sonar precedence, claims one "
                      "step late), stops by separation or collision avoidance at any step, no avoidance entry into the CR: never two "
                      "drones in X", "no two in X over the depth", ENC + " + formal/check_fleet.py", _run(fl, fl.two_in_cr()), "unsat",
                      timeout_ms=1800000, note=f"depth {L + depth} steps of {T['DT']:.0f} s"), verbose)
    fo = Fleet(3, L + 40, L, T, latch_delay=1, precedence=False, push="any", **PREMISES[3])
    rep.add(check("GG4o declared limit: a collision-avoidance override of the guard from the hold or the queue (no allowed direction "
                  "opens the threat) can put a second drone into the CR (n=3); monitored at run time (GATE_GUARD_OVERRIDE, CR entries)",
                  "expect a run with two drones in X", ENC + " + formal/check_fleet.py", _run(fo, fo.two_in_cr()), "sat",
                  timeout_ms=900000), verbose)
    del check_fleet
    return rep


if __name__ == "__main__":
    r = run()
    r.save()
    print("all as expected" if r.ok else "SOME CHECK NOT AS EXPECTED")
