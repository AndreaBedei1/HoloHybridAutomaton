"""P2 - critical-region mutual exclusion:  G( sum_i inside_CR_i <= 1 )   (v2: sector-pattern rule).

The rule (holo_fleet/ha/mutex_rule.py) is evaluated here with Z3 terms - the same functions the
drones run.  Queue geometry and thresholds come from holo_fleet.config.GateRule.

M1  Pairwise rule on the bearing model (Z3, linear arithmetic over angles): two drones whose
    headings differ by |delta| <= 2 heading_tol, with every cone boundary uncertain by +-fuzz
    (hull angular size, octree leaf): never both PRIORITY.  The bound |delta| + 2 fuzz <= 30 deg is
    tight: M1m (fuzz too large) and M1m2 (a rule that ignores LEFT) must give counterexamples.
M2  Progress / no deadlock of the abreast queue, n = 2..6 (Z3): with every drone within queue_tol of
    its queue point and within heading_tol of the gate axis, for EVERY set of occupied queue points
    with at most one vacant point between two occupied ones (degraded formations, drones already
    passed), exactly one queued drone - the leftmost occupied - has PRIORITY (no deadlock: some drone
    may go; no double priority); every other one sees a left neighbour in pure LEFT (WAIT) inside the
    queue bracket.  M2s (scope): two adjacent vacant points put the two remaining drones out of each
    other's bracket - both PRIORITY (expected SAT): outside the scope, the occupancy latch (M3) is then
    the only protection.
M2v Stacked queue (numeric, 3-D cones): the 2 x 2 queue of a template with two depth layers (columns
    stack_column_spacing apart, layers stack_spacing apart), every pair perturbed over a grid of the
    position (+-queue_tol per axis) and heading (+-heading_tol) tolerances, every cone boundary within
    +-fuzz free: every pair has ONE possible relation, and for every occupancy exactly the first in the
    order left-top, left-bottom, right-top, right-bottom has PRIORITY; the static rank is never needed.
    M2m (mutation, the v2 rule): vertical-only patterns resolved by the static rank with unknown ranks
    - no drone has PRIORITY in the stacked queue: a deadlock (expected).
M3  Occupancy latch (Z3, bounded model checking of a timed abstraction, 30 s): drone A commits and
    crosses the CR at a speed in [v_lo, v_pass] without stopping; drone B, next in the order, sees A
    (in FRONT, ranges -> along-axis thresholds computed from the geometry, M4) with a two-capture
    confirmation, is blind to A while the gate bars mask it, latches BUSY on a corridor echo,
    needs an echo beyond the CR (EXITING) and t_clear without corridor echoes to turn FREE, and
    commits only with PRIORITY held for t_clear and FREE.  Property: never A and B both in the CR.
    M3m: a belief without the latch (FREE as soon as nothing is seen) must give a counterexample.
M4  Geometry of the queue (numeric, n = 2..6, exact config values): (a) the CR lies inside the
    FRONT cone of every queue point with the heading tolerance and fuzz; (b) the merge path of a
    committed drone keeps merge_clearance from the next queue point; (c) queued neighbours stay
    outside the warning band; (d) every queued drone sees a crossing drone unmasked in its
    corridor for at least the confirmation time; (e) the masking interval ends inside the
    corridor or beyond it (no permanent masking).  M4m: a queue line at 70 deg must fail (a).
M4f The queue neighbours a decision needs are never hidden by the gate frame (an echo inside the structure
    window predicted from the gate map is classified STRUCTURE and its relation is lost), with the measured
    holding error: in an abreast queue the adjacent pairs (n = 2..6), in a stacked queue every pair - its
    queue line (mission.queue_line) is moved back until this holds.  The pair around ONE vacant point of
    an abreast queue is hidden for n = 3 at this gate: that occupancy (the middle drone missing) is
    outside the scope of M2 (listed in the note); moving the line back would put the exits beyond the range
    at which the outer drones detect a hull (probe runs: occupancy-latch timeouts).  M4fm: the stacked
    queue at the cone-only line hides its diagonal neighbours (seen in a probe run).
M5  Progress of the queue (argument + spec): for every occupancy in scope some queued drone has
    PRIORITY - the first in the order left first, then top first (M2, M2v); the belief returns FREE
    after the previous drone is seen beyond the CR (or after t_occ_max, logged); the commit edge is
    then enabled (formal/check_determinism.py, G1/D2).  A WAIT caused by a drone that is not at its
    queue point (approaching, passing) ends when that drone reaches its queue point or moves beyond the
    CR.  Under the fairness assumptions "a committed drone completes its passage" and "every drone in
    the approach zone reaches its queue point or leaves the zone", every queued drone eventually
    commits.  The static rank (the plan's queue order) is only the last tie-break for an ambiguous
    pattern (UP and DOWN at once), counted in every run.  Not covered: a drone that stops for good
    inside the approach zone (a permanent fault there) blocks the drones on its right.

Scope: one critical region at a time, n <= 6 drones in the abreast queue (n = 6 needs the queue
line 9.1 m back).  Not proved: the sonar sees a crossing drone in its corridor (sensing
assumption, measured), a committed drone does not stop inside the CR for t_occ_max.
"""

from __future__ import annotations

import dataclasses
import math
import sys

import numpy as np
import z3

from common import CheckResult, Report, check  # noqa: E402

import itertools

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.flows import pass_path_points
from holo_fleet.ha import mutex_rule
from holo_fleet.mission import (Slot, abreast_slots, queue_assignment, queue_line, queue_order,
                                structure_masked_pairs)
from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, R_IN, R_OUT, centre_distance_lower

ENC = "holo_fleet/ha/mutex_rule.py (shared relation) + formal/check_mutex.py"
AXIS = {"F": 0.0, "L": 90.0, "B": 180.0, "R": -90.0}
HALF = 60.0
HOLD_ERR = DEFAULT.gate.hold_err_m   # measured position-holding error of a queued drone (results/v2/ASSUMPTIONS.md)
V_LO = 0.12           # a committed drone crosses the masked part of the CR within t_occ_max (2.7 m / 25 s)


class _Z:
    And, Or, Not = staticmethod(z3.And), staticmethod(z3.Or), staticmethod(z3.Not)


def _angdist(beta, axis, tag):
    """|wrap(beta - axis)| with an integer wrap variable; returns (distance, constraints)."""
    k = z3.Int(f"k_{tag}")
    x = z3.Real(f"x_{tag}")
    dist = z3.Real(f"ad_{tag}")
    return dist, [x == beta - axis + 360 * k, x >= -180, x < 180, dist == z3.If(x >= 0, x, -x)]


def _membership(beta, fuzz, tag):
    """Sector flags seen from one drone: certain inside 60-fuzz, certain outside 60+fuzz, free between."""
    flags, cons = {}, []
    for s, a in AXIS.items():
        d, c = _angdist(beta, a, f"{tag}{s}")
        f = z3.Bool(f"in_{tag}{s}")
        cons += c + [z3.Implies(d <= HALF - fuzz, f), z3.Implies(d >= HALF + fuzz, z3.Not(f))]
        flags[s] = f
    return flags, cons


def priority(flags, rule=mutex_rule.relation):
    rel = rule(flags["F"], flags["B"], flags["L"], flags["R"], z3.BoolVal(False), z3.BoolVal(False), _Z)
    return rel[mutex_rule.PRIORITY]


def pair_constraints(delta_max, fuzz):
    beta, delta = z3.Reals("beta delta")
    fi, ci = _membership(beta, fuzz, "i")
    fj, cj = _membership(beta + 180 - delta, fuzz, "j")
    return fi, fj, ci + cj + [beta >= -180, beta < 180, delta >= -delta_max, delta <= delta_max]


def _wait_only_front(F, B, L, R, U, D, Lg):
    """Mutation: a drone yields only to what is ahead (LEFT ignored)."""
    wait = F
    return {mutex_rule.WAIT: wait, mutex_rule.PRIORITY: Lg.And(Lg.Not(wait), Lg.Or(B, L, R)),
            mutex_rule.RANK: Lg.And(Lg.Not(Lg.Or(F, B, L, R)), Lg.Or(U, D)), "NONE": Lg.Not(Lg.Or(F, B, L, R, U, D))}


# ---------------------------------------------------------------------------------------------- M2 subsets
def occupancies(n: int, max_gap: int = 2):
    """Non-empty sets of occupied queue points (indices, left first) with consecutive indices at most
    ``max_gap`` apart, i.e. at most max_gap - 1 vacant points between two occupied ones."""
    for r in range(1, n + 1):
        for S in itertools.combinations(range(n), r):
            if all(b - a <= max_gap for a, b in zip(S, S[1:])):
                yield S


def abreast_relations(G, n: int, fuzz: float):
    """Z3 constraints + the relation of every ordered pair (None: outside the queue bracket)."""
    lats = G.queue_laterals(n)
    cons, rel = [], {}
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            gap = abs(lats[i] - lats[j])
            dev = math.degrees(math.atan2(2 * G.queue_tol, gap - 2 * G.queue_tol)) + G.heading_tol_deg
            beta = z3.Real(f"b_{n}_{i}_{j}")
            nominal = -90.0 if lats[j] < lats[i] else 90.0      # j on my right / left
            cons += [beta >= nominal - dev, beta <= nominal + dev]
            fl, c = _membership(beta, fuzz, f"q{n}_{i}_{j}_")
            cons += c
            echo = gap + 2 * G.queue_tol - 0.30 - 0.29          # farthest echo of that neighbour
            rel[(i, j)] = (mutex_rule.relation(fl["F"], fl["B"], fl["L"], fl["R"], z3.BoolVal(False),
                                               z3.BoolVal(False), _Z) if echo <= G.queue_bracket_m else None)
    return cons, rel


def leader_violation(rel, S):
    """Not exactly the leftmost occupied point has PRIORITY (no WAIT relation inside the bracket)."""
    prio = {i: z3.And(*[z3.Not(rel[(i, j)][mutex_rule.WAIT]) for j in S if j != i and rel[(i, j)] is not None])
            for i in S}
    first = min(S)
    return z3.Not(z3.And(prio[first], *[z3.Not(prio[k]) for k in S if k != first]))


# ---------------------------------------------------------------------------------------------- M2v stacked queue
STACK_2X2 = (Slot(0.0, 1.75, 1.75), Slot(0.0, 1.75, -1.75), Slot(0.0, -1.75, 1.75), Slot(0.0, -1.75, -1.75))
STACK_PAIR = (Slot(0.0, 1.75, 1.75), Slot(0.0, 1.75, -1.75), Slot(0.0, -1.75, 0.0))     # formations "stack_pair"


def _cone_flags(rel: np.ndarray, yaw_deg: float, fuzz: float):
    """Sector flags of a target in direction ``rel`` (gate frame) seen by a level drone with heading error
    yaw_deg: "in" (certain), "out" (certain) or "?" (within +-fuzz of the 60 deg boundary)."""
    c, sn = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    Rz = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    u = Rz.T @ (rel / np.linalg.norm(rel))
    out = {}
    for k, a in AXES.items():
        ang = math.degrees(math.acos(float(np.clip(a @ u, -1.0, 1.0))))
        out[k] = "in" if ang <= HALF - fuzz else ("out" if ang >= HALF + fuzz else "?")
    return out


def possible_relations(G, p_i, p_j, fuzz: float, relation=mutex_rule.relation, grid: int = 5):
    """Every relation drone i can assign to drone j over the tolerance grid and the free cone boundaries."""
    names = ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")
    rels = set()
    g = np.linspace(-2 * G.queue_tol, 2 * G.queue_tol, grid)
    nominal = np.asarray(p_j, float) - np.asarray(p_i, float)
    for d in itertools.product(g, g, g):
        for yaw in (-G.heading_tol_deg, 0.0, G.heading_tol_deg):
            fl = _cone_flags(nominal + np.array(d), yaw, fuzz)
            unc = [k for k, v in fl.items() if v == "?"]
            base = {k for k, v in fl.items() if v == "in"}
            for bits in itertools.product((False, True), repeat=len(unc)):
                pat = base | {k for k, b in zip(unc, bits) if b}
                r = relation(*[k in pat for k in names], mutex_rule.PY_RULE_LOGIC)
                rels.add(next(k for k in (mutex_rule.WAIT, mutex_rule.PRIORITY, mutex_rule.RANK, "NONE") if r[k]))
    return rels


def _old_relation(F, B, L, R, U, D, Lg):
    """The v2 rule (mutation): any vertical-only pattern is RANK."""
    horiz = Lg.Or(F, B, L, R)
    wait = Lg.Or(F, L)
    return {mutex_rule.WAIT: wait, mutex_rule.PRIORITY: Lg.And(Lg.Not(wait), Lg.Or(B, R)),
            mutex_rule.RANK: Lg.And(Lg.Not(horiz), Lg.Or(U, D)), "NONE": Lg.And(Lg.Not(horiz), Lg.Not(U), Lg.Not(D))}


def stacked_queue(cfg: FleetConfig, fuzz: float, relation=mutex_rule.relation, old_decide: bool = False,
                  slots=STACK_2X2) -> dict:
    """M2v / M2m: possible relations of every pair of a stacked queue and the leaders of every occupancy."""
    G = cfg.gate
    qa = queue_assignment(slots, G)
    order = queue_order(slots, G)
    q_s = queue_line(slots, G, cfg.perc)
    n = len(slots)
    pts = [np.array([q_s, l, dz]) for l, dz in qa]
    rels = {(i, j): possible_relations(G, pts[i], pts[j], fuzz, relation) for i in range(n) for j in range(n) if i != j}
    far = max(float(np.linalg.norm(pts[i] - pts[j])) + 2 * math.sqrt(3) * G.queue_tol for i in range(n) for j in range(n))
    out = {"queue_points": [[round(float(x), 2) for x in p] for p in pts], "order": order,
           "ambiguous_pairs": sorted(f"{i}->{j}" for (i, j), r in rels.items() if len(r) > 1),
           "rank_possible": any(mutex_rule.RANK in r for r in rels.values()), "farthest_pair_m": round(far, 2),
           "occupancies": []}
    ok = not out["ambiguous_pairs"] and far <= G.queue_bracket_m + 0.6
    for r in range(1, n + 1):
        for S in itertools.combinations(range(n), r):
            leaders = []
            for i in S:
                rel_i = [next(iter(rels[(i, j)])) for j in S if j != i]
                if old_decide:     # v2: RANK resolved against unknown ranks (their_ranks = [-1] * count)
                    d = mutex_rule.decide(rel_i, order[i], their_ranks=[-1] * sum(x == mutex_rule.RANK for x in rel_i))
                else:
                    d = mutex_rule.decide(rel_i, order[i], their_ranks=[order[j] for j in S if j != i])
                if d == mutex_rule.PRIORITY:
                    leaders.append(i)
            first = min(S, key=lambda k: order[k])
            out["occupancies"].append({"occupied": list(S), "leaders": leaders})
            ok &= leaders == [first]
    out["ok"] = bool(ok)
    out["deadlocks"] = sum(1 for o in out["occupancies"] if not o["leaders"])
    # M4s geometry of the stacked queue: CR inside the FRONT cone of every queue point (3-D angle), and the
    # pass path of every drone keeps merge_clearance from the queue points of the drones after it in the order
    worst = 0.0
    for p in pts:
        son = p + np.array([float(MOUNTS["FRONT"][0]), 0.0, 0.0])
        for a, b, c in itertools.product((-1, 1), repeat=3):
            v = np.array([a * G.cr_half_len, b * G.cr_half_width, c * G.cr_half_height]) - son
            worst = max(worst, math.degrees(math.acos(v[0] / np.linalg.norm(v))))
    out["cr_bearing_max_deg"] = round(worst + G.heading_tol_deg, 2)
    clear = 99.0
    for i in range(n):
        path = pass_path_points(G, q_s, qa[i][0], qa[i][1])
        later = [pts[j] for j in range(n) if order[j] > order[i]]
        for a, b in zip(path[:-1], path[1:]):
            for t in np.linspace(0.0, 1.0, 201):
                x = a + t * (b - a)
                for q in later:
                    clear = min(clear, float(np.linalg.norm(x - q)))
    out["merge_clearance_min"] = round(clear, 3)
    return out


# ---------------------------------------------------------------------------------------------- M4 geometry
def queue_points(G, n, perc=DEFAULT.perc):
    """Deployed abreast queue of n drones: the queue line of the shared plan (mission.queue_line: the bounds
    of GateRule.queue_s, moved back until no in-scope neighbour is hidden by the gate frame, M4f)."""
    return queue_line(abreast_slots(n), G, perc), G.queue_laterals(n)


def geometry(cfg: FleetConfig, n: int, queue_line=None) -> dict:
    """Numeric facts of the abreast queue for n drones (exact config values; ``queue_line`` overrides it)."""
    G, env, perc = cfg.gate, cfg.env, cfg.perc
    q_s, lats = queue_points(G, n)
    if queue_line is not None:
        q_s = queue_line
    out = {"n": n, "queue_s": round(q_s, 3)}
    # (a) CR inside the FRONT cone of every queue point: worst bearing of a CR corner, + heading tolerance
    worst = 0.0
    for l in lats:
        sx = q_s + float(MOUNTS["FRONT"][0])
        for a in (-1, 1):
            for b in (-1, 1):
                cs, cl = a * G.cr_half_len, b * G.cr_half_width
                worst = max(worst, abs(math.degrees(math.atan2(cl - l, cs - sx))))
    out["cr_bearing_max_deg"] = round(worst + G.heading_tol_deg, 2)
    # (b) merge clearance of the committed drone from the next (right) queue point
    a = math.radians(G.merge_angle_deg)
    clear = 9.0
    for k in range(n - 1):
        lq, ln = lats[k], lats[k + 1]
        if lq <= 0.3:
            continue
        k_s = G.merge_s - abs(lq) / math.tan(a)
        pts = [(q_s, lq)] + ([(k_s, lq)] if k_s > q_s + 0.1 else []) + [(G.merge_s, 0.0)]
        for (s0, l0), (s1, l1) in zip(pts[:-1], pts[1:]):
            for t in np.linspace(0.0, 1.0, 201):
                clear = min(clear, math.hypot(s0 + t * (s1 - s0) - q_s, l0 + t * (l1 - l0) - ln))
    out["merge_clearance_min"] = round(clear, 3)
    # (c) queued neighbours outside the warning band: conservative distance of a LEFT/RIGHT echo
    r_echo = G.queue_spacing - 2 * HOLD_ERR - float(MOUNTS["LEFT"][1]) - 0.29 - env.eps_range_near
    out["queue_neighbour_d_lower"] = round(centre_distance_lower(r_echo, "LEFT", env.eps_range_far, perc.sonar,
                                                                 centre_in_cone=True), 3)
    # (d)/(e) a drone crossing on the axis seen by each queue point: corridor, masking, beyond (along-axis s of A)
    rows = []
    for l in lats:
        son = np.array([q_s + float(MOUNTS["FRONT"][0]), l, 0.0])
        half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
        q = son
        near = float(np.linalg.norm(q - np.clip(q, -half, half)))
        far = max(float(np.linalg.norm(q - half * np.array([x, y, z]))) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1))
        lo_corr, hi_corr = near - R_OUT - env.eps_range_far, far + env.eps_range_near
        # gate bars seen from this sonar: posts at l = +-(inner/2 + 0.09), top/bottom bars at dz = +-(inner/2 + 0.09)
        bar_pts = [np.array([x, y, z]) for x in (-0.11, 0.11) for y in np.linspace(-0.93, 0.93, 25) for z in (-0.93, 0.93)] + \
                  [np.array([x, y, z]) for x in (-0.11, 0.11) for y in (-0.93, 0.93) for z in np.linspace(-0.93, 0.93, 25)]
        rb = [float(np.linalg.norm(p - son)) for p in bar_pts if
              math.degrees(math.acos(np.clip((p - son)[0] / np.linalg.norm(p - son), -1, 1))) <= HALF + 3.0]
        m_lo, m_hi = min(rb) - perc.structure_tol_m, max(rb) + perc.structure_tol_far_m
        ss = np.linspace(-6.0, 6.0, 2401)
        rng_A = np.array([math.hypot(s - son[0], l) for s in ss]) - 0.29         # near surface of A (hull abeam extent)
        corr = (rng_A >= lo_corr) & (rng_A <= hi_corr)
        masked = (rng_A >= m_lo) & (rng_A <= m_hi)
        seen_corr = corr & ~masked & (ss < 0.8)
        first_mask = float(ss[np.argmax(masked)]) if masked.any() else None
        exposure = float(seen_corr[ss < (first_mask if first_mask is not None else 9)].sum() * (ss[1] - ss[0]))
        after = ss[(ss > (first_mask or -9)) & ~masked]
        unmask_s = float(after.min()) if after.size else None
        beyond_s = float(ss[np.argmax((rng_A > hi_corr) & (ss > 0))]) if ((rng_A > hi_corr) & (ss > 0)).any() else None
        corr_from = float(ss[np.argmax(corr)]) if corr.any() else None
        rows.append({"queue_l": l, "corridor": (round(lo_corr, 2), round(hi_corr, 2)), "mask": (round(m_lo, 2), round(m_hi, 2)),
                     "exposure_m": round(exposure, 2), "corridor_from_s": corr_from, "mask_from_s": first_mask,
                     "visible_again_s": unmask_s, "beyond_s": beyond_s})
    out["views"] = rows
    out["min_exposure_s"] = round(min(r["exposure_m"] for r in rows) / G.v_pass, 2)
    out["unmask_before_beyond"] = all(r["visible_again_s"] is None or r["beyond_s"] is None or r["visible_again_s"] <= r["beyond_s"] + 1e-9
                                      for r in rows)
    out["mask_inside_cr_or_after"] = all(r["mask_from_s"] is None or r["mask_from_s"] >= -G.cr_half_len - 0.3 for r in rows)
    return out


# ---------------------------------------------------------------------------------------------- M3 timed latch model
def latch_bmc(cfg: FleetConfig, geo: dict, steps: int = 420, latch: bool = True, v_lo: float = V_LO, view=None):
    """A crosses on the axis; B (queue view ``view``) decides with the deployed belief logic (incl. t_occ_max)."""
    G = cfg.gate
    dt = 0.1
    tc = int(round(G.t_clear / dt))
    t_occ = int(round(G.t_occ_max / dt))
    v_hi = G.v_pass
    view = view or min(geo["views"], key=lambda r: r["exposure_m"])
    s_corr_lo = view["corridor_from_s"] if view["corridor_from_s"] is not None else 99.0
    m_lo_s = view["mask_from_s"] if view["mask_from_s"] is not None else 99.0
    m_hi_s = view["visible_again_s"] if view["visible_again_s"] is not None else 99.0
    b_s = view["beyond_s"] if view["beyond_s"] is not None else 99.0
    reach = int(math.floor((abs(geo["queue_s"]) - G.cr_half_len) / v_hi / dt))       # B cannot reach the CR faster
    sA = [z3.Real(f"sA_{k}") for k in range(steps + 1)]
    seen = [z3.Bool(f"seen_{k}") for k in range(steps + 1)]          # A visible to B in FRONT (unmasked)
    corr = [z3.Bool(f"corr_{k}") for k in range(steps + 1)]          # confirmed corridor echo
    beyond = [z3.Bool(f"beyond_{k}") for k in range(steps + 1)]
    busy = [z3.Bool(f"busy_{k}") for k in range(steps + 1)]
    since = [z3.Int(f"since_{k}") for k in range(steps + 1)]          # steps since the last corridor echo
    quiet = [z3.Int(f"quiet_{k}") for k in range(steps + 1)]          # steps since last corridor echo after an exit
    prio = [z3.Int(f"prio_{k}") for k in range(steps + 1)]            # steps of continuous PRIORITY
    commit = [z3.Bool(f"commit_{k}") for k in range(steps + 1)]
    cons = [sA[0] == geo["queue_s"], z3.Not(busy[0]), quiet[0] == tc, prio[0] == 0, z3.Not(commit[0]), since[0] == 0]
    for k in range(steps):
        cons.append(z3.And(sA[k + 1] - sA[k] >= v_lo * dt, sA[k + 1] - sA[k] <= v_hi * dt))
    for k in range(steps + 1):
        vis = z3.Or(sA[k] < m_lo_s, sA[k] > m_hi_s)
        cons.append(seen[k] == vis)
        in_corr_range = z3.And(sA[k] >= s_corr_lo, sA[k] <= b_s)
        conf = z3.And(seen[k], in_corr_range) if k == 0 else z3.And(seen[k], seen[k - 1], in_corr_range)
        cons.append(corr[k] == conf)
        cons.append(beyond[k] == z3.And(seen[k], sA[k] > b_s))
    for k in range(steps):
        cons.append(since[k + 1] == z3.If(corr[k + 1], 0, since[k] + 1))
        if latch:
            nb = z3.If(corr[k + 1], True, z3.If(z3.And(busy[k], z3.Or(beyond[k + 1], since[k + 1] > t_occ)), False, busy[k]))
        else:
            nb = corr[k + 1]                                          # mutation: no memory of an unseen drone
        cons.append(busy[k + 1] == nb)
        cons.append(quiet[k + 1] == z3.If(z3.Or(corr[k + 1], busy[k + 1]), 0, quiet[k] + 1))
        # B's relation: WAIT while A is seen in FRONT within the CR far range (corridor or nearer)
        wait = z3.And(seen[k + 1], sA[k + 1] <= b_s)
        cons.append(prio[k + 1] == z3.If(wait, 0, prio[k] + 1))
        free = z3.And(z3.Not(busy[k + 1]), quiet[k + 1] >= tc)
        cons.append(commit[k + 1] == z3.Or(commit[k], z3.And(prio[k + 1] >= tc, free)))
    # violation: B committed at step c, reaches the CR not before c + reach, while A is still in the CR
    viol = []
    for c in range(steps + 1):
        for k in range(c + reach, steps + 1):
            viol.append(z3.And(commit[c], z3.Not(commit[c - 1]) if c > 0 else z3.BoolVal(True),
                               sA[k] >= -G.cr_half_len, sA[k] <= G.cr_half_len))
    cons.append(z3.Or(*viol) if viol else z3.BoolVal(False))
    return cons, {"view": view["queue_l"], "mask_s": (m_lo_s, m_hi_s), "beyond_s": b_s, "reach_steps": reach}


# ---------------------------------------------------------------------------------------------- run
def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("mutex")
    G = cfg.gate
    dmax = 2 * G.heading_tol_deg
    fuzz = (30.0 - dmax) / 2.0
    # M1
    fi, fj, cons = pair_constraints(dmax, fuzz)
    rep.add(check(f"M1 never both PRIORITY (|delta| <= {dmax:.0f} deg, fuzz {fuzz:.0f} deg)",
                  "|delta| + 2 fuzz <= 30 -> not(PRIORITY_i and PRIORITY_j)", ENC,
                  cons + [priority(fi), priority(fj)]), verbose)
    fi, fj, cons = pair_constraints(dmax, fuzz + 2.0)
    rep.add(check(f"M1m mutation: fuzz {fuzz + 2:.0f} deg (|delta| + 2 fuzz > 30) must allow double PRIORITY",
                  "expect counterexample (beta, delta)", ENC, cons + [priority(fi), priority(fj)], expect="sat"), verbose)
    fi, fj, cons = pair_constraints(dmax, fuzz)
    rep.add(check("M1m2 mutation: yield only to FRONT (LEFT ignored) must allow double PRIORITY",
                  "expect counterexample", ENC,
                  cons + [priority(fi, _wait_only_front), priority(fj, _wait_only_front)], expect="sat"), verbose)
    # M2 progress / no deadlock of the abreast queue, every occupancy in scope
    for n in range(2, 7):
        q_s, _lats = queue_points(G, n, cfg.perc)
        cons, rel = abreast_relations(G, n, fuzz)
        sets = list(occupancies(n))
        dev_adj = math.degrees(math.atan2(2 * G.queue_tol, G.queue_spacing - 2 * G.queue_tol)) + G.heading_tol_deg
        rep.add(check(f"M2 queue n={n}: for every occupancy with <= 1 vacant point between neighbours ({len(sets)} sets), "
                      f"exactly the leftmost occupied drone has PRIORITY",
                      "queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first)", ENC,
                      cons + [z3.Or(*[leader_violation(rel, S) for S in sets])],
                      note=f"queue line s = {q_s:.2f} m, adjacent bearing deviation <= {dev_adj:.1f} deg; "
                           f"no deadlock (a leader exists) and no double priority"), verbose)
    cons, rel = abreast_relations(G, 4, fuzz)
    rep.add(check("M2s scope: two adjacent vacant queue points (n=4, occupied {0, 3}) -> both PRIORITY",
                  "expect SAT: outside the scope of M2, the occupancy latch (M3) is the only protection", ENC,
                  cons + [leader_violation(rel, (0, 3))], expect="sat",
                  note="gap 10.5 m > queue bracket 7.5 m: the two drones do not count each other"), verbose)
    # M2v stacked queue (numeric, 3-D cones) and the v2 rule as a mutation (deadlock)
    sp = stacked_queue(cfg, fuzz, slots=STACK_PAIR)
    st = stacked_queue(cfg, fuzz)
    st["ok"] = st["ok"] and sp["ok"]
    rep.add(CheckResult("M2v stacked queues (2 x 2, and the 3-drone stack_pair of the demo): one relation per pair over "
                        "the tolerances, exactly the first in the order left first, then top first has PRIORITY for every "
                        "occupancy (15 + 7)",
                        "grid of +-queue_tol, +-heading_tol, +-fuzz -> unique relation; leader = first(S); no RANK",
                        "holo_fleet/ha/mutex_rule.py + mission.queue_assignment (numeric)", "holds",
                        verdict="holds" if st["ok"] else "violated", passed=st["ok"],
                        note=f"queue points {st['queue_points']}, ambiguous pairs {st['ambiguous_pairs'] or 'none'}, "
                             f"static rank possible: {st['rank_possible']}, farthest pair {st['farthest_pair_m']} m"), verbose)
    ok_s = all(x["cr_bearing_max_deg"] + 5.0 <= 60.0 and x["merge_clearance_min"] >= G.merge_clearance - 1e-6
               for x in (st, sp))
    rep.add(CheckResult("M4s stacked queue geometry: CR inside every FRONT cone; each pass path keeps merge_clearance "
                        "from the queue points of the drones after it", "3-D bearing + heading_tol + fuzz <= 60 deg; "
                        "min distance >= merge_clearance", "config.GateRule.queue_s + control/flows.pass_path_points",
                        "holds", verdict="holds" if ok_s else "violated", passed=ok_s,
                        note=f"{st['cr_bearing_max_deg']} deg; clearance {st['merge_clearance_min']} m "
                             f">= {G.merge_clearance} m"), verbose)
    old = stacked_queue(cfg, fuzz, relation=_old_relation, old_decide=True)
    old_p = stacked_queue(cfg, fuzz, relation=_old_relation, old_decide=True, slots=STACK_PAIR)
    old["deadlocks"] = old["deadlocks"] if old_p["deadlocks"] else 0
    rep.add(CheckResult("M2m mutation: the v2 rule (vertical-only pattern -> RANK, ranks unknown) deadlocks the stacked "
                        "queue (no drone has PRIORITY)", "expect violation", "mutex_rule (v2)", "violated",
                        verdict="violated" if old["deadlocks"] else "holds", passed=old["deadlocks"] > 0,
                        note=f"{old['deadlocks']} of 15 occupancies without a leader, e.g. "
                             f"{next((o['occupied'] for o in old['occupancies'] if not o['leaders']), None)}"), verbose)
    st_close = stacked_queue(dataclasses.replace(cfg, gate=dataclasses.replace(G, stack_column_spacing=G.queue_spacing)), fuzz)
    rep.add(CheckResult("M2vm mutation: stacked columns only queue_spacing (3.5 m) apart -> a diagonal pair is ambiguous",
                        "expect violation", "config.GateRule.stack_column_spacing", "violated",
                        verdict="violated" if not st_close["ok"] else "holds", passed=not st_close["ok"],
                        note=f"ambiguous pairs {st_close['ambiguous_pairs']}: left-bottom / right-top seen in UP/DOWN only "
                             f"at the tolerance limits"), verbose)
    # M4 geometry (numeric) for n = 2..6
    geos = {n: geometry(cfg, n) for n in range(2, 7)}
    ok_a = all(g["cr_bearing_max_deg"] + 5.0 <= 60.0 for g in geos.values())
    rep.add(CheckResult("M4a CR inside the FRONT cone of every queue point (n = 2..6)",
                        "max CR bearing + heading_tol + fuzz <= 60 deg", "config.GateRule.queue_s", "holds",
                        verdict="holds" if ok_a else "violated", passed=ok_a,
                        note="; ".join(f"n={n}: {g['cr_bearing_max_deg']} deg" for n, g in geos.items())), verbose)
    ok_b = all(g["merge_clearance_min"] >= G.merge_clearance - 1e-6 for g in geos.values())
    rep.add(CheckResult("M4b merge path keeps merge_clearance from the next queue point (n = 2..6)",
                        "min distance >= merge_clearance", "control/flows.py pass_path", "holds",
                        verdict="holds" if ok_b else "violated", passed=ok_b,
                        note="; ".join(f"n={n}: {g['merge_clearance_min']} m" for n, g in geos.items())), verbose)
    ok_c = all(g["queue_neighbour_d_lower"] >= cfg.sep.d_warning_exit for g in geos.values())
    rep.add(CheckResult(f"M4c queued neighbours outside the warning band (holding error <= {HOLD_ERR} m)", "d_lower(neighbour) >= d_warning_exit",
                        "config + perception bound", "holds", verdict="holds" if ok_c else "violated", passed=ok_c,
                        note=f"d_lower {geos[3]['queue_neighbour_d_lower']} m vs d_warning_exit {cfg.sep.d_warning_exit} m"), verbose)
    need = 3 * 0.1
    ok_d = all(g["min_exposure_s"] >= need for g in geos.values())
    rep.add(CheckResult("M4d a crossing drone is seen unmasked in the corridor for >= 3 captures",
                        "exposure / v_pass >= 0.3 s", "perception/gate_perception.py", "holds",
                        verdict="holds" if ok_d else "violated", passed=ok_d,
                        note="; ".join(f"n={n}: {g['min_exposure_s']} s" for n, g in geos.items())), verbose)
    ok_e = all(v["mask_from_s"] is None or (v["visible_again_s"] is not None and v["visible_again_s"] <= G.cr_half_len + 2.0
                                             and v["corridor_from_s"] is not None and v["corridor_from_s"] < v["mask_from_s"])
               for g in geos.values() for v in g["views"])
    span = max((v["visible_again_s"] or 0) - (v["mask_from_s"] or 0) for g in geos.values() for v in g["views"])
    rep.add(CheckResult("M4e masking interval is bounded and starts after the corridor is entered",
                        "corridor_from < mask_from and visible_again <= cr_half_len + 2 m", "perception", "holds",
                        verdict="holds" if ok_e else "violated", passed=ok_e,
                        note=f"masked span <= {span:.2f} m along the axis"), verbose)
    # M4f no queue neighbour a decision needs is hidden by the gate frame (structure window of the classifier)
    rows, ok_f, scope = [], True, []
    for n in range(2, 7):
        q_s, lats = queue_points(G, n, cfg.perc)
        pts = [(l, 0.0) for l in lats]
        adj = {(i, j) for i in range(n) for j in range(n) if abs(i - j) == 1}
        gap = {(i, j) for i in range(n) for j in range(n) if abs(i - j) == 2}
        ok_f &= not structure_masked_pairs(pts, q_s, G, cfg.perc, pairs=adj)
        hidden = sorted({tuple(sorted(x[:2])) for x in structure_masked_pairs(pts, q_s, G, cfg.perc, pairs=gap)})
        if hidden:
            scope.append(f"n={n}: pair(s) {hidden} around a vacant point")
        rows.append(f"n={n}: s={q_s:.2f}")
    for nm, slots in (("stack 2x2", STACK_2X2), ("stack_pair", STACK_PAIR)):
        q_x = queue_line(slots, G, cfg.perc)
        m = structure_masked_pairs(queue_assignment(slots, G), q_x, G, cfg.perc)
        ok_f &= not m
        rows.append(f"{nm}: s={q_x:.2f}")
    rep.add(CheckResult("M4f queue neighbours a decision needs are never hidden in the gate's structure window "
                        "(abreast: adjacent pairs, n = 2..6; stacked: every pair; +-hold_err)",
                        "echo of the neighbour outside [window - tol, window + tol_far] of every sector that sees it",
                        "mission.queue_line + perception/sonar_processing.py (structure window)",
                        "holds", verdict="holds" if ok_f else "violated", passed=ok_f,
                        note="; ".join(rows) + (". Scope of M2 (hidden, excluded): " + "; ".join(scope) if scope else "")),
            verbose)
    rep.scope_hidden = scope
    qa_st = queue_assignment(STACK_2X2, G)
    lats_st = tuple(sorted({l for l, _dz in qa_st}, reverse=True))
    q_an = G.queue_s(2, max(abs(dz) for _l, dz in qa_st), lats=lats_st)
    m = structure_masked_pairs(qa_st, q_an, G, cfg.perc)
    rep.add(CheckResult("M4fm mutation: the stacked queue at the cone-only line (no structure clearance) hides the "
                        "diagonal neighbours", "expect violation", "mission.queue_line", "violated",
                        verdict="violated" if m else "holds", passed=bool(m),
                        note=f"s = {q_an:.2f}: {len(m)} masked views, e.g. {m[:1]} (seen in a probe run: diagonal "
                             f"neighbour classified STRUCTURE, the vertical relation decided alone)"), verbose)
    bad = geometry(cfg, 4, queue_line=-4.0)
    rep.add(CheckResult("M4m mutation: n=4 queue line at s = -4 m must violate M4a", "expect violation",
                        "config.GateRule", "violated",
                        verdict="violated" if bad["cr_bearing_max_deg"] + 5.0 > 60.0 else "holds",
                        passed=bad["cr_bearing_max_deg"] + 5.0 > 60.0,
                        note=f"{bad['cr_bearing_max_deg']} deg + fuzz > 60"), verbose)
    # M3 timed latch model: every queue point as the observer B
    for n in (2, 3, 4):
        for view in geos[n]["views"]:
            if view["queue_l"] < -1e-6 and any(abs(v["queue_l"] + view["queue_l"]) < 1e-6 for v in geos[n]["views"]):
                continue                                              # mirror image of a view already checked
            cons, info = latch_bmc(cfg, geos[n], view=view)
            rep.add(check(f"M3 occupancy latch, n={n}, observer at l={view['queue_l']:+.2f}: never both in the CR (42 s BMC)",
                          "latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR)", ENC, cons,
                          note=f"masked s in {info['mask_s']}, beyond s > {info['beyond_s']}, B needs >= {info['reach_steps']} "
                               f"steps to the CR, A crosses at >= {V_LO} m/s"), verbose)
    # the queue closest to the gate (n = 2): the shortest way to the CR, where the masked interval matters most
    worst = max(geos[2]["views"], key=lambda v: v["queue_l"])
    cons, _ = latch_bmc(cfg, geos[2], latch=False, view=worst)
    rep.add(check("M3m mutation: belief without latch (FREE when nothing is seen) must violate P2",
                  "expect counterexample", ENC, cons, expect="sat",
                  note=f"n=2, observer at l={worst['queue_l']:+.2f}"), verbose)
    rep.geometry = geos
    return rep


def main() -> int:
    rep = run()
    path = rep.save()
    print(f"mutex: {sum(r.passed for r in rep.results)}/{len(rep.results)} checks passed -> {path}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
