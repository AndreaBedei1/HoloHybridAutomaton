"""Timed occupancy-latch release (STALE) and the admission bound (perception/gate_perception.py, control/flows.py).

Deployed logic (gate_perception.stale_release / entry_after, flows.Flows.gate):
* BUSY -> STALE when the last corridor echo is not before BUSY began, no corridor echo for more than t_occ_max, nothing heard
  possibly inside and nothing known inside.  STALE is not ``occ_busy``: the drone may commit and move to the stage-2 hold,
  which is outside the CR (control/gate_safety.py keeps a non-admitted drone out of it).
* After a timed release (release == TIMER) admission also needs t >= entry_after = last corridor sighting + t_admit_stale;
  (strict: at t = entry_after the occupant can be exactly on the closed CR boundary); a positive exit witness (EXITING)
  removes the bound.  The give-up timer of the hold runs from max(hold start, eligible time).

ST1 (proved): the deployed rule - admission strictly after max(last sighting, start of the dwell in the admission zone) +
t_admit_stale, a sighting being a corridor echo or any hull within near_sight_m - with Pi1 (no centre in the CR before admission),
Pi2 (progress >= v_cross_min after admission) and premise N (an occupant admitted while the follower dwells in the zone is
sighted by it at or after its admission) puts the first occupant beyond passage_clear_s at the follower's admission, by cases:
admitted before the dwell (the dwell alone covers it, no sighting needed) or during it (N).  ST6 (numeric): the zone geometry
behind N.  ST1s: the earlier argument from the last sighting only, which needs premise S; ST1d: without S and without the dwell
term it fails.  ST1p the P2 part (> cr_s_max) with the shorter bound.  Mutations: bound - 2 s, N dropped, no progress, and the
e3e31f0 release (FREE at t_occ_max, no admission bound) with an immediate admission.
ST2 (proved): one step of the deployed latch never gives FREE from BUSY or STALE; FREE comes only from EXITING (a witness).
ST3 (proved): with the corridor quiet (no new sighting moves entry_after) and the admission conditions holding, the hold is never
given up: admission happens at the first step with t >= max(commit + admit_wait, entry_after), a time bound fixed in advance
(the ranking eligible - t decreases by env.dt per step), so the commit/hold/give-up cycle cannot repeat.
ST4 (numeric): the Python predicates and their Z3 encoding agree on a grid of inputs (runtime/formal consistency).
"""

from __future__ import annotations

import itertools
import sys
from pathlib import Path

import numpy as np
import z3

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import CheckResult, Report, check  # noqa: E402
from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.perception.gate_perception import entry_after, stale_release  # noqa: E402

ENC = "linear real arithmetic over the deployed guards and the declared crossing premise (v_cross_min)"
FREE, BUSY, STALE, EXITING = 0, 1, 2, 3


def z3_stale_release(t, last, since, corridor, heard, known, t_occ):
    return z3.And(z3.Not(corridor), z3.Not(heard), z3.Not(known), last >= since, t - last > t_occ)


def Q(x: float):
    """A configuration constant as the decimal it is written as (repr), not the binary value of the float."""
    return z3.RealVal(repr(float(x)))


def timeline(cfg: FleetConfig, bound: float, s_end: float, premise_s: bool = True, v: float = None):
    """Occupant admitted at t_a, last seen by the follower at t_seen, follower admitted at t_b > t_seen + bound."""
    G = cfg.gate
    v = G.v_cross_min if v is None else v
    t_a, t_seen, t_b, s_a, s_b = z3.Reals("t_a t_seen t_b s_a s_b")
    cons = [s_a >= Q(G.stage2_hold_s) - Q(G.admit_zone_m) - Q(G.nav_err_m), t_b > t_seen + Q(bound), t_b >= t_a,
            s_b >= s_a + Q(v) * (t_b - t_a), s_b <= Q(s_end)]
    if premise_s:
        cons.append(t_seen >= t_a)
    return cons


def timeline_dwell(cfg: FleetConfig, bound: float, s_end: float, premise_n: bool = True, dwell: bool = True,
                   v: float = None):
    """The deployed rule: occupant A admitted at t_a, the follower B dwells in the admission zone from t_o to its
    admission t_b, its last sighting (corridor echo or a hull within near_sight_m) at t_s; B is admitted at
    t_b > max(t_s, t_o) + bound.  Premise N: an occupant admitted while B dwells in the zone (t_a > t_o) is sighted by B
    at or after its admission (t_s >= t_a).  No premise S."""
    G = cfg.gate
    v = G.v_cross_min if v is None else v
    t_a, t_o, t_s, t_b, s_a, s_b = z3.Reals("t_a t_o t_s t_b s_a s_b")
    cons = [s_a >= Q(G.stage2_hold_s) - Q(G.admit_zone_m) - Q(G.nav_err_m), t_b >= t_a, t_o <= t_b,
            t_b > t_s + Q(bound), s_b >= s_a + Q(v) * (t_b - t_a), s_b <= Q(s_end)]
    if dwell:
        cons.append(t_b > t_o + Q(bound))
    if premise_n:
        cons.append(z3.Implies(t_a > t_o, t_s >= t_a))
    return cons


def zone_distance(cfg: FleetConfig, e_obs: float, e_adm: float, dz_err: float = 0.05):
    """Largest true centre distance between two drones whose estimates are both in the admission zone: along the axis
    admit_zone_m + hold_err_m, laterally 2 hold_lateral_allow, vertically the span of the acquisition offsets, plus the
    observer's and the admitted drone's horizontal position errors and the depth errors (numeric, triangle inequality)."""
    from holo_fleet.control.flows import ACQ_OFFSETS

    G = cfg.gate
    dz_span = max(dz for _, dz in ACQ_OFFSETS) - min(dz for _, dz in ACQ_OFFSETS)
    horiz = float(np.hypot(G.admit_zone_m + G.hold_err_m, 2 * G.hold_lateral_allow)) + e_obs + e_adm
    return float(np.hypot(horiz, dz_span + 2 * dz_err))


def latch_step(t_occ: float):
    """One update of the deployed latch (gate_perception.update): returns (constraints, prev, new)."""
    prev, new = z3.Ints("prev new")
    corridor, heard, known, witness = z3.Bools("corridor heard known witness")
    t, last, since, seen_exit = z3.Reals("t last since seen_exit")
    t_clear = DEFAULT.gate.t_clear
    s1 = z3.Int("s1")
    s2 = z3.Int("s2")
    s3 = z3.Int("s3")
    cons = [z3.And(prev >= FREE, prev <= EXITING)]
    # corridor or heard: BUSY
    cons.append(s1 == z3.If(z3.Or(corridor, heard), BUSY, prev))
    last1 = z3.If(corridor, t, last)
    # a witness needs the corridor empty; heard / known inside cancel it
    wit = z3.And(witness, z3.Not(corridor), z3.Not(heard), z3.Not(known))
    stale = z3_stale_release(t, last1, since, corridor, heard, known, t_occ)
    cons.append(s2 == z3.If(z3.Or(s1 == BUSY, s1 == STALE),
                            z3.If(wit, EXITING, z3.If(z3.And(s1 == BUSY, stale), STALE, s1)), s1))
    exit_t = z3.If(z3.And(z3.Or(s1 == BUSY, s1 == STALE), wit), t, seen_exit)
    cons.append(s3 == z3.If(z3.And(s2 == EXITING, z3.Not(corridor), z3.Not(heard), z3.Not(known),
                                   t - z3.If(exit_t > last1, exit_t, last1) >= t_clear), FREE, s2))
    cons.append(new == s3)
    return cons, prev, new


def consistency(cfg: FleetConfig) -> CheckResult:
    G = cfg.gate
    bad = []
    n = 0
    for t, last, since in itertools.product((0.0, 10.0, 24.9, 25.0, 25.1, 40.0), (-1e9, 0.0, 5.0, 15.0), (-1e9, 0.0, 3.0, 20.0)):
        for corridor, heard, known in itertools.product((False, True), repeat=3):
            n += 1
            py = stale_release(t, last, since, corridor, heard, known, G.t_occ_max)
            zz = z3.is_true(z3.simplify(z3_stale_release(z3.RealVal(t), z3.RealVal(last), z3.RealVal(since),
                                                         z3.BoolVal(corridor), z3.BoolVal(heard), z3.BoolVal(known),
                                                         G.t_occ_max)))
            if py != zz:
                bad.append((t, last, since, corridor, heard, known))
    for release, last in itertools.product((None, "TIMER", "WITNESS"), (-1e9, 0.0, 12.5)):
        n += 1
        want = last + G.t_admit_stale if release == "TIMER" else -1e9
        if abs(entry_after(release, last, G.t_admit_stale) - want) > 1e-9:
            bad.append((release, last))
    r = CheckResult(prop=f"ST4 runtime stale_release / entry_after agree with the Z3 encoding ({n} input combinations)",
                    formula="python(x) == z3(x) for every sampled x", encoding="numeric grid", expect="holds")
    r.verdict = "holds" if not bad else "violated"
    r.passed = not bad
    r.note = "" if not bad else str(bad[:5])
    return r


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("stale")
    G = cfg.gate
    T, T_p2 = G.t_admit_stale, (G.cr_s_max - G.stage2_hold_s + G.admit_zone_m + G.nav_err_m) / G.v_cross_min
    rep.add(check(f"ST1 deployed rule (dwell + sighting, t_admit_stale = {T:.2f} s), premises Pi1, Pi2, N, no premise S: the "
                  "first occupant is beyond passage_clear_s at the follower's admission",
                  "t_b > max(t_s, t_o) + T, N: t_a > t_o -> t_s >= t_a, s(t) >= s_a + v_cross_min (t - t_a) -> s(t_b) > passage_clear_s",
                  ENC, timeline_dwell(cfg, T, G.passage_clear_s),
                  note="case t_a <= t_o: the dwell alone; case t_a > t_o: the sighting of premise N"), verbose)
    rep.add(check("ST1n mutation: the deployed rule without premise N (an occupant admitted during the dwell, never sighted)",
                  "expect counterexample", ENC, timeline_dwell(cfg, T, G.passage_clear_s, premise_n=False), expect="sat"), verbose)
    rep.add(check("ST1d mutation: the bound from the last sighting only (no dwell term) without premise S",
                  "expect counterexample: an occupant seen, then admitted much later, never seen again", ENC,
                  timeline_dwell(cfg, T, G.passage_clear_s, premise_n=False, dwell=False), expect="sat"), verbose)
    rep.add(check(f"ST1s alternative: from the last sighting only under premise S, after {T:.2f} s the occupant is beyond "
                  "passage_clear_s", "S, Pi2, t_b > t_seen + T -> s(t_b) > passage_clear_s",
                  ENC, timeline(cfg, T, G.passage_clear_s)), verbose)
    rep.add(check(f"ST1p P2 part: after {T_p2:.2f} s the occupant is beyond the CR exit plane cr_s_max",
                  "same premises -> s(t_b) > cr_s_max", ENC, timeline(cfg, T_p2, G.cr_s_max)), verbose)
    rep.add(check("ST1m mutation: a bound 2 s shorter admits while the occupant can still be in the passage",
                  "expect counterexample", ENC, timeline(cfg, T - 2.0, G.passage_clear_s), expect="sat"), verbose)
    rep.add(check("ST1m2 mutation: without premise S (sighting before the occupant's admission) the bound is not enough",
                  "expect counterexample", ENC, timeline(cfg, T, G.passage_clear_s, premise_s=False), expect="sat"), verbose)
    rep.add(check("ST1m3 mutation: an occupant that does not progress (v = 0, outside the premise) defeats any bound",
                  "expect counterexample", ENC, timeline(cfg, T, G.passage_clear_s, v=0.0), expect="sat"), verbose)
    rep.add(check(f"ST1m4 mutation: the e3e31f0 release (FREE after t_occ_max = {G.t_occ_max:.0f} s, no admission bound)",
                  "admission at t_seen + t_occ_max: expect counterexample", ENC,
                  timeline(cfg, G.t_occ_max, G.cr_s_max), expect="sat"), verbose)
    rep.add(check("ST1w witness: the timeline premises are satisfiable", "expect SAT (non-vacuity)", ENC,
                  timeline(cfg, T, 1e6), expect="sat"), verbose)
    cons, prev, new = latch_step(G.t_occ_max)
    rep.add(check("ST2 one step of the deployed latch never gives FREE from BUSY or STALE (time alone never frees)",
                  "prev in {BUSY, STALE} -> new != FREE", ENC, cons + [z3.Or(prev == BUSY, prev == STALE), new == FREE]), verbose)
    rep.add(check("ST2w witness: BUSY -> STALE is reachable in one step", "expect SAT", ENC,
                  cons + [prev == BUSY, new == STALE], expect="sat"), verbose)
    # ST3: the deployed hold (flows.Flows.gate).  eligible = max(commit + admit_wait, entry_after); admitted at the first
    # step with t >= eligible while the admission conditions (admissible, loc_ok, entry_ok, in the admission zone) hold;
    # give-up when not admitted and t - max(hold start, eligible) > t_occ_max + t_clear.
    t, E, W, h = z3.Reals("t E W h")
    elig = z3.If(W > E, W, E)
    admitted = z3.And(t >= W, t > E)
    give_up = z3.And(z3.Not(admitted), t - z3.If(h > elig, h, elig) > G.t_occ_max + G.t_clear)
    rep.add(check("ST3 with the admission conditions holding the hold is never given up: admission comes first",
                  "conditions -> not (not admitted and t - max(h, eligible) > t_occ_max + t_clear)", ENC, [give_up]), verbose)
    rep.add(check("ST3w witness: before the eligible time the drone waits at the hold", "expect SAT", ENC,
                  [z3.Not(admitted), t >= h], expect="sat"), verbose)
    old_give_up = z3.And(z3.Not(admitted), t - z3.If(h > W, h, W) > G.t_occ_max + G.t_clear)
    rep.add(check("ST3m mutation: a give-up timer started from the hold and commitment only (not entry_after) gives up "
                  "before the timed admission (commit/give-up loop)", "expect counterexample", ENC,
                  [old_give_up, E > W], expect="sat"), verbose)
    rep.add(consistency(cfg), verbose)
    from holo_fleet.perception.structure_aiding import CONSIST_M, FIT_RMS_M, UNIQUE_M

    e_adm = CONSIST_M + UNIQUE_M + FIT_RMS_M
    d_zone = zone_distance(cfg, G.nav_err_m, e_adm)
    ok = d_zone < G.near_sight_m
    r = CheckResult(prop=f"ST6 premise N geometry: two drones in the admission zone are at most {d_zone:.2f} m apart < near_sight_m "
                         f"= {G.near_sight_m} m (observer error nav_err_m = {G.nav_err_m} m, admitted drone validated within "
                         f"{e_adm:.2f} m)", formula="max true centre distance (triangle inequality) < near_sight_m <= d_lower bound",
                    encoding="numeric bound over the zone extents (config, flows.ACQ_OFFSETS, structure_aiding constants)",
                    expect="holds")
    r.verdict, r.passed = ("holds" if ok else "violated"), ok
    r.note = ("the sighting itself needs the hull's echo in some healthy sector (the P1 detection premise); a hull within "
              "near_sight_m has d_lower < near_sight_m since d_lower is a lower bound of the centre distance (S0)")
    rep.add(r, verbose)
    r = CheckResult(prop=f"ST5 reported: t_admit_stale = {T:.2f} s (P2 part {T_p2:.2f} s, P1 clearance part {T - T_p2:.2f} s), "
                         f"t_occ_max = {G.t_occ_max:.0f} s", formula="reported", encoding="config", expect="holds")
    r.verdict, r.passed = "holds", True
    rep.add(r, verbose)
    return rep


if __name__ == "__main__":
    rep = run()
    rep.save()
    sys.exit(0 if rep.ok else 1)
