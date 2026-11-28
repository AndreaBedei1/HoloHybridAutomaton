"""Timed occupancy-latch release (STALE) and the admission bound (perception/gate_perception.py, control/flows.py).

Deployed logic (gate_perception.stale_release / timed_admission / entry_after, flows.Flows.gate).  Every admission that rests on
no positive exit witness after the last sighting (STALE, BUSY, or FREE without any witness: a B0 lead drone, the hidden partner
of a concurrent commitment of formal M4f) is bounded; B2 with fresh reports of every other drone rests on B2C-E1 instead.  So the
B0 P2 argument is the witness path (formal M3) or this timed path (ST1), independent of the queue precedence M2:
* BUSY -> STALE when the last corridor echo is not before BUSY began, no corridor echo for more than t_occ_max, nothing heard
  possibly inside and nothing known inside.  STALE is not ``occ_busy``: the drone may commit and move to the stage-2 hold,
  which is outside the CR (control/gate_safety.py keeps a non-admitted drone out of it).
* After a timed release (release == TIMER) admission also needs t >= entry_after = last corridor sighting + t_admit_stale;
  (strict: at t = entry_after the occupant can be exactly on the closed CR boundary); a positive exit witness (EXITING)
  removes the bound.  The give-up timer of the hold runs from max(hold start, eligible time).

ST1 (proved): the deployed rule - admission strictly after max(last corridor sighting, start of the dwell in the admission
zone) + t_admit_stale, and every admission (timed, witness or B2) only at a step with the near-field clearance: no hull with
d_lower below admit_clear_m - with Pi1 (no centre in the CR before admission), Pi2 (progress >= v_cross_min after admission),
the P1 detection premise (d_lower is a lower bound of the centre distance) and the zone geometry (ST6: two drones whose
estimates are both in the admission zone are less than admit_clear_m apart) puts the first occupant beyond passage_clear_s at
the follower's admission, by cases: admitted before the dwell (the dwell alone covers it, no sighting needed) or during it
(impossible: the follower then dwells within admit_clear_m of it, so it had no clearance).  Neither premise S nor a near-field
sighting is needed (5f9f049 restarted the bound at every hull within 3.0 m: the drone queued behind the hold renewed it at
nearly every capture, no B0 lead admitted in v31).  ST1s: the earlier argument from the last sighting only, which needs premise
S; ST1d: without the dwell term it fails.  ST1p the P2 part (> cr_s_max) with the shorter bound.  Mutations: clearance dropped,
a zone wider than the clearance (the s-only zone of 5f9f049), bound - 2 s, no progress, and the e3e31f0 release (FREE after
t_occ_max, no admission bound) with an immediate admission.  ST7 (reported, progress): the queue standoff of a drone that
is not committed exceeds the admission clearance.
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
from holo_fleet.perception.gate_perception import entry_after, stale_release, timed_admission  # noqa: E402

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


def timeline_adm(cfg: FleetConfig, bound: float, s_end: float, clearance: bool = True, dwell: bool = True,
                 d_zone: float = None, v: float = None):
    """The deployed rule: occupant A admitted at t_a, the follower B dwells in the admission zone from t_o to its admission
    t_b, its last corridor sighting at t_s; B is admitted at t_b > max(t_s, t_o) + bound.  Every admission needs the
    near-field clearance: A's bound d_low of its distance to B at t_a is >= admit_clear_m, d_low <= the true distance d (the
    P1 detection premise), and B dwelling in the zone at t_a (t_a > t_o) is within d_zone of A (ST6).  No premise S, no
    near-field sighting."""
    G = cfg.gate
    v = G.v_cross_min if v is None else v
    t_a, t_o, t_s, t_b, s_a, s_b, d, d_low = z3.Reals("t_a t_o t_s t_b s_a s_b d d_low")
    cons = [s_a >= Q(G.stage2_hold_s) - Q(G.admit_zone_m) - Q(G.nav_err_m), t_b >= t_a, t_o <= t_b,
            t_b > t_s + Q(bound), s_b >= s_a + Q(v) * (t_b - t_a), s_b <= Q(s_end), d_low <= d]
    if dwell:
        cons.append(t_b > t_o + Q(bound))
    if clearance:
        cons.append(d_low >= Q(G.admit_clear_m))
    if d_zone is not None:
        cons.append(z3.Implies(t_a > t_o, d <= Q(d_zone)))
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
    """Largest true centre distance between two drones whose estimates are both in the admission zone
    (GateRule.in_admission_zone): along the axis admit_zone_m + admit_zone_fwd_m, laterally 2 admit_zone_lat_m, vertically
    2 (acq_dz_m + admit_zone_dz_m), plus the observer's and the admitted drone's horizontal position errors and the depth
    errors (numeric, triangle inequality)."""
    G = cfg.gate
    horiz = float(np.hypot(G.admit_zone_m + G.admit_zone_fwd_m, 2 * G.admit_zone_lat_m)) + e_obs + e_adm
    return float(np.hypot(horiz, 2 * G.zone_dz_half() + 2 * dz_err))


def queue_clearance(cfg: FleetConfig) -> CheckResult:
    """ST7 (reported, progress): the queue standoff of a drone that is not committed (control/flows.py,
    GateRule.queue_standoff_m) exceeds the admission clearance by the margin that leaves the lead drone at the hold its
    clearance at most captures (the onboard bound is 0.40-0.54 m below the true distance at the median, v31)."""
    G = cfg.gate
    margin = G.queue_standoff_m - G.admit_clear_m
    r = CheckResult(prop=f"ST7 reported: queue standoff {G.queue_standoff_m} m = admit_clear_m + {margin:.2f} m (a progress "
                         "margin; the queue points stay abreast, formal M2)", formula="reported", encoding="config",
                    expect="holds")
    r.verdict, r.passed = "holds", margin > 0.0
    r.note = ("a drone that is not committed backs off while a DYNAMIC FRONT hull is nearer than queue_standoff_m; the bound of "
              "the lead drone on it is then about queue_standoff_m plus the follower pessimism minus the lead pessimism (v32 "
              "with a follower 4.4 m behind the hold: >= 3.69 m at the 1st percentile)")
    return r


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
    for release, last, comm in itertools.product((None, "TIMER", "WITNESS"), (-1e9, 0.0, 12.5), (False, True)):
        n += 1
        want = -1e9 if (release == "WITNESS" or comm) else last + G.t_admit_stale
        if (abs(entry_after(release, last, G.t_admit_stale, comm) - want) > 1e-9
                or timed_admission(release, comm) != (release != "WITNESS" and not comm)):
            bad.append((release, last, comm))
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
    d_zone = zone_distance(cfg, G.zone_err_h_m, G.zone_err_h_m, G.zone_err_v_m)
    rep.add(check(f"ST1 deployed rule (dwell + corridor sighting, t_admit_stale = {T:.2f} s; near-field clearance "
                  f"admit_clear_m = {G.admit_clear_m} m at every admission), premises Pi1, Pi2, P1 detection, Z, zone geometry "
                  "(ST6), no premise S, no near-field sighting: the first occupant is beyond passage_clear_s at the follower's "
                  "admission", "t_b > max(t_s, t_o) + T, d_low(t_a) >= admit_clear_m, d_low <= d, t_a > t_o -> d <= d_zone, "
                  "s(t) >= s_a + v_cross_min (t - t_a) -> s(t_b) > passage_clear_s",
                  ENC, timeline_adm(cfg, T, G.passage_clear_s, d_zone=d_zone),
                  note=f"case t_a <= t_o: the dwell alone; case t_a > t_o: excluded by the clearance (d_zone = {d_zone:.3f} m)"),
            verbose)
    rep.add(check("ST1n mutation: an admission without the near-field clearance (an occupant admitted during the dwell of "
                  "another drone in the zone)", "expect counterexample", ENC,
                  timeline_adm(cfg, T, G.passage_clear_s, clearance=False, d_zone=d_zone), expect="sat"), verbose)
    rep.add(check("ST1z mutation: a zone wider than the clearance (the s-only zone of 5f9f049: two drones in it can be "
                  "farther apart than admit_clear_m)", "expect counterexample", ENC,
                  timeline_adm(cfg, T, G.passage_clear_s, d_zone=G.admit_clear_m + 1.0), expect="sat"), verbose)
    rep.add(check("ST1d mutation: the bound from the last corridor sighting only (no dwell term) without premise S",
                  "expect counterexample: an occupant admitted before the follower arrived, never seen by it", ENC,
                  timeline_adm(cfg, T, G.passage_clear_s, dwell=False, d_zone=d_zone), expect="sat"), verbose)
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
    # deployed (flows.Flows.gate), timed: W = commitment + wait, E = entry_after (corridor bound), h = hold start, Z = start of
    # the dwell in the admission zone (premise: reached within t_occ_max of the hold start), D = Z + T; admission at t when
    # t >= W, t > E, t > D and the conditions hold; give-up when not admitted and t - max(h + T, W, E) > t_occ_max + t_clear
    # (the anchor is fixed in advance: one full dwell from the hold start, not the dwell term itself)
    t, E, W, h, Z = z3.Reals("t E W h Z")
    arrive = G.t_occ_max
    D = Z + Q(T)
    admitted = z3.And(t >= W, t > E, t > D)
    mx = z3.If(h + Q(T) > W, h + Q(T), W)
    mx = z3.If(mx > E, mx, E)
    give_up = z3.And(z3.Not(admitted), t - mx > Q(G.t_occ_max + G.t_clear))
    rep.add(check("ST3 with the admission conditions holding in the zone (dwell begun within t_occ_max of the hold start) the "
                  "hold is never given up before admission: admission comes first",
                  "Z >= h, Z <= h + t_occ_max -> not (not admitted and t - max(h + T, W, E) > t_occ_max + t_clear)", ENC,
                  [give_up, Z >= h, Z <= h + Q(arrive)]), verbose)
    rep.add(check("ST3w witness: before the eligible time the drone waits at the hold", "expect SAT", ENC,
                  [z3.Not(admitted), t >= h, Z >= h], expect="sat"), verbose)
    rep.add(check("ST3g witness: a drone held outside the zone (no dwell) gives up: the give-up is reachable", "expect SAT",
                  ENC, [t - mx > Q(G.t_occ_max + G.t_clear), z3.Not(admitted)], expect="sat"), verbose)
    old_give_up = z3.And(z3.Not(admitted), t - z3.If(h > W, h, W) > G.t_occ_max + G.t_clear)
    rep.add(check("ST3m mutation: a give-up timer started from the hold and commitment only (not entry_after nor the dwell) "
                  "gives up before the timed admission (commit/give-up loop)", "expect counterexample", ENC,
                  [old_give_up, E > W], expect="sat"), verbose)
    t0 = z3.Real("t0")
    # f583bde: the anchor included the dwell term, which is t + T while the drone is out of the zone: never given up
    mo = z3.If(h > W, h, W)
    mo = z3.If(mo > E, mo, E)
    stuck = [t >= h + 1000.0, z3.Not(t - z3.If(mo > t + Q(T), mo, t + Q(T)) > G.t_occ_max + G.t_clear), t0 == t]
    rep.add(check("ST3o mutation: the f583bde give-up (anchored on the dwell term, obs.t + T out of the zone) never fires for a "
                  "drone held outside the zone (an endless hold, review finding)", "expect counterexample", ENC, stuck,
                  expect="sat"), verbose)
    rep.add(consistency(cfg), verbose)
    from holo_fleet.control.flows import ACQ_OFFSETS

    ok = d_zone < G.admit_clear_m and all(abs(dz) <= G.acq_dz_m + 1e-12 and abs(dl) <= G.admit_zone_lat_m
                                          for dl, dz in ACQ_OFFSETS)
    r = CheckResult(prop=f"ST6 zone geometry: under premise Z (estimate within {G.zone_err_h_m} m horizontally and "
                         f"{G.zone_err_v_m} m vertically of the truth while in the zone) two drones whose estimates are in the "
                         f"admission zone are at most {d_zone:.3f} m apart < admit_clear_m = {G.admit_clear_m} m; the "
                         "acquisition offsets lie in the zone",
                    formula="max true centre distance (triangle inequality) < admit_clear_m <= d_lower bound",
                    encoding="numeric bound over the zone extents (GateRule.in_admission_zone, flows.ACQ_OFFSETS, "
                             "structure_aiding constants)", expect="holds")
    r.verdict, r.passed = ("holds" if ok else "violated"), ok
    r.note = ("the clearance needs the hull's echo in some healthy sector (the P1 detection premise); a hull within "
              "admit_clear_m has d_lower < admit_clear_m since d_lower is a lower bound of the centre distance (S0); "
              "premise Z is declared (tighter than nav_err_m) and monitored by the referee (zone_nav)")
    rep.add(r, verbose)
    # ST6l: with the declared committed navigation error alone (nav_err_m per horizontal axis, nav_depth_err_m vertically)
    # two drones in the zone can be farther apart than the clearance
    xa, xb, ea, eb = [z3.RealVector(n, 3) for n in ("xa", "xb", "ea", "eb")]
    lim = []
    for x in (xa, xb):
        lim += [x[0] >= Q(G.stage2_hold_s - G.admit_zone_m), x[0] <= Q(G.stage2_hold_s + G.admit_zone_fwd_m),
                z3.Abs(x[1]) <= Q(G.admit_zone_lat_m), z3.Abs(x[2]) <= Q(G.zone_dz_half())]
    for e in (ea, eb):
        lim += [z3.Abs(e[0]) <= Q(G.nav_err_m), z3.Abs(e[1]) <= Q(G.nav_err_m),
                z3.Abs(e[2]) <= Q(cfg.acoustic.nav_depth_err_m)]
    dsq = z3.Sum([((xa[i] + ea[i]) - (xb[i] + eb[i])) ** 2 for i in range(3)])
    rep.add(check("ST6l declared limit: with only the committed navigation premise (nav_err_m per horizontal axis, the depth "
                  "allowance vertically) two drones in the admission zone can be farther apart than admit_clear_m: the "
                  "timed admission rests on premise Z", "expect counterexample", "nonlinear real arithmetic (Z3)",
                  lim + [dsq >= Q(G.admit_clear_m) ** 2], expect="sat"), verbose)
    rep.add(queue_clearance(cfg), verbose)
    r = CheckResult(prop=f"ST5 reported: t_admit_stale = {T:.2f} s (P2 part {T_p2:.2f} s, P1 clearance part {T - T_p2:.2f} s), "
                         f"t_occ_max = {G.t_occ_max:.0f} s", formula="reported", encoding="config", expect="holds")
    r.verdict, r.passed = "holds", True
    rep.add(r, verbose)
    return rep


if __name__ == "__main__":
    rep = run()
    rep.save()
    sys.exit(0 if rep.ok else 1)
