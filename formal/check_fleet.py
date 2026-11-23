"""Bounded model checking of the interacting automata of the fleet at the gate (Z3).

The local checks (determinism, separation, the latch M3, the freshness S2, the precedence M2, the ranking functions of
the formation) are per drone or per pair.  This model composes n drones and checks what only the composition shows:
P2 across the fleet, the absence of a deadlock between the drones that passed and those that queue, the timers, the
permanent failures and the conditional recovery of the formation.

Abstraction (discrete time, one step = DT seconds; timers rounded up to steps).  Each drone is in one of
  A approach, Q queued, C committed (stage-2 hold before the CR), X in the CR, B passed and waiting beyond the gate (in the
  FRONT corridor of the queue: the reduced-progress wait of the formation), D formation re-established, F dead (a hull).
Assumed from the local checks, not re-proved here: the latch of a queued drone is BUSY while another drone is in C or X
(M3, S1, S2: the claim and the corridor echo arrive within the time from the queue to the CR); a drone in X leaves it
(speed floor inside the CR, M3); the leftmost queued drone present has the priority (M2), a dead hull at its queue point
counting as present until it has risen (T_RISE).  Modelled here:
  * the latch keeps BUSY while an echo stays in the corridor (a drone waiting at B, or a hull that died there and has not
    risen yet) unless every other drone is accounted for at that receiver: a live drone by a fresh fix, a dead drone only by
    rule D (not heard for t_silent_gate_s), never without it;
  * a live drone's fix is fresh at a receiver nondeterministically, with the fairness premise that within every FAIR steps
    there is one step at which every live drone is fresh at every receiver (premise V and the channel model);
  * the stage-2 hold gives up after T_HOLD steps (t_occ_max + t_clear) and the drone queues again;
  * a drone waiting at B leaves for D only when no live drone has still to pass and every dead drone is vacant (t_silent_s);
  * separation: before step L any live drone can be stopped for any number of steps (collision avoidance preempts the gate
    modes, H1-H3), the timers running;
  * permanent failures: before step L, in A, Q, C, B or D, never in X; the scope (holo_fleet/scope.py) puts them outside the
    exclusion zone with the margin a receiver adds to a fix, where rule D applies (inside it a dead drone holds the gate for ever:
    out of the scope, not modelled).  A drone that fails committed keeps its claim until it is known dead;
  * rule D at a receiver needs another live drone it hears (acoustic.py: a receiver that hears nobody declares no one dead).
Premises of the recovery: H, a dead hull stops counting as a queued drone for the sonar precedence once it has risen (it does
not when it floats at the surface above the queue in still water: F2s4, a declared limit); n = 2 at most one
failure and none while committed; n = 3, 4 at most n - 2 failures (every receiver
keeps hearing another drone).  The two cases left out are declared limits and checked as such (F2s2, F2s3).  Not modelled: the
admission blocked by the localization of the gate (loc_ok; the give-up branch of the hold is unreachable under the invariant, so
that case is outside F2), the release delays of the latch (t_occ_max, t_clear, exit confirmation) beyond
one step.  The entry from C to X needs no live drone in X and no claim of another drone known at the receiver (S1: entry_ok of the
code holds the entry on a heard report of committed or admitted).  F0 and F2 take the claims as learnt within the step; F1d relaxes
it: the latch and the stage-2 check see the claims d steps late.  P2 then holds while d does not exceed the shortest stay in C
(one step) plus the step the precedence puts between two commitments: d <= 2 with M2, d <= 1 without it (tight: F1d3, F1d2n).
An entry that bypasses the stage-2 check (a committed drone moved into the CR by separation warning or collision avoidance while it
holds; the drone keeps its claim: report_phase, S9) keeps the invariant (F0p, F2p).  With a claim learnt one step late two drones
can be committed together (F1d1 shows the stage-2 check alone still keeps them apart), and with entries by avoidance as well the
v11 controller had a counterexample: one drone enters by its own stage-2 check before the claim of the other is known, and the
other, held by that claim, is moved in by avoidance while the first is inside; it survives the motion limit of push_dwell.  The
escape planner alone is not antisymmetric along the axis (F1pa), so nothing excluded that forward motion.  The corrected
controller (control/gate_safety.py) edits the final command of every drone that is not admitted to the CR - committed or queued,
mission modes and avoidance modes alike: a command is applied only if the backup that follows it (the command for one control
period plus the largest data age, braking at a_brake, holding with the error of the drift) keeps the true centre out of the CR
under the declared navigation error and drift (formal/check_gate_safety.py: P2B1 per-step soundness, P2B1r the computed reach,
P2B3 adversarial trajectories).  So, while the declared premises hold, there is no entry by avoidance and F1p, the combination of
claims learnt one step late with the entries of the corrected controller, reduces to F1d1: never two drones in X over the depth
(bounded model checking from the launch, not an induction; the stage-2 check is made at the hold, at most
(cr_s_min - stage2_hold_s + 0.4) / v_pass = 3.8 s before the entry, within one step).  Outside the declared envelope (a
navigation error above nav_err_m, a drift above w_drift_max, an estimate jump, or a step with no P2-safe command: P2_RECOVERY or
JOINTLY_INFEASIBLE with P2 uncertified, logged by the controller; P2B2s shows that a P2-safe command with a margin below the
step deficit can lead there) entries by avoidance remain possible, and the counterexample stays: F1pe and F1pme (with the motion
limit) are declared limits with their trace.  Likewise F0qe, a queued drone moved into the CR (inside the envelope a queued drone
is filtered too and F0s covers it).  F1pk (formal/check_f1p_kinematic.py, reported) gives the case positions with the deployed
flows.
The 32 s bound of S2 is a different quantity (how long a report of not engaged may exclude a corridor echo) and does not bound
the delay of a claim between two drones that commit together.
Properties, for n = 2, 3, 4.  F0: the invariant (at most one drone in C or X) holds at the launch and is inductive under every
step, failures and stops included, so P2 holds at every time with no bound (k-induction, k = 1).  F2: from any state of the
invariant, with no further failure or stop and the fairness of the fixes, every live drone reaches D within K steps: after the
last failure and the last stop of an execution the fleet re-forms (P3, or P3-deg with the vacancies) within K * DT seconds of
the abstraction; F2t shows K tight.  Mutations and scope: F1m a latch that ignores the drones in C lets two into the CR; F2m
without rule D the composition deadlocks (a drone
waiting beyond the gate in the corridor, a queued drone whose latch it holds, a dead drone never accounted for); F2f without the
fairness premise the recovery is not guaranteed.  F3 non-vacuity: from the launch, a run with a failure re-forms.
The model does not cover: the geometry and timing inside the CR (M3, S2, M4), positions (the waiting point is assumed inside
the corridor, the worst case), the formation errors (the ranking functions of the P3 suite), failures inside the exclusion zone.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import z3

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402

ENC = "formal/check_fleet.py"
A, Q, C, X, B, D, F = range(7)
NAMES = "abcdef"


def timers(cfg: FleetConfig = DEFAULT, dt: float = 5.0) -> dict:
    """Timers in steps of dt seconds, rounded up (the order of the thresholds is kept: vacancy before death)."""
    from holo_fleet.scope import obstruction_half
    from holo_fleet.sim.scenarios import SCENARIOS

    up = lambda s: int(math.ceil(s / dt - 1e-9))
    S = cfg.scope
    h_top = float(obstruction_half(SCENARIOS["lost_drone_mutex"](cfg).judged_gates[0], cfg)[2])
    return {"DT": dt, "T_DEAD": up(cfg.acoustic.t_silent_gate_s), "T_VAC": up(cfg.form.t_silent_s),
            "T_HOLD": up(cfg.gate.t_occ_max + cfg.gate.t_clear),
            "T_RISE": up(S.rise_lag_s + (h_top + S.coast_v_m) / S.v_rise_min_mps), "T_APP": 4, "FAIR": up(35.0)}


class Fleet:
    def __init__(self, n: int, depth: int, L: int, T: dict, rule_d: bool = True, fair: bool = True, latch_claims: bool = True,
                 free: bool = False, max_fail: int = None, fail_in_c: bool = True, hull_clears: bool = True, latch_delay: int = 0,
                 stage2_claims: bool = True, precedence: bool = True, push: str = "none", push_min_ct: int = 0):
        """free: the first state is any state of the invariant (inv), not the launch; L: failures and stops only before step L;
        max_fail: at most this many permanent failures (default n - 1); fail_in_c: failures also while committed, before the CR.
        latch_delay: the latch and the stage-2 check see the claims of the others latch_delay steps late (premise C relaxed);
        stage2_claims: the entry from C to X also needs no claim of another drone known at the receiver (S1, entry_ok of the code:
        a heard report of committed or admitted holds the entry), seen with the same delay; precedence False: no sonar precedence
        between the queued drones (M2 gives no relation, as two drones out of sonar range of each other).
        push: an entry into the CR that bypasses the stage-2 check (separation warning or collision avoidance moving a drone that
        holds before the CR): "committed" a drone in C may enter X at any step; "any" also a queued drone.
        push_min_ct: a push needs at least this many steps already spent in C (the motion limit of push_dwell; the natural
        entry needs one)."""
        max_fail = n - 1 if max_fail is None else max_fail
        self.fail_in_c = fail_in_c
        self.n, self.depth, self.L, self.T = n, depth, L, T
        self.cap = cap = max(T.values()) + 2
        S = z3.Solver()
        m = [[z3.Int(f"m{NAMES[i]}_{t}") for i in range(n)] for t in range(depth + 1)]
        ct = [[z3.Int(f"ct{NAMES[i]}_{t}") for i in range(n)] for t in range(depth + 1)]
        age = [[z3.Int(f"age{NAMES[i]}_{t}") for i in range(n)] for t in range(depth + 1)]
        loc = [[z3.Int(f"loc{NAMES[i]}_{t}") for i in range(n)] for t in range(depth + 1)]
        busy = [[z3.Bool(f"busy{NAMES[i]}_{t}") for i in range(n)] for t in range(depth + 1)]
        stall = [[z3.Bool(f"stall{NAMES[i]}_{t}") for i in range(n)] for t in range(depth)]
        pushed = [[z3.Bool(f"push{NAMES[i]}_{t}") if push != "none" else z3.BoolVal(False) for i in range(n)] for t in range(depth)]
        fail = [[z3.Bool(f"fail{NAMES[i]}_{t}") for i in range(n)] for t in range(depth)]
        acc = [[[z3.Bool(f"acc{NAMES[i]}{NAMES[k]}_{t}") for k in range(n)] for i in range(n)] for t in range(depth + 1)]
        self.m, self.age, self.busy, self.fail, self.ct, self.loc = m, age, busy, fail, ct, loc
        sat = lambda e: z3.If(e > cap, cap, e)
        dead = lambda t, i: m[t][i] == F
        if free:
            S.add(self.inv(0))
        else:
            for i in range(n):
                S.add(m[0][i] == A, ct[0][i] == 0, age[0][i] == 0, loc[0][i] == -1)
        for t in range(depth + 1):
            for i in range(n):
                S.add(m[t][i] >= A, m[t][i] <= F, ct[t][i] >= 0, age[t][i] >= 0)
        hears = lambda t, i, k: z3.Or([z3.Not(dead(t, l)) for l in range(n) if l not in (i, k)] or [z3.BoolVal(False)])
        if rule_d:
            known = lambda t, i, k: z3.If(dead(t, k), z3.And(age[t][k] >= T["T_DEAD"], hears(t, i, k)), acc[t][i][k])
        else:
            known = lambda t, i, k: z3.And(z3.Not(dead(t, k)), acc[t][i][k])
        allfresh = lambda t: z3.And([z3.Or(dead(t, k), acc[t][i][k]) for i in range(n) for k in range(n) if k != i])
        if fair:
            for t in range(0, depth + 2 - T["FAIR"]):
                S.add(z3.Or([allfresh(s) for s in range(t, min(t + T["FAIR"], depth + 1))]))
        for t in range(depth + 1):
            for i in range(n):
                ts = max(t - latch_delay, 0)
                occ = z3.Or([z3.Or(z3.And(z3.Not(dead(ts, k)), z3.Or(m[ts][k] == X, m[ts][k] == C) if latch_claims else m[ts][k] == X),
                                   z3.And(dead(ts, k), loc[ts][k] == C, z3.Not(known(ts, i, k))))
                             for k in range(n) if k != i] or [z3.BoolVal(False)])
                echo = z3.Or([z3.Or(m[t][k] == B, z3.And(dead(t, k), z3.Or(loc[t][k] == B, loc[t][k] == C), age[t][k] < T["T_RISE"]))
                              for k in range(n) if k != i] or [z3.BoolVal(False)])
                allk = z3.And([known(t, i, k) for k in range(n) if k != i] or [z3.BoolVal(True)])
                prev = busy[t - 1][i] if t > 0 else (z3.Bool(f"busy{NAMES[i]}_init") if free else z3.BoolVal(False))
                S.add(busy[t][i] == z3.Or(occ, z3.And(prev, echo, z3.Not(allk))))
        def claim_known(t, i):
            ts = max(t - latch_delay, 0)
            return z3.Or([z3.Or(z3.And(z3.Not(dead(ts, k)), z3.Or(m[ts][k] == C, m[ts][k] == X)),
                                z3.And(dead(ts, k), loc[ts][k] == C, z3.Not(known(ts, i, k))))
                          for k in range(n) if k != i] or [z3.BoolVal(False)])

        nf = []
        for t in range(depth):
            for i in range(n):
                mi, mn, c, cn = m[t][i], m[t + 1][i], ct[t][i], ct[t + 1][i]
                others = [k for k in range(n) if k != i]
                live_x = z3.Or([z3.And(m[t][k] == X) for k in others] or [z3.BoolVal(False)])
                seen = (lambda tt, kk: age[tt][kk] < T["T_RISE"]) if hull_clears else (lambda tt, kk: z3.BoolVal(True))
                prio = z3.And([z3.Not(z3.Or(m[t][k] == Q, z3.And(dead(t, k), z3.Or(loc[t][k] == Q, loc[t][k] == C), seen(t, k))))
                               for k in range(i)] or [z3.BoolVal(True)]) if precedence else z3.BoolVal(True)
                enter_ok = z3.And(z3.Not(live_x), z3.Not(claim_known(t, i))) if stage2_claims else z3.Not(live_x)
                pending = z3.Or([z3.Or(m[t][k] == A, m[t][k] == Q, m[t][k] == C, m[t][k] == X) for k in others] or [z3.BoolVal(False)])
                vacant = z3.And([z3.Implies(dead(t, k), age[t][k] >= T["T_VAC"]) for k in others] or [z3.BoolVal(True)])
                if t >= L:
                    S.add(z3.Not(fail[t][i]), z3.Not(stall[t][i]))
                S.add(z3.Implies(fail[t][i], z3.Or([mi == A, mi == Q, mi == B, mi == D] + ([mi == C] if fail_in_c else []))))
                nf.append(z3.If(fail[t][i], 1, 0))
                S.add(age[t + 1][i] == z3.If(dead(t, i), sat(age[t][i] + 1), 0))
                S.add(loc[t + 1][i] == z3.If(fail[t][i], mi, loc[t][i]))
                stay = z3.And(mn == mi, cn == sat(c + 1))
                go = lambda tgt: z3.And(mn == tgt, cn == 0)
                live_step = z3.If(mi == A, z3.If(c >= T["T_APP"], go(Q), z3.Or(stay, z3.And(c >= 1, go(Q)))),
                            z3.If(mi == Q, z3.If(z3.And(prio, z3.Not(busy[t][i])), go(C), stay),
                            z3.If(mi == C, z3.If(enter_ok,
                                                 z3.If(c >= 2, go(X), z3.Or(stay, z3.And(c >= 1, go(X)))),
                                                 z3.If(c >= T["T_HOLD"], go(Q), stay)),
                            z3.If(mi == X, z3.If(c >= 2, go(B), z3.Or(stay, z3.And(c >= 1, go(B)))),
                            z3.If(mi == B, z3.If(z3.And(z3.Not(pending), vacant), go(D), stay),
                                  stay)))))
                pushable = z3.Or(mi == C, mi == Q) if push == "any" else (mi == C)
                S.add(z3.Implies(pushed[t][i], z3.And(pushable, c >= push_min_ct)))
                S.add(z3.If(dead(t, i), z3.And(mn == F, cn == 0),
                      z3.If(fail[t][i], z3.And(mn == F, cn == 0),
                      z3.If(pushed[t][i], go(X),
                      z3.If(stall[t][i], stay, live_step)))))
        S.add(z3.Sum(nf + [z3.If(m[0][i] == F, 1, 0) for i in range(n)]) <= max_fail)
        self.S = S

    def inv(self, t):
        """The invariant: at most one drone in C or X (a commit needs a FREE latch, which every drone in C or X holds BUSY,
        and the priority, which one queued drone has at a time); the bookkeeping of the hulls and the timers."""
        m, ct, age, loc, n, cap = self.m, self.ct, self.age, self.loc, self.n, self.cap
        one = z3.Sum([z3.If(z3.Or(m[t][i] == C, m[t][i] == X), 1, 0) for i in range(n)]) <= 1
        book = [z3.And(m[t][i] >= A, m[t][i] <= F, ct[t][i] >= 0, ct[t][i] <= cap, age[t][i] >= 0, age[t][i] <= cap,
                       z3.If(m[t][i] == F, z3.Or([loc[t][i] == A, loc[t][i] == Q, loc[t][i] == B, loc[t][i] == D] + ([loc[t][i] == C] if self.fail_in_c else [])),
                             z3.And(loc[t][i] == -1, age[t][i] == 0))) for i in range(n)]
        return z3.And([one] + book)

    def live_at(self, t, i):
        return self.m[t][i] != F

    def two_in_cr(self):
        return z3.Or([z3.And(self.m[t][i] == X, self.m[t][k] == X) for t in range(self.depth + 1)
                      for i in range(self.n) for k in range(i + 1, self.n)])

    def not_recovered(self):
        t = self.depth
        return z3.Or([z3.And(self.live_at(t, i), self.m[t][i] != D) for i in range(self.n)])

    def recovered_with_failure(self):
        t = self.depth
        return z3.And(z3.And([z3.Or(self.m[t][i] == D, self.m[t][i] == F) for i in range(self.n)]),
                      z3.Or([self.m[t][i] == F for i in range(self.n)]))


def _run(fl: Fleet, prop) -> list:
    return list(fl.S.assertions()) + [prop]


def push_dwell(cfg: FleetConfig, n: int, dt: float) -> tuple:
    """Motion limit of an entry by avoidance: the shortest path of a committed drone from its queue point (within queue_tol of
    the queue line) to the CR entry plane, cr_s_min - queue_s(n) - queue_tol, at the highest ground speed of the Envelope (v_escape plus
    the unrejected drift w_drift_max, the speed bound of the P1 proof).  Returns (seconds, steps already spent in C before a push
    can reach the CR, rounded up like every timer of the model)."""
    G, env = cfg.gate, cfg.env
    t_min = (G.cr_s_min - G.queue_s(n) - G.queue_tol) / (env.v_escape + env.w_drift_max)
    return t_min, max(int(math.ceil(t_min / dt - 1e-9)) - 1, 0)


def modes(fl: Fleet, prop) -> str:
    """Mode series of one counterexample of ``prop``, one letter per step (A Q C X B D F), up to the first step with two drones
    in X, and the steps with an entry by avoidance.  Solved again, so not necessarily the model stored with the check."""
    S = z3.Solver()
    S.set("timeout", 600000)
    S.add(*_run(fl, prop))
    if S.check() != z3.sat:
        return "no counterexample"
    M, n, letters = S.model(), fl.n, "AQCXBDF"
    rows, pushes = ["" for _ in range(n)], []
    for t in range(fl.depth + 1):
        ms = [M.eval(fl.m[t][i], model_completion=True).as_long() for i in range(n)]
        for i in range(n):
            rows[i] += letters[ms[i]]
        if sum(m == X for m in ms) >= 2:
            break
        pushes += [(t, NAMES[i]) for i in range(n) if t < fl.depth and z3.is_true(M.eval(z3.Bool(f"push{NAMES[i]}_{t}"),
                                                                                           model_completion=True))]
    return " | ".join(f"{NAMES[i]}: {rows[i]}" for i in range(n)) + f"; entries by avoidance (step, drone): {pushes}"


MIRROR = {"FRONT": "REAR", "REAR": "FRONT", "LEFT": "RIGHT", "RIGHT": "LEFT", "UP": "DOWN", "DOWN": "UP"}


def escape_forward_pairs(cfg: FleetConfig = DEFAULT) -> list:
    """Two drones heading along the gate axis that see each other (mirrored sector patterns), each at or beyond its hold point (the
    hold pulls it back or holds it: the mission velocity has no forward component): the patterns for which the deployed escape
    planner can move both forward (collision avoidance, the warning filter, the band of the filter), over headings, mission
    velocities and current estimates.  Non-empty: the planner gives no antisymmetry along the axis to rule out two pushes at once."""
    import numpy as np

    from holo_fleet.control.escape import EscapePlanner

    pl, env = EscapePlanner(cfg), cfg.env
    vds = [np.array(v, float) for v in ([0, 0, 0], [-0.1, 0, 0], [-0.3, 0, 0], [-0.1, 0.1, 0], [-0.1, -0.1, 0], [0, 0.2, 0],
                                        [0, -0.2, 0], [-0.05, 0, 0.05], [0, 0, -0.1])]

    def forward(P):
        for hd in np.linspace(0.0, 2 * np.pi, 13)[:-1]:
            for vd in vds:
                for cur in (None, np.array([0.1, 0.0, 0.0]), np.array([-0.1, 0.0, 0.0])):
                    pl.reset()
                    if pl.escape_velocity([(P, env.v_open)], [], hd, env.v_escape, current_body=cur, mission_body=vd).v_body[0] > 1e-6:
                        return True
                for req in (env.v_open, 0.0):
                    pl.reset()
                    if pl.warning_velocity(vd, [(P, req)], [], hd, env.v_max_nominal).v_body[0] > 1e-6:
                        return True
        return False
    return [sorted(P) for P in pl.regions if forward(P) and forward(frozenset(MIRROR[x] for x in P))]


K_FREE = {2: 26, 3: 40, 4: 52}
PREMISES = {2: dict(max_fail=1, fail_in_c=False), 3: dict(max_fail=1), 4: dict(max_fail=2)}


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("fleet")
    T = timers(cfg)
    tag = (f"DT {T['DT']:.0f} s, T_DEAD {T['T_DEAD']}, T_VAC {T['T_VAC']}, T_HOLD {T['T_HOLD']}, T_RISE {T['T_RISE']}, FAIR {T['FAIR']} "
           f"steps")
    for n in (2, 3, 4):
        kw = PREMISES[n]
        prem = ("at most one failure, none while committed" if n == 2 else f"at most {kw['max_fail']} failure(s), also while committed")
        fl = Fleet(n, 0, 0, T, **kw)
        rep.add(check(f"F0i n={n}: the launch satisfies the invariant", "init -> inv", ENC, _run(fl, z3.Not(fl.inv(0))), "unsat",
                      note=tag), verbose)
        fl = Fleet(n, 1, 1, T, free=True, **kw)
        rep.add(check(f"F0s n={n}: the invariant is inductive under every step, failures ({prem}) and stops included",
                      "inv & step -> inv'  (inv: at most one drone in C or X; bookkeeping)", ENC, _run(fl, z3.Not(fl.inv(1))),
                      "unsat", note=tag), verbose)
        K = K_FREE[n]
        fl = Fleet(n, K, 0, T, free=True, **kw)
        rep.add(check(f"F2 n={n} ({prem}): from any state of the invariant, with no further failure or stop, every live drone "
                      f"re-forms within {K} steps",
                      "inv & fairness of the fixes -> all live drones in D at K (no deadlock, no livelock)", ENC,
                      _run(fl, fl.not_recovered()), "unsat", timeout_ms=900000, note=tag), verbose)
        fl = Fleet(n, K - 2, 0, T, free=True, **kw)
        rep.add(check(f"F2t n={n}: the bound is tight (some state needs more than {K - 2} steps)", "expect a run short of D at K - 2",
                      ENC, _run(fl, fl.not_recovered()), "sat", timeout_ms=900000, note=tag), verbose)
    L, K = 12, K_FREE[3]
    fl = Fleet(3, L + K, L, T, **PREMISES[3])
    rep.add(check("F3 non-vacuity, n=3: from the launch, a run with a permanent failure re-forms the degraded formation",
                  "exists a run with a failure and every live drone in D", ENC, _run(fl, fl.recovered_with_failure()), "sat",
                  timeout_ms=600000, note=tag), verbose)
    fs2 = Fleet(2, 60, 0, T, free=True, max_fail=1, fail_in_c=True)
    rep.add(check("F2s2 declared limit: with two drones, a failure while committed blocks the survivor (it hears nobody, so it declares "
                  "no one dead)", "expect a run short of D at 60 steps", ENC, _run(fs2, fs2.not_recovered()), "sat",
                  timeout_ms=900000, note=tag), verbose)
    fs3 = Fleet(3, 60, 0, T, free=True, max_fail=2)
    rep.add(check("F2s3 declared limit: the last survivor of three is blocked by a dead drone that failed committed",
                  "expect a run short of D at 60 steps", ENC, _run(fs3, fs3.not_recovered()), "sat", timeout_ms=900000, note=tag), verbose)
    fh = Fleet(3, 60, 0, T, free=True, hull_clears=False, **PREMISES[3])
    rep.add(check("F2s4 declared limit: without premise H (a hull floating above the queue keeps counting as a queued drone) the "
                  "survivors yield to it for ever (med12)", "expect a run short of D at 60 steps", ENC, _run(fh, fh.not_recovered()),
                  "sat", timeout_ms=900000, note=tag), verbose)
    fm = Fleet(3, K, 0, T, free=True, rule_d=False, **PREMISES[3])
    rep.add(check("F2m mutation: without rule D a dead drone is never accounted for and the composition deadlocks (n=3)",
                  "expect a run with a live drone short of D at K", ENC, _run(fm, fm.not_recovered()), "sat",
                  timeout_ms=900000, note=tag), verbose)
    ff = Fleet(3, K, 0, T, free=True, fair=False, **PREMISES[3])
    rep.add(check("F2f scope: without the fairness of the fixes the recovery is not guaranteed (n=3)",
                  "expect a run with a live drone short of D at K", ENC, _run(ff, ff.not_recovered()), "sat",
                  timeout_ms=900000, note=tag), verbose)
    for d, prec, exp in ((1, True, "unsat"), (2, True, "unsat"), (3, True, "sat"), (1, False, "unsat"), (2, False, "sat")):
        fd = Fleet(3, L + K, L, T, latch_delay=d, precedence=prec, **PREMISES[3])
        what = "the sonar precedence M2" if prec else "no precedence between the queued drones (M2 gives no relation)"
        rep.add(check(f"F1d{d}{'' if prec else 'n'} premise C relaxed, n=3, {what}: the latch and the stage-2 check learn of a claim {d} "
                      f"step(s) ({d * T['DT']:.0f} s) late; {'never two drones in X' if exp == 'unsat' else 'two drones in X (the bound is tight)'}",
                      "no two in X over the depth (from the launch)" if exp == "unsat" else "expect a run with two drones in X", ENC,
                      _run(fd, fd.two_in_cr()), exp, timeout_ms=600000, note=tag), verbose)
    for d in (1, 2):
        fd = Fleet(3, L + K, L, T, latch_delay=d, stage2_claims=False, **PREMISES[3])
        rep.add(check(f"F1c{d} mutation: a stage-2 check that ignores the claims (only a drone in X holds the entry) with a latch {d} step(s) "
                      "late lets two into the CR (n=3)", "expect a run with two drones in X", ENC, _run(fd, fd.two_in_cr()), "sat",
                      timeout_ms=600000, note=tag), verbose)
    for n in (2, 3, 4):
        fp = Fleet(n, 1, 1, T, free=True, push="committed", **PREMISES[n])
        rep.add(check(f"F0p n={n}: with an entry into the CR that bypasses the stage-2 check (a committed drone moved by separation warning or "
                      "collision avoidance, at any step) the invariant stays inductive",
                      "inv & step (push of a drone in C to X) -> inv'", ENC, _run(fp, z3.Not(fp.inv(1))), "unsat", note=tag), verbose)
        fp = Fleet(n, K_FREE[n], 0, T, free=True, push="committed", **PREMISES[n])
        rep.add(check(f"F2p n={n}: the recovery within {K_FREE[n]} steps holds with those pushes", "inv & pushes & fairness -> all live drones in D at K",
                      ENC, _run(fp, fp.not_recovered()), "unsat", timeout_ms=900000, note=tag), verbose)
    fp = Fleet(3, L + K, L, T, latch_delay=1, push="none", **PREMISES[3])
    G = cfg.gate
    t_adm = (G.cr_s_min - G.stage2_hold_s + 0.4) / G.v_pass
    rep.add(check("F1p corrected controller, n=3: claims learnt one step late (two drones can commit together) and entries into the "
                  "CR only through the stage-2 check, an entry by avoidance from a P2-safe state being excluded by the P2 filter "
                  "(check_gate_safety P2B1, P2B3) while the declared premises hold: never two drones in X over the depth",
                  "no two in X over the depth (from the launch), push = none", ENC, _run(fp, fp.two_in_cr()), "unsat",
                  timeout_ms=600000, note=tag + f"; the same composition as F1d1; admission at the stage-2 hold at most "
                                                f"{t_adm:.1f} s before the entry (< DT)"), verbose)
    fp = Fleet(3, L + K, L, T, latch_delay=1, push="committed", **PREMISES[3])
    r = check("F1pe declared limit: outside the declared envelope of the P2 filter (navigation error above nav_err_m, drift above "
              "w_drift_max, an estimate jump, no P2-safe command) a committed drone can still be moved into the CR by avoidance, and "
              "with claims learnt one step late two drones are in X (n=3; the v11 counterexample of F1p)",
              "expect a run with two drones in X", ENC, _run(fp, fp.two_in_cr()), "sat", timeout_ms=600000, note=tag)
    if r.verdict == "sat":
        r.note += "; one counterexample trace " + modes(fp, fp.two_in_cr())
    rep.add(r, verbose)
    t_push, k_push = push_dwell(cfg, 3, T["DT"])
    fp = Fleet(3, L + K, L, T, latch_delay=1, push="committed", push_min_ct=k_push, **PREMISES[3])
    r = check(f"F1pme declared limit: F1pe with the motion limit of an entry by avoidance (at least {t_push:.1f} s from the queue "
              f"point to the CR, {k_push} step(s) in C before a push): still two drones in X (n=3): a concurrent commitment, one "
              "entry by the stage-2 check before the claim of the other is known, then the other moved in while the first is inside",
              "expect a run with two drones in X", ENC, _run(fp, fp.two_in_cr()), "sat", timeout_ms=600000, note=tag)
    if r.verdict == "sat":
        r.note += "; one counterexample trace " + modes(fp, fp.two_in_cr())
    rep.add(r, verbose)
    fq = Fleet(3, 1, 1, T, free=True, push="any", **PREMISES[3])
    rep.add(check("F0qe declared limit: outside the declared envelope of the P2 filter a queued, uncommitted drone moved into the CR "
                  "by avoidance breaks the invariant (inside it the queued drones are filtered too: F0s)",
                  "expect inv & push of a drone in Q -> not inv'", ENC, _run(fq, z3.Not(fq.inv(1))), "sat", note=tag), verbose)
    import time
    from common import CheckResult
    t0 = time.time()
    pairs = escape_forward_pairs(cfg)
    rep.add(CheckResult("F1pa scope: the deployed escape planner, without the P2 filter, is not antisymmetric along the gate axis "
                        "(two drones at their hold points seeing each other can both be moved forward)", "exists mirrored patterns with a forward choice for both",
                        "holo_fleet/control/escape.py (exhaustive over the sector patterns)", "counterexample",
                        "counterexample" if pairs else "none", bool(pairs), round(time.time() - t0, 3),
                        note=f"{len(pairs)} mirrored patterns, e.g. {pairs[:3]}"), verbose)
    import check_f1p_kinematic
    rep.add(check_f1p_kinematic.check(cfg), verbose)
    fc = Fleet(3, L + K, L, T, latch_claims=False, stage2_claims=False, **PREMISES[3])
    rep.add(check("F1m mutation: a latch that ignores the drones in the stage-2 hold, with a stage-2 check that ignores the claims, lets two into the CR (n=3, from the launch)",
                  "expect a run with two drones in X", ENC, _run(fc, fc.two_in_cr()), "sat", timeout_ms=600000, note=tag), verbose)
    return rep


if __name__ == "__main__":
    r = run()
    r.save()
    print("all as expected" if r.ok else "SOME CHECK NOT AS EXPECTED")
