# Formal verification summary

Generated 2026-11-09 21:50:50 by `formal/check_properties.py`.

UNSAT = the negated property has no model in the abstraction, i.e. the property HOLDS for the model; SAT is expected for the satisfiability / non-vacuity checks (a witness must exist) and for the mutation tests (deliberately broken designs must yield a counterexample); mutation Om2 expects UNSAT (a broken observation domain loses the reachability of an edge).  The observation domain of every suite is the conjunction of the invariants of holo_fleet/ha/observation_invariants.py.

## Local determinism & priority hierarchy  (104/104 as expected, 0.6 s)

Encoding: `holo_fleet/ha/spec.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| D1 determinism [FORMATION_FOLLOW] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FORMATION_FOLLOW] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FORMATION_FOLLOW] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FORMATION_FOLLOW] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FORMATION_FOLLOW] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [FORMATION_FOLLOW] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [FORMATION_FOLLOW] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [FORMATION_FOLLOW] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FORMATION_FOLLOW] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [FORMATION_FOLLOW] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [SEPARATION_WARNING] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [SEPARATION_WARNING] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [SEPARATION_WARNING] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [SEPARATION_WARNING] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [SEPARATION_WARNING] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [SEPARATION_WARNING] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [SEPARATION_WARNING] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [SEPARATION_WARNING] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [SEPARATION_WARNING] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [SEPARATION_WARNING] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [COLLISION_AVOIDANCE] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [COLLISION_AVOIDANCE] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [COLLISION_AVOIDANCE] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [COLLISION_AVOIDANCE] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [COLLISION_AVOIDANCE] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [COLLISION_AVOIDANCE] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [COLLISION_AVOIDANCE] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [COLLISION_AVOIDANCE] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [COLLISION_AVOIDANCE] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [COLLISION_AVOIDANCE] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [MUTEX_APPROACH] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [MUTEX_APPROACH] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [MUTEX_APPROACH] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [MUTEX_APPROACH] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [MUTEX_APPROACH] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [MUTEX_APPROACH] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [MUTEX_APPROACH] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [MUTEX_APPROACH] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [MUTEX_APPROACH] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [MUTEX_APPROACH] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [MUTEX_YIELD] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [MUTEX_YIELD] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [MUTEX_YIELD] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [MUTEX_YIELD] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [MUTEX_YIELD] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [MUTEX_YIELD] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [MUTEX_YIELD] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [MUTEX_YIELD] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [MUTEX_YIELD] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [MUTEX_YIELD] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [MUTEX_PASS] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [MUTEX_PASS] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [MUTEX_PASS] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [MUTEX_PASS] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [MUTEX_PASS] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [MUTEX_PASS] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [MUTEX_PASS] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [MUTEX_PASS] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [MUTEX_PASS] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [MUTEX_PASS] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [FORMATION_RECOVERY] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FORMATION_RECOVERY] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FORMATION_RECOVERY] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FORMATION_RECOVERY] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FORMATION_RECOVERY] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [FORMATION_RECOVERY] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [FORMATION_RECOVERY] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [FORMATION_RECOVERY] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FORMATION_RECOVERY] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [FORMATION_RECOVERY] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [FAILSAFE_HOLD_OR_RETREAT] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FAILSAFE_HOLD_OR_RETREAT] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FAILSAFE_HOLD_OR_RETREAT] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FAILSAFE_HOLD_OR_RETREAT] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FAILSAFE_HOLD_OR_RETREAT] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [FAILSAFE_HOLD_OR_RETREAT] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [FAILSAFE_HOLD_OR_RETREAT] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [FAILSAFE_HOLD_OR_RETREAT] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FAILSAFE_HOLD_OR_RETREAT] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [FAILSAFE_HOLD_OR_RETREAT] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [FORMATION_WAIT_REJOIN] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FORMATION_WAIT_REJOIN] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FORMATION_WAIT_REJOIN] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FORMATION_WAIT_REJOIN] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FORMATION_WAIT_REJOIN] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [FORMATION_WAIT_REJOIN] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [FORMATION_WAIT_REJOIN] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [FORMATION_WAIT_REJOIN] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FORMATION_WAIT_REJOIN] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [FORMATION_WAIT_REJOIN] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| D1 determinism [DEGRADED_FORMATION] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [DEGRADED_FORMATION] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [DEGRADED_FORMATION] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [DEGRADED_FORMATION] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [DEGRADED_FORMATION] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 MUTEX_PASS entry condition [DEGRADED_FORMATION] | !committed & guard(->MUTEX_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback without give-up [DEGRADED_FORMATION] | committed & target in {MUTEX_APPROACH, MUTEX_YIELD} -> entry_abort | UNSAT | UNSAT | yes |
| G2c a give-up clears the commitment [DEGRADED_FORMATION] | committed & edge to {MUTEX_APPROACH, MUTEX_YIELD} enabled -> it sets committed := false | UNSAT | UNSAT | yes |
| G3 no commit under hazard [DEGRADED_FORMATION] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| L1 readable guards exact [DEGRADED_FORMATION] | forall legal obs, edge k: no earlier edge enabled -> (label_k <-> guard_k) | UNSAT | UNSAT | yes |
| M1 mutation: commit without priority must violate G1 | expect counterexample | SAT | SAT | yes |
| M2 mutation: gate pass ignoring collision risk must violate H2 | expect counterexample | SAT | SAT | yes |
| MA mutation: a give-up that keeps the commitment (the withdrawn retreat) must violate G2c | expect counterexample | SAT | SAT | yes |
| ML mutation: a wrong readable guard (collision risk drawn at d_warning) must violate L1 | expect counterexample | SAT | SAT | yes |

## Observation consistency (perception -> automaton interface)  (32/32 as expected, 1.3 s)

Encoding: `holo_fleet/ha/observation_invariants.py + holo_fleet/ha/spec.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| O1 the observation invariants are jointly satisfiable | exists obs: N1 & I1 & I2 & I3 & I4 | SAT | SAT | yes |
| O2 N1 ranges: excludes observations admitted by the other invariants | exists obs: others & !(form_err >= 0  and  t_ok >= 0  and  d_min <= D_NONE) | SAT | SAT | yes |
| O2 I1 priority only at the queue point: excludes observations admitted by the other invariants | exists obs: others & !(has_prio -> at_queue) | SAT | SAT | yes |
| O2 I2 the queue point lies in the approach zone: excludes observations admitted by the other invariants | exists obs: others & !(at_queue -> mutex_zone) | SAT | SAT | yes |
| O2 I3 a passed gate is out of the approach zone: excludes observations admitted by the other invariants | exists obs: others & !(passed -> !mutex_zone) | SAT | SAT | yes |
| O2 I4 t_ok counts only while the own slot error is ok: excludes observations admitted by the other invariants | exists obs: others & !(t_ok > 0 -> form_err < e_ok) | SAT | SAT | yes |
| O2 I5 an entry is given up only by a committed drone in the approach zone: excludes observations admitted by the other invariants | exists obs: others & !(entry_abort -> committed & mutex_zone & !passed) | SAT | SAT | yes |
| O3 non-vacuity [FORMATION_FOLLOW]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [SEPARATION_WARNING]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [COLLISION_AVOIDANCE]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [MUTEX_APPROACH]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [MUTEX_YIELD]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [MUTEX_PASS]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [FORMATION_RECOVERY]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [FAILSAFE_HOLD_OR_RETREAT]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [FORMATION_WAIT_REJOIN]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O3 non-vacuity [DEGRADED_FORMATION]: every edge enabled on some consistent observation | for each edge e: exists consistent obs with guard(e) | SAT | SAT | yes |
| O4 consistent observation -> exactly one edge [FORMATION_FOLLOW] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [SEPARATION_WARNING] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [COLLISION_AVOIDANCE] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [MUTEX_APPROACH] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [MUTEX_YIELD] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [MUTEX_PASS] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [FORMATION_RECOVERY] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [FAILSAFE_HOLD_OR_RETREAT] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [FORMATION_WAIT_REJOIN] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O4 consistent observation -> exactly one edge [DEGRADED_FORMATION] | invariants -> exactly one outgoing guard enabled | UNSAT | UNSAT | yes |
| O5 monitor: any observation -> exactly one edge; inconsistent -> only the fault edge to FAILSAFE | sense_ok' = sense_ok & consistent: exactly one guard, and !consistent -> target FAILSAFE | UNSAT | UNSAT | yes |
| Q1 calm, not committed, at the queue point -> commit or yield (every mode) | at_queue -> enabled edge in {commit, yield} | UNSAT | UNSAT | yes |
| Om1 mutation: without I2 a queued drone takes a formation edge (Q1 must fail) | expect counterexample | SAT | SAT | yes |
| Om2 mutation: v1 constraint passed -> at_queue makes pass_done unreachable | expect UNSAT (reachability lost) | UNSAT | UNSAT | yes |
| Om3 mutation: no monitor -> an inconsistent observation drives a mission edge | expect counterexample | SAT | SAT | yes |

## P1 inter-vehicle separation  (15/15 as expected, 227.8 s)

Encoding: `formal/check_separation.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| S0 onboard distance never above the true distance of the nearest hull (numeric, multi-hull scenes) | forall sampled scenes/errors/detection boundaries: d_hat <= min_O d(O) | HOLDS | HOLDS | yes |
| S0m mutation: uncertified bound (centre assumed inside every member cone) must overestimate | expect: some scene with d_hat > d | VIOLATED | VIOLATED | yes |
| S1a BMC: no violation within 4 s from any start with d_hat >= d_warning | Init & T^K -> G_[0,K] d >= d_safe | UNSAT | UNSAT | yes |
| S1b k-induction: G(d >= d_safe) for all time (k=13) | P(s_0..s_{k-1}) & T -> P(s_k), base case covered by S1a (K >= k) | UNSAT | UNSAT | yes |
| S1c configured d_warning above the smallest verified one | d_warning >= d_warning_min | HOLDS | HOLDS | yes |
| S1m mutation: d_warning=1.59 must violate P1 | expect counterexample (trace of d_i, c_i) | SAT | SAT | yes |
| S1o one-sided reaction at the configured a_brake: reported (not guaranteed) | Init & T^K -> G d >= d_safe, a_pair = a_brake | SAT | SAT | yes |
| S1d one-sided reaction at the calibrated deceleration 0.743 m/s^2: no violation within 4 s | Init & T^K -> G d >= d_safe, a_pair = a_brake_measured | UNSAT | UNSAT | yes |
| S2a CA escape, one threat: guaranteed opening per unit speed >= g_min (all patterns) | forall P: certified G(P, e*(P)) >= g_min | HOLDS | HOLDS | yes |
| S2b SW filter, one threat: velocity <= v_max with guaranteed opening >= v_open (all patterns) | forall P: exists v: min_u -v.u >= v_open | HOLDS | HOLDS | yes |
| S2c CA escape, two threats within 90 deg: positive guarantee | forall P1,P2 with all region directions <= 90 deg apart: max_e min G > 0 | HOLDS | HOLDS | yes |
| S2d vertical escapes: FRONT+DOWN -> up component, FRONT+UP -> down component | e*(F+D).z > 0.35 and e*(F+U).z < -0.35 | HOLDS | HOLDS | yes |
| S2m mutation: 'always reverse' escape must fail S2a for some pattern | expect: some P with G(P, REAR) < g_min | VIOLATED | VIOLATED | yes |
| S3 at most one vertex angle >= 90 deg (pairwise d >= d_safe) | d_ij >= d_safe -> not(angle_i >= 90 and angle_j >= 90) | UNSAT | UNSAT | yes |
| S4 blackout <= 0.8s starting at d >= 2.34 m keeps d >= d_safe | FAILSAFE hold with unrejected drift w_rel | UNSAT | UNSAT | yes |

## P2 critical-region mutual exclusion  (43/44 as expected, 602.3 s)

Encoding: `holo_fleet/ha/mutex_rule.py + formal/check_mutex.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| M1 never both PRIORITY (|delta| <= 12 deg, fuzz 9 deg) | /delta/ + 2 fuzz <= 30 -> not(PRIORITY_i and PRIORITY_j) | UNSAT | UNSAT | yes |
| M1m mutation: fuzz 11 deg (|delta| + 2 fuzz > 30) must allow double PRIORITY | expect counterexample (beta, delta) | SAT | SAT | yes |
| M1m2 mutation: yield only to FRONT (LEFT ignored) must allow double PRIORITY | expect counterexample | SAT | SAT | yes |
| M2 queue n=2: for every occupancy with <= 1 vacant point between neighbours (3 sets), exactly the leftmost occupied drone has PRIORITY | queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first) | UNSAT | UNSAT | yes |
| M2 queue n=3: for every occupancy with <= 1 vacant point between neighbours (7 sets), exactly the leftmost occupied drone has PRIORITY | queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first) | UNSAT | UNSAT | yes |
| M2 queue n=4: for every occupancy with <= 1 vacant point between neighbours (14 sets), exactly the leftmost occupied drone has PRIORITY | queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first) | UNSAT | UNSAT | yes |
| M2 queue n=5: for every occupancy with <= 1 vacant point between neighbours (26 sets), exactly the leftmost occupied drone has PRIORITY | queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first) | UNSAT | UNSAT | yes |
| M2 queue n=6: for every occupancy with <= 1 vacant point between neighbours (46 sets), exactly the leftmost occupied drone has PRIORITY | queue_tol, heading_tol -> PRIORITY_first(S) and not PRIORITY_k (k in S, k > first) | UNSAT | UNSAT | yes |
| M2s scope: two adjacent vacant queue points (n=4, occupied {0, 3}) -> both PRIORITY | expect SAT: outside the scope of M2, the occupancy latch (M3) is the only protection | SAT | SAT | yes |
| M2v stacked queues (2 x 2, and the 3-drone stack_pair of the demo): one relation per pair over the tolerances, exactly the first in the order left first, then top first has PRIORITY for every occupancy (15 + 7) | grid of +-queue_tol, +-heading_tol, +-fuzz -> unique relation; leader = first(S); no RANK | HOLDS | HOLDS | yes |
| M4s stacked queue geometry: CR inside every FRONT cone; each pass path keeps merge_clearance from the queue points of the drones after it | 3-D bearing + heading_tol + fuzz <= 60 deg; min distance >= merge_clearance | HOLDS | HOLDS | yes |
| M2m mutation: the v2 rule (vertical-only pattern -> RANK, ranks unknown) deadlocks the stacked queue (no drone has PRIORITY) | expect violation | VIOLATED | VIOLATED | yes |
| M2vm mutation: stacked columns only queue_spacing (3.5 m) apart -> a diagonal pair is ambiguous | expect violation | VIOLATED | VIOLATED | yes |
| M4a CR inside the FRONT cone of every queue point (n = 2..6) | max CR bearing + heading_tol + fuzz <= 60 deg | HOLDS | HOLDS | yes |
| M4b merge path keeps merge_clearance from the next queue point (n = 2..6) | min distance >= merge_clearance | HOLDS | HOLDS | yes |
| M4c queued neighbours outside the warning band (holding error <= 0.1 m) | d_lower(neighbour) >= d_warning_exit | HOLDS | HOLDS | yes |
| M4d a crossing drone is seen unmasked in the corridor for >= 3 captures | exposure / v_pass >= 0.3 s | HOLDS | HOLDS | yes |
| M4e masking interval is bounded and starts after the corridor is entered | corridor_from < mask_from and visible_again <= cr_half_len + 2 m | HOLDS | HOLDS | yes |
| M4f queue neighbours a decision needs are never hidden in the gate's structure window (abreast: adjacent pairs, n = 2..6; stacked: every pair; +-hold_err) | echo of the neighbour outside [window - tol, window + tol_far] of every sector that sees it | HOLDS | HOLDS | yes |
| M4g exit observable, abreast n=2: beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g exit observable, abreast n=3: beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g domain boundary, abreast n=4: refused by the planner (queue points beyond queue_lateral_max) | outside the validated domain: no obligation; reported | VIOLATED | VIOLATED | yes |
| M4g domain boundary, abreast n=5: refused by the planner (queue points beyond queue_lateral_max) | outside the validated domain: no obligation; reported | VIOLATED | VIOLATED | yes |
| M4g domain boundary, abreast n=6: refused by the planner (queue points beyond queue_lateral_max) | outside the validated domain: no obligation; reported | VIOLATED | VIOLATED | yes |
| M4g exit observable, stack 2 x 2 (n=4): beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g exit observable, stack_pair (n=3, demo): beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g exit observable, template triangle (n=3, gate scenarios): beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g exit observable, template line3 (n=3, gate scenarios): beyond the CR every passing drone gives every queued drone an exit witness (any sector, bars' occlusion, measured detection range) for >= 3 captures | exists path interval: in cone, line of sight clear, outside the structure window, range <= 10.0 m, W1 at the worst echo error | HOLDS | HOLDS | yes |
| M4g domain boundary, stack 3 + 2 (n=5): refused by the planner (queue points beyond queue_lateral_max) | outside the validated domain: no obligation; reported | VIOLATED | VIOLATED | yes |
| M4fm mutation: the stacked queue at the cone-only line (no structure clearance) hides the diagonal neighbours | expect violation | VIOLATED | VIOLATED | yes |
| M4m mutation: n=4 queue line at s = -4 m must violate M4a | expect violation | VIOLATED | VIOLATED | yes |
| M3 occupancy latch, n=2, observer at l=+1.75: never both in the CR (42 s BMC) | latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR) | UNSAT | UNSAT | yes |
| M3h occupancy latch, n=2, observer at l=+1.75: the crossing drone may stop anywhere before the CR: never both in the CR | as M3, the speed floor applying only inside the CR | UNSAT | UNKNOWN | **NO** |
| M3 occupancy latch, n=3, observer at l=+3.50: never both in the CR (42 s BMC) | latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR) | UNSAT | UNSAT | yes |
| M3h occupancy latch, n=3, observer at l=+3.50: the crossing drone may stop anywhere before the CR: never both in the CR | as M3, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| M3 occupancy latch, n=3, observer at l=+0.00: never both in the CR (42 s BMC) | latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR) | UNSAT | UNSAT | yes |
| M3h occupancy latch, n=3, observer at l=+0.00: the crossing drone may stop anywhere before the CR: never both in the CR | as M3, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| M3 occupancy latch, n=4, observer at l=+5.25: never both in the CR (42 s BMC) | latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR) | UNSAT | UNSAT | yes |
| M3h occupancy latch, n=4, observer at l=+5.25: the crossing drone may stop anywhere before the CR: never both in the CR | as M3, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| M3 occupancy latch, n=4, observer at l=+1.75: never both in the CR (42 s BMC) | latch + persistence + exit + t_clear + t_occ_max -> not(A in CR and B in CR) | UNSAT | UNSAT | yes |
| M3h occupancy latch, n=4, observer at l=+1.75: the crossing drone may stop anywhere before the CR: never both in the CR | as M3, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| M3m mutation: belief without latch (FREE when nothing is seen) must violate P2 | expect counterexample | SAT | SAT | yes |
| M3b mutation: a FRONT-only echo nearer than the corridor makes neither a relation nor a latch input (round 2, F1): must violate P2 | expect counterexample | SAT | SAT | yes |
| M3f scope: an occupant that stops inside the CR (fault) is released by the t_occ_max fallback | expect counterexample: crossing speed >= V_LO is an assumption of M3 | SAT | SAT | yes |

## P3 formation recovery (liveness, ranking functions)  (24/24 as expected, 3.4 s)

Encoding: `formal/check_formation.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| F1 along: ranking decrease in the saturated band [1.11, 8.0] m | lo <= /e/ <= hi -> /e'/ <= /e/ - eps_far | UNSAT | UNSAT | yes |
| F2 along: ranking decrease in the linear band [0.44, 1.11] m | lo <= /e/ <= hi -> /e'/ <= /e/ - eps_near | UNSAT | UNSAT | yes |
| F3 along: the ball |e| <= 0.44 m is invariant | /e/ <= e* -> /e'/ <= e* | UNSAT | UNSAT | yes |
| F1 lateral: ranking decrease in the saturated band [0.89, 8.0] m | lo <= /e/ <= hi -> /e'/ <= /e/ - eps_far | UNSAT | UNSAT | yes |
| F2 lateral: ranking decrease in the linear band [0.36, 0.89] m | lo <= /e/ <= hi -> /e'/ <= /e/ - eps_near | UNSAT | UNSAT | yes |
| F3 lateral: the ball |e| <= 0.36 m is invariant | /e/ <= e* -> /e'/ <= e* | UNSAT | UNSAT | yes |
| F4a RECOVERY & calm & no gate & own slot recovered -> only the edge to the selected formation mode | guard(e) & target(e) != FOLLOW / WAIT_REJOIN / DEGRADED (by neighbors_ok, degraded) is unsatisfiable | UNSAT | UNSAT | yes |
| F4b ... and that edge is enabled | self_recovered -> guard(edge to the selected mode) | UNSAT | UNSAT | yes |
| F4c FORMATION_FOLLOW & calm & no gate & own slot lost -> FORMATION_RECOVERY | self_lost -> target = RECOVERY | UNSAT | UNSAT | yes |
| F4d FORMATION_FOLLOW & own slot ok & neighbour missing -> FORMATION_WAIT_REJOIN | !self_lost & !neighbors_ok -> target = WAIT_REJOIN | UNSAT | UNSAT | yes |
| F4c FORMATION_WAIT_REJOIN & calm & no gate & own slot lost -> FORMATION_RECOVERY | self_lost -> target = RECOVERY | UNSAT | UNSAT | yes |
| F4d FORMATION_WAIT_REJOIN & own slot ok & neighbour missing -> FORMATION_WAIT_REJOIN | !self_lost & !neighbors_ok -> target = WAIT_REJOIN | UNSAT | UNSAT | yes |
| F4c DEGRADED_FORMATION & calm & no gate & own slot lost -> FORMATION_RECOVERY | self_lost -> target = RECOVERY | UNSAT | UNSAT | yes |
| F4d DEGRADED_FORMATION & own slot ok & neighbour missing -> FORMATION_WAIT_REJOIN | !self_lost & !neighbors_ok -> target = WAIT_REJOIN | UNSAT | UNSAT | yes |
| F6 bounded waiting: a neighbour that never reappears is declared vacant within t_rejoin = 30.0 s (300 running steps), and the drone leaves FORMATION_WAIT_REJOIN | ranking t_rejoin - tau: after N running steps the WAIT_REJOIN self-loop is disabled | UNSAT | UNSAT | yes |
| F6r ranking: t_rejoin - tau >= 0 and decreases by dt per running step until the slot is vacant | 0 <= tau < t_rejoin & tau' = tau + dt -> (t_rejoin - tau') = (t_rejoin - tau) - dt | UNSAT | UNSAT | yes |
| F6g bounded waiting with the gate-delay hold: max(H - h, 0) + (t_rejoin - tau) >= 0 strictly decreases every running step (by dt on a timer step, by min(dt, H - h) > 0 on a hold step, at most ceil(H / dt) of them), so a neighbour that never reappears is vacant within t_rejoin + gate_delay_max + dt | h >= 0 & 0 <= tau < t_rejoin & step -> R' < R & R' >= 0 | UNSAT | UNSAT | yes |
| F6gm mutation: a hold without its bound H may never let the timer run (vacancy never declared) | expect counterexample | SAT | SAT | yes |
| Fm4 mutation: no timeout (v2 baseline) -> the drone may wait for ever (still waiting after N steps) | expect counterexample | SAT | SAT | yes |
| F5 envelope at the recovery speed -> the catch-up of F1 keeps its margin | g(V + h) <= A  ->  min(V, xs - h) - v_clock - s_rate - w_res > 0 | UNSAT | UNSAT | yes |
| Fm3 mutation: envelope at the survey speed only -> the catch-up margin of F1 can vanish | expect counterexample | SAT | SAT | yes |
| Em1 mutation: the v1-v2 scalar bound |w| <= 0.6 m/s admits head currents with which the survey speed is not deliverable | expect counterexample | SAT | SAT | yes |
| Fm1 mutation: no catch-up margin (v_recovery = v_clock) must break F1 (along) | expect counterexample | SAT | SAT | yes |
| Fm2 mutation: k_slot = 0 must break F2 (lateral) | expect counterexample | SAT | SAT | yes |

## Variant B2 range-bearing beacon (own assumptions; B0 checks unchanged)  (13/13 as expected, 63.8 s)

Encoding: `perception/acoustic.py + perception/acoustic_gate.py + formal/check_acoustic.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| B2-A1 envelope coverage: every position compatible with a measurement inside the envelope lies in the region the perception builds | max over draws of (distance to samples - slack) <= 0 | HOLDS | HOLDS | yes |
| B2-A2 decision soundness: no measurement inside the envelope (multipath excess included) of a hull in the CR (any queue point n=2..6, heading within heading_tol) passes the outside test (by region or by range) or the exclusion test | forall CR grid x envelope corners: region or range clear of the CR -> false; could be occupant | HOLDS | HOLDS | yes |
| B2-M4g availability (reported), n=2: every queued drone gets an acoustic exit evidence of every passing drone (bars' and queued hulls' occlusion, worst envelope error, worst ping phase) and can account for the other queued drones | exists 3 consecutive clear fixes with s_lower > exit face before the path end; queued drones excluded at t_lc | HOLDS | HOLDS | yes |
| B2-M4g availability (reported), n=3: every queued drone gets an acoustic exit evidence of every passing drone (bars' and queued hulls' occlusion, worst envelope error, worst ping phase) and can account for the other queued drones | exists 3 consecutive clear fixes with s_lower > exit face before the path end; queued drones excluded at t_lc | HOLDS | HOLDS | yes |
| B2-M4g domain boundary, n=4: refused by the planner (queue points beyond queue_lateral_max), reported | exists 3 consecutive clear fixes with s_lower > exit face before the path end; queued drones excluded at t_lc | HOLDS | HOLDS | yes |
| B2-M4g domain boundary, n=5: refused by the planner (queue points beyond queue_lateral_max), reported | exists 3 consecutive clear fixes with s_lower > exit face before the path end; queued drones excluded at t_lc | VIOLATED | VIOLATED | yes |
| B2-M4g domain boundary, n=6: refused by the planner (queue points beyond queue_lateral_max), reported | exists 3 consecutive clear fixes with s_lower > exit face before the path end; queued drones excluded at t_lc | VIOLATED | VIOLATED | yes |
| B2-P payload soundness: a sender truly in the CR whose estimate errs within its sent bound (cylinder |e_h| <= h, |e_z| <= z, radius hypot(h, z), through the 56-bit packet) is never cleared by the payload rule nor placed beyond the exit face | forall sampled centres in the CR and errors in the cylinder: rep_cr_possible and not A3 | HOLDS | HOLDS | yes |
| B2-Pm mutation: the radius max(h, z) (not a ball over the cylinder) must clear an occupant | expect counterexample | VIOLATED | VIOLATED | yes |
| B2-PE phantom-echo soundness: a fleet drone truly in the CR, heard inside the declared envelope (multipath, ages up to the tie window, with or without a payload within its bound), is never excluded by the phantom-echo rule | randomized: no_fleet_drone -> false | HOLDS | HOLDS | yes |
| B2-PEg phantom-echo soundness of the fix geometry alone (no status in any packet): an occupant is never excluded | randomized: no_fleet_drone -> false, status removed | HOLDS | HOLDS | yes |
| B2-PEm mutation: without the keep-out claim a drone carried into the CR uncommitted reports not engaged and is excluded | expect counterexample | VIOLATED | VIOLATED | yes |
| B2-S acoustic fixes never reach separation, collision avoidance, precedence or the sonar witness; acoustic exit evaluation requires the FRONT corridor empty (H6: the payload balls reach the map fit only as its identity premise, through Perception, never as a position input) | AST: no acoustic name in the safety / precedence modules; guarded call | HOLDS | HOLDS | yes |

## Payload v2: gate status, silence, verified passage  (27/27 as expected, 322.6 s)

Encoding: `perception/payload.py + perception/acoustic_gate.py + perception/passage.py + formal/check_status.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| S1 positive occupancy: a report of committed or admitted of another drone makes the latch BUSY and holds the entry | randomized: claim -> occ_busy and not entry_ok | HOLDS | HOLDS | yes |
| S1m mutation: claims ignored, the latch is not held | expect violation | VIOLATED | VIOLATED | yes |
| S2h M3-status n=2, observer l=+1.75, the crossing drone may stop before the CR: never both in the CR | as S2, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| S2 M3-status n=2, observer l=+1.75, freshness of a report of not engaged 1.95 s, v 0.7, offset 1.35: never both in the CR | latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits) | UNSAT | UNSAT | yes |
| S2h M3-status n=3, observer l=+3.50, the crossing drone may stop before the CR: never both in the CR | as S2, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| S2 M3-status n=3, observer l=+3.50, freshness of a report of not engaged 2.50 s, v 0.7, offset 1.35: never both in the CR | latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits) | UNSAT | UNSAT | yes |
| S2h M3-status n=3, observer l=+0.00, the crossing drone may stop before the CR: never both in the CR | as S2, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| S2 M3-status n=3, observer l=+0.00, freshness of a report of not engaged 2.50 s, v 0.7, offset 1.35: never both in the CR | latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits) | UNSAT | UNSAT | yes |
| S2h M3-status n=4, observer l=+5.25, the crossing drone may stop before the CR: never both in the CR | as S2, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| S2 M3-status n=4, observer l=+5.25, freshness of a report of not engaged 2.50 s, v 0.7, offset 1.35: never both in the CR | latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits) | UNSAT | UNSAT | yes |
| S2h M3-status n=4, observer l=+1.75, the crossing drone may stop before the CR: never both in the CR | as S2, the speed floor applying only inside the CR | UNSAT | UNSAT | yes |
| S2 M3-status n=4, observer l=+1.75, freshness of a report of not engaged 2.50 s, v 0.7, offset 1.35: never both in the CR | latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits) | UNSAT | UNSAT | yes |
| S2m mutation: a freshness of 5 s (the first design) violates P2 at the declared worst case | expect counterexample | SAT | SAT | yes |
| S2b mutation: no latch input in the band of a FRONT-only echo (round 2, F1) violates P2 | expect counterexample | SAT | SAT | yes |
| S3 evidence order: a status older than a later fix placing the drone possibly in the CR never excludes it | randomized: last accepted fix in the CR with t_tx > status t_tx -> not excluded | HOLDS | HOLDS | yes |
| S3m mutation: without the evidence order the stale status excludes the drone | expect violation | VIOLATED | VIOLATED | yes |
| S4 silence: a drone silent after reporting committed or admitted keeps the gate held (claim, never excluded), at any duration | randomized over silence durations up to a day | HOLDS | HOLDS | yes |
| S4m mutation: silence releasing a claim is detected | expect violation | VIOLATED | VIOLATED | yes |
| S5 text search: no acoustic or status name in the relation loop of GatePerception.update (does not see indirect paths) | text: the relation loop and decision of GatePerception.update | HOLDS | HOLDS | yes |
| S6 drones at abreast queue points are never FRONT-only targets of each other (n=2,3, tolerances; only those points) | enumeration over queue points, tolerances, heading tolerance | HOLDS | HOLDS | yes |
| S7 verified passage: never verified with the true centre inside the CR grown by R_OUT, a blocked drone never verifies | randomized: fit error, DVL scale and dropouts, blocked drones | HOLDS | HOLDS | yes |
| S9 sender rule: a true centre in the CR (estimate within the declared navigation error) is in the keep-out box, and inside the box the report is a claim for every history | randomized + enumeration of passage.report_phase | HOLDS | HOLDS | yes |
| S9m mutation: a box without the navigation-error margin leaves an occupant outside | expect violation | VIOLATED | VIOLATED | yes |
| S8 freshness deployed by GatePerception per layout: positive and below the shortest time from the queue to the CR at the speed bound | 0 < status_fresh_s <= (/queue_s/ - CR - R_OUT - queue_tol - nav_err) / v_bound - latency | HOLDS | HOLDS | yes |
| S6b no relation implies latch input (real class, no beacon): a FRONT-only echo anywhere in the band makes no relation and the latch is BUSY | randomized over the band | HOLDS | HOLDS | yes |
| S11 silence or a stale report never discards a corridor echo | randomized: no fresh evidence -> no_fleet_drone false | HOLDS | HOLDS | yes |
| S11m mutation: letting silence exclude a drone discards the echo | expect violation | VIOLATED | VIOLATED | yes |

**Overall: SOME CHECKS FAILED**
