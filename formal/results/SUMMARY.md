# Formal verification summary

Generated 2026-10-06 17:02:34 by `formal/check_properties.py`.

UNSAT = the negated property has no model in the abstraction, i.e. the property HOLDS for the model; SAT is expected only for the mutation tests (deliberately broken designs must yield a counterexample).

## Local determinism & priority hierarchy  (66/66 as expected, 0.2 s)

Encoding: `holo_fleet/ha/spec.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| D1 determinism [FORMATION_FOLLOW] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FORMATION_FOLLOW] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FORMATION_FOLLOW] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FORMATION_FOLLOW] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FORMATION_FOLLOW] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [FORMATION_FOLLOW] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [FORMATION_FOLLOW] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FORMATION_FOLLOW] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [SEPARATION_WARNING] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [SEPARATION_WARNING] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [SEPARATION_WARNING] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [SEPARATION_WARNING] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [SEPARATION_WARNING] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [SEPARATION_WARNING] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [SEPARATION_WARNING] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [SEPARATION_WARNING] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [COLLISION_AVOIDANCE] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [COLLISION_AVOIDANCE] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [COLLISION_AVOIDANCE] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [COLLISION_AVOIDANCE] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [COLLISION_AVOIDANCE] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [COLLISION_AVOIDANCE] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [COLLISION_AVOIDANCE] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [COLLISION_AVOIDANCE] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [GATE_APPROACH] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [GATE_APPROACH] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [GATE_APPROACH] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [GATE_APPROACH] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [GATE_APPROACH] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [GATE_APPROACH] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [GATE_APPROACH] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [GATE_APPROACH] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [GATE_YIELD] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [GATE_YIELD] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [GATE_YIELD] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [GATE_YIELD] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [GATE_YIELD] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [GATE_YIELD] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [GATE_YIELD] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [GATE_YIELD] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [GATE_PASS] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [GATE_PASS] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [GATE_PASS] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [GATE_PASS] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [GATE_PASS] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [GATE_PASS] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [GATE_PASS] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [GATE_PASS] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [FORMATION_RECOVERY] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FORMATION_RECOVERY] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FORMATION_RECOVERY] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FORMATION_RECOVERY] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FORMATION_RECOVERY] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [FORMATION_RECOVERY] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [FORMATION_RECOVERY] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FORMATION_RECOVERY] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| D1 determinism [FAILSAFE_HOLD_OR_RETREAT] | forall legal obs: at most one outgoing guard enabled | UNSAT | UNSAT | yes |
| D2 completeness [FAILSAFE_HOLD_OR_RETREAT] | forall legal obs: some guard enabled | UNSAT | UNSAT | yes |
| H1 fault=>FAILSAFE [FAILSAFE_HOLD_OR_RETREAT] | fault -> target = FAILSAFE | UNSAT | UNSAT | yes |
| H2 d<d_ca=>COLLISION_AVOIDANCE [FAILSAFE_HOLD_OR_RETREAT] | !fault & d_min < d_ca -> target = COLLISION_AVOIDANCE | UNSAT | UNSAT | yes |
| H3 warning band=>SEPARATION_WARNING [FAILSAFE_HOLD_OR_RETREAT] | !fault & d_ca_exit <= d_min < d_warning -> target = SEPARATION_WARNING | UNSAT | UNSAT | yes |
| G1 GATE_PASS entry condition [FAILSAFE_HOLD_OR_RETREAT] | !committed & guard(->GATE_PASS) -> at_queue & !occ_busy & has_prio | UNSAT | UNSAT | yes |
| G2 no PASS->YIELD fallback [FAILSAFE_HOLD_OR_RETREAT] | committed -> target not in {GATE_APPROACH, GATE_YIELD} | UNSAT | UNSAT | yes |
| G3 no commit under hazard [FAILSAFE_HOLD_OR_RETREAT] | commit edge enabled -> d_min >= d_warning (or >= d_warning_exit from SW) | UNSAT | UNSAT | yes |
| M1 mutation: commit without priority must violate G1 | expect counterexample | SAT | SAT | yes |
| M2 mutation: gate pass ignoring collision risk must violate H2 | expect counterexample | SAT | SAT | yes |

## P1 inter-vehicle separation  (9/9 as expected, 288.1 s)

Encoding: `formal/check_separation.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| S1a BMC: no violation within 12 s from any admissible start | Init & T^K -> G_[0,K] d >= d_safe | UNSAT | UNSAT | yes |
| S1b k-induction: G(d >= d_safe) for all time (k=11) | P(s_0..s_{k-1}) & T -> P(s_k), with base case covered by S1a (K >= k) | UNSAT | UNSAT | yes |
| S1m mutation: d_warning=1.97 must violate P1 | expect counterexample (trace of d_i, c_i) | SAT | SAT | yes |
| S2a SW filter feasible with v_open when threats <= 90 deg apart | u1.u2 >= 0 -> exists v, /v/ <= cap: v.u_j <= -v_open (witness -0.4 (u1+u2)//u1+u2/) | UNSAT | UNSAT | yes |
| S2b CA escape opens every threat at >= v_open/v_escape when <= 90 deg apart | theta <= 90 & vertical term away from threats -> e.u_j <= -(v_open/v_escape)/e/ | UNSAT | UNSAT | yes |
| S2c CA escape opens a single threat at >= v_open/v_escape | e = normalise(-u + w vz z), vz.u_z <= 0 -> e.u <= -v_open/v_escape | UNSAT | UNSAT | yes |
| S2m mutation: strong vertical escape regardless of threat geometry must break S2b | expect counterexample | SAT | SAT | yes |
| S3 at most one vertex angle >= 90 deg (pairwise d >= d_safe) | d_ij >= d_safe -> not(angle_i >= 90 and angle_j >= 90) | UNSAT | UNSAT | yes |
| S4 blackout <= 0.8s starting at d >= 2.08 m keeps d >= d_safe | FAILSAFE hold with unrejected drift w_rel | UNSAT | UNSAT | yes |

## P2 critical-region mutual exclusion  (17/17 as expected, 0.1 s)

Encoding: `holo_fleet/ha/gate_rule.py + formal/check_mutex.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| X0a decision classes mutually exclusive | no measurement satisfies two classes | UNSAT | UNSAT | yes |
| X0b decision classes exhaustive | every measurement satisfies some class | UNSAT | UNSAT | yes |
| M1 no simultaneous commit (independent errors <= eps) | COMMIT_i(meas_i) & COMMIT_j(meas_j) is impossible | UNSAT | UNSAT | yes |
| M2a no mutual waiting (deadlock freedom of the pairwise rule) | never (WAIT_i & WAIT_j): at least one commits or retreats | UNSAT | UNSAT | yes |
| M2b robust wait is justified | WAIT_ROBUST_i -> COMMIT_j | UNSAT | UNSAT | yes |
| M2c retreat targets are consistent (never both BACKOFF_REAR) | BACKOFF_REAR_i & BACKOFF_REAR_j impossible (sign of ds is robust) | UNSAT | UNSAT | yes |
| M3 priority persistence during the transit window | COMMIT_j(t0) & A_mono & (along-tied at t0 -> A_hold) -> not COMMIT_i(t) | UNSAT | UNSAT | yes |
| M3b late arrivals cannot commit before the committed drone is visible | t_arrival(i) >= (approach_len - commit_window)/v_approach > t_visible(j) | UNSAT | UNSAT | yes |
| M4 committed drone past occ_gamma+eps is perceived in the occupied zone | s_j in [s_q+gamma+eps, exit-eps], /l_j/ <= W-eps -> occ_busy_i | UNSAT | UNSAT | yes |
| M5a queued drones are outside the critical region | s <= s_q + hold_tol_s -> not in CR | UNSAT | UNSAT | yes |
| M5b exited drones are outside the critical region | s > cr_half_len + occ_exit_margin - eps -> not in CR | UNSAT | UNSAT | yes |
| M6a invariant 'at most one drone committed' is inductive | Inv & T -> Inv'  (T constrained by M1, M3, M4) | UNSAT | UNSAT | yes |
| M6b initial state satisfies the invariant | all queued -> Inv | UNSAT | UNSAT | yes |
| M6c invariant + M5 implies G(occupancy_CR <= 1) | only committed drones can be inside the CR (M5), at most one is committed | UNSAT | UNSAT | yes |
| Mm1 mutation: gap band mu_s_hi-mu_s_lo=0.65 <= 2eps breaks M3 | expect counterexample | SAT | SAT | yes |
| Mm2 mutation: margins below eps (mu_s_hi=0.4) allow a double commit | expect counterexample | SAT | SAT | yes |
| Mm3 mutation: 'commit or wait' (no retreat classes) allows mutual waiting | expect counterexample | SAT | SAT | yes |

## P3 bounded formation recovery  (10/10 as expected, 6.3 s)

Encoding: `formal/check_formation.py`

| check | formula | expected | verdict | ok |
|---|---|---|---|---|
| B invariance of the along tolerance box (E <= 0.519 m) | E <= tol -> E' <= tol | UNSAT | UNSAT | yes |
| B invariance of the lateral tolerance box (E <= 0.229 m, within |y| <= 0.275 m) | /y/ <= A1 & E <= tol -> E' <= tol | UNSAT | UNSAT | yes |
| L lane-error box |y_i| <= 2.0 m is invariant | /y/ <= box -> /y'/ <= box | UNSAT | UNSAT | yes |
| L1 lane-error box |y_i| <= 0.275 m is invariant | /y/ <= box -> /y'/ <= box | UNSAT | UNSAT | yes |
| F3 tolerances fit the recovered threshold e_ok | sqrt(0.519^2 + 0.229^2) + 0.03 <= e_ok = 0.6 | UNSAT | UNSAT | yes |
| R ranking lemmas - along: decrease >= delta_k per step in every band down to 0.519 m | E in [lo_k, hi_k] -> E' <= E - delta_k  (one step, all noise, all saturations) | UNSAT | UNSAT | yes |
| R ranking lemmas - lateral phase 1 (lane error, saturated regime): decrease >= delta_k per step in every band down to 0.275 m | E in [lo_k, hi_k] -> E' <= E - delta_k  (one step, all noise, all saturations) | UNSAT | UNSAT | yes |
| R ranking lemmas - lateral phase 2 (formation error inside |y| <= A1): decrease >= delta_k per step in every band down to 0.229 m | E in [lo_k, hi_k] -> E' <= E - delta_k  (one step, all noise, all saturations) | UNSAT | UNSAT | yes |
| F4 worst-case recovery bound T=60.0 s <= referee deadline 75.0 s | sum_k ceil(width_k/delta_k) dt_f <= T_deadline (axes recover concurrently) | UNSAT | UNSAT | yes |
| Fm mutation: without along-track consensus the error does not decrease | expect counterexample | SAT | SAT | yes |

**Overall: ALL CHECKS AS EXPECTED**
