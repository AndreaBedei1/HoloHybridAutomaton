# REPORT v2 - leaderless, communication-free UUV fleet with six realistic sonars and verifiable local hybrid automata

Branch `realistic-sonar-formations-v2`.  v1 (25-ray idealised proximity ring, imaging sonar, three drones) is
summarised in [baseline_v1](results/baseline_v1) and in the git history up to `ad092aa`.

## 1. Contribution of v2

1. **Realistic sensing.** Every drone carries six identical `SinglebeamSonar` sensors, one per hull face:
   120 deg cone, 0.3-12 m, 5 cm bins, 10 Hz, intensity and range noise.  They return echo profiles
   without bearing.  No idealised sensor is left.
2. **The simulator fixed at the root.** A minimal HoloOcean 2.3.0 patch (four commits, twelve files,
   [docs/HOLOOCEAN_OCTREE_PATCH.md](docs/HOLOOCEAN_OCTREE_PATCH.md)) makes the sonars see the scene that
   is really there:
   * runtime props are visible;
   * stale caches leave no ghosts;
   * a reset no longer crashes the engine;
   * every agent is visible, whatever its initialisation order;
   * faces on cell boundaries are kept.

   The patch passes a regression test and is not submitted upstream.
3. **An echo is not a drone.** Every echo is classified STRUCTURE, SEABED, DYNAMIC or UNKNOWN from
   onboard information and the gate map, with M-of-N confirmation.  Targets live on sector patterns, and
   their distance has a **sound** conservative bound.
4. **3-D behaviour from occupied sectors.**
   * A certified warning filter and escape planner on sector patterns, including vertical escapes.
   * A communication-free traffic rule.
   * A sector-based critical-region rule with an occupancy latch.
   * Generic formation templates with sonar-based relative correction and a planned rendezvous beyond
     gates.
5. **Formal verification redone for the new sensing.**
   * P1: one-sided distance bound, exhaustive escape lemmas on the deployed planner.
   * P2: sector rule, N = 2..6 queues, timed latch.
   * P3: liveness by ranking functions, with explicit fairness and no time bound.
   * Determinism and mutation tests are kept.
6. **Demos with a dashboard** that separates what each drone knows from the ground truth.
7. **Observation consistency.**
   * The relations that the perception guarantees between the variables of the abstract observation
     are written once (`holo_fleet/ha/observation_invariants.py`).
   * The same module is the runtime check between perception and automaton: a violation is logged and
     the automaton takes the existing fault edge to FAILSAFE.
   * It is also the domain of every Z3 suite, with satisfiability, non-vacuity and mutation checks
     (DI-22, DI-23, DI-26).

The coordination is **decentralized and communication-free, driven by shared mission constraints and
local sensing**.  The drones share, before the dive:
* the mission plan, with its formation clock;
* the formation template;
* the gate map;
* the automaton rules.

At runtime they exchange no message.  Nothing in the fleet "emerges spontaneously": each drone tracks
its own slot of the shared plan and corrects it with its own sonars.

## 2. Sensing

Phase-1 probe ([docs/v2/SONAR_PROBE.md](docs/v2/SONAR_PROBE.md)) and benches (`scripts/sonar_bench.py`):

* **Range accuracy.** Within about one bin plus one octree leaf (at most 0.055 m) for BlueROV2s, boxes
  and gate bars.  The beam width is as configured: 60 deg, 62.5 deg at 3 m because of the hull's
  angular size.
* **Near field.** Strong reflectors closer than about 1 m are detected beyond the nominal cone (a gate
  post at 0.63 m and 78 deg off-axis).  The structure prediction widens the cone by atan(0.25 m / r)
  (DI-12).
* **False alarms and confirmation.** The intensity-noise tail sets the threshold: 0.30 gives about
  0.05 expected single-bin false alarms per run.  A detection must also be confirmed in 2 of 3
  captures.
* **Coverage.** 204/204 bench placements are detected (axes, edges, the 8 diagonals; two target yaws;
  1-5 m).  The observed pattern always lies within the geometric prediction.  Near-field pockets exist
  only below 0.8 m centre distance.
* **Classification bench A-F, as expected.**
  * gate only: STRUCTURE;
  * drone only: DYNAMIC;
  * drone in front of the gate: DYNAMIC + STRUCTURE;
  * two drones in one cone: two DYNAMIC;
  * seabed only: SEABED;
  * drone before the seabed onset: DYNAMIC before SEABED;
  * drone inside the seabed clutter: never declared a clear vehicle; it becomes a conservative UNKNOWN
    target from the clutter onset.
* **Performance.** About 15 ms per engine tick for 1, 3, 4 or 6 drones with six sonars each, a
  real-time factor of about 2.2 without cameras (section 6).

## 3. Formal verification ([formal/results/SUMMARY.md](formal/results/SUMMARY.md))

| suite | checks | as expected | time |
|---|---|---|---|
| Local determinism & priority hierarchy | 66 | 66 | 0.2 s |
| Observation consistency (perception -> automaton interface) | 27 | 27 | 0.6 s |
| P1 inter-vehicle separation | 13 | 13 | 91.4 s |
| P2 critical-region mutual exclusion | 19 | 19 | 82.8 s |
| P3 formation recovery (liveness, ranking functions) | 14 | 14 | 0.7 s |
| **total** | **139** | **139** | |

Key numbers:
* **Observation domain.** Every suite quantifies over the observations that satisfy the invariants of
  `holo_fleet/ha/observation_invariants.py`:
  * N1: ranges;
  * I1: `has_prio -> at_queue`;
  * I2: `at_queue -> gate_zone`;
  * I3: `passed -> !gate_zone`;
  * I4: `t_ok > 0 -> form_err < e_ok & neighbors_ok`.

  The observation-consistency suite checks:
  * O1, O2: the domain is satisfiable, and each invariant excludes observations that all the others
    admit;
  * O3: every edge of every mode is enabled on some consistent observation (non-vacuity);
  * O4: a consistent observation enables exactly one edge;
  * O5: for ANY observation, the monitored one (`sense_ok := sense_ok & consistent`) enables exactly
    one edge, and an inconsistent one only the fault edge to FAILSAFE;
  * Q1: a queued drone stays in the gate protocol.

  Mutations, each with its expected verdict:
  * Om1: without I2, a queued drone takes a formation edge (SAT);
  * Om2: the v1 domain relation `passed -> at_queue` makes `pass_done` unreachable (UNSAT): the v1
    determinism proof was vacuous for the exit edge (DI-22);
  * Om3: without the monitor, an inconsistent observation drives a mission edge (SAT).
* **P1.**
  * The smallest d_warning verified on the radial model is 1.84 m (2.4 m configured).
  * c_max = 1.2 m/s.
  * Certified single-threat escape opening >= 0.50 per unit speed (g_min); warning-filter opening
    >= 0.2 m/s for every one of the 26 sector patterns.
  * The onboard distance bound is never above the true one on 10^5 random poses.  The uncertified
    variant overestimates by up to 0.43 m and is caught.
* **P2.**
  * Never two PRIORITY drones when |delta| + 2 fuzz <= 30 deg.
  * Exactly one PRIORITY drone in abreast queues of 2-6.
  * The latch holds for every queue view; without the latch a counterexample is found.
* **P3.**
  * Ranking decreases are certified in the saturated and linear bands (along and lateral).
  * The ball |e| <= e* is invariant: e* = 0.44 m along, 0.36 m lateral.
  * The mutations are caught: no catch-up margin; k_slot = 0.
  * Current envelope (DI-27), with the plant curve and the authority shared with the runtime:
    * F5: for every head current admitted at the recovery speed, the catch-up of F1 keeps its margin.
    * Fm3: the envelope evaluated at the survey speed only admits a current that removes that margin
      (counterexample: 0.25 m/s).
    * Em1: the old scalar bound admits head currents with which not even the survey speed is
      deliverable (counterexample: 0.5 m/s).

**Assumptions** (measured on the runs in [results/v2/ASSUMPTIONS.md](results/v2/ASSUMPTIONS.md)):
* both drones of a pair detect each other;
* sonar data age <= tau_max;
* the closing speed of a pair is <= c_max;
* braking and drift bounds from the plant calibration;
* queued drones hold their point within 0.1 m;
* a committed drone crosses the CR at >= 0.12 m/s;
* for P3, perturbations end and the residual disturbance after the integrator is <= 0.05 m/s;
* the current is inside the control-feasible envelope (E1, DI-27).  The velocities the drones are
  asked for must stay deliverable against the true current within the authority, and the current must stay
  within the exercised range (0.6 m/s horizontal, 0.25 m/s vertical).  The residual drifts w_rel, w_drift
  (P1) and w_res (P3) exist only there.
  * The drones' own view (E2: ENVELOPE_VIOLATION after 4 s of undeliverable requests) agrees with E1 on
    every run except `integrated_short`, which is at the margin.
  * E1 fails on `formation_recovery_head_current` and `formation_gust` (both declared by the drones) and
    on `integrated_short` (drone 0's gate passage, up to 1.12 times the authority);
* every observation satisfies the observation invariants: 0 violations on every run.

## 4. Experiments (HoloOcean, one seed each, communication off)

| scenario | drones | P1 min distance [m] (d_safe 1.0) | P2 max CR occupancy | P3 episodes: recovery [s] (after the perturbation) | contacts | messages | control-feasible envelope / vehicles | determinism / observation violations | static rank uses | RTF (with cameras) |
|---|---|---|---|---|---|---|---|---|---|---|
| `p1_head_on` | 2 | 4.044 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 1.025 |
| `p1_vertical_escape` | 4 | 2.697 (vehicle clearance 2.35) | - | 18.1 (10.0) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.953 |
| `p1_two_lines` | 6 | 3.188 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 1.023 |
| `p1_close_encounter` | 2 | 2.931 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.916 |
| `formation_triangle` | 3 | 4.598 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.863 |
| `formation_square` | 4 | 3.450 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.937 |
| `formation_six` | 6 | 3.447 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 1.074 |
| `formation_recovery_head_current` | 4 | 3.293 | - | 8.1 (5.1) | 0 | 0 | OUTSIDE (1.52 x authority) / 1 self-declared violation | 0 / 0 | 0 | 1.015 |
| `formation_gust` | 4 | 2.710 | - | 7.6 (6.3) | 0 | 0 | OUTSIDE (2.06 x authority) / 1 self-declared violation | 0 / 0 | 0 | 1.029 |
| `gate_single` | 3 | 3.342 | 1 | 68.8 (5.0) | 0 | 0 | inside / ok | 0 / 0 | 0 | 1.068 |
| `integrated_short` | 3 | 3.320 | 1 | 69.1 (4.9) | 0 | 0 | OUTSIDE (1.12 x authority) / ok | 0 / 0 | 0 | 0.844 |

All runs are COMPLETE, no controller uses ground truth, and the automaton determinism and observation
consistency violations are 0 in every run.  P3 recovery is measured from the loss; in brackets it is
measured from the end of the last perturbation (jet, encounter, last gate passage).  In the gate
scenarios the formation is broken on purpose by the abreast queue and re-formed at the rendezvous.
"Vehicle envelope" is the drone's own view: a self-declared violation is persistent thrust saturation.

* **P1**
  * `p1_head_on`: both drones give way to the right at 8 m and pass at about 4 m without entering the
    warning band.
  * `p1_two_lines`: six drones.
    * The flanked middle drones give way UP (one line) and DOWN (the other line); the outer drones
      give way to the right.
    * Four drones enter SEPARATION_WARNING briefly while passing at 3.2-3.3 m, because the bound is
      loose for overlap patterns.
  * `p1_close_encounter`: two fleet drones on perpendicular legs reach the crossing point together,
    with the traffic rule ON and no current.
    * The rule's give-way to the right fires on the noisy closing estimate but cannot resolve a
      crossing.
    * Both drones enter SEPARATION_WARNING (23.9 / 24.0 s and 26.7 / 26.8 s) when the conservative
      distance falls below 2.4 m; its minimum is 1.97 m.
    * The true distance is 2.96 m at the first intervention, where the pair closes at 0.19 m/s.  Its
      minimum is 2.93 m.
    * The warning filter then opens the pair at up to 0.67 m/s, with directions REAR+RIGHT and
      FWD+RIGHT.  The sectors involved are FRONT+LEFT, LEFT and LEFT+REAR.
    * The bound never reaches d_ca, so COLLISION_AVOIDANCE is not exercised here and was not forced
      (DI-25).
    * Mode sequence of each drone: FOLLOW -> SW -> RECOVERY -> SW -> RECOVERY -> FOLLOW.
  * `p1_vertical_escape`: the traffic rule is switched off, so only the safety layer acts against a
    vehicle that does not react.
    * The boxed-in centre drone enters SEPARATION_WARNING twice and COLLISION_AVOIDANCE once, and
      escapes with UP components.
    * The rear drone does the same when the vehicle reaches it.
    * The minimum distance between drones is 2.70 m, and the clearance to the vehicle 2.35 m.
* **P2** (`gate_single`, `integrated_short`)
  * Abreast queue; entry order left, centre, right; one drone at a time in the CR.
  * Static rank never used, no occupancy timeout.
  * The formation re-forms at the rendezvous beyond the gate, 4.9-5.0 s after the last passage.
  * The SEPARATION_WARNING episodes during the gate phase come from the deliberately loose sound
    distance bound (DI-13). They are safe and short.
* **P3**
  * The triangle, the square and the six-drone line never lose the formation under in-envelope
    currents (lateral, diagonal, with a vertical component).
  * `formation_recovery_head_current`, the main P3 validation: a 0.6 m/s jet against the motion (the
    old scalar bound) on the rear-left drone of the square (drift measured at the drone: 0.59 m/s).
    It is outside the control-feasible envelope: the head limit at 0.30 m/s is about 0.41 m/s (DI-27).
    * At nominal authority the drone makes about 0.71 m/s through the water, so it cannot follow its
      slot at 0.30 m/s.  Its thrust saturates (5.6 s in total).
    * At 19.3 s its own monitor declares ENVELOPE_VIOLATION, and it holds in FAILSAFE until 23.3 s.
      The true formation error is 0.8 m at that moment.
    * The formation is lost at 21.0 s (true error peaking at 1.85 m) and recovered at 29.1 s: 8.1 s
      after the loss, 5.1 s after the end of the jet.
    * Hit drone: FOLLOW -> FAILSAFE -> RECOVERY -> FOLLOW (FOLLOW again at 33.3 s, onboard).  The two
      front drones: FOLLOW -> RECOVERY -> FOLLOW.
    * P1 minimum 3.29 m, navigation error <= 0.14 m, sonar data age 0, no FAILSAFE after the jet.
    * A loss with every assumption valid could not be produced (DI-24).
  * `formation_gust`, the stress test outside the envelope: a 0.85 m/s jet (0.94 m/s with the
    background current) breaks the square.
    * The jet drone declares ENVELOPE_VIOLATION and holds in FAILSAFE.
    * The formation is recovered 7.6 s after the loss.

Figures: [figures/v2/](figures/v2/) (sensor, p1, p2, p3).  GIFs: [figures/v2/gifs/](figures/v2/gifs/).

## 5. Design iterations

All 27 iterations, with problem, cause, fix, why the fix is principled and result, are in
[DESIGN_ITERATIONS.md](DESIGN_ITERATIONS.md).  The ones that changed the design:

| iteration | what changed |
|---|---|
| DI-1..7 | the octree patch (invisible props, ghosts, reset crash, cell-boundary faces, cache hygiene) |
| DI-9 | agents invisible to every sonar because of init order (fourth patch commit) |
| DI-10 | head-on standoff → sector traffic rule |
| DI-11 | threshold set from the noise tail |
| DI-12 | near-field widening of the structure window |
| DI-13 | unsound distance bound replaced by a certified one, with a numeric check and a mutation |
| DI-14 | occupancy corridor = exact range interval of the CR; far-side structure tolerance |
| DI-15 | M-of-N confirmation for every unexplained echo |
| DI-16 | BACKOFF removed: WAIT for front or left; merged targets get the answer that is safe for both |
| DI-17 | queue line from the cone and the merge clearance; rendezvous beyond the gate |
| DI-18 | formation recovery speed and neighbour association |
| DI-19 | escape demonstrated without the traffic rule; hysteresis |
| DI-20, DI-21 | neighbour association per sector with memory; structure-masked neighbours not required |
| DI-22 | the formal observation domain carried a v1 relation (`passed -> at_queue`): replaced by the shared invariants, non-vacuity checked |
| DI-23 | for n >= 5 the outer queue points lay outside the approach zone: lateral bound contains the own queue point |
| DI-24 | no current inside every assumption breaks the formation; against the motion the claimed 0.6 m/s is not met (about 0.4 m/s at survey speed) |
| DI-25 | a right-angle crossing is outside the head-on traffic rule: the warning filter acts, collision avoidance is not reached |
| DI-26 | observation consistency check between perception and automaton (runtime + Z3) |
| DI-27 | the scalar current bound replaced by a control-feasible, direction-aware envelope (authority, calibrated plant curve, requested speed): one definition for the onboard monitor, the run evaluation, the tests and Z3 |

Two late fixes came from checking the demo figures and the onboard summaries, not only the referee:
* DI-20: the square's merged neighbour echoes and intermittent far returns kept the drones in
  FORMATION_RECOVERY although the formation was perfect.  Association is now per (target, sector),
  with 1 s of memory, and only neighbours within 7.5 m are required.
* DI-21: a neighbour whose echo falls inside the gate's structure window is unobservable, not
  missing.

## 6. Limits

* **Scope of the proofs.** The proofs are on abstractions: a pairwise radial model with the triangle
  lemma, tables over all sector patterns, a timed model of the latch, and a per-axis formation law.
  They are not proofs over HoloOcean's physics.  The link between the two is the shared guard code and
  the measured assumptions.
* **Number of drones.** N is not arbitrary: P2 is checked for queues of 2..6 and demonstrated with 3.
  Six-drone formations and encounters are demonstrated; no gate is demonstrated with six drones (the
  queue line would be 9.5 m back).
* **Distance bound.** The sound bound is loose for overlap patterns (median 0.8 m), so warnings start
  earlier than strictly needed.
* **Mapped structure masks drones.** A drone in the gate opening is invisible to the others.  The
  occupancy latch covers this; two drones masked at the same time are outside the envelope.
* **Seabed clutter.** Near the seabed a drone in the clutter cannot be told apart from the bottom; the
  reaction is conservative (UNKNOWN).
* **Long range.** Weak returns at 10-12 m are intermittent.  Formation checks only require neighbours
  within 7.5 m.
* **Current envelope (DI-27).** The v2 baseline first declared |current| <= 0.6 m/s in every direction.
  That was too general: what a drone can do depends on the direction of the current relative to the motion
  it is asked for, and on the speed of that motion.
  * The envelope is now control-feasible.  The steady command for the through-water velocity, on the
    body axes and with the calibrated plant curve, must stay within the authority (0.40 nominal); the
    current must also stay within the exercised range (0.6 m/s horizontal, 0.25 m/s vertical).
  * At the 0.30 m/s survey speed: head <= 0.41 m/s, lateral <= 0.60 m/s.  At 0.50 m/s (recovery
    catch-up, gate passage): head <= 0.21 m/s, lateral <= 0.57 m/s.
  * The plant curve is accurate to 6 % on its calibration points.
  * The onboard monitor sees an undeliverable request only through persistent saturation or its own
    steady command.  A request beyond the authority by a few per cent can show up as a lag instead:
    `integrated_short`, drone 0's diagonal gate passage at 0.5 m/s against the 0.30 m/s cross-current,
    is outside the envelope for about 5 s without being declared onboard.  Flagging such lags would put
    a committed drone in FAILSAFE during a gate passage; that was not done.
  * P3's catch-up needs the envelope evaluated at the recovery speed (F5).  Between about 0.21 and
    0.41 m/s of head current the slot can be held, but the recovery margin of the proof is not
    guaranteed (Fm3).
* **No formation loss inside every assumption.** With an unrejected drift <= w_drift_max = 0.15 m/s
  the slot loop keeps the error far below e_lost.  Every P3 loss in the demos comes from a
  perturbation beyond what the drone can reject:
  * a head current above its authority;
  * the 0.85 m/s gust;
  * the safety layer's own manoeuvres;
  * the deliberate gate queue.

  The P3 property is proved for what happens after such a perturbation ends (assumption A1).  It does
  not say that in-envelope currents cause losses (DI-24); inside the control-feasible envelope none of
  the runs loses its formation.
* **Collision avoidance in fleet-only encounters.** In the right-angle crossing the warning filter
  stops the closing before the conservative bound reaches d_ca.  COLLISION_AVOIDANCE is demonstrated
  only against the non-cooperative vehicle of `p1_vertical_escape` (DI-25).
* **Evidence.** One seed per scenario, by design: no statistics.
* **Speed.** With the visualisation cameras the demos run at 0.84-1.07x real time on the final runs, and at about 0.6x on a busier machine.

## 7. Reproducing

```bash
python formal/check_properties.py
python -m pytest -q
python scripts/run_all_demos.py && python scripts/make_figures_v2.py && python scripts/validate_assumptions_v2.py
python scripts/run_demo.py --scenario gate_single        # watch one
```

## 8. V2 BASELINE COMPLETE

The v2 baseline is frozen with the following evidence (one seed each; README section "V2 BASELINE
COMPLETE" for the numbers):

| aspect | scenario |
|---|---|
| SENSING | `sonar_classification` |
| P1, preventive (traffic rule) | `p1_two_lines` |
| P1, safety layer | `p1_close_encounter` (fleet drones only, traffic rule on), `p1_vertical_escape` (non-cooperative vehicle, rule off) |
| P2 | `gate_single` |
| P3 | `formation_recovery_head_current` (main validation), `formation_gust` (stress test outside the envelope) |
| INTEGRATED | `integrated_short` |

**P3 evidence.**
* `formation_gust` is a stress test outside the envelope.
* `formation_recovery_head_current` is the main experimental validation of P3: recovery after a
  perturbation that ends, which is what the property claims.
  * It is outside the control-feasible envelope (a 0.6 m/s head current against an about 0.41 m/s
    limit), and the hit drone declares it.
  * The scenario is the one that revealed that the scalar bound was too general, corrected in DI-27.
  * It was requested as "formation recovery inside the envelope".  It is named after what it shows,
    because no formation loss inside every assumption was achievable (DI-24).

**Formal verification.** 139 checks with the expected verdict:
* determinism 66;
* observation consistency 27;
* P1 13;
* P2 19;
* P3 14 (including the current envelope: F5, Fm3, Em1).
