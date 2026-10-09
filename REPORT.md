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
6. **Demos with a fleet view** (v2.1, DI-30): the whole fleet at a glance - slots, states, missing drones, the
   gate queue, P1 / P2 / P3 - with readable events and a timeline of the automaton states.
7. **Observation consistency.**
   * The relations that the perception guarantees between the variables of the abstract observation
     are written once (`holo_fleet/ha/observation_invariants.py`).
   * The same module is the runtime check between perception and automaton: a violation is logged and
     the automaton takes the existing fault edge to FAILSAFE.
   * It is also the domain of every Z3 suite, with satisfiability, non-vacuity and mutation checks
     (DI-22, DI-23, DI-26).
8. **Lost drones without communication** (v2.1, DI-29): a missing neighbour is waited for on the own slot
   (FORMATION_WAIT_REJOIN), declared vacant after t_rejoin = 37.5 s (DEGRADED_FORMATION, original slots kept,
   no reconfiguration), re-included if it comes back; a lost drone rejoins from behind along its own lane;
   a drone whose thrusters fail detects it and stays in FAILSAFE until it moves again as asked.
9. **The gate as a mutual-exclusion zone** (v2.1, DI-28): MUTEX_* modes, precedence left first, then top
   first, static rank only for an ambiguous pattern; a latent deadlock of stacked queues removed; queue
   progress checked for every occupancy with at most one vacant queue point between neighbours.

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
| Local determinism & priority hierarchy (10 modes) | 82 | 82 | 0.3 s |
| Observation consistency (perception -> automaton interface) | 31 | 31 | 0.8 s |
| P1 inter-vehicle separation | 13 | 13 | 86.8 s |
| P2 critical-region mutual exclusion and queue progress | 27 | 27 | 91.1 s |
| P3 formation recovery (liveness, ranking functions) and bounded waiting | 22 | 22 | 3.1 s |
| **total** | **175** | **175** | |

Key numbers:
* **Observation domain.** Every suite quantifies over the observations that satisfy the invariants of
  `holo_fleet/ha/observation_invariants.py`:
  * N1: ranges;
  * I1: `has_prio -> at_queue`;
  * I2: `at_queue -> mutex_zone`;
  * I3: `passed -> !mutex_zone`;
  * I4: `t_ok > 0 -> form_err < e_ok` (the own error only: a missing neighbour is not the drone's error).

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
  * Progress / no deadlock: exactly one PRIORITY drone, the leftmost occupied, for every occupancy of abreast
    queues of 2-6 with at most one vacant point between neighbours (96 occupancies).  M2s: two adjacent
    vacant points give two PRIORITY drones (outside the scope).
  * Stacked queues (M2v, numeric over the tolerance box): one relation per pair, exactly one leader for every
    occupancy (left first, then top first), the static rank never needed.  M2m: the v2 rule deadlocks
    (no leader in 5 of 15 occupancies of the 2 x 2 stack); M2vm: columns 3.5 m apart make a diagonal pair
    ambiguous.
  * M4f: the queue neighbours a decision needs are never hidden in the gate's structure window; M4fm: the
    stacked queue at the cone-only line hides them.  M4s: stacked-queue geometry and pass-path clearance.
  * The latch holds for every queue view (n = 2, 3, 4); without the latch a counterexample is found.
* **P3.**
  * Ranking decreases are certified in the saturated and linear bands (along and lateral).
  * The ball |e| <= e* is invariant: e* = 0.44 m along, 0.36 m lateral.
  * The mutations are caught: no catch-up margin; k_slot = 0.
  * F4: the automaton edges between FORMATION_RECOVERY and the three formation-keeping modes.
  * F6 (BMC over 376 steps) and F6r (ranking): a neighbour that never reappears is declared vacant within
    t_rejoin of running time and the drone leaves FORMATION_WAIT_REJOIN.  Fm4: without the timeout it may
    wait for ever.
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
| `p1_head_on` | 2 | 4.051 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.851 |
| `p1_vertical_escape` | 4 | 2.501 (vehicle clearance 2.23) | - | open at the end (lost at 16.9 s, final error 0.52 m) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.855 |
| `p1_two_lines` | 6 | 3.209 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.823 |
| `p1_close_encounter` | 2 | 2.683 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.884 |
| `formation_triangle` | 3 | 4.576 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.899 |
| `formation_square` | 4 | 3.454 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.862 |
| `formation_six` | 6 | 3.463 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.923 |
| `formation_recovery_head_current` | 4 | 3.343 | - | 11.5 (7.7) | 0 | 0 | OUTSIDE (1.52 x authority) / 1 self-declared violation | 0 / 0 | 0 | 0.868 |
| `formation_gust` | 4 | 2.698 | - | 15.3 (10.5) | 0 | 0 | OUTSIDE (2.08 x authority) / 1 self-declared violation | 0 / 0 | 0 | 0.87 |
| `gate_single` | 3 | 3.293 | 1 | 68.7 (5.2) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.907 |
| `integrated_short` | 3 | 3.335 | 1 | 70.6 (5.6) | 0 | 0 | OUTSIDE (1.12 x authority) / ok | 0 / 0 | 0 | 0.876 |
| `lost_drone_rejoin` | 4 | 3.268 | - | 27.3 (18.1) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.861 |
| `lost_drone_timeout` | 4 | 3.448 | - | open at the end (lost at 14.7 s, final error 12.30 m); P3-deg: re-formed without drone_2 (absent at 50.8 s, final error 0.06 m) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.861 |
| `mutex_deadlock_resolution` | 3 | 3.424 | 1 | 58.6 (3.7) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.779 |
| `line_parallel_mutex` | 3 | 2.943 | 1 | 63.8 (1.8) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.886 |
| `lost_drone_mutex` | 3 | 2.897 | 1 | open at the end (lost at 8.1 s, final error 13.36 m) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.905 |

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

All 30 iterations, with problem, cause, fix, why the fix is principled and result, are in
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
| DI-28 | gate protocol renamed MUTEX; precedence left first, then top first, static rank only for ambiguous patterns; latent deadlock of stacked queues removed; stacked queue geometry (columns 5 m apart, no neighbour hidden by the gate frame) |
| DI-29 | lost drones: FORMATION_WAIT_REJOIN, slot vacant after t_rejoin, DEGRADED_FORMATION on the original slots; confirmed re-inclusion; rejoin from behind; FAILSAFE probe so that a dead drone stays in FAILSAFE; give-way suspended in FAILSAFE; indistinguishable neighbours confirmed as a group |
| DI-30 | fleet-centred demo view instead of the onboard-vs-referee dashboard |

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
* **Evidence.** One seed per scenario, by design: no statistics.  HoloOcean is not bit-reproducible (sensor
  noise), so one run can take a different branch: `p1_vertical_escape` escaped backwards in v2.1 (upwards in
  the baseline) and its formation was still converging at the end of the 45 s run.
* **Lost drones (v2.1).** Without communication the others cannot slow down consistently, so they do not:
  the lost drone catches up with the 0.2 m/s recovery margin, or its slot is declared vacant after
  t_rejoin = 37.5 s.  A drone of a stacked pair seen from the side cannot be told apart from its partner
  (one echo, same range): the pair is observable only as a group, so the loss of one of them is noticed
  only by the drones that see them separately.  A drone that fails inside a queued drone's bracket, on its
  left, blocks it (the queued drone waits for a drone that never moves).  The degraded formation is not
  reconfigured: the hole stays.
* **Mutual-exclusion zone (v2.1).** Progress is checked for occupancies with at most one vacant queue point
  between neighbours; two adjacent vacant points (M2s) and the pair around one vacant point of a 3-drone
  queue at G06 (hidden by the gate frame, M4f) are outside the scope; moving that queue line back would put
  the exits beyond the range at which the outer queued drone detects a hull (about 8.5 m).  A passing drone
  near the gate frame may not see a queued neighbour within 5 m (1.2 s in `mutex_deadlock_resolution`):
  the separation then rests on the pass-path geometry.
* **Speed.** With the visualisation cameras the demos run at 0.84-1.07x real time on the final runs, and at about 0.6x on a busier machine.

## 7. Reproducing

```bash
python formal/check_properties.py
python -m pytest -q
python scripts/run_all_demos.py && python scripts/make_figures_v2.py && python scripts/validate_assumptions_v2.py
python scripts/run_demo.py --scenario lost_drone_rejoin  # watch one (fleet view)
python scripts/render_demo.py results/v2/demos/lost_drone_timeout    # fleet-view GIF again, from the logs
```

## 8. V2 BASELINE COMPLETE

The v2 baseline is frozen with the following evidence (one seed each; README section "V2 BASELINE
COMPLETE" for the numbers of the baseline runs at `327f64a`; the runs were regenerated once in v2.1, current
numbers in section 4):

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

## 9. Lost drones, the mutual-exclusion zone and the fleet view (v2.1)

No communication was added: no inter-drone message, no acoustic ping, no heartbeat.  A drone uses its own
sensors and navigation and the mission plan shared before the dive.

### 9.1 Automaton

Ten modes (two new), one priority order:
`FAILSAFE_HOLD_OR_RETREAT > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX_PASS > MUTEX_YIELD >
MUTEX_APPROACH > FORMATION_RECOVERY > {FORMATION_WAIT_REJOIN, DEGRADED_FORMATION, FORMATION_FOLLOW}`.
The three formation-keeping modes share the follow flow and differ only in what the drone knows about its
neighbours.  New abstract variable `degraded` (some expected neighbour's slot declared vacant);
`neighbors_ok` now ignores vacant slots; `t_ok` counts the own error only (invariant I4 updated).

| from a formation-keeping mode, calm, no gate | edge | to |
|---|---|---|
| own slot error > e_lost | formation_lost | FORMATION_RECOVERY |
| own slot ok, an expected neighbour missing | neighbour_missing | FORMATION_WAIT_REJOIN |
| own slot ok, no neighbour missing, a slot vacant | slot_vacant | DEGRADED_FORMATION |
| own slot ok, nobody missing, nothing vacant | follow | FORMATION_FOLLOW |
| from any other mode, calm, no gate: own slot recovered for t_ok_hold | formation_recovered / recovered_wait_rejoin / recovered_degraded | the formation-keeping mode selected as above |

### 9.2 Temporary loss vs permanent failure

* **Missing.**  A neighbour expected within the sensing range (7.5 m), observable (healthy sectors, not
  hidden by a mapped structure) and not associated for 1 s.  It is **back** only when confirmed near its
  slot: three associations within 1 s, each within half the range gate (one spurious echo of another
  neighbour does not count).
* **Waiting.**  FORMATION_WAIT_REJOIN: the drone keeps its slot on the shared clock and does not slow down
  (a slowdown without communication could not be applied by the whole fleet consistently).
* **Timeout.**  t_rejoin = 7.5 m / (0.5 - 0.3) m/s = 37.5 s of formation time: the time a lost drone still
  at the edge of the sensing range needs to catch up.  The timer runs only while the drone itself keeps its
  slot and the shared clock moves.  Then the slot is declared vacant (a latch, per drone, no agreement
  needed): DEGRADED_FORMATION, the remaining drones on their **original** slots, the hole left open (no
  reconfiguration).  The slot is re-included when its drone is confirmed there for 1 s.
* **Rejoin.**  The lost drone recovers its own slot at the recovery speed (0.2 m/s faster than the clock),
  from behind along its own lane when it is off its lane (DROP_BACK, TO_LANE).
* **Self-diagnosis of a failed drone.**  Thrusters that do not respond -> persistent saturation ->
  ENVELOPE_VIOLATION -> FAILSAFE.  In FAILSAFE the drone performs a heave probe; it clears the violation
  only when its DVL shows the probe delivered (projection gain >= 0.5 over 4 s).  A dead drone therefore
  stays in FAILSAFE; a repaired one resumes.

### 9.3 Mutual-exclusion zone

* Precedence from sector patterns: LEFT first; then TOP first (only above: wait; only below: go);
  static rank (the queue order of the plan) only for an ambiguous UP+DOWN pattern.
* The v2 rule deadlocked any vertically stacked queue (both stacked drones waited for an unknown rank);
  found by the formal mutation M2m.
* Stacked templates queue in columns 5 m apart (M2v: the diagonal pair is always horizontal), at a queue
  line where the gate frame hides no queue neighbour (M4f, found in a probe run), with a 3-D pass path that
  descends only on the diagonal (M4s).
* Progress: for every occupancy with at most one vacant queue point between neighbours exactly one queued
  drone has priority (M2: abreast queues of 2..6, 96 occupancies; M2v: stacked queues).  Out of scope:
  two adjacent vacant queue points (M2s: both remaining drones claim priority; the occupancy latch is
  then the only protection), the pair around one vacant point of a 3-drone queue at G06 (hidden by the gate
  frame, M4f), and a drone that stops for good inside a queued drone's bracket on its left.

### 9.4 What is guaranteed (prudently)

* **P1** is a property of the model: under its assumptions (pairwise radial model with the one-sided onboard
  distance bound, sensing within tau_max, closing speed <= c_max, residual drifts inside the
  control-feasible envelope) a pair never comes closer than d_safe.  It is not a claim that real vehicles
  cannot collide.  The experiments report what was observed: zero contacts, the minimum distances, the
  warnings and the avoidance manoeuvres.
* **P3 (nominal)** `G(formation_lost -> F formation_recovered)`, no time bound, under fairness assumptions
  A1-A5 including "every lost drone can move again".
* **P3-deg** for a drone that never comes back: every neighbour that expected it leaves
  FORMATION_WAIT_REJOIN within t_rejoin of formation time (F6, ranking function on the timer), and the
  remaining drones recover their original slots after every perturbation (F1-F3 apply to each of them
  unchanged).  The formation is then degraded but stable; nothing closes the hole.

### 9.5 Experiments

| scenario | sim time | what happens | result |
|---|---|---|---|
| `lost_drone_rejoin` | 60 s | the rear-left drone's thrusters fail from 8.1 to 15.1 s (simulator fault, no drone told); the 0.35 m/s lateral current drags it 2.2 m off its lane | it detects the fault itself (ENVELOPE_VIOLATION at 13.3 s, FAILSAFE until its heave probe is delivered at 21.5 s), returns to its lane behind the fleet (TO_LANE) and advances along it, FOLLOW at 44.4 s; the three others wait on their slots (FORMATION_WAIT_REJOIN from 12.7-13.3 s to 33.9-40.8 s); no slot declared vacant; P3 recovered in 27.3 s; min distance 3.27 m |
| `lost_drone_timeout` | 64 s | the same drone's thrusters fail for good at 8.1 s | its neighbours miss it from 14.1 / 18.7 / 21.1 s and, each on its own after 37.5 s, declare its slot vacant (51.5 / 56.1 / 58.5 s): DEGRADED_FORMATION on their original slots, the hole left open, formation error of the three 0.06 m; the failed drone stays in FAILSAFE; referee: absent at 50.8 s, P3-deg PASS (the nominal P3 is not claimed) |
| `mutex_deadlock_resolution` | 75 s | an upper / lower pair on the left lane and a drone on the right queue at G06 | entry order left-top, left-bottom, right; occupancy <= 1; static rank never used; every drone back in FOLLOW beyond the gate; min distance 3.42 m |
| `line_parallel_mutex` | 75 s | three drones abreast reach the gate side by side (parity) | left to right, occupancy <= 1, static rank never used; min distance 2.94 m (short SEPARATION_WARNINGs while the first drone merges in front of its neighbour) |
| `lost_drone_mutex` | 72 s | the left drone's thrusters fail for good at 1.1 s, about 9 m before the queue line | the leftmost drone still present goes first (entry order drone 1, drone 2); occupancy <= 1; the two wait for the missing drone beyond the gate (timer frozen during the rendezvous hold); min distance 2.90 m |

### 9.6 Fleet view

The demo UI is now fleet-centred (`holo_fleet/ui/fleet_view.py`, DI-30): top view with the common path,
the planned slots, the drones coloured by state with trails, the gate with its critical region and numbered
queue points (and a side view for depth layers); P1 / P2 / P3, queue progress and the counts of active,
missing and assumed-failed drones; one row per drone; readable events; a timeline of the automaton states.
