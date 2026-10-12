# REPORT v2 - leaderless, communication-free UUV fleet with six realistic sonars and verifiable local hybrid automata

Branch `realistic-sonar-formations-v2`. Design history is recorded in [DESIGN_ITERATIONS.md](DESIGN_ITERATIONS.md).

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
6. **Demos with a fleet view** (DI-30): the whole fleet at a glance - slots, states, missing drones, the
   gate queue, P1 / P2 / P3 - with readable events and a timeline of the automaton states.
7. **Observation consistency.**
   * The relations that the perception guarantees between the variables of the abstract observation
     are written once (`holo_fleet/ha/observation_invariants.py`).
   * The same module is the runtime check between perception and automaton: a violation is logged and
     the automaton takes the existing fault edge to FAILSAFE.
   * It is also the domain of every Z3 suite, with satisfiability, non-vacuity and mutation checks
     (DI-22, DI-23, DI-26).
8. **Lost drones without communication** (DI-29, DI-31): the remaining drones enter a
   reduced-progress FORMATION_WAIT_REJOIN mode to facilitate catch-up while preserving the formation (each
   drone slows its own formation reference from its own perception, no message); the slot is declared
   vacant after t_rejoin = 30 s, derived from the catch-up of the recovery speed (DEGRADED_FORMATION,
   original slots kept, no reconfiguration, survey speed resumed), re-included if the drone comes back; a
   lost drone rejoins from behind along its own lane; a drone whose thrusters fail detects it and stays in
   FAILSAFE until it moves again as asked.
9. **The gate as a mutual-exclusion zone** (DI-28): MUTEX_* modes, precedence left first, then top
   first, static rank only for an ambiguous pattern; a latent deadlock of stacked queues removed; queue
   progress checked for every occupancy with at most one vacant queue point between neighbours.
10. **The automaton made visible** (DI-31): every edge carries a readable guard proved equal to the
   executed one under the priority order (Z3) and a cause written to the transition log at every mode change;
   the automaton diagram is generated from the spec (`figures/v2/automaton/`); the fleet view shows the
   state, the last transition and its cause for one drone.

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
| Local determinism & priority hierarchy (10 modes), readable guards (L1, ML) | 93 | 93 | 0.6 s |
| Observation consistency (perception -> automaton interface) | 31 | 31 | 1.0 s |
| P1 inter-vehicle separation | 13 | 13 | 95.5 s |
| P2 critical-region mutual exclusion and queue progress | 27 | 27 | 104.8 s |
| P3 formation recovery (liveness, ranking functions) and bounded waiting | 22 | 22 | 3.0 s |
| **total** | **186** | **186** | |

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
  * F6 (BMC over 300 steps: t_rejoin = 30 s) and F6r (ranking): a neighbour that never reappears is declared
    vacant within t_rejoin of running time and the drone leaves FORMATION_WAIT_REJOIN.  Fm4: without the
    timeout it may wait for ever.
  * The reference speed is a free variable in [v_wait, v_nominal] at every step (reduced progress, DI-31):
    the binding case of F1 is v_nominal, a slower reference only adds catch-up.
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
    every current run, including `integrated_short` with its 0.27 m/s gust.
  * E1 fails on `formation_recovery_head_current` and `formation_gust`, both declared by the drones;
* every observation satisfies the observation invariants: 0 violations on every run.

## 4. Experiments (HoloOcean, one seed each, communication off)

| scenario | drones | P1 min distance [m] (d_safe 1.0) | P2 max CR occupancy | P3 episodes: recovery [s] (after the perturbation) | contacts | messages | control-feasible envelope / vehicles | determinism / observation violations | static rank uses | RTF (with cameras) |
|---|---|---|---|---|---|---|---|---|---|---|
| `p1_head_on` | 2 | 4.051 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.851 |
| `p1_vertical_escape` | 4 | 2.722 (vehicle clearance 2.11) | - | 18.0 (10.7) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.559 |
| `p1_two_lines` | 6 | 3.114 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.641 |
| `p1_close_encounter` | 2 | 2.683 | - | n/a | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.884 |
| `formation_triangle` | 3 | 4.576 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.899 |
| `formation_square` | 4 | 3.454 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.862 |
| `formation_six` | 6 | 3.463 | - | never lost | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.923 |
| `formation_recovery_head_current` | 4 | 3.370 | - | 6.9 (4.7) | 0 | 0 | OUTSIDE (1.52 x authority) / 1 self-declared violation | 0 / 0 | 0 | 0.769 |
| `formation_gust` | 4 | 2.725 | - | 10.0 (5.4) | 0 | 0 | OUTSIDE (2.07 x authority) / 1 self-declared violation | 0 / 0 | 0 | 0.716 |
| `gate_single` | 3 | 3.313 | 1 | 68.3 (5.1) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.428 |
| `integrated_short` | 3 | 3.336 | 1 | 70.2 (5.2) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.563 |
| `lost_drone_rejoin` | 4 | 3.364 | - | 20.7 (11.5) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.685 |
| `lost_drone_timeout` | 4 | 3.342 | - | open at the end (lost at 14.6 s, final error 13.67 m); P3-deg: re-formed without drone_2 (absent at 43.2 s, final error 0.10 m) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.655 |
| `mutex_deadlock_resolution` | 3 | 3.424 | 1 | 58.6 (3.7) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.779 |
| `line_parallel_mutex` | 3 | 2.943 | 1 | 63.8 (1.8) | 0 | 0 | inside / ok | 0 / 0 | 0 | 0.886 |
| `lost_drone_mutex` | 3 | 3.014 | 1 | open at the end (lost at 8.3 s, final error 13.41 m) | 0 | 0 | inside / 1 declared by the faulty drone | 0 / 0 | 0 | 0.631 |

All current runs are COMPLETE, no controller uses ground truth, and the automaton determinism and observation
consistency violations are 0 in every run. P3 recovery is measured from the loss; in
brackets it is measured from the end of the last perturbation (jet, encounter, last gate passage).  In the gate
scenarios the formation is broken on purpose by the abreast queue and re-formed at the rendezvous.
"Vehicle envelope" is the drone's own view: a self-declared violation is persistent thrust saturation.

* **P1**
  * `p1_head_on`: both drones give way to the right at 8 m and pass at about 4 m without entering the
    warning band.
  * `p1_two_lines`: six drones.
    * The flanked middle drones give way UP (one line) and DOWN (the other line); the outer drones
      give way to the right.
    * Five drones enter SEPARATION_WARNING for 1.4-6.0 s while the lines pass (true minimum 3.11 m),
      because the bound is loose for overlap patterns.
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
    * The boxed-in centre drone enters SEPARATION_WARNING twice and COLLISION_AVOIDANCE once (4.3 s),
      and escapes in 3-D, with UP and DOWN components.
    * The rear drone enters SEPARATION_WARNING three times when the vehicle reaches it, and escapes
      backwards, with UP and DOWN components.
    * The minimum distance between drones is 2.72 m, and the clearance to the vehicle 2.11 m.  The
      formation, lost at 17.5 s, is recovered at 35.5 s, 10.7 s after the end of the encounter.
* **P2** (`gate_single`, `integrated_short`)
  * Abreast queue; entry order left, centre, right; one drone at a time in the CR.
  * Static rank never used, no occupancy timeout.
  * The formation re-forms at the rendezvous beyond the gate, 5.1-5.2 s after the last passage.
  * `integrated_short` is inside the control-feasible envelope with the current 0.27 m/s gust (section 8.4).
  * The SEPARATION_WARNING episodes during the gate phase come from the deliberately loose sound
    distance bound (DI-13). They are safe and short.
* **P3**
  * The triangle, the square and the six-drone line never lose the formation under in-envelope
    currents (lateral, diagonal, with a vertical component).
  * `formation_recovery_head_current`, the main P3 validation: a 0.6 m/s jet against the motion (the
    old scalar bound) on the rear-left drone of the square (drift measured at the drone: 0.59 m/s).
    It is outside the control-feasible envelope: the head limit at 0.30 m/s is about 0.41 m/s (DI-27).
    * At nominal authority the drone makes about 0.71 m/s through the water, so it cannot follow its
      slot at 0.30 m/s.  Its thrust saturates (5.8 s in total).
    * At 20.5 s its own monitor declares ENVELOPE_VIOLATION, and it holds in FAILSAFE until 24.5 s.
      The true formation error is 0.8 m at that moment.
    * The formation is lost at 22.4 s (true error peaking at 1.69 m) and recovered at 29.3 s: 6.9 s
      after the loss, 5.3 s after the end of the jet.
    * Hit drone: FOLLOW -> WAIT_REJOIN (0.3 s) -> FAILSAFE -> RECOVERY -> FOLLOW (FOLLOW again at 31.6 s,
      onboard).  The two front drones miss it: FOLLOW -> WAIT_REJOIN -> FOLLOW, with reduced progress;
      the rear-right drone stays in FOLLOW.
    * P1 minimum 3.37 m, navigation error <= 0.15 m, sonar data age 0, no FAILSAFE after the jet.
    * A loss with every assumption valid could not be produced (DI-24).
  * `formation_gust`, the stress test outside the envelope: a 0.85 m/s jet (0.94 m/s with the
    background current) breaks the square.
    * The jet drone declares ENVELOPE_VIOLATION at 19.0 s and holds in FAILSAFE until 28.3 s.
    * The formation is lost at 25.0 s and recovered at 35.0 s: 10.0 s after the loss, 5.4 s after the
      end of the perturbation.

Figures: [figures/v2/](figures/v2/) (scenario views, sensor, p1, p2, p3 and automaton).

## 5. Design iterations

All 31 iterations, with problem, cause, fix, why the fix is principled and result, are in
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
| DI-31 | reduced-progress WAIT through a local formation time (no communication): the lost drone slows its own reference, a drone follows the drone ahead in its lane and the slowdown ends front to back, progress consensus only near the own slot, no new give-way while catching up; t_rejoin derived from the catch-up (30 s); readable guards proved equal to the spec, transition log with causes, automaton diagram generated from the spec, automaton card in the fleet view; `integrated_short` gust 0.27 m/s (inside the envelope) |

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
* **Current envelope (DI-27).** What a drone can do depends on the direction of the current relative to the motion
  it is asked for, and on the speed of that motion.
  * The envelope is now control-feasible.  The steady command for the through-water velocity, on the
    body axes and with the calibrated plant curve, must stay within the authority (0.40 nominal); the
    current must also stay within the exercised range (0.6 m/s horizontal, 0.25 m/s vertical).
  * At the 0.30 m/s survey speed: head <= 0.41 m/s, lateral <= 0.60 m/s.  At 0.50 m/s (recovery
    catch-up, gate passage): head <= 0.21 m/s, lateral <= 0.57 m/s.
  * The plant curve is accurate to 6 % on its calibration points.
  * The onboard monitor sees an undeliverable request only through persistent saturation or its own
    steady command. The current `integrated_short` run remains inside the envelope through the gate passage.
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
* **Evidence.** One seed per scenario, by design: no statistics. HoloOcean is not bit-reproducible (sensor
  noise), so repeated runs can take a different branch. The listed recovery times are single runs.
* **Lost drones.** The reduced progress is applied by each drone from its own perception:
  * drones notice an absence at different times, or not at all (range-only sensing: an echo of another
    neighbour can stand in for the missing drone for tens of seconds), so their formation references differ
    by a few tenths of a metre for a while; the same-lane following prevents compressions, other pairs only
    stretch and are re-aligned by the progress consensus, which is slow (gain 0.08 /s) and capped (2 m);
  * the same-lane following is limited to template slots on the same lane; a template without such pairs
    relies on the consensus alone;
  * the formation stays behind the plan by the accumulated delay (up to t_rejoin x (1 - v_wait / v_nominal)
    = 10 s per lost drone): the schedule is not recovered;
  * a late detector times the absence from its own detection, so the neighbours of one lost drone may
    declare its slot vacant tens of seconds apart (each resumes the survey speed on its own; the drone ahead
    in a lane sets the pace of the one behind);
  * the timeout bound d_rejoin is configured (the sensing range), not estimated: a drone lost farther away
    is declared vacant even if it could still come back (it is then re-included when it does);
  * a drone of a stacked pair seen from the side cannot be told apart from its partner (one echo, same
    range): the pair is observable only as a group, so the loss of one of them is noticed only by the
    drones that see them separately;
  * a drone that fails inside a queued drone's bracket, on its left, blocks it (the queued drone waits for
    a drone that never moves);
  * the degraded formation is not reconfigured: the hole stays.
* **Mutual-exclusion zone.** Progress is checked for occupancies with at most one vacant queue point
  between neighbours; two adjacent vacant points (M2s) and the pair around one vacant point of a 3-drone
  queue at G06 (hidden by the gate frame, M4f) are outside the scope; moving that queue line back would put
  the exits beyond the range at which the outer queued drone detects a hull (about 8.5 m).  A passing drone
  near the gate frame may not see a queued neighbour within 5 m (1.2 s in `mutex_deadlock_resolution`):
  the separation then rests on the pass-path geometry.
* **Speed.** Current demo real-time factors with visualization cameras are listed in section 4 and depend on
  other jobs sharing the machine.

## 7. Reproducing

```bash
python formal/check_properties.py
python -m pytest -q
python scripts/run_all_demos.py && python scripts/make_figures_v2.py && python scripts/validate_assumptions_v2.py
python scripts/run_demo.py --scenario lost_drone_rejoin  # watch one (fleet view)
python scripts/render_demo.py results/v2/demos/lost_drone_timeout    # fleet-view GIF again, from the logs
```

## 8. Reduced-progress waiting, a derived timeout, the automaton made visible

Still no communication: no message, no ping, no shared variable.  Every quantity below is computed by each drone
from its own sonar ranges, its own navigation and the mission plan shared before the dive (DI-31).

### 8.1 Reduced progress while a neighbour is missing

When a drone is missing, the remaining drones enter a reduced-progress FORMATION_WAIT_REJOIN mode to facilitate
catch-up while preserving the formation.

* **Why slow down.** With the formation at v_wait = 0.20 m/s, a lost drone closes on its slot at
  v_recovery - v_wait = 0.50 - 0.20 = 0.30 m/s.
* **Why not stop.**  The survey goes on, the vehicles keep steerage and their current compensation, and a drone
  that never comes back costs at most t_rejoin x (1 - v_wait / v_nominal) = 10 s of mission time.
* **Why no message.**  The fleet has no communication channel by design (no modem, no ping, no heartbeat): the
  coordination comes from the plan shared before the dive and from each drone's own sensing, and a lost drone is
  the one least likely to be reachable.  So each drone decides on its own, when it notices, and the others see
  the effect through their sonars.
* **Why 0.20 m/s.** From the configuration, not tuned on a run: the 0.30 m/s catch-up speed is 3 times the
  worst residual drift of the envelope (0.15 m/s, compared with 0.05 m/s at nominal speed); two drones
  that notice the absence 7 s apart drift apart along-track at
  v_nominal - v_wait = 0.10 m/s, i.e. 0.45 m with the 5 s ramp, below e_ok = 0.5 m.  A slower WAIT would buy
  catch-up with formation error.
* **How: a local formation time, not a slower vehicle.**  Slowing the vehicle alone would leave its slot moving
  at the planned speed: a formation error.  Each drone evaluates the shared FormationClock at its own formation
  time `t_f = t - tau` (holo_fleet/control/formation_time.py) and lowers the rate of t_f so that its slot advances
  at v_wait, with a ramp of 0.02 m/s^2 into and out of it.  The slot of the perception, the slot of the flow and
  the feed-forward of the slot tracking all use t_f.  Who slows down, each from its own observation:
  * a drone keeping its slot that misses an expected neighbour (FORMATION_WAIT_REJOIN);
  * the lost drone itself: out of its slot it misses its neighbours too (the "missing" relation is symmetric: a
    range between two drones against their template distance), so its slot stays level with the slowed fleet
    until it is back near it;
  * a drone that has not noticed (range-only sensing: an echo of another neighbour can stand in for the missing
    one): it sees the drone ahead of it in its lane closer than planned by more than 0.08 m and slows down too
    (ramp 0.05 m/s^2); on its slot it keeps slowing until that drone pulls away, so in a lane the slowdown ends
    front to back;
  * other pairs only stretch, and the sonar progress consensus (now only within e_lost of the own slot)
    re-aligns them.
* **Bounded.**  At most t_rejoin of moving formation time per episode; the nominal rate in the mutex zone (the
  gate protocol has its own timing); planned holds stay as planned.  Afterwards the rate returns to the planned
  one with the same ramp; tau is kept: the formation does not sprint to recover the schedule.
* **Not a hidden channel.**  tau is a private state of each drone (a test checks that the module keeps no
  module or class variable); the neighbours see the slowdown only through their sonars, as a change of range.
  The simulator state is read by the referee and the fleet view only.

### 8.2 Timeout derived from the recovery capability

`t_rejoin = min(t_rejoin_max, d_rejoin / (v_recovery - v_wait) + margin) = min(60, 7.5 / 0.30 + 5) = 30 s`

* **Why a timeout.**  A lost drone may never come back (a permanent fault): the others must not slow down and
  wait for ever (formal mutation Fm4: without the timeout they may).
* **Why from the recovery capability.**  It is the time a lost drone still at the edge of the sensing range
  needs to close the gap at the catch-up speed, plus a margin for the ramp, the confirmation and the final
  approach.  The cap bounds the cost of a configuration with a small catch-up; v_recovery <= v_wait is rejected.
* **d_rejoin = 7.5 m is a configured bound** (the range of the neighbour check), not an onboard estimate: after
  its last association a missing drone is not observed at all, and the last residual says nothing about how far
  it drifted since.
* **Flow.**  MISSING -> FORMATION_WAIT_REJOIN -> FORMATION_FOLLOW (`neighbor_rejoined`) or DEGRADED_FORMATION
  (`rejoin_timeout`).  Each drone times the absence from its own detection (a latch per drone, no agreement).
* **DEGRADED_FORMATION:** no reconfiguration, every drone on its original slot, the hole
  stays open, the drone is re-included if it comes back (`slot_reoccupied`); the survey speed is resumed.

### 8.3 The automaton made visible

* **Readable guards.**  Every edge of the spec carries a label: a Python expression over the observation
  variables and the configured thresholds (holo_fleet/ha/guard_text.py).  Z3 proves, mode by mode, that under
  the priority order the edge selected by the labels is the edge selected by the executed guards (L1); a wrong
  label is caught (ML).  The main guards:

| priority | to | guard | cause in the log |
|---|---|---|---|
| 1 | FAILSAFE_HOLD_OR_RETREAT | `not sense_ok or not env_ok` | `sensor_fault`, `envelope_violation` |
| 2 | COLLISION_AVOIDANCE | `d_min < d_ca` (1.7 m; kept while `d_min < d_ca_exit`, 2.0 m) | `collision_risk` |
| 3 | SEPARATION_WARNING | `d_ca <= d_min < d_warning` (2.4 m; exit 2.7 m) | `separation_warning` |
| 4 | MUTEX_PASS | `mutex_zone and at_queue and has_prio and not occ_busy`; then `committed and not passed` | `mutex_priority_and_free` |
| 4 | MUTEX_YIELD | `mutex_zone and at_queue and (occ_busy or not has_prio)` | `mutex_occupied`, `no_priority` |
| 4 | MUTEX_APPROACH | `mutex_zone and not at_queue` | `mutex_zone_entered` |
| 5 | FORMATION_RECOVERY | `committed and passed`; from formation keeping `form_err > e_lost` (1.2 m) | `mutex_exit_passed`, `own_slot_lost` |
| 6 | FORMATION_WAIT_REJOIN | `not neighbors_ok` | `neighbor_missing` |
| 6 | DEGRADED_FORMATION | `neighbors_ok and degraded` | `rejoin_timeout` (from WAIT_REJOIN) |
| 6 | FORMATION_FOLLOW | `neighbors_ok and not degraded` | `neighbor_rejoined`, `slot_reoccupied` |
| | back to formation keeping from any other mode | `form_err < e_ok and t_ok >= t_ok_hold` (0.5 m, 2 s) and the neighbour guard | `slot_recovered`; a hazard left: `fault_cleared`, `collision_risk_cleared`, `separation_clear` |

  The mission-level rules (rows 4-6) apply only when no hazard holds; otherwise the drone remains in its mode.
* **Transition log.**  Every mode change writes a `transition` event (time, from, to, edge, cause, readable
  guard) to events.jsonl and the last one to the state record.
* **Diagram generated from the spec** (scripts/make_automaton_diagram.py, from build_edges):
  figures/v2/automaton/automaton_full.{png,svg} (the ten states in six priority bands, all 120 edges, the exact
  guards, compact self-loops) and automaton_presentation.{png,svg} (the same automaton in plain words, for
  slides).  A test checks that every edge of the spec is drawn, nothing else, with the label of the spec.
* **Fleet view.**  An automaton card for one selected drone (`--select K`): its state on the priority layout,
  the last transition with its cause and guard, the speed of its formation reference and its delay behind the
  plan.

### 8.4 integrated_short inside the control-feasible envelope

Only the scenario changed, not the controller: the cross-current gust 0.35 -> 0.27 m/s.  Replaying the logged
gate passage of drone 0 against the envelope gives 1.12 times the authority at 0.35 m/s, 1.03 at 0.30, 0.996 at
0.28 and 0.979 at 0.27: 0.27 m/s is the first value with a margin.  The run: at most 0.98 times the authority
(net cross-current 0.22 m/s), occupancy <= 1, entry order drone 0, 2, 1, min distance 3.34 m, re-formed 5.2 s
after the last passage.  The onboard view (E2) and the referee (E1) now agree on every run.

### 8.5 Current demo runs (one seed each)

| scenario | missing (onboard, per drone) | end of the wait | formation reference / fleet speed | formation error of the others | min distance, collisions |
|---|---|---|---|---|---|
| `lost_drone_rejoin` (thrusters of the rear-left drone off 8.1-15.1 s, 0.35 m/s lateral current) | 13.2 / 13.4 / 15.5 s | `neighbor_rejoined` at 29.9 / 33.0 / 33.4 s | v_ref 0.20 m/s (mean in WAIT 0.21-0.23); fleet 0.30 -> 0.22 -> 0.30 m/s | <= 0.20 m (0.07 m at the end) | 3.36 m, 0 |
| `lost_drone_timeout` (the same drone's thrusters off for good at 8.1 s) | 14.0 / 16.1 / 22.5 s | `rejoin_timeout` at 43.9 / 46.0 / 52.4 s (30 s each) | v_ref 0.20 m/s (mean in WAIT 0.21); fleet 0.30 -> 0.22 -> 0.30 m/s | <= 0.19 m (0.10 m at the end) | 3.34 m, 0 |

* `lost_drone_rejoin`: the lost drone detects its own fault (FORMATION_RECOVERY at 11.3 s `own_slot_lost`,
  FAILSAFE 13.4-21.6 s `envelope_violation` / `fault_cleared`), returns behind the fleet along its lane and is
  back in its slot at 35.6 s (`slot_recovered`). Referee: P3 recovered in 20.7 s.
  The others end 5.8-6.7 s behind the plan.
* `lost_drone_timeout`: the three remaining drones end in DEGRADED_FORMATION on their original slots, 10 s
  behind the plan, at the survey speed; the failed drone stays in FAILSAFE.  Referee: absent at 43.2 s, P3-deg
  PASS (final error 0.10 m); the nominal P3 stays open, as expected.
* `lost_drone_mutex`: the waiting happens beyond the gate, during the planned rendezvous hold: the clock is at
  rest there, so there is nothing to slow down and the timer is frozen; entry order drone 1, drone 2,
  occupancy <= 1, min distance 3.01 m.
* Other current runs (rows in section 4): `p1_vertical_escape` recovered its formation in 18.0 s; the
  head-current and gust formations lost and recovered in 6.9 s and 10.0 s, outside the envelope.

### 8.6 What is claimed, and what is not

* P1 and P2 are unchanged: the reduced progress only lowers the speed of the slot reference (never above
  v_nominal), and the mutex zone runs at the nominal rate.
* P3 is unchanged in form, under the same assumptions: the abstraction lets the reference move at any speed in
  [v_wait, v_nominal] at every step; the binding case of F1 is v_nominal, a slower reference only adds
  catch-up.  P3-deg: every neighbour leaves FORMATION_WAIT_REJOIN within t_rejoin = 30 s of formation time (F6,
  300 steps; F6r ranking function).
* The runs show the behaviour once per scenario (no multi-seed).
* **Not claimed:** that a lost drone always comes back; that the fleet is deadlock-free in every possible
  geometry; that collisions are impossible; that all drones learn at the same time that a drone is missing -
  each notices on its own, at its own time, or not at all, and the slowdown is designed for that.  Limits:
  section 6 ("Lost drones").

### 8.7 Verification

* Formal: 186 checks with the expected verdict (determinism 93 with L1 and ML; observations 31; P1 13; P2 27;
  P3 22 with the free reference speed and F6 over 300 steps).
* Tests: the formation time (ramp, budget, mutex zone, holds, the delay kept, a private state of each drone);
  the derived timeout; same-lane following and the front-to-back release; the progress consensus near the own
  slot; no give-way while catching up; every edge of the spec in the diagram and nothing else; the transition
  causes, in unit tests and on the runs; the reduced progress and the survey speed on the runs; integrated_short
  inside the envelope.  The isolation tests (no ground truth reaches a controller, no message channel) cover the
  new modules.
