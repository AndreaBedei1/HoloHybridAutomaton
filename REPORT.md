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
| P1 inter-vehicle separation | 13 | 13 | 88.1 s |
| P2 critical-region mutual exclusion | 19 | 19 | 65.8 s |
| P3 formation recovery (liveness, ranking functions) | 11 | 11 | 0.5 s |

Key numbers:
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

**Assumptions** (measured on the runs in [results/v2/ASSUMPTIONS.md](results/v2/ASSUMPTIONS.md)):
* both drones of a pair detect each other;
* sonar data age <= tau_max;
* the closing speed of a pair is <= c_max;
* braking and drift bounds from the plant calibration;
* queued drones hold their point within 0.1 m;
* a committed drone crosses the CR at >= 0.12 m/s;
* for P3, perturbations end and the residual disturbance after the integrator is <= 0.05 m/s.

## 4. Experiments (HoloOcean, one seed each, communication off)

| scenario | drones | P1 min distance [m] (d_safe 1.0) | P2 max CR occupancy | P3 episodes / recovery [s] (after the last perturbation) | contacts | messages | current envelope | static rank uses | RTF (with cameras) |
|---|---|---|---|---|---|---|---|---|---|
| `p1_head_on` | 2 | 4.054 | - | n/a | 0 | 0 | inside | 0 | 0.638 |
| `p1_vertical_escape` | 4 | 2.489 (vehicle clearance 2.44) | - | 25.2 (14.3) | 0 | 0 | inside | 0 | 0.604 |
| `p1_two_lines` | 6 | 3.209 | - | n/a | 0 | 0 | inside | 0 | 0.627 |
| `formation_triangle` | 3 | 4.604 | - | never lost | 0 | 0 | inside | 0 | 0.535 |
| `formation_square` | 4 | 3.459 | - | never lost | 0 | 0 | inside | 0 | 0.577 |
| `formation_six` | 6 | 3.443 | - | never lost | 0 | 0 | inside | 0 | 0.616 |
| `formation_gust` | 4 | 2.778 | - | 5.7 (4.7) | 0 | 0 | OUT (0.85 m/s jet, flagged) | 0 | 0.608 |
| `gate_single` | 3 | 3.308 | 1 | 63.2 (5.0) | 0 | 0 | inside | 0 | 0.592 |
| `integrated_short` | 3 | 3.324 | 1 | 69.5 (4.7) | 0 | 0 | inside | 0 | 0.616 |

All runs COMPLETE, determinism-monitor violations 0, ground truth used by controllers: NO.  P3 recovery is measured from the loss; in brackets from the end of the last perturbation (jet, encounter, last gate passage).  In the gate scenarios the formation is broken on purpose by the abreast queue and re-formed at the rendezvous.

* **P1**
  * `p1_head_on`: both drones give way to the right at 8 m and pass at about 4 m without entering the
    warning band.
  * `p1_two_lines`: six drones. The flanked middle drones give way UP (one line) and DOWN (the other
    line); the outer drones give way to the right.
  * `p1_vertical_escape`: the traffic rule is switched off, so only the safety layer acts against a
    vehicle that does not react.
    * The boxed-in centre drone goes through SEPARATION_WARNING and COLLISION_AVOIDANCE and escapes
      upwards: REAR+UP, then FWD+UP, about 1.1 m of climb.
    * The rear drone does the same when the vehicle reaches it.
    * The minimum distance between drones stays above d_safe, and the clearance to the vehicle stays
      above 2.4 m.
* **P2** (`gate_single`, `integrated_short`)
  * Abreast queue; entry order left, centre, right; one drone at a time in the CR.
  * Static rank never used, no occupancy timeout.
  * The formation re-forms at the rendezvous beyond the gate, about 5 s after the last passage.
  * The SEPARATION_WARNING episodes during the gate phase come from the deliberately loose sound
    distance bound (DI-13). They are safe and short.
* **P3**
  * The triangle, the square and the six-drone line never lose the formation under in-envelope
    currents. The maximum referee formation error is reported in the table.
  * `formation_gust`: a 0.85 m/s jet breaks the square.
    * The jet drone declares ENVELOPE_VIOLATION and holds in FAILSAFE.
    * The formation is lost after the jet and recovered 6-7 s later.
    * The onboard current estimate follows the jet and returns to the 0.1 m/s background.

Figures: [figures/v2/](figures/v2/) (sensor, p1, p2, p3).  GIFs: [figures/v2/gifs/](figures/v2/gifs/).

## 5. Design iterations

All 21 iterations, with problem, cause, fix, why the fix is principled and result, are in
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
* **Evidence.** One seed per scenario, by design: no statistics.
* **Speed.** The demos run at about 0.6x real time because of the visualisation cameras.

## 7. Reproducing

```bash
python formal/check_properties.py
python -m pytest -q
python scripts/run_all_demos.py && python scripts/make_figures_v2.py && python scripts/validate_assumptions_v2.py
python scripts/run_demo.py --scenario gate_single        # watch one
```
