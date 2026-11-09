# Guarantees, evidence and operating envelope

What the system guarantees, under which assumptions, and of which kind each piece of evidence is.  The checks are
in `formal/` (run `python formal/check_properties.py`; results in `formal/results/SUMMARY.md`); the regression suite
in `tests/`; the campaign in `docs/ACCEPTANCE_CRITERIA.md`.

Kinds of evidence used below:

* **proof** - an SMT query (z3) that is UNSAT for every value of the variables: an unbounded statement about the
  encoded model (k-induction included);
* **BMC** - bounded model checking: UNSAT up to a stated horizon;
* **exhaustive numeric** - every point of a finite grid or every sector pattern is checked;
* **randomized** - many random samples (scenes, errors, attitudes) and no counterexample: evidence, not a proof;
* **mutation** - a deliberately broken variant must produce a counterexample, so the check is not vacuous;
* **empirical** - HoloOcean runs (the campaign);
* **assumption** - stated, not checked.

## P1 - separation (safety)

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| The onboard distance never exceeds the true distance of the nearest hull, for single and merged multi-target echoes (association-free touch-set bound, perception/distance_bound.py) | randomized (S0, ~35 000 scenes incl. cube corners, edges, near field, square-formation pairs, extreme errors, random detection boundary in 60 deg +- one leaf); mutation S0m | echo error in [-eps_near, +eps_far]; relative hull tilt <= 10 deg (measured 0.03 deg); direction-lattice covering radius (numeric bound, tests/test_distance_bound.py) |
| A missed capture becomes a stale reading (EchoHold) | unit tests | consecutive misses <= tau_max |
| From any state with d_hat >= d_warning the separation stays >= d_safe | BMC S1a (4 s) + proof S1b (k-induction, k = 13); mutation S1m | BOTH drones of the pair detect each other and brake (a_pair = 2 a_brake); speed, current and staleness bounds of config.Envelope; staleness allowance v_close * age |
| The same with ONE drone reacting (a missed detection, a drone holding in FAILSAFE, a non-cooperative vehicle) | BMC S1d at the per-drone deceleration calibrated in the simulator (0.743 m/s^2, results/calibration/plant_calibration.json); reported S1o: SAT at the configured, conservative a_brake = 0.5 (the smallest d_warning that verifies is then 2.56 m > 2.4 m) | the one-sided claim rests on the simulator calibration, not on the configured margin; to be re-measured on hardware |
| The escape opens every threat pattern | exhaustive numeric S2a-S2d over the sector patterns; mutation S2m | the six-sector geometry |
| At most one vertex angle >= 90 deg among pairwise-separated drones | proof S3 | - |
| A sonar blackout <= 0.8 s from d >= 2.34 m keeps d >= d_safe | proof S4 | FAILSAFE hold, unrejected drift bound |
| Under LOW and MEDIUM, P1 held in every run | empirical (campaign) | the echo bound is exceeded under LOW (per-run mean 0.86 % of captures beyond +eps_far, maximum 8.4 %; data age up to 0.3 s > tau_max) and MEDIUM (3.6 %, maximum 15.6 %): P1 under LOW and MEDIUM is tested, not proved |
| Assumption (D) of the distance bound: no echo means no object in the cone | assumption | the simulator's sonar is a hard-edged cone with uniform detection; real beam patterns, fading and crosstalk violate it (external validity) |

## P2 - mutual exclusion at the gate (safety)

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| Never two drones with PRIORITY; exactly the first of the queue order leads (no deadlock), abreast n = 2..6 and the stacks | proof M1, M2 (every occupancy with <= 1 vacant point), numeric M2v; mutations M1m, M1m2 | heading and position tolerances; M2s documents the scope (two adjacent vacant points) |
| Queue geometry: CR inside every FRONT cone, merge clearance, neighbours never hidden in the structure window | exhaustive numeric M4a-M4f, M4s; mutations M4m, M4fm | gate map; hold error of a queued drone |
| Every queued drone has a sonar exit witness of the drone that passes, inside the validated domain | exhaustive numeric M4g (path sampling, bars' occlusion, structure window, detection range 10 m) | queue points within GateRule.queue_lateral_max = 3.5 m (planner check; abreast n >= 4 is refused and reported as the domain boundary); geometric availability only: in HoloOcean the witness from a lateral queue point is rarely obtained (detection beyond the frame) |
| Never both in the CR with the occupancy latch | BMC M3 (42 s, every queue view; still UNSAT at 65 s, longer than the slowest admissible crossing); mutation M3m (no latch) | the occupant crosses at >= V_LO without stopping (FAILSAFE, CA or SW inside the CR violate it: dwell up to 12.5 s observed); every entry into the CR is seen in the FRONT corridor (violated: 9 MEDIUM crossings classified STRUCTURE were missed by a queued drone); every exit witness is the occupant (violated: 69 of 260 MEDIUM sonar witnesses match no drone, 7 premature releases).  In the campaign P2 held through unmodelled kinematic margins and precedence |
| Stage-2 entry check at the hold point (no CR-compatible echo over 3 captures; B2: no drone heard inside) | reported M4h (formal/check_stage2.py): the fraction of the CR observable from the hold point | the far part of the CR, where hull and bar echoes share ranges, is masked to the sonar; a silent occupant there is covered only by the latch argument above |
| A committed drone gives up only behind the give-up plane, never once admitted, and the give-up clears its commitment | proof G2, G2c (formal/check_determinism.py, every mode); mutation MA (the withdrawn flow-only retreat keeps the commitment: counterexample); unit test tests/test_reloc_hold.py | its own gate-frame navigation error <= GateRule.nav_err_m (1.0 m); a give-up caused by a persistent stage-2 condition repeats (commit - hold - give-up): a livelock, judged STALLED, not excluded |
| A drone stopped inside the CR by a fault is released by the t_occ_max fallback | BMC M3f (counterexample, scope) | outside the B0 guarantee; B2 blocks the fallback while that drone is heard inside |
| Under every profile, no P2 violation | empirical (campaign) | - |

## P3 - formation (liveness)

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| Along-track and lateral recovery: ranking decrease in the saturated and linear bands, invariant ball | proof F1-F3; mutations Fm1, Fm2 | declared current envelope (F5, mutation Fm3, Em1), catch-up margin |
| The automaton selects the right formation mode | proof F4a-F4d | - |
| A neighbour that never reappears is declared vacant within t_rejoin | BMC F6, ranking F6r; mutation Fm4 | timer runs only while the drone keeps its slot and the clock moves |
| An association identifies a neighbour only when every other required neighbour is accounted for; a present neighbour is matched before a missing or vacant one; B2 reserves the echo of a heard drone | unit tests (tests/test_slot_identification.py, mutation-checked) | range-only limit: one displaced neighbour in the overlap of two adjacent cones looks like two neighbours (B0) |
| Recovery and degraded formation in HoloOcean | empirical (campaign) | documented limitations: relative along-track drift under acoustic outage; relocalization beside the gate opening (C5, relaxed in B2 when every other drone is located) |
| Scope of F1-F3 | - | the ranking functions concern each drone's own estimated slot error against its own (possibly delayed) reference; they do not imply recovery of the true formation: navigation error and the consistency of the private formation times are outside them (B2 re-synchronises the formation times from identified fixes, control/formation_time.py) |

## Automaton and interfaces

| Claim | Evidence |
| --- | --- |
| Local determinism and completeness of the guards, the priority hierarchy (safety over gate over formation), fault -> FAILSAFE from every mode | proof (93 checks, holo_fleet/ha/spec.py) |
| The perception-to-automaton observation invariants | proof (31 checks) |
| No determinism or observation-consistency violation at run time | empirical (every run counts them) |

## B2 - one-way acoustic identity beacon

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| Every position compatible with a measurement inside the declared envelope lies in the region the perception builds | randomized B2-A1 | envelope = MEDIUM channel at 3 sigma + array calibration bias + one-sided clear-path multipath excess (docs/ACOUSTIC_CHANNEL_MODEL.md) |
| No measurement of a hull in the CR passes the acoustic exit, outside or exclusion test | exhaustive numeric B2-A2/A3 | as above |
| The acoustic exit evidence is available | reported B2-M4g: holds for n = 2 and 3 (exit by range once the drone is far enough beyond) | - |
| H6 payload soundness: a sender truly in the CR whose estimate errs within its sent bound (|e_h| <= h, |e_z| <= z, radius hypot(h, z), through the 64-bit packet of payload v2) is never cleared by the payload rule nor placed beyond the exit face | randomized B2-P (20 000 centres in the CR, boundary over-sampled); mutation B2-Pm (radius max(h, z) clears an occupant) | the sender's NavBound holds (perception/nav.py: declared compass, DVL, depth envelope; logged against the truth in every run, tag NAV_BOUND_PREMISE_VIOLATED); the HIGH profile is outside it; the 1 kbit/s short-packet air time is an unverified hardware assumption (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, 2b) |
| An unknown occupant never becomes FREE by a timeout | design + unit tests (tests/test_payload_b2.py, tests/test_gate_veto.py): a drone whose latest accepted fix placed it possibly in the CR keeps the latch BUSY until cleared; an acoustic-only episode needs A2 of every drone heard possibly inside | scope: drones heard at least once; the sonar latch premise (every entry crosses the FRONT corridor visibly) for drones never heard, and a silent stalled occupant in the masked far half of the CR (BLACKOUT) remains outside the guarantee (M3f, M4h); the same rule holds the stage-2 entry of a committed drone; liveness cost: a drone silent after such a fix blocks the gate. Payload claims hold only inside the declared navigation envelope (MEDIUM sensors): runs beyond it (HIGH) and any run tagged NAV_BOUND_PREMISE_VIOLATED are reported and counted as premise failures, never as P2 evidence for the payload rules |
| A drone heard inside the CR blocks the timeout fallback and the stage-2 entry | design: two accepted fixes on one track, the latest within acoustic.fresh_s (one lost ping is no gap), on the clear-path region (no one-sided multipath excess) | a veto added to the sonar rules: a miss leaves the sonar-only behaviour; the clear-path region removes the false vetoes of drones waiting beyond the exit |
| Acoustic data never reach separation, collision avoidance, precedence or the sonar witness | static (AST) B2-S; tests/test_b2_isolation.py | - |
| The channel is strictly one-way: periodic TDMA broadcast with position, error bound and gate phase; no request, reply, acknowledgement or command | code structure (sim/beacon.py) and tests | hardware assumptions H1-H6 (H6 has no off-the-shelf product combining the 64 ms frame, a 3-D array and a timed one-way range) |
| The gate status never enters the precedence: a FRONT-only echo in the band makes no relation and always feeds the latch | S5 (text search), S6b on the real class, M3, M3h, S2, S2h with the code's predicate; mutations M3b, S2b | latch abstraction of M3 |
| A claim (committed or admitted) holds the gate, for ever if the drone falls silent | S1, S4; mutations S1m, S4m | a received packet is genuine (CRC-16) |
| Silence or an old report never discards a positive corridor echo | S11; mutation S11m | - |
| A drone truly in the CR never reports not engaged or passed | S9, B2-PE; mutations S9m, B2-PEm; B2-PEg for the fix geometry alone | gate-frame navigation error <= nav_err_m + R_OUT |
| A verified passage has the true centre beyond the CR grown by the hull | S7 (randomized) | DVL scale and fit error of the sampled ranges; the REAR and odometry branches never ran in a campaign (42 of 42 passages by the crossing signature) |
| A failed drone may stay in the CR for good, or leave it by itself | bench calibration/probe_buoyant_ascent.py | the ascent is an emulation; the rise speed is assumed, not measured |

## Reproducibility

The wrapper noise (sonar dropout, blackouts, range jitter, fades, DVL outages, latencies, the acoustic channel) is
seeded from the scenario seed; HoloOcean's own sensor noise (DVL, IMU, magnetometer, depth, sonar AddSigma /
MultSigma / RangeSigma) uses the engine's unseeded generator.  Runs are therefore statistically, not bit-for-bit,
reproducible.  Every run records the commit, the engine build (patched root, binary sha256), the fleet configuration
hash, the seeds and the profiles.
