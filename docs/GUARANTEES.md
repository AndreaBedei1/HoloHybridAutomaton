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
| Stage-2 entry check at the hold point (no CR-compatible echo over 3 captures; B2: no drone heard inside) | reported M4h (formal/check_stage2.py): the fraction of the CR observable from the hold point | the far part of the CR, where hull and bar echoes share ranges, is masked to the sonar; the check does not read the latch: a drone committed before a hull arrives is held at the hold point only by a claim, a fix placing the hull inside, or the drift reach of a silent drone (S12) |
| A committed drone gives up only behind the give-up plane, never once admitted, and the give-up clears its commitment | proof G2, G2c (formal/check_determinism.py, every mode); mutation MA (the withdrawn flow-only retreat keeps the commitment: counterexample); unit test tests/test_reloc_hold.py | its own gate-frame navigation error <= GateRule.nav_err_m (1.0 m), which a wrong map correction can break (RC12: 2.7 m); a give-up caused by a persistent stage-2 condition repeats (commit - hold - give-up): a livelock, judged STALLED, not excluded |
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
| An unknown occupant never becomes FREE by a timeout | design + unit tests (tests/test_payload_b2.py, tests/test_gate_veto.py): a drone whose latest accepted fix placed it possibly in the CR keeps the latch BUSY until cleared; an acoustic-only episode needs A2 of every drone heard possibly inside | scope: drones heard at least once; the sonar latch premise (every entry crosses the FRONT corridor visibly) for drones never heard, and a silent stalled occupant in the masked far half of the CR (BLACKOUT) remains outside the guarantee (M3f, M4h); the same rule holds the stage-2 entry of a committed drone; liveness cost: a drone silent after such a fix blocks the gate until it is dead (the claim row). Payload claims hold only inside the declared navigation envelope (MEDIUM sensors): runs beyond it (HIGH) and any run tagged NAV_BOUND_PREMISE_VIOLATED are reported and counted as premise failures, never as P2 evidence for the payload rules |
| A drone heard inside the CR blocks the timeout fallback and the stage-2 entry | design: two accepted fixes on one track, the latest within acoustic.fresh_s (one lost ping is no gap), on the clear-path region (no one-sided multipath excess) | a veto added to the sonar rules: a miss leaves the sonar-only behaviour; the clear-path region removes the false vetoes of drones waiting beyond the exit |
| Acoustic data never reach separation, collision avoidance, precedence or the sonar witness | static (AST) B2-S; tests/test_b2_isolation.py | the exception of the map fit: through the own pose an echo can be relabelled STRUCTURE, leaving the targets of those layers (row of the along-axis correction) |
| The channel is strictly one-way: periodic TDMA broadcast with position, error bound and gate phase; no request, reply, acknowledgement or command | code structure (sim/beacon.py) and tests | hardware assumptions H1-H6 (H6 has no off-the-shelf product combining the 64 ms frame, a 3-D array and a timed one-way range) |
| The gate status never enters the precedence: a FRONT-only echo in the band makes no relation and always feeds the latch | S5 (text search), S6b on the real class, M3 and S2 with the code's predicate; M3h (42 s) and S2h (32 s) with the crossing drone stopped before the CR, bounded: beyond, M3hs and S2hs give the counterexample of a stop behind the bars with no claim heard; mutations M3b, S2b | latch abstraction of M3; the 26 s give-up of the code is not modelled |
| A claim (committed or admitted) holds the gate until the drone reports otherwise or is dead: not heard for `t_silent_gate_s` = 70 s by a receiver that hears the others. A receiver that hears nobody declares no one dead | S1, S4; mutations S1m, S4m | a received packet is genuine (CRC-16); premise V: the channel never silences a live drone for 70 s (longest silence of a live sender observed: 31 s; checked on every batch) |
| A silence shorter than `t_silent_gate_s` never excludes a drone from a gate decision, and never discards a positive corridor echo | S11; mutation S11m | a hull in the part of the CR masked by the map window has no echo: 45-92 % of the CR, depending on the viewpoint (reviewer estimate); there only a claim or a fix place it |
| A committed drone carried into the far part of the CR or beyond, not admitted (the avoidance layer, the water), goes on to the exit and never back through it | tests/test_reloc_hold.py; logged as `PAST_CENTRE` | the rule has no lateral term: a drone carried beside the frame goes on past the gate without having passed it and the gate is counted as missing, with no safety consequence; the stage-2 check does not apply to it: it did not choose to enter; its passage is not verified, so it reports committed up to the exit, then not engaged, and if it returned to the CR it would report not engaged (the row of the sender rule does not hold on this branch) |
| Heading loop with integral action: a queued drone keeps its heading tolerance through a wait of minutes | tests/test_lowlevel_heading.py; empirical (RC10) | `heading_tol_deg` is a guard of `at_queue`, unchanged. Simulator limit: a compass bias that depends on thruster current is not modelled; on a vehicle the integrator would turn the hull to null the apparent error, up to a few degrees of true heading |
| A correction of the estimate accepted without a validated fit, within the current commitment, is undone when a hold at the gate ends with no valid localization, after the give-up edge and never while committed. A step under the identity rule is at most 0.5 m | tests/test_unconfirmed_corrections.py; logged as `UNCONFIRMED_CORRECTIONS_UNDONE` | a recovery heuristic: a hold without validation does not refute a correction. The map fit is not proved correct: a fit can explain the echoes about as well as a pose on the far side of a bar (also without noise) or mirrored about the axis (RC12), and in the MEDIUM batches these corrections do not lower the error on average (278 to 309 corrections depending on the count, mean change +0.004 to +0.02 m), which balances rescues against damage. An undo can remove a correct correction made within the commitment before the hold ended. What is corrected after the end of the hold stays (the sum is sealed then). The distance bound of the separation layer uses the raw ranges, but which echoes are targets depends on the pose (structure window), and a wrong correction changes it. Admission needs a validated fit; the past-centre path does not. The rule makes a wrong correction recoverable, and does not prevent it; closed-loop recovery is an observation of the campaign, not a guarantee. Open: a correct fit refused at 0.5-1.1 m of lateral error (s960). No property models the step, and an undo of correct corrections is not detected; the navigation error premise is not re-established by it. In open-loop replay of the logged hold timeouts it matters once, in s995 (lateral error 2.77 m to 0.46 m); counts over the batches depend on how a correction and an undo are matched and are not stated |
| A dead drone (the claim row) is excluded from every gate decision: possible occupant, claim, discard of a corridor echo as a phantom | S12 (Z3, nonlinear real arithmetic) on the exclusion zone; mutation S12m; tests/test_status_payload.py | the operating scope (`holo_fleet/scope.py`), a development decision of 2026-11-12 taken after the INCOMPLETE runs s960, s1068 and s1071, not pre-registered: no permanent failure inside the exclusion zone E of a gate. E holds the positions from which a passive hull, coasting at most `coast_m` = 1 m and then carried by a current of at most w_h horizontally and w_down downward, can reach the CR and the frame (grown by R_OUT) before it has risen above them at `v_rise_min_mps` = 0.05 m/s or more; every position when w_down >= 0.05 m/s. Outside E a dead hull is never in or at the CR. The current bounds are those declared for the mission, not the onboard estimate; flow through the frame and the wash of a live drone are not modelled. Failures inside E are outside the scope: not run, no special handling. Premise V of the claim row |
| Along-axis correction from one FRONT echo | unit tests, randomized on the axis; open-loop replay by the reviewer | identification by elimination only: a hull not accounted for by the premise can be taken for a bar and classified STRUCTURE; bounded to MAX_CORR_M between validated fits; needs the wide simulated FRONT sonar |
| A drone truly in the CR never reports not engaged or passed | S9 (including the report of the controller for a gate already passed), B2-PE; mutations S9m, B2-PEm; B2-PEg for the fix geometry alone | gate-frame navigation error <= nav_err_m + R_OUT; a single gate per mission in every experiment (a second gate is not tested, nor the 2-bit wrap) |
| A verified passage has the true centre beyond the CR grown by the hull | S7 (randomized) | DVL scale and fit error of the sampled ranges; the REAR and odometry branches never ran in a campaign (42 of 42 passages by the crossing signature) |
| A failed drone may stay in the CR for good, or leave it by itself | bench calibration/probe_buoyant_ascent.py | the ascent is an emulation; the rise speed is assumed, not measured |

## Reproducibility

The wrapper noise (sonar dropout, blackouts, range jitter, fades, DVL outages, latencies, the acoustic channel) is
seeded from the scenario seed; HoloOcean's own sensor noise (DVL, IMU, magnetometer, depth, sonar AddSigma /
MultSigma / RangeSigma) uses the engine's unseeded generator.  Runs are therefore statistically, not bit-for-bit,
reproducible.  Every run records the commit, the engine build (patched root, binary sha256), the fleet configuration
hash, the seeds and the profiles.
