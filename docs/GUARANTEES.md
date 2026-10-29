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
| From any state with d_hat >= d_warning the separation stays >= d_safe | BMC S1a (4 s) + proof S1b (k-induction, k = 13); mutation S1m | speed, current and staleness bounds of config.Envelope; staleness allowance v_close * age |
| The escape opens every threat pattern | exhaustive numeric S2a-S2d over the sector patterns; mutation S2m | the six-sector geometry |
| At most one vertex angle >= 90 deg among pairwise-separated drones | proof S3 | - |
| A sonar blackout <= 0.8 s from d >= 2.34 m keeps d >= d_safe | proof S4 | FAILSAFE hold, unrejected drift bound |
| Under LOW and MEDIUM, P1 held in every run | empirical (campaign) | MEDIUM exceeds the echo bound in ~1.2 % of captures (+-2-bin jitter): tested, not proved |

## P2 - mutual exclusion at the gate (safety)

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| Never two drones with PRIORITY; exactly the first of the queue order leads (no deadlock), abreast n = 2..6 and the stacks | proof M1, M2 (every occupancy with <= 1 vacant point), numeric M2v; mutations M1m, M1m2 | heading and position tolerances; M2s documents the scope (two adjacent vacant points) |
| Queue geometry: CR inside every FRONT cone, merge clearance, neighbours never hidden in the structure window | exhaustive numeric M4a-M4f, M4s; mutations M4m, M4fm | gate map; hold error of a queued drone |
| Every queued drone has a sonar exit witness of the drone that passes, inside the validated domain | exhaustive numeric M4g (path sampling, bars' occlusion, structure window, detection range 10 m) | queue points within GateRule.queue_lateral_max = 3.5 m (planner check; abreast n >= 4 is refused and reported as the domain boundary); geometric availability only: in HoloOcean the witness from a lateral queue point is rarely obtained (detection beyond the frame) |
| Never both in the CR with the occupancy latch | BMC M3 (42 s, every queue view); mutation M3m (no latch) | the occupant crosses at >= V_LO: the t_occ_max fallback is safe only for a healthy crossing |
| A drone stopped inside the CR by a fault is released by the t_occ_max fallback | BMC M3f (counterexample, scope) | outside the B0 guarantee; B2 blocks the fallback while that drone is heard inside |
| Under every profile, no P2 violation | empirical (campaign) | - |

## P3 - formation (liveness)

| Claim | Evidence | Assumptions |
| --- | --- | --- |
| Along-track and lateral recovery: ranking decrease in the saturated and linear bands, invariant ball | proof F1-F3; mutations Fm1, Fm2 | declared current envelope (F5, mutation Fm3, Em1), catch-up margin |
| The automaton selects the right formation mode | proof F4a-F4d | - |
| A neighbour that never reappears is declared vacant within t_rejoin | BMC F6, ranking F6r; mutation Fm4 | timer runs only while the drone keeps its slot and the clock moves |
| An association identifies a neighbour only when every other required neighbour is accounted for; a present neighbour is matched before a missing or vacant one; B2 reserves the echo of a heard drone | unit tests (tests/test_slot_identification.py, mutation-checked) | range-only limit: one displaced neighbour in the overlap of two adjacent cones looks like two neighbours (B0) |
| Recovery and degraded formation in HoloOcean | empirical (campaign) | documented limitations: B0 relative along-track drift; relocalization beside the gate opening (C5) |

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
| The acoustic exit evidence is available | reported B2-M4g: not available in the validated gate geometry under this envelope; B2 then uses the sonar latch | - |
| Acoustic data never reach separation, collision avoidance, precedence or the sonar witness | static (AST) B2-S; tests/test_b2_isolation.py | - |
| The channel delivers identity pings only (TDMA, no payload, no handshake) | code structure (sim/beacon.py) and tests | hardware assumptions H1-H5 |

## Reproducibility

The wrapper noise (sonar dropout, blackouts, range jitter, fades, DVL outages, latencies, the acoustic channel) is
seeded from the scenario seed; HoloOcean's own sensor noise (DVL, IMU, magnetometer, depth, sonar AddSigma /
MultSigma / RangeSigma) uses the engine's unseeded generator.  Runs are therefore statistically, not bit-for-bit,
reproducible.  Every run records the commit, the engine build (patched root, binary sha256), the fleet configuration
hash, the seeds and the profiles.
