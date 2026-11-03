# Independent review A (formal methods / hybrid systems / multi-robot coordination)

Written by an independent reviewer subagent working only from the repository, at HEAD a6efd0f, in read-only mode.
Scratch scripts are in the session scratchpad (`reviewA`: fr.py, wit.py, ff.py, p2margin.py, m3h.py, s1.py).
`tests/test_exit_witness.py` and `tests/test_mutex_rule.py` pass (20/20). Reproduced faithfully and slightly condensed.

**Status of the claims (as of 2026-11-03):**
- **Not yet verified by the analysis team:** the one-sided braking BMC (S1, SAT); the 61 of 281 LOW/MEDIUM sonar
  witnesses whose range differs from the occupant's by more than 1 m; the 9 false-FREE crossings; the "186 vs 205
  checks" count; the M3 rerun at 65 s.
- **Consistent with the team's own forensics:** the seven premature releases (FRONT witnesses at 8.07-10.57 m against a
  true occupant distance of about 6-7 m); the minimum entry gap of about 11.4-11.5 s; the classifier rule; the
  post-freeze change ad4fb5e.

## A. Summary of the claimed contribution

The work presents a leaderless BlueROV2 fleet in HoloOcean 2.3 with a patched octree. Every vehicle runs the same
10-mode local controller with priority-ordered guards (`ha/spec.py`). Sensing is six idealised 120° single-beam sonars
plus dead reckoning. B0 exchanges no runtime messages; B2 adds a one-way, identity-only TDMA ping from which the
receiver derives range and bearing.

The claimed properties are:
- **P1:** separation;
- **P2:** one-at-a-time gate crossing, using a sonar occupancy latch with an exit witness and a 25 s timeout;
- **P3 and P3-deg:** formation recovery, and a degraded formation after a permanent loss.

The evidence is 205 formal checks (Z3 BMC, k-induction, ranking functions, numeric sweeps, mutations) and a 624-run
campaign. REPORT.new §14-22 is empty.

## B. Major concerns

**B1. By the authors' own frozen criteria, the campaign contains safety failures labelled PASS.**
- ACCEPTANCE_CRITERIA §2 item 3 makes "no release of the gate while its occupant is inside it" a safety requirement,
  and says a single safety failure stops the campaign.
- `robustness_metrics.analyze_run` computes `false_release_total`, but `safe` ignores it.
- There are 7 such runs, all PASS. Each is a sonar FRONT witness, BUSY → EXITING, with the occupant in MUTEX_PASS still
  inside the CR at s = 0.32-0.76.
- The witness ranges (8.07-10.57 m) do not match the true observer-occupant distance (about 6-7 m), so a different echo
  triggered the release.

**B2. The exit witness is unsound under noise, and the M3 model cannot represent this.**
- `exit_witness.py` W1-W3 accepts any fresh echo beyond `r_far` that some hull could produce beyond the CR. It does not
  establish that the echo comes from the occupant.
- In `check_mutex.py::latch_bmc`, the witness is `beyond[k] == seen[k] & sA[k] > b_s`, i.e. the occupant's own
  ground-truth position.

| Profile | Sonar witnesses | Range differs from the true occupant range by > 1 m | Matching no drone within 1 m |
|---|---|---|---|
| CLEAN | 60 | 0 | 0 |
| LOW/MEDIUM | 281 | 61 | 56 |

- Sonar releases rise with noise (B0: 14 under CLEAN, 41-48 under MEDIUM), which suggests noise-generated witnesses.

**B3. The other premises of M3 are violated too.**
- *Perfect detection.* The model sets `seen[k] == vis`. In 9 MEDIUM runs, a queued drone held FREE for 3.1-3.3 s while
  another drone crossed the entire CR; the occupant's echo was classified STRUCTURE (e.g.
  B0/MEDIUM-ALL/gate_single/s1). The latch never latched, and the observer was protected only because it had no
  PRIORITY. M4d (at least 3 captures) leaves almost no margin against MEDIUM dropout.
- *Monotone crossing at ≥ V_LO.* The automaton's priority (FAILSAFE > CA > SW > MUTEX) lets a committed drone hold or
  escape inside the CR. This happened in about 40 of 195 gate runs, with dwell up to 12.5 s. `env_ok = false` can last
  without bound.
- A false witness, a missed corridor detection and a stall inside the CR were each observed separately. Their
  conjunction defeats P2, and nothing excludes it. P2 held thanks to unmodelled margins (a minimum gap of 11.4 s:
  follower travel plus t_clear). Precedence and the latch share one common-mode dependency: sonar detection.

**B4. The P1 proof covers a cooperative pairwise 1-D abstraction.**
- S1 has both drones brake (`a_pair = 2 a_brake`) when `d_hat < d_warning`.
- With one-sided braking (`a_pair = a_brake`) and every other value unchanged, the 4 s BMC is SAT: a violation from
  d_warning = 2.4 m (reviewer's s1.py).
- This case is relevant for a non-cooperative vehicle, a thrust-limited drone in FAILSAFE, or a missed detection in a
  sector reported healthy. The premise appears only in the check's docstring, not in GUARANTEES.
- S0 is randomized evidence, not a proof. N-drone composition rests on S2 and S3, and S2c lists threat pairs with no
  guarantee (e.g. FRONT+REAR).
- Empirically P1 was never stressed: minimum separation 2.26 m, intruder minimum 1.95 m, against d_safe = 1.0 m.

**B5. The verified "hybrid automaton" is the trivial part.**
- `spec.py` is an if/elif priority chain with a single latched Boolean. The 93 determinism and completeness queries
  confirm what holds largely by construction.
- The safety-critical timers and clocks (t_clear, t_occ_max, prio_since, missing timers, tau, latch state) live in
  perception code and are modelled only in hand-built slices (M3, F6).
- Neither `H_1 || ... || H_N` nor the N-drone timed queue is ever verified. UPPAAL or KeYmaera X would make this
  explicit.
- As implemented, the framing has documentation value but little analytical leverage.

**B6. LIVENESS_FAILURE is practically unreachable.**
- A run is SAFE_DEGRADED when `not live and conservative`, where conservative means any sensor-fault FAILSAFE entry. Under
  MEDIUM-*, 231 of 234 runs contain one.
- All 64 non-PASS runs are SAFE_DEGRADED.
- The relocalization hold (17 runs) is a permanent hold, i.e. a liveness failure labelled "safe".

**B7. Requirements and deadlines are self-referential and come from older code.**
- They are inherited from CLEAN reference runs of the v2 era (12 Oct; e759156), which predate C3/C5/P2/B2.
- A property is required only where the reference achieved it: lost_drone_mutex requires neither P3 nor P3-deg, and the
  p1_* scenarios require no P3.
- Degraded runs get 1.5·T_clean(B0) + 20 s; CLEAN runs get 3·T_ref + 30 s. B0 CLEAN at 48/48 is therefore close to true
  by construction.

**B8. The protocol was not frozen and the evaluation set is not held out.**
- The B0 CLEAN runs, which fix every deadline, ran at 1a14872.
- C2 included B2 runs; B2 code was then changed (ad4fb5e, A2 "exit by range") and the B2 runs regenerated. This is not
  recorded in ACCEPTANCE_CRITERIA.
- `analyze_campaign.py` was committed after the campaign.
- The stages (raw_stageB*) used the same scenarios and seeds 0-2. The final campaign is a regression run, not an
  independent evaluation.

**B9. The statistics do not support the B0 vs B2 conclusion.**
- Engine noise is unseeded; even under CLEAN, the B0 and B2_BLACKOUT trajectories of the same seed differ. The pairing is
  nominal.
- B2_BLACKOUT vs B0 under MEDIUM-ALL gives 10 vs 3, about as large as the headline 13 vs 2 (p = 0.0074).
- The proper control, B2_MEDIUM vs B2_BLACKOUT, gives 7 vs 3, p ≈ 0.34.
- Three seeds per heterogeneous scenario, pooled Wilson intervals and 8 uncorrected tests complete the picture.

## C. Minor concerns

1. The documents contradict each other on B2: the acoustic exit is described as unavailable, while B2-M4g HOLDS for
   n = 2, 3 and there were 14 acoustic releases in B2_CLEAN.
2. The README says "186/186"; the summary contains 93 + 31 + 13 + 38 + 22 + 8 = 205.
3. M3's 42 s horizon is shorter than the slowest admissible crossing (n = 3: 51.7 s). A rerun at 65 s is still UNSAT
   (presentation only); the horizon should be justified or replaced by induction.
4. The "exhaustive numeric" checks (M4a-g, B2-A2's 162,000 cases, M4g at 0.1 m) are grid sweeps with no
   discretisation-error argument.
5. F6r is an arithmetic identity, not evidence for P3-deg.
6. After a hazard mode, `pass_continue` resumes the passage without re-checking occupancy. This is not modelled in M3.
7. M2s and M4f leave out-of-scope occupancies that no scenario exercises.
8. External validity: an idealised sensor (120° hard cone, range-independent detection up to 12 m, geometric ranges, no
   interference); three provenance contradictions left open (DVL age, MEDIUM range jitter against eps_range,
   `structure_tol_far_m`). The last plausibly drives the STRUCTURE misclassification of B3 (not verified).
9. The terminal monitor stops at SUCCESS, so behaviour after success is unobserved.
10. P2 is judged on hull centres; the hull dimensions should be stated.

## D. Potentially invalid or overstated claims

- **README "communication-free":** B2 is one-way communication.
- **REPORT.new §12 "P2 ... M3":** M3's premises are falsified in the logs.
- **GUARANTEES "no P2 violation":** true only for double occupancy, not for the authors' own release criterion.
- **REPORT.new §13 "P1 ... CLEAN and LOW inside the bounds":** only the cooperative pairwise abstraction is proved.
- **"No deadlock":** M2 is a static snapshot; there are 20 gate_not_completed runs and 17 relocalization holds.
- **"P3-deg every healthy drone declares the slot vacant":** this held in 1/9 runs (B0 MEDIUM) and 0/3 (B2_BLACKOUT
  MEDIUM-ALL).
- **Zero LIVENESS_FAILURE.**
- **The B2 benefit at p = 0.0074.**

## E. Missing experiments or analyses

1. Adversarial P2 stress tests: an occupant held in FAILSAFE or SW inside the CR for more than t_occ_max; a drone parked
   beyond the exit; clutter beyond the gate; MEDIUM dropout during the corridor exposure; a queue point closer to the CR.
2. P1 at the boundary: one-sided reaction, a faster intruder, multiple simultaneous threats, the S2c pairs without a
   guarantee.
3. A held-out scenario and seed set (gate geometry, queue sizes, currents, fault timing).
4. Run-to-run variance for identical configurations, and a B0 ≡ B2_BLACKOUT trace-equivalence test.
5. External baselines: a reservation or token protocol, and a CBF or velocity-obstacle avoidance layer.
6. An N-drone timed model of queue and latch (e.g. UPPAAL) with missed detections, spurious echoes and interruptions, plus
   conformance against the logs.
7. Gate throughput and latency.

## F. Required revisions

1. Enforce acceptance criterion 3, reclassify the 7 runs and apply the stop-and-diagnose rule.
2. Make the witness sound by construction (track continuity or a range-rate association), or drop it from the safety
   argument, and add spurious-witness, missed-detection and stall nondeterminism to M3.
3. State the P1 assumptions in GUARANTEES (mutual detection, both drones braking, the caps, a_brake); prove or test the
   one-sided case, or exclude the non-cooperative vehicle from the claim.
4. Resolve the conflict between M3 and FAILSAFE/CA/SW inside the CR: forbid them, or bound their duration and prove it.
5. Make SAFE_DEGRADED causal and count relocalization holds as liveness failures.
6. Use absolute, scenario-specified requirements and regenerate the references at the campaign commit.
7. Rerun at one commit, record the deviations, add held-out scenarios, seeds and repetitions, and compare B2 with
   B2_BLACKOUT using a multiplicity correction.
8. Fix the documents (C1, C2, the README) and fill REPORT §14-22 with sized claims.
9. Relabel the "exhaustive numeric" checks, or add discretisation bounds.

## G. Property verdicts

| Property | Verdict | Basis |
|---|---|---|
| P1 | proved only for the abstraction | cooperative pairwise model, randomized S0, the caps, a_brake, mutual detection; not established for the N-drone implementation; empirically supported with large margins, boundary untested; MEDIUM tested only |
| P2 | snapshot precedence proved (M1/M2); latch proved only for an unfaithful model | M3's witness, detection and crossing premises are contradicted by the logs; empirically no double occupancy in 195 gate runs, owing to unmodelled margins; violated under the authors' release criterion in 7 runs |
| P3 | proved for the abstraction | ranking lemmas on the estimated slot error under A1-A5, one of which is no FAILSAFE; the gap between estimate and truth is not covered; often not reached under MEDIUM |
| P3-deg | not established under degraded sensing | F6 only proves that a counter reaches its threshold; the premise that false sightings do not reset the timer is assumed and empirically violated; evidence from one scenario |

## H. Related work (verified by the reviewer)

- Henzinger, LICS 1996, doi:10.1109/LICS.1996.561342.
- Tomlin, Pappas & Sastry, IEEE TAC 43(4) 1998, doi:10.1109/9.664154.
- Loos, Platzer & Nistor, FM 2011, doi:10.1007/978-3-642-21437-0_6 (verified distributed car platoon of arbitrary size:
  the N-agent composition missing here).
- Mitsch, Ghorbal, Vogelbacher & Platzer, IJRR 36(12) 2017, doi:10.1177/0278364917733549 (passive and passive-friendly
  safety under sensor uncertainty).
- Wang, Ames & Egerstedt, IEEE T-RO 33(3) 2017, doi:10.1109/TRO.2017.2659727 (multi-robot barrier certificates).
- Eustice, Singh & Whitcomb, JFR 28(1) 2011, doi:10.1002/rob.20365.
- Rypkema, Schmidt & Fischell, Field Robotics 2 2022, doi:10.55417/fr.2022026.
- Potokar et al., ICRA 2022, doi:10.1109/ICRA46639.2022.9812353.

**Novelty.** The new elements are the touch-set distance bound, the sonar occupancy-latch protocol, and the
transparent provenance and mutation infrastructure. The formal techniques are standard and applied to small, separately
built slices.

## I. Recommendation

**Major revision.** It is not acceptable for T-RO or Autonomous Robots as it stands. Acceptance requires F1-F7. If P2
cannot be restated as a guarantee of the implementation, the work should be repositioned as an empirically evaluated,
formally motivated architecture rather than a verified one.
