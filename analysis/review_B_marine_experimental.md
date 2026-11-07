# Independent review B (marine robotics / experimental methodology)

The reviewer worked from the repository only, without the conclusions of the analysis team, at HEAD a6efd0f, in
read-only mode. The scratch files are in the session scratchpad (`reviewB`). The review is reproduced here and slightly
condensed; the substance is unchanged.

**Claims of the review already verified by the analysis team on 2026-11-03:**
- **B1:** `robustness_metrics.py:336-342` turns any sensor-fault FAILSAFE entry into SAFE_DEGRADED.
- **B3:** the ACCEPTANCE_CRITERIA freeze is at 1dc9d3f (29 Oct 23:09). ad4fb5e (30 Oct 00:52) came after it. The final
  phase reports "58 done, 566 to do".
- **B9:** the exceedance of +eps_far is, as a mean (maximum) per run:

  | Profile | Mean | Maximum |
  |---|---|---|
  | LOW-ALL | 0.86 % | 8.4 % |
  | MEDIUM-SUBSET | 1.8 % | — |
  | MEDIUM-ALL | 3.6 % | 15.6 % |

  The maximum data age is 0.3 s under LOW and 0.4 s under MEDIUM. The false-DYNAMIC fraction under MEDIUM-ALL has a mean
  of 2.7 %.
- **C1:** `at_queue` is logged both as a bool and as the string "False", so the string test is buggy.

**Not yet verified:**
- the B0 vs BLACKOUT McNemar recomputation (7 vs 3, p = 0.34);
- the per-cell stall counts (B0 CLEAN 1/15 vs B2_CLEAN 12/15);
- the chattering statistics.

## A. Summary

The work presents a leaderless fleet of simulated BlueROV2s in HoloOcean 2.3 with a sonar-octree patch:
- a 10-mode hybrid automaton whose guards are checked with Z3 (determinism, completeness, priority) and used at run
  time;
- six wide-beam (120°) single-beam sonars plus DVL/compass dead reckoning;
- an association-free conservative distance bound for P1;
- a sonar occupancy latch with an exit witness and a timeout fallback for P2 at a 2×2 m gate;
- formation keeping from a shared time-parameterised plan, with vacancy after t_rejoin for P3 and P3-deg.

B2 adds a one-way TDMA identity ping (one-way travel time plus inverted USBL), used only for formation association and
to block or permit latch releases.

The evaluation has 624 runs (13 cells × 16 scenarios × 3 seeds) and reports 0 safety failures, 0 liveness failures and
64 SAFE_DEGRADED runs, plus B0/B2 McNemar tests. The provenance documentation is unusually candid.

## B. Major concerns

**B1. "Liveness failure" is practically unreachable.**
- A run that misses requirements is SAFE_DEGRADED if any healthy drone entered sensor-fault FAILSAFE at any time
  (`robustness_metrics.py:336-342`). No causal link to the missed requirement is required.
- Such entries occur in 47-48 of 48 runs of every MEDIUM cell, and in 293 of the 560 PASS runs.
- All 64 non-PASS runs are SAFE_DEGRADED; in 55 of them the only conservative event is such an entry.
- The defect was already named in the diagnosis (`results/b2/diagnosis/REPORT.md`, C6) and kept.
- The missed requirements are classic liveness failures: P3 not recovered (25 runs), gate not completed (20), P3-deg
  not re-formed (10).

**B2. Pass criteria are circular and lenient.**
- The requirements are those of the scenario's committed CLEAN reference. P3 is required only where the reference
  achieved it: lost_drone_mutex can never fail on P3.
- The references record no code version and were committed at e759156, before the C1/C3/C5 fixes.
- Degraded deadlines are 1.5·T_clean(B0 CLEAN) + 20 s.
- All CLEAN runs, B2 included, only need to succeed within 3·T_ref + 30 s. For gate_single, T_ref = 80 s gives a
  horizon of 270 s, against a mean B0 CLEAN completion of 142 s.
- "CLEAN 48/48" is therefore close to true by construction.

**B3. The B2 algorithm changed after the freeze and after part of the campaign had run.**
- Stage C2 ran B2 runs at c86052f. ad4fb5e then added the acoustic exit by range.
- Stage E restarted with "58 done, 566 to do"; the earlier B2 runs were discarded unreported.
- ACCEPTANCE_CRITERIA.md has not been updated. Several documents still say the acoustic exit is unavailable:
  ACOUSTIC_CHANNEL_MODEL §9, GUARANTEES (B2), ACCEPTANCE_CRITERIA §3, REPORT.new.md §13.
- In fact there were 41 acoustic releases, and formal reports B2-M4g HOLDS for n = 2 and 3.

**B4. The fixes were designed on the test set.** C1-C5 were derived from an earlier campaign on the same 16 scenarios
and seeds 0-2, and the final campaign reuses them. There is no held-out scenario, geometry or seed set.

**B5. The statistics do not support the B2 benefit.**
- 3 seeds per scenario; the pooled pass rate is clustered by scenario.
- Engine noise is unseeded, so "paired" McNemar overstates the pairing.
- p = 0.0074 does not survive a Bonferroni correction over 8 tests (threshold 0.00625).
- The proper control, B2_MEDIUM vs B2_BLACKOUT, gives 7 vs 3, p = 0.34.
- B2_BLACKOUT vs B0, claimed equivalent, gives 10 vs 3 (40 vs 33 PASS): either BLACKOUT ≠ B0 or the noise floor is as
  large as the claimed effect.
- lost_drone_timeout under MEDIUM (B2_MEDIUM 7/9 vs BLACKOUT 0/3 vs B0 1/9) is suggestive only.

**B6. B2 causes stalls hidden by the classification.**
- Gate runs with a stall of 20 s or more (queue waiting, CR empty, nobody passing): B0 CLEAN 1/15, B2_CLEAN 12/15.
- Example: in B2_CLEAN/CLEAN/lost_drone_mutex/s0, drone 2 holds BUSY from about 30 s to 108 s while drone 1 waits
  about 6 m beyond the gate. The cause is the t_occ_max fallback blocked by `heard_inside`, with an envelope sized for
  MEDIUM including a 3 m multipath excess.
- Mean gate completion, B0 CLEAN vs B2_CLEAN: lost_drone_mutex 80 vs 130 s, line_parallel_mutex 86 vs 121 s,
  integrated_short 99 vs 124 s. These runs pass only because of the generous horizon.

**B7. Premature exit witnesses and a fault-unsafe timeout.**
- Seven SONAR witnesses fired while the passer was still in the CR (FRONT sector, 7.7-10.6 m; s = 0.32-0.76 m against
  an exit face at 0.8 m; it stayed 0.1-1.0 s longer).
- No violation occurred because EXITING blocks entry and t_clear = 1 s. In the worst case t_clear was used up.
- Acceptance criterion 3 defines this event as a safety failure, but the classifier ignores it.
- Most releases use the 25 s timeout, which assumes a healthy occupant; no fault inside the CR is tested.

**B8. The sonar is an idealisation of a sensor that does not exist, and P1 rests on it.**
- No single-beam sonar wider than about 25-30° exists.
- HoloOcean's cone is hard-edged with uniform sensitivity and full detection up to RangeMax.
- There is no inter-transducer interference (36 co-located 10 Hz sonars plus the pinger).
- Assumption (D) in `distance_bound.py` (no echo means no object in the cone) uses negative evidence that real beam
  patterns, fading and crosstalk break. S0 validates consistency with the simulator only.

**B9. The echo envelope is exceeded more than stated.**
- The share beyond +eps_far is 0.02 % in CLEAN, 0.42 % in LOW-ALL and 2.8 % in MEDIUM-ALL (plus 3.7 % below
  -eps_near). The analysis team's per-run means are 0.86 % in LOW-ALL and 3.6 % in MEDIUM-ALL.
- The maximum age is 0.3 s under LOW, above tau_max.
- The audit cannot see errors beyond 0.6 m.
- "P1 proved under CLEAN/LOW" is therefore not supported.

**B10. The acoustic channel is reasonable but optimistic.**
- Multipath excess is uniform over 0.3-3 m regardless of geometry.
- Latency is constant.
- There is no SNR, noise or interference model.
- The DOA bias depends on azimuth only.
- The clock offset is constant per run (CSAC-class only).
- The residual-risk analysis was not re-run after the envelope was widened.
- The hardware is research-grade and its 2-20 m performance between moving vehicles is undemonstrated.

## C. Minor concerns

1. The `at_queue` string bug: `not gs.get("at_queue")` never skips, so false_busy and false_free are computed over the
   whole run.
2. The "expected" column of the reported and availability formal rows appears to follow the observed verdicts; for
   example, the B2-M4g domain boundary at n = 4 is HOLDS/HOLDS.
3. Chattering is not reported: under MEDIUM-ALL a mean of 33 events per run, maximum 180.
4. Navigation model gaps:
   - the compass bias has fixed magnitude and does not depend on heading;
   - there is no DVL scale factor;
   - the gate map is exact;
   - MEDIUM navigation error reaches 1.66 m, beyond MAX_CORR = 1.2 m.
5. The P1 scenarios are mild: closing speeds ≤ 0.64 m/s, minimum separation 2.26 m against 1.0 m, so the boundary is
   never exercised.
6. There is one gate geometry and at most 3 drones per gate.
7. Only binary classes are reported, no completion-time distributions; recovery times reach 160-180 s even in CLEAN.
8. B0 CLEAN (1a14872) and B2 (ad4fb5e) come from different commits.
9. The Rypkema figures have not been checked against the journal PDF.

## D. Potentially invalid or overstated claims

- **README:3-6, "communication-free … no inter-drone communication".** Wrong for B2: an identity-coded TDMA ping is a
  one-way broadcast.
- **REPORT.new.md §2, "With a silent channel, B2 is B0".** Contradicted by 40 vs 33 PASS.
- **REPORT.new.md §13 and ACCEPTANCE_CRITERIA §3, "CLEAN and LOW inside the bounds; MEDIUM about 1.2 %".**
  Contradicted by the audit.
- **"No release while occupied" as a safety criterion.** The seven such runs are classed PASS.
- **"Acoustic exit not available".** Outdated.
- **"B2 only adds caution".** It hides stalls of up to 68 s.
- **P1 "proof".** It rests on the hard-cone assumption (D).
- **"0 LIVENESS_FAILURE".** An artefact of the classifier.

## E. Missing experiments

1. A held-out evaluation: new seeds, a second gate geometry, new spacings.
2. A realistic sonar model (soft edge, sidelobes, range- and aspect-dependent detection, crosstalk), or at least a P1
   sensitivity study to assumption (D).
3. An occupant fault inside the CR.
4. Experiments at the envelope boundary: closing speed, current, tau_max, faster intruders.
5. A B2 ablation against BLACKOUT with 10-20 seeds per scenario-cell and a stated effect size.
6. Distributions of completion, stall and recovery times.
7. A re-run of the reflection residual-risk analysis.
8. An explanation of the B0 vs BLACKOUT discrepancy.

## F. Required revisions

1. Make SAFE_DEGRADED causal, or report LIVENESS_FAILURE as it stands, and re-tabulate.
2. Use a priori requirements and deadlines with versioned references.
3. Record the ad4fb5e deviation and the discarded runs, and align the documents with the code.
4. Make the classifier and acceptance criterion 3 agree on false releases.
5. Correct the envelope-exceedance statements and the scope of P1.
6. Use the proper B2 control, a multiplicity correction and clustering (a mixed-effects model or per-scenario exact
   tests); drop the "paired" wording.
7. Fix the `at_queue` bug and report chattering and stalls.
8. Update the README; state the hardware requirements of both variants (B0 also needs a common clock for the plan).

## G. Property verdicts

| Property | Verdict | Basis |
|---|---|---|
| P1 | partially supported | 0 contacts, minimum 2.26 m, intruder minimum 1.95 m; formal S0-S4 conditional on (D) and on the eps bounds, which are exceeded; the boundary is never exercised |
| P2 | partially supported | maximum occupancy 1; n ≤ 3; one gate; seven premature witnesses; timeout safety assumes a healthy occupant; B2 stalls |
| P3 | partially supported | recovered in CLEAN/LOW; about 40 MEDIUM runs not recovered and reclassified SAFE_DEGRADED; required only where the old reference achieved it; F1-F5 concern an abstraction |
| P3-deg | B0: not supported under MEDIUM; B2: partially supported | B0 1/9; BLACKOUT 0/3; B2_MEDIUM 7/9; one scenario only; F6 is essentially a timer argument |

## H. Related work (verified by the reviewer)

- Fiorelli et al. 2006, IEEE JOE, doi:10.1109/JOE.2006.880429.
- Leonard et al. 2010, J. Field Robotics, doi:10.1002/rob.20366.
- Ghabcheloo et al. 2009, SIAM J. Control Optim., doi:10.1137/060678993.
- Eustice, Singh, Whitcomb 2011, J. Field Robotics, doi:10.1002/rob.20365.
- Webster et al. 2012, IJRR, doi:10.1177/0278364912446166.
- Rypkema, Schmidt, Fischell 2022, Field Robotics, doi:10.55417/fr.2022026: one-way range-and-angle relative
  navigation, essentially the B2 sensing concept.
- Bahr, Leonard, Fallon 2009, IJRR, doi:10.1177/0278364908100561.
- Paull et al. 2014, IEEE JOE, doi:10.1109/JOE.2013.2278891.
- Stojanovic & Preisig 2009, IEEE Commun. Mag., doi:10.1109/MCOM.2009.4752682.
- Platzer & Clarke 2009, FM, doi:10.1007/978-3-642-05089-3_35.
- Potokar et al. 2022, ICRA, doi:10.1109/ICRA46639.2022.9812353.
- Chitre 2008 (doi:10.4031/002533208786861263) was taken from references.bib and not independently checked.

**Novelty.** Three elements are new:
1. the association-free touch-set bound;
2. sonar-only gate mutual exclusion through the latch;
3. SMT-checked guards integrated with a large campaign.

Items 1 and 2 are tied to a non-existent sensor. B2 adds no sensing novelty beyond Rypkema et al.; its latch use is new
but of unproven benefit.

## I. Recommendation

**Major revision.** It is not acceptable for IEEE JOE or Field Robotics as it stands.

- **Strengths:** careful engineering; the run-time automaton is the formally checked one; mutation tests; ground-truth
  isolation; full provenance; a candid parameter audit.
- **Weaknesses:**
  - the evaluation design favours the implementation;
  - the classifier cannot output liveness failures;
  - the fixes were tuned on the test set;
  - there was an undocumented post-freeze change;
  - the B2 benefit does not survive the proper control;
  - B2 causes hidden stalls;
  - P1 and P2 rest on an idealised sonar;
  - several documents contradict the evaluated code.
