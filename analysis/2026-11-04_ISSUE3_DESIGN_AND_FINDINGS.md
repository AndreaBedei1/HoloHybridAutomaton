# Issue 3: verified findings and next implementation

**State at the 14:00 resumption:** baseline complete; an uncommitted Issue 3 behavior candidate is under review and
is **not accepted**. No acceptance criterion or P1/P2/P3 definition has changed. Infrastructure commits `51440d2`
and `db20d11` are authored by Andrea Bedei and pushed. The investigation record below predates implementation;
the resumed-session findings at the end supersede its in-progress status.

The user's latest steering removes historical infrastructure root-cause reproduction as a blocker: two actual
workers completed successfully, so continue with the substantive issues. Keep every process/property/Git constraint.

## Baseline (completed before resumption)

Two supervised Git Bash sessions were started on 2026-11-04 at 10:42 Europe/Rome:

```
HOLO_FLEET_ENGINE_SLOTS=2 bash scripts/run_dev_diagnostics.sh results/robustness/raw_dev/before_issue3 0 2 5
HOLO_FLEET_ENGINE_SLOTS=2 bash scripts/run_dev_diagnostics.sh results/robustness/raw_dev/before_issue3 1 2 5
```

Tool session IDs at launch: 49007 and 99345. Inspect the real process tree and current slot metadata after resuming;
never act on an old PID alone. Per-repetition raw roots are `r0`..`r4`, 11 cases each, 55 runs total. The case list
starts with mutex_deadlock_resolution, integrated_short and gate_single MEDIUM-SUBSET, then the other original cases.
Two slot files enforce the project cap. No full/formal suite may run concurrently with these simulations.

Initial results include a STALLED integrated_short repetition and subsequent PASS repetitions, all observed P1/P2
held and worker exits 0. Preserve failures; these are development observations, not frozen results. Some runs record
51440d2 and later ones db20d11: the intervening change preserves invalid-attempt evidence and declares psutil; the
controller/perception code is identical. Do not change production behavior until the required before evidence is secured.

Completed result: 55 runs, 46 PASS, five STALLED, four INCOMPLETE; every worker exit 0, all P1/P2 held.
`results/b2/consolidation/before_issue3/{SUMMARY.md,runs.csv}` preserves the aggregate. One baseline run has E3 and
E2.to_free despite a PASS label: the pre-existing classifier omits these unsafe-action disjuncts. This is a required
referee fix, not valid safety evidence or permission to change criteria.

## Independent review, verified against the code

- `GatePerception` never sets FREE to BUSY from heard_inside, so a masked occupant can permit E3 even while stage 2
  prevents E4. `can_go` uses at_queue, !occ_busy and has_prio.
- Sonar witnesses bypass heard_inside. EXITING can also become FREE even when positive occupancy reappears.
- `ExitWitness.update` has no three-capture confirmation. It returns the first W1/W2/W3 candidate.
- Capture confirmation must count distinct capture times (`t - Target.age`), not callbacks or held EchoHold echoes.
  Range continuity must include observer sensor displacement, target speed and existing sonar errors. Do not require
  increasing range from a lateral/moving observer. Three captures improve evidence but do not prove identity.

## Executed in-envelope multipath counterexample

Reproduced without an engine, using the actual AcousticTracker/AcousticExit and deployed configuration:

- true observer gate s=-2.179; estimated s=-1.179 (declared 1 m navigation error);
- occupant centre s=0.8, l=dz=0 (inclusive CR boundary);
- true range 2.979 m, persistent +3 m excess, measured range 5.979 m, zero angle errors;
- three accepted fixes on track 0, latest age 0.25 s;
- `heard_inside=False`, while `clear_of_cr(latest)=False` under the complete region.

Artifact: `results/dev/issue3/multipath_counterexample.json`. The clear-path filtering removes exactly the nearer
part required to cover this allowed multipath bias. Reusing the complement of `clear_of_cr` is the reviewer's minimum
safe full-envelope veto correction; assess liveness honestly rather than weakening the envelope/CR/criteria.

An additional executed geometry sweep showed that a parked drone at rally_s=6 m can remain possibly inside under
the full deployed region (queue s=-5.406478684, lateral -3.5/0/+3.5); A2 by range becomes available farther beyond.
This is the safety/liveness ambiguity to evaluate, not permission to tune a threshold or lengthen a deadline.

## Bounds and stress coverage

- B2-V1 can hold only with an acquired same-track pair, complete measurement/navigation/motion coverage, and bounded
  accepted-fix gaps. One consecutive lost ping: latest age <=2P+latency=2.4<fresh_s=2.5; previous age <=4P+latency=4.4
  within the 4.5 s search window. MEDIUM has 4 s outages and unbounded iid loss strings; BLACKOUT gives no completeness.
- A deterministic expiration bound after a true centre merely leaves the CR is impossible: repeated accepted fixes
  arbitrarily close beyond the boundary renew possible occupancy forever. Bound expiration after the last qualifying
  evidence, or assume robust clearance and bounded delivery. Report the unconditional claim as unmet; do not alter P2.
- A pre-existing stalled occupant plus an E7 follower hold necessarily has an earlier E3 commit if the occupant never
  left. Correct FREE->BUSY integration should prevent that commit and give E7=0. E7>0 with E3=0 requires occupancy
  after a lawful commitment, before stage 2. Build a justified late-occupancy stress from existing primitives, or bring
  any required criterion/property change to the user; never exempt E3 or silently relax E7.

## Next

1. Inspect/finish the baseline safely, without starting a third engine.
2. Write the Issue 3 design and failing tests for positive occupancy in FREE/EXITING, witness veto/confirmation,
   stale-capture rejection and the deployed-envelope counterexample; add the stalled-occupant scenario.
3. Implement the smallest defensible correction, keep acoustic data outside ExitWitness/separation/precedence,
   update B2-S and the formal obligations, quantify channel/veto gaps, and correct ACOUSTIC_CHANNEL_MODEL Section 9.
4. Independently review the actual diff, verify every finding, and run the required after repetitions.
5. Continue Issues 4, 1, 2, remaining referee items, formal rerun, freeze, final campaign/report, final --no-ff merge.

Independent methodological findings to verify before freeze are saved separately in
`analysis/2026-11-04_CRITERIA_REVIEW.md`. No acceptance value or formula was changed.

## Resumed-session candidate and evidence

The current uncommitted candidate uses the complete `clear_of_cr` region for positive occupancy, sets FREE/EXITING
to BUSY while heard, vetoes sonar releases and stage-2 admission, and keeps acoustic-only episodes separate from the
sonar sighting clock. Sonar exit evidence now requires three distinct consecutive captures with compatible range
intervals, including observer displacement and existing range/hull/motion bounds. Per-sector capture ages are retained
through target grouping; changing the oldest member age cannot counterfeit captures. Acoustic data remains outside
the sonar witness and separation/precedence.

Independent reviews identified and the author verified:

- Grouped target `age=max(member ages)` could counterfeit captures as older members expired. Preserve `Target.ages`
  and use each witness sector's own age. Two added regressions failed before this repair and passed afterward.
- An acoustic-only BUSY episode had no positive-clearance release path without a corridor sighting. A regression
  reproduced BUSY despite three A2 confirmations for every other drone. The candidate now has an A2-only release
  for that episode, without fabricating a sonar sighting; unknown identities still cannot be inferred outside.
- Selecting the nearest candidate selected EchoHold's old nearer capture while a real hull receded; confirmation
  never reached three. A real EchoHold/build_targets regression failed on that implementation. Select the newest
  eligible capture, then the nearest range at that timestamp. Keep held echoes unchanged for separation.
- Full-envelope ambiguity can persist outside indefinitely. The following counterexample was executed independently
  and reproduced by the author; it is a contract limitation, not a request to reduce uncertainty.

### Indistinguishable occupancy histories

Observer `(s,l,z)=(-4.6,0,-5)`, exact measured range 8.4 m, zero azimuth/elevation, a reception every second:

| Compatible world | True centre s | Persistent range excess | CR membership |
|---|---:|---:|---|
| inside | 0.8 m | +3 m | inside the inclusive boundary |
| outside | 3.8 m | 0 m | outside |

Both worlds produce identical accepted same-track fixes forever. Full-envelope completeness requires the veto in
the inside world, so the same onboard algorithm must veto in the outside world. No timer-only change can guarantee
expiration 4.5 s after true-centre exit while preserving that completeness. Artifact:
`results/dev/issue3/indistinguishable_occupancy.json` (31 accepted receptions, veto from t=1 through t=30).

This theoretical limit does not waive empirical acceptance. In the real candidate CLEAN gate_single s0 probe
(`results/robustness/raw_dev/issue3_full_region_probe/B2_CLEAN/CLEAN/gate_single/s0`), the first drone committed at
t=7.6 and held before the CR through the unchanged deadline t=254.3: STALLED, exit 0, P1 minimum 3.219 m, P2 max
occupancy 0, E2/E3/E4/E5/E7=0. The full region makes queued drones behind the approaching observer compatible with
the CR. This is a new practical failure and prevents accepting the candidate. Do not restore the unsound clear-path
filter, narrow the envelope, extend deadlines or relabel this result.

The simultaneous BLACKOUT attempt was INVALID because the listener guard refused port 1985. The startup race and
subsequent two-orchestrator validation are recorded in the infrastructure log. It supplies no BLACKOUT gate result.
The probe preceded the per-sector-age, newest-capture and acoustic-only A2 repairs. Its unchanged full-region
predicate is still the source of the demonstrated ambiguity; no after-run success is claimed for the current tree.

### Review and verification status

The latest independent capture/producer review found no further concrete defect in its code scope after the fixes.
The author also executed mutations removing the maximum capture gap and discarding the cumulative interval; the
two new tests reject their respective mutations. Theoretical contract and empirical CLEAN liveness remain open.

The full file-by-file candidate suite completed: 39 files, 464 passed, one native engine regression deselected,
no retries or failures (`results/dev/tests_issue3_candidate_20261104`). The latest focused recheck after the final
capture/gate edits completed 85 tests across exit_witness, gate_veto, acoustic_b2, gate_perception and b2_isolation.
The full suite's earlier exit_witness run had 21 tests; the focused rerun includes its two additional regressions.
This is unit/static evidence, not the required final formal rerun or after-simulation campaign.

### Decision required for the Issue 3 closure contract (proposal, not adopted)

Preserve P1/P2/P3, E3/E4/E7 definitions, all verdict priorities and empirical CLEAN/no-new-stall requirements.
Replace only the unattainable unconditional theoretical expiration claim with a conditional observability claim:
expiration after the last compatible reception, or after sufficient positively outside evidence; a bound from true
exit additionally needs a stated delivery and robust-clearance assumption. Report boundary ambiguity and longer
outages as residual limitations. This does not excuse the current CLEAN stall, which still needs a solution.

For a permanent occupant already inside before a follower commits, stage-2 HOLD implies that prior commit and hence
E3>0. Correct stage-1 protection instead has E7=0. Keep the metrics unchanged: use that case to demand safe stage-1
blocking; exercise E7>0 separately with occupation beginning after a lawful commitment. The scenario and its timing
must be specified before running it, without tuning to results.

The handoff's Section 10 explicitly asks for the unconditional bound and E7 coverage; Section 1.9/14 requires a stop
and report before changing property/acceptance requirements. Obtain the user's explicit choice about this closure
contract before treating Issue 3 as closed or proceeding to Issue 4. No such approval has been received yet.
An explicit question with these two alternatives was sent during the resumed session; elapsed time is not approval.
The startup fix is committed and pushed as `bce9194`. Save the Issue 3 candidate as a checkpoint, not an accepted
controller revision. Preserve `formal/results/mutex.json`'s pre-existing modification until the deliberate rerun.
