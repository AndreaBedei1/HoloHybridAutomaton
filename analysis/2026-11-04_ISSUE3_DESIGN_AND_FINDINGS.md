# Issue 3: verified findings and next implementation

**State:** investigation/design only. No gate, witness, formation, automaton or acceptance behavior was changed in
this session. Infrastructure commits: `51440d2`, `db20d11`, both authored by Andrea Bedei and pushed.

The user's latest steering removes historical infrastructure root-cause reproduction as a blocker: two actual
workers completed successfully, so continue with the substantive issues. Keep every process/property/Git constraint.

## Baseline in progress

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
