# Round-9 review, underwater: `176b8b8` against `de4cc5b`

What I read: the diff, `scope.py`, `acoustic_gate.py`, `acoustic.py`, `perception.py`, `p3.py`, `refine_v8`, P3_CONTRACT, GUARANTEES and the log. I re-evaluated the 210 runs of med11, med12, st1, st2 and reg2 with the committed referee. It reproduces 187 PASS, 8 PREMISE_VIOLATED, 6 P3_FAILURE and 7 INCOMPLETE; two st2 runs of the removed `st_bd_lost_cross_030` fail to load. I also read med13 and the state logs. Everything was read-only and no simulator ran. My scripts are in the session scratchpad, outside the repository. They ran at below-normal priority, including an 8-worker pool whose children inherit the priority class. Two sub-second JSON summaries ran at normal priority. I did not open the formal review. Tags: **[M]** measured, **[D]** derived by me.

## BLOCKER

**B1. Rule D is still unconditional in the running code, so round 8 B1 is not resolved.** [M]
- `AcousticExit.dead` (`acoustic_gate.py:197-200`) returns True when `tracker.fixes[j]` is empty. That branch is meant for "never heard".
- The tracker forgets every fix older than `_KEEP_S` = 30 s on each update (`acoustic.py:52`, `:214`). After `t_silent_gate_s` = 70 s the list is always empty, so a silent drone is declared dead wherever it was last heard. The zone check is never reached.
- Reproduction (scratch script): the three cases of `test_rule_d_releases_a_silent_claim_only_outside_the_zone`, with the tracker updated every second (drone 2 heard, as in the real pipeline). Results: a claim last heard **in the CR** is released (`claims = {}`), and so is one 6.5 m before the gate at 0.3 m/s. The unit test passes only because it never calls `update` after the last fix.
- In the logs: all 67 dead declarations in the failure runs of med12, med13 and st1 came at 70-80 s, whatever the position. The med12 note "rule D works" (`st_lost_at_queue`) and the safe-stop rows of GUARANTEES describe code that never runs.
- Fix:
  - use `last_accepted` (it is kept for ever), and test "never heard" as `j not in tracker.heard`;
  - add a test that drives `tracker.update` for at least 75 s.
- Three margin defects to fix at the same time:
  - (a) The depth of `f.p` comes from the acoustic elevation (box ±13.5°: ±0.8 m at 3.5 m, ±1.6 m at 7 m). The `scope.py` docstring says "the depth is measured", which is not true here. Under current, 1 m of depth error moves the reach by 1.1·w_h/0.045 ≈ 7 m at 0.3 m/s. Test the region samples (they carry depth) against E, or use the payload depth (depth sensor, 0.05 m).
  - (b) `t_lost_s` is counted from the last *accepted* fix, but the stated premise is the last *heard* reception. Add `v_bound·(heard[j] − f.t)`.
  - (c) The receiver's bound is the one at decision time; it can use `pose.bound_at(f.t)`.
- Effect of the fix [D]:
  - The real onboard margin is 6.9-10.3 m. It is made of the fix radius (3.5-4.7 m, mostly the one-sided 3 m multipath allowance, with the box clipped at the receiver), the receiver bound at decision time (2.4-4.9 m) and 0.7 m. `tests/test_scope.py:14` budgets 1.5 m.
  - I replayed the decision with the true onset positions. 25 of 67 declarations would then hold: all of `st_lost_at_queue` and `st_lost_committed` (already INCOMPLETE), and all of `st_lost_left_cross` and `st_lost_right_tail` at 0.2 m/s, which are PASS today.
  - So the failure-recovery envelope under current shrinks to failures beyond E plus about 8-10 m. The stress failure runs have to be repeated.

## MAJOR

**M1. The navigation premises turn liveness failures of the vehicle into PREMISE_VIOLATED.** [M]
- How the rule works: `refine_v8` (`robustness_metrics.py:516-521`) classes a failed gate as PREMISE_VIOLATED once the committed gate-frame error has exceeded 1 m. It is booked to the drone only if the **first** excess follows its own map fit within 30 s (`:581-586`).
- `med11 lost_drone_mutex_rise_high` s1103 is a MEDIUM-ALL standard scenario and goes from INCOMPLETE to PREMISE_VIOLATED:
  - its first excess is 1.002 m at 198.9 s, with no fit in the 30 s before;
  - so the 2.2-2.5 m excess that its own validated fit caused at 440 s (round 8 M1) is counted as external.
- `med12 MEDIUM_HT lost_drone_mutex` s1211 also goes from INCOMPLETE to PREMISE_VIOLATED. Its sensors are MEDIUM, and it spent 259 s above 1 m (up to 2.56 m).
- Both are the open compass-bias class. The dead reckoning and aiding of the vehicle produce this error under the declared sensor errors, so it is not an external premise. Two statements are refuted:
  - `P3_CONTRACT.md:54`: "a controller deadlock is never classed as an external infeasibility";
  - `:58`, which says the 8 PREMISE_VIOLATED are boundary probes or compass sensitivity. These two are neither.
- Premise N (absolute error > e_ok = 0.5 m) does not separate failures from successes. Under MEDIUM-ALL it is measured in 12 of 236 MET P3a windows and 17 of 112 MET P3b windows. Under MEDIUM_HB4 it is in 11 of 23 windows, 7 of them MET.
- N is also absolute, while the P3 error removes the mean translation (`p3._frame_dev`). A common drift does not affect the formation.
- Fix:
  - under CLEAN, LOW and MEDIUM, gate-frame navigation and N become tags only; the run stays INCOMPLETE or P3_FAILURE;
  - measure N on the error relative to the healthy drones' mean;
  - keep PREMISE_VIOLATED for W, O, A and V, and for profiles declared beyond the envelope (HB4, current beyond authority).
- The W probes are classed honestly: `st_bd_head_040` and `st_bd_oblique_040` are PREMISE_VIOLATED[W] at 0.40 m/s, beyond the 0.21 m/s head limit.

**M2. Round 8 monitor 2.iii is missing.** No monitor flags a *validated* fit that raises the true gate-frame error by more than 0.5 m, the s1103 mechanism. The cap at `structure_aiding.py:407` stops fits larger than 1 m. A wrong fit of 0.9 m still validates. Add it as a referee tag; it is offline only and cheap.

## MINOR (declare, or fix in passing)

- **m1. Rule A-P band** (`perception.py:376`, `:396`).
  - The band is e_lost + `fix_uncertainty` = 1.2 + 3.9-4.7 m, about 5-6 m at 3.5-7 m. The term is dominated by the one-sided multipath excess, which the radius makes symmetric.
  - The s2000 mechanism is a relative navigation error from opposite compass biases, not a fix error. The term widens the band, but not along the error that caused the problem.
  - The band also feeds `acoustic_residuals` (`:475`).
  - Measured [M]: with both drones in FORMATION modes away from the gate, a fix counts as present while the neighbour's true deviation exceeds 1.2 m in 5.5 % of cases (med12) and 5.9 % (med13); above 2.5 m, 2.3 % and 2.2 %. That is no measurable increase, and my slot reference is approximate. No safety path uses the band (P1 is sonar-only).
  - Better: intersect the region with the e_lost ball (as A-D does at `:426`), add the sender's reported bound, and keep the residual at e_lost.
- **m2. Physical numbers.**
  - `v_rise_min` 0.045 m/s is only 4 % below the model value of 0.047 m/s. With 33w + 190w², a net buoyancy of 0.15 kg instead of 0.2 kg gives 0.037 m/s, outside the model. Make a measured rise test a deployment requirement, with a margin of at least 20 %.
  - `coast_h` 1.0 m comes from 1.22 s × v, so it covers 0.82 m/s through the water. That is exactly 0.5 m/s plus a 0.3 m/s head current: no margin at the edge of the envelope, and measured in the simulator only.
  - The rise lag (2 s against τ ≈ 0.35 s), `coast_v` and the drift factor 1.1 (4 % measured) are credible for uniform current only.
  - Declared honestly.
- **m3. The declared current** is the scenario's true bound (`runner.py:93`). In the field it is an operator declaration, and exceeding it silently voids the rule D condition. State this.
- **m4. Floating-hull limit** (`st_lost_at_queue`, `st_lost_committed`, 0 of 6 complete). It is honestly declared, and after B1 these runs are safe stops anyway. A fix without acoustic data exists:
  - the receiver knows its own depth and `surface_z`;
  - an UP echo, or a LEFT one at high elevation, at a range of at least depth − surface − R_OUT − the depth error is at the surface, above every queue depth of the plan, so it is not a queued drone.
- **m5. Contract and code disagree.**
  - Premise A in the contract includes "no live sender silent"; `p3.py:155-159` only checks a deaf receiver.
  - Premise S for n = 2 ("none while committed") is not checked (`p3.py:172`).
- **m6. P3 from the vehicle side.** Tolerance 1 m, hold 8.9 s, v_close 0.10 m/s and the 5 s margin are credible for a BlueROV2 at 0.3 m/s. Recoveries took 0-49 s against deadlines of 24-77 s. A NOT_MET inside the envelope is possible (the certificate's rate is 0.014 m/s), and this is declared. In the field the contract can only be measured against an external reference (USBL or LBL to within 0.2 m); say so.

## Pre-registrable criteria (adjusted)

Frozen commit, engine hash recorded, held-out seeds, every run counted.

1. **Safety:** as in round 8. 0 SAFETY_FAILURE, UNSAFE_ACTION or SUT_ERROR in at least 300 runs; P1 at least 1.0 m; at most one drone in the CR; Clopper-Pearson bounds.
2. **Monitors:**
   - live declared dead = 0, with p50, p99 and max of the live silence (HOLD if above 35 s);
   - wrong validated fit = 0 (M2);
   - every failure outside E at the declared current;
   - **new:** a log of every rule D decision (released or held, margin, fix age, depth source). A release with the true onset inside E is 0.
3. **Liveness (CLEAN, LOW, MEDIUM):** PREMISE_VIOLATED[N or gate-nav] counts as a failure.
   - `lost_drone_mutex` family: at least 60 runs, at most 2 failures.
   - Standard set: at least 17 of 18 per profile.
   - Every failure is assigned to a declared class.
4. **P3:** in the MEDIUM and LOW runs with a passed gate, P3_FAILURE counts as a failure, under the same bound. Report the deadline slack distribution.
5. **Near the gate:**
   - queue and committed in still water: P1 and P2 required, a safe stop expected and declared;
   - after the exit, and n = 2: the mission is required;
   - **new safe-stop probe:** a failure inside E (in the CR at 0.2 m/s), at least 5 seeds, judged on P1, P2 and the gate held, not INVALID.
6. **Stress and sensitivity:**
   - stress as in round 8;
   - compass bias at 3 and 4 deg with at least 10 seeds each, and |e_l| at s = −3 against the bias;
   - MEDIUM_HT with at least 10 runs.

**Freeze: no.** Conditions:
- B1 fixed through the tracker, with a pipeline-level test, and margins (a)-(c) corrected; then the failure stress set and the `lost_drone_mutex` family re-run, and the envelope text of GUARANTEES and the scope restated with the onboard margin;
- M1: navigation premises no longer change the class under the validation profiles;
- M2: the monitor added.

Then yes, with criteria 1-6.
