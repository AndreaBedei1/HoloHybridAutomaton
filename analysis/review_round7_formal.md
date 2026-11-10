# Review round 7: formal verification, the undo of unconfirmed corrections

Commit `9e9cade` on `feature/b2-medium-robustness`, against `3e3db7b`. Independent reviewer, 2026-11-10. Code and results were not modified; no simulator started; no `formal/check_*.py` run; no underwater review opened. Not committed.
Tags: [O] observed (logs, real classes on crafted inputs, mutations on a private copy); [I] inferred. Scripts are in my own folder `r7_formal_only`.

**Verdict: I would not freeze on this commit yet.** The root cause of RC12 holds up against the logs, and on the logged cases the undo does what it is for. But the undo step feeds position-gated transitions on the next tick (N13, MAJOR), one of which drives a drone through the CR with no localization, and the GUARANTEES row is false as written. Fix N13, correct the row, and add the missing test of the s995 path. Then, subject to the targeted batch and the regression, I would freeze.

## 1. Root cause of s995

**It holds, with corrections to the note.** [O] From the state logs of `drone_2` against the referee truth:
* 8 `ENTRY_HOLD_TIMEOUT`, all `loc_ok=false`, last at 731.3 s. P1 3.46 m, P2 occupancy 1, DEADLINE. Holds at true `l` of 3.4-4.2 m, `s` about -1.5.
* The two fits, in the gate frame: at 77.3 s lateral error -0.57 to -1.70; at 79.4 s -1.67 to -2.76. Both `identified=true`. So 2.2 m, as stated.
* Replay of the logged echoes: at the first fit the fitted pose costs 0.820, the truth 0.843, the mirror of the truth 0.838, with the RIGHT echo unpredicted at the fitted pose and free. This is a near-tie, as the note says.

What the note omits or overstates:
* The first fit is 0.73 m from the mirror image, the second 0.14 m, so "stood at the mirror image" is exact only for the second.
* The lateral error was already -0.57 m before either fit, and three along-axis corrections (-0.26, -0.20, -0.55) took the `s` error to -0.76 m. The total to undo at the first timeout is 2.23 m, not only the two fits.
* Both fits came during COLLISION_AVOIDANCE, with the drone moving laterally.
* The identity rule was available only because the dead `drone_0` left the premise at 71.1 s (`perception.py:310`) [I, `others` is not logged]. Of 134 identified corrections, 107 occurred with a drone dead for 70 s or more, but the worsening is not concentrated there (mean change -0.003 m against +0.054 m without) [O].
* The statistic: I count 285 such corrections in 105 gate runs of med5, med6, med7, med9, 97 better, 80 worse by more than 0.05 m, mean +0.004 m; identified alone 62 better, 64 worse. "309, 151, 64, +0.02" does not reproduce, though the conclusion (no gain on average) does.

Would the undo have helped? [O] At the first timeout (102.5 s) the true error goes from (-0.76, -2.67) to (+0.06, -0.59). Over all 24 no-localization timeouts in those batches, 14 had something to undo: 10 improved, including all 8 of s995; 4 worsened, each by 0.21 m or less. Closed-loop recovery is not verified.

## 2. Soundness

**N13, MAJOR. The step feeds position-gated transitions on the next tick.** `control/controller.py:309,157`, `control/flows.py:340,356`, `perception/perception.py:323`. The step is not bounded by `MAX_CORR_M` (reconstructed from the logs: 2.2 m at the first timeout of s995, 2.9 m if it were never undone) and is applied after `flows.gate` and the gate report of that tick (`controller.py:188,221,225`). Real classes:
* Forward. [O] Corrections had moved the estimate back 1.5 m; at the hold point the timeout fires, the undo puts `s` at +0.34, next tick `past_centre=True`, no hold, `v_along=+0.49`, `PAST_CENTRE` logged. A drone with no valid localization then goes through the CR with stage 2 and the give-up off. If the corrections were right, the true `s` is -1.16.
* Back. [O] Corrections had moved it forward 1.2 m; the undo gives `s=-2.36`, behind the give-up plane, without moving. Next tick `abort_request=True`, then give-up, then (0, GP_NONE), since the estimate is outside the box. If the true position were in the CR, its claim is gone [I].
* Same tick. [O] Timeout with `s` already behind the plane: `abort_request` is computed before the undo and read by the automaton on the next tick, when the estimate is -0.79. `GUARANTEES.md:41` ("only behind the give-up plane") does not hold on that tick.
* `cr_cleared` is safe: the undo does not touch `loc_t`, and admission still needs `loc_ok` [O, reading].
* Reach is narrow. [O] Of the 14: none would latch `past_centre` (closest +0.11 against 0.20, at the eighth timeout of s995, which the undo would have reset earlier), one lands behind the plane by 0.02 m. Latching needs an along-axis undo of about 1.38 m from the hold point.
* Resolve: keep the undo from carrying the estimate through a gate transition, by clipping its along-axis part to stay between the give-up plane and the hold plane, or by blocking the `past_centre` latch once `hold_timeout` is set; and clear `abort_request` when an undo is applied.

**Estimator: sound where it can be checked.**
* NavBound stays valid and monotone. The new error is at most the old error plus the step, and `corrected` adds the step. [I] `|step| <= aid <= horizontal`, so the undo cannot fire the integrity flag in a run.
* Stored fixes and the pose history move with the estimate [O], which keeps the frame consistent.
* Not shifted: `AcousticExit.sighting` [O] and `flows.hold_pos` [I]. Pre-existing for fits of up to 1.2 m; now up to 2.9 m. MINOR.
* `unconfirmed` is not cleared by a measured crossing (`_apply_crossing`) [O]. Multi-gate only. MINOR.

**State after an undo: a hole in principle, closed by the data so far.** No property models a step of the estimate. S9, B2-PE, the give-up plane, the `past_centre` threshold and S2's offset all assume the gate-frame error is within `nav_err_m`, which the undo does not re-establish; NavBound grows but none of these read it. [O] In the large cases the undo restores that premise (0.6 m). An undo of correct corrections would not, and nothing detects that.

## 3. GUARANTEES and other documents

* **Row at `GUARANTEES.md:81`: false as written.** "Separation uses the raw ranges": true of the distance bound, but which echoes are targets depends on the pose, as row 74 itself states and round 3 showed (a hull echo relabelled STRUCTURE). "Admission needs a validated fit": true of `cr_cleared`, not of the `past_centre` path (N13). So "neither rests on these corrections being right" must go or be narrowed.
* `design note:117` and `config.py:356` ("by at most `MAX_CORR_M`"): false, s995 reached 2.2 m in two fits.
* `structure_aiding.py` docstring ("goes back to what dead reckoning says"): it goes back to the last validated fit.
* `GUARANTEES.md:41`: needs the same-tick exception, or the fix.

## 4. The new tests

**Useful but they miss the path that failed.** [O] 11 mutations on a private copy: 6 killed, 5 survive.
* Survivors: M1 identity-rule corrections not accumulated (`structure_aiding.py:393`), the s995 path itself; M5 a safe correction does not reset; M6 fit buffers kept; M9 undo does not shift the fixes; M11 undo only if `entry_ok`.
* The bound assertion in test 3 passes through `inf`: with `unconfirmed` set by hand the step exceeds `horizontal`, integrity fires and the value is infinite, a state a run does not reach [O]. It detects a missing `corrected` call (M8 is killed), but not a wrong amount.
* Essential and untested: an identity-rule correction through to its undo; the next tick after an undo (N13); the case `entry_ok=false`.

## 5. Freeze

Must be fixed first:
1. **MAJOR, N13**: the forward latch, the backward give-up and the stale `abort_request`.
2. **MINOR, required before the tag because they are claims**: the row at line 81, `GUARANTEES.md:41`, `design note:117`, `config.py:356`.
3. **MINOR, tests**: the identity-rule path, the post-undo tick, `entry_ok=false`.

Can wait, declared: `sighting` and `hold_pos` not shifted, `unconfirmed` across a crossing, the RC12 statistic restated. Also regenerate `formal/results` at the tag from a clean tree: the working copy shows modified result files, not mine.

Not verified: any simulator run or closed-loop recovery after an undo; the targeted batch and the regression; the Z3 rows (no solver model changed in this diff).
