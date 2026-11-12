# Round-7 review, underwater physics and acoustics: commit `9e9cade` against `3e3db7b`

Checked against the diff, the completed `med9` (`e034755`), `med5`-`med7`, `reg1`, and the trigger run `lost_drone_mutex` s995. RC12 was read as a claim, after measuring. Read-only; no simulator; scripts in my own folder `r7_underwater`. I did not open any formal review.
Tags: **[M]** measured from logs; **[D]** derived by me, including open-loop replays of the logged echoes through the project's `StructureAiding` and `EchoClassifier`; **[A]** assumption.

## Verdict

**Do not freeze this commit as it stands. Two small changes first (R1, R2), then the targeted batch and regression at the commit you freeze (R3).** No BLOCKER: safety held in all 68 `med9` runs and nothing here weakens the separation layer. If the owner will not change the code again, freezing `9e9cade` is defensible only with R1 and R2 declared as limits, and the report must not claim the s995 mechanism is fixed until R3 shows recovery.

## 1. The round-6 condition and s995

**Not met as I wrote it.** [M]
- Standard set: 53 of 54 PASS, one INCOMPLETE (s995). P1 at least 2.32 m, P2 holds everywhere, no SAFETY_FAILURE or UNSAFE_ACTION.
- `ENTRY_HOLD_TIMEOUT` in the 18 standard gate runs: **9 in `med9`** (s995: 8, s994: 1), against 0 in `med6`, 0 in `med5`, 0 in `med3`, 5 in `med4`, 3 in the 4 runs of `med7`. Whole batch 10 with `lost_drone_mutex_carried` s987. Every one has `loc_ok` False.
- The adrift cost stayed small: at most 13.8 s per standard gate run, longest episode 6.8 s.

**s995 is a new mechanism, not a new outcome class.** It is INCOMPLETE like s920, s960 and s981, in the same scenario family. It is the first where the fit itself caused the failure.

## 2. What produced the wrong corrections

**Measured on drone_2 at t = 76.7-79.4 s [M, D]:**
- Not `_pred`: at the true pose the model predicts FRONT, RIGHT, UP and DOWN within about 0.1-0.17 m of the echoes.
- Not heading: within 1.5 deg of the gate axis; compass bias -2 deg, about 0.07 m at 2 m.
- Not spurious echoes: the RIGHT echoes at 1.77-2.27 m are the real bar (predicted 1.79-2.16 m). One 1.32 m sample at 76.7 is the only stray.
- **The cost function.** `_score` (`structure_aiding.py:196`) gives a sector with an echo and no predicted bar a cost of 0, and every candidate pays the same cap for LEFT because it is in `must` (`:342`). Only LEFT and RIGHT break the left-right symmetry, and neither contributes. FRONT, UP and DOWN are symmetric about the axis. Cost at the true pose and at its mirror image agree to 0.001-0.03 in the replay. The candidate on the wrong side is preferred because it pays nothing for the real RIGHT echo: at t = 79.0, fit 0.811 against 0.836 at the true pose.
- Cone geometry is what makes this bite: at s = -1.7 m the bars are seen 50-65 deg off the side boresights, so a 0.5 m change in l brings a bar in or out of the 120 deg cone.
- "Mirrored" is only half right. The first correction (77.3 s) landed at l = -1.16 against a mirror image at -0.54. The second (79.4 s) landed at -1.31, the mirror of the true +1.46 by then. The drone had meanwhile been driven to the far side by the first error.

**Is the identity fit worth keeping? Yes, with less authority [M].** 128 identity fits in `med5`-`med7` and `med9`:

| Initial error | n | Better > 5 cm | Worse > 5 cm | Mean change |
|---|---|---|---|---|
| below 0.2 m | 25 | 1 | 22 | +0.43 m |
| 0.2-0.4 | 48 | 23 | 22 | +0.12 |
| 0.4-0.7 | 29 | 16 | 11 | 0.00 |
| 0.7 and above | 26 | 20 | 6 | -0.46 |

They rescue a drone with a large error and damage one with a small error. Harm is concentrated at the queue line (92 fits, mean +0.06), including a repeated false jump of about 0.9 m towards the axis at the l = -3.5 m queue point (9-11 cases, l estimate going to about -2.6 with truth near -3.5), and about 40 repeating flips of 0.5-1.0 m in `lost_drone_mutex_carried` s988, where the dead hull is the likely source. A step cap is better supported than removal [D, one-step replay]: at most 0.5 m per correction gives 44 worse and 75 better, mean -0.065, worst +0.5 m, against 61 worse and 60 better, mean +0.03, worst +1.09 m uncapped. The free-pass filter is not supported either: I count 19 fits with one free sector and 2 with two; those with one or more were better more often than worse (14 better, 7 worse), and I could not reproduce the note's 48 % against 35 %.

## 3. The undo and the reported average

**Physically weak in its premise, useful in practice.** A hold that ends without localization does not refute the corrections. Absence of validation is not evidence against them, the same reasoning as silence. s960 is the counter-example: the fit is right and still unvalidated (section 4). As a recovery heuristic it is fine.
- Replay of all 24 logged timeouts with `loc_ok` False [D]: the undo matters once, s995 first timeout, error 2.77 m to 0.46 m. In the other 23 it moves the error by at most 0.07 m, never worse by more than 5 cm.
- It makes the estimate worse when the sum contains corrections of real drift made during a long wait [D]. Undoing at an arbitrary moment at the queue would worsen the error by more than 0.1 m in 33 of 86 drone-runs, worst +1.18 m (`med7` s981, a fit that had removed 1.15 m), typically +0.1-0.4 m. No logged timeout has hit this yet, but the sum is reset only by a validated fit, which cannot occur at an off-axis queue point (clearance below 0 at |l| = 3.5 m).
- After the undo the drone is back in the state that preceded the failure: 0.46 m lateral error at the hold, the same precursor as s960. Recovery is not shown; the corrections could recur. Only R3 can settle it.

**The reported average is not reproduced and hides the point [M].** I count 278 corrections of these two kinds, not 309: 153 better, 79 worse by more than 5 cm, mean +0.004 m (note: 151, 64, +0.02). The sign is the same, but the two kinds differ: axial n = 150, mean -0.02, worst +0.13 m; identity n = 128, mean +0.03, worst +1.09 m. Validated fits, for comparison: n = 142, mean -0.22, worst +0.13 m. An average near zero is the balance of rescuing large errors and damaging small ones, so it is not a measure of worth.

## 4. s960

**Different mechanism, same precursor and same end state.** [M]
- In s960 no identity fit was accepted for drone_1. The fit is good and refused: 1500 hold steps with `fit_ok`, clearance below 0, `identified` False. In s995 two identity fits were accepted and wrong. The rule's acceptance does not track correctness in either direction.
- Both reach the hold with about 0.55 m of lateral dead-reckoning error (s960 +0.54 at 40 s, s995 -0.57 at 76 s), and both loop to the deadline (9 and 8 timeouts).
- Note corrections: "0.26 m of corrections applied" should be 1.09 m in 5 axial steps (lateral 0); "lateral error 0.5 m from the start" grows from 0.04 m at t = 2 to 0.51 m at t = 30.
- The undo does nothing here (sum at most 0.27 m), as the note says.

## 5. What must be fixed first

- **R1. MAJOR. Limit the undo to corrections made since the last commit** (`structure_aiding.py:237-243`, `controller.py:305-312`). Reset the sum on the commit edge. The s995 repair is unchanged in replay (2.37 m undone, 2.77 to 0.46 m), and the counterfactual exposure falls from 33 of 86 drone-runs to 2 [D]. The worst case stays at about +1.1 m: the `med7` s981 fit that removed 1.15 m was made inside a commit, so a later timeout in that same commit would still undo it. One line, and declare the residual.
- **R2. MAJOR. Cap the step of an identity fit** at about 0.5 m (`structure_aiding.py:393`). It keeps a wrong fit within `MAX_CORR_M` of any later one, which is the actual cause of "beyond the reach", and it removes the fat tail. Declare it as a limit if not done.
- **R3. MAJOR, a test requirement.** The targeted batch must include `lost_drone_mutex` at MEDIUM with enough seeds to show completion, not only that `UNCONFIRMED_CORRECTIONS_UNDONE` fires. Report completion, timeouts and any recurrence of the wrong fit.
- **MINOR.** Correct the note's figures (309, 0.26 m, "mirrored", the 48/35 filter). Report corrections by type and by initial error. The s960 class (a correct fit refused at 0.5-1.1 m of lateral error) stays open; declare it.

## Not verified

No engine run at `9e9cade`. The working tree carries uncommitted edits by someone else (the undo deferred to the give-up edge in `controller.py`, comments in `structure_aiding.py` and `config.py`); they are not part of this review and do not touch the scoring code my replays used. Replays are open-loop and reconstruct the unconfirmed sum from log fields. The step-cap effect is one correction at a time; its closed-loop value is untested. `reg1` was partial and at `3e3db7b`: in its 20 gate runs at CLEAN and LOW, no identity or axial corrections and no hold timeouts, so the new path did not run there [M].

## Re-check at 8fdcbb4

Tags as above. Code and tests taken with `git archive` of `8fdcbb4`; `HEAD` was `8fdcbb4` with these files clean throughout. No `med10` result was read. Read-only; no simulator; scripts in my own folder `r7_recheck`. I did not open any formal review.

**Verdict. R1 and R2 are implemented as I meant, and the cap does what it was for. Freeze on `8fdcbb4` exactly as committed: no, for one small code change (M1) and one correction to GUARANTEES (M2). With those two, yes, subject to `med10` and the criteria below.** No BLOCKER. If the owner will not change the code again, `8fdcbb4` can be frozen only with M1 declared as a limit, giving the s981 number.

M1 comes from the deferral of the undo to the give-up edge. The deferral is right in itself (never move the estimate while committed). But the sum is read at the edge, so it also takes corrections made after the timeout, which the timeout says nothing about. In the logs this removes a correct fit: 0.04 m becomes 1.18 m. Reading the sum at the timeout and applying it at the edge removes that case and changes no other.

`med10` transfers to the fixed commit for every run in which no correction was applied between a `loc_ok`-false timeout and its give-up edge, because the two codes then behave the same. Only the runs where that fails need rerunning.

### Pre-registration of the evidence for R3

Written and appended to this file before reading any `med10` result. None of my scripts in this re-check opens `med10`; the baseline counts come from `med4`-`med9`.

Family F: `lost_drone_mutex` seeds 1060-1072 (13 runs) and the two rise variants, seeds 1061-1064 (8 runs), 21 runs. Whole batch: 18 standard + 12 + 8 = 38 runs. A run classed INVALID is rerun on the same seed and does not count. An incomplete batch cannot be accepted.
Baseline of F, `med5`-`med9` [M]: 24 of 27 PASS, 3 INCOMPLETE (s960 rise_low, s981, s995), 22 hold timeouts, 3 runs with more than 2, the wrong-fit signature in 1 run. With `med4`: 26 of 30 PASS, 27 timeouts, 4 runs with more than 2.
Wrong-fit signature: an identity or axial correction after which the true horizontal error is at least 1.2 m and has risen by at least 0.3 m.

**ACCEPT, all of:**

- A1. All 38 runs: 0 SAFETY_FAILURE, 0 UNSAFE_ACTION, 0 SUT_ERROR; P2 holds everywhere.
- A2. F: at least 20 of 21 PASS.
- A3. F: at most 10 `ENTRY_HOLD_TIMEOUT` in total and at most 1 run with more than 2.
- A4. Signature: in no run of F; across the 38 at most 1, and that run PASSES.
- A5. The 18 standard runs: at least 17 PASS.
- A6. Each `UNCONFIRMED_CORRECTIONS_UNDONE` of at least 0.3 m: true error afterwards at most 0.1 m above the error before in at least 2 of 3 such events, and none worse by more than 0.5 m. If no such event occurs, recovery by the undo is **not demonstrated**: this does not block the freeze, and the report must say so.

**HOLD** (no freeze; root cause, then up to 12 fresh seeds of `lost_drone_mutex`): exactly 2 INCOMPLETE in F, or A3, A5 or A6 failed with no reject condition.

**REJECT, any of:** a SAFETY_FAILURE or UNSAFE_ACTION; 3 or more INCOMPLETE in F; a run of F INCOMPLETE with the signature; an undo worse by more than 0.5 m in a run that ends INCOMPLETE.

Power, so that nobody reads more into it [D]: P(at least 20 of 21) is 0.31 at the baseline failure rate of 1 in 9, 0.72 at 5 %, 0.93 at 2 %. 21 runs cannot show that the failure rate fell. Acceptance means "not worse, and the mechanism did not recur".

### 1. R1 and R2 as committed

**Implemented as meant.** [M]

- R1: the sum is reset at the commit edge (`start_commitment`), by a validated fit, and by a measured crossing. R2: the cap acts only on the path with `safe` false; validated fits are not capped; the capped value, not the raw one, enters the sum (`structure_aiding.py` lines 394-403).
- `tests/test_unconfirmed_corrections.py`: 11 of 11 pass on the archive of `8fdcbb4`.
- `reg1` (`3e3db7b`) still stands for these paths [D]: CLEAN 36/36 and LOW 36/36 PASS, 0 hold timeouts, so the undo never acts. The only identity steps are 0.416, 0.297 and 0.389 m, all below the cap. The two MEDIUM `lost_drone_mutex_carried` runs in `reg1` (s996, s997, INCOMPLETE) are outside that claim.

**Replays through the committed `StructureAiding`, cap on and off.** [D] Open loop: logged echoes along the logged trajectory. Whether the identity rule is available can only be bracketed: "logged" allows it only at steps where the log shows it, "always" at every step. The truth lies between.

| Case, \|e\| [m] at +5 / +20 / +35 s after the logged fit | Uncapped | Capped, logged | Capped, always |
|---|---|---|---|
| s995 trigger (start 76.4 s, the two wrong fits) | 2.74 / 2.68 / 2.56 | 1.08 / 1.02 / 0.89 | 0.58 / 0.03 / 0.11 |
| s981 rescue (1.32 m initial) | 0.08 / 0.10 / 0.09 | 0.73 / 0.63 / 0.59 | 0.14 / 0.10 / 0.14 |

- **Does the cap harm the rescues? Not materially.** Only 6 identity fits at 0.7 m or more lie outside the carried stress runs, and one of those is the s995 wrong fit. In the other four besides s981, the change at +35 s is -0.14 to +0.04 m in both availability modes. s981 is the one that depends on availability: 0.59 m if the rule is lost after the first step, 0.14 m if not. Summary over the 6: at +5 s median 0.15 m uncapped, 0.36 m capped and logged, 0.17 m capped and always; cases still above 0.5 m at +35 s: 4, 4, 2.
- **Repeated capped steps towards a wrong pose: not seen.** On the s995 trigger the uncapped replay reproduces the logged failure (2.74 m). With the rule as logged only the first capped step lands, in the wrong direction (1.08 m). With the rule always available the estimate reaches the truth after three steps (0.58 m at +5 s, 0.03 m at +20 s). That is one case.
- Over 14 wrong fits from a small error (below 0.2 m, then worse by more than 0.4 m): median of the peak error 1.00 m uncapped, 0.63 capped; worst 1.50 and 1.25. Outside the carried runs (8 cases): median 0.82 and 0.51, worst 1.18 and 0.69. Final error at +35 s, capped against uncapped: 6 better by more than 0.1 m, 4 worse, 4 unchanged. In the 4 worse the uncapped replay made 2 to 4 corrections that removed the jump, and the capped one ended at 0.45 to 1.10 m after 1 to 3 steps. 2 of the 14 (carried s988) produce no correction in replay, capped or not, and count as unchanged.
- Carried s988 (hull at the bar, 100-200 s): 5 corrections uncapped, 2 capped; peak 1.21 and 0.87 m; but median error 0.44 and 0.76 m. Fewer flips, not a smaller error.
- A replay started at 60 s does **not** reproduce s995: no correction with the rule as logged, and with it always available the fit acts before its premise existed (71.1 s) and stays right. That window says nothing about the cap. The sensitivity to the starting state is itself a warning.

### 2. M1: the undo takes corrections made after the timeout

The undo as committed on the logged corrections of `med5`, `med6`, `med7`, `med9`, against the sum read at the timeout and applied at the edge. [D]

| Semantics, 23 timeouts with `loc_ok` false | With something to undo | Improved / worsened | Worst | At least 0.3 m and worse by over 0.1 m |
|---|---|---|---|---|
| As committed (read at the edge) | 9 | 4 / 5 | +1.14 m | 1 of 8 |
| Sealed at the timeout | 8 | 4 / 4 | +0.08 m | 0 of 7 |

- The only event that differs is med7 `lost_drone_mutex` s981 drone_1: timeout at 324.3 s, a correct identity fit at 339.0 s (error 1.32 to 0.17 m), give-up edge at 342.4 s. As committed the undo removes it; error 0.04 to 1.18 m. With the cap the step would have been 0.5 m at a time, so the loss is 0.5 m if one step landed and up to 1.15 m if all did.
- Evidence is thin in both directions. All 8 events of 0.3 m or more come from **two runs**. One rescue (s995, first timeout, 2.74 to 0.46 m) and one harm (s981). The other 6 are later timeouts of s995 replaying a stuck state that the first undo would have prevented; they are not independent and the error barely moves (2.70 to 2.71, 2.57 to 2.66).
- Limit of the proposal: one logged case supports it. The argument is that a timeout condemns what preceded it, not what follows.

### 3. The rewritten statements

- **Physically accurate** [M/D]: the undo after the give-up edge and never while committed; step at most 0.5 m; the sum unbounded until a validated fit (config comment); a hold without validation does not refute a correction; separation uses raw ranges while target selection depends on the pose; admission needs a validated fit and the past-centre path does not; the undo can remove a correct correction (s981); 8 timeouts in 732 s, and true lateral offset at those timeouts +3.38 to +3.74 m, so "3.5 m off the axis" holds.
- **Not accurate, M2.** GUARANTEES row 81: "14 timeouts with something to undo, 10 improved, 4 worsened by 0.21 m or less". I cannot reproduce it. I get 8 to 11 events under five semantics (read at the edge, sealed, at the timeout instant; with and without the reset at the commit edge), never 14/10/4. It also contradicts the same row's s981 sentence: at committed semantics one undo worsens by 1.14 m. "In the logged cases with a large error it did restore it" holds for one event. If the count used other batches or another definition, it should say so; otherwise use the table above.
- **"So a wrong fit stays within reach of a later one"** (design note, docstring): true per step only. The replay supports it on one trigger; there is no bound on the sum. Say "per step".
- **"Open-loop replay: 44 worse and 75 better..."**: my round-7 figure, quoted correctly, but it is a one-step truncation of the logged corrections with no new fit. The replays above are the ones with a new fit.
- **"Also without noise (RC12)"** (row 81): the noise-free test covers the far side of a bar only. The mirrored near-tie was measured on noisy logged echoes. The design note has it right.
- First fit "0.73 m from the mirror": I measured 0.62 m (definition-dependent, no consequence).
- `GAIN = 1.0` comment, "a partial step can stop at an ambiguous pose": the cap now makes partial steps. Two of the 4 worse cases above (s932, s995 `gate_stalled_occupant`) stop after one capped step at about 0.5 m, consistent with that, not shown to be it. Worth one sentence.

### 4. What must be fixed first

- **BLOCKER: none.** Separation is untouched by R1, R2 or the undo.
- **MAJOR M1 (code).** Seal the sum when the timeout fires, apply it at the give-up edge, drop it on a validated fit or a measured crossing. Add a test: a correction after the timeout survives the undo.
- **MAJOR M2 (documents).** Replace or source the 14/10/4 figure and "did restore it".
- **R3 stays open** until `med10` is judged against the pre-registration above; the report must not claim the s995 mechanism fixed before that.
- **MINOR.** The wording items of section 3.

### Not verified

No engine run at `8fdcbb4`. Replays are open-loop: the changed estimate does not feed back into the trajectory, so they show what the fit does with those echoes, not what the fleet would do. The corrections in section 2 were logged by older code, without the cap. Sealing is proposed and evaluated on logs only; I did not implement or run it. No check of the formal side of the proposal (N13).

Process: the call I used for below-normal priority in my scripts (`SetPriorityClass(GetCurrentProcess(), 0x4000)` through `ctypes` without declared types) fails silently on this machine with error 6, invalid handle. My scripts therefore ran at normal priority, in this re-check and in the earlier rounds where I used the same call. I noticed it during this re-check and lowered the long replay by hand after about 13 minutes. I cannot tell whether this slowed any simulation.
