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
