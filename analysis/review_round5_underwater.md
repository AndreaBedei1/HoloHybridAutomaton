# Round-5 review, underwater physics and acoustics: `AcousticExit.adrift` at commit `ef54530`

Baseline `analysis/review_round4_underwater.md`, checked against `git diff 02f6a52 ef54530`, `med6` and `med7`; of the resolution note, only the underwater table. `med8` had no finished run when I read it. No simulator; code and results untouched.
Tags: **[M]** measured from repository logs or files; **[D]** derived by me, including an open-loop replay of the rule on finished runs and a Monte Carlo on `ChannelModel`; **[A]** assumption of the repository.

## Verdict

**Not yet lifted, one condition left.** The rule answers R1 by the numbers I can check, the premises hold in the simulated uniform currents, and the text corrections are done. Nothing has run in the engine at this commit. Lift the verdict if `med8` shows:

1. `lost_drone_mutex_carried`: no P2 violation, no contact of a live drone, no UNSAFE_ACTION on both seeds.
2. Standard set: no new failure class and no material rise in `ENTRY_HOLD_TIMEOUT` (prediction below).

Round-4 conditions 1 and 3 are met [M, diffs]; condition 2 is met for what `med7` finished (11 runs, s981 the only non-PASS). The carried failure is expected to become INCOMPLETE, not PASS: see L1.

## 1. The physical premises

**The estimate is fit for steady currents [M].** Median over 30 s of |`current_est`| against the true drift, live drones, `med6` and `med7`:
- Uniform currents: median error 0.005-0.045 m/s, p95 at most 0.06. The claim of 0.04 stands. It falls below the bound only in the first 6-10 s of a mission.
- Bias is on the safe side: +0.03 to +0.05 in still water, +0.09 at 0.35 m/s while moving (the low level has no quadratic drag term).
- The dead drone's own estimate reads 0; do not include it when reproducing these figures.

**It is not fit for transients [M].** With the 30 s median, true drift exceeded the bound by up to 0.46-0.74 m/s, 1-3 % of the time, on 3-5 of 12 drones in `formation_recovery_head_current`, `lost_drone_rejoin` and `formation_gust`. In `integrated_short`, in the acceptance matrix, by 0.065 m/s on all 9 drones. A jet near a gate can under-reach by the excess times its duration, several metres at worst [D]. `GUARANTEES.md` says only "the transient error is larger"; give the figures.

**What the premises omit** [D, A]:
- Shear: the reach covers only the 1.4 m climb to the top of the CR, so shear is small unless the column is stratified.
- Flow through the frame: the opening is 36 % solid, so speed-up can reach 1.2-1.5 U; at 0.3 m/s that is +0.06-0.16 m/s, at or above the margin.
- Thruster wash from a live drone at the hold point: not modelled, and it can exceed 0.1 m/s on a hull at a metre or two.
- A hull held by something else (tether, another structure) and released later: the reach stops growing once the rise time is spent, so it is uncovered.
- Rise lower bound: 0.05 m/s is near my 0.047 for +0.2 kg; less buoyancy lengthens the reach, 0.026 m/s at 1 N.
- A live drone that moves under power after its last fix and then fails is treated as passive since that fix. Rare, small.
- On a real vehicle the estimate depends on calibration: a 20 % slope error gives about 0.06 m/s at 0.3 m/s, and real drag is several times the engine's. Over-estimates cost liveness only.

## 2. Does it answer R1?

Yes, in the case I could replay. [D] On `med7` s981, with `drone_0` as observer, `adrift` holds from about t = 32 s (silence 9.7 s, water median 0.3 m/s), before the hull reaches the CR at about t = 41 s and before the admission at t = 45 s, and it would have held `entry_ok` False (`gate_perception.py:213`, `acoustic_gate.py:187`). `med8` must confirm it closed-loop.

Uncovered, all declared in `GUARANTEES.md`: a hull that reported GP_PASSED and is carried back; a hull never heard; transients; the premises above.

**L1. MINOR, declare the size.** The hold is for ever while silent, and isotropic. A failure anywhere within reach blocks the gate for the mission even if the hull has long since gone downstream. [D] At U about 0.05 m/s the reach includes the queue line; at 0.3 m/s it extends at least 4.6 m behind it. The acceptance set is unaffected: the `lost_drone_mutex` hull is 13.6 m from the CR in still water, and `gate_stalled_occupant` is held by a claim anyway. `lost_drone_mutex_carried` is not `safety_only`, so expect INCOMPLETE.

**MINOR.** The adrift set is not logged. Add it to the state log before the final campaign, or `med8` and the stress set cannot separate its cost from the latch's.

## 3. Liveness: predictions to check against `med8`

Silence before the rule fires on a queue-line sender [D]: 13.5-26 s at U up to 0.05 m/s (position bound 5 and 1 m), 4.5-10 s at 0.3 m/s.

| Channel | Prediction [D] | Check |
|---|---|---|
| MEDIUM, standard set | No measurable cost. Replay on about 38 gate runs, senders with no claim: 3 runs with any adrift, longest episode 5 s, largest total 27.5 s, in the already stuck s960. Monte Carlo: clear queue links never silent 15 s; a hull-shadowed 7 m pair held 3 % of the time at 15 s, 5 s episodes | Standard PASS unchanged; `ENTRY_HOLD_TIMEOUT` near the `med6` level |
| MEDIUM stress | `lost_drone_mutex_carried` no P2, INCOMPLETE; rise variants and `lost_drone_mutex` unaffected (hull out of reach in still water) | Class and P2 |
| MEDIUM_HT, reach 15-20 s | Gate held for one observer 8-9 % of the time in a 169 s mission, 14-16 % in 705 s; P(any episode) 0.45-0.56 at 169 s, 0.9 or more at 705 s; mean episode 21-43 s, p95 73-190 s; about 15 % with one link hull-shadowed (reach 15 s) | Delays of tens of seconds to minutes and repeated give-ups; no P2 violation. INCOMPLETE possible with a long OFF |

## 4. R2 to R5

| Item | Status [M] |
|---|---|
| R2 | Reworded; `loc_ok` stated as the direct cause, the rest stated as not established |
| R3 | Stated in the pre-registration |
| R4 | 29.3 s, 240 runs with 5 contacts, `_axial` declared where it is |
| R5 | Accepted and declared |

## Not verified

Any behaviour of the rule in the engine. The replay is open-loop (the drones did not obey the rule; fix age and position are reconstructed from `acoustic_fixed` and truth). Real wake, shear and frame flow are argued, not measured.
