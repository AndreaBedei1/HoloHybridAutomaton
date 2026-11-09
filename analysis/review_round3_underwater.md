# Round-3 review, underwater physics and acoustics: second pass on commit `6ce8ba7`

Checked against `git diff c6d8c98 6ce8ba7` and the code. `analysis/REVIEW_ROUND2_RESOLUTION.md` was read last, as a claim. No simulator; code and results untouched.
Tags: **[M]** measured from repository logs or files; **[D]** derived by me, including offline runs of the project's own `EchoClassifier`, `StructureAiding._axial` and `ChannelModel`; **[A]** assumption of the repository.

## Verdict

No new BLOCKER. F1, F8 and F4's model gap are honestly handled. Two resolutions rest on scenarios that do not test what they are named for (F3, partly F1), and one rationale is only partly true ("a drifting body in the CR is a corridor echo"). The new along-axis correction is well conditioned but identifies its echo only by elimination. `med5` holds 12 runs, none of them stress scenarios or gate scenarios, so every statement about those scenarios below is a prediction to check, not a result.

## Status of the round-2 findings

| ID | Code at 6ce8ba7 | Status |
|---|---|---|
| F1 rise speed | `config.py:387-391` corrected to 0.047 / 0.102 and range 0.05-0.13. Design note line 59 still says 0.05-0.18 [M] | Resolved in code, stale in the note; test weak (see below) |
| F2 pinned occupant | `gate_pinned_occupant` added, outside acceptance | Addressed by construction; no result yet; depends on one ping (N1) |
| F3 drifting dead drones | `fresh_only` in `no_fleet_drone`; sender claim in keep-out; `lost_drone_mutex_carried` | Partly; scenario does not carry a hull into the CR (N2); stale premise text remains |
| F4 loss process | `MEDIUM_HT` added, §10 declares | Declared; 31 s and "< 1e-3" statements not corrected (N4) |
| F7 bench check | `currents.py` unchanged; commands 0.06 / 0.135 used instead | Acceptable workaround; "measured" is wrong (N5) |
| F8 diffracted packet | `acoustic_channel.py` return unchanged; §10 line declares it | Honestly declared |
| F9 lateral pull | `perception.py` unchanged; not declared in §10 or GUARANTEES | **Open.** "Bounded by the median" is false for one neighbour (N6) |

## New or remaining concerns

### N1. MAJOR. "A drifting body in the CR is a corridor echo" is true only for the half of the CR the frame does not mask
The resolution's reason for omitting a drift-reach bound. An echo inside the structure window is labelled STRUCTURE before it reaches the latch (`sonar_processing.py:193-201`, tolerances 0.30 / 0.70 m).
- [D] Hull centres on a grid over the CR, project classifier, true pose: masked **52 %** from the on-axis queue point, **92 %** from the l = -3.5 m queue point, 45 % from the hold point. On the axis at dz = 0, every s >= 0 is masked from the queue. The pinned hull (dz 0.868) gives 4.99 m against a bar window opening at 4.82 m: STRUCTURE from every queue point at estimate error 0 and +0.5 m.
- [D] Error is what unmasks it: at -0.5 m the on-axis share drops to 33 %. The new corrections that drive the error to zero therefore increase masking.
- What is covered [D]: a body in transit. It is visible in the first half, latches BUSY, crosses in about 5 s at 0.3 m/s, and the latch holds 25 s after the last echo. A hull pressed on the upstream face of a post is also visible.
- What is not: a hull stopped in the masked part with no claim and no fix placing it inside. Then `t_occ_max` = 25 s releases (`gate_perception.py:251-254`). `GUARANTEES.md:72` already lists the "masked far half" as outside the guarantee for BLACKOUT; the same limit applies here and the resolution should not imply otherwise.
- The exit path keeps the 70 s rule. `fresh_only` applies only to `no_fleet_drone` (`acoustic_gate.py:132`); `evaluate`, `last_known_inside`, `heard_inside` still use `status_excluded` with silence (`:287,313,349,365`). Failure under `MEDIUM_HT`: a live GP_NONE drone unheard on one link for 70 s commits and enters; its echo enters the masked half with the corridor empty; rule A4 releases. [D] P(gap >= 70 s) is 0.01-0.24 per link per mission on that channel.
- `gate_pinned_occupant` needs stratifying. `spec.dead` is true from t_on = 1 s, so the drone emits at most two pings (t = 0, possibly 1 s) [D]. A survivor at about 15 m hears each with P about 0.7 [D]. A survivor that heard it is held back by a claim or `last_known_inside`; one that heard nothing (8-29 % per survivor) has only a masked echo. Report the three branches separately, or the PASS count will say little.
- Resolve: state the masked-half limit in §10; report P2 in `MEDIUM_HT` by whether the entrant's link was OFF; remove `gone` from `evaluate` or require a drift-reach bound.

### N2. MAJOR. `lost_drone_mutex_carried` does not test F3
`scenarios.py:499-509`, `runner.py:73-74`. The docstring says "the first drone to reach the queue line"; the code fixes `drone_0`.
- [M] `drone_0` has `queue_lateral` 3.5 m (`run_config.json`). [D] A 0.3 m/s along-axis current carries it past the frame with 1.96 m of lateral clearance from the outer post; it never approaches the CR.
- [M] In 24 earlier gate runs of `drone_0` (med3, med4), its true s while queued is -5.60 to -4.94, median -5.53, against the trigger at s >= -5.40; it is already committed in 18 of 24 at the first crossing of -5.40. So the failure will mostly occur after GP_COMMITTED and the gate is held by a claim for ever, not through the GP_NONE path F3 concerns (no-current data).
- [D] Even on the axis at nominal rise, the hull bottom is 1.55 m at the bar against a bar top of 1.245 m: it passes over. On the axis at 0.05 m/s it enters and scrapes under the top bar. My round-2 text said so; neither that case, the 0.6 m/s case nor the exit-side case exists.
- Stale premises remain: `config.py:358-359` ("has risen several metres"), `acoustic_gate.py:151` ("and rose, cannot enter").
- Resolve: fail the on-axis drone (rank 1, `queue_lateral` 0) or set the trigger by its own queue state; add the low rise rate; correct the text.

### N3. MAJOR for claims, MINOR for the simulation. `_axial` is well conditioned, but "identified as a bar" is elimination only
`structure_aiding.py:252-280`.
- Sound [D, on the real class]: dr/ds = 0.91-0.98 at s = -2.7 to -5.0 m; correction accuracy matches the echo offset; two echoes 0.3-0.5 m apart are refused by `UNIQUE_M`; never moves the lateral estimate; capped at 1.2 m.
- The residual test `res[k] <= 0.15 m` discriminates nothing: `ds` is free, so **any** single echo within about +-1.0-1.2 m of the predicted bar is accepted (echo offsets from -1.0 to +1.0 m, outside the 0.15 m dead band, all returned corrections [D]). Identity comes only from `echoes_not_drones`, from which `_other_regions` drops any drone silent for 70 s, so a dead hull, or under `MEDIUM_HT` a live one, can be taken for the bar. The design note says "no occupancy decision reads the premise". True of the premise, not of the result: the corrected estimate sets the structure window and the bands the latch reads.
- Direction of the error [D]: a nearer echo moves the estimate forward (holds short, conservative). A lateral error of 1.0 m biases the correction by +0.25 m at s = -2.7 (+0.15 at -3.5, none at -5.0), also forward. A backward correction needs a lone echo beyond the bar with the bar undetected, impossible in the simulator but not in water.
- Sonar physics [D, A]: on the axis the nearest bar point is an inner edge about 1 m off axis; the front face has no specular point towards the sonar, so a real return is off-specular or edge-diffracted and weak. The simulator returns it at full level from a 120 deg cone that matches no product. With the real 25 deg beam the bars are outside the cone at any on-axis range below about 4.5 m: the echo this correction needs would not exist where RC7 needs it (about 2.7 m). A 0.25 x 0.30 m section is also an assumption about the frame.
- The design note (section 5) records that an along-axis correction caused a contact in development; this one acts only before the CR and on weaker evidence than the crossing measurement. No gate run yet at this commit.
- Resolve: log every axial correction with truth in the campaign; state that it is specific to the 120 deg cone; require a second confirming view, or reject when a silent drone was last near the frame.

### N4. MINOR. `MEDIUM_HT` is a fair one-point stress; its description and the old statistics are not exact
`acoustic_channel.py:92-94,110,141-152`.
- Fair as a sensitivity [D]: P(OFF > 40 s) = 0.103, mean OFF 36 s, OFF fraction 0.23. On a 7 m clear link: P(gap >= 45 s) 0.03 (90 s mission) to 0.38 (705 s); P(any expected link >= 45 s) 0.32-0.95 per run, >= 70 s 0.11-0.83. That is a real test of the thresholds. Reachable by `--protocol B2_MEDIUM_HT` [M].
- §10 says "parameters from a plot". Only the OFF tail is (Morozs Fig. 6c, "poor" links). The ON mean 120 s is not in that paper, and PSR 0.77 is a "good" link whose OFF law ends near 35 s. It mixes classes; say so.
- All links start ON, so short missions feel less of it; a stationary start would be fairer.
- `lr` uses key `[seed, 5102]`, the same as `beacon.py:64` `payload_rng`: the two are the same stream, so the comment on separate draws is wrong. Harmless in size; change the key [M].
- Still uncorrected [M]: `config.py:222` and design note line 72 (31 s, a far-pair statistic); `config.py:358` cites "< 1e-3" to a document that has no such calculation.

### N5. MINOR. Rise variants: right ends, weak test, wrong word
- The commands give the ends of the range [D]: 0.06 → table command 0.620 → 0.051 m/s (bench interpolation 0.051); 0.135 → 1.066 → 0.135 (bench interpolation 0.129). Within 0.05-0.13.
- "Measured" in `scenarios.py:513` and "measured speeds" in the resolution are not: `buoyant_ascent.json` has no 0.06 or 0.135 point and is unchanged since c6d8c98 [M]. Re-run the bench or say interpolated.
- Weak test [D]: in `gate_stalled_occupant` the status holds the gate regardless of the hull, and in `lost_drone_mutex` the hull is abreast, 9 m before the gate, in no one's lane. The interaction in my F1 (a follower reaching a rising hull) needs a front or mid-lane failure; I wrongly suggested the three existing scenarios would show it. `lost_drone_timeout` fails a rear drone, so it would not either. Add a front-drone TOTAL_FAILURE at the low rise rate.

### N6. MINOR. F9 is not bounded by the median
The median rejects nothing with one neighbour, where the Monte Carlo of round 2 gave p99 +0.8 m towards the neighbour; with two, `np.median` is the mean. The cap (2 m) bounds it and the separation layer keeps it a liveness issue. No document states it; one line in §10 would do.

### N7. Watch in the results
With a dead drone, `no_fleet_drone` can never return True (no fresh evidence), so the phantom discard is off for the whole run after a failure, and a FRONT-only bar echo in the widened band now always feeds the latch (`gate_perception.py:169-170`). That is the RC1 condition in the scenario where RC1 occurred. Conservative for safety; check `lost_drone_mutex` in the next batch.

## Limits documents: do they say what a report must

`ACOUSTIC_CHANNEL_MODEL.md` §10 states the bounce excess, in-plane cap, self-noise, CRC, diffracted packet, loss process and H6 correctly. Missing, and needed: thresholds of 45 s and 70 s are simulation-calibrated; masked half (N1); 120 deg sonar specific to the axial correction (N3); F9 and the F12 payload length limit; engine drag and coast (no document states the N vs cN mismatch) [M].

`ACCEPTANCE_CRITERIA.md` v6: the threshold is right, 63/66 gives p = 0.0382 and 62/66 gives 0.078 [D]. Two gaps:
- It declares the failure and jet scenarios not like for like, then pools them in the test. 15 of the 66 runs are in those scenarios and 5 of the 10 v10 failures [M, design note list]. Add a like-for-like line: 46/51 at v10 [M]. There 50/51 gives p = 0.102 and only 51/51 reaches 0.028 [D], so 63/66 can be met with no significant gain in the scenarios that are comparable; report it as descriptive.
- Stress scenarios are "reported" with no seeds, N or metric pre-registered. Fix them before the freeze, with the stratification of N1.

## Verified / not verified

Verified: the diff and current code; offline runs named above; 24 earlier `drone_0` gate runs; Fisher values; `spec.dead`; `MEDIUM_HT` wiring. Not verified: any behaviour in the engine at this commit (`med5` incomplete, no gate or stress run); real sonar returns, argued from physics only; values read from plots in round 2.
