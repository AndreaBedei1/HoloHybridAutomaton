# Review round 3: formal verification, second pass

Commit `6ce8ba7` against `c6d8c98`, branch `feature/b2-medium-robustness`. Independent reviewer, 2026-11-09. Code and results were not modified; no simulator started.
Tags: [O] observed (solver output, the real classes on crafted inputs, log data); [I] inferred. `REVIEW_ROUND2_RESOLUTION.md` was read last and treated as a claim; the underwater reviews were not read.

**Verdict: do not freeze yet.** F1, F3, F4, F5 and F12 are resolved. F2 is only half fixed and stays a blocker. F6 is partly fixed, and the resolution's statement about it is false at controller level. The new M3h and S2h are UNSAT only because their horizon is too short.

## Status of the round 2 findings

| Finding | Status | Basis |
|---|---|---|
| F1 BLOCKER, no relation and no latch | **Resolved** | `gate_perception.py:168-170,182`: FRONT-only in `[r_band, r_far]` is always a corridor echo. [O] Real class, no beacon, centre queue, A at the hold point: `occ=BUSY`, `can_go=False`. [O] 1451 grid cases (s, view, heading) on the real class: none with neither relation nor latch. But see N1 for the proof |
| F2 BLOCKER, silence overrides an echo | **Not resolved** | Fixed in `no_fleet_drone` only; below |
| F3 MAJOR, claim bypassed at the next gate | **Resolved** (reported path) | [O] real `_update_gate_report`: verified gate 0, in the box of gate 1, uncommitted: (1, GP_COMMITTED). Residual: N5 |
| F4 MAJOR, S9 tautology | **Resolved** | S9 is now the sender rule with a mutation; the filter is no longer called a barrier. `flows.py:428` still leaves the exit side unfiltered, but the status logic no longer rests on it |
| F5 MAJOR, filter edits avoidance | **Resolved** | `controller.py:192-193`: filter on `v_mis` only; the SW, CA and FAILSAFE outputs are not edited [O, reading] |
| F6 MAJOR, S10 and reversal | **Partly** | S10 deleted; the rest below |
| F7 MAJOR, labels | **Mostly resolved** | S5, S6, S2 relabelled; S8 reads the deployed value; B2-PEg added ([O] 0/3000 with status off); S1m, S4m, S11m meaningful. Open: `docs/GUARANTEES.md:74` (N2); S6 and S7 still without mutation |
| F10 MINOR, permanent claim | **Disclosed** | `GUARANTEES.md:77` states it. Accepted as design. Still no scenario of a drone dying GP_COMMITTED before the CR (`lost_drone_mutex_carried` dies on arrival, GP_NONE) |
| F12 MINOR, version | **Resolved** | `CRITERIA_VERSION = "v6"`. Side effect: `robustness_metrics.py:447` skips the replay comparison for pre-v6 logs; [O] class unchanged on 4 med3 runs re-analysed |

## Open and new

### BLOCKER

**F2 (narrowed, not resolved). Silence still releases the sonar latch.** `perception/acoustic_gate.py:349,365` (A4), `:331`, `:162-167`.
`fresh_only` was applied to `no_fleet_drone` (`:132`) only. `evaluate` still excludes a drone silent 70 s after GP_NONE, so a BUSY latch is released as soon as the corridor empties, and the corridor empties precisely when the bars mask an occupant in the CR.
* Scenario: A alive, unheard by B for 70 s (C heard), commits and enters; B sees it, the latch goes BUSY, A is masked inside the CR; A4 fires on silence; B may commit.
* [O] Real classes, only the silence differs (echo 2 s, then masked): 29 s: BUSY throughout; 79 s: EXITING at once, FREE 1.0 s after the last echo, `entry_ok=True`.
* My round 2 recommendation ("keep it only for the acoustic vetoes") was too narrow; the author followed it. The resolution's "Fixed" and the sentence in the underwater table ("silence no longer discards it") hold only while the echo is visible.
* Likelihood is low in MEDIUM, but the owner's rule has no probability clause, and the author's own `MEDIUM_HT` has P(OFF > 70 s) = 5 % per OFF period [I, from its parameters], against the P < 1e-3 per mission in the `config.py` comment.
* Resolve: use fresh evidence only in `evaluate` and `evaluate_clearance`. [O] Monkeypatched in a scratch process, the latch then releases 25.0 s after the last echo through the `t_occ_max` fallback: a bounded cost, already the outage-mode rule.

### MAJOR

**N1. M3h and S2h are horizon artefacts.** `formal/check_mutex.py:464,715`, `formal/check_status.py:104`, cited at `docs/GUARANTEES.md:76`.
For n = 3, l = +-3.5 the bars mask A from s = -0.93, before the CR at -0.8. A "may stop anywhere before the CR" can therefore stop masked, the latch times out, and B commits.
* [O] M3h: 42 s UNSAT (141 s), **48 s SAT** (B commits at 36.8 s with A stopped at -0.93; A in the CR 47.6-48.0 s). S2h: 32 s UNSAT, **42 s SAT** (B commits at 31.3 s; A in the CR 26.5-38.3 s).
* Scope: no claim heard (B0, or packets lost). Views whose mask starts inside the CR cannot do this: [O] M3h n=2 l=+1.75 is UNSAT in 252 s at the 900 s timeout, which closes the UNKNOWN. Plain M3 re-run: [O] UNSAT for n=2 and n=3; M3b SAT.
* The code gives up after 26 s (`flows.py:338`), which the model omits. With that bound added, Z3 returned UNKNOWN in 170 s; by arithmetic a violation then needs a crossing slower than about 0.17 m/s, against 0.5 m/s nominal, but within M3's own floor of 0.12 m/s [I].
* Resolve: a horizon above queue-to-mask + `t_occ_max` + `t_clear` + B's travel (45 s for M3h), the 26 s bound and a crossing premise in the model, and a label that says what remains.

**F6 (partly). A drone back in the box still reports GP_PASSED.** `control/controller.py:328`, `:180`; `perception/passage.py:58-61`; `perception/acoustic_gate.py:165`; `control/flows.py:176-179`.
`near` is evaluated against the current gate `go` only, and `gp.advance()` has moved it on. [O] Real `_update_gate_report`: single gate, verified, advanced, truly in the CR: (0, GP_PASSED); two gates, same. Only before the advance: (0, GP_COMMITTED). S9 enumerates `report_phase`, not the controller.
So `GUARANTEES.md:79`, the docstrings and the resolution's "a drone back in the box reports a claim" are false. A fresh GP_PASSED still discards an echo, and in the exit evaluation it excludes at any age [O]. Reversal at a static reference remains unguarded; plans keep slots at s >= 2 m unless sigma moves them [I], so reach is narrow. Resolve: test the box of the verified gate as well.

### MINOR

**N2. `_axial` (`perception/structure_aiding.py:252-280,337`): useful, but the argument for it is wrong.**
* It does what it is for on real data. [O] Open-loop replay of the real `update` on med4 `lost_drone_mutex` sensing, the dead drone left out after 70 s as at `perception.py:310` (18 080 steps, 796 firings, repeated because the logged estimate is not corrected): median error 0.56 m before, 0.06 m after; 4 % worsened; largest error after 0.31 m. `gate_single` and `line_parallel_mutex`: 0 firings. [O] Closed loop, 6 finished med5 gate runs: 1 firing, error 0.16 to 0.12 m. [O] Synthetic: an error of +-0.6 m with the bar visible is corrected within 0.05 m; a second echo 0.5 m or more from the bar is rejected.
* "Only the own pose reads it" (resolution, design section 5) is wrong. [O] With no bar return any single echo within +-1.2 m is adopted (own error +0.5 to +1.35 m, beyond `MAX_CORR_M` in total), and the classifier then labels it STRUCTURE (DYNAMIC at bar-0.5, UNCONFIRMED at bar-0.9): it leaves `targets`, hence `d_min` and the corridor. With `others=None` it stays DYNAMIC, so acoustic data do reach the separation input: `GUARANTEES.md:74` is false on this path.
* Reach is narrow, hence MINOR. A missing bar return with one other echo in the window is common ([O] 44 runs of 4 or more steps in 18 med4 gate runs, in the samples the hull of a live drone, which the identity test rejects), but a drone left out has been silent 70 s: [I] a free-rising hull is then at least 3.5 m above the gate, outside the window, and a pinned one within about 0.5 m of the bar. I accept the departure from my F2 recommendation on reach, not on its justification. A live drone unheard for 70 s is the residual case.
* Also: [O] a lateral error of 0.6-1.0 m at s >= -3 induces -0.2 to -0.25 m along the axis; `recent_ax` is not cleared on the early returns of `update` or on a successful fit, so the 4 samples need not be consecutive; three unit tests, no randomized check. Resolve: correct the claims; cap the total axial correction until a validated fit; clear `recent_ax` on every path; add a randomized check with `others=[]`.

**N3. Give-way in the rejoin lane.** `control/flows.py:206` is right: [O] real `rejoin_entry` on the displaced lane returns None, the old code sent it to DROP_BACK. But `perception/perception.py:440` works against it: [O] `lateral_far` = +2.5 for an offset of -2.5, and the integrator reaches lam = +2.0 (net lane displacement -1.23 m at 10 s, -0.5 m at 60 s).
Only while far from the slot with a neighbour on its slot within 7.5 m, so [I] it partly undoes RC8 in a long standoff with line-mates nearby. Liveness only. Resolve: subtract the give-way offset from the residual.

**N4. Liveness cost of F1 and F2 together.** [O] With a dead neighbour a persistent no-drone echo in the band holds BUSY for 150 s and more, no timeout (`acoustic_gate.py:132`, `gate_perception.py:168-170`): a dead drone can never be freshly excluded, so `no_fleet_drone` is off for the rest of the mission. `_axial` is the only mitigation. Evidence awaited from med5 `lost_drone_mutex`.

**N5. Sender rule residuals.** `perception/passage.py:56-59`: `in_progress` precedes the box, so an unverified passage reports (0, GP_ADMITTED) in the box of gate 1 [O]. The 2-bit wrap is unchanged. Both need more than one gate; no scenario has one.

## Specific questions

* **M3/S2 predicate:** yes, for the relation and the latch. [O] Grid: differences are 234 cases where the model has WAIT and the class no relation but BUSY (safe direction), and 2 boundary cases. `front_only_s` is the all-headings intersection; `band_from_s` is an artefact (-6.0) for n = 2, harmless because `front_only_s` starts later.
* **Sender order and index:** right for the current gate; wrong for the gate just passed (F6) and for an unverified earlier gate (N5). Index is `gp.idx` for a claim, `p.idx` for ADMITTED and PASSED [O].
* **`_axial` and dead drones:** N2.
* **Rejoin lane:** N3.

## Verified and not verified

Verified: as tagged; scripts in the scratch directory only. [O] `check_status.run()` without `save()` at this commit: all PASS. [O] 47 tests in `test_status_payload.py` and `test_structure_aiding.py` pass. The `formal/results` files shown as modified in `git status` are the author's run in progress, not mine.
Not verified: any simulator run; `_axial` in closed loop beyond the 6 finished med5 gate runs (open-loop replay used logged fixes and the truth as the fix region); a P2 violation in simulation (none seen: F2 and N1 are unexercised paths); n = 4 with the extended horizon; the channel model behind the 70 s bound; med5 as a whole.
