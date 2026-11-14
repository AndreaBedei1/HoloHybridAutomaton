# Review round 2: formal verification and hybrid-automaton soundness

Commit `c6d8c98` on `feature/b2-medium-robustness`, against `120b400`. Independent reviewer, 2026-11-09. Code and results were not modified.
Tags: [O] observed (solver output, the real classes run on crafted inputs, or log data); [I] inferred by reading or argument.

**Verdict: do not freeze.** In 54 runs safety held with wide margins, and no P1 or P2 near miss exists. But two changes break stated owner requirements, and
several "holds" entries prove less than their labels. Both blockers are small to fix.

## Findings

### BLOCKER

**F1. The sonar WAIT was removed where the latch does not replace it.** `perception/gate_perception.py:179` against `:167`, `:154`.
A FRONT-only echo makes no relation from `r_near - nav_err_m` up, but counts as a corridor echo only from `r_near`. Between them, a 1 m band, it has no effect at all. The change is unconditional,
so it also applies to B0 and to any receiver that has not decoded a claim. This is what the owner asked to preserve (sonar precedence).
* Scenario, n = 3, observer B at the centre queue point (l = 0, queue_s = -5.41). A (left drone) is committed and on the axis with s in [-2.02, -1.01], which includes the stage-2 hold point
  s = -1.18 (echo 3.76 m; corridor from 3.86; relation dropped from 2.86). `flows.py:320-321` still says queued drones see it and WAIT.
* [O] Real `GatePerception`, beacon off, A at the hold point: `relations=[]`, `has_prio=True`, `occ=FREE`, `entry_ok=True`, so `can_go=True`. B commits while A is committed in front of it.
* [O] Z3: I replaced the WAIT predicate in the S2 and M3 models with the one the code computes. Centre view n = 3 is **SAT** at the deployed parameters (S2: v_hi 0.7, offset 1.35, freshness 2.5 s: B commits at 5.7 s with A at
  s = -1.91, A in the CR 13.6-26.9 s) and at M3's parameters with no status (B commits at 15.4 s, A in the CR 24.6-32.0 s). With a crossing speed floor of 0.30 m/s or the nominal 0.45-0.55 m/s it is UNSAT, at 0.20 m/s SAT:
  only kinematic margin separates the two, and a stage-2 hold or an SW or FAILSAFE stop en route has speed 0. Views n = 2 and l = +-3.5 stay UNSAT (the pattern is not FRONT-only there). n = 4 not run.
* The evidence is on the wrong model. `formal/check_mutex.py:488` (M3) still has WAIT up to `beyond_s` and the diff does not touch it; `formal/check_status.py:131` (S2) has WAIT up to `corridor_from_s`. Neither matches line 179.
* [O] In B2 the claim closes the gap: in 30 A-B pairs of the 18 gate runs B reached `has_prio` at its queue 11.1 s or more after A's commit (A at s between -1.64 and +0.36), so `occ_busy` was the only thing holding it. The latch was BUSY at least 7.1 s earlier.
  Commit to BUSY took up to 6.4 s. The longest interval without a fresh fix at the queue (live pairs, 15 gate runs) has median 6.2 s, maximum 11.2 s, against a budget of about 10 s (A's path from commit to the band, 9 s [I], plus `t_clear`). P2 at the centre queue point now depends on the acoustic channel.
* Resolve: make "no relation" imply "latch input" (the same lower bound in both), or keep WAIT in the band unless the echo fits a mapped bar under the +-nav_err pose hypotheses. Have M3 and S2 use the code's predicate.
  Add a case with A stopped at the hold point. Then re-check RC1 liveness, which is why the band was widened.

**F2. Silence alone overrides a positive sonar echo.** `perception/acoustic_gate.py:159,164` through `:131` (`no_fleet_drone`); also `perception/perception.py:310` (map fit).
A last report of GP_NONE, older than freshness by any amount, excludes the drone after 70 s of silence, and the corridor echo is then discarded as a phantom. Since F1's change that echo has no relation either.
* Scenario: A is alive but unheard by B for 70 s (shadowed link, n >= 3, C heard). A commits and enters; B sees its hull in the corridor; B treats it as a phantom and commits.
* [O] Real classes, only the silence duration differs: 29 s: `phantom=False, occ=BUSY`; 79 s: `phantom=True, occ=FREE, entry_ok=True, relations=[]`.
* The owner's rule has no probability clause. The 70 s bound rests on a channel model (`config.py`, P < 1e-3 per mission), not on data; S4 tests only the committed/admitted path, so this path has no check. Likelihood is low: live pairs in the 15 gate runs checked stay under 22 s.
* Resolve: do not let `gone` discard a sonar echo, nor remove a drone from `_other_regions`; keep it, if needed, only for the acoustic vetoes. [O] No PHANTOM_ECHO fired in any run with a dead drone (`lost_drone_*`, `gate_stalled_occupant`), so no observed liveness cost.
  If the owner prefers to keep it, record it as an explicit exception.

### MAJOR

**F3. The keep-out claim is bypassed from the second gate on.** `perception/passage.py:56-60`, `control/controller.py:310,327-329`, `perception/acoustic_gate.py:160-163`.
`report_phase` tests `verified` before `near`, and a verified passage keeps being reported until the next commit. An uncommitted drone carried into the keep-out box of gate k+1 reports (k, GP_PASSED), and observers on k+1 exclude it.
[O] `report_phase(True, False, False, True)` = GP_PASSED; a ping whose fix and packet both place the sender in the CR gives `status_excluded=True`, also 30 s later for the same index (GP_PASSED has no freshness limit).
This is exactly the failure B2-PE was extended to catch, and B2-PE does not reach it. Every scenario uses G06 alone, so gate index, wrap (2 bits: gate k+4 aliases k, a dead claim blocks it, a stale GP_PASSED excludes) and this path have no experiment.
Resolve: evaluate `near` first and report the current gate. Bound GP_PASSED by freshness, or state and enforce a single-gate scope.

**F4. S9 is tautological and its label is false; the barrier is weaker than described.** `control/flows.py:426-434`, `formal/check_status.py:271`, design section 4.
S9 asserts `v @ axis <= 0` after a filter that removes the positive axial component. Lateral and vertical motion into the CR is untouched; a drone at s > 0.8 is unfiltered (line 426) and can back into the CR; a drone inside the CR cannot leave forward.
[O] 16 860 random cases outside the CR and in the band: the filtered velocity still reduces the distance to the CR in 36.1 %. Resolve: state the real property (axial, entry side), or implement a barrier on the distance to the grown CR and check that.

**F5. The keep-out filter edits avoidance output.** `control/controller.py:207-208`; the docstring says "avoidance included".
For an uncommitted drone in the band, a threat behind and an escape forward, the axial escape is removed. S2a-S2d and S1 reason on the unfiltered escape, so "separation untouched" holds for the lateral correction but not for the diff. [O] No activation: no uncommitted record lies in the band in the 18 gate runs. [I] One-sided S1d still covers the pair if the other drone reacts.
Resolve: apply the filter to mission modes only (the near-claim already handles a carried drone) or extend the escape checks.

**F6. S10 checks a guard that no longer exists, and reversal is unguarded.** `formal/check_status.py:276-283`, `control/flows.py:176-179`, `perception/acoustic_gate.py:162-163`.
The `exit_s + 1.0` guard entered in 419a780 and was removed in be4333a, the commit that added S10; S10 is now arithmetic on constants. "Straight to the slot" at `v_clock = 0` reverses whenever the slot is behind the drone, past the gate there is no keep-out, and GP_PASSED still excludes for ever.
[O] No drone came back: true s after verification never fell below 3.08 m. [I] Plans keep slots at s >= 2.0 m (`pair_column`, rally 6 m), but sigma up to 2 m could take it into the CR. Resolve: delete S10 or restore a guard; give GP_PASSED a positive basis (freshness or fix).

**F7. Other checks prove less than their labels.**
* B2-PE (`formal/check_acoustic.py:315-316`): `est` lies within nav_err of a point in the CR, so `inside_keepout` is always true and every packet is GP_COMMITTED; `no_fleet_drone` returns at the claim. [O] 2598 of 3000 samples short-circuit. With status disabled the geometry still gives 0/3000 excluded, so nothing is broken, but the committed check no longer tests it. B2-PEm only re-states that GP_NONE excludes.
* S8 (`check_status.py:286-299`): `fresh = min(a, b)` then `fresh <= b`; it can fail only if negative, and it recomputes the formula instead of reading `GatePerception`.
* S6: abreast queue points, n = 2, 3 only. The relation at risk is with a drone between the queue line and the CR (F1).
* S5 is a substring search on the source between two markers. True by reading [I], and it misses the indirect path: fixes and silence enter `StructureAiding.update` (`perception.py:293-313`, `:493`), a map fit moves the estimate, the estimate moves `r_near`, `at_queue` and the relations. B2-S passes only because the data arrive as a parameter.
* S2 covers GP_NONE freshness only (not GP_PASSED, another gate, or `gone`); A on the axis, no observer navigation error in the bands. No mutation exists for S1, S4, S6, S7, S9, S10; S2m varies freshness only and could not have revealed F1.
Resolve: name each check for what it does; add a B2-PE variant with status off; bind checks to deployed values and predicates.

### MINOR

**F8. Lateral correction `lam`.** `perception/perception.py:437-440`, `control/flows.py:154-158`. [I] Sign and convergence are right: for a drone on slot the residual is delta + lam_k - lam_i, a stable consensus with a 2 m clip. Limits:
"independent of the own displacement" holds only up to the fix age (bias v_lateral x age, up to 0.75 m, in the direction of motion); `slot_err` includes the give-way offset (the integrator works against it, line 508-509); with one neighbour a single multipath fix enters unfiltered. It only moves the reference, so a bias costs P3 liveness and cannot break P1 or P2 (separation layer unaffected). No effect on `rejoin_entry` beyond the moved lane. No evidence in med3 (feature absent at be4333a).
[O] In 22 finished med4 runs max |lam| = 1.09 m, no saturation (partial; not relied on). Open, cause not studied: med4 `p1_two_lines` s921 ended DEADLINE with drones 0 and 5 in SEPARATION_WARNING, 16 m from their slots.

**F9. FR <-> SW chatter.** Sampled at 10 Hz, so not Zeno, but no dwell bound exists. [O] Up to 539 switches per drone in 462 s; 17-75 % of SW episodes last <= 0.2 s. The exit is not a hysteresis crossing: `d_min` jumps by a median 0.48-0.53 m in one step (from 2.31-2.36 to [2.7, 3.0] in 80-100 % of exits), more than the 0.3 m band and more than motion allows, so a perception switch of `d_lower` [I]. Not new: v10 shows it (`formation_gust` 170/24/92 transitions).
Safety-neutral (min distance 2.8 m or more in these runs). It is the observable face of the two INCOMPLETE runs (final slot error 2.3-3.6 m). Resolve: analyse `d_lower` continuity; do not add a dwell inside the safety guard without re-proving S1.

**F10. Permanent claim wider than stated.** `acoustic_gate.py:166-171`, `passage.py:60`. A drone dying GP_COMMITTED at its queue point (3.4 m or more from the CR), or uncommitted in the box, blocks the gate for good; the design note mentions only a failure under the bar. It follows from the owner's rule, but no scenario covers it (`lost_drone_mutex` fails before commit).
[O] In `gate_stalled_occupant` the drone leaves the CR after 14 s (out by 34.8-37.4 s) and the survivors wait 670 s more; PASS comes from `safety_only`, so these runs say nothing about liveness, and the "stalled" stress on P2 lasts 14 s.

**F11.** `acoustic_gate.py:156`: a fix in the same ping never supersedes the status (only `t_tx >` older). [O] Sound inside the premise (the sender then claims), but it removes the cross-check exactly when the sender's navigation fails.

**F12. Criteria.** `referee/criteria.py:178-179` changes the permanent-fault deadline (+5 s, horizon +15 s per fault) with `CRITERIA_VERSION` still v5 (`:61`). [I] No med3 classification depends on it: DEADLINE runs have no permanent fault; `gate_stalled_occupant` only runs longer. Bump the version. `referee.py` time precision, `envelope.py` "DEAD" and `sensor_audit.py` do not change classes; no tolerance was added [I].

**F13. Documents.** `docs/GUARANTEES.md` is untouched by the diff: no row for status, claims, keep-out or S1-S10; `:39` still gives M3 with the old WAIT; `:71` "56-bit"; `:75` "identity pings only (... no payload)". `acoustic_gate.py:44-46` contradicts `:49-50` and `status_claim` on GP_COMMITTED. `flows.py:320-321` is stale (F1). The commit text "no reversal" is wrong (F6).

## Assessment by question

1. **P2 and status.** Sound [O and I]: a claim forces BUSY and not `entry_ok` in every latch state (S1, reading), and blocks the `t_occ_max` fallback; a dead committed or admitted drone stays claimed (S4, reading); reordering is handled by newest `t_tx` (`acoustic.py:229`); stale packets are dropped before the status is read (`:186`); an abort cannot clear an admitted claim. Unsound: F1, F2, F3, F6. Two simultaneous commits: none in 18 runs [O]; the path exists through F1 [O in model].
2. **Formal checks.** I ran `check_status.run()` without `save()` at low priority (the script's `__main__` overwrites `formal/results/status.json`): 16 of 16 PASS. Meaningful: S1, S3 + S3m, S4, S7 within its premises, S2 as a model of freshness. Not as labelled: S2 and M3 against the code (F1), S9, S10, S8, B2-PE (F4, F6, F7).
3. **Automaton.** Livelock: F9; symmetric hold-timeout abort is documented and was not observed (0 steps with two committed). Permanent claim: F10. `lam`: F8.
4. **Precedence, separation, avoidance.** Precedence: yes (F1, directly; indirectly via the map fit, F7). Avoidance: yes (F5). Referee and criteria: F12 only.
5. **med3 (54 runs at be4333a).** [O] TERMINAL: 52 SUCCESS, 2 DEADLINE, as in the campaign log. P1 and P2 recomputed from the truth columns match `referee_metrics` in all 54: minimum between drones 2.39 m (`p1_vertical_escape` s912; 2.30 m to the scripted intruder, which the referee keeps apart), no contact. Gate runs (18): occupancy at most 1, smallest gap between different occupants 17.1 s, never two centres within 1 m of the CR.
   369 PHANTOM_ECHO events, all by status: no other drone was within 4.37 m of the CR at any of them. A live drone truly in the CR always reported GP_ADMITTED and was committed (1363 records). 42 verified passages, all by crossing (`rear` and `odometry` never exercised), true s at verification 3.08 m or more against 1.19 m needed.
   Navigation premise `nav_err_m` = 1.0: within |s| < 3 m max error 0.61 m; at the queue max 1.05 m (1.6 % of samples above 1.0); whole approach lateral max 1.22 m. It holds where the keep-out relies on it. All these runs are B2 with a working channel: they cannot show F1 or F2.

## Verified and not verified

Verified: as tagged above; scratch scripts only. `git status` shows no tracked file changed; this file is the only one I added.
Not verified: any simulator run (none started); B0 behaviour in HoloOcean (F1 in B0 is shown on the perception class and in Z3, not in a simulation); a P2 violation (none observed in any run: F1 and F2 are unexercised paths, not failures seen); `lam` in closed loop; S2 with the faithful predicate for n = 4; the channel model behind the 70 s bound; the two DEADLINE fixes of c6d8c98; med4 as a whole.
