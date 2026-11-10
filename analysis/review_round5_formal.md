# Review round 5: formal verification, the `adrift` rule

Commit `ef54530` on `feature/b2-medium-robustness`, against `02f6a52`. Independent reviewer, 2026-11-10. Code and results were not modified; no simulator started.
Tags: [O] observed (real classes on crafted inputs, log data); [I] inferred. Of `REVIEW_ROUND4_RESOLUTION.md` I read only the formal table; no underwater review was opened. I did not open `s981`, because med7 is off limits, so the rule has not been replayed on the run that triggered it.

**Verdict: no new blocker inside the pre-registered campaign, but my no-blocker verdict now carries one condition.** `adrift` is sound on its stated premises, and while it is active its effects on the latch, the exit evidence and the commit guard are right. One MAJOR (N6): the hold can end with no new evidence, and a committed drone is then admitted, which is the shape of `s981`. I would fix it before tagging; the fix is small. If it is not fixed, the premise must be declared. M1 to M3 are resolved.

On round 4: I listed the not-engaged hull in the masked CR as a limit and did not count it as a blocker. `s981` reached that path, so that call was too lenient.

## Findings

**N6, MAJOR. The reach uses today's water for yesterday's drift.** `perception/acoustic_gate.py:194,202`, `perception/gate_perception.py:275`.
`v` is the median of the last 30 s at the moment of evaluation, so the reach can shrink with no new fix. Scenario: 0.3 m/s carries a dead hull into the CR, where it pins under the top bar; the current then stops.
* [O] Real `GatePerception`, last fix at the centre queue point, 60 s at 0.3 m/s: `adrift={1}`, BUSY, `entry_ok=False`. After 30 s at 0: `adrift` empty, BUSY, `entry_ok=True`. A drone already committed is admitted.
* [O] Conversely a 3 s gust of 0.6 m/s never enters the median (v = 0.10 m/s).
* `docs/GUARANTEES.md:79` promises the hold "for ever if it stays silent", and its premise says "uniform over the reach", not steady in time. No matrix or stress scenario combines a varying current with a dead drone near the gate [I], so the campaign will not show this.
* Resolve: keep the reach monotone per drone until its next accepted fix (a latch on membership is enough). Add a falling-water case to S12. Otherwise declare steady water as a premise and drop "for ever".

**N7, MINOR. `deaf` ends the hold, and the state left is neither rule.** `acoustic_gate.py:192`, `gate_perception.py:253,275`.
[O] Last survivor, or n = 2: `adrift` is empty from 45 s; the latch then stays BUSY for ever, because `t_occ_max` is refused for an acoustic-only episode, while `entry_ok=True`. So it is not "for ever" (row 79) and not "B0 rules" (the docstring): a queued survivor is locked out and a committed one is admitted. n >= 3 with another live drone is unaffected. Resolve: pick one behaviour and declare it.

**N8, MINOR. Unstated premise: passive since the last accepted fix.** `acoustic_gate.py:202`.
[O] Hull paths with the drone still under power at 0.5 m/s after its last fix: missed 0/1200 up to 4 s, 1/1200 at 6 s, 35/1200 at 10 s. The margin absorbs the usual fix gaps. State it in row 79.

## The four questions

1. **Predicate and S12.** Sound on the stated premises. [O] S12 0/1500 missed; S12m 341/500 missed, so the mutation bites. [O] A payload depth up to 0.45 m too shallow: 0/1200 missed, so that worry of mine did not hold. [O] On med6 `lost_drone_mutex_carried` the 30 s median is 0.21-0.26 m/s from 9 to 29 s against 0.30 true, within the margin; at 5 s it is 0.17, outside it, in the first seconds only. S12 tests the reach, not its label: it builds the path from the premises, never calls `GatePerception.update` (`test_adrift.py` does), and varies neither the water in time nor the deaf branch. Rename it, or extend it.
2. **Joining `heard`.** Right while active: [O] BUSY, entry refused, fallback blocked. A live drone reopens: side queue point at 0.3 m/s, BUSY after 10.6 s of silence; once heard, FREE about 3 s later with packets or after 3 fixes without. [O] Replay on the 18 med6 standard gate runs, an upper bound since status is ignored and fixes are reconstructed from the log: live sources 0-13 s per drone per run, longest streak 6.0 s; the dead drone of `lost_drone_mutex` is never adrift. Cost to the matrix is small. Never reopening: by design for a dead drone within reach. The reach is large and direction-blind, about 4.7 m to the CR in still water (the margin alone gives 2.8 m) and about 13 m at 0.3 m/s [I], so a death near the gate ends the mission even after the hull has surfaced. Consistent with the owner's rule; a deployment cost to state. The wrong state is the one in N6 and N7: BUSY for ever with `entry_ok=True`.
3. **F2, N1, N4.** F2: `status_excluded` is unchanged and `adrift` only adds holds ([O] S11 0/300), N7 being the exception. N1: the models are unchanged; `adrift` can only remove commits [I], so M3hs and S2hs remain valid scope statements. N4: worse inside the reach, where a dead drone now closes the gate with no echo at all; unchanged beyond it.
4. **M1 to M3.** Resolved [O, reading]: `robustness_metrics.py:172`, `GUARANTEES.md:74`, `referee.py:216`. The count beside the acceptance is still to be produced at the campaign. Separately, `ACCEPTANCE_CRITERIA.md` now says 240 runs and 5 contacts for med3-med6; I recount 251 and 4 on disk. The text should say how it counts.

## Disclosures

* `analysis/review_round4_formal.md` was committed with two unfilled placeholders, because a new request arrived before I finished it. The values are: S2hs at the deployed 2.5 s, SAT in 174 s; `check_status.run()` without `save()`, all PASS. I have not edited that tracked file.
* Process error of mine. My write of `r5_water.py` failed because a file of that name already existed in the scratch directory, which another agent is also using, and I then ran that file by mistake. It reads med6 and med7 logs and writes nothing, so med7 logs were read against the constraint, and I saw its summary table. I did not use it; the water figures above come from my own script on med6 only. Because of the shared directory, independence is not perfect on that one table.

## Verified and not verified

Ran at below-normal priority, scripts in the scratch directory, nothing written to `formal/results` (its modified files are not mine). [O] 43 tests pass (`test_adrift`, `test_b2_isolation`, `test_status_payload`). [O] S1, S3, S4, S6b, S11, S12 and S12m on the real classes at this commit.
Not verified: the rule on the `s981` log; any simulator run; med7 and med8; the Z3 rows at this commit (no model changed); a passive hull under a current that is not uniform in space.
