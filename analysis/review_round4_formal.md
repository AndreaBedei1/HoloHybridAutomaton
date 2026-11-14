# Review round 4: formal verification, third pass

Commit `02f6a52` on `feature/b2-medium-robustness`, against my baseline `analysis/review_round3_formal.md`. Independent reviewer, 2026-11-10. Code and results were not modified; no simulator started; `results/robustness/raw_dev/med7` not touched.
Tags: [O] observed (solver output, the real classes on crafted inputs, log data); [I] inferred. Of `REVIEW_ROUND3_RESOLUTION.md` I read only the formal table and the stress section; the underwater reviews were not opened.

**Verdict: my "do not freeze" no longer stands.** I find no blocker at `02f6a52` on the formal side. F2, N1 and F6 are closed; N2 and N3 are fixed; N4 and N5 are declared costs that do not change the verdict. Criteria v7 opens no path by which a safety failure of a live drone is classified as not a failure. This is a judgement on the code and the checks only; it does not replace the med7 result or the final campaign.

## Round 3 items

| Item | Status | Basis |
|---|---|---|
| F2 BLOCKER, silence releases the latch | **Closed** | `perception/acoustic_gate.py:163,164-166`: `gone` removed, silence excludes nobody. [O] Real classes, echo 2 s then masked: the latch stays BUSY and goes FREE at the same 25 s fallback for 29, 79 and 500 s of silence. The only uses of `silent` left are the map-fit premise (`perception.py:310`) and formation vacancy (`:280`) |
| N1 MAJOR, M3h / S2h horizon | **Closed by disclosure** | Labels state 42 s and 32 s; `check_mutex.py:730` and `check_status.py:374` add the counterexamples; `GUARANTEES.md:76` says what remains. [O] M3hs SAT (68 s), S2hs SAT (121 s). I asked for the 26 s give-up in the model; it is not there, and the label says so. Acceptable: the property is no longer overclaimed |
| F6 MAJOR, back in the box reports GP_PASSED | **Closed** | `perception/passage.py:65-75`, `control/controller.py:326`. [O] Real `_update_gate_report`, verified and advanced: truly in the CR (0, GP_COMMITTED); s = +3.0 beyond the box (0, GP_PASSED); two gates, back at gate 0 (0, GP_COMMITTED); box of gate 1 (1, GP_COMMITTED) |
| N2 MINOR, `_axial` | **Fixed** | `structure_aiding.py:283` caps the total until a validated fit; `recent_ax` cleared on every path by the `update` wrapper; the claims are corrected (`GUARANTEES.md:79`). [O] Hull at bar-1.3: no correction (was +1.35); closing echo: total 1.16 m, within 1.2 |
| N3 MINOR, give-way | **Fixed** | `control/flows.py:154`: the far-law integrator stands down during a give-way [O, reading; not run in closed loop] |
| N4 MINOR, liveness with a dead neighbour | **Kept, declared** | The price of the silence rule. One stress run in med6 (`lost_drone_mutex_rise_low` s960, campaign log). Does not change the verdict |
| N5 MINOR, more than one gate | **Unchanged, declared** | [O] Unverified passage in the box of gate 1 still reports (0, GP_ADMITTED). `GUARANTEES.md:80` limits the claim to a single gate per mission, as in every experiment |

## Criteria v7 (`referee/referee.py:214-217`)

**Sound.** I looked for a path by which a live drone's failure escapes, and did not find one.
* Only an edge of a drone whose mode is "DEAD" is rerouted. That mode comes from `sim/spec.py:56-59` (TOTAL_FAILURE after `t_on`) at `runner.py:215`, and such a drone runs no controller. The `modes` and `collision` indices are both in `sim.names` order.
* P1 is independent of the sensor. Every pair, dead drones included, is judged by distance (`referee.py:193-199`), and two hulls cannot touch above 2 R_OUT = 0.78 m, below `d_safe` = 1.0 m [I]. [O] Real `Referee` and `TerminalMonitor`: live drone at 0.75 m from the dead hull with its own sensor silent: FAILURE, "P1 violated". Live drone against a bar: FAILURE, "contact". Dead hull against a bar alone: no failure.
* P2 still counts dead drones in the CR (`referee.py:220`), and the live sensor path is unchanged (`terminal.py:75`, `robustness_metrics.py:164,351`).
* Offline replay. `replay_terminal` receives only the logged live edges, so a dead edge cannot enter it. A replayed mode is "DEAD" only if the online one was: the record is written before the step, the online label after it [I, from `runner.py:198-215`]. [O] `analyze_run` on scratch copies of a real med6 run with patched metrics: live edge in a v7 log REPLAY_MISMATCH (my injection left the logged outcome SUCCESS; acceptance item 2 rejects it); live edge in a v6 log SAFETY_FAILURE; dead edge recorded in `dead_drone_contacts` PASS; dead edge left in the old field SAFETY_FAILURE.
* The disclosure is accurate. [O] Scan of all 251 runs of med3-med6: 4 with a contact, all stress, each the faulted drone itself, 10 s or more after its fault, no P1 or P2 violation.
* Limit, not a defect: a contact while a just-failed drone coasts cannot be told from one its controller was already headed for. That concerns only the faulted drone, once per run.

## Minor, none blocks the tag

* **M1.** `dead_drone_contacts` is not read by `analyze_run` (only `referee.py:314` writes it), so such a contact in a matrix scenario would not appear in the class tables. Report a count beside the acceptance.
* **M2.** `GUARANTEES.md:74` ("acoustic data never reach separation...") is still unqualified, while row 79 states that an echo can be relabelled STRUCTURE through the own pose. Add the exception.
* **M3.** The comment at `referee.py:215-216` credits `d_collision`; a contact can occur above 0.6 m and is caught by `d_safe` ([O] d = 0.75: `contacts=0`, `p1=1`). Comment only.
* **Housekeeping.** `git status` shows `formal/results/*` modified, not by me. Tag from a clean tree.

## Limits that stay in force

These are declared; a reader should not take the candidate to cover them.
* No claim heard, a stop behind the bars before the CR, and a slow crossing: M3hs and S2hs. By arithmetic with the 26 s give-up, a crossing below about 0.17 m/s [I].
* A not-engaged dead hull stopped in the masked part of the CR is released after `t_occ_max` (`GUARANTEES.md:78`). Acoustic silence plays no part ([O] above), so I do not count it against the owner's rule; it is the absence of a sonar echo, the v10 rule. The owner may want to confirm that reading.
* A drone silent 70 s leaves the identity premise of the map fit (`perception.py:310`).

## Verified and not verified

Ran at below-normal priority, scripts in the scratch directory, nothing written to `formal/results`. [O] M3hs, S2hs; S2HS_DEPLOYED_PLACEHOLDER CHECK_STATUS_PLACEHOLDER [O] 85 tests pass (`test_status_payload`, `test_structure_aiding`, `test_terminal_and_scope`, `test_scenarios`). [O] Committed `formal/results`: mutex 45, status 28, acoustic 13 rows, none failing. Between `d2ca747` (med6: 54/54 standard PASS by the log) and `02f6a52` the code under test differs only in comments, the referee and stress scenarios.
Not verified: any simulator run; med7; closed-loop effect of N3; the collision sensor of a live drone struck by a dead hull in the engine (`OnActorHit` is per actor [O, source], and P1 covers it anyway); n = 4 at extended horizons.
