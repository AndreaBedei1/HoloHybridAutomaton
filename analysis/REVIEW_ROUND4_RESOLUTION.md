# Independent review round 4: resolution (reviewed commit 02f6a52)

The two reports (`review_round4_formal.md`, `review_round4_underwater.md`) were written without either reviewer seeing the other.

The formal reviewer lifted "do not freeze": no blocker at 02f6a52. The underwater reviewer made the freeze conditional on R1 and R2.

## Formal / hybrid automata

| Finding | Resolution |
|---|---|
| M1 `dead_drone_contacts` not in the analysis | Reported per run as `P1.dead_hull_contacts`; to be counted beside the acceptance |
| M2 GUARANTEES row on acoustic isolation unqualified | The exception of the map fit stated |
| M3 comment on `d_collision` | Corrected |
| Limit: a not-engaged dead hull stopped in the masked part is released after `t_occ_max` | Superseded inside the reach by `AcousticExit.adrift` |

## Underwater / acoustics

| Finding | Resolution |
|---|---|
| R1 MAJOR, a live drone admitted into a CR holding a dead hull (med7 `lost_drone_mutex_carried` s981, P2 violated at 48.2 s, separation 1.18 m), confirmed on the logs | Fixed: `AcousticExit.adrift`. A silent drone whose last fix is within the reach of a passive hull (water estimate plus 0.10 m/s, until it has risen over the top of the CR at 0.05 m/s) holds the latch and the admission until it is heard again. S12 on the real class (1500 paths, 0 missed), mutation S12m, `tests/test_adrift.py`. The mechanism is stated in GUARANTEES and in `gate_perception.py` |
| R2 the s960 conclusion | Reworded (round 3 resolution) |
| R3 `gate_pinned_occupant` exercises only the claim path | Stated in the pre-registration |
| R4 corrections | 29.3 s attribution, 240 runs with 5 contacts, `_axial` declared in the docstring and GUARANTEES |
| R5 criteria v7 fair, runs continue past contact on unvalidated contact mechanics | Accepted and declared here |

The reach rule is new code, so both reviewers are asked to check it, and the standard set is rerun for its liveness cost.

## Round 5 (reviewed at ef54530; reports `review_round5_formal.md`, `review_round5_underwater.md`)

| Finding | Resolution |
|---|---|
| Formal N6 MAJOR, the reach could shrink with no new fix (a falling water estimate) | Fixed: the hold rests on the fix (`adrift_held`), S12 extended with a falling estimate, unit test |
| Formal N7, a deaf receiver ended the hold and left BUSY with `entry_ok` true | Fixed: a deaf receiver keeps the holds it has and starts no new one; declared |
| Formal N8, the hull is passive from its last fix | Declared in GUARANTEES |
| Underwater, premises and transients | Figures stated in GUARANTEES |
| Underwater, the adrift set was not logged | Logged per drone in the state record (`gate.adrift`) |
| Underwater conditions on med8 | Checked on the final runs |
| Count of contacts (the formal reviewer recounted 251 and 4) | 240 run directories and 5 contacts: 251 counts the retried attempts, and the fifth, med5 s941, has no metrics file because the run was INVALID |

Disclosures. The round 4 formal file has two unfilled placeholders; the reviewer gave the values afterwards: S2hs at the deployed 2.5 s SAT in 174 s, `check_status.run()` all PASS. In round 5 the formal reviewer ran by mistake a script of the other reviewer found in the shared scratch directory; it read med6 and med7 logs, wrote nothing, and the reviewer did not use the output. Both initial assessments had been delivered four rounds earlier.

Batch med8 was stopped and is not evidence: by an error of the orchestration two launches ran at once on 32116e6 and ef54530 (`results/robustness/raw_dev/med8_mixed_stopped`). The one finished run read from it is the source of RC11.

## Round 6, formal (reviewed at e034755; report `review_round6_formal.md`)

| Finding | Resolution |
|---|---|
| N6 to N8 | Closed by the reviewer |
| N9 MAJOR, one step past the centre disabled the hold for the whole episode | Fixed as the reviewer proposed: released when the estimate falls back before the CR; unit test of the spike |
| N10 the threshold against the declared navigation error | Fixed: `nav_err_m - cr_half_len`, inside the CR under the declared error |
| N11 the rule was neither declared nor logged | Declared (GUARANTEES), logged as `PAST_CENTRE` |
| N12 no verified passage on this branch | Declared (GUARANTEES) |
| `formal/results` not regenerated | Generated at the tag, from a clean tree |

## Round 6, underwater (reviewed at e034755; report `review_round6_underwater.md`)

| Finding | Resolution |
|---|---|
| Condition 1, carried scenario at the deployed code | Lifted by the reviewer |
| Condition 2, the standard set | Finished: 53 of 54 PASS, and one new INCOMPLETE, `lost_drone_mutex` s995. Root cause RC12, fixed, and sent back to both reviewers |
| The carried scenario at the frozen commit | Part of the pre-registered stress set, run on the frozen code |
| MINOR compass bias depending on thruster current | Declared (GUARANTEES, heading row) |
| MINOR the past-centre rule has no lateral term | Declared (GUARANTEES, past-centre row) |

Disclosure. `check_status.py` has mutation runs for S1, S3, S4, S9, S11 and S12. S5, S6, S6b, S7 and S8 have none.

## Round 7, formal (reviewed at 9e9cade; report `review_round7_formal.md`)

| Finding | Resolution |
|---|---|
| N13 MAJOR, the undo step fed position-gated transitions of the hold on the next tick, and the give-up request of the same tick | Fixed: the step waits until the commitment is cleared (give-up edge). The reviewer proposed clipping the along-axis part and blocking the latch; this removes the interaction instead, since `past_centre`, `abort_request` and the hold exist only while committed |
| False claims: the GUARANTEES row, `GUARANTEES.md:41`, the design note and `config.py` on `MAX_CORR_M`, the docstring on dead reckoning | Corrected |
| Tests: 5 of 11 mutations survived; identity-rule path, tick after an undo, `entry_ok` false | Added, including a noise-free wrong identity-rule correction through to its undo |
| `unconfirmed` not cleared by a measured crossing | Fixed |
| The RC12 statistic does not reproduce | Both counts stated; the conclusion stands |
| `sighting` and `hold_pos` are not shifted by a correction | Declared here: true of every correction of the estimate, not new with the undo |

## Round 7, underwater (reviewed at 9e9cade; report `review_round7_underwater.md`)

| Finding | Resolution |
|---|---|
| The round 6 condition is not met: 9 `ENTRY_HOLD_TIMEOUT` in the 18 standard gate runs, s995 a new mechanism | Accepted. Fixed below; the condition is re-evaluated on the targeted batch |
| R1 MAJOR, the undo should cover only the corrections since the last commit | Done (`start_commitment`, reset on the commit edge) |
| R2 MAJOR, cap the step of an identity fit at about 0.5 m | Done (`IDENTITY_STEP_M`); the axial path keeps its bound on the total |
| R3 MAJOR, the batch must show `lost_drone_mutex` completing over enough seeds | The batch has 12 seeds of it plus 4 each of the two rise variants; completion, timeouts and recurrence of a wrong fit reported |
| A hold without validation does not refute a correction | Wording corrected: a recovery heuristic |
| MINOR figures, corrections by type and initial error, s960 declared | Done (design note, GUARANTEES); the refusal reason is now logged |

## Round 7 re-check, underwater (at 8fdcbb4; appended to `review_round7_underwater.md`)

| Finding | Resolution |
|---|---|
| M1 MAJOR, the undo also took corrections made after the timeout | Fixed: the sum is sealed at the timeout and undone at the give-up edge; a validated fit or a measured crossing drops the seal |
| M2 MAJOR, the figure 14/10/4 and "did restore it" not reproducible | Removed; the row states the one case that matters in replay |
| R3, criteria pre-registered by the reviewer before any med10 result | To be judged on med10. Runs with a correction between a timeout and its give-up edge need repeating on the fixed code |
| MINOR wording (reach per step, one-step truncation, the noise-free case is the far side of a bar, 0.73 m, the `GAIN` comment) | Done |

Disclosure. The reviewer's priority call (`SetPriorityClass` through `ctypes` without declared types) fails silently on this machine: its scripts ran at normal priority in this re-check and in earlier rounds where it used that call, against the instruction. It lowered the long replay by hand after about 13 minutes; that replay held about 15 GB with 44 GB free. Whether it slowed a simulation cannot be told.

## Round 8 (reviewed at c3de0a8; reports `review_round8_formal.md`, `review_round8_underwater.md`)

| Finding | Resolution |
|---|---|
| B1 (both): rule D drops the claim of a hull pinned in the CR after 70 s (the RC9 path); the zone covers the gate phase under current | Rule D only for a drone last known outside the zone for the declared current, grown by the fix region, the own bound and one TDMA period; otherwise the claim stands for ever (safe stop). The exposure is stated in GUARANTEES |
| Formal B2: no rule D at n = 2; failures while committed outside the zone | Modelled as the code (deaf-receiver guard, claim kept); recovery proved for n = 2 without failures while committed and for n >= 3 with at most n - 2 failures; the rest declared and checked as limits (F2s2, F2s3). In-scope failure scenarios near the gate and a two-drone failure added |
| Formal M1: the latch premise of S2 (32 s) | Stated as premise C of P2 |
| Formal M2: the model leaves out the localization block and the release delays | Stated as not modelled; the open class stays empirical |
| Formal M3: fairness of 10 s refuted (26 s in st1) | 35 s window; bounds re-computed (26, 40, 52 steps) |
| Formal M4: S12 and the zone ignored the downward coast and the rise lag | Both in the zone and in S12 (from above the top too); mutation S12l |
| Formal M5, underwater: no per-run scope check | The referee classes a run with a failure inside the zone INVALID |
| Underwater M1: s1103 was validated by a wrong fit while committed | Log corrected; a validated correction above nav_err_m no longer validates and is capped at 0.5 m; per-run monitor of the gate-frame navigation premise |
| Underwater M2: the open class is mostly compass bias, and rule D's 70 s blocks the identity correction | Stated; sensitivity on the compass bias to be run before the freeze |
| Underwater M3: premise V is a premise of P2; long-tail channel | Listed in the P2 table, monitored per run; MEDIUM_HT sensitivity to be run before the freeze |
| Underwater M4: rise speed above the model value, leak, coast bound | v_rise_min 0.045 m/s; a leak outside the model; coast measured in the simulator (at most 0.39 m at 0.39 m/s), not validated on a vehicle |
| Underwater M5: two-drone fleets | As formal B2 |
| Underwater M6 and formal M6: sample sizes, development decisions | The final campaign sizes follow the reviewers' criteria; the scope and rule D are declared as development decisions |

## Round 9, formal (reviewed at 176b8b8; report `review_round9_formal.md`)

| Finding | Resolution |
|---|---|
| B1: rule D read the zone from the fix list, pruned after 30 s, so at 70 s a hull last heard in the CR was declared dead and released | Fixed: the last accepted fix (kept for ever); never heard = absent from `heard`; heard but never located holds the gate. S4 now runs on real receptions, mutation S4b (the round 8 code) is caught; the test of rule D drives the tracker for 75 s |
| M1: margin of rule D | Motion from the fix to the last reception plus one period, vertical margin, no fall back on a rejected fix |
| M2: F2 contradicted by med12 (floating hull) | Premise H stated, declared limit F2s4 (SAT); GUARANTEES and the P3 document conditional on it |
| M3: fairness and premise C unmonitored | Fairness monitored per run (`fairness`, longest all-fresh gap); premise C stated, not monitored (a declared gap of the evidence) |
| M4: p3 against its document | Premise N at the controller band e_ok = 0.5 m; premise A second clause; n = 2 failure while committed not admissible; NOT_ADMISSIBLE classed PREMISE_VIOLATED (premise S); docstring of D_b |
| M5: deadlocks classed PREMISE_VIOLATED | Gate-frame navigation attributed per drone (external only if some drone's first excess does not follow its own correction); W judged on the mission velocity; O and the progress of a blocked passage stay as declared |
| M6: P3_FAILURE wording | Split: P3_LATE (met after the deadline within the run, premises held) and P3_FAILURE (never met) |
| M7: stale formal results | Regenerated and committed at the freeze |
| MINOR | Declared in the P3 document |

