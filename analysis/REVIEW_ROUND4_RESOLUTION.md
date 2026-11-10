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
