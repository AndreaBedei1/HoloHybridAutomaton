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
