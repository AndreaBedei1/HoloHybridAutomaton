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
