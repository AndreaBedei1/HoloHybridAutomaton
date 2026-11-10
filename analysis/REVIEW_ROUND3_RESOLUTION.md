# Independent review round 3: resolution (reviewed commit 6ce8ba7, fixes in d2ca747 and in the commit that adds this note)

The two reports (`review_round3_formal.md`, `review_round3_underwater.md`) were written without either reviewer seeing the other.

## Formal / hybrid automata

| Finding | Resolution |
|---|---|
| F2 BLOCKER, silence still releases the sonar latch | Fixed: `status_excluded` excludes nobody by silence; a dead drone that was not engaged is released by `t_occ_max` |
| N1 M3h and S2h horizon artefacts | Scope counterexamples M3hs and S2hs added and labelled: the stop before the CR with no claim heard, in the views whose mask starts before the CR. The label says what remains |
| F6 a drone back in the box reports GP_PASSED | Fixed: `passage.gate_report` also tests the box of the verified gate |
| N2 `_axial` | Claims corrected; total correction capped until a validated fit; `recent_ax` cleared on every path |
| N3 rejoin lane against `lateral_far` | Fixed: the lateral residual stands down during a give-way |
| N4 liveness cost of F1 and F2 | Observed once (med6 `lost_drone_mutex_rise_low` s960, below); kept, because it is the price of the owner's rule on silence |
| N5 sender rule with more than one gate | Not changed: no scenario has one; declared |

## Underwater / acoustics

| Finding | Resolution |
|---|---|
| N1 masked half of the CR | Declared (GUARANTEES); the `evaluate` path no longer uses silence |
| N2 `lost_drone_mutex_carried` did not test F3 | Fixed: the on-axis drone fails, by its own queue trigger, at the low rise rate |
| N3 `_axial` elimination only, 120 deg cone | Declared in the docstring and GUARANTEES; every correction is logged in `nav_aid` |
| N4 statistics and `MEDIUM_HT` text | Corrected here (`config.py`, design note, section 10) |
| N5 no front failure at the low rise rate | New stress scenario `lost_drone_front_rise_low` |
| N6 F9 not bounded by the median | Declared (section 10): only the 2 m cap bounds it |
| N7 phantom discard off after a failure | Observed once, as formal N4 |
| Acceptance gaps | Like-for-like subset and stress set pre-registered (ACCEPTANCE_CRITERIA v6 and v7) |

## Findings from the stress runs of med6 (d2ca747)

The conclusions below were superseded in part by round 4 (`review_round4_underwater.md`, R1, R2): see RC9 in the design note.

* 4 SAFETY_FAILURE, all the hull of a dead drone against the top bar. Criteria v7 records this and no longer ends the run. Disclosed in ACCEPTANCE_CRITERIA. No live drone had touched anything, but these runs stopped at the contact with a committed drone 2-3 m from the CR: round 4 (R1) shows what it would have done.
* `gate_pinned_occupant` started its drone with the navigation at the launch point of the plan, 9 m from the gate, so it reported not being there: scenario defect, fixed, with a check over every scenario.
* `lost_drone_mutex_rise_low` s960 INCOMPLETE: safe, lateral error 0.45-1.13 m, the dead hull on the surface 10-13 m away. Corrected by round 4: the direct cause at the hold point is `loc_ok` False (1871 of 1879 hold steps): the map fit is good but refused, with `identified` False in 78 % of them; why `identified` stays False is not established (`others` is not logged). It is not shown to be a cost of the rule on silence.
