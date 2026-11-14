# Independent review round 2: resolution (commit reviewed: c6d8c98)

The two reports (`review_round2_formal.md`, `review_round2_underwater.md`) were written without either reviewer seeing the other.

## Formal / hybrid automata

| Finding | Resolution |
|---|---|
| F1 BLOCKER, no relation and no latch input in a 1 m band | Fixed: the band feeds the latch. M3 and S2 model the code's predicate, new M3h and S2h with the crossing drone stopped before the CR, mutations M3b and S2b, S6b on the real class |
| F2 BLOCKER, silence overrides a positive echo | Fixed: `no_fleet_drone` accepts only fresh status (S11, S11m). `_other_regions` kept, with the reason in the design note: only the own pose reads it, the change is bounded |
| F3 keep-out claim bypassed at the next gate | Fixed: the claim precedes a verified passage and is about the current gate. GP_PASSED no longer discards an echo once stale |
| F4 S9 tautological | Replaced by the sender rule check with a mutation; the filter is no longer claimed as a barrier |
| F5 filter edits the avoidance output | Fixed: mission velocity only |
| F6 S10 checks a removed guard | S10 deleted; a drone back in the box reports a claim |
| F7 checks prove less than labelled | S5, S6, S2 relabelled, S8 reads the deployed value, B2-PEg with the status off, mutations S1m, S4m, S11m. S7 still has no mutation |
| F8 lam | Kept: sign and convergence confirmed by the reviewer |
| F9 chatter | Not changed: present in v10, safety-neutral |
| F10 a drone dying committed blocks the gate for good | By design (silence never clears a claim); disclosed |
| F11 same-ping fix does not supersede status | Kept: a position region cannot clear a CR, that is why the status exists |
| F12 deadline changed with version v5 | CRITERIA_VERSION v6 |
| F13 stale documents | GUARANTEES, comments, design note updated |

## Underwater / acoustics

| Finding | Resolution |
|---|---|
| F1 rise speed | Text corrected; sensitivity scenarios at the ends of the range |
| F2 no pinned occupant in any run | New stress scenario `gate_pinned_occupant` |
| F3 dead drones drift | New stress scenario `lost_drone_mutex_carried`. No drift-reach bound: a drifting body in the CR is a corridor echo, and silence no longer discards it |
| F4 loss process | Sensitivity channel `MEDIUM_HT` (per-link heavy-tailed OFF periods) |
| F5 reflection geometry | Not implemented; declared limit (ACOUSTIC_CHANNEL_MODEL section 10) |
| F6 H6 hardware | Declared (GUARANTEES, ACOUSTIC_CHANNEL_MODEL section 10) |
| F7 bench check at 0.05 | Left failing and reported; commanded values chosen so that the measured speeds are the ends of the range |
| F8-F14 minor | Declared where they limit a claim; F9 bounded by the median and the integrator cap |
