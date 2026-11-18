# Validation experiments (Lorenzo's three requests)

Branch `lorenzo-validations`. The v11 controller is unchanged from `main` (0d376a6). The branch adds the parametric channel, the B2NA baseline flag (default off), the referee monitors, the `em_delay_head` scenario and the formal checks. Pilot: 64 runs, seed 6000, commit 58060df (development data). Held-out: 100 runs, seeds 6100-6104, commit 4d45dd0. The held-out configurations were chosen from the pilot by the rule committed before the pilot was read (`scripts/select_heldout.py`, `analysis/VALIDATION_LOG.md`; the selection is in `results/validation/report/heldout_selection.txt`). No run is pending: 164/164 completed (last at 19:06, 2026-11-16). Raw runs: `results/validation/pilot`, `results/validation/heldout` (not tracked). Tables and figures: `results/validation/report` (held-out), `results/validation/report_pilot`.

The numbers below are from the held-out runs unless marked "pilot". Times are simulated seconds. "Last gate" is the time of the last passage through the gate. Mission completion is not informative: the plan is timed and absorbs the gate delay, so it stays at 150-162 s in gate_single.

## 1. Communication robustness

One factor of the LOW channel at a time. Held-out levels: the extreme of each factor, plus the boundary pair where the pilot showed a change (loss 0.7/0.9, burst 30/60).

| channel (gate_single, 5 seeds) | class | mean gate wait s (range) | last gate s (range) | min separation m |
|---|---|---|---|---|
| LOW (baseline) | 5 PASS | 29.9 (29.0-31.0) | 73.7 (72.4-75.5) | 3.20 |
| beacon period 8 s | 5 PASS | 36.8 (31.5-41.1) | 85.8 (76.2-92.2) | 3.23 |
| latency 2 s | 5 PASS | 44.3 (40.7-45.8) | 101.5 (90.9-105.1) | 3.24 |
| jitter 1 s | 5 PASS | 38.6 (33.6-44.8) | 91.7 (81.6-113.4) | 3.22 |
| random loss 0.7 | 5 PASS | 32.6 (29.9-35.0) | 78.1 (73.5-82.5) | 3.25 |
| random loss 0.9 | 5 PASS | 41.2 (33.2-50.7) | 94.4 (81.6-109.4) | 3.24 |
| bursts of 30 s every 90 s | 5 PASS | 37.6 (30.4-45.3) | 88.2 (74.5-97.5) | 3.21 |
| bursts of 60 s every 90 s | 3 PASS, 2 P3_LATE | 47.9 (30.2-66.5) | 107.1 (74.6-128.5) | 3.05 |
| asymmetric loss 0.9 | 5 PASS | 31.7 (30.0-33.7) | 79.1 (74.7-84.4) | 3.26 |
| blackout | 5 PASS | 37.9 (31.4-45.7) | 93.2 (83.6-105.2) | 3.27 |

lost_drone_mutex (one drone dies, 5 seeds): LOW, loss 0.7 and loss 0.9 are 5/5 PASS each, with P3c MET. Under blackout the result is 5/5 PREMISE_VIOLATED. With no packet at all, the vacancy of the dead drone is declared from silence only at 129-131 s, beyond its 48 s deadline: premise A of P3c is violated, by design of the channel. formation_square: 5/5 PASS under LOW and under blackout.

Safety: P1 and P2 held in all 164 runs (minimum true separation 2.86 m held-out, 2.96 m pilot).

Findings:
- A finite degradation with no other fault caused no P3 failure in the 80 held-out communication runs. The effect is longer waiting at the gate: up to +18 s of mean wait and +33 s on the last passage. 2 of 5 runs with 60 s bursts are P3_LATE: drone 2 re-formed after its deadline.
- Packet losses do produce false vacancies of live drones, but the drones are later re-included: 15 in the five runs with 60 s bursts, 3 with loss 0.9, 1 with asymmetric loss 0.9. In those configurations the fairness premise (a fix of every drone at every receiver within 35 s) is violated in 4-5 of 5 runs. The longest gaps are 59-61 s for the 60 s bursts.
- One P3 failure did occur, in the pilot: loss 0.9, lost_drone_mutex, seed 6000, P3_FAILURE through P3c. Drone 1 declared the slot of the live drone 2 vacant at 45.2 s, after a silence longer than the 45 s vacancy timer. The run's longest reception gap was 49.1 s, so the fairness premise was violated. The five held-out seeds of the same configuration passed. So a strong random loss alone can cause a P3 failure, but only by producing a silence longer than the premises allow. It was rare here (1 of 6 runs) and is outside the guarantee, not a contradiction of it.
- A complete loss (blackout) stops the declaration of a real vacancy in time (premise violated, 5/5). The progress of the live drones still completed.

Lorenzo's hypothesis therefore holds in this sweep, with one qualification. Finite delays, jitter, period, bursts and asymmetry only lengthened waiting and completion. Prolonged or near-total loss is what breaks P3, through a silence longer than the premise allows: a 60 s burst gave P3_LATE (2/5), loss 0.9 gave a P3c failure (1/6), blackout gave PREMISE_VIOLATED (5/5). The sweep covers one factor at a time and 5 seeds per configuration, so rare failures at the other levels cannot be excluded.

Figure: `results/validation/report/comm_sweep.png` (last gate passage, mean and range, against the level of each factor).

## 2. Proof-to-runtime audit of premise C

### Episode counts
An episode is a commit edge of one drone, counted for each other drone that is queued and not committed at that instant; the delay runs until that drone's latch is not FREE. The 448 final runs give 1007 episodes. The figure of 688 is the subset B2_CLEAN 78 + B2_LOW 78 + B2_MEDIUM 492 + B0 40. The remaining 319 are the ablation B2NP_MEDIUM (296) and the sensitivity B2_MEDIUM_HT (23). Both figures use the same definition.

Results over the 1007 episodes:
- 972 within 5 s, none beyond 32 s, 3 censored;
- maximum per protocol: CLEAN 1.7 s, LOW 2.6 s, MEDIUM 7.3 s, B0 14.4 s, B2NP 6.6 s, HT 5.2 s;
- delays above 10 s occur only in 6 B0 runs, which have no beacon and rely on sonar evidence only.

Validation runs: 80 episodes in the pilot and 170 held-out, maximum 12.1 s, with delays above 10 s only under latency 2 s, loss 0.9 and blackout. There was no concurrent commitment in the validation runs.

### Discrepancy with the fleet model
The composition model (`formal/check_fleet.py`) lost P2 with a latch one step (5 s) late. The cause was the model, not the bound. Its entry from the stage-2 hold into the CR checked only for a live drone already in the CR. The code also holds the entry on a heard claim (COMMITTED/ADMITTED) of another drone: `entry_ok` in `gate_perception.py`, lemma S1 of `check_status.py`. The model now has that guard; the bound was not reduced. Results:
- **F1d1, F1d2 (n = 3, with the sonar precedence M2):** with the latch and the stage-2 check both learning of a claim 1 or 2 steps (5 or 10 s) late, two drones are never in the CR together (bounded, from the launch).
- **F1d3:** at 3 steps two drones can be in the CR, so the bound is tight.
- **F1d1n, F1d2n (no precedence, as for two queued drones out of sonar range):** P2 holds at 1 step and fails at 2.
- **Continuous reading:** P2 holds while the claim delay does not exceed the shortest time from commitment to admission. That time is one model step, plus one more step when the precedence separates the two commitments. By geometry at v_pass it is 5.2 s for n = 2, 7.7 s for n = 3 and 10.1 s for n = 4, assuming a straight path and no jumps of the estimate.
- **Mutations F1c1, F1c2, F1m:** without the stage-2 claim guard, a delay of one step already lets two drones into the CR.
- **Other checks:** F0 (inductive invariant) and F2 (bounded recovery, tight) still hold for n = 2, 3, 4.

The 32 s of S2 is a different quantity. It bounds how long a report of "not engaged" may exclude a corridor echo, and it is proved in the timed two-drone model of S2. It does not bound the delay of a claim between two drones that commit together. The 32 s statement in `docs/GUARANTEES.md` has been restated accordingly.

### st_lost_middle s5201 (B2_MEDIUM)
The middle drone was dead. Drones 0 and 2 were at the outer queue points, 7 m apart and out of sonar range of each other, so the precedence gave no relation.
- **Concurrent commitment:** drone 2 committed at 22.2 s and drone 0 at 22.7 s, both with a FREE latch. The extended monitor shows that after the second commit, drone 0 learnt of drone 2's claim in 0.3 s and drone 2 learnt of drone 0's in 0.6 s. Neither was admitted before learning of the other. Both held at stage 2 (entry_ok false).
- **Occupancy:** the referee never saw more than one drone in the CR (occupancy ≤ 1, P2 true, minimum separation 2.60 m).
- **Entry without admission:** while holding, drone 2 was pushed forward by collision avoidance and separation warnings against drone 0. It entered the CR without an ADMIT (CR occupied from 53.7 s), crossed the centre at 63.5 s and left. Drone 0 was admitted at 80.0 s, after drone 2 had passed.

This was a concurrent commitment, not a simultaneous occupancy of the CR. The P2 margin came from the stage-2 claim check of the drone that stayed outside, and from the claim the moved drone keeps inside the keep-out box (`report_phase`, S9).

Formal follow-up (`formal/check_fleet.py`, push option):
- **F0p, F2p:** an entry into the CR that bypasses the stage-2 check, by a committed drone at any step, keeps the inductive invariant and the bounded recovery for n = 2, 3, 4. With claims learnt in time, no other drone is in the hold or the CR.
- **F1p (declared limit):** after a concurrent commitment (claims one step late), two such entries at once put two drones in the CR in the model.
- **F0q (declared limit):** a queued drone moved into the CR breaks the invariant.
- **F1pa (scope):** an antisymmetry lemma on the deployed escape planner would exclude the double entry, but it does not hold. Two drones at their hold points that see each other, with the hold pulling back or holding, can both be given a forward component in 8 mirrored patterns. Most of these components are small (up to 0.05 m/s in the band of the warning filter).

Runtime audit of every CR entry (`scripts/audit_cr_entries.py`, `results/validation/audit_cr_entries.json`): 612 runs (448 final, 164 validation), 1057 entries. 1055 were ADMITTED, 2 PUSHED (both drone 2 of s5201, an entry and a re-entry) and 0 UNCOMMITTED; at none of them was another drone inside the CR. No P2 violation and no safety defect were demonstrated, so the v11 controller is unchanged. A constraint in the escape and warning filter that keeps a held drone out of the CR would close the gap, but it is not part of v11. Regression tests: `tests/test_cr_entry.py`.

### What is proved, conditional, experimental
- **Proved (Z3, the abstraction in 5 s steps):**
  - P2 for n = 2, 3, 4 at every time (F0, k-induction), with claims learnt within the step, also with entries of committed drones that bypass the stage-2 check (F0p, F2p);
  - P2 from the launch for n = 3 with claims learnt up to 10 s late (with M2) or 5 s late (without M2), bounded, tight, when every entry passes the stage-2 check;
  - bounded recovery F2.
- **Conditional:**
  - on refined premise C: two drones that commit together learn of each other's claim before either is admitted. It is now monitored per run (`commitment_visibility.concurrent`, `admitted_without_evidence`);
  - on the latch, freshness and precedence lemmas M3, S1, S2 and M2;
  - on premises V, H and the fairness of the fixes.
- **Experimental only:**
  - entries into the CR that bypass the stage-2 check: 2 of 1057, never with another drone inside; two at once after a concurrent commitment are not excluded by the model (F1p, F1pa);
  - the delays at runtime: 1007 + 250 episodes, maximum 14.4 s (B0) and 7.3 s with B2 in the final campaign;
  - one concurrent commitment in 1257 episodes, learnt in 0.3-0.6 s;
  - P2 under a hold overridden by collision avoidance (s5201);
  - P2 in the validation runs (164/164).

Checks re-run: formal suite 291/291 as expected (`formal/results/SUMMARY.md`), unit suite 47 files green, premise C re-audit of the 448 final runs (`results/validation/audit_premise_c_final448.json`) and of the validation runs (`audit_premise_c_pilot.json`, `audit_premise_c_heldout.json`).

## 3. Emergence: B2 against B2NA

Drone 2 of the 4-drone square is pushed back by a jet between t = 8 and 15 s. In `em_delay_head` the jet is head-on, (-0.70, 0, 0) m/s; in `lost_drone_rejoin` it is (-0.70, 0.55, 0). B2NA is the same plan with the neighbour-based adaptation switched off. Five held-out seeds per cell; values are mean (range).

| scenario | protocol | neighbours' reaction after jet onset s | adaptation, neighbour-seconds in [8, 45] s | progress yielded by the neighbours m | min reference speed m/s | max formation error m | recovery after the jet s | mean error after the jet m |
|---|---|---|---|---|---|---|---|---|
| em_delay_head | B2 | 4.7 (1.7-5.7) | 48.3 (39.0-58.9) | 2.48 (1.95-2.94) | 0.20 | 2.33 (2.30-2.38) | 11.0 (10.2-12.1) | 0.36 (0.34-0.39) |
| em_delay_head | B2NA | none | 0 | 0 | 0.30 | 2.74 (2.68-2.79) | 17.0 (16.1-17.8) | 0.57 (0.50-0.66) |
| lost_drone_rejoin | B2 | 4.7 (4.7-4.7) | 67.7 (63.3-70.4) | 4.58 (4.10-5.23) | 0.20 | 2.83 (2.73-2.94) | 14.7 (13.4-15.9) | 0.52 (0.45-0.59) |
| lost_drone_rejoin | B2NA | none | 0 | 0 | 0.30 | 3.55 (3.51-3.59) | 24.2 (23.8-24.4) | 0.86 (0.84-0.88) |

All 20 runs are PASS; the minimum separation is 2.86 m. With B2, the three neighbours lower their reference speed from 0.30 to 0.20 m/s about 5 s after the jet starts. That is when the delayed drone's acoustic fix shows it behind its slot. They yield 2.5-4.6 m of progress, and the speed returns to nominal by about 30-37 s. With B2NA they keep 0.30 m/s throughout. The adaptation lowers the peak formation error by 0.41 m (em_delay_head) and 0.72 m (lost_drone_rejoin) and shortens the recovery by 6.0 s and 9.4 s. The seed ranges of B2 and B2NA do not overlap on any of these measures. The response is local: each neighbour uses only its own fixes of the delayed drone. Figure: `results/validation/report/emergence.png` (formation error and the neighbours' reference speed, B2 blue, B2NA red).

## Limitations
- One factor at a time, 5 held-out seeds per configuration, and only the levels selected by the rule. Interactions between factors and rare events at the other levels are not tested.
- The selection rule's duration criterion used the scenario's end time. That time is fixed by the plan, so the criterion was insensitive; the selection was driven by completion, P3 and the extreme levels.
- Fairness is logged, not used in the classification. The P3 failure of the pilot is classified P3_FAILURE although its fairness premise was violated.
- The fleet model is an abstraction in 5 s steps. The correspondence between one step and the shortest commit-to-admission time is argued from geometry, not checked.
