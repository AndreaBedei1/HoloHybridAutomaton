# MEDIUM robustness campaign log

Branch `feature/b2-medium-robustness`. Baseline: tag `b2-freeze-v10` (9866242), MEDIUM-ALL 56/66. Development batches below are not
validation: the seeds were used to find and fix causes. The validation campaign uses frozen code and held-out seeds.

| batch | commit | scenarios and seeds | runs | PASS | INCOMPLETE | SAFETY | other |
|---|---|---|---|---|---|---|---|
| v10_repro | 9866242 (v10) | the ten v10 failures, same seeds | 10 | 7 | 3 | 0 | |
| med1 | f50260a | the ten v10 failures, same seeds | 10 | 8 | 2 | 0 | |
| med2 | 419a780 | 18 scenarios, 900-902 | 54 | 42 | 10 | 1 | 1 REPLAY_MISMATCH |
| med3 | be4333a | 18 scenarios, 910-912 | 54 | 52 | 2 | 0 | |

Causes closed between batches:

* med2 to med3: DROP_BACK/PASS limit cycle at a static reference (most of the med2 INCOMPLETE runs); gate_single s902 contact caused by the crossing
  correction moving the along-track estimate (SAFETY_FAILURE).
* med3: `formation_gust` s912 and `lost_drone_rejoin` s911, one cause (RC6). Compass biases of opposite sign leave a relative
  navigation error of 1-2 m after the run; the lanes are nearer in reality than planned; the drone returning from behind enters the
  separation band of the neighbour on the next lane and stops 2.3-3.9 m short of its slot until the deadline.
  Fix: `Perception.lateral_far`, the lateral residual of the identified fixes compensated for the own displacement from the slot,
  integrated into the existing lateral offset. The safety layer is untouched.
* med2 `line_parallel_mutex` s901 REPLAY_MISMATCH: the referee logged times at 1 ms and the replayed hold timer fell short of
  `t_ok_hold` at the last sample. Fix: times logged at 1 microsecond; no tolerance added to the criteria.

## Status records
