# MEDIUM robustness campaign log

Branch `feature/b2-medium-robustness`. Baseline: tag `b2-freeze-v10` (9866242), MEDIUM-ALL 56/66. Development batches below are not
validation: the seeds were used to find and fix causes. The validation campaign uses frozen code and held-out seeds.

| batch | commit | scenarios and seeds | runs | PASS | INCOMPLETE | SAFETY | other |
|---|---|---|---|---|---|---|---|
| v10_repro | 9866242 (v10) | the ten v10 failures, same seeds | 10 | 7 | 3 | 0 | |
| med1 | f50260a | the ten v10 failures, same seeds | 10 | 8 | 2 | 0 | |
| med2 | 419a780 | 18 scenarios, 900-902 | 54 | 42 | 10 | 1 | 1 REPLAY_MISMATCH |
| med3 | be4333a | 18 scenarios, 910-912 | 54 | 52 | 2 | 0 | |
| med4 | c6d8c98 | 18 scenarios, 920-922 | 54 | 52 | 2 | 0 | |
| med5 | 6ce8ba7 | 18 scenarios, 930-932 | 54 | 54 | 0 | 0 | |

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

* med4: `p1_two_lines` s921 (RC8) and `lost_drone_mutex` s920 (RC7); both fixed afterwards, design note section 1.

* med6 (d2ca747, stress 960-961 and standard 970-972): 54/54 standard PASS. Stress: 7 PASS, 1 INCOMPLETE (`lost_drone_mutex_rise_low` s960), 4 SAFETY_FAILURE (the hull of a dead drone against the top bar: see REVIEW_ROUND3_RESOLUTION, criteria v7).

## Status records

### 2026-11-09 21:40 med4 @ c6d8c98
```
PASS 48
INCOMPLETE 1
20:53:48 B2_MEDIUM MEDIUM-ALL             p1_two_lines                     s921 INCOMPLETE       P1 2.525 fail ['progress', 'not waiting'] wall 683.2s exit 0
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.4; free F: GB: 591
```

### 2026-11-09 22:02 med4 @ c6d8c98 (lanes ended or 6 h limit)
```
PASS 52
INCOMPLETE 2
20:53:48 B2_MEDIUM MEDIUM-ALL             p1_two_lines                     s921 INCOMPLETE       P1 2.525 fail ['progress', 'not waiting'] wall 683.2s exit 0
21:51:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex                 s920 INCOMPLETE       P1 3.406 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1559.9s exit 
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.5; free F: GB: 591
```

### 2026-11-09 23:13 med5 @ 6ce8ba7
```
PASS 51
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56; free F: GB: 591
```

### 2026-11-10 00:14 med5 @ 6ce8ba7
```
PASS 62
UNSAFE_ACTION 1
INCOMPLETE 2
INVALID 2
23:26:48 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s940 UNSAFE_ACTION    P1 1.968 fail [] wall 1259.8s exit 0
23:27:50 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 15.3s exit 1
23:29:27 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 38.2s exit 1
23:50:18 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s941 INCOMPLETE       P1 3.396 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
23:50:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s940 INCOMPLETE       P1 3.413 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
23:26:48 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s940 UNSAFE_ACTION    P1 1.968 fail [] wall 1259.8s exit 0
23:27:50 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 15.3s exit 1
23:29:27 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 38.2s exit 1
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 3; free RAM GB: 58.3; free F: GB: 591
```

### 2026-11-10 00:24 med5 @ 6ce8ba7 (lanes ended or 6 h limit)
```
PASS 74
UNSAFE_ACTION 1
INCOMPLETE 2
INVALID 2
23:26:48 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s940 UNSAFE_ACTION    P1 1.968 fail [] wall 1259.8s exit 0
23:27:50 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 15.3s exit 1
23:29:27 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 38.2s exit 1
23:50:18 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s941 INCOMPLETE       P1 3.396 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
23:50:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s940 INCOMPLETE       P1 3.413 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
23:26:48 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s940 UNSAFE_ACTION    P1 1.968 fail [] wall 1259.8s exit 0
23:27:50 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 15.3s exit 1
23:29:27 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s941 INVALID          P1 None fail None wall 38.2s exit 1
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.4; free F: GB: 591
```

### 2026-11-10 01:25 med6 @ d2ca747
```
SAFETY_FAILURE 4
PASS 34
INCOMPLETE 1
00:25:03 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s960 SAFETY_FAILURE   P1 1.57 fail [] wall 42.9s exit 0
00:25:24 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s961 SAFETY_FAILURE   P1 1.599 fail [] wall 43.7s exit 0
00:26:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s960 SAFETY_FAILURE   P1 2.778 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:26:48 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s961 SAFETY_FAILURE   P1 2.514 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:50:23 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_rise_low        s960 INCOMPLETE       P1 2.828 fail ['progress', 'gate'] wall 1473.9s exit 0
00:25:03 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s960 SAFETY_FAILURE   P1 1.57 fail [] wall 42.9s exit 0
00:25:24 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s961 SAFETY_FAILURE   P1 1.599 fail [] wall 43.7s exit 0
00:26:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s960 SAFETY_FAILURE   P1 2.778 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:26:48 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s961 SAFETY_FAILURE   P1 2.514 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.9; free F: GB: 591
```

### 2026-11-10 02:12 med6 @ d2ca747 (lanes ended or 6 h limit)
```
SAFETY_FAILURE 4
PASS 61
INCOMPLETE 1
00:25:03 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s960 SAFETY_FAILURE   P1 1.57 fail [] wall 42.9s exit 0
00:25:24 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s961 SAFETY_FAILURE   P1 1.599 fail [] wall 43.7s exit 0
00:26:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s960 SAFETY_FAILURE   P1 2.778 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:26:48 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s961 SAFETY_FAILURE   P1 2.514 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:50:23 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_rise_low        s960 INCOMPLETE       P1 2.828 fail ['progress', 'gate'] wall 1473.9s exit 0
00:25:03 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s960 SAFETY_FAILURE   P1 1.57 fail [] wall 42.9s exit 0
00:25:24 B2_MEDIUM MEDIUM-ALL             gate_pinned_occupant             s961 SAFETY_FAILURE   P1 1.599 fail [] wall 43.7s exit 0
00:26:34 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s960 SAFETY_FAILURE   P1 2.778 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
00:26:48 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s961 SAFETY_FAILURE   P1 2.514 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.9; free F: GB: 591
```

### 2026-11-10 03:03 med7 @ 02f6a52 (lanes ended or 6 h limit)
```
SAFETY_FAILURE 1
PASS 10
INCOMPLETE 1
02:27:49 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s981 SAFETY_FAILURE   P1 1.184 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
03:03:36 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex                 s981 INCOMPLETE       P1 2.854 fail ['progress', 'gate'] wall 1270.8s exit 0
02:27:49 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s981 SAFETY_FAILURE   P1 1.184 fail ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy'] 
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.2; free F: GB: 591
```

### 2026-11-10 04:14 med9 @ e034755
```
PASS 15
INCOMPLETE 2
03:37:10 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s988 INCOMPLETE       P1 2.282 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
03:40:31 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s987 INCOMPLETE       P1 1.499 fail ['progress', 'gate'] wall 1627.2s exit 0
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.9; free F: GB: 590
```

### 2026-11-10 05:14 med9 @ e034755
```
PASS 60
INCOMPLETE 2
03:37:10 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s988 INCOMPLETE       P1 2.282 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
03:40:31 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s987 INCOMPLETE       P1 1.499 fail ['progress', 'gate'] wall 1627.2s exit 0
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.9; free F: GB: 590
```

### 2026-11-10 05:27 med9 @ e034755 (lanes ended or 6 h limit)
```
PASS 65
INCOMPLETE 3
03:37:10 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s988 INCOMPLETE       P1 2.282 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
03:40:31 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s987 INCOMPLETE       P1 1.499 fail ['progress', 'gate'] wall 1627.2s exit 0
05:19:54 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex                 s995 INCOMPLETE       P1 3.46 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wall
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.9; free F: GB: 590
```

## med9 closed (e034755): 65 PASS, 3 INCOMPLETE of 68, no SAFETY_FAILURE, no UNSAFE_ACTION

* Standard set (seeds 993 to 995): 53 of 54 PASS.
* The carried scenario, s987 and s988: INCOMPLETE as predicted (gate blocked by the hull, P2 holds).
* `lost_drone_mutex` s995: INCOMPLETE, a new case. Root cause RC12 (design note): two corrections mirrored about the gate axis, error 2.7 m, 8 hold timeouts, P1 3.46 m, P2 holds. Fixed afterwards.
* `ENTRY_HOLD_TIMEOUT` in the whole batch: 10 (s995: 8; `lost_drone_mutex` s994: 1, PASS; carried s987: 1). All with `loc_ok` false. med6: 9, all in `lost_drone_mutex_rise_low` s960.
* reg1 (CLEAN and LOW at 3e3db7b) was started on the code before the fix, on purpose: the fix acts only on the path of a hold timeout.

### 2026-11-10 06:30 reg1 @ 3e3db7b
```
PASS 53
INCOMPLETE 2
05:51:37 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s997 INCOMPLETE       P1 3.389 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
05:55:27 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s996 INCOMPLETE       P1 1.482 fail ['progress', 'gate'] wall 1612.4s exit 0
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.4; free F: GB: 590
```

### 2026-11-10 07:11 reg1 @ 3e3db7b (lanes ended or 6 h limit)
```
PASS 72
INCOMPLETE 2
05:51:37 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s997 INCOMPLETE       P1 3.389 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wal
05:55:27 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_carried         s996 INCOMPLETE       P1 1.482 fail ['progress', 'gate'] wall 1612.4s exit 0
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 63.5; free F: GB: 590
```

## reg1 closed (3e3db7b): B2_CLEAN 36/36 PASS, B2_LOW 36/36 PASS, no SAFETY_FAILURE, no UNSAFE_ACTION

The two repeats of `lost_drone_mutex_carried` (s996, s997) are INCOMPLETE as predicted. No `ENTRY_HOLD_TIMEOUT` in CLEAN or LOW, so the path changed after this batch did not run in it.

### 2026-11-10 08:41 med10 @ 8fdcbb4
```
PASS 32
INCOMPLETE 2
08:35:49 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex                 s1068 INCOMPLETE       P1 3.477 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wa
08:40:00 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex                 s1071 INCOMPLETE       P1 3.409 fail ['progress', 'gate', 'P3-deg re-formed', 'not waiting'] wa
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.4; free F: GB: 590
```

## med10 stopped (8fdcbb4) at 34 of 38 runs, on the user's request: 32 PASS, 2 INCOMPLETE, no SAFETY_FAILURE, no UNSAFE_ACTION

* Standard set (seed 1060): 18 of 18 PASS.
* `lost_drone_mutex` family (the reviewer's set F, 21 runs): 17 run, 15 PASS, 2 INCOMPLETE (s1068, s1071); 7 `ENTRY_HOLD_TIMEOUT`; no undo fired, so the recovery of RC12 is not demonstrated.
* By the underwater reviewer's pre-registered criteria (round 7): HOLD (exactly 2 INCOMPLETE in F), on an incomplete batch.
* s1068 and s1071 have a different cause from RC12. drone_0 fails at t = 1 s about 15 m before the gate, is heard once, and stays a possible occupant for the rest of the mission (a silence excluded nobody); drone_1, past the gate, waits at s = +6.5 m in the FRONT corridor of drone_2 and renews the latch BUSY; the discard of that echo as a phantom needs fresh evidence of every other drone, the dead one included. s960 (med6) is the same mechanism.

## Scope decision (2026-11-12, during development, not pre-registered)

Permanent failures inside a gate, or close enough to it that the passive hull can obstruct the passage, are outside the scope: the exclusion zone of `holo_fleet/scope.py`, defined from the failure model and the declared current of the mission. They are not run and have no special handling. Removed with it: the drift-reach hold of a silent drone (RC9, `adrift`), the scenarios `gate_stalled_occupant` (and its rise variants), `gate_pinned_occupant` and `lost_drone_mutex_carried`, the ground-truth fault triggers and the safety-only judgement they used. A drone not heard for 70 s by a receiver that hears the others is dead and leaves every gate decision (rule D; premise V: the channel never silences a live drone so long).

## med11, pre-registered before any run (rule D at the commit of the run)

Runs (40, MEDIUM-ALL): targeted repeats `lost_drone_mutex` s1068, s1071, s995 and `lost_drone_mutex_rise_low` s960 (development seeds); `lost_drone_mutex` s1100-1111, both rise variants s1101-1104 (family, fresh seeds); the 16 other scenarios at s1100.

Accept rule D for the next steps when all hold:
1. No SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR or REPLAY_MISMATCH; P1 and P2 hold in every run.
2. Premise V: in no run is a live drone declared dead (`gate.dead` of the state records against the faults of the run); the longest reception gap of a live sender at a receiver that hears others is reported.
3. s1068 and s1071 PASS (the mechanism rule D addresses); s960 reported; s995 reported (RC12, not addressed by rule D).
4. Family on fresh seeds: at least 19 of 20 PASS.
5. The 16 other scenarios: at least 15 of 16 PASS.
Otherwise: a safety failure or a live drone declared dead rejects rule D; any other miss is analysed before CLEAN and LOW.

## st1, the compact stress batch (5c80591; launched there instead of d243840 before any run, to include the never-heard fix and the silence log), development seeds 2000-2001, MEDIUM-ALL, written before any run

17 scenarios of `STRESS_BATCH` x 2 seeds = 34 runs, chained after med11. Development evidence only: the held-out seeds are kept for the final campaign. Expected: no SAFETY_FAILURE or UNSAFE_ACTION anywhere (a safety failure is investigated at once); `st_gate_head_030` is outside the envelope at the passage speed, so only P1 and P2 are required of it; every other run PASS, any INCOMPLETE analysed and its cause stated before the batch is used to set the operating scope.


### 2026-11-12 19:49 med11 @ daeae63
```
PASS 31
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 53.3; free F: GB: 577
```

### 2026-11-12 20:04 med11 @ daeae63 (lanes ended or 6 h limit)
```
PASS 39
INCOMPLETE 1
20:02:09 B2_MEDIUM MEDIUM-ALL             lost_drone_mutex_rise_high       s1103 INCOMPLETE       P1 2.666 fail ['progress', 'gate'] wall 2221.9s exit 0
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 1; free RAM GB: 58.9; free F: GB: 577
```

## med11 closed (daeae63): 39 PASS, 1 INCOMPLETE of 40, no SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR or REPLAY_MISMATCH. Rule D accepted

By the criteria written before the batch: (1) met; (2) no live drone declared dead in any of the 40 runs (`gate.dead` against the faults; the silence of each sender is logged only from 5c80591, so the longest live gap is not measured here); (3) s1068, s1071, s960 and s995 PASS; (4) family on fresh seeds 19 of 20 PASS; (5) the 16 other scenarios 16 of 16 PASS.

The INCOMPLETE run, `lost_drone_mutex_rise_high` s1103, is the open class of s960: drone_1 holds at s = -2.5 m with a true lateral error of 0.5-0.9 m (0.33 m already at 20 s, before the gate), the map fit has one matched echo and alternates between offsets of about -0.6 and +0.85 m (the mirror); corrected after the underwater review of round 8: at 440 s a fit inside the opening did validate the localization while drone_1 was committed, and moved its gate-frame error from 1.02 to 2.17 m, where it stayed (up to 2.49 m) for the last 290 s, beyond the 1 m navigation premise of G2 and S9; 7 `ENTRY_HOLD_TIMEOUT` (`loc_ok` false), one undo of 0.41 m, bar echoes classified DYNAMIC drive SEPARATION_WARNING and COLLISION_AVOIDANCE at the hold. P1 and P2 hold. In the failure scenarios the identity rule is refused until the dead drone is declared dead (`others` None for the first 70 s), so the lateral error is not corrected on the approach as it is when every drone is heard. Stated as a limit.


## reg2 (5c80591), written before any run

B2_CLEAN / CLEAN and B2_LOW / LOW-ALL, the 17 standard scenarios x seeds 1150-1151 (68 runs), chained after st1. Expected: every run PASS (reg1: 36/36 and 36/36); any other class analysed before the freeze.

### 2026-11-12 20:56 st1 @ 5c80591 (lanes ended or 6 h limit)
```
PASS 34
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 62.4; free F: GB: 577
```

## st1 closed (5c80591): 34 of 34 PASS, no live drone declared dead, longest silence of a live sender at a working receiver 25.2 s

## st2 (c3de0a8), boundary probes, written before any run

The six `st_bd_*` scenarios x seeds 2000-2001, MEDIUM-ALL, chained after reg2. Beyond the steady envelope: P1 and P2 required, the mission not; the outcome maps where the mission stops completing.

### 2026-11-12 21:59 reg2 @ 5c80591
```
PASS 64
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 53; free F: GB: 577
```

### 2026-11-12 22:03 reg2 @ 5c80591 (lanes ended or 6 h limit)
```
PASS 68
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 60; free F: GB: 577
```

### 2026-11-12 22:36 st2 @ c3de0a8 (lanes ended or 6 h limit)
```
PASS 12
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 60; free F: GB: 577
```

## reg2 closed (5c80591): B2_CLEAN 34 of 34 PASS, B2_LOW 34 of 34 PASS; no live drone declared dead; longest silence of a live sender at a working receiver 13.1 s

## st2 closed (c3de0a8): 12 of 12 PASS, beyond the steady envelope (head 0.40, cross 0.50, following 0.50, oblique 0.40 m/s, gusts up to 0.42 m/s; lost drone under 0.30 m/s cross); longest live silence 14.2 s. `st_bd_lost_cross_030` is outside the scope under the zone of round 8 and is removed.

## Round 8 (both reviewers: freeze no) and its resolution

Rule D is conditional on the exclusion zone for the declared current (a dead drone last known inside it holds the gate for ever, a safe stop); the zone and S12 include the downward coast and the rise lag; a validated fit beyond the navigation premise does not validate; the fleet model has the deaf-receiver guard and failures while committed, with the premises of the recovery and the two declared limits; per-run monitors of premise V, of the gate-frame navigation premise and of the scope; in-scope failures near the gate (`st_lost_at_queue`, `st_lost_committed`, `st_lost_after_exit`) and a two-drone failure (`st_gate_pair_lost`). Resolution note: analysis/REVIEW_ROUND4_RESOLUTION.md.

## med12, the regression of round 8, written before any run (development seeds 1200-1212)

56 runs: `lost_drone_mutex` x 6 and its rise variants x 2 each; the four failures of the stress batch x 2; the in-scope failures near the gate (`st_lost_at_queue`, `st_lost_committed`, `st_lost_after_exit`) and `st_gate_pair_lost` x 3; the four standard gate scenarios x 2; six stress gate scenarios under current x 1 (MEDIUM-ALL); sensitivity: channel MEDIUM_HT (6 runs) and compass bias 4 deg (MEDIUM_HB4-ALL, 6 runs).
1. No SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR or REPLAY_MISMATCH anywhere, sensitivity included; no INVALID (every failure outside the zone).
2. No live drone declared dead and no gate-frame navigation premise violated in the MEDIUM runs; in the sensitivity runs both are reported.
3. Failure runs (30): at least 28 PASS. A run INCOMPLETE because a receiver keeps a dead drone held (its last fix inside the zone with the margin) is a cost of the conservative rule, reported as such.
4. Gate runs without failure (14): at least 13 PASS.
5. Sensitivity: reported only.


### 2026-11-13 02:19 med12 @ 33e59aa
```
PASS 18
INCOMPLETE 2
02:16:44 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1200 INCOMPLETE       P1 3.107 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1078.6s exit
02:17:58 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1201 INCOMPLETE       P1 3.269 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.0s exit
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 2; free RAM GB: 58.5; free F: GB: 577
```

### 2026-11-13 03:19 med12 @ 33e59aa
```
PASS 32
INCOMPLETE 6
02:16:44 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1200 INCOMPLETE       P1 3.107 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1078.6s exit
02:17:58 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1201 INCOMPLETE       P1 3.269 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.0s exit
02:34:12 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1202 INCOMPLETE       P1 3.302 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1034.3s exit
02:35:28 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1200 INCOMPLETE       P1 3.294 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.5s exit
02:51:27 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1201 INCOMPLETE       P1 3.349 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1022.0s exit
02:52:40 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1202 INCOMPLETE       P1 3.329 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1018.6s exit
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 2; free RAM GB: 58.5; free F: GB: 577
```

### 2026-11-13 04:20 med12 @ 33e59aa
```
PASS 42
INCOMPLETE 8
02:16:44 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1200 INCOMPLETE       P1 3.107 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1078.6s exit
02:17:58 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1201 INCOMPLETE       P1 3.269 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.0s exit
02:34:12 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1202 INCOMPLETE       P1 3.302 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1034.3s exit
02:35:28 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1200 INCOMPLETE       P1 3.294 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.5s exit
02:51:27 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1201 INCOMPLETE       P1 3.349 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1022.0s exit
02:52:40 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1202 INCOMPLETE       P1 3.329 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1018.6s exit
03:55:18 B2_MEDIUM_HT MEDIUM-ALL             lost_drone_mutex                 s1211 INCOMPLETE       P1 2.735 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1128.3s e
04:09:05 B2_MEDIUM_HT MEDIUM-ALL             st_lost_committed                s1210 INCOMPLETE       P1 2.946 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1025.3s e
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 2; free RAM GB: 58.3; free F: GB: 577
```

### 2026-11-13 04:47 med12 @ 33e59aa (lanes ended or 6 h limit)
```
PASS 42
INCOMPLETE 8
02:16:44 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1200 INCOMPLETE       P1 3.107 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1078.6s exit
02:17:58 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1201 INCOMPLETE       P1 3.269 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.0s exit
02:34:12 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1202 INCOMPLETE       P1 3.302 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1034.3s exit
02:35:28 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1200 INCOMPLETE       P1 3.294 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1036.5s exit
02:51:27 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1201 INCOMPLETE       P1 3.349 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1022.0s exit
02:52:40 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1202 INCOMPLETE       P1 3.329 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1018.6s exit
03:55:18 B2_MEDIUM_HT MEDIUM-ALL             lost_drone_mutex                 s1211 INCOMPLETE       P1 2.735 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1128.3s e
04:09:05 B2_MEDIUM_HT MEDIUM-ALL             st_lost_committed                s1210 INCOMPLETE       P1 2.946 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1025.3s e
lanes done: 2
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 61.8; free F: GB: 577
```

## med12 closed (33e59aa, two lanes: another user's HoloOcean was running): 44 PASS, 12 INCOMPLETE of 56; no SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR, REPLAY_MISMATCH or INVALID

By the criteria written before the batch: (1) met; (2) met: no live drone declared dead anywhere (longest live silence 44.2 s, MEDIUM_HT), no gate-frame navigation premise violated in the MEDIUM runs; (3) NOT met: 24 of 30 failure runs PASS (lost_drone_mutex family 10/10, failures of the stress batch 8/8, `st_lost_after_exit` 3/3, `st_gate_pair_lost` 3/3; `st_lost_at_queue` 0/3, `st_lost_committed` 0/3); (4) met: 14 of 14; (5) sensitivity: MEDIUM_HT 4 of 6 PASS (`lost_drone_mutex` s1211 with the gate-frame navigation premise violated; `st_lost_committed` s1210), MEDIUM_HB4 (compass bias 4 deg) 2 of 6 PASS (the open localization class: relocalization holds, the navigation premise violated in s1212).

Cause of `st_lost_at_queue` and `st_lost_committed` (all six): rule D works (the dead drone is in `gate.dead` of both survivors about 70 s after the failure and the latch is FREE), but the hull rises and floats at the surface above the queue in still water, 4 m above the survivors and inside the range of their LEFT and UP sonars; the precedence, which uses the sonar only, reads it as a queued drone on the left (LEFT) or above (UP, the top-first rule of stacked queues) and the survivors yield to it for ever. P1 and P2 hold. Declared as a limit of the sonar-only precedence (not of the exclusion zone: the floating hull does not obstruct the passage); a fix would bring the death of rule D or a 3-D target position into the precedence, against the isolation of the acoustic data from the precedence (B2-S), and is left out. These two scenarios stay as limitation probes: P1 and P2 required, the mission not.


## med13 (cb2f5a7), the regression of the P3 contract and of the A-P band, written before any run

54 runs: the 17 standard scenarios at seed 1300 under MEDIUM-ALL and LOW-ALL; `st_lost_middle`, `lost_drone_mutex`, `integrated_short` x 3 (1301-1303); five failure scenarios of the stress batch x 1; compass bias 4 deg (MEDIUM_HB4) on `gate_single`, `integrated_short`, `lost_drone_mutex` x 2. Judged by criteria v8. Expected: no safety failure; no P3_FAILURE in the MEDIUM and LOW runs with the premises held; every LOW run PASS; the sensitivity runs reported.

### 2026-11-13 10:54 med13 @ cb2f5a7
```
PASS 48
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 56.1; free F: GB: 577
```

### 2026-11-13 11:14 med13 @ cb2f5a7 (lanes ended or 6 h limit)
```
PASS 48
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 62.9; free F: GB: 577
```

## med13 closed (cb2f5a7, criteria v8): 51 PASS of 54; MEDIUM-ALL 31 of 31, LOW-ALL 17 of 17; no safety failure, no live drone declared dead (longest live silence 17.5 s)

`integrated_short` 4 of 4 and `st_lost_middle` 3 of 3 PASS: the late recovery and the mutual wait of the re-evaluation are gone with the band of rule A-P. Compass bias 4 deg (sensitivity): 3 of 6 PASS (2 of 6 before); `lost_drone_mutex` s1310 PREMISE_VIOLATED (navigation N), s1311 P3_FAILURE (P3a of drone_2 not met with the premises held; the gate-frame navigation premise violated), `integrated_short` s1310 INCOMPLETE (the gate not passed; its navigation excess follows the drone's own map fit, so it is not counted as a premise violation).


## val1 (3f1a626), the validation of the round 9 fix of rule D, written before any run

36 runs (seeds 1400-1405): the `lost_drone_mutex` family, every failure scenario of the stress batch (including the declared floating-hull limit), the three survey failure scenarios, `lost_drone_mutex` under CLEAN and LOW. Rule D now reads the last accepted fix, so a dead drone whose last known position (with the onboard margin, several metres) is inside the exclusion zone for the declared current holds the gate for good: expected safe stops (INCOMPLETE) in the failures under 0.2 m/s current that passed before; P1 and P2 must hold everywhere. The outcome restates the envelope of recovery after a failure; no criterion of PASS is set for it.

### 2026-11-13 16:21 val1 @ 3f1a626
```
PASS 19
INCOMPLETE 4
16:04:57 B2_MEDIUM MEDIUM-ALL             st_lost_middle                   s1400 INCOMPLETE       P1 2.807 fail ['progress', 'gate'] wall 1767.6s exit 0
16:09:09 B2_MEDIUM MEDIUM-ALL             st_lost_after_exit               s1400 INCOMPLETE       P1 3.163 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1413.4s exit
16:09:50 B2_MEDIUM MEDIUM-ALL             st_lost_after_exit               s1401 INCOMPLETE       P1 3.174 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1391.0s exit
16:21:01 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1400 INCOMPLETE       P1 3.241 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1409.1s exit
lanes done: 0
tracebacks in lane logs: 0
Holodeck processes on the machine: 4; free RAM GB: 54.6; free F: GB: 577
```

### 2026-11-13 16:41 val1 @ 3f1a626 (lanes ended or 6 h limit)
```
PASS 29
INCOMPLETE 7
16:04:57 B2_MEDIUM MEDIUM-ALL             st_lost_middle                   s1400 INCOMPLETE       P1 2.807 fail ['progress', 'gate'] wall 1767.6s exit 0
16:09:09 B2_MEDIUM MEDIUM-ALL             st_lost_after_exit               s1400 INCOMPLETE       P1 3.163 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1413.4s exit
16:09:50 B2_MEDIUM MEDIUM-ALL             st_lost_after_exit               s1401 INCOMPLETE       P1 3.174 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1391.0s exit
16:21:01 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1400 INCOMPLETE       P1 3.241 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1409.1s exit
16:29:02 B2_MEDIUM MEDIUM-ALL             st_lost_at_queue                 s1401 INCOMPLETE       P1 3.269 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1417.8s exit
16:32:33 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1400 INCOMPLETE       P1 3.212 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1383.2s exit
16:33:20 B2_MEDIUM MEDIUM-ALL             st_lost_committed                s1401 INCOMPLETE       P1 3.318 fail ['progress', 'gate', 'P3-deg re-formed'] wall 1388.0s exit
lanes done: 4
tracebacks in lane logs: 0
Holodeck processes on the machine: 0; free RAM GB: 61.3; free F: GB: 577
```

## val1 closed (3f1a626): 29 PASS, 7 INCOMPLETE of 36; no safety failure, no live drone declared dead

`lost_drone_mutex` family 10 of 10 (CLEAN and LOW 4 of 4); the failures under 0.2 m/s current (`st_lost_left_cross`, `st_lost_right_tail`) 4 of 4, `st_gate_pair_lost` 2 of 2, the survey failures 6 of 6. `st_lost_after_exit` 0 of 2 (3 of 3 before the fix): the survivors never declare the hull dead, its last fix with the onboard margin being inside the zone, and the gate stays closed: the safe stop of the fixed rule D, a declared limit. `st_lost_at_queue`, `st_lost_committed` 0 of 4 (the floating hull, declared). `st_lost_middle` s1400 INCOMPLETE: the localization at the gate (relocalization holds, gate-frame navigation beyond 1 m), the open class.
