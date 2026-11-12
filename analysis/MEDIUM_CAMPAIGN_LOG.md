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

## st1, the compact stress batch (d243840), development seeds 2000-2001, MEDIUM-ALL, written before any run

17 scenarios of `STRESS_BATCH` x 2 seeds = 34 runs, chained after med11. Development evidence only: the held-out seeds are kept for the final campaign. Expected: no SAFETY_FAILURE or UNSAFE_ACTION anywhere (a safety failure is investigated at once); `st_gate_head_030` is outside the envelope at the passage speed, so only P1 and P2 are required of it; every other run PASS, any INCOMPLETE analysed and its cause stated before the batch is used to set the operating scope.

