# Final campaign summary

624 valid runs (0 INVALID, excluded).  Code commits: {'1a1487281ae0f7b52a4dc9543741b3ce3b3447c5': 48, 'ad4fb5eaf643dc500803c7317570348b5efb438e': 566, 'c86052f476e3ceea6b45c2eb3c0515c10490fb5f': 10}; runs from a modified tree: 0; engine binaries: {'a5e9aa1fb7756c06b83af3a0896e42e6188f0390dd7a3d3d6e6e0532549fa028': 624}.


## Classes per cell (PASS rate with Wilson 95 % interval)

| protocol | profile | runs | invalid | PASS | SAFE_DEGRADED | LIVENESS_FAILURE | SAFETY_FAILURE | pass_rate | pass_ci95 | collisions | p1_min_m | false_releases | timeout_releases | sonar_releases | acoustic_releases | recovery_mean_s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | CLEAN | 48 | 0 | 48 | 0 | 0 | 0 | 1.0 | 0.926-1.0 | 0 | 2.373 | 0 | 20 | 14 | 0 | 62.0 |
| B0 | LOW-ALL | 48 | 0 | 47 | 1 | 0 | 0 | 0.979 | 0.891-0.996 | 0 | 2.424 | 0 | 13 | 18 | 0 | 53.7 |
| B0 | MEDIUM-ALL | 48 | 0 | 33 | 15 | 0 | 0 | 0.688 | 0.547-0.801 | 0 | 2.55 | 2 | 8 | 41 | 0 | 51.4 |
| B0 | MEDIUM-ONE | 48 | 0 | 43 | 5 | 0 | 0 | 0.896 | 0.778-0.955 | 0 | 2.386 | 0 | 15 | 24 | 0 | 66.9 |
| B0 | MEDIUM-SUBSET | 48 | 0 | 33 | 15 | 0 | 0 | 0.688 | 0.547-0.801 | 0 | 2.495 | 0 | 10 | 48 | 0 | 57.3 |
| B2_BLACKOUT | CLEAN | 48 | 0 | 48 | 0 | 0 | 0 | 1.0 | 0.926-1.0 | 0 | 2.642 | 0 | 14 | 17 | 0 | 55.7 |
| B2_BLACKOUT | MEDIUM-ALL | 48 | 0 | 40 | 8 | 0 | 0 | 0.833 | 0.704-0.913 | 0 | 2.483 | 0 | 6 | 42 | 0 | 60.6 |
| B2_CLEAN | CLEAN | 48 | 0 | 48 | 0 | 0 | 0 | 1.0 | 0.926-1.0 | 0 | 2.506 | 0 | 1 | 16 | 14 | 69.2 |
| B2_LOW | LOW-ALL | 48 | 0 | 46 | 2 | 0 | 0 | 0.958 | 0.86-0.988 | 0 | 2.46 | 0 | 2 | 17 | 12 | 65.0 |
| B2_MEDIUM | CLEAN | 48 | 0 | 48 | 0 | 0 | 0 | 1.0 | 0.926-1.0 | 0 | 2.5 | 0 | 16 | 13 | 5 | 62.1 |
| B2_MEDIUM | MEDIUM-ALL | 48 | 0 | 44 | 4 | 0 | 0 | 0.917 | 0.804-0.967 | 0 | 2.26 | 1 | 3 | 45 | 0 | 49.2 |
| B2_MEDIUM | MEDIUM-ONE | 48 | 0 | 42 | 6 | 0 | 0 | 0.875 | 0.753-0.941 | 0 | 2.597 | 1 | 9 | 22 | 8 | 61.2 |
| B2_MEDIUM | MEDIUM-SUBSET | 48 | 0 | 40 | 8 | 0 | 0 | 0.833 | 0.704-0.913 | 0 | 2.4 | 3 | 9 | 38 | 2 | 50.5 |

## PASS per scenario and cell

| scenario | B0 CLEAN | B0 LOW-ALL | B0 MEDIUM-ALL | B0 MEDIUM-ONE | B0 MEDIUM-SUBSET | B2_BLACKOUT CLEAN | B2_BLACKOUT MEDIUM-ALL | B2_CLEAN CLEAN | B2_LOW LOW-ALL | B2_MEDIUM CLEAN | B2_MEDIUM MEDIUM-ALL | B2_MEDIUM MEDIUM-ONE | B2_MEDIUM MEDIUM-SUBSET |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| formation_gust | 3/3 | 3/3 | 3/3 | 3/3 | 1/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 |
| formation_recovery_head_current | 3/3 | 3/3 | 1/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| formation_six | 3/3 | 3/3 | 2/3 | 3/3 | 1/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 2/3 |
| formation_square | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 1/3 | 2/3 |
| formation_triangle | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| gate_single | 3/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 2/3 | 2/3 |
| integrated_short | 3/3 | 3/3 | 1/3 | 3/3 | 1/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 1/3 |
| line_parallel_mutex | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| lost_drone_mutex | 3/3 | 3/3 | 2/3 | 0/3 | 1/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 2/3 |
| lost_drone_rejoin | 3/3 | 3/3 | 0/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| lost_drone_timeout | 3/3 | 3/3 | 0/3 | 1/3 | 0/3 | 3/3 | 0/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 2/3 |
| mutex_deadlock_resolution | 3/3 | 3/3 | 2/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 2/3 |
| p1_close_encounter | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| p1_head_on | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| p1_two_lines | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 |
| p1_vertical_escape | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |

## Cause tags of the non-PASS runs

| tag | runs |
|---|---|
| sensor_fault_failsafe | 64 |
| false_reoccupation | 32 |
| waiting_for_missing_neighbour | 28 |
| gate_not_completed | 20 |
| relocalization_hold | 17 |
| vacancy_not_declared | 13 |
| false_vacancy | 9 |

## Paired B0 / B2 comparison (same scenario, sensor profile and seed)

| b2 | profile | pairs | both_pass | b0_only_pass | b2_only_pass | neither | mcnemar_p | b0_safety_failures | b2_safety_failures |
|---|---|---|---|---|---|---|---|---|---|
| B2_CLEAN | CLEAN | 48 | 48 | 0 | 0 | 0 | 1.0 | 0 | 0 |
| B2_MEDIUM | CLEAN | 48 | 48 | 0 | 0 | 0 | 1.0 | 0 | 0 |
| B2_BLACKOUT | CLEAN | 48 | 48 | 0 | 0 | 0 | 1.0 | 0 | 0 |
| B2_LOW | LOW-ALL | 48 | 45 | 2 | 1 | 0 | 1.0 | 0 | 0 |
| B2_MEDIUM | MEDIUM-ONE | 48 | 38 | 5 | 4 | 1 | 1.0 | 0 | 0 |
| B2_MEDIUM | MEDIUM-SUBSET | 48 | 30 | 3 | 10 | 5 | 0.0923 | 0 | 0 |
| B2_MEDIUM | MEDIUM-ALL | 48 | 31 | 2 | 13 | 2 | 0.0074 | 0 | 0 |
| B2_BLACKOUT | MEDIUM-ALL | 48 | 30 | 3 | 10 | 5 | 0.0923 | 0 | 0 |

## Safety failures and false releases

| protocol | profile | scenario | seed | class | p1_min_sep_m | collisions | false_releases | cause_tags |
|---|---|---|---|---|---|---|---|---|
| B0 | MEDIUM-ALL | lost_drone_mutex | 0 | PASS | 3.106 | 0 | 1 | sensor_fault_failsafe;relocalization_hold |
| B0 | MEDIUM-ALL | mutex_deadlock_resolution | 1 | PASS | 3.466 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
| B2_MEDIUM | MEDIUM-ALL | line_parallel_mutex | 0 | PASS | 2.923 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
| B2_MEDIUM | MEDIUM-ONE | mutex_deadlock_resolution | 2 | PASS | 3.418 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
| B2_MEDIUM | MEDIUM-SUBSET | gate_single | 1 | PASS | 3.403 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
| B2_MEDIUM | MEDIUM-SUBSET | gate_single | 2 | PASS | 3.111 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
| B2_MEDIUM | MEDIUM-SUBSET | integrated_short | 1 | PASS | 3.31 | 0 | 1 | false_reoccupation;sensor_fault_failsafe |
