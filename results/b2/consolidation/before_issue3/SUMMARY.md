# Development baseline before Issue 3

55 runs: five repetitions of each of the 11 specified cases; identical wrapper seeds, unseeded engine noise.
All workers exited 0; P1 and P2 held in all runs. These are development observations, not frozen final results.

| Case | Classes | E2.to_free | E3 | E4 | E5 | E7 |
|---|---|---:|---:|---:|---:|---:|
| B2_BLACKOUT / MEDIUM-ALL / formation_six | {'INCOMPLETE': 3, 'PASS': 2} | 0 | 0 | 0 | 0 | 0 |
| B2_CLEAN / CLEAN / gate_single | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |
| B2_CLEAN / CLEAN / lost_drone_mutex | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |
| B2_LOW / LOW-ALL / mutex_deadlock_resolution | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-ALL / formation_six | {'PASS': 4, 'INCOMPLETE': 1} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-ONE / formation_square | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-ONE / gate_single | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-SUBSET / gate_single | {'PASS': 5} | 1 | 1 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-SUBSET / integrated_short | {'STALLED': 1, 'PASS': 4} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-SUBSET / lost_drone_mutex | {'STALLED': 4, 'PASS': 1} | 0 | 0 | 0 | 0 | 0 |
| B2_MEDIUM / MEDIUM-SUBSET / lost_drone_timeout | {'PASS': 5} | 0 | 0 | 0 | 0 | 0 |

Classifications are those written by the pre-change classifier. Its known omission of E3 and E2.to_free from UNSAFE_ACTION must be corrected before freeze; the raw counts above are retained.
The controller/perception was unchanged across the infrastructure commits recorded by these runs. Raw evidence stays in the original directories.
