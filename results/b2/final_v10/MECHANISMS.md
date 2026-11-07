# Mechanisms of the non-PASS runs

raw root: F:\Andrea\HoloHybridAutomaton\results\robustness\raw_b2final_v10

## Counts by mechanism

* **FORMATION_NOT_RECOVERED**: 8 runs; B2_MEDIUM/MEDIUM-ALL integrated_short x3, B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x2, B2_MEDIUM/MEDIUM-ALL formation_gust x1, B2_MEDIUM/MEDIUM-ALL formation_recovery_head_current x1, B2_MEDIUM/MEDIUM-ALL lost_drone_rejoin x1
* **FALSE_VACANCY**: 6 runs; B2_MEDIUM/MEDIUM-ALL integrated_short x2, B2_MEDIUM/MEDIUM-ALL formation_gust x1, B2_MEDIUM/MEDIUM-ALL formation_recovery_head_current x1, B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x1, B2_MEDIUM/MEDIUM-ALL lost_drone_rejoin x1
* **scripted_fault**: 5 runs; B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x4, B2_MEDIUM/MEDIUM-ALL lost_drone_rejoin x1
* **GATE_OR_PROGRESS_OTHER**: 4 runs; B2_MEDIUM/MEDIUM-ALL formation_gust x1, B2_MEDIUM/MEDIUM-ALL formation_recovery_head_current x1, B2_MEDIUM/MEDIUM-ALL integrated_short x1, B2_MEDIUM/MEDIUM-ALL lost_drone_rejoin x1
* **VACANCY_NOT_DECLARED**: 4 runs; B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x4
* **GIVE_UP_CYCLES**: 3 runs; B2_MEDIUM/MEDIUM-ALL integrated_short x2, B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x1
* **GATE_LATCH_BUSY_POSSIBLE_OCCUPANT**: 2 runs; B2_MEDIUM/MEDIUM-ALL integrated_short x2
* **GATE_LATCH_BUSY_OTHER**: 1 runs; B2_MEDIUM/MEDIUM-ALL lost_drone_mutex x1
* **STILL_WAITING_FOR_NEIGHBOUR**: 1 runs; B2_MEDIUM/MEDIUM-ALL lost_drone_rejoin x1

## Runs

* `B2_MEDIUM/MEDIUM-ALL/formation_gust/s800` INCOMPLETE: GATE_OR_PROGRESS_OTHER; FORMATION_NOT_RECOVERED; FALSE_VACANCY | missing ['progress', 'P3 recovered', 'no false vacancy']
    - drone 0: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1], 'nav_bound': 7.679}
    - drone 1: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [0, 2], 'vacant': [], 'nav_bound': 7.126}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [1], 'vacant': [], 'nav_bound': 6.641}
    - drone 3: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [], 'nav_bound': 7.634}
* `B2_MEDIUM/MEDIUM-ALL/formation_recovery_head_current/s800` INCOMPLETE: GATE_OR_PROGRESS_OTHER; FORMATION_NOT_RECOVERED; FALSE_VACANCY | missing ['progress', 'P3 recovered', 'no false vacancy']
    - drone 0: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1], 'nav_bound': 7.649}
    - drone 1: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [0, 3], 'vacant': [], 'nav_bound': 7.244}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [3], 'vacant': [], 'nav_bound': 7.141}
    - drone 3: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [1, 2], 'vacant': [], 'nav_bound': 7.27}
* `B2_MEDIUM/MEDIUM-ALL/integrated_short/s800` INCOMPLETE: GATE_LATCH_BUSY_POSSIBLE_OCCUPANT (a drone not confirmed outside); GIVE_UP_CYCLES (1); FORMATION_NOT_RECOVERED; FALSE_VACANCY | missing ['progress', 'gate', 'P3 recovered', 'no false vacancy']
    - drone 0: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1, 2], 'nav_bound': 11.678}
    - drone 1: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -5.39, 'missing': [2], 'vacant': [], 'nav_bound': 8.258}
    - drone 2: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'BUSY', 'loc_ok': False, 'entry': None, 'acoustic': {'possible': [1], 'not_outside': [1], 'excluded': [0]}, 's': -5.4, 'missing': [0, 1], 'vacant': [], 'nav_bound': 9.716}
* `B2_MEDIUM/MEDIUM-ALL/integrated_short/s803` INCOMPLETE: GATE_LATCH_BUSY_POSSIBLE_OCCUPANT (a drone not confirmed outside); GIVE_UP_CYCLES (2); FORMATION_NOT_RECOVERED | missing ['progress', 'gate', 'P3 recovered']
    - drone 0: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'BUSY', 'loc_ok': False, 'entry': None, 'acoustic': {'possible': [1, 2], 'not_outside': [1]}, 's': -5.41, 'missing': [2], 'vacant': [], 'nav_bound': 10.51}
    - drone 1: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'BUSY', 'loc_ok': False, 'entry': None, 'acoustic': {'possible': [0], 'not_outside': [0], 'excluded': [2], 'fixes': [[664.0, True, 'missing', False], [665.0, True, 'missing', False], [666.0, True, 'missing', False], [668.0, True, 'missing', False]]}, 's': -5.4, 'missing': [], 'vacant': [], 'nav_bound': 8.907}
    - drone 2: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -5.39, 'missing': [0, 1], 'vacant': [], 'nav_bound': 9.063}
* `B2_MEDIUM/MEDIUM-ALL/integrated_short/s804` INCOMPLETE: GATE_OR_PROGRESS_OTHER; FORMATION_NOT_RECOVERED; FALSE_VACANCY | missing ['progress', 'gate', 'P3 recovered', 'no false vacancy']
    - drone 0: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1], 'nav_bound': 10.667}
    - drone 1: {'mode': 'SEPARATION_WARNING', 'committed': True, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -3.55, 'missing': [0], 'vacant': [], 'nav_bound': 19.679}
    - drone 2: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1], 'nav_bound': 10.868}
* `B2_MEDIUM/MEDIUM-ALL/lost_drone_mutex/s801` INCOMPLETE: GATE_LATCH_BUSY_OTHER (corridor echo / no acoustic diagnostics); GIVE_UP_CYCLES (1); FORMATION_NOT_RECOVERED; FALSE_VACANCY; VACANCY_NOT_DECLARED; scripted_fault | missing ['progress', 'gate', 'P3-deg re-formed', 'P3-deg vacancy', 'no false vacancy']
    - drone 0: {'mode': 'FAILSAFE_HOLD_OR_RETREAT', 'committed': False, 'occ': 'BUSY', 'loc_ok': False, 'entry': None, 'acoustic': {}, 's': -14.17, 'missing': [1, 2], 'vacant': [], 'nav_bound': 6.968}
    - drone 1: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [0, 2], 'nav_bound': 11.589}
    - drone 2: {'mode': 'MUTEX_YIELD', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -5.41, 'missing': [0, 1], 'vacant': [], 'nav_bound': 13.838}
* `B2_MEDIUM/MEDIUM-ALL/lost_drone_mutex/s802` INCOMPLETE: FORMATION_NOT_RECOVERED; VACANCY_NOT_DECLARED; scripted_fault | missing ['P3-deg re-formed', 'P3-deg vacancy']
    - drone 0: {'mode': 'FAILSAFE_HOLD_OR_RETREAT', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -14.07, 'missing': [], 'vacant': [], 'nav_bound': 9.123}
    - drone 1: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [0], 'nav_bound': 18.576}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [0], 'vacant': [], 'nav_bound': 15.457}
* `B2_MEDIUM/MEDIUM-ALL/lost_drone_mutex/s803` INCOMPLETE: VACANCY_NOT_DECLARED; scripted_fault | missing ['P3-deg vacancy']
    - drone 0: {'mode': 'FAILSAFE_HOLD_OR_RETREAT', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -13.98, 'missing': [1, 2], 'vacant': [], 'nav_bound': 7.598}
    - drone 1: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [0], 'nav_bound': 11.564}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [0], 'vacant': [], 'nav_bound': 13.084}
* `B2_MEDIUM/MEDIUM-ALL/lost_drone_mutex/s804` INCOMPLETE: VACANCY_NOT_DECLARED; scripted_fault | missing ['P3-deg vacancy']
    - drone 0: {'mode': 'FAILSAFE_HOLD_OR_RETREAT', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -14.08, 'missing': [1, 2], 'vacant': [], 'nav_bound': 6.234}
    - drone 1: {'mode': 'DEGRADED_FORMATION', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [0], 'nav_bound': 13.972}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': 'ADMIT', 'acoustic': None, 's': -1000000000.0, 'missing': [0], 'vacant': [], 'nav_bound': 12.567}
* `B2_MEDIUM/MEDIUM-ALL/lost_drone_rejoin/s800` INCOMPLETE: GATE_OR_PROGRESS_OTHER; FORMATION_NOT_RECOVERED; FALSE_VACANCY; STILL_WAITING_FOR_NEIGHBOUR; scripted_fault | missing ['progress', 'P3 recovered', 'no false vacancy', 'not waiting']
    - drone 0: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [], 'vacant': [1], 'nav_bound': 9.677}
    - drone 1: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [0], 'vacant': [], 'nav_bound': 7.065}
    - drone 2: {'mode': 'FORMATION_RECOVERY', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [1], 'vacant': [], 'nav_bound': 7.229}
    - drone 3: {'mode': 'FORMATION_WAIT_REJOIN', 'committed': False, 'occ': 'FREE', 'loc_ok': False, 'entry': None, 'acoustic': None, 's': -1000000000.0, 'missing': [1], 'vacant': [], 'nav_bound': 8.722}
