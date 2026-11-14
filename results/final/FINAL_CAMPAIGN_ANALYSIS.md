# Final campaign, criteria v8 (results/robustness/raw_v11_final)

Runs planned 448, found 448, missing 0 []

Classes, all runs: {'PASS': 403, 'P3_FAILURE': 3, 'INCOMPLETE': 25, 'P3_LATE': 7, 'PREMISE_VIOLATED': 10}

## Item 2, safety
- SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR, REPLAY_MISMATCH: 0 []
- P1 violated in 0 runs, P2 violated in 0 runs, of 448
- one-sided 95 % Clopper-Pearson upper bound of the per-run rate of a safety failure: 0.0067
- smallest true separation over the campaign: 2.397 m
- dead-hull contacts recorded (criteria v7, not contacts of the fleet): 0

## Item 3, INVALID
- 0 []

## Item 4, monitors
- premise V (a live drone declared dead): 0 runs, 0 in the validation profiles
- wrong validated fits: 1 runs (1 fits): [('MEDIUM-ALL', 'st_bd_cross_050', 5201)]
- gate-frame navigation premise exceeded while committed: 7 runs; H6 navigation bound exceeded: 0 runs
- fairness gap above 35 s: 15 runs [('B2_LOW', 'LOW-ALL', 'formation_six'), ('B2_LOW', 'LOW-ALL', 'formation_six'), ('B2_LOW', 'LOW-ALL', 'formation_six'), ('B2_MEDIUM', 'MEDIUM-ALL', 'formation_six'), ('B2_MEDIUM', 'MEDIUM-ALL', 'formation_six'), ('B2_MEDIUM', 'MEDIUM-ALL', 'formation_six'), ('B2_MEDIUM', 'MEDIUM-ALL', 'st_formation_six_oblique'), ('B2_MEDIUM', 'MEDIUM-ALL', 'st_formation_six_oblique')]; gap p50 5.6 s, max 767.0 s

## Item 5, standard set (block A)
- CLEAN: 66 of 66 complete (threshold 65) MET; classes {'PASS': 66}
- LOW-ALL: 66 of 66 complete (threshold 65) MET; classes {'PASS': 66}
- MEDIUM-ALL: 66 of 66 complete (threshold 63) MET; classes {'P3_LATE': 2, 'PASS': 64}
  - formation_gust s5000: P3_LATE [] ['false_reoccupation', 'sensor_fault_failsafe']
  - formation_gust s5002: P3_LATE [] ['sensor_fault_failsafe']

## Item 6, lost-drone family (block B)
- 58 of 60 complete (threshold 58) MET; classes {'PASS': 58, 'INCOMPLETE': 2}; 95 % upper bound of the failure rate 0.101
  - lost_drone_mutex s5102: INCOMPLETE ['progress', 'gate'] ['GATE_NAV_PREMISE_VIOLATED', 'false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']
  - lost_drone_mutex_rise_high s5108: INCOMPLETE ['progress', 'gate', 'P3-deg re-formed'] ['GATE_NAV_PREMISE_VIOLATED', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']

## Item 7, stress batch (block C)
- P1 and P2 violated in 0 of 130
- inside the envelope: 90 of 90 complete (threshold 81) MET
  - st_gate_head_020 (inside): {'PASS': 5}
  - st_gate_head_030 (inside): {'PASS': 5}
  - st_gate_tail_030 (inside): {'PASS': 5}
  - st_gate_cross_030 (inside): {'PASS': 5}
  - st_gate_oblique_025 (inside): {'PASS': 5}
  - st_gate_sinus (inside): {'PASS': 5}
  - st_gate_gusts (inside): {'PASS': 5}
  - st_gate_pair (inside): {'PASS': 5}
  - st_gate_line3_cross (inside): {'PASS': 5}
  - st_gate_stack_cross (inside): {'PASS': 5}
  - st_lost_left_cross (inside): {'PASS': 5}
  - st_lost_middle (inside): {'PASS': 5}
  - st_lost_right_tail (inside): {'PASS': 3, 'P3_LATE': 2}
  - st_lost_survey_gusts (inside): {'PASS': 5}
  - st_p1_head_on_cross (inside): {'PASS': 5}
  - st_p1_two_lines_sinus (inside): {'PASS': 5}
  - st_formation_six_oblique (inside): {'PASS': 5}
  - st_bd_head_040 (beyond the envelope): {'PREMISE_VIOLATED': 5}
  - st_bd_cross_050 (beyond the envelope): {'PASS': 5}
  - st_bd_tail_050 (beyond the envelope): {'PASS': 5}
  - st_bd_oblique_040 (beyond the envelope): {'PREMISE_VIOLATED': 2, 'P3_LATE': 3}
  - st_bd_gusts_030 (beyond the envelope): {'PASS': 5}
  - st_lost_at_queue (declared-limit probe): {'INCOMPLETE': 5}
  - st_lost_committed (declared-limit probe): {'INCOMPLETE': 5}
  - st_lost_after_exit (declared-limit probe): {'INCOMPLETE': 3, 'PASS': 2}
  - st_gate_pair_lost (inside): {'PASS': 5}

## Item 8, ablation (block D) against B2 (block A, MEDIUM)
- gate_single: B0 {'PASS': 3}; B2NP_MEDIUM {'INCOMPLETE': 3}; B2 {'PASS': 6}
- integrated_short: B0 {'PASS': 3}; B2NP_MEDIUM {'INCOMPLETE': 2, 'PASS': 1}; B2 {'PASS': 6}
- mutex_deadlock_resolution: B0 {'PASS': 3}; B2NP_MEDIUM {'INCOMPLETE': 3}; B2 {'PASS': 6}
- line_parallel_mutex: B0 {'PASS': 3}; B2NP_MEDIUM {'INCOMPLETE': 1, 'PASS': 2}; B2 {'PASS': 6}
- lost_drone_mutex: B0 {'P3_FAILURE': 3}; B2NP_MEDIUM {'PASS': 3}; B2 {'PASS': 6}

## Item 8, sensitivity (block E)
- B2_MEDIUM MEDIUM_HB3-ALL gate_single: {'PASS': 5}
- B2_MEDIUM MEDIUM_HB3-ALL lost_drone_mutex: {'PASS': 4, 'PREMISE_VIOLATED': 1}
- B2_MEDIUM MEDIUM_HB4-ALL gate_single: {'PASS': 5}
- B2_MEDIUM MEDIUM_HB4-ALL lost_drone_mutex: {'INCOMPLETE': 1, 'PASS': 2, 'PREMISE_VIOLATED': 2}
- B2_MEDIUM_HT MEDIUM-ALL gate_single: {'PASS': 5}
- B2_MEDIUM_HT MEDIUM-ALL lost_drone_mutex: {'PASS': 5}

## Item 9, P3 verdicts
- P3a: {'MET': 724, 'NA': 68, 'NOT_MET': 2, 'PREMISE_VIOLATED': 17, 'LATE': 5}
- P3b: {'MET': 374, 'NOT_MET': 2, 'NA': 33, 'LATE': 2}
- P3c: {'NOT_MET': 3, 'MET': 145}
- P3b slack to the deadline (s), MET runs: n 374, min 2.5, p10 6.1, median 19.7

## Item 10, every run not PASS
- [D] B0 MEDIUM-ALL lost_drone_mutex s5300: P3_FAILURE; failed []; tags ['false_reoccupation', 'false_vacancy', 'sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N']; P3a premises {}
- [D] B0 MEDIUM-ALL lost_drone_mutex s5301: P3_FAILURE; failed []; tags ['false_reoccupation', 'false_vacancy', 'sensor_fault_failsafe', 'relocalization_hold']; P3b NOT_MET []; P3a premises {'drone_1': ['N'], 'drone_2': ['N']}
- [D] B0 MEDIUM-ALL lost_drone_mutex s5302: P3_FAILURE; failed []; tags ['false_reoccupation', 'false_vacancy', 'sensor_fault_failsafe', 'relocalization_hold']; P3b NOT_MET []; P3a premises {'drone_1': ['N'], 'drone_2': ['N']}
- [D] B2NP_MEDIUM MEDIUM-ALL gate_single s5300: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL gate_single s5301: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL gate_single s5302: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL integrated_short s5300: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL integrated_short s5302: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL line_parallel_mutex s5300: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL mutex_deadlock_resolution s5300: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL mutex_deadlock_resolution s5301: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered', 'not waiting']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'waiting_for_missing_neighbour', 'gate_not_completed']; P3b NA None; P3a premises {}
- [D] B2NP_MEDIUM MEDIUM-ALL mutex_deadlock_resolution s5302: INCOMPLETE; failed ['progress', 'gate', 'P3 recovered', 'not waiting']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'waiting_for_missing_neighbour', 'gate_not_completed']; P3b NA None; P3a premises {'drone_0': ['N'], 'drone_1': ['N']}
- [A] B2_MEDIUM MEDIUM-ALL formation_gust s5000: P3_LATE; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b LATE ['N']; P3a premises {}
- [A] B2_MEDIUM MEDIUM-ALL formation_gust s5002: P3_LATE; failed []; tags ['sensor_fault_failsafe']; P3b LATE ['N']; P3a premises {}
- [B] B2_MEDIUM MEDIUM-ALL lost_drone_mutex s5102: INCOMPLETE; failed ['progress', 'gate']; tags ['GATE_NAV_PREMISE_VIOLATED', 'false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [B] B2_MEDIUM MEDIUM-ALL lost_drone_mutex_rise_high s5108: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['GATE_NAV_PREMISE_VIOLATED', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_head_040 s5200: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_head_040 s5201: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_head_040 s5202: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_head_040 s5203: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_head_040 s5204: PREMISE_VIOLATED; failed []; tags ['sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_oblique_040 s5200: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_oblique_040 s5201: PREMISE_VIOLATED; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['W', 'N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_oblique_040 s5202: P3_LATE; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_oblique_040 s5203: P3_LATE; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_bd_oblique_040 s5204: P3_LATE; failed []; tags ['false_reoccupation', 'sensor_fault_failsafe']; P3b MET ['N', 'W']; P3a premises {'drone_0': ['N'], 'drone_1': ['W', 'N'], 'drone_2': ['W', 'N']}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_after_exit s5200: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_after_exit s5202: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_after_exit s5204: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_at_queue s5200: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_at_queue s5201: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_at_queue s5202: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_at_queue s5203: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_at_queue s5204: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_committed s5200: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_committed s5201: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_committed s5202: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_committed s5203: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_committed s5204: INCOMPLETE; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['sensor_fault_failsafe', 'gate_not_completed']; P3b NA None; P3a premises {}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_right_tail s5202: P3_LATE; failed []; tags ['sensor_fault_failsafe', 'relocalization_hold']; P3b MET ['N']; P3a premises {'drone_0': ['N'], 'drone_1': ['N']}
- [C] B2_MEDIUM MEDIUM-ALL st_lost_right_tail s5203: P3_LATE; failed []; tags ['sensor_fault_failsafe']; P3b MET ['N']; P3a premises {'drone_0': ['N'], 'drone_1': ['N']}
- [E] B2_MEDIUM MEDIUM_HB3-ALL lost_drone_mutex s5402: PREMISE_VIOLATED; failed ['progress', 'gate']; tags ['GATE_NAV_PREMISE_VIOLATED', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [E] B2_MEDIUM MEDIUM_HB4-ALL lost_drone_mutex s5400: INCOMPLETE; failed ['progress', 'gate']; tags ['sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [E] B2_MEDIUM MEDIUM_HB4-ALL lost_drone_mutex s5402: PREMISE_VIOLATED; failed ['progress', 'gate']; tags ['GATE_NAV_PREMISE_VIOLATED', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
- [E] B2_MEDIUM MEDIUM_HB4-ALL lost_drone_mutex s5404: PREMISE_VIOLATED; failed ['progress', 'gate', 'P3-deg re-formed']; tags ['GATE_NAV_PREMISE_VIOLATED', 'sensor_fault_failsafe', 'relocalization_hold', 'gate_not_completed']; P3b NA None; P3a premises {}
