# P3, the formation contract (criteria v8)

Status: candidate definition of 2026-11-13, to be frozen after the round 9 reviews and before the final campaign. Implementation: `holo_fleet/referee/p3.py` (offline, ground truth and logs; never fed to a controller), classes in `holo_fleet/referee/robustness_metrics.py` (`refine_v8`), tests in `tests/test_p3_contract.py`.

P3 is a conditional liveness property. It is required only under the premises below, each monitored per run on ground truth; it is split in three parts, each with a deadline. Tolerance and hold are unchanged: formation error at most E_TOL = 1 m, held for T_HOLD = 4 / k_slot = 8.9 s (the 2 % settling time of the first-order slot loop).

## Definitions

Errors are measured in the frame of the mission path, after the best (mean) translation of the drones concerned, as the formation error of the referee (`referee.formation_error`).

- **P3a, individual slot recovery after the gate.** A healthy drone k crosses the gate at t_x(k): its true centre goes beyond the exit plane (s > exit_s = 3.5 m) after having been before the critical region. Within D_a(k) of t_x(k), the deviation of k from its template slot, relative to the healthy drones already across, is at most E_TOL for T_HOLD. For the first drone across the window starts at the second crossing (a deviation needs two drones). Not applicable when only one drone crosses.
- **P3b, fleet formation after the gate.** After the last healthy crossing t_last, the formation error of the healthy drones on their original slots is at most E_TOL for T_HOLD within D_b of t_last. In a mission without a gate the window starts at the end of the last declared disturbance.
- **P3c, degraded formation after an admissible permanent failure** at t_f:
  1. every healthy drone that expects the lost drone as a neighbour (template distance at most `neighbour_range_m` = 7.5 m) declares its slot vacant within t_silent_s + 2 s = 47 s of t_f when it hears another drone; the last survivor, which hears nobody and learns nothing from silence, within t_rejoin = 30 s of running time of its B0 timer, frozen in the mutex modes and while the plan clock holds at the rendezvous (formal F6g);
  2. no healthy drone holds vacant the slot of a drone that is not lost;
  3. P3b holds for the healthy drones, the window starting at the later of t_last and the last vacancy.

## Deadlines

D = e0 / v_close + t_rejoin_margin + T_HOLD, with:

- e0 the deviation (P3a) or formation error (P3b) at the start of the window, measured;
- v_close = v_slot_max - v_nominal = 0.40 - 0.30 = 0.10 m/s: the slowest closing speed the protocol leaves (a drone following at its speed cap while the reference of the others has resumed the planned speed) [PROT];
- t_rejoin_margin = 5 s: ramp-in, confirmation and the first hold of the loop [PROT].

Each deadline is extended by the time the drone spends in a mode that pre-empts the formation by priority (SEPARATION_WARNING, COLLISION_AVOIDANCE: P1 first; FAILSAFE_HOLD_OR_RETREAT on a sensor fault) and by the time inside declared disturbance windows (a jet beyond the authority, a run declared outside the envelope). Example: a drone 2.4 m off at the last crossing has D = 24 + 5 + 8.9 = 37.9 s.

The deadline is a requirement on the nominal dynamics. It is not the guaranteed bound: the ranking functions of `formal/check_formation.py` certify convergence in the worst case of the declared envelope (current estimation residual, clock speeds) at 0.014 m/s along the path and 0.25 m/s across it in the saturated band (F1), 0.0115 and 0.0097 m/s in the linear band (F2), into an invariant ball of 0.44 and 0.36 m (F3). The certificate guarantees eventual recovery under the premises; the deadline asks for the nominal speed and a miss of it is a failure of the contract, not of the proof. The vacancy timers are bounded by F6, F6g, and the fleet-level recovery by `formal/check_fleet.py` F2 (26, 40, 52 steps of 5 s for n = 2, 3, 4, under its own premises).

## Premises

Monitored on ground truth over the window from its start to its deadline (`p3.premises`). A part that is not met becomes PREMISE_VIOLATED only when one of them is measured; otherwise it is NOT_MET.

| Premise | Monitor |
| --- | --- |
| W, current and control authority | the true current at the drone keeps its commanded velocity control-feasible (`control/current_envelope.check_current`), except for at most 2 s in a row |
| N, navigation | the horizontal error of the onboard estimate is at most e_ok = 0.5 m (the controller's band inside E_TOL), except for at most 2 s in a row. An excess that starts within 30 s of a correction applied by the drone's own map fit is the controller's (self-induced) and does not count as a premise violation |
| A, acoustics (P3c) | the receiver hears another drone in the window, and no live sender is silent for t_silent_s at a working receiver (`gate.silence`) |
| O, feasible path | no dead hull within d_warning_exit = 2.7 m of the drone for more than 10 s of the window (a physical obstruction) |
| S, admissible failures | every permanent failure outside the exclusion zone of `holo_fleet/scope.py` (a run with one inside is INVALID); at most n - 2 failures for n >= 3; for n = 2 none while committed (formal F2s2) |

Sonar availability enters as the FAILSAFE pre-emption that extends the deadline, not as a premise.

## Classes of a run (criteria v8)

| Class | Meaning |
| --- | --- |
| PASS | online SUCCESS (every requirement of criteria v3 at the end) and every part of P3 MET or not applicable |
| P3_FAILURE | the gate passed; a part of P3 NOT_MET, or a formation requirement missed at the end, with the premises held: a genuine liveness failure of the formation |
| PREMISE_VIOLATED | every failed part of P3, and a failed gate, explained by a premise measured on ground truth (P3: W, N, A, O; gate: premise V, or the gate-frame navigation premise when it does not follow the drone's own correction) |
| INCOMPLETE | the gate or the planned progress not reached with the premises held: a genuine liveness failure of the passage |
| SAFETY_FAILURE, UNSAFE_ACTION, INVALID, REPLAY_MISMATCH, SUT_ERROR | as in criteria v7 |

A controller deadlock is never classed as an external infeasibility: PREMISE_VIOLATED needs a measured premise, and a navigation excess caused by the drone's own map fit, or a hull that only confuses the sonar precedence without being near the drone, is not one.

## Development evidence (re-evaluated with the contract, 210 runs of med11, med12, st1, st2, reg2)

- 187 PASS, 8 PREMISE_VIOLATED (the boundary probes beyond the envelope: current W and navigation N measured; the compass-bias sensitivity), 6 P3_FAILURE, 7 INCOMPLETE.
- P3_FAILURE: `integrated_short` s1100, s1201 (late: the formation re-forms to 0.94 m after the last crossing, diverges to 2.9 m after drone_2 waits for a neighbour it does not confirm (FORMATION_WAIT_REJOIN, reduced progress) and falls behind, and re-forms about 30 s after the deadline; the same wait as the next case, to be re-checked after the fix); `st_lost_middle` s2000 (two survivors 7 m apart waited for each other: rule A-P did not count a neighbour present within e_lost, because of the error of its fix; fixed in cb2f5a7); three runs of the compass-bias sensitivity (the same mutual wait).
- INCOMPLETE: `st_lost_at_queue`, `st_lost_committed` (the hull floating above the queue blocks the sonar-only precedence; a failure of the passage, not of P3).
- med13 (cb2f5a7, after the fix of rule A-P, judged by v8): MEDIUM-ALL 31 of 31 and LOW-ALL 17 of 17 PASS, among them `integrated_short` 4 of 4 and `st_lost_middle` 3 of 3; compass bias 4 deg 3 of 6 PASS (one P3_FAILURE, one PREMISE_VIOLATED, one INCOMPLETE).

