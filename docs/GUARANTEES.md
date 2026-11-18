# Guarantees

What the controller guarantees, on which evidence, under which premises, how each premise is monitored at run time and
what happens when it fails. The formal checks are in `formal/` (`python formal/check_properties.py`, 300 checks, all
with their expected verdict; `formal/results/SUMMARY.md`). The raw runs of the final validation and the
evidence of earlier campaigns are in the external archive (`MANIFEST.json` of each archive version).

Kinds of evidence:

- **proof**: an SMT query (Z3) that is UNSAT for every value of its variables, unbounded for the encoded model
  (k-induction and inductive invariants included);
- **BMC**: bounded model checking up to a stated horizon;
- **exhaustive numeric**: every point of a finite grid or every sector pattern;
- **randomized**: many random samples and no counterexample (evidence, not proof);
- **mutation**: a deliberately broken variant must give a counterexample, so the check is not vacuous;
- **empirical**: HoloOcean runs.

Every proof is on an abstraction of the vehicles and their sensing, not on HoloOcean's physics. Separate proofs of
separation, mutual exclusion and formation do not by themselves imply their composition; where a composition is
claimed, a model of the composition is checked (fleet model F0-F2), and its limits are listed.

## P1, separation: every pair of live vehicles stays at least 1 m apart

| Claim | Evidence | Premises |
|---|---|---|
| The onboard distance is a lower bound of the true distance to the nearest hull | randomized (S0, about 35 000 scenes); mutation | echo error within the declared band; hull tilt <= 10 deg |
| From any state with onboard distance >= d_warning the separation stays >= d_safe, both vehicles reacting | BMC S1a (4 s) and proof S1b (k-induction) | speed, current and staleness bounds of `config.Envelope` |
| The same with one vehicle reacting | BMC S1d at the simulator deceleration 0.743 m/s^2; SAT at the configured 0.5 m/s^2 (S1o) | the simulator calibration, not yet measured on a vehicle |
| The escape opens every threat pattern | exhaustive numeric S2a-S2d | six-sector geometry |

Monitors: referee P1 on ground truth in every run; echo-bound exceedance per run. In the captures where the echo error
leaves the declared band (the MEDIUM profile is a stress level beyond it), P1 is tested, not proved.

## P2, mutual exclusion: at most one vehicle centre in the critical region (CR) of a gate

| Claim | Evidence | Premises |
|---|---|---|
| Never two vehicles with priority; the first of the queue order leads | proof M1, M2; mutations | heading and position tolerances at the queue |
| The occupancy latch never admits two vehicles | BMC M3 (42 s); mutation M3m | every entry into the CR seen in the FRONT corridor; the occupant crosses without stopping |
| A claim holds the gate until its sender reports otherwise or is dead for the gate (rule D) | S1, S4 on recorded receptions; mutations | premise V; a received packet is genuine (CRC-16) |
| Composition of n = 2, 3, 4 vehicles at one gate (failures, stops, claims learnt up to 2 steps late with M2): never two vehicles in the CR | proof F0 (inductive invariant, k = 1), F1d (delay bound, tight: F1d3, F1d2n); recovery F2 | 5 s step abstraction; latch, freshness and precedence taken from M3, S1, S2, M2 |
| An avoidance velocity moves one committed vehicle into the CR without the stage-2 check: the invariant and the recovery still hold | proof F0p, F2p (n = 2, 3, 4) | the moved vehicle keeps its claim (`report_phase`, S9) |

Premises of P2 under B2, their monitors and the behaviour when they fail:

| Premise | Monitor | When it fails |
|---|---|---|
| **V**: the channel never silences a live vehicle for 70 s at a receiver that hears another vehicle | `PREMISE_V_VIOLATED` per run | a live vehicle can be declared dead for the gate |
| **C**: a queued vehicle learns of a commitment (claim or corridor echo) before it commits; two that commit together learn of each other's claim before either is admitted | event-ordered audit (`referee/premise_audit.py`): SATISFIED, CONCURRENT, VIOLATED, INCONCLUSIVE | a lost claim lets a second vehicle commit; the stage-2 check then holds it until the CR is seen free |
| **H6**: the sender's navigation error lies within its sent bound | `NAV_BOUND_PREMISE_VIOLATED` | payload rules are not used as P2 evidence for that run |
| Gate-frame navigation error of a committed vehicle <= 1 m | `GATE_NAV_PREMISE_VIOLATED` | admission waits for a validated fit; the run may end INCOMPLETE (the open localization class) |
| Packets arrive within `latency_max_s` = 0.4 s | received / accepted / rejected / stale counts per run | late packets are dropped before decoding: the claims are lost and the gate runs on the sonar only |

The verification horizon of S2h (32 s) is the time a crossing vehicle may stop before the CR in a bounded model; it is
not a delivery deadline. The bound on the use of a report is the freshness cutoff `status_fresh_s` (1.95 s for the
two-vehicle stacked plan, 2.5 s otherwise).

Declared limits (checked as satisfiable, not excluded):

- F1p: two vehicles that commit together, with the claims learnt one step late, can both be moved into the CR by
  avoidance; F0q: a queued, uncommitted vehicle moved into the CR by avoidance breaks the invariant; F1pa: the escape
  planner is not antisymmetric along the gate axis, so the model cannot exclude either case. The referee logs every
  CR entry as ADMITTED, PUSHED or UNCOMMITTED, with the vehicles inside at that instant;
- a stopped occupant beyond the 32 s horizon with no claim heard (S2hs); a vehicle stopped in the CR by a fault and
  released by the `t_occ_max` fallback (M3f).

## P3, formation: slot recovery, fleet formation and degraded formation within their deadlines

| Claim | Evidence | Premises |
|---|---|---|
| Along-track and lateral recovery of a vehicle's own estimated slot | proof F1-F3 (ranking functions, invariant balls); mutations | declared current envelope; catch-up margin |
| The automaton selects the right formation mode | proof F4a-F4d | - |
| A neighbour that never reappears is declared vacant within `t_rejoin` | BMC F6, ranking F6r | the timer runs while the vehicle keeps its slot |
| The fleet re-forms from any state of the gate invariant once failures and stops end (26, 40, 52 steps of 5 s for n = 2, 3, 4) | proof F2 (bounded recovery) | at most n - 2 failures; fairness: a fresh fix within every 35 s |

The ranking functions concern each vehicle's estimated slot, not the true formation; the P3 deadlines assume the nominal
closing speed and are not proved. Monitors: P3a, P3b, P3c verdicts per run (`referee/p3.py`); fairness gaps; false vacancies.

The contract judged per run (offline, from ground truth; tolerance E_TOL = 1 m held for T_HOLD = 8.9 s):

- **P3a**, slot recovery after the gate: each healthy vehicle that has crossed is back within E_TOL of its template slot,
  relative to the vehicles already across, within its deadline;
- **P3b**, fleet formation: after the last crossing (or the last declared disturbance) the formation error of the healthy
  vehicles on their slots is within E_TOL within its deadline;
- **P3c**, degraded formation after an admissible permanent failure: every neighbour of the lost vehicle declares its slot
  vacant within 47 s (the last survivor within `t_rejoin` = 30 s of its timer), no slot of a live vehicle is held vacant,
  and P3b then holds for the survivors.

Deadline D = e0 / 0.10 m/s + 5 s + T_HOLD (e0 the error at the start of the window), extended by the time spent in a mode
that pre-empts the formation (P1 first, failsafe) and inside declared disturbance windows. D is a requirement on the
nominal dynamics, not the proved bound. A part not met is PREMISE_VIOLATED only when a premise is measured on ground
truth: W (current within the control authority), N (onboard horizontal error <= 0.5 m), A (acoustics, P3c), O (no dead
hull obstructing the path), S (failures outside the exclusion zone of `scope.py`, at most n - 2), plus fairness.

Run classes (`referee/robustness_metrics.py refine_v8`): PASS (mission complete, every applicable part of P3 met);
P3_LATE (a part met after its deadline); P3_FAILURE (a part never met with the premises held); PREMISE_VIOLATED;
INCOMPLETE (gate or progress not reached with the premises held); SAFETY_FAILURE, UNSAFE_ACTION, INVALID.


## Isolation of the beacon

Beacon data never reach separation, collision avoidance, the gate precedence or the sonar exit witness (static check
B2-S, `tests/test_b2_isolation.py`); controllers never read ground truth (`tests/test_isolation.py`). The beacon is
strictly one-way: no request, reply, acknowledgement or command.

## Empirical evidence: final validation

`python scripts/run_sensor_robustness_campaign.py --phase validation`: 34 runs, seeds 8000-8001, all from one clean
commit (d5f424e) and one engine build. Empirical, not proved.

| Protocol / profile | Scenarios | Runs | Result |
|---|---|---|---|
| B2 MEDIUM / MEDIUM-ALL | p1_head_on, formation_six, lost_drone_rejoin, gate_single, mutex_deadlock_resolution, lost_drone_mutex, integrated_short | 14 | 14 PASS |
| B0 / LOW-ALL | the same seven | 14 | 11 PASS; `lost_drone_mutex` 2 P3_FAILURE (false vacancy); `integrated_short` s8000 P3_LATE |
| B2 latency 0.35 s and 0.45 s / LOW-ALL | gate_single | 4 | 4 PASS |
| B2 MEDIUM / MEDIUM-ALL | gate_four | 2 | 2 PASS |

- P1 and P2 held in all 34 runs; smallest true separation 2.84 m; at most one vehicle in the CR.
- Premise C audit: 64 pairs SATISFIED, 0 CONCURRENT, 0 VIOLATED, 0 INCONCLUSIVE. CR entries: 64, all ADMITTED.
- Packets accepted on board: B2 MEDIUM 97.4 % (0 stale); latency 0.35 s 67.4 % (32.0 % stale); latency 0.45 s 0 %
  (100 % stale).

Declared limits of liveness: a failure at or near the queue in still water (the
floating hull blocks the sonar-only precedence); the localization at the gate (lateral error with an ambiguous map fit,
sensitive to compass bias); two vehicles, or the last survivor, after a failure while committed; a failure within the
onboard margin of the exclusion zone ends in a safe stop; without a beacon (B0) a lost vehicle is vacated by a heuristic
that also vacates live ones (P3c not met).

Threats to validity: simulation only (sonar and acoustic channel are models; coast bound and rise speed measured in
the simulator); one gate geometry; the declared current equals the true bound of each scenario; the scope, rule D and
the P3 contract were set during development; HoloOcean's own sensor noise is unseeded, so runs are reproducible
statistically, not bit for bit (the wrapper noise, the acoustic channel and the controllers are seeded and
deterministic; every run records commit, engine build, configuration and seeds in `run_config.json`).

The controller is the one frozen as `b2-freeze-v11`: its code is identical to that tag apart from comments and
ablation switches whose defaults keep the frozen behaviour (`FormationRule.neighbour_adaptation`, the sweep channels).
