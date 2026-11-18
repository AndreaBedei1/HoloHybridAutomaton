# Guarantees

What the controller guarantees, on which evidence, under which premises, how each premise is monitored at run time and
what happens when it fails. The formal checks are in `formal/` (`python formal/check_properties.py`, 321 checks, all
with their expected verdict; `formal/results/SUMMARY.md`). The full evidence of every campaign, including the negative
results, is in the versioned archive of the project (`MANIFEST.json` of each archive version).

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
claimed, a model of the composition is checked (GG4, F0-F2), and its limits are listed.

## P1, separation: every pair of live vehicles stays at least 1 m apart

| Claim | Evidence | Premises |
|---|---|---|
| The onboard distance is a lower bound of the true distance to the nearest hull | randomized (S0, about 35 000 scenes); mutation | echo error within the declared band; hull tilt <= 10 deg |
| From any state with onboard distance >= d_warning the separation stays >= d_safe, both vehicles reacting | BMC S1a (4 s) and proof S1b (k-induction) | speed, current and staleness bounds of `config.Envelope` |
| The same with one vehicle reacting | BMC S1d at the simulator deceleration 0.743 m/s^2; SAT at the configured 0.5 m/s^2 (S1o) | the simulator calibration, not yet measured on a vehicle |
| The escape opens every threat pattern | exhaustive numeric S2a-S2d | six-sector geometry |
| A guarded pair at its hold points keeps separation | exhaustive numeric GG2 (pair opening 0.153 m/s) and proof GG5 | both inside the bound; both beyond it: bounded only (GG5b) |

Monitors: referee P1 on ground truth in every run; echo-bound exceedance per run. When the echo bound fails (LOW 0.86 %
of captures, MEDIUM 3.6 % on average), P1 is tested, not proved.

## P2, mutual exclusion: at most one vehicle centre in the critical region (CR) of a gate

| Claim | Evidence | Premises |
|---|---|---|
| Never two vehicles with priority; the first of the queue order leads | proof M1, M2; mutations | heading and position tolerances at the queue |
| The occupancy latch never admits two vehicles | BMC M3 (42 s); mutation M3m | every entry into the CR seen in the FRONT corridor; the occupant crosses without stopping |
| A claim holds the gate until its sender reports otherwise or is dead for the gate (rule D) | S1, S4 on recorded receptions; mutations | premise V; a received packet is genuine (CRC-16) |
| **Gate guard**: no avoidance velocity moves a vehicle that is not admitted toward the CR beyond `GUARD_GAIN (s_hold - s)`, except a logged collision-avoidance override | exhaustive numeric GG1 over the deployed planner; mutation GG1m (the v11 planner) | sector geometry; gate axis within 15 deg of the body axis |
| Under the guard the estimated centre stays outside the CR face | proof GG3 (margin 0.079 m at 0.743 m/s^2), GG3a (0.059 m at 0.5 m/s^2); mutation GG3m | gate-frame navigation error <= 1 m; residual drift <= `w_drift_max` |
| A guard hold (no allowed move back) keeps station | proof GG3k (within 0.20 m of its start at <= 0.05 m/s, 0.40 m at <= 0.20 m/s); mutation GG3h (the first version without station keeping) | as GG3 |
| Composition of n = 2, 3, 4 vehicles with the guard: concurrent commitments, claims one step late, stops by separation or avoidance at any step: never two vehicles in the CR | proof GG4 (n = 2, 3, 4); fleet model F0-F2 (inductive invariant, bounded recovery) | 5 s step abstraction; latch, freshness and precedence taken from M3, S1, S2, M2 |

Premises of P2 under B2, their monitors and the behaviour when they fail:

| Premise | Monitor | When it fails |
|---|---|---|
| **V**: the channel never silences a live vehicle for 70 s at a receiver that hears another vehicle | `PREMISE_V_VIOLATED` per run | a live vehicle can be declared dead for the gate; observed only under the long-tail channel, with no wrong declaration |
| **C**: a queued vehicle learns of a commitment (claim or corridor echo) before it commits; two that commit together learn of each other's claim before either is admitted | event-ordered audit (`referee/premise_audit.py`): SATISFIED, CONCURRENT, VIOLATED, INCONCLUSIVE | a lost claim lets a second vehicle commit; the stage-2 check then holds it until the CR is seen free (observed: 3 VIOLATED pairs in 70 held-out runs, each after 1-2 claim pings that did not arrive under the MEDIUM channel losses; P2 held) |
| **H6**: the sender's navigation error lies within its sent bound | `NAV_BOUND_PREMISE_VIOLATED` | payload rules are not used as P2 evidence for that run |
| Gate-frame navigation error of a committed vehicle <= 1 m | `GATE_NAV_PREMISE_VIOLATED` | admission waits for a validated fit; the run may end INCOMPLETE (the open localization class) |
| Packets arrive within `latency_max_s` = 0.4 s | received / accepted / rejected / stale counts per run | late packets are dropped before decoding: the claims are lost and the gate runs on the sonar only (latency 0.45 s and above: 100 % stale) |

The verification horizon of S2h (32 s) is the time a crossing vehicle may stop before the CR in a bounded model; it is
not a delivery deadline. The bound on the use of a report is the freshness cutoff `status_fresh_s` (1.95 s for the
two-vehicle stacked plan, 2.5 s otherwise).

Declared limits (checked as satisfiable, not excluded):

- GG4o: a collision-avoidance override from the hold or the queue can put a second vehicle into the CR; logged as
  `GATE_GUARD_OVERRIDE` with every CR entry (ADMITTED, PUSHED, UNCOMMITTED) by the referee;
- a guard hold that starts at the bound at speed with maximal drift can bring the estimated centre about 0.3 m into
  the CR (GG3 with GG3k); logged as `GATE_GUARD_HOLD`;
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
closing speed and are not proved. Monitors: P3a, P3b, P3c verdicts per run ([`P3_CONTRACT.md`](P3_CONTRACT.md));
fairness gaps; false vacancies.

## Isolation of the beacon

Beacon data never reach separation, collision avoidance, the gate precedence or the sonar exit witness (static check
B2-S, `tests/test_b2_isolation.py`); controllers never read ground truth (`tests/test_isolation.py`). The beacon is
strictly one-way: no request, reply, acknowledgement or command.

## Empirical evidence

| Batch | Runs | Result |
|---|---|---|
| Final campaign, `b2-freeze-v11` (without the guard) | 448 | P1, P2 in all; every v8 criterion met |
| Post-freeze validation (communication sweep, emergence) | 164 (80 held-out communication runs, 20 held-out emergence runs, 64 pilot) | P1, P2 in all |
| Adversarial rear pusher, guard v2 / first guard / v11 | 6 / 6 / 6 | P1, P2 in all; CR entries not admitted 3 / 22 / 162, none with another vehicle inside; one v11 run ended in a contact that was not with another vehicle (SAFETY_FAILURE by the terminal criteria) |
| Held-out with the guard (6 gate scenarios, B2 MEDIUM and B0 LOW, seeds 7300-7304; `st_lost_middle` seeds 7310-7319), and v11 paired on the 15 `st_lost_middle` seeds | 70 + 15 | P1, P2 in all; no CR entry not admitted; B2 39 of 40 PASS (v11 final campaign on the same scenarios: 74 of 75); concurrent commitments on `st_lost_middle` in 10 of 15 runs with the guard and 10 of 15 without (v11 paired: 15 of 15 PASS) |
| Latency just below and above the acceptance limit (0.35, 0.45 s) and B2 LOW | 30 | all complete; accepted packets 63.6 %, 0 %, 98.9 % |
| Four vehicles at one gate (stacked square), B2 MEDIUM and B0 LOW | 10 | P1, P2 in all; 9 PASS, 1 P3_LATE |

Declared limits of liveness (safety held in every case observed): a failure at or near the queue in still water (the
floating hull blocks the sonar-only precedence); the localization at the gate (lateral error with an ambiguous map fit,
sensitive to compass bias); two vehicles, or the last survivor, after a failure while committed; a failure within the
onboard margin of the exclusion zone ends in a safe stop; without a beacon (B0) a lost vehicle is vacated by a heuristic
that also vacates live ones (P3c not met).

Threats to validity: simulation only (sonar and acoustic channel are models; coast bound and rise speed measured in
the simulator); one gate geometry; the declared current equals the true bound of each scenario; the scope, rule D and
the P3 contract were set during development; HoloOcean's own sensor noise is unseeded, so runs are reproducible
statistically, not bit for bit.
