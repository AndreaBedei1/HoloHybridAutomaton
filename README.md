# HoloHybridAutomaton

Decentralised control of BlueROV2 fleets in HoloOcean: the same local hybrid automaton on every vehicle, passive one-way acoustic
beacons between vehicles. Three properties are checked with Z3 on abstractions, on the deployed code by enumeration, and in
simulation by a referee that judges every run from ground truth.

![automaton](figures/automaton/automaton_presentation.png)

## 1. Scope

One local hybrid automaton per vehicle (10 modes, prioritised guards, `holo_fleet/ha/spec.py`) runs on six single-beam
sonars, DVL, IMU, compass, depth sensor, the mission plan shared before the dive and the acoustic receptions. There is no
leader and no request, reply or command between vehicles.

- **P1, separation:** the centre distance of every pair stays at least `d_safe` = 1.0 m.
- **P2, mutual exclusion:** at most one vehicle in the critical region (CR) of a gate, and only after admission.
- **P3, eventual formation recovery:** after a loss the formation is recovered; after an admissible permanent failure the
  healthy vehicles recover the degraded formation, with the lost slots vacant and no false vacancy.

**Controller.** B2 (the automaton with passive one-way acoustic beacons) is the controller. B0 is the same controller with the
communication disabled (`config.communication_disabled`; only `beacon.mode` differs, `tests/test_single_controller.py`), a baseline
for the paper. No other variant exists in the code.

**Operating domain** (declared in `holo_fleet/config.py`, `sim/sensor_noise.py`, `scope.py`). Outside it no property is claimed;
the controller logs which property it can no longer certify.

- 2 to 4 vehicles at one gate: abreast queues of 2 or 3, stacked 2 x 2, a 3-vehicle stack pair. Abreast queues of 4 or more
  are refused when the plan is loaded (`queue_lateral_max` = 3.5 m). `formation_six` (6 vehicles) is simulated only.
- Horizontal current inside the control-feasible envelope (`control/current_envelope.py`, validated to 0.6 m/s), vertical
  current at most 0.25 m/s, unrejected drift at most 0.15 m/s per axis (P1, P2 filter), residual disturbance at most 0.05 m/s
  once the integrator has converged (P3; the proof margin vanishes at 0.064 m/s, `config/parameter_sources.json`).
- Navigation error at most `nav_err_m` = 1.0 m per axis in the gate frame; depth -9.0 to -1.5 m; braking at least 0.5 m/s^2;
  nominal speed at most 0.5 m/s. Acoustic envelope: the MEDIUM channel at 3 sigma (`AcousticConfig`), latency at most 0.40 s,
  clock skew at most 0.05 s, TDMA period 1 s.
- Sensor noise profiles CLEAN, LOW, MEDIUM. HIGH is a stress profile outside the domain (section 7: sensor-fault failsafe,
  no passage, P1 and P2 held).
- Permanent failures only outside the exclusion zone of the gate (`holo_fleet/scope.py`) and never inside the CR: one failure
  for 2 vehicles (none while committed), at most n - 2 for n >= 3.

## 2. Layout

```
holo_fleet/   ha/ edge table and sector precedence; perception/ sonar, navigation, gate belief, acoustic fixes, payload;
              control/ mode flows, P2 filter (gate_safety.py), escape planner, formation clock; referee/ ground-truth
              judging; sim/ HoloOcean session, scenarios, beacon and channel models; ui/ HoloFleetViewer; config.py
              (every threshold, gain and envelope: frozen dataclass `FleetConfig`, also imported by the formal suites)
formal/       Z3 suites check_*.py, runner check_properties.py, results/ (SUMMARY.json, SUMMARY.csv, one JSON per suite)
tests/        unit, integration and isolation tests
scripts/      demos, campaign runner, test runner, engine slots and isolation probe, legacy index check, archive builder
calibration/  simulator probes behind the measured parameters (outputs in results/calibration/)
config/       parameter_sources.json (source, class, validity of each parameter), scope_drones.json
results/      calibration/, targeted_validation/ (summary.json, summary.csv)
legacy/       moved material of earlier variants and campaigns, INDEX.json (sha256 per item)
patches/ figures/ references.bib   HoloOcean sonar patch and build script; diagrams; citation keys
```

## 3. Run

**Install.** Windows 10/11, Python >= 3.9 (conda), HoloOcean 2.3.0 with the Ocean world
([documentation](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/)), Unreal Engine 5.3 (to build the patched engine),
the Marine Race Arena ([doi:10.5281/zenodo.22757860](https://doi.org/10.5281/zenodo.22757860), gate G06).

```bash
conda create -n holo_fleet_ha --clone <environment with the HoloOcean 2.3.0 client> -y && conda activate holo_fleet_ha
pip install -e .                      # numpy, scipy, matplotlib, opencv-python, z3-solver, imageio, psutil
git clone https://github.com/AndreaBedei1/HoloDroneCompetition.git     # beside this repository, or set the variable below
```

Unpatched HoloOcean 2.3.0 sonars miss props spawned at run time (the gates), echo ghosts of earlier scenes from a stale octree
cache and can crash after a reset. `patches/holoocean-2.3.0-sonar-octree-rebuild.patch` (12 files) adds `env.rebuild_sonar_octree()`,
called once after the static scene is spawned; no sensor model or message format changes.

```bash
git clone https://github.com/byu-holoocean/HoloOcean.git && cd HoloOcean     # check out the 2.3.0 sources
git apply <this repo>/patches/holoocean-2.3.0-sonar-octree-rebuild.patch     # `git apply --check` first verifies the base
pip install -e client --no-deps       # copy patches/build_and_install_patched_holoocean.ps1 here, then:
powershell -ExecutionPolicy Bypass -File build_and_install_patched_holoocean.ps1 -UE "<Unreal Engine 5.3 folder>" -Root <patched root>
```

The script builds the Win64 Development engine and creates the patched root (content shared with the official installation by
junctions) with `PATCH_INFO.json` (source commit, SHA-256 of the executable); the campaign runner refuses to run without it. Engine
check: `python calibration/octree_rebuild_regression.py`, `python -m pytest -q tests/test_octree_patch.py`. Environment:
`HOLO_FLEET_HOLOOCEAN_ROOT` (patched root, default `~/holoocean_patched_root`), `MARINE_RACE_ARENA_ROOT` (Marine Race Arena checkout,
default `../HoloDroneCompetition`), `HOLO_FLEET_ENGINE_SLOTS` (concurrent engines, 1 to 4, default 1), `HOLO_FLEET_SLOT_WAIT_S`
(seconds a run waits for a free slot, default 0).

**Engine slots and ports.** Each engine holds an OS lock on one slot (`results/.holo_slots`) and runs on its own writable root
`<patched root>_slot<k>` made by `scripts/prepare_engine_roots.ps1 -SourceRoot <patched root> -Slots 0,1,2,3` (existing roots are never
overwritten). A launch carries a UUID, cleanup ends only the engine whose pid, creation time, parent and UUID match, the Unreal
TraceLog port 1985 is reserved so that the engine listens elsewhere, and an unverified listener refuses the launch.
`python scripts/verify_engine_isolation.py --out <dir>` checks with two engines that each client receives its own sensor.

```bash
python scripts/run_tests.py <out dir> --quick           # development tests and formal checks, no engine
python scripts/run_tests.py <out dir> [workers]         # complete formal suite and all Python tests under their configured skip conditions
python formal/check_properties.py [--quick]             # formal checks only; results/ (complete) or results_quick/ (development)
python scripts/run_demo.py --list                       # scenarios with duration and title
python scripts/run_demo.py --scenario lost_drone_rejoin [--headless] [--profile MEDIUM --seed 1] [--b2 OFF|CLEAN|LOW|MEDIUM|BLACKOUT]
python scripts/run_sensor_robustness_campaign.py --only gate_single --protocols B0 B2_MEDIUM --profiles CLEAN MEDIUM-ALL \
    --seeds 0 1 --raw <raw dir> [--dry-run] [--require-clean] [--shard i --nshards n]
```

The test runner writes a log for every subprocess and exits nonzero on an unexpected verdict or a failed test. Development mode
runs the automaton, observation, basic formation and B2 communication checks (the B2 step model runs in complete mode).
It also runs the unmarked Python tests. Complete mode runs the longer navigation and gate regressions, all 11 formal suites,
and preserves expected counterexamples in the formal report. The patched-engine smoke test is opt-in via `HOLO_FLEET_ENGINE_TESTS=1`
on a host prepared with isolated engine roots and ports.

`--b2 OFF` runs B0. HoloFleetViewer shows the fleet from above with P1/P2/P3 status and the automaton of the vehicle chosen with
`--select K`; it keeps every step (arrow keys, `PgUp`/`PgDn`, `Home`/`End`, `Space`). The campaign runner starts every run in its own
process at below-normal priority with a watchdog, resumes interrupted batches and writes `run_config.json` (commit, engine build,
configuration, seeds), state logs, `events.jsonl`, `referee_timeseries.csv`, `referee_metrics.json` and `run_metrics.json` (class,
terminal outcome, CR entries, beacon statistics) under `<protocol>/<profile>/<scenario>/s<seed>/`. Protocols: `B0` or
`B2_<channel>`, channel in `CLEAN LOW MEDIUM MEDIUM_HT BLACKOUT LOW_MUTE0` or a sweep `SW-<key><value>` (keys
`per lat jit loss burst asym`). Profiles: `CLEAN` or `<LEVEL>-<SCOPE>`, LEVEL in `LOW MEDIUM MEDIUM_HB3 MEDIUM_HB4 HIGH`, SCOPE in `ALL SUBSET ONE`
(`config/scope_drones.json`).

## 4. Controller

**Automaton.** Modes: `FORMATION_FOLLOW`, `FORMATION_WAIT_REJOIN`, `DEGRADED_FORMATION`, `FORMATION_RECOVERY`, `MUTEX_APPROACH`,
`MUTEX_YIELD`, `MUTEX_PASS`, `SEPARATION_WARNING`, `COLLISION_AVOIDANCE`, `FAILSAFE_HOLD_OR_RETREAT`. The 13 edges of a mode are in
priority order and all guards are evaluated at runtime as a determinism monitor. Priority: failsafe, collision avoidance
(`d_min` < 1.7 m, exit 2.0 m), separation warning (< 2.4 m, exit 2.7 m), mutex modes, formation recovery, formation keeping.
`d_min` is a sonar-only lower bound of the pair distance; the escape planner picks 3-D directions with a guaranteed opening speed.

**Geometry.** The CR is the approach before the gate plus the passage. Gate frame: s along the axis, l left, z up. `GateRule`:
s in [-1.05, 0.55] m (length 1.6 m), |l| <= 1.0 m, |z| <= 1.0 m (the opening of G06 is 2.0 m x 2.0 m). The exit plane is the bar
depth 0.30 m (half 0.15 m) plus the hull circumscribed radius `R_OUT` = 0.39 m, 0.54 m, rounded to 0.55 m. Stage-2 hold
`stage2_hold_s` = -2.5525 m: `cr_s_min` - (`nav_err_m` + 0.3525 m reach of a hold + 0.05 m slack) = -2.4525 m, minus the measured
hold error 0.10 m. Abreast queue lines (`GateRule.queue_s`): -2.55 m (n = 1), -5.39 m (2), -5.75 m (3); stacked 2 x 2: -5.76 m (Q1).

**Gate protocol.**
1. *Precedence.* A vehicle waits at its queue point. Its relation to each neighbour comes from the sonar sector pattern only
   (`ha/mutex_rule.py`): the leftmost queued vehicle present has priority, after `t_clear` = 1.0 s.
2. *Occupancy latch.* BUSY on an unexplained FRONT echo in the CR range or on a claim (B2); EXITING on an exit witness (sonar, or
   acoustic with the corridor empty); FREE `t_clear` later, or after `t_occ_max` = 25 s without corridor echoes (fallback; it
   assumes an occupant crosses the CR at `v_cross_min` = 0.12 m/s or more).
3. *Commit* (edge `commit`): calm (no fault, no separation hazard), at the queue point, sonar priority, latch free.
4. *Admission.* The committed vehicle follows the pass path to the stage-2 hold and enters the CR only when the automaton is in
   `MUTEX_PASS` with healthy sensors, the gate localization is validated, the stage-2 check holds (no unexplained corridor echo for
   3 captures with the FRONT sonar healthy, no claim, no vehicle heard or known inside), the estimated s is at least
   `stage2_hold_s` - 0.4 m, and the commitment is at least `admit_wait_s` old. B2: `commit_admit_min_s` = 4.0 s if every other
   vehicle of the plan was heard fresh (report emitted at most `admit_fresh_s` = 3.5 s ago), else `commit_admit_min_s` +
   `t_cross_max` = 41.52 s, with `t_cross_max` = (0.55 + 2.5525 + 0.4 + 1.0) / 0.12 = 37.52 s. B0: 0. Silence is never evidence: a
   vehicle not heard is unknown, a claim (COMMITTED or ADMITTED report) stands until its sender reports otherwise, a silent vehicle
   is never excluded as a possible occupant.
5. *Give-up.* Without admission, 26 s (`t_occ_max` + `t_clear`) after the wait ends, the vehicle retreats behind the abort plane and
   queues again (`ENTRY_HOLD_TIMEOUT`). Without a validated localization it scans acquisition offsets at the hold; a committed
   vehicle stops behind a vehicle ahead in its FRONT sector (standoff).

**P2 filter** (`control/gate_safety.py`). It edits the final velocity command of every vehicle not admitted to the CR and within 8 m
of it (mission and avoidance modes). A command is applied only if its backup cannot bring the true centre into the CR column
(the CR at every depth): the command for 0.35 s (control period plus largest data age), braking at 0.5 m/s^2, then station keeping
with hold error `w_drift_max` / `k_hold` = 0.30 m, drift 0.15 m/s in any direction, true position within `nav_err_m` of the estimate
on every axis. Selection uses a robust margin with an onboard-consistent lookahead (next estimate, velocity within its measurement
bound), and the applied command hands a latched certificate to the next step; certificates are dropped on a position correction, a
new gate or a larger drift bound (DVL outage). With no certified command the controller logs `P2_UNCERTIFIED` and retreats
(`P2_RECOVERY`). In avoidance modes each decision is classified `JOINTLY_FEASIBLE`, `UNCERTAIN` or `JOINTLY_INFEASIBLE` and logged.

**B2 beacon** (`perception/payload.py`, `sim/beacon.py`, `perception/acoustic.py`). One-way broadcast, no handshake, no reply. Vehicle k
transmits in slot k of a 1 s TDMA period an identity ping and a 64-bit packet: frame counter 2 bit, x and y 12 + 12 bit (0.05 m), depth
8 bit (0.05 m), error bound 6 bit (0.1 m steps to 3.9 m, then 0.5 m; 63 invalid), gate index 2 bit, gate phase 2 bit (none, committed,
admitted, passed), 4 bit reserved, CRC-16/CCITT-FALSE. The receiver measures range (one-way travel time, synchronised clocks) and
bearing (array) itself, builds a fix with an error region from the declared envelope, and attaches the packet only if it agrees with
the measurement. A reception delivered more than `latency_max_s` = 0.40 s after its arrival is dropped; a fix is fresh for
`fresh_s` = 2.5 s. Beacon data enter perception only (claims and acoustic exit evidence at the latch, admission wait, formation
association, vacancy after `t_silent_s` = 45 s without a packet), never `d_min`, `sense_ok`, the sector precedence or the escape
planner (`tests/test_b2_isolation.py`). Controllers never read ground truth (`tests/test_isolation.py`).

## 5. Guarantees

The full-run baseline at `e3e31f0` (`formal/results/SUMMARY.json`) records 402 checks, all with the expected verdict: 188 proved, 36 bounded, 50 numeric, 58 mutation, 26 witness,
14 scope, 11 reported, 19 declared limits (11 suites, about 40 min; formula, verdict and counterexample per check in `SUMMARY.csv`). The current complete runner has 392 checks: D1 and D2
already imply the ten former O4 assertions, one per mode. O5 still checks the runtime monitor on inconsistent observations.
Only `proved` (UNSAT of a universal or inductive property) and `bounded` (UNSAT over a horizon) support a guarantee; `numeric`
checks enumerate or sample the deployed code; mutation, witness, scope and limit counterexamples are never evidence of safety.
Every guarantee is conditional on section 1.

**Automaton**, proved for every legal observation and mode: determinism and completeness (D1, D2), failsafe, collision and warning
priority (H1-H3), gate entry, give-up and hazard rules (G1-G3), exact readable guards (L1), one edge per monitored observation (O5).

**P1.**
- Proved: `G(d >= d_safe)` for all time by k-induction, k = 13 (S1b); triangle lemma, of three vehicles at most one sees the other
  two more than 90 deg apart (S3); a gate-constrained vehicle that may only stop with a partner that opens (S1gb, k = 15).
- Bounded: start-up window 4 s (S1a); calibrated deceleration 0.743 m/s^2 (S1d); sonar blackout of 0.8 s from 2.34 m (S4);
  gate-constrained pair, 6 s (S1ga).
- Numeric: the onboard distance bound never exceeds the true hull distance (S0); guaranteed opening speed of the escape and the
  warning filter for every sector pattern (S2a-S2d); gate-aware selection, 25,272 decisions (J1).

**P2** (premises: P2 filter, an admitted vehicle progresses at least 0.12 m/s until it leaves the CR, truthful self-reports).
- Proved: never two vehicles with priority (M1); precedence selects exactly one vehicle for 2 to 6 queue points with at most one vacant
  point between neighbours (M2; M2s marks the limit); at most one vehicle committed or in the CR, inductive for n = 2, 3, 4 under
  steps, failures, stops and entries bypassing the stage-2 check, with claims learnt within the step (F0i, F0s, F0p); P2 over every
  loss pattern, latency, slot and clock offset unless the pair is mutually unheard (B2C-E1), with at most 2 or 40 consecutive losses
  per link (B2C-E2, B2C-E2k), concurrent commitment, stationary vehicle, failed transmitter, third-vehicle reports (B2C-Ea, Eb, Ee,
  E3); silence never excludes (B2C-D); soundness of the P2 test (P2B1); braking reach and station-keeping band (P2R1-P2R3d); base
  case and recursive availability of a certified command, the selection rule taken as an axiom (P2R5, P2R4).
- Bounded: occupancy latch, 42 s (M3, M3h); status freshness, 32 s (S2, S2h); claims learnt 1 or 2 steps late (F1d1, F1d2; F1d3 shows the bound tight), combined with P2 filter entries (F1p); step
  model of 100 s with at most 2 consecutive losses (B2C-S1).
- Numeric: queue and exit-observability geometry (M4a-M4g, Q1); reach never under-estimated, error below 3e-4 m (P2B1r); no P2-safe
  command enters the CR in 21,850 adversarial trajectories (P2B3) or passes under or over it (P2B6); station keeping and robust
  selection (P2B4, P2B5); acoustic envelope and phantom-echo soundness (B2-A1, B2-A2, B2-PE); exclusion zone (S12); recursive
  availability, 4,948 commands and 118,752 next states, none without a certified action (P2R4n).

**P3.**
- Proved: ranking functions of slot tracking, along and lateral, with invariant balls of 0.44 m and 0.36 m (F1-F3); mode selection
  (F4a-F4d); the coupled 2-norm law, far and near band, at nominal and edge clock speed (C1f, C1n); clock consensus (C2a-C2d);
  rendezvous hold, clock growth, clock-speed domain, vertical channel (C3e, C3h, C4, C1z); vacancy of a neighbour that never
  reappears (F6r, F6g).
- Bounded: vacancy within `t_rejoin` = 30 s (F6); recovery of the fleet within 26, 40 and 52 steps of 5 s for 2, 3 and 4 vehicles from
  every state of the invariant with no further failure or stop and fresh reports (F2, F2p; F2t shows the bound tight).

## 6. Limitations

- **Mutually unheard pair.** After 41 or more consecutive losses in both directions between two vehicles (B2C-Es) both admissions
  degrade and P2 rests on the B0 mechanisms only, precedence and latch (B2C-EL, B2C-S1b); the stage-2 sonar is not certified (B2C-G1).
- **P2 filter envelope.** P2R4l: a DVL outage that begins, or a position correction, between steps after a certificate is handed can
  leave no certified action; P2R4 takes the selection rule as an axiom, checked numerically (J1, P2R4n). The MEDIUM-ALL run of
  `gate_single` logged 9 `P2_UNCERTIFIED` events and 21 failsafe entries, with P1 and P2 holding. F1pe, F1pme, F0qe: with a navigation
  error, drift or estimate jump outside the envelope an entry by avoidance is possible and two vehicles can be in the CR.
- **Permanent failure.** Silence is not evidence, so a dead vehicle is never accounted for and survivors can stall safely: F2c (failure
  while committed keeps its claim), F2d (failure outside the commitment, n = 3, 4, any K), F2s2 (n = 2), F2s3 (last survivor of 3), F2s4
  (a hull floating above the queue). The bounds F2 do not model the degraded admission wait.
- **False vacancy.** A live vehicle with a mute transmitter is declared vacant after `t_silent_s` = 45 s. In the targeted run
  `b2c_mute_transmitter` the class is `P3_FAILURE` only for the requirement "no false vacancy"; P1, P2 and the P3 contract held.
- **Liveness cost.** A vehicle not freshly heard delays every admission to 41.52 s after the commitment. In the targeted runs
  `lost_drone_mutex` and `b2c_mute_transmitter` commitment to passage took 54 to 64 s (raw logs), above the earlier referee budget
  (`gate_bounded_response_pre_budget_update` false); the B2 budget now includes `commit_admit_min_s` + `t_cross_max`. In `b2c_blackout`
  vehicle 1 gave up its stage-2 entry twice because no localization could be validated while the others were unknown (the map-aiding
  identification is withheld), and passed 205.7 s after its commitment against a budget of 86 s (`gate_bounded_response` false);
  the class is PASS and P1, P2 hold, the cost is time.
  A vehicle without a validated gate localization is never admitted.
- **B0, 3 vehicles.** M4g: in abreast, triangle and line layouts of 3 vehicles a queued vehicle may get no sonar exit witness of a
  passing vehicle (detection range); the latch then falls back to `t_occ_max`.
- **P1.** Proved pairwise; several neighbours rest on the triangle lemma and the two-threat escape (S2c), not on a fleet model.
  S1gm: two vehicles that can only stop (each gate-constrained, neither opens) close below `d_safe` within 15 s; queue geometry (Q1)
  and standoff are meant to keep this unreachable, shown in simulation, not proved. A stacked column on the axis
  (`gate_rear_approach`) puts a queued vehicle 1.59 m from the hold, inside the warning band (Q1, reported).
- **P3.** C1l1 (near band at the transient consensus bound), C1l2 (B2 far band at the clock-speed edge with adversarial lateral
  consensus), C2l (leak-only consensus of an abreast B2 pair) have no ranking. C1x: the per-axis ranking F1 is not a ranking of
  the deployed coupled law; C1f and C1n cover it.
- **Not covered by any proof.** Failures inside the exclusion zone, the continuous geometry inside the CR (M4h: 0.40 of it is observable
  from the hold), the real hold loop (first-order abstraction), field conditions. Every result is on HoloOcean models.

## 7. Evidence

**Targeted HoloOcean validation** (`results/targeted_validation/summary.json|csv`, criteria v9): 12 runs, seed 0, one run per
configuration. Seven runs are at code commit `ecd18ee`. The five with a silent or lost vehicle (`lost_drone_mutex`, `lost_drone_timeout`,
`lost_drone_rejoin`, `b2c_blackout`, `b2c_mute_transmitter`) were repeated at `a62c9b3` after the rule that a vehicle not heard is never
left out of the map-aiding identity premise (`perception/perception.py`, `perception/structure_aiding.py`); that rule only acts when a
vehicle stays unheard for 70 s or more. P1 and P2 hold in every run. The referee crossing budget (`referee/gate_events.py`) was updated
after the runs and the table is re-derived with it.

| Scenario | Protocol, noise | Class | P1 | P2 | Min. sep. (m) | Notes |
|---|---|---|---|---|---|---|
| line_parallel_mutex | B0, CLEAN | PASS | PASS | PASS | 3.19 | 3 vehicles abreast, gate at 118.1 s |
| line_parallel_mutex | B2_CLEAN, CLEAN | PASS | PASS | PASS | 2.91 | gate at 80.9 s |
| mutex_deadlock_resolution | B2_CLEAN, CLEAN | PASS | PASS | PASS | 3.39 | stacked queue |
| integrated_short | B2_CLEAN, CLEAN | PASS | PASS | PASS | 3.17 | survey, gate, re-form |
| lost_drone_mutex | B2_CLEAN, CLEAN | PASS | PASS | PASS | 3.45 | permanent failure before the gate, gate at 160.6 s |
| lost_drone_timeout | B2_CLEAN, CLEAN | PASS | PASS | n/a | 3.45 | degraded formation |
| lost_drone_rejoin | B2_CLEAN, CLEAN | PASS | PASS | n/a | 2.86 | 1 failsafe entry (envelope) |
| formation_recovery_head_current | B2_CLEAN, CLEAN | PASS | PASS | n/a | 3.44 | 1 failsafe entry (envelope) |
| b2c_blackout | B2_BLACKOUT, CLEAN | PASS | PASS | PASS | 3.20 | every packet lost, gate at 369.5 s (186.8 s before the map-aiding rule), see section 6 |
| gate_single | B2_CLEAN, MEDIUM-ALL | PASS | PASS | PASS | 3.02 | 9 `P2_UNCERTIFIED`, 21 failsafe entries |
| gate_single | B2_CLEAN, HIGH-ALL | PREMISE_VIOLATED | PASS | PASS | 3.25 | sensor-fault failsafe, no passage |
| b2c_mute_transmitter | B2_LOW_MUTE0, CLEAN | P3_FAILURE | PASS | PASS | 2.77 | "no false vacancy" missed; P3 contract met, recovery at 222.6 s |

One seed per configuration: observations, not a statistical bound. Raw run directories are kept outside the repository (local archive).

**Kinematic reconstruction F1pk** (reported; perception and plant idealised): 312 episodes per controller. The deployed controller has
0 entries into the CR without admission and 0 double occupancy; the mutation with the earlier hold point and no P2 filter has 126
episodes with an entry without admission. P1 is violated in 24 of 144 episodes, all against a non-reacting third vehicle.

**Baseline unit tests** (commit `e3e31f0`, one pytest process per file): 687 passed, 1 skipped, 0 failed. The development and complete runners above execute each formal suite at most once.

## 8. Reproducibility

Archived campaigns and their analysis scripts, historical result sets and earlier controller variants are indexed in
`legacy/INDEX.json` (original path, description, related tags and commits, sha256 and git blob id per item). Re-check the hashes
with `python scripts/verify_legacy_index.py` (also `tests/test_legacy_index.py`). Earlier states are tagged `b2-freeze-v4` to
`b2-freeze-v11` and `b2-v11-validation`. `scripts/build_archive.py` builds a versioned archive of raw runs outside the repository.
