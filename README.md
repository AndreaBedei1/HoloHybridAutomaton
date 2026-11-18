# HoloHybridAutomaton

Decentralised coordination of BlueROV2 fleets in HoloOcean with identical local hybrid automata, formally checked with
Z3 and evaluated in simulation, with an optional one-way acoustic beacon.

Every vehicle runs its own copy of one local hybrid automaton on its own sonars, onboard navigation and a mission plan
shared before the dive. There is no leader and no request, reply or command between vehicles.

Principal contributions:

- a local hybrid automaton (10 modes, prioritised guards) whose determinism, priority hierarchy and observation
  interface are proved on its specification (`holo_fleet/ha/spec.py`);
- a sonar-only separation layer with a certified distance lower bound and a 3-D escape planner (P1);
- a gate protocol with a sector precedence rule, an occupancy latch and a stage-2 check, composed for 2 to 4 vehicles
  in a Z3 fleet model (P2);
- formation keeping, recovery and degraded formation after permanent failures with ranking-function certificates (P3);
- architecture B2: a one-way TDMA acoustic beacon (64-bit packet) that never enters separation, collision avoidance or
  the gate precedence, with an event-ordered runtime audit of its premise C;
- a referee that judges every run from ground truth, and a reproducible campaign runner.

The controller is the one frozen as tag `b2-freeze-v11`: its code is identical to that tag apart from comments and
ablation switches whose defaults keep the frozen behaviour (`FormationRule.neighbour_adaptation`, the sweep channels).

![automaton](figures/automaton/automaton_presentation.png)

## Architecture

```
sensors      six single-beam sonars, DVL, IMU, compass, depth; B2: acoustic beacon receiver
perception   holo_fleet/perception/  echo classification, distance bound, targets, gate belief, navigation, acoustic fixes
automaton    holo_fleet/ha/          one mode per vehicle; edges, guards and priorities in spec.py
flows        holo_fleet/control/     the velocity of the active mode, escape planner, low-level loops
referee      holo_fleet/referee/     evaluation from ground truth only (never fed to a controller)
```

Modes: `FORMATION_FOLLOW`, `FORMATION_WAIT_REJOIN`, `DEGRADED_FORMATION`, `FORMATION_RECOVERY`, `MUTEX_APPROACH`,
`MUTEX_YIELD`, `MUTEX_PASS`, `SEPARATION_WARNING`, `COLLISION_AVOIDANCE`, `FAILSAFE_HOLD_OR_RETREAT`. Priority:

```
FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX > FORMATION_RECOVERY > formation keeping
```

At a gate a vehicle waits at its queue point, commits when it has priority and believes the critical region (CR) free,
and checks once more at the stage-2 hold point before entering. In B2 each vehicle broadcasts position, error bound,
gate index and phase (CRC-16, period 1 s); receivers measure range and bearing themselves; packets older than 0.4 s on
delivery are dropped. Beacon data never reach separation, collision avoidance, the gate precedence or the sonar exit
witness (`tests/test_b2_isolation.py`); controllers never read ground truth (`tests/test_isolation.py`). B0 is the same
controller without the beacon.

## Guarantees and assumptions

Every proof is on an abstraction of the vehicles and their sensing, not on HoloOcean's physics. Kinds of evidence:
*proof* (Z3 UNSAT, unbounded for the encoded model, k-induction or inductive invariant), *BMC* (bounded to a stated
horizon), *exhaustive numeric*, *randomized* (evidence, not proof), *mutation* (a broken variant must give a
counterexample, so the check is not vacuous), *empirical* (HoloOcean runs). Separate proofs do not imply the
composition of the properties beyond the fleet model.

**P1, separation** (every pair of live vehicles stays at least 1 m apart).
Formally verified:
- the onboard distance is a lower bound of the true distance (randomized, about 35 000 scenes, plus mutation);
- from onboard distance >= d_warning the separation stays >= d_safe with both vehicles reacting (BMC S1a, proof S1b);
- the escape opens every threat pattern (exhaustive S2a-S2d).

Assumptions: echo error within the declared band, hull tilt <= 10 deg, speed, current and staleness bounds of
`config.Envelope`. With one vehicle reacting the bound holds at the simulator deceleration 0.743 m/s^2 (S1d), not at
the configured 0.5 m/s^2 (S1o). Where the echo error leaves the declared band (the MEDIUM profile is a stress level
beyond it), P1 is tested, not proved.

**P2, mutual exclusion** (at most one vehicle centre in the CR of a gate).
Formally verified:
- precedence: never two vehicles with priority (M1, M2);
- occupancy latch, bounded to 42 s (M3);
- claims held until the sender reports otherwise or is dead for the gate (S1, S4);
- fleet model for n = 2, 3, 4 with failures and stops: the invariant is inductive (F0) with claims learnt up to 2 steps
  late (F1d), and fleet recovery (F2).

The guarantee is conditional:
- every vehicle enters the CR through the stage-2 check, or is a committed vehicle moved in by separation or collision
  avoidance while holding its claim (F0p, F2p);
- entries by avoidance of a queued vehicle, or of two vehicles committed concurrently, are outside the proved
  assumptions;
- the referee classifies every CR entry as ADMITTED, PUSHED or UNCOMMITTED.

Premises under B2, each monitored per run:

| Premise | Monitor | When it fails |
|---|---|---|
| V: no live vehicle silenced for 70 s at a receiver that hears another | `PREMISE_V_VIOLATED` | a live vehicle can be declared dead for the gate |
| C: a queued vehicle learns of a commitment before it commits | event-ordered audit (`referee/premise_audit.py`) | a second vehicle commits; the stage-2 check holds it until the CR is seen free |
| H6: the sender's navigation error lies within its sent bound | `NAV_BOUND_PREMISE_VIOLATED` | payload rules are not used as P2 evidence |
| gate-frame navigation error <= 1 m | `GATE_NAV_PREMISE_VIOLATED` | admission waits for a validated fit |
| packets within `latency_max_s` = 0.4 s | received / accepted / rejected / stale counts | late packets are dropped; the gate runs on the sonar only |

**P3, formation** (slot recovery, fleet formation and degraded formation within deadlines).
Formally verified:
- ranking functions for each vehicle's estimated slot (F1-F3), not for the true formation;
- formation mode selection (F4);
- vacancy timers (F6);
- fleet re-formation after failures and stops end, in 26, 40 and 52 steps of 5 s for n = 2, 3, 4 (F2).

Assumptions: declared current envelope, at most n - 2 failures, fairness (a fresh fix within every 35 s). The deadlines
are requirements on the nominal dynamics, not proved bounds.

The referee judges three parts, with tolerance 1 m held for 8.9 s:

| Part | Required |
|---|---|
| P3a | each crossed vehicle returns to its slot |
| P3b | the fleet re-forms after the last crossing |
| P3c | after an admissible permanent failure, the neighbours declare the slot vacant within 47 s and no live slot is held vacant |

Deadline D = e0 / 0.10 m/s + 5 s + 8.9 s, extended while a higher-priority mode or a declared disturbance is active.

Run classes (`referee/robustness_metrics.py refine_v8`):

| Class | Meaning |
|---|---|
| PASS | mission complete and P3 met in time |
| P3_LATE | a part of P3 met after its deadline |
| P3_FAILURE | a part of P3 never met with the premises held |
| PREMISE_VIOLATED | the failure is explained by a measured premise |
| INCOMPLETE | gate or progress not reached |
| SAFETY_FAILURE, UNSAFE_ACTION, INVALID | the remaining classes |

Declared limits of liveness:
- a failure at or near the queue in still water;
- the localization at the gate under an ambiguous map fit;
- two vehicles, or the last survivor, after a failure while committed;
- a failure inside the onboard margin of the exclusion zone ends in a safe stop;
- in B0 a lost vehicle is vacated by a heuristic that also vacates live ones.

Threats to validity:
- simulation only, with one gate geometry;
- the declared current equals the true bound of each scenario;
- HoloOcean's own sensor noise is unseeded, so runs repeat statistically, not bit for bit. The wrapper noise, the
  acoustic channel and the controllers are seeded and deterministic.

## Installation

Windows 10/11, Python >= 3.9 (conda), HoloOcean 2.3.0 on Unreal Engine 5.3.2 (Win64 Development) with the sonar octree
patch.

```bash
conda create -n holo_fleet_ha --clone ocean -y        # "ocean": the environment with the HoloOcean 2.3.0 client
conda activate holo_fleet_ha
pip install -e .                                      # numpy, scipy, matplotlib, opencv-python, z3-solver, imageio, psutil
```

### Patched HoloOcean engine

Unpatched HoloOcean 2.3.0 sonars have four defects:
- they do not see props spawned at run time (the gates);
- they echo ghosts of earlier scenes from a stale octree cache;
- they crash after a reset that removed a runtime agent;
- they can miss agents depending on the order in which sonars are initialised.

`patches/holoocean-2.3.0-sonar-octree-rebuild.patch` adds the client command `env.rebuild_sonar_octree(region=None)`,
removes destroyed agents from the sonar loop and builds agent octrees with a clean query. It changes no sensor model,
parameter or message format. The patched engine lives in its own root; the official installation in
`%LOCALAPPDATA%\holoocean` is never modified. `holo_fleet/sim/holo_env.py` calls the rebuild once after spawning the
static scene.

In a HoloOcean 2.3.0 source checkout (here `F:\Andrea\holoocean-octree-patch`, branch `sonar-octree-rebuild` on top
of `pristine-2.3.0`):

```bash
git apply --check <repo>/patches/holoocean-2.3.0-sonar-octree-rebuild.patch
git apply <repo>/patches/holoocean-2.3.0-sonar-octree-rebuild.patch
pip install -e F:\Andrea\holoocean-octree-patch\client --no-deps   # patched client, into holo_fleet_ha
```

Then copy `patches/build_and_install_patched_holoocean.ps1` to the root of that checkout (it builds `engine/Holodeck.uproject`) and run:

```powershell
powershell -ExecutionPolicy Bypass -File build_and_install_patched_holoocean.ps1 -UE "C:\Program Files\Epic Games\UE_5.3" -Root F:\Andrea\holoocean_patched_root
```

The root holds `PATCH_INFO.json` with the source commit and exe SHA-256. The campaign runner refuses to run without the
patched root and records the SHA-256 of its binaries in every run.

Verify the engine (about 1 minute; every check must be true; writes `results/calibration/octree_patch/regression.json`):

```bash
python calibration/octree_rebuild_regression.py
python -m pytest -q tests/test_octree_patch.py
```

To roll back, unset `HOLO_FLEET_HOLOOCEAN_ROOT` / `HOLODECKPATH` and reinstall the official client
(`pip install holoocean==2.3.0`); gate scenarios then lose their structure echoes and the wrapper warns.

| Variable | Default | Effect |
|---|---|---|
| `HOLO_FLEET_HOLOOCEAN_ROOT` | `F:\Andrea\holoocean_patched_root` | root of the patched engine |
| `MARINE_RACE_ARENA_ROOT` | `~/Desktop/HoloDroneCompetition` | Marine Race Arena (gate G06, needed by the gate scenarios) |
| `HOLO_FLEET_ENGINE_SLOTS` | 1 | concurrent HoloOcean instances (1 to 4); each slot needs a private root: `powershell -ExecutionPolicy Bypass -Command "& ./scripts/prepare_engine_roots.ps1 -Slots 0,1,2,3"` |
| `HOLO_FLEET_SLOT_WAIT_S` | 0 | seconds a run waits for a free slot |

Check without the engine: `python -m pytest -q -m "not slow and not holoocean"`.

## Interactive viewer and demonstrations

```bash
python scripts/run_demo.py --list                                   # scenarios with duration and title
python scripts/run_demo.py --scenario gate_single                   # HoloOcean viewport + Fleet View
python scripts/run_demo.py --scenario lost_drone_rejoin --select 2  # automaton panel of drone 2
```

The Fleet View shows the fleet from above with the planned slots, P1/P2/P3 status, events and the automaton panel of
the selected vehicle (click a vehicle or press `0`-`9`). A demo runs until the mission is complete (plus 8 s) or its
deadline. Options: `--headless` (no viewport), `--no-window` (no Fleet View), `--duration S`, `--save [--out DIR]`
(keep the run; without it nothing is written).

| Topic | Scenario | What to look for |
|---|---|---|
| Collision avoidance | `p1_head_on`, `p1_vertical_escape` | warning and 3-D escape, separation >= 1 m |
| Gate mutual exclusion | `gate_single`, `mutex_deadlock_resolution`, `gate_four` | queue, one vehicle at a time in the CR |
| Formation recovery | `formation_recovery_head_current`, `formation_gust` | formation lost and recovered |
| Displaced vehicle | `lost_drone_rejoin` | reduced progress and rejoin from behind |
| Permanent failure | `lost_drone_timeout`, `lost_drone_mutex` | vacancy, degraded formation, gate order without the lost vehicle |
| Integrated mission | `integrated_short` | survey, cross-current at the gate, one at a time, re-form |

## Configuration: B0/B2, noise profiles, seeds

Every threshold, gain and envelope is a field of the frozen dataclass `FleetConfig` (`holo_fleet/config.py`), also
imported by the formal suites; change values with `dataclasses.replace`. The source, class and validity of every
parameter are in `config/parameter_sources.json` (citation keys in `references.bib`).

| Choice | Demo (`run_demo.py`) | Campaign runner (`--protocols` / `--profiles` / `--seeds`) |
|---|---|---|
| B0 (no beacon) | default | `B0` |
| B2 beacon on a channel level | `--b2 CLEAN\|LOW\|MEDIUM\|BLACKOUT` | `B2_<level>`, also `MEDIUM_HT`; one-factor sweeps `B2_SW-<key><value>`, keys `per lat jit loss burst asym` (e.g. `B2_SW-lat0.35`); ablations `B2NP_<level>` (no payload), `B2NA_<level>` (no neighbour adaptation) |
| Sensor noise | `--profile CLEAN\|LOW\|MEDIUM\|HIGH` | `CLEAN` or `<LEVEL>-<SCOPE>`, LEVEL in `LOW MEDIUM MEDIUM_HB3 MEDIUM_HB4`, SCOPE in `ALL SUBSET ONE` (fixed drones in `config/scope_drones.json`) |
| Seed | `--seed N` | `--seeds N ...` |

## Reproducing the experiments

`scripts/run_sensor_robustness_campaign.py` runs each simulation in its own process at below-normal priority, with a
watchdog, resumes interrupted batches and records commit, engine build, configuration and seeds in `run_config.json`.
The `final` and `validation` phases refuse to run from a modified working tree; use a clean checkout (e.g. a git
worktree) of the commit to test.

```bash
python scripts/run_sensor_robustness_campaign.py --phase validation --raw <raw dir>   # final validation, 34 runs
python scripts/run_sensor_robustness_campaign.py --phase final --raw <raw dir>        # frozen campaign matrix, 448 runs
python scripts/run_sensor_robustness_campaign.py --phase targeted --only gate_single \
    --protocols B2_MEDIUM B0 --profiles MEDIUM-ALL --seeds 1 2 --raw <raw dir>
python scripts/run_sensor_robustness_campaign.py --phase validation --raw <raw dir> --dry-run   # list the runs
```

Shard over engines with `--shard i --nshards n` and `HOLO_FLEET_ENGINE_SLOTS=n`. Each run directory holds
`run_config.json`, per-vehicle state logs, `events.jsonl`, `referee_timeseries.csv`, `referee_metrics.json` and
`run_metrics.json` (class, terminal outcome, premise C audit, CR entries, beacon statistics).

Analysis (no simulator):

```bash
python scripts/analyze_validation.py <out dir> <raw dir>        # tables (VALIDATION_TABLES.md) and figures
python scripts/audit_premise_c.py <out.json> <raw dir>          # premise C and every CR entry, from the logs
python scripts/analyze_final_campaign.py <raw dir>              # acceptance criteria of the final matrix
python formal/check_properties.py                               # every Z3 suite -> formal/results/SUMMARY.{json,csv}
python scripts/build_archive.py --dest <root> --version <name> --refs <refs> --runs <raw dirs>   # versioned archive
```

The raw runs are not in the repository. They are kept in a versioned archive outside it; each version holds a git
bundle of the history, source tarballs and a `MANIFEST.json` with the sha256 of every file:
- `v13-final-v11-2026-11-18`: the final validation (raw runs, tables, logs, formal results);
- `v14-reference-runs-2026-11-18`: the CLEAN reference runs;
- `v12-review-2026-11-18`: the earlier campaigns, reports and reviews.

The formal results of every check (formula, verdict, counterexample) are in `formal/results/SUMMARY.csv` and
`SUMMARY.json`.

## Repository layout

```
holo_fleet/   automaton, controllers, perception, simulation, referee, Fleet View; runner.py runs one scenario
formal/       Z3 suites; formal/results/ (SUMMARY.csv, SUMMARY.json, one JSON per suite)
scripts/      demos, campaign runner, analysis, archive builder, engine slots and isolation probe
calibration/  simulator probes behind the measured parameters (outputs in results/calibration/, read by the code)
config/       parameter_sources.json (parameter provenance registry), scope_drones.json
tests/        unit, integration and isolation tests
patches/      HoloOcean patch and its build script
figures/      automaton diagrams and calibration figures
```
