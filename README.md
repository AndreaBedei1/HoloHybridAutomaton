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
witness (`tests/test_b2_isolation.py`); controllers never read ground truth (`tests/test_isolation.py`). B2 on the MEDIUM
channel is the final architecture; B0, the same controller without the beacon, is the communication-free baseline.

## Installation

Requirements: Windows 10/11, Python >= 3.9 (conda), [HoloOcean](https://github.com/byu-holoocean/HoloOcean) 2.3.0
with its Ocean world, Unreal Engine 5.3 (to build the patched engine) and the Marine Race Arena.

1. **HoloOcean 2.3.0.** Install the official HoloOcean 2.3.0 client and the Ocean world as described in the
   [HoloOcean 2.3.0 documentation](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/). The official installation
   is used as the source of the world content and is never modified.

2. **Project environment.**

   ```bash
   conda create -n holo_fleet_ha --clone <environment with the HoloOcean 2.3.0 client> -y
   conda activate holo_fleet_ha
   pip install -e .        # numpy, scipy, matplotlib, opencv-python, z3-solver, imageio, psutil
   ```

3. **Marine Race Arena** (gate G06 of the gate scenarios;
   [doi:10.5281/zenodo.22757860](https://doi.org/10.5281/zenodo.22757860)). Clone it beside this repository, or
   anywhere and set `MARINE_RACE_ARENA_ROOT` to the clone:

   ```bash
   git clone https://github.com/AndreaBedei1/HoloDroneCompetition.git
   ```

4. **Patched HoloOcean engine.** Unpatched HoloOcean 2.3.0 sonars have four defects:
   - they do not see props spawned at run time, such as the gates;
   - they echo ghosts of earlier scenes from a stale octree cache;
   - they crash after a reset that removed a runtime agent;
   - they can miss agents depending on sonar initialisation order.

   `patches/holoocean-2.3.0-sonar-octree-rebuild.patch` fixes this. It touches 12 files of the engine C++ sources and
   the Python client, and adds `env.rebuild_sonar_octree(region=None)`, which `holo_fleet/sim/holo_env.py` calls once
   after spawning the static scene. It changes no sensor model, parameter or message format. The engine is not
   distributed here: it is built from the HoloOcean 2.3.0 sources with the patch applied.

   ```bash
   git clone https://github.com/byu-holoocean/HoloOcean.git && cd HoloOcean   # check out the 2.3.0 sources
   git apply --check <this repo>/patches/holoocean-2.3.0-sonar-octree-rebuild.patch    # verifies the base
   git apply <this repo>/patches/holoocean-2.3.0-sonar-octree-rebuild.patch
   pip install -e client --no-deps                                                     # patched client
   ```

   Copy `patches/build_and_install_patched_holoocean.ps1` into the root of that checkout (it builds
   `engine/Holodeck.uproject`) and run it:

   ```powershell
   powershell -ExecutionPolicy Bypass -File build_and_install_patched_holoocean.ps1 -UE "<Unreal Engine 5.3 folder>" -Root <patched root>
   ```

   The script builds the Win64 Development engine and creates the patched root (default
   `%USERPROFILE%\holoocean_patched_root`). In that root it reuses the official world content through junctions and
   writes `PATCH_INFO.json` with the source commit and the SHA-256 of the executable. The official installation in
   `%LOCALAPPDATA%\holoocean` stays untouched.

5. **Verify the engine.** This takes about 1 minute; every check must be true:

   ```bash
   python calibration/octree_rebuild_regression.py      # writes results/calibration/octree_patch/regression.json
   python -m pytest -q tests/test_octree_patch.py
   ```

   The campaign runner refuses to run without the patched root and records its binaries' SHA-256 in every run.

| Variable | Default | Effect |
|---|---|---|
| `HOLO_FLEET_HOLOOCEAN_ROOT` | `~/holoocean_patched_root` | root of the patched engine |
| `MARINE_RACE_ARENA_ROOT` | `../HoloDroneCompetition` (beside this repository) | Marine Race Arena checkout |
| `HOLO_FLEET_ENGINE_SLOTS` | 1 | concurrent HoloOcean instances (1 to 4); each slot needs a private root: `powershell -ExecutionPolicy Bypass -Command "& ./scripts/prepare_engine_roots.ps1 -SourceRoot <patched root> -Slots 0,1,2,3"` |
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

## Guarantees and limitations

Three kinds of statement are kept apart: formal results on abstractions (Z3), simulation observations, and engineering
assumptions. `formal/results/SUMMARY.json` gives every check a `kind`; only `proved` and `bounded` support a guarantee. An
expected counterexample (`limit`, `scope`, `mutation`, `witness`) is never evidence of safety, and `open_limits` lists the
unresolved ones. Current run: 302 checks, all with the expected verdict. They are 146 proved, 33 bounded, 37
numeric, 6 open limits, 10 scope, 40 mutations, 23 witnesses and 7 reported.

**Formally proved on the abstractions.**
- Determinism and priority hierarchy of the automaton for every legal observation (`holo_fleet/ha/spec.py`).
- P1, pairwise: the onboard distance lower bound, the escape and warning-filter guarantees, and `d >= d_safe = 1.0 m` for
  all time by k-induction. The triangle lemma extends the pairwise argument to a fleet.
- P2 at one gate, when every claim is learnt within one step of 5 s (premise C): at most one vehicle committed or in the
  CR. The invariant is inductive for 2, 3 and 4 vehicles, with failures, stops and entries by avoidance of one committed
  vehicle (`F0s`, `F0p`). The queue precedence holds for 2 to 6 queue points.
- P3: ranking functions of slot tracking (along and lateral), and the vacancy of a neighbour that never reappears.

**Bounded model checking only.**
- The occupancy latch M3: 42 s horizon.
- The status freshness S2: 32 s.
- The P1 start-up window S1a: 4 s.
- Recovery of the fleet within 26, 40 and 52 steps for 2, 3 and 4 vehicles (`F2`, `F2p`). These start from every state of
  the invariant, with no further failure or stop.
- P2 with claims learnt up to 2 steps late, without entries by avoidance (`F1d1`, `F1d2`; the bound is tight, `F1d3`).

**Supported fleet sizes.** P2 and recovery: 2 to 4 vehicles at one gate. Queue layouts:
- abreast for 2 or 3 vehicles;
- stacked 2 x 2 and the 3-vehicle stack pair;
- abreast queues of 4 or more are refused by the planner.

P1 is proved pairwise. Several neighbours are covered by the triangle lemma and the two-threat escape, not by a fleet
model. Formation keeping is per vehicle; `formation_six` (6 vehicles) is simulated only.

**Assumptions** (provenance and limits in `config/parameter_sources.json`).
- **Environment.**
  - Currents inside the control-feasibility envelope (`control/current_envelope.py`).
  - Unrejected drift at most 0.15 m/s for P1.
  - For P3, a residual disturbance of at most 0.05 m/s once the integrator has converged. This rests on window means of
    one simulated vehicle at survey speed. The proof fails above 0.064 m/s.
- **Sensors.** The sonar model of `perception/sonar_geometry.py`, hull tilt within the bound of
  `perception/distance_bound.py`, and the declared navigation error (1 m in the gate frame).
- **Communication.**
  - B0 needs none.
  - B2 adds claims and assumes four premises:
    - **C:** a claim is known before another vehicle can commit. The premise C audit checks it at runtime.
    - **V:** no live vehicle outside the exclusion zone is silent for 70 s.
    - **Fairness:** a 35 s fairness window of the fixes. It holds in every gate scenario of the validation, at most 26.3 s,
      and is exceeded in `formation_six`, which is outside the fleet model.
    - **Freshness:** packets older than 0.4 s on delivery are dropped.
- **Failures.**
  - Permanent: a failed vehicle stops moving and transmitting.
  - Never inside the CR, and only outside the exclusion zone of `holo_fleet/scope.py`.
  - At most 1 failure for 2 vehicles (none while committed), 1 for 3, and 2 for 4. A sunk or risen hull stops counting
    as queued (premise H).

**Open limitations** (expected counterexamples, unresolved).
- **F1p / F1pm.** A concurrent commitment combined with one entry into the CR by avoidance can put two vehicles in the CR,
  in the abstraction. The counterexample needs a claim learnt one step late (two vehicles committed together). One vehicle
  enters by its own stage-2 check, or is moved in; the other, held by that claim, is moved in by avoidance while the first
  is inside.
  - It survives the motion limit of an entry by avoidance (`F1pm`: at least 6.1 s from the queue point to the CR).
  - The deployed escape planner is not antisymmetric along the gate axis (`F1pa`), and no gate rule edits the avoidance
    output of a committed vehicle. The model has no positions to exclude it.
  - A kinematic reconstruction with the deployed gate flow and escape planner, with idealised sensing (`F1pk`, reported),
    finds single entries by avoidance in 48 of 384 episodes and two vehicles in the CR in none.
    - It finds none either when no claim is ever known, and 20 of 24 when avoidance is also removed.
    - So in that reconstruction the separation layer and the pass path converging on the gate axis keep two vehicles out of
      the CR together, not the gate protocol.
    - This suggests that a realisation of F1p needs sensing or dynamics outside those idealisations. It is not a proof, and
      it does not test the stage-2 logic.
  - P2 is therefore guaranteed only while premise C holds or while no committed vehicle is moved into the CR by
    avoidance during a concurrent commitment.
- **F0q.** A queued, uncommitted vehicle moved into the CR by avoidance.
- **F2s2, F2s3.** A failure while committed that leaves a single survivor.
- **F2s4.** A hull floating above the queue (no premise H).
- The `scope` checks mark the other bounds: a vehicle stopped before the CR beyond the latch horizon, an occupant that
  stops inside the CR, and recovery without fairness.
- Not covered by any proof: failures inside the exclusion zone, the continuous geometry inside the CR, and field
  conditions. Every result is on HoloOcean models.

**What the final validation shows.** Archive `v13-final-v11-2026-11-18`: 34 runs of the v11 controller at commit `d5f424e`.
- **Configurations:** 7 scenarios in B0 (LOW noise), 8 in B2 MEDIUM, and gate_single at beacon latencies of 0.35 s and
  0.45 s; seeds 8000 and 8001.
- **P1 and P2:** both held in every run, with a minimum separation of 2.84 m and at most one vehicle in the CR. All 64 CR
  entries were admitted by the stage-2 check, none by avoidance.
- **Classes:** 31 PASS. The B2 MEDIUM runs are 16 of 16 PASS. In B0, `lost_drone_mutex` failed P3 on both seeds (a false
  vacancy) and `integrated_short` s8000 was P3_LATE.
- **Premise C:** the audit at that commit classified all 64 pairs SATISFIED. The corrected audit, re-run on the unchanged
  raw logs (separate archive `v17-premise-c-v13-reaudit-2026-11-19`, original outcome alongside), gives the same 64
  SATISFIED:
  - observable blocking: 57 blocked by the latch, 7 hand-offs after an exit witness;
  - claim knowledge: 36 certified by a usable claim, 8 claims undeliverable by the channel timing (the 0.45 s latency runs,
    and slot 2 at 0.35 s), 20 not applicable (B0).
- No concurrent commitment occurred, so these runs do not exercise F1p.
- They are observations on a finite matrix, not a statistical bound. The 448-run campaign is a separate dataset and is not
  aggregated with them.

The archives are on local storage only. Each has a `MANIFEST.json` with the sha256 of every file, and v13 also holds the
code tarballs, a git bundle and every `run_config.json`; its checksums were verified on 2026-11-19. They have no public
location, so the results can be reproduced only by re-running the campaign from this repository.

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

The raw runs are not in the repository. The final validation is kept in the versioned archive
`v13-final-v11-2026-11-18` outside it (raw runs, tables, logs, formal results, a git bundle of the history, source
tarballs and a `MANIFEST.json` with the sha256 of every file).

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
