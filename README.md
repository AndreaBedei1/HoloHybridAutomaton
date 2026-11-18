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
  the gate precedence, with an offline audit of its premise C against the integer-tick TDMA timing and the
  behaviour logged by each vehicle;
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
python scripts/audit_premise_c.py <out dir> <label>=<raw dir> ...   # premise C pairs and CR entries -> runs/pairs/entries CSV, summary JSON
python scripts/analyze_final_campaign.py <raw dir>              # acceptance criteria of the final matrix
python formal/check_properties.py                               # every Z3 suite -> formal/results/SUMMARY.{json,csv}
python scripts/build_archive.py --dest <root> --version <name> --refs <refs> --runs <raw dirs>   # versioned archive
```

The raw runs are not in the repository. They are kept in a versioned archive outside it; each version holds a git
bundle of the history, source tarballs and a `MANIFEST.json` with the sha256 of every file:
- `v13-final-v11-2026-11-18`: the final validation (raw runs, tables, logs, formal results);
- `v14-reference-runs-2026-11-18`: the CLEAN reference runs;
- `v12-review-2026-11-18`: the earlier campaigns, reports and reviews.
- `v16-premise-c-reaudit-2026-11-18`: the offline premise C and CR-entry re-audit of the final campaign, the
  post-freeze validation and the final validation (`runs.csv`, `pairs.csv`, `entries.csv`, `summary.json`);
  `v15-premise-c-reaudit-2026-11-18` is the first pass, kept unchanged, superseded by v16 (resolution of concurrent
  pairs from the logged end of each commitment).

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
