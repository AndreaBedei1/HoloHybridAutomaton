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
and checks once more at the stage-2 hold point before entering. Separation and collision avoidance rank above the gate:
they may move a held vehicle into the CR (see the declared limits below). In B2 each vehicle broadcasts position, error
bound, gate index and phase (CRC-16, period 1 s); receivers measure range and bearing themselves; packets older than
0.4 s on delivery are dropped. B0 is the same controller without the beacon.

## Properties and their scope

| Property | Statement | Formally verified (on an abstraction) | Conditional on |
|---|---|---|---|
| **P1** separation | every pair of live vehicles stays >= 1 m apart | distance bound (randomized + mutation), S1a BMC, S1b k-induction, escape S2a-S2d | echo error within the declared band, speed/current/staleness envelope (`config.Envelope`); one-vehicle reaction only at the simulator deceleration (S1d) |
| **P2** mutual exclusion | at most one vehicle centre in the CR of a gate | M1, M2 precedence; M3 latch (BMC 42 s); fleet model F0 (inductive, n = 2, 3, 4), F1d (claims up to 2 steps late), F0p/F2p (one avoidance push of a committed vehicle) | premises V, C, H6 and the 1 m gate-frame navigation error, monitored per run |
| **P3** formation | slot recovery, fleet formation and degraded formation within deadlines | ranking functions F1-F3 (convergence of each vehicle's estimated slot), vacancy timers F6, fleet recovery F2 | current within control authority, navigation error, fairness, admissible failures; the deadlines themselves are not proved |

Declared limits, checked as satisfiable and not excluded: two vehicles that commit together, with claims one step late,
both pushed into the CR by avoidance (F1p); a queued, uncommitted vehicle pushed into the CR (F0q); the escape planner is
not antisymmetric along the gate axis (F1pa); a stopped occupant beyond the 32 s horizon with no claim heard (S2hs).
The referee classifies every CR entry as ADMITTED, PUSHED or UNCOMMITTED. Separate proofs do not imply the composition
of the three properties beyond the fleet model. Premises, monitors, failure behaviour, the P3 contract and the run
classes are in [`docs/GUARANTEES.md`](docs/GUARANTEES.md).

## Installation

Windows 10/11, Python >= 3.9 (conda), HoloOcean 2.3.0 on Unreal Engine 5.3.2 with the sonar octree patch.

```bash
conda create -n holo_fleet_ha --clone ocean -y        # "ocean": the environment with the HoloOcean 2.3.0 client
conda activate holo_fleet_ha
pip install -e .                                      # numpy, scipy, matplotlib, opencv-python, z3-solver, imageio, psutil
```

Unpatched HoloOcean 2.3.0 sonars do not see props spawned at run time (the gates). The project runs on a patched engine
in its own root; the official installation is not modified ([`docs/HOLOOCEAN_OCTREE_PATCH.md`](docs/HOLOOCEAN_OCTREE_PATCH.md)):

```bash
pip install -e <holoocean-octree-patch>/client --no-deps
powershell -ExecutionPolicy Bypass -File patches/build_and_install_patched_holoocean.ps1
```

The patch is `patches/holoocean-2.3.0-sonar-octree-rebuild.patch`. The campaign runner refuses to run without the
patched root and records the sha256 of its binaries in every run. The gate scenarios need gate G06 of the Marine Race
Arena.

| Variable | Default | Effect |
|---|---|---|
| `HOLO_FLEET_HOLOOCEAN_ROOT` | `F:\Andrea\holoocean_patched_root` | root of the patched engine |
| `MARINE_RACE_ARENA_ROOT` | `~/Desktop/HoloDroneCompetition` | Marine Race Arena (gate G06) |
| `HOLO_FLEET_ENGINE_SLOTS` | 1 | concurrent HoloOcean instances (1 to 4); each slot needs a private root: `powershell -ExecutionPolicy Bypass -Command "& ./scripts/prepare_engine_roots.ps1 -Slots 0,1,2,3"` |
| `HOLO_FLEET_SLOT_WAIT_S` | 0 | seconds a run waits for a free slot |

Check: `python -m pytest -q -m "not slow and not holoocean"` (no engine needed).

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
imported by the formal suites. Change values with `dataclasses.replace`, never by mutation. Parameter sources:
[`docs/PARAMETER_PROVENANCE.md`](docs/PARAMETER_PROVENANCE.md), `config/parameter_sources.json`, `references.bib`.

| Choice | Demo (`run_demo.py`) | Campaign runner (`--protocols` / `--profiles` / `--seeds`) |
|---|---|---|
| B0 (no beacon) | default | `B0` |
| B2 beacon on a channel level | `--b2 CLEAN\|LOW\|MEDIUM\|BLACKOUT` | `B2_<level>`; also `MEDIUM_HT`; one-factor sweeps `B2_SW-<key><value>`, keys `per lat jit loss burst asym` (e.g. `B2_SW-lat0.35`); ablations `B2NP_<level>` (no payload), `B2NA_<level>` (no neighbour adaptation) |
| Sensor noise | `--profile CLEAN\|LOW\|MEDIUM\|HIGH` | `CLEAN` or `<LEVEL>-<SCOPE>`, LEVEL in `LOW MEDIUM MEDIUM_HB3 MEDIUM_HB4`, SCOPE in `ALL SUBSET ONE` (fixed drones in `config/scope_drones.json`) |
| Seed | `--seed N` | `--seeds N ...` |

The seed drives the wrapper sensor noise and the acoustic channel; controllers are deterministic. HoloOcean's own
sensor noise is unseeded, so runs repeat statistically, not bit for bit.

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
python scripts/build_archive.py --dest <root> --version <name> --refs <refs> --runs <raw dirs>   # versioned archive
```

The raw runs are not in the repository. They are kept in a versioned archive outside it, each version with a git
bundle of the history, source tarballs and a `MANIFEST.json` with the sha256 of every file:
`v13-final-v11-2026-11-18` holds the final validation (raw runs, tables, logs, formal results);
`v14-reference-runs-2026-11-18` the CLEAN reference runs; `v12-review-2026-11-18` the earlier campaigns, reports and
reviews.

## Results

**Formal verification** (`python formal/check_properties.py`, summary in
[`formal/results/SUMMARY.md`](formal/results/SUMMARY.md)): 300 of 300 checks return their expected verdict (UNSAT for
properties, SAT for mutations, witnesses and declared limits).

| Suite | Checks |
|---|---|
| Local determinism and priority hierarchy | 104 |
| Observation consistency | 32 |
| P1 separation | 15 |
| P2 mutual exclusion | 45 |
| P3 formation recovery | 24 |
| B2 range-bearing beacon | 13 |
| Gate-status payload, silence, verified passage | 32 |
| Fleet composition at the gate | 35 |

**Final validation** (empirical; `--phase validation`, seeds 8000-8001, commit d5f424e, one engine build, 34 runs):

| Protocol / profile | Scenarios | Runs | Result |
|---|---|---|---|
| B2 MEDIUM / MEDIUM-ALL | p1_head_on, formation_six, lost_drone_rejoin, gate_single, mutex_deadlock_resolution, lost_drone_mutex, integrated_short | 14 | 14 PASS |
| B0 / LOW-ALL | the same seven | 14 | 11 PASS; `lost_drone_mutex` 2 P3_FAILURE (false vacancy); `integrated_short` 1 P3_LATE |
| B2 latency 0.35 / 0.45 s, LOW-ALL | gate_single | 4 | 4 PASS |
| B2 MEDIUM / MEDIUM-ALL | gate_four | 2 | 2 PASS |

P1 and P2 held in all 34 runs (smallest true separation 2.84 m). Premise C: 64 pairs SATISFIED, none CONCURRENT,
VIOLATED or INCONCLUSIVE; 64 CR entries, all ADMITTED. Packets accepted on board: 97.4 % on B2 MEDIUM, 67.4 % at
0.35 s latency (32.0 % stale), 0 % at 0.45 s (100 % stale). The B0 P3 failures are the declared B0 limit: without a
beacon a lost vehicle is vacated by a heuristic that also vacates live ones.

Unit and integration tests: `python -m pytest -q -m "not slow and not holoocean"`; everything, including the Z3 and
HoloOcean tests: `python -m pytest -q`.

## Repository layout

```
holo_fleet/   automaton, controllers, perception, simulation, referee, Fleet View; runner.py runs one scenario
formal/       Z3 suites and formal/results/
scripts/      demos, campaign runner, analysis, archive builder, engine slots and isolation probe
calibration/  simulator probes behind the measured parameters (outputs in results/calibration/, read by the code)
config/       parameter registry, fixed scope drones
tests/        unit, integration and isolation tests
patches/      HoloOcean patch and its build script
figures/      automaton diagrams and calibration figures
docs/         GUARANTEES.md, PARAMETER_PROVENANCE.md, HOLOOCEAN_OCTREE_PATCH.md
```
