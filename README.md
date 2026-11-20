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
- a gate-aware safety layer: mutual exclusion as a constraint on the final velocity command of every vehicle not
  admitted to the CR, a stage-2 hold placed for the declared navigation error, and avoidance choices classified as
  jointly feasible, uncertain or jointly infeasible (`holo_fleet/control/gate_safety.py`);
- formation keeping, recovery and degraded formation after permanent failures with ranking-function certificates (P3);
- architecture B2: a one-way TDMA acoustic beacon (64-bit packet) that never enters separation, collision avoidance or
  the gate precedence, with an event-ordered runtime audit of its premise C;
- a referee that judges every run from ground truth, and a reproducible campaign runner.

Two controller versions are documented. **v11** is the one frozen as tag `b2-freeze-v11`, validated by the 34-run archive
`v13-final-v11-2026-11-18` and the separate 448-run campaign. The **current controller** is v11 with the gate-safety
revision (stage-2 hold, P2 filter, gate-aware avoidance, queue lines, standoff). The v11 evidence does not apply to it;
its own targeted HoloOcean runs are summarised in `results/gate_safety_v1/`.

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
and checks once more at the stage-2 hold point, 2.30 m before the gate centre, before entering. Until it is admitted,
no command of any mode, avoidance included, may let its true centre reach the CR under the declared bounds. In B2 each vehicle broadcasts position, error bound,
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
python scripts/run_demo.py --scenario gate_single                   # HoloOcean viewport + HoloFleetViewer
python scripts/run_demo.py --scenario lost_drone_rejoin --select 2  # automaton panel of drone 2
```

HoloFleetViewer shows the fleet from above with the planned slots, P1/P2/P3 status, events and the automaton panel
of the selected vehicle (click a vehicle or press `0`-`9`). It keeps every step: during the run and after it, the arrow
keys move back and forth (left/right 0.1 s, up/down 1 s, PgUp/PgDn 10 s, Home/End), Space pauses and plays on from the
paused instant, End returns to live; the window stays open after the run until it is closed (X, `q` or `Esc`). A demo
runs until the mission is complete (plus 8 s) or its deadline. Options: `--headless` (no viewport), `--no-window` (no
HoloFleetViewer), `--duration S`, `--save [--out DIR]` (keep the run; without it nothing is written).

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
| B0 (no beacon) | `--b2 OFF` | `B0` |
| B2 beacon on a channel level | `--b2 CLEAN\|LOW\|MEDIUM\|BLACKOUT` (default `CLEAN`) | `B2_<level>`, also `MEDIUM_HT`; one-factor sweeps `B2_SW-<key><value>`, keys `per lat jit loss burst asym` (e.g. `B2_SW-lat0.35`); ablations `B2NP_<level>` (no payload), `B2NA_<level>` (no neighbour adaptation) |
| Sensor noise | `--profile CLEAN\|LOW\|MEDIUM\|HIGH` | `CLEAN` or `<LEVEL>-<SCOPE>`, LEVEL in `LOW MEDIUM MEDIUM_HB3 MEDIUM_HB4`, SCOPE in `ALL SUBSET ONE` (fixed drones in `config/scope_drones.json`) |
| Seed | `--seed N` | `--seeds N ...` |

## Guarantees and limitations

Three kinds of statement are kept apart: formal results on abstractions (Z3), simulation observations, and engineering
assumptions. `formal/results/SUMMARY.json` gives every check a `kind`; only `proved` and `bounded` support a guarantee, and
`numeric` checks are exhaustive or sampled evaluations of the deployed code. An expected counterexample (`limit`, `scope`,
`mutation`, `witness`) is never evidence of safety, and `open_limits` lists the unresolved ones. Current run: 322 checks, all with the expected
verdict: 148 proved, 35 bounded, 45 numeric, 7 open limits, 11 scope, 44 mutations, 23 witnesses and 9 reported.

The guarantees are conditional. P1 holds where a safe trajectory stays dynamically feasible under the declared sensing,
dynamics and drift; P2 holds while the declared navigation and drift bounds hold. Outside that envelope the controller
reports which property it can no longer certify instead of claiming it.

**Formally proved on the abstractions.**
- Determinism and priority hierarchy of the automaton for every legal observation (`holo_fleet/ha/spec.py`).
- P1, pairwise: the onboard distance lower bound, the escape and warning-filter guarantees, and `d >= d_safe = 1.0 m` for
  all time by k-induction. The triangle lemma extends the pairwise argument to a fleet.
- P1 with a gate-constrained vehicle (`S1gb`, k-induction): a vehicle that may only stop, paired with one that opens at
  `v_open`, keeps `d >= d_safe`. Two vehicles that may only stop do not (`S1gm`, the jointly infeasible pair).
- P2 at one gate, when every claim is learnt within one step of 5 s (premise C): at most one vehicle committed or in the
  CR. The invariant is inductive for 2, 3 and 4 vehicles, with failures and stops (`F0s`, `F0p`). The queue precedence
  holds for 2 to 6 queue points.
- The P2 test of one command (`P2B1`): a positive margin, a navigation error within the declared bound and a displacement
  within the per-axis reach keep the true centre out of the CR.
- P3: ranking functions of slot tracking (along and lateral), and the vacancy of a neighbour that never reappears.

**Bounded model checking only.**
- The occupancy latch M3: 42 s horizon. The status freshness S2: 32 s. The P1 start-up window S1a: 4 s; the
  gate-constrained pair `S1ga`: 6 s.
- Recovery of the fleet within 26, 40 and 52 steps for 2, 3 and 4 vehicles (`F2`, `F2p`), from every state of the
  invariant with no further failure or stop.
- P2 with claims learnt up to 2 steps late (`F1d1`, `F1d2`; tight, `F1d3`), and the combined case `F1p` for the current
  controller: concurrent commitments and the stage-2 check with entries by avoidance excluded by the P2 filter.

**Numeric checks of the deployed gate-safety code** (`formal/check_gate_safety.py`).
- The reach is never under-estimated (`P2B1r`, error below 3e-4 m against a 0.05 m slack). No command called P2-safe
  enters the CR on adversarial trajectories inside the declared envelope (`P2B3`), nor passes under or over it (`P2B6`).
- Station keeping on a latched anchor keeps its margin while its deviation stays within `e_track` (`P2B4`; 0.051 m left
  at the stage-2 target with the measured velocity), and every command chosen with a positive robust margin leaves
  braking P2-safe one step later (`P2B5`).
- Over every single and paired sector pattern and positions around the hold, the gate-aware planner returns a P2-safe
  command whenever one exists and the full P1 share whenever a P2-safe one gives it (`J1`, 25,272 decisions).
- Queue geometry (`Q1`): the stage-2 hold keeps 3.2 m (centre) from every other queue point for the abreast and stacked
  layouts; the thinnest conservative distance is 2.43 m against `d_warning` 2.4 m (abreast, 3 vehicles).
- Strict invariance of the one-step margin fails (`P2B2s`, a declared scope): the latency drift is re-applied at every step.
  This is why the deployed selection uses the robust margin and the latched station keeping.

**Gate safety of the current controller** (`holo_fleet/control/gate_safety.py`, `escape.py`, `flows.py`).
- **Defect fixed.** The v11 stage-2 hold was at `s = -1.179 m`, 0.38 m from the CR, against a declared gate-frame
  navigation error of 1 m: an estimate 1 m short put the centre at `s = -0.179 m`, inside the CR (`P2Bm`). The hold is now
  `GateRule.stage2_hold_s = -2.3025 m`: the nearest point where holding is P2-safe,
  `-(0.8 + 1.0 + 0.15 x 0.35 + 0.30 + 0.05) = -2.2025 m`, minus the measured holding error of 0.10 m. The terms are the CR
  half-length, the navigation error, the drift during the decision latency, the hold-loop deviation against the declared
  drift and a slack. The CR occupancy is the centre definition of P2; the hull dimension enters P1 and the structure
  keep-out, not this margin.
- **P2 filter.** For every vehicle not admitted, each command is P2-safe only if its backup cannot reach the CR column.
  The backup is the command for one control period plus the largest data age, braking at `a_brake`, then station keeping.
  It includes the navigation error on every axis, the measured-velocity error and the drift. A command is also required to
  leave braking safe one step later. The filter edits mission commands and selects among avoidance candidates; an
  emergency manoeuvre is never an authorization to enter.
- **Gate-aware avoidance.** The warning filter and the escape choose with their own rules among the P2-safe candidates
  (directions x speeds, and station keeping) and classify each decision:
  - JOINTLY_FEASIBLE: a P2-safe candidate gives the vehicle its full P1 share;
  - UNCERTAIN: only a non-closing P2-safe command exists, so P1 rests on the threat opening (`S1g`), which the vehicle
    cannot observe; also when a sector of a threat pattern is unhealthy;
  - JOINTLY_INFEASIBLE: no P2-safe command is non-closing, or none is P2-safe. Then the CR stays a hard constraint while
    it is enforceable and P1 is best effort inside it; without any P2-safe command the largest P2 margin among the
    non-closing commands is taken. The decision is logged with the property that could not be certified.
  Where the CR removes no candidate, the planner of the P1 proofs runs unchanged.
- **Prevention.** The queue lines keep every queue point out of the warning band of a held vehicle and never ahead of the
  hold (n = 1: -2.30 m, 2: -5.14 m, 3: -5.50 m, 4: -6.63 m; v11: -1.91, -4.18, -5.41, -6.63). A committed vehicle stops
  behind a vehicle ahead in its FRONT sector (the standoff, under the stage-2 timer).
- **Localization at the hold.** Without a validated gate localization the vehicle visits vertical offsets at the hold.
  In the synthetic B0 scan (real map fit, bar echoes only) the localization is acquired for an along-axis bias
  (estimate minus truth) from -1.0 to +0.3 m and at +0.5, +0.6 m, not at +0.4 m or from +0.7 m. There the vehicle holds,
  logs ENTRY_HOLD_TIMEOUT after 26 s and gives up: no entry, no deadlock, but no passage.

**Kinematic reconstruction** (`formal/check_f1p_kinematic.py`, reported; perception idealised, all vehicles running the
deployed decision rules, a moving third vehicle, navigation bias and braking dynamics): 312 episodes per controller.
- v11: 126 episodes with an entry into the CR without admission, 82 of them by avoidance (the F1p mechanism is physically
  realisable as single entries; no double occupancy was found).
- Current controller: no entry without admission and no double occupancy. Uncertain and jointly infeasible decisions
  occur, the latter almost only with three concurrent commitments, beyond premise C.
- Cost: concurrent commitments can stall in mutual separation warning (no passage within 90 s in 49 of 168 episodes,
  55 for v11), and an occupant plus a vehicle behind the held one slows the passages.

**Supported fleet sizes.** P2 and recovery: 2 to 4 vehicles at one gate. Queue layouts: abreast for 2 or 3 vehicles;
stacked 2 x 2 and the 3-vehicle stack pair; abreast queues of 4 or more are refused by the planner. P1 is proved pairwise;
several neighbours are covered by the triangle lemma and the two-threat escape, not by a fleet model. Formation keeping is
per vehicle; `formation_six` (6 vehicles) is simulated only.

**Assumptions** (provenance and limits in `config/parameter_sources.json`).
- **Environment.**
  - Currents inside the control-feasibility envelope (`control/current_envelope.py`).
  - Unrejected drift at most 0.15 m/s per axis, for P1 and for the P2 filter.
  - For P3, a residual disturbance of at most 0.05 m/s once the integrator has converged. This rests on window means of
    one simulated vehicle at survey speed. The proof fails above 0.064 m/s.
- **Vehicle.** Braking at no less than 0.5 m/s², a first-order hold loop (deviation at most drift / gain), velocity caps.
- **Sensors.** The sonar model of `perception/sonar_geometry.py`, hull tilt within the bound of
  `perception/distance_bound.py`, and the declared navigation error: 1 m in the gate frame, on every axis for the filter.
- **Communication.**
  - B0 needs none.
  - B2 adds claims and assumes four premises:
    - **C:** a claim is known before another vehicle can commit. The premise C audit checks it at runtime.
    - **V:** no live vehicle outside the exclusion zone is silent for 70 s.
    - **Fairness:** a 35 s fairness window of the fixes. It holds in every gate scenario of the v13 validation, at most
      26.3 s, and is exceeded in `formation_six`, which is outside the fleet model.
    - **Freshness:** packets older than 0.4 s on delivery are dropped.
- **Failures.**
  - Permanent: a failed vehicle stops moving and transmitting.
  - Never inside the CR, and only outside the exclusion zone of `holo_fleet/scope.py`.
  - At most 1 failure for 2 vehicles (none while committed), 1 for 3, and 2 for 4. A sunk or risen hull stops counting
    as queued (premise H).

**Open limitations** (expected counterexamples or known gaps, unresolved).
- **F1pe, F1pme, F0qe.** Entries into the CR by avoidance are excluded only inside the declared envelope. With the
  navigation error, the drift or an estimate jump outside it, the F1p counterexample returns (two vehicles in the CR after
  a concurrent commitment), with the motion limit as well, and a queued vehicle can be moved in.
- **S1gm.** Two vehicles that can only stop, facing each other, are not kept apart: a jointly infeasible configuration.
  The queue geometry and the standoff are meant to keep it unreachable; that is shown in simulation, not proved.
- **Localization.** The acquisition gap above (bias +0.4 m, from +0.7 m) and, in HoloOcean, the negative-bias stalls
  (-0.5 m, -1.0 m) cost the passage, not safety.
- **Stacked column on the axis.** Queue bound (c) ignores depth: a column stacked on the axis (only the `stack_axis`
  template of `gate_rear_approach`) puts a queued vehicle 2.17 m from the hold, inside its warning band.
- **Liveness.** Mutual warning stalls after concurrent commitments are outside the stage-2 timer.
- **F2s2, F2s3.** A failure while committed that leaves a single survivor. **F2s4.** A hull floating above the queue.
- The `scope` checks mark the other bounds: a vehicle stopped before the CR beyond the latch horizon, an occupant that
  stops inside the CR, recovery without fairness, and the strict invariance of the one-step P2 margin.
- Not covered by any proof: failures inside the exclusion zone, the continuous geometry inside the CR, the hold loop of the
  real vehicle (first-order abstraction), and field conditions. Every result is on HoloOcean models.

**Evidence by controller version.**
- **v11 (commit `d5f424e`): archive `v13-final-v11-2026-11-18`**, 34 runs.
  - **Configurations:** 7 scenarios in B0 (LOW noise), 8 in B2 MEDIUM, and gate_single at beacon latencies of 0.35 s and
    0.45 s; seeds 8000 and 8001.
  - **P1 and P2:** both held in every run, with a minimum separation of 2.84 m and at most one vehicle in the CR. All 64
    CR entries were admitted by the stage-2 check, none by avoidance.
  - **Classes:** 31 PASS. The B2 MEDIUM runs are 16 of 16 PASS. In B0, `lost_drone_mutex` failed P3 on both seeds (a false
    vacancy) and `integrated_short` s8000 was P3_LATE.
  - **Premise C:** all 64 pairs SATISFIED, before and after the audit correction (separate archive
    `v17-premise-c-v13-reaudit-2026-11-19`): 57 blocked by the latch, 7 hand-offs after an exit witness; claim knowledge 36
    certified, 8 undeliverable by the channel timing, 20 not applicable (B0).
  - No concurrent commitment occurred, so these runs do not exercise F1p. The 448-run campaign is a separate dataset and is
    not aggregated with them. These runs say nothing about the current controller.
- **Current controller: `results/gate_safety_v1/`** (targeted HoloOcean runs at the commit recorded in its manifest).
  - **Runs:** 22 targeted runs, B2 on the CLEAN channel (seed 8000) and B0 for the +-1.0 m bias cases: the gate set
    (gate_single, st_gate_pair, gate_four, line_parallel_mutex, mutex_deadlock_resolution, st_lost_committed, a stage-2
    hold with an along-axis navigation bias of -1.0, -0.5, +0.5 and +1.0 m, a vehicle held by an occupant with a third
    vehicle approaching it from behind) and the P1 set (p1_head_on, p1_close_encounter, p1_vertical_escape, p1_two_lines,
    formation_recovery_head_current, lost_drone_rejoin). Code at `8e7ebb5`, st_gate_pair at `9b42cec` (the final queue
    line); raw logs in the local archive `v18-gate-safety-targeted-2026-11-20`.
  - **P1 and P2:** held in every run (minimum separation between fleet vehicles 2.68 m, at most one vehicle in the CR, no
    CR entry without admission). The true clearance of a held vehicle to the CR was 1.49-1.78 m, against 0.43 m in v13
    and 0.76-0.81 m in v11 reference runs of the same scenarios.
  - **Classes:** the 5 gate scenarios and the 6 P1 scenarios PASS; the rear approach PASS (the held vehicle stays
    UNCERTAIN, holding, and is not moved into the CR); bias +0.5 and +1.0 (B2) PASS, the localization acquired at the
    hold. Not completed, P1 and P2 kept: bias -0.5 (B2, held 47 s, never admitted), -1.0 (B2 and B0, stalled before the
    hold on a bar echo the map no longer explains, as with v11), +1.0 in B0 (the acquisition limit), and st_lost_committed
    (a committed vehicle that fails 3 m before the frame, outside the scope of `holo_fleet/scope.py`).
  - **Physical reproduction with v11:** in an earlier version of the rear-approach scenario the v11 held vehicle escaped
    forward between the occupant and the rear vehicle and came within 1.0 m of the occupant. A silent occupant next to
    the bars is masked from the stage-2 check in both versions (the masked part of `M4h`, covered by B2 claims).
  - A single seed per configuration and one noise level: observations, not a statistical bound.

The archives are on local storage only. Each has a `MANIFEST.json` with the sha256 of every file, and v13 also holds the
code tarballs, a git bundle and every `run_config.json`. They have no public location, so the results can be reproduced
only by re-running the campaign from this repository.

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
holo_fleet/   automaton, controllers, perception, simulation, referee, HoloFleetViewer; runner.py runs one scenario
formal/       Z3 suites; formal/results/ (SUMMARY.csv, SUMMARY.json, one JSON per suite)
scripts/      demos, campaign runner, analysis, archive builder, engine slots and isolation probe
calibration/  simulator probes behind the measured parameters (outputs in results/calibration/, read by the code)
config/       parameter_sources.json (parameter provenance registry), scope_drones.json
tests/        unit, integration and isolation tests
patches/      HoloOcean patch and its build script
figures/      automaton diagrams and calibration figures
```
