# DISCOVERY - workspace inspection (Phase 0)

Inspection done on 2026-10-06 before writing any project code. Nothing was deleted or modified outside
`F:\Andrea\HoloHybridAutomaton` (this project folder, a fresh git repository).

## 1. What is on the machine

| Location | Content | Relevance |
|---|---|---|
| `~/Desktop/HoloDroneCompetition` | **Marine Race Arena** (git repo, `marine_race_arena/` package, paper in `article/`, 78-run validated benchmark) | **the existing marine arena: re-used** |
| `~/Desktop/HoloOcean-2.3.0` | HoloOcean 2.3.0 source (client + docs + examples) | API reference, sensor semantics |
| `~/Desktop/HoloCoordinantion/HoloCoordination` | earlier coordination demos (scenario builder, acoustic request/response) | not used: coordination there is message-based |
| `~/Desktop/HoloObstacleAvoidance`, `HoloMotion`, `HoloRace`, `DroneMarino*` | other HoloOcean experiments | not needed |
| `F:\Andrea\HoloHybridAutomaton\ppt\` | the user's presentation (`presentazione_flotta_automi_ibridi.pptx`) | left untouched (git-ignored) |

### HoloOcean / Python environment

* HoloOcean **2.3.0** client installed from source in the conda env **`ocean`** (Python 3.9.25, numpy 2.0.2,
  scipy, matplotlib 3.9.4, opencv 4.12, pandas, pytest); installed world package: `Ocean`
  (worlds `OpenWater`, `PierHarbor`, ...). The arena uses `OpenWater-Hovering`.
* `z3-solver` was **missing** in `ocean`. Because `ocean` backs the frozen 78-run results of the arena paper,
  it was **not modified**: a dedicated env **`holo_fleet_ha`** was cloned from it and `z3-solver` + `imageio`
  were added there. This project is installed in it in editable mode (`pip install -e .`).

### The Marine Race Arena (HoloDroneCompetition)

* Three official tracks (`marine_race_arena/tracks/*.json`): Horseshoe Bay (12 gates), Vertical Serpent (17),
  Mixed Endurance (22); gate aperture 1.5 x 1.5 m, bars 0.18 m, depth 0.22 m.
* Gates are created at runtime with `env.spawn_prop("box", ...)` by `adapters/visual_spawner.py`
  (`HoloOceanVisualSpawner`, "hybrid_micro" mode: dense micro-blocks for top/bottom bars + uniform pillars).
* Config loader (`config/loader.py`), gate geometry (`arena/gate.py`, `arena/gate_factory.py`), referee for
  race scoring (`referee/`), runners (`run.py`, `scripts/run_marine_race.py`, `run_benchmark.py`, ...),
  controllers (`rule_gate_baseline`, `rule_gate_center_then_commit`, `leader_follower`, RL policies).
* Conventions worth keeping: BlueROV2, control scheme 0 (8 thrusters), normalised (surge, sway, heave, yaw)
  commands mapped by `BaseRaceAdapter.command_to_bluerov2_thrusters`; privileged sensors (Pose, Velocity, ...)
  stripped before any controller sees the data; currents applied with `env.set_ocean_currents`.

## 2. What is re-used and how

| Re-used from the arena | How |
|---|---|
| Horseshoe Bay track file (all 12 gates spawned, G06/G07 are the critical regions of Scenario B; scenarios A and the pair test run in its open interior) | `holo_fleet/arena_bridge.py` imports the arena package from its own folder (no copy); override with `MARINE_RACE_ARENA_ROOT` |
| `load_track_config`, `GateFactory` (gate frames, bar boxes) | static map given to the drones (clutter rejection, gate frame) and to the referee |
| `HoloOceanVisualSpawner` | spawns the arena gates in our multi-agent HoloOcean scene |
| BlueROV2 thruster mapping | `arena_bridge.thruster_command` reproduces `command_to_bluerov2_thrusters` exactly (re-implemented as a 10-line pure function because the original is a method of the race adapter) |
| privileged-sensor stripping policy | same policy, implemented in `holo_fleet/sim/holo_env.py` with a whitelist |

**Not re-used (and why):** the race runner/adapter and its referee are built around a *race* (ordered
gate crossings, staggered single-file release, acoustic beacons, scoring); this project needs a
multi-agent loop with per-agent currents, sensor stress injection and a referee for separation,
critical-region occupancy and formation error. `leader_follower` is not used on purpose: it relies on
acoustic messages, while this project must be leaderless and communication-free.

**Scene constraint found:** in the Horseshoe Bay layout gate **G05 lies ~9 m before G06 on G06's axis**
(bars at lateral -2.5...-0.8 m in G06's frame). A first gate run with a 13 m approach corridor drove a
drone into G05. The approach zone was shortened to 4 m before the queue line and a structure safety filter
was added to every mode (see REPORT.md).

## 3. Probe measurements (probe/*.py, HoloOcean 2.3.0, BlueROV2)

| Probe | Finding | Used for |
|---|---|---|
| `probe_sensors.py` | `RangeFinderSensor` ray casts **hit other vehicles and runtime-spawned gate props**; rays do not hit the own hull | 3D proximity sonar (25 rings x 72 beams) |
| `probe_sensors.py --sonar` | `ImagingSonar` sees **other vehicles** (e.g. 2.61-2.70 m for a drone 3 m ahead) but **not** runtime-spawned props (gates are invisible to it) | forward-looking sonar = vehicles only; gates via proximity sonar + map |
| `probe_calib.py` | ring beam k points to body azimuth -k*360/N (clockwise); `LaserAngle` positive = **down**; magnetometer heading = atan2(m_y, m_x) | sensor geometry in `perception/sensor_suite.py` |
| `probe_calib.py` | 15 deg ring spacing misses a drone 1 m below at 3 m | 5 deg elevation spacing, -60..+60 deg |
| `probe_motion.py` | DVL body velocity x fwd / y left / z up; IMU gyro z has the opposite sign of yaw rate; surge 0.3 -> 0.77 m/s, 1.0 -> 2.15 m/s; heave 0.3 -> 1.04 m/s; yaw weakly damped | low-level loop design |
| `probe_current.py` | `set_ocean_currents(v)` is a force: steady drift of an unactuated vehicle 0.012 / 0.117 / 0.391 / 1.191 m/s for v = 0.3 / 1 / 2 / 4 | currents specified as **effective drift**, inverted through this table |
| `scripts/calibrate_plant.py` | with the deployed DVL PI loop: stop from 0.46 m/s in 0.9 s (>= 0.5 m/s^2), reversal to -0.3 m/s in 0.5 s; station keeping residual 0.001 / 0.001 / 0.06 / 0.28 m/s at 0.25 / 0.40 / 0.60 / 0.80 m/s drift | formal plant assumptions (a_brake, current envelope 0.6 m/s) |
| `probe_cameras.py` | sensor `location` offsets work (metres); **camera pitch is positive downward**; looking straight down into deep water renders black | visualisation cameras (chase + side views) |
| run `dev_form_02` | the hull returns localise the neighbour's geometric centre, ~0.12 m behind HoloOcean's agent reference (IMU socket): per-axis error <= 0.28 m | perception error bound eps_rel = 0.30 m |
