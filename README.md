# HoloFleet-HA: a leaderless, communication-free UUV fleet built from verifiable local hybrid automata

Three simulated underwater drones (BlueROV2, HoloOcean 2.3.0) survey the seabed in formation and pass
through the narrow gates of an existing **marine arena** one at a time. They have **no leader**, they
**exchange no messages**, and **every drone runs the same local hybrid automaton** fed only by its own
onboard sensors (3D proximity sonar, forward-looking imaging sonar, DVL, compass/IMU, depth, camera).
Fleet-level properties emerge from the composition `H_fleet = H_1 || H_2 || H_3`, where the drones are
coupled only through the physical environment they observe:

| | property | formula | how it is established |
|---|---|---|---|
| **P1** | inter-vehicle separation (safety) | `G( forall i != j : d_ij >= d_safe )` | Z3: pairwise k-induction (unbounded) + local 3D-escape and triangle lemmas; HoloOcean referee |
| **P2** | critical-region mutual exclusion (safety) | `G( sum_i inside_CR_i <= 1 )` | Z3: lemmas on the shared priority rule (no double commit, no deadlock, priority persistence, occupancy visibility) + inductive invariant; HoloOcean referee |
| **P3** | bounded formation recovery (liveness) | `G( formation_lost -> F_[0,T] formation_recovered )` | Z3: ranking-function proof with a worst-case bound T; HoloOcean referee |

The controller never reads simulator ground truth. Ground truth is used only by the **referee**,
offline and online, to judge each run. Tests enforce this by scanning imports and checking at runtime
which sensor keys reach a controller.

> Start with [REPORT.md](REPORT.md) for results, verified properties and limits, and
> [DISCOVERY.md](DISCOVERY.md) for what was found on this machine and what is re-used.

---

## 1. Architecture

```mermaid
flowchart LR
  subgraph SIM["holo_fleet.sim (owns ground truth)"]
    HO[HoloOcean 2.3.0<br/>OpenWater + Horseshoe Bay gates<br/>(marine_race_arena)] -->|onboard sensors only| F[SensorFrame per drone]
    HO -->|PoseSensor, VelocitySensor, CollisionSensor| R[Referee P1/P2/P3]
    C[current field<br/>effective drift] --> HO
  end
  subgraph DRONE["holo_fleet.perception + ha + control (identical on every drone)"]
    F --> P[Perception<br/>DVL+compass dead reckoning<br/>sonar point clouds -> tracks<br/>gate frame, formation estimate]
    P -->|abstract observation| A[Local hybrid automaton<br/>8 modes, shared guard spec]
    A -->|mode| FL[Mode flows<br/>formation / gate / QP safety filter / 3D escape / failsafe]
    P --> FL
    FL --> LL[DVL velocity PI loop + heading loop]
  end
  LL -->|surge, sway, heave, yaw -> 8 thrusters| HO
  SPEC[holo_fleet/ha/spec.py<br/>holo_fleet/ha/gate_rule.py] --- A
  SPEC --- Z3[formal/ Z3 checks]
```

Package layout:

```text
holo_fleet/
  config.py              every threshold, gain and envelope assumption (single source of truth for runtime AND proofs)
  mission.py             static mission plan uploaded before the dive: survey path, formation slots, gate map
  arena_bridge.py        re-use of ~/Desktop/HoloDroneCompetition/marine_race_arena (tracks, gate factory, spawner)
  ha/spec.py             hybrid-automaton edge table; guards written once for Python AND Z3 (logic backend)
  ha/gate_rule.py        communication-free priority rule for the critical region (shared with Z3)
  ha/automaton.py        runtime automaton + determinism monitor
  perception/            sensor suite, dead reckoning, proximity/forward sonar processing, tracker, observations
  control/               per-mode flows (formation, gate queue/pass, QP safety filter, escape, failsafe), low level
  sim/                   HoloOcean multi-agent wrapper, current fields, scenarios
  referee/               ground-truth validator (the only reader of simulator state)
  comms/                 optional intermittent acoustic channel - OFF by default
  analysis.py            offline figures and perception-vs-truth statistics
formal/                  Z3 encodings + check_properties.py (results in formal/results/)
scripts/                 run_experiment, run_all, analyze_results, render_report, calibrate_plant, validate_assumptions
tests/                   isolation, no-comms default, runtime/Z3 conformance, rule properties, geometry, artifacts
probe/                   Phase-0 HoloOcean probes (sensor conventions, plant and current calibration, cameras)
results/                 demonstrative runs (metrics, logs, figures, GIFs) + SUMMARY / ASSUMPTIONS
figures/                 copies of the key figures
```

## 2. Drones and onboard sensors

BlueROV2, control scheme 0 (8 thrusters), commands mapped with the arena's own BlueROV2 mapping.

| sensor (log name) | HoloOcean type | socket / mounting | FOV | range [m] | Hz | noise | role |
|---|---|---|---|---|---|---|---|
| IMUSensor | IMUSensor | IMUSocket / hull centre | - | - | 30 | accel 0.01, gyro 0.005 (std) | yaw-rate damping, attitude monitoring |
| Compass | MagnetometerSensor | IMUSocket / hull centre | - | - | 30 | 0.005 std per axis | heading (AHRS role) |
| DVLSensor | DVLSensor | DVLSocket / hull bottom, 4 beams 22.5 deg | - | 50 | 10 | 0.01 m/s per beam | body velocity over ground: dead reckoning, velocity loop |
| DepthSensor | DepthSensor | DepthSocket / pressure port | - | - | 30 | 0.02 m | depth keeping, vertical escape, relative depth |
| ProxSonar_e-60 ... e+60 | RangeFinderSensor x25 | IMUSocket / hull centre | 360 deg az x [-60, 60] deg el, 5 x 5 deg | 10 | 10 | 0.04 m range std + per-beam dropout | 3D proximity sonar: other drones and gate bars (safety layer) |
| AltimeterDown / AltimeterUp | RangeFinderSensor | IMUSocket | single beam +/-90 deg | 30 | 10 | - | vertical escape availability |
| FrontSonar | ImagingSonar | SonarSocket / bow | 90 deg az x 20 deg el | 0.5-12 | 5 | Rayleigh 0.02, mult 0.05 | forward-looking sonar: vehicles ahead |
| FrontCamera | RGBCamera | CameraSocket / bow | 90 deg, 320x240 | - | 5 | - | imagery for checks and figures (not used by the safety logic) |

Why a ray-cast proximity sonar: the HoloOcean imaging sonar works on a static octree that does **not**
contain runtime-spawned props (the arena gates), while `RangeFinderSensor` rays hit both vehicles and
gate props (probe results in DISCOVERY.md). The ring array is an abstraction of an omnidirectional
obstacle-avoidance sonar. Range noise and dropout are added by the simulator-side sensor model.
Referee-only sensors (PoseSensor, VelocitySensor, CollisionSensor) and two visualisation cameras
(ChaseCamera, SideCamera) are stripped before any controller sees the data.

## 3. The local hybrid automaton

Modes: `FORMATION_FOLLOW`, `SEPARATION_WARNING`, `COLLISION_AVOIDANCE`, `GATE_APPROACH`, `GATE_YIELD`,
`GATE_PASS`, `FORMATION_RECOVERY`, `FAILSAFE_HOLD_OR_RETREAT`, plus a latched `committed` bit.
Priority (checked in Z3): **failsafe > collision avoidance > separation warning > critical region >
formation recovery > mission following**. Guards use abstract observations computed by perception:
`d_min` (staleness-inflated nearest distance), `occ_busy`, `has_prio`, `at_queue`, `passed`, `gate_zone`,
`form_err`, `t_ok`, `neighbors_ok`, `sense_ok`, `env_ok`.

| mode | flow (desired velocity) |
|---|---|
| FORMATION_FOLLOW / RECOVERY | slot tracking on the survey line: along-track **consensus** on perceived relative positions (`v = v_nom + k * mean residual`), lane keeping + relative lateral consensus, depth keeping |
| SEPARATION_WARNING (d < 2.3 m) | **QP safety filter**: smallest change of the mission velocity that never closes on any threat and opens at >= 0.2 m/s; closing intent becomes a right-hand sidestep (antisymmetric, so drones meeting head-on pass port to port) |
| COLLISION_AVOIDANCE (d < 1.6 m) | escape at 0.5 m/s along the bisector away from the threats, **plus a vertical component** when all threats are above (or all below): the 3D escape |
| GATE_APPROACH / GATE_YIELD | go to / hold the queue point (side slots at +/-2.6 m lateral, centre slot 2.3 m back), retreat when the priority rule says so |
| GATE_PASS | leave the queue line straight, merge onto the gate axis, traverse the critical region |
| FAILSAFE | blind (sonar data older than 0.2 s): hold position and move to a pre-assigned depth layer; out of envelope: hold and return to the depth band |
| every mode | **structure safety filter**: never close on gate bars nearer than 0.7 m (sonar returns matched to the arena map) |

### Leaderless gate protocol without messages

Each drone evaluates every neighbour it perceives in the gate's approach corridor using **its own
measurement** of the relative gate-frame position (along `s`, lateral `l`, vertical `z`), with the shared
rule in `holo_fleet/ha/gate_rule.py`. The result is one of six exclusive classes:

* `COMMIT`: closer to the gate (by more than 1.5 m), or along-tie and more to the left (by more than
  2.3 m), or both ties and higher (by more than 0.6 m). Each level is decisive only beyond a margin
  larger than twice the measurement error, and the next level is consulted only inside a strictly
  narrower tie band.
* `WAIT_ROBUST`: the other drone wins even under worst-case errors.
* `WAIT_FRONT`, `BACKOFF_REAR`, `BACKOFF_RIGHT`, `BACKOFF_RANK`: undecided cases are resolved by the
  robustly-identified rear or right drone stepping back. As a last resort a pre-configured static rank
  is used, identical on every drone and known before the mission, so it is not communication.

A drone commits only from the queue line, only if it beats every perceived queued drone with fresh data,
and only if nobody is perceived in the occupied zone (from just past the queue line to 1 m past the gate's
critical region). Z3 proves that two drones never both commit and never both wait, and that a commit
keeps its priority until the committing drone is visible in the occupied zone.

## 4. Installation

The project runs in a clone of the arena's `ocean` env (left untouched) with `z3-solver` added:

```bash
conda create -n holo_fleet_ha --clone ocean -y
conda activate holo_fleet_ha
pip install z3-solver imageio
pip install -e .            # from this folder
```

The marine arena is imported from `~/Desktop/HoloDroneCompetition` (override with
`MARINE_RACE_ARENA_ROOT`). HoloOcean 2.3.0 with the `Ocean` package must be installed (see DISCOVERY.md).

## 5. Commands

```bash
conda activate holo_fleet_ha
python formal/check_properties.py                       # all Z3 checks -> formal/results/SUMMARY.md
python formal/check_properties.py --quick               # skip the slowest suite (P1, ~90 s)
python scripts/run_experiment.py --scenario pair_crossing
python scripts/run_experiment.py --scenario formation_current --current medium --n-drones 3
python scripts/run_experiment.py --scenario formation_current --current medium --no-jet
python scripts/run_experiment.py --scenario gate_arena --n-drones 3
python scripts/run_experiment.py --scenario stress
python scripts/run_experiment.py --scenario gate_arena_comms        # optional comparison (comms ON)
python scripts/analyze_results.py --run <run_id>         # verdicts, perception stats, figures
python scripts/render_report.py --run <run_id>           # + GIFs, sensor sheet, RUN_REPORT.md
python scripts/validate_assumptions.py                   # empirical check of every formal assumption
python scripts/run_all.py                                # everything (about 1.5 h wall time)
python -m pytest -q                                      # tests (add -m "not slow" to skip the P1 Z3 suite)
python scripts/calibrate_plant.py                        # plant calibration used by the formal models
```

Add `--viewport` to `run_experiment.py` to watch the HoloOcean window.

## 6. Reading a run folder (`results/<run_id>/`)

| file | content |
|---|---|
| `run_config.json` | scenario, seeds, currents, stress settings, slots, full `FleetConfig` |
| `events.jsonl` | automaton decisions (`PASS`, `YIELD`, `RETRY_PASS`, `EXITED`, `FORMATION_LOST/RECOVERED`), `ENVELOPE_VIOLATION`, releases |
| `drone_i_state.jsonl` | mode, latched commit, onboard nav estimate, enabled edges, determinism monitor |
| `drone_i_observations.jsonl` | abstract observation + full local observation (neighbours, gate frame, formation) |
| `drone_i_actions.jsonl` | desired velocity, authority, normalised command |
| `referee_timeseries.csv` | ground truth: positions, pairwise distances, CR occupancy, formation error, drift |
| `referee_metrics.json` | P1/P2/P3 verdicts, episodes, envelope flags, determinism, message counters |
| `summary.csv` | one-line summary |
| `figures/` | trajectories, distances, occupancy, formation error, modes, depth, perception error, sensors, scene |
| `chasecamera.gif`, `sidecamera.gif`, `RUN_REPORT.md` | visual checks and per-run report |

Bulky per-step logs (`*_observations.jsonl`, `*_actions.jsonl`) and raw camera frames stay on disk and
are not versioned (see `.gitignore`); `scripts/run_all.py` regenerates them.

## 7. Results

See [REPORT.md](REPORT.md) (metrics, figures, formal results, assumption validation) and
[results/SUMMARY.md](results/SUMMARY.md).

## 8. What is proved, what is validated, what is not claimed

* **Proved in Z3, on abstractions whose guards are the runtime code itself:** local determinism, completeness and
  the priority hierarchy; P1 for a pair under explicit assumptions (perception error and staleness,
  speed caps, braking capability from calibration, unrejected differential drift), plus the local lemmas
  that make the pairwise argument hold inside a three-drone fleet; P2 via the shared priority rule and an
  inductive invariant; P3 as a ranking-function bound on the formation law. Every suite includes
  mutation tests that must produce counterexamples.
* **Validated empirically in HoloOcean**: the closed loop (real sensing, real thruster dynamics, currents,
  gate props) through the ground-truth referee, and each formal assumption via
  `scripts/validate_assumptions.py`.
* **Not claimed**: a proof over HoloOcean's continuous physics; guarantees when an assumption is violated
  (the referee flags out-of-envelope currents, and the drones themselves declare `ENVELOPE_VIOLATION`);
  more than 3 drones; static-obstacle avoidance beyond mapped gate structures.
