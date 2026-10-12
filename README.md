# HoloFleet-HA

Communication-free coordination of BlueROV2 fleets with local hybrid automata in HoloOcean.

HoloFleet-HA simulates fleets of BlueROV2 drones in HoloOcean. Every drone runs its own copy of the same
local hybrid automaton. There is no leader and no inter-drone communication: each drone uses only its own
sonars and onboard navigation, plus a mission plan shared before the dive (path, formation clock, formation
template, gate map). With this, the fleet keeps a formation, keeps its separation, avoids collisions, crosses
a narrow gate one drone at a time, recovers its formation after a disturbance and copes with lost drones.

## 1. How it works

```
Sensors        six single-beam sonars, DVL, IMU, compass, depth
   |
Perception     echo classification, neighbour targets, gate belief, formation check
   |
Abstract observation
   |
Local hybrid automaton   one mode per drone (holo_fleet/ha/)
   |
Mode controller          the continuous flow of that mode (holo_fleet/control/)
   |
BlueROV2 thrusters
```

The fleet is the parallel composition `H_fleet = H_1 || ... || H_N` of identical automata, coupled only
through what each drone senses in the water.

- Each drone executes its own instance of the same automaton.
- No message is exchanged at runtime.
- The referee and the Fleet View may read simulator ground truth, for evaluation and visualization only.
- The controllers never receive ground truth (checked by tests).

## 2. The hybrid automaton

| what | where |
|---|---|
| Automaton specification (modes, edges, guards) | [`holo_fleet/ha/spec.py`](holo_fleet/ha/spec.py) |
| Runtime execution | [`holo_fleet/ha/automaton.py`](holo_fleet/ha/automaton.py) |
| Continuous mode controllers | [`holo_fleet/control/`](holo_fleet/control/) |
| Diagrams (generated from the spec) | [`automaton_presentation.svg`](figures/v2/automaton/automaton_presentation.svg), [`automaton_full.svg`](figures/v2/automaton/automaton_full.svg) |

![automaton](figures/v2/automaton/automaton_presentation.png)

| state | what the drone does |
|---|---|
| `FORMATION_FOLLOW` | Tracks its own slot of the shared formation plan. |
| `FORMATION_WAIT_REJOIN` | Keeps its slot at reduced progress while an expected neighbour is missing. |
| `DEGRADED_FORMATION` | Keeps its original slot after a missing neighbour has been declared vacant (no reconfiguration). |
| `FORMATION_RECOVERY` | Returns to its own slot, from behind along its own lane when it is off it. |
| `MUTEX_APPROACH` | Moves to its queue point in front of the gate. |
| `MUTEX_YIELD` | Waits at the queue point while the gate is occupied or another drone has priority. |
| `MUTEX_PASS` | Crosses the gate alone. |
| `SEPARATION_WARNING` | Brakes and opens the distance to a neighbour that is too close. |
| `COLLISION_AVOIDANCE` | Escapes in 3-D from an imminent collision. |
| `FAILSAFE_HOLD_OR_RETREAT` | Holds position on a sensor fault or when it cannot deliver its commands. |

The priority order

```
FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX > FORMATION_RECOVERY > formation keeping
```

is encoded in the guards of the automaton itself, not in a separate scheduler: a lower-priority edge can
fire only when the guards of every higher one are false. The real guards are in `spec.py`.

## 3. Live demo

```bash
conda activate holo_fleet_ha
python scripts/run_demo.py --list
python scripts/run_demo.py --scenario lost_drone_rejoin
```

By default a demo is **live only**: it opens HoloOcean and the Fleet View, prints a summary at the end,
does not modify the official results and keeps no new files.

The Fleet View shows:
- the whole fleet from above, with the planned slots, the states of the drones and a state timeline;
- P1 / P2 / P3 status and readable events;
- an automaton panel for one drone: current state, last transition, its guard and its reason.

Click a drone on the map, or its row in the side panel, to select it (keys `0`-`9` work too).
`--select K` chooses the initial drone:

```bash
python scripts/run_demo.py --scenario lost_drone_rejoin --select 2
```

Other options: `--headless` (no HoloOcean window), `--no-window` (no Fleet View), `--duration S`.

## 4. Available scenarios

**P1 - Separation / collision avoidance**

| scenario | purpose |
|---|---|
| `p1_head_on` | Two drones meet head-on and give way by the sector traffic rule. |
| `p1_vertical_escape` | Traffic rule off: drones escape in 3-D from a non-cooperative vehicle. |
| `p1_two_lines` | Two survey lines meet head-on; flanked drones give way vertically. |
| `p1_close_encounter` | Right-angle crossing with simultaneous arrival; the separation warning keeps the drones apart. |

**Formation**

| scenario | purpose |
|---|---|
| `formation_triangle` | Triangle survey under a lateral current. |
| `formation_square` | 2 x 2 square survey under a diagonal current. |
| `formation_six` | Six-drone survey line under a current with a vertical component. |
| `formation_recovery_head_current` | A head current beyond what one drone can deliver; the formation is lost and recovered. |
| `formation_gust` | Stress test: a strong gust breaks the formation, which then recovers. |

**P2 - Mutual exclusion at the gate**

| scenario | purpose |
|---|---|
| `gate_single` | Three drones cross arena gate G06 one at a time and re-form beyond it. |
| `integrated_short` | Survey, cross-current at the gate, one-at-a-time passage and re-formation. |
| `mutex_deadlock_resolution` | A stacked pair and a third drone queue at the gate: left first, then top first. |
| `line_parallel_mutex` | A line abreast reaches the gate side by side. |

**Lost drone**

| scenario | purpose |
|---|---|
| `lost_drone_rejoin` | Temporary thruster failure: the others slow down, the drone rejoins from behind. |
| `lost_drone_timeout` | Permanent failure: after a timeout the others continue in a degraded formation. |
| `lost_drone_mutex` | A drone is lost before the gate; the leftmost drone still present goes first. |

**Perception bench**

| scenario | purpose |
|---|---|
| `sonar_classification` | Sonar echo classes: gate structure, drone, both in one cone, seabed, clutter. |

## 5. Recommended demos

If you only want to see the main behaviours:

```bash
python scripts/run_demo.py --scenario formation_six               # formation keeping with six drones
python scripts/run_demo.py --scenario p1_vertical_escape          # warning, avoidance and 3-D escape
python scripts/run_demo.py --scenario mutex_deadlock_resolution   # queue order and one-at-a-time passage
python scripts/run_demo.py --scenario lost_drone_rejoin           # WAIT_REJOIN, FAILSAFE, rejoin from behind
python scripts/run_demo.py --scenario lost_drone_timeout          # timeout, then DEGRADED_FORMATION
python scripts/run_demo.py --scenario integrated_short            # survey, gate and re-formation together
```

## 6. Saving results

| command | effect |
|---|---|
| `python scripts/run_demo.py --scenario X` | Temporary demo, nothing is kept. |
| `python scripts/run_demo.py --scenario X --save` | Keeps the run in `results/v2/demos/X/`, only if no result exists there yet. |
| `python scripts/run_demo.py --scenario X --save --overwrite` | Intentionally regenerates the existing result. |
| `python scripts/run_demo.py --scenario X --save --overwrite --media` | Also keeps the frames and the GIF. |

Only a complete run is saved. `--out DIR` (with `--save`) saves under another folder.

## 7. Formal verification

The project includes Z3 checks of:
- automaton determinism and completeness, and the priority order;
- observation consistency (perception to automaton interface);
- P1 inter-vehicle separation;
- P2 critical-region mutual exclusion and queue progress, under stated assumptions;
- P3 formation recovery and the degraded behaviour with a drone that never returns.

```bash
python formal/check_properties.py
```

Current status: 186 / 186 checks give the expected verdict. The proofs are on abstractions of the drones
and their sensing, not on HoloOcean's physics. Details: [`formal/results/SUMMARY.md`](formal/results/SUMMARY.md).

## 8. Tests

```bash
python -m pytest -q -m "not slow and not holoocean"   # fast tests
python -m pytest -q                                   # all tests, including the Z3 suites and HoloOcean
```

On the current branch: 264 fast tests and 267 tests in total pass.

## 9. Project structure

```
holo_fleet/
  ha/           hybrid automaton: spec, runtime, guards, mutex precedence
  control/      continuous mode controllers, escape planner, low level
  perception/   sonar geometry, echo classification, onboard perception
  sim/          HoloOcean integration and scenarios
  referee/      evaluation only (ground truth)
  ui/           Fleet View and HoloOcean fleet camera
scripts/        demos, figures, automaton diagram, benches
formal/         Z3 verification suites and their results
tests/          unit, integration and isolation tests
figures/v2/     figures and automaton diagrams
results/v2/     canonical demo results and measured assumptions
```

## 10. Installation

The project runs on HoloOcean 2.3 with a sonar octree patch, in the conda environment `holo_fleet_ha`.

```bash
conda create -n holo_fleet_ha --clone ocean -y
conda activate holo_fleet_ha
pip install z3-solver imageio
pip install -e .
pip install -e F:\Andrea\holoocean-octree-patch\client --no-deps                            # patched client
powershell -ExecutionPolicy Bypass -File patches/build_and_install_patched_holoocean.ps1   # patched engine
```

The patched engine lives in its own root (`F:\Andrea\holoocean_patched_root`, override with
`HOLO_FLEET_HOLOOCEAN_ROOT`); the official HoloOcean installation is not modified. The gate scenarios
import the Marine Race Arena from `~/Desktop/HoloDroneCompetition` (override with `MARINE_RACE_ARENA_ROOT`).
See [`docs/HOLOOCEAN_OCTREE_PATCH.md`](docs/HOLOOCEAN_OCTREE_PATCH.md) for the patch.

## 11. Results and documentation

| topic | document |
|---|---|
| Detailed results | [`REPORT.md`](REPORT.md) |
| Design history and fixes | [`DESIGN_ITERATIONS.md`](DESIGN_ITERATIONS.md) |
| Formal verification | [`formal/results/SUMMARY.md`](formal/results/SUMMARY.md) |
| Current demo results | [`results/v2/demos/SUMMARY.md`](results/v2/demos/SUMMARY.md) |
| HoloOcean sonar patch | [`docs/HOLOOCEAN_OCTREE_PATCH.md`](docs/HOLOOCEAN_OCTREE_PATCH.md) |
| Sonar validation | [`docs/v2/SONAR_PROBE.md`](docs/v2/SONAR_PROBE.md) |
