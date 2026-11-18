# Architecture

HoloFleet-HA runs a fleet of BlueROV2 vehicles in HoloOcean. Every vehicle executes its own copy of one local hybrid
automaton. There is no leader and no request, reply or command between vehicles.

## Data flow of one vehicle

```
sensors        six single-beam sonars, DVL, IMU, compass, depth; B2: one-way acoustic beacon receiver
   |
perception     holo_fleet/perception/   echo classification, distance bound, neighbour targets, gate belief,
   |                                    navigation and its error bound, formation check, acoustic fixes
abstract observation                    the inputs of the guards (holo_fleet/ha/observation_invariants.py)
   |
hybrid automaton  holo_fleet/ha/        one mode per vehicle; edges, guards and priorities in spec.py
   |
mode controller   holo_fleet/control/   the continuous flow of the active mode, escape planner, gate guard
   |
low level         control/lowlevel.py   velocity and heading loops, thruster allocation
```

The fleet is the parallel composition `H_1 || ... || H_N` of identical automata, coupled only through what each vehicle
senses in the water and, in architecture B2, through the one-way beacon.

## Hybrid automaton

Ten modes (`holo_fleet/ha/spec.py`):

| Mode | Behaviour |
|---|---|
| `FORMATION_FOLLOW` | tracks its own slot of the shared formation plan |
| `FORMATION_WAIT_REJOIN` | keeps its slot at reduced progress while an expected neighbour is missing |
| `DEGRADED_FORMATION` | keeps its original slot after a missing neighbour is declared vacant |
| `FORMATION_RECOVERY` | returns to its own slot, from behind along its own lane |
| `MUTEX_APPROACH` | moves to its queue point in front of the gate |
| `MUTEX_YIELD` | waits at the queue point while the gate is occupied or another vehicle has priority |
| `MUTEX_PASS` | crosses the gate alone |
| `SEPARATION_WARNING` | brakes and opens the distance to a neighbour that is too close |
| `COLLISION_AVOIDANCE` | escapes in 3-D from an imminent collision |
| `FAILSAFE_HOLD_OR_RETREAT` | holds position on a sensor fault or when it cannot deliver its commands |

Priority, encoded in the guards (a lower edge fires only when every higher guard is false):

```
FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX > FORMATION_RECOVERY > formation keeping
```

Diagrams generated from the specification: `figures/automaton/` (`python scripts/make_automaton_diagram.py`).

## Gate passage

A vehicle reaches its queue point and waits there (`MUTEX_YIELD`). It commits (`MUTEX_PASS`) only when it has priority
by the sector rule (`ha/mutex_rule.py`) and its occupancy belief of the critical region (CR) is free. Before entering the
CR it stops at the stage-2 hold point and checks once more that nothing occupies the CR (sonar, and under B2 no vehicle
heard inside and no claim of another vehicle).

The **gate guard** (`control/flows.py gate_guard`, `control/escape.py`) bounds the avoidance layer at the gate. A
vehicle inside the keep-out box in front of the CR and not admitted may receive a separation or escape velocity only
while `v . a <= GUARD_GAIN (s_hold - s)` along the gate axis `a`. When no allowed velocity opens the threat:

- separation warning: `GUARD_HOLD` (station keeping where the hold began, never forward) or `GUARD_RETREAT` (back along
  the axis, when that opens every threat);
- collision avoidance: `OVERRIDE`, the unconstrained escape, logged as `GATE_GUARD_OVERRIDE`.

`GateRule.avoidance_guard = False` restores the behaviour of `b2-freeze-v11` (protocols with `NG_` in the campaign
runner).

## Architecture B2: one-way acoustic beacon

Every vehicle broadcasts a 64-bit packet on a TDMA schedule (period 1 s, slot k): its onboard position estimate, error
bound, gate index and gate phase, CRC-16. Receivers measure range and bearing themselves (`perception/acoustic.py`).
A packet older than `AcousticConfig.latency_max_s` (0.4 s) on delivery is dropped before its payload is decoded.

Beacon data never reach separation, collision avoidance, the gate precedence or the sonar exit witness (static test
`tests/test_b2_isolation.py`). They feed the gate occupancy latch and the stage-2 check (`perception/acoustic_gate.py`),
the vacancy of a lost vehicle and the association of neighbours. B0 is the same controller without a beacon.

## Package layout

```
holo_fleet/
  config.py      single source of every threshold, gain and envelope (also imported by formal/)
  mission.py, formations.py, scope.py
  ha/            automaton specification, runtime, guard texts, precedence rule, observation invariants
  control/       mode flows, escape planner, gate guard, formation time, low level
  perception/    sonar geometry and processing, distance bound, targets, navigation, gate perception, acoustic
  sim/           HoloOcean environment, scenarios, sensor and channel noise, beacon, engine lease and watchdog
  referee/       evaluation from ground truth only: P1, P2, P3, criteria, premise audit, metrics
  ui/            Fleet View and fleet camera
  runner.py      one run: scenario, controllers, simulator, referee, logs
formal/          Z3 suites; formal/results/SUMMARY.md
scripts/         demos, campaign orchestrator, analysis, archive builder, engine isolation
calibration/     simulator probes behind the measured parameters (outputs in results/calibration/)
tests/           unit, integration and isolation tests
```

Controllers never receive ground truth; the referee and the Fleet View read it for evaluation and display only
(`tests/test_isolation.py`, `tests/test_b2_isolation.py`).
