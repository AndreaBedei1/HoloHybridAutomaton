# HoloFleet-HA

Decentralised coordination of BlueROV2 fleets with local hybrid automata in HoloOcean, aided by a one-way acoustic
beacon.

Every vehicle runs its own copy of the same local hybrid automaton on its own sonars and onboard navigation and on a
mission plan shared before the dive. There is no leader and no request, reply or command between vehicles. In
architecture B2 each vehicle also broadcasts a 64-bit packet on a TDMA schedule (position, error bound, gate phase,
CRC-16) and every receiver measures range and bearing itself. The beacon never enters separation, collision avoidance
or the gate precedence.

The fleet keeps a formation, keeps its separation, avoids collisions, crosses a narrow gate one vehicle at a time,
re-forms beyond it and copes with lost vehicles:

- **P1**, separation: every pair of live vehicles stays at least 1 m apart;
- **P2**, mutual exclusion: at most one vehicle in the critical region of a gate;
- **P3**, formation: slot recovery, fleet formation and degraded formation within their deadlines.

![automaton](figures/automaton/automaton_presentation.png)

## Quick start

```bash
conda activate holo_fleet_ha
python scripts/run_demo.py --list
python scripts/run_demo.py --scenario gate_single
python -m pytest -q -m "not slow and not holoocean"
python formal/check_properties.py
```

## Documentation

| Topic | Document |
|---|---|
| Installation (Python environment, patched HoloOcean engine, arena, several engines) | [`docs/INSTALL.md`](docs/INSTALL.md) |
| Usage (demos, scenarios, tests, formal checks, campaigns, analysis, archive) | [`docs/USAGE.md`](docs/USAGE.md) |
| Architecture (automaton, gate passage and gate guard, beacon, package layout) | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| Configuration (`FleetConfig`, profiles, channels, protocols, environment variables) | [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) |
| Parameter provenance | [`docs/PARAMETER_PROVENANCE.md`](docs/PARAMETER_PROVENANCE.md) |
| Guarantees, premises, monitors and limits | [`docs/GUARANTEES.md`](docs/GUARANTEES.md) |
| Formation contract P3 | [`docs/P3_CONTRACT.md`](docs/P3_CONTRACT.md) |
| HoloOcean sonar octree patch | [`docs/HOLOOCEAN_OCTREE_PATCH.md`](docs/HOLOOCEAN_OCTREE_PATCH.md) |
| Formal results | [`formal/results/SUMMARY.md`](formal/results/SUMMARY.md) |

Reports, campaign logs, reviews and raw runs are kept in a versioned archive outside the repository; the history of
the repository keeps every earlier version of them.

## Repository layout

```
holo_fleet/    the package: automaton, controllers, perception, simulation, referee, user interface
formal/        Z3 suites and their summary
scripts/       demos, campaign runner, analysis, archive builder, engine slots
calibration/   simulator probes behind the measured parameters
tests/         unit, integration and isolation tests
results/       calibration/ (probe outputs read by the code) and reference/ (CLEAN reference runs)
patches/       the HoloOcean patch and its build script
figures/       automaton diagrams and calibration figures
```
