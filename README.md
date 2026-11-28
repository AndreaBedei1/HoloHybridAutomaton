# HoloHybridAutomaton

Decentralised control of a leaderless BlueROV2 fleet in the HoloOcean simulator. Every vehicle runs the same local hybrid
automaton. There is no leader and no request, reply or command between vehicles. The fleet keeps a formation, passes a gate
one vehicle at a time and recovers the formation afterwards. Z3 checks the safety and recovery properties on abstractions and
on the deployed code. In simulation a referee judges every run from ground truth.

## What it is for

The repository shows that a fleet can satisfy three requirements with local decisions only:

- **P1:** the vehicles stay separated;
- **P2:** at most one vehicle is in the critical region of a gate;
- **P3:** the formation is recovered after a disturbance.

It contains the controller, the Z3 proofs, the simulation campaign and the audit that judges every run.

## How it works

Each vehicle uses six single-beam sonars, DVL, IMU, compass and a pressure depth sensor. It also receives two acoustic
signals:

- **Acoustic beacon (B2).** Each vehicle broadcasts its position and gate phase in its own TDMA slot. The broadcast is one-way,
  with no handshake.
- **Gate beacon.** A fixed pinger on each gate gives a position fix. It is mission infrastructure, not communication between
  vehicles.

The automaton has 10 modes in strict priority order:

1. failsafe;
2. collision avoidance and separation warning;
3. the gate modes (approach, yield, pass);
4. formation recovery and formation keeping.

A safety filter edits every velocity command near the gate so that no vehicle can enter the critical region without
admission. Silence is never taken as evidence: a vehicle that is not heard is treated as possibly present.

**B2** is the controller and the contribution. **B0** is the same controller with the acoustic beacon between vehicles disabled.
It is only a baseline.

![Hybrid automaton](figures/automaton/automaton_presentation.png)

## Install and run

Requirements:

- Windows 10/11 and Python 3.9 or later (conda);
- [HoloOcean 2.3.0](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/) with the Ocean world;
- Unreal Engine 5.3, to build the patched engine;
- the Marine Race Arena ([doi:10.5281/zenodo.22757860](https://doi.org/10.5281/zenodo.22757860)).

```bash
conda create -n holo_fleet_ha --clone <env with the HoloOcean 2.3.0 client> -y && conda activate holo_fleet_ha
pip install -e .
git clone https://github.com/AndreaBedei1/HoloDroneCompetition.git      # the Marine Race Arena
```

Unpatched HoloOcean 2.3.0 sonars miss gates spawned at run time. Apply
`patches/holoocean-2.3.0-sonar-octree-rebuild.patch` to a HoloOcean 2.3.0 source checkout. Copy
`patches/build_and_install_patched_holoocean.ps1` into that checkout (next to its `engine/` folder) and run it there with
`-UE <UE 5.3 folder> -Root <patched root>`. The campaign runner refuses an
unpatched engine. Set these environment variables:

| Variable | Meaning |
|---|---|
| `HOLO_FLEET_HOLOOCEAN_ROOT` | the patched engine root |
| `MARINE_RACE_ARENA_ROOT` | the Marine Race Arena checkout |
| `HOLO_FLEET_ENGINE_SLOTS` | number of concurrent engines, 1 to 4 |

```bash
python scripts/run_demo.py --list                                  # scenarios
python scripts/run_demo.py --scenario line_parallel_mutex          # B2, with the fleet dashboard
python scripts/run_demo.py --scenario line_parallel_mutex --b2 OFF # B0
python scripts/run_demo.py --scenario integrated_short --profile MEDIUM --seed 1 --b2 MEDIUM [--headless] [--select K]
python -m pytest -q -m "not slow and not holoocean"                # tests, no engine
python formal/check_properties.py [--quick]                        # Z3 catalogue
```

The dashboard (`holo_fleet/ui/`) shows the fleet from above with the P1/P2/P3 status and the automaton of vehicle K. You can
step through the run with the arrow keys, `PgUp`/`PgDn`, `Home`/`End` and `Space`.

## Results

| Evidence | Result |
|---|---|
| Paired campaign v46 (9 cases x 3 seeds x 2 protocols, every run to the mission terminal) | **B2: 27/27 accepted**; B0: 26/27 (one documented formation-association limit) |
| Safety in all 54 runs | 0 P1 violations, 0 contacts; in the 36 gate runs, peak occupancy 1, 0 premature admissions, 0 false releases |
| LOW, v47 | B0 and B2 PASS, accepted |
| HIGH, v47 (stress, outside the declared premises) | B0 PREMISE_VIOLATED, B2 INCOMPLETE; P1 and P2 held |
| Z3 catalogue (final tree) | 475 checks, all as expected: 202 proved, 43 bounded, 59 numeric; 84 mutations, 32 witnesses, 13 scope, 26 declared limits, 16 reported |

These results come from `results/final_paired_validation.json`, `results/final_run_audit.json`,
`results/low_high_run_audit.json` and `formal/results/SUMMARY.json`. Every guarantee depends on declared premises and on the
HoloOcean models.

See [docs/validation-report.md](docs/validation-report.md) for:

- the requirement definitions and the full property table;
- the scope of the Z3 guarantees;
- the premises and the remaining limitations;
- the reproduction steps.
