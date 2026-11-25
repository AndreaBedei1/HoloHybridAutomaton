# Safety and liveness validation

## Reference and operating domain

The reference tree is `6121f7032ad5d74c3a4827f202a1ad44302d7b08`. The architecture is one local hybrid automaton, six single-beam sonars, DVL, IMU, compass, pressure depth, a surveyed mission plan and passive B2 acoustic reception. B0 uses the same controller with the acoustic path disabled. Offline referee data do not enter controller decisions. The sensor and dynamic bounds are conditional on the assumed capture ages, error bounds, operating currents and actuator response. HoloOcean noise profiles, including MEDIUM, can exceed deterministic premises. See [the README](../README.md) and [parameter registry](../config/parameter_sources.json).

- **P1** requires every distinct pair of formation or independent fleet vehicles to stay at centre distance strictly greater than 1 m. The interval proof requires a sound lower bound on the *actual closest* pair and true ground-speed bounds for both vehicles. Warning and avoidance commands remain best effort when those premises fail.
- **P2** requires at most one admitted vehicle in the closed gate critical region, longitudinally `[-0.8, 0.0] m`. Physical clearance through `+0.55 m` is a separate decision. A BUSY latch cannot clear on time alone.
- **P3** requires the healthy fleet to return to the stated formation tolerance after the disturbance. Eventual recovery requires a long enough physically feasible tracking regime and usable information. Regular messages alone cannot constrain the damage from repeated safety interruptions.

## Failure analysis of the reference traces

All 49 MEDIUM certificate-loss events match a control tick where the reported DVL velocity error bound jumps from 0.098 to 0.700 m/s. The immediately preceding command had a certificate in every case. There were no position-correction or structure-guard events on those ticks and the offline referee found the gate region empty. Seed 0 contributed 48 events, including 46 on drone 0. On 29 of those warning ticks every escape-grid candidate failed P2. The resulting commands were emergency choices, with their lost certification recorded. There is no evidence of an unsafe admission in these traces.

| Reference time since formation loss | MEDIUM seed 0 | MEDIUM seed 1 |
|---|---:|---:|
| First gate exit | 348.6 s | 33.5 s |
| First to last gate exit | 63.8 s | 52.5 s |
| Last exit to onset of sustained formation tolerance | 44.4 s | 12.9 s |
| Total | 456.8 s | 98.9 s |

Before the final exit, drone 0 spent 150.4 s in separation warning in seed 0 versus 1.4 s in seed 1. In seed 0 the four cancelled commitments lasted about 133.3 s total. Localization was invalid for 130.3 s of those windows. In the same run, the longest failed reapproach took 63.5 s, including 51.1 s in warning. Waiting on acoustic reports was the sole admission blocker for at most 1.8 s per vehicle in the audited pre-admission states. After the final exit, drone 1 was in formation recovery for 42.1 s versus 12.7 s. These durations overlap between vehicles.

In the archived 110 s simultaneous-commitment probe, drone 1 timed out at about 31.7 s, and drone 0 cleared commitment at 93.5 s following separation interventions. The latter recommitted at 102.8 s without having crossed. A new run of exactly that reference code and seed injected both commitments at 0.899991 s. Two vehicles entered sequentially, peak occupancy was one, minimum distance was 2.720 m, and recovery was still open at 110 s. This shows variability even at fixed seed and demonstrates why neither trace establishes liveness. Raw replicate evidence in archive `v24-final-validation-2026-11-25/baseline/line3_simultaneous_commit` has referee SHA-256 `a9408c775cffeea7f3f29967b4979afc0e3b0838388e5fcaa06cf820c4d1087f`.

## Experimental protocol

The [matrix](../config/publication_validation.json) fixes scenarios, conditions, seeds and observation windows. It pairs B0 and B2 on every seed. The comparison matches non-communication configurations and revisions before calculating differences. Modes and certificate status use drone-control samples. Separation and occupancy use referee samples. Formation recovery is measured per episode, and a missed crossing or unfinished recovery at the end of a fixed window remains censored. The simultaneous-commitment arm injects the same state in both protocols. The fault injection is an experiment and does not run in an ordinary mission.

Run the two-engine isolation check before starting two workers. Use the same patched roots and shared registry for both commands. A single worker can be used when two isolated instances are unavailable.

```powershell
$env:HOLO_FLEET_SLOT_REGISTRY = '<shared project engine-slot registry>'
$env:HOLO_FLEET_HOLOOCEAN_ROOT = '<patched engine root prefix>'
$env:MARINE_RACE_ARENA_ROOT = '<arena checkout>'
python scripts/verify_engine_isolation.py --out <outside-repo-isolation-dir>
python scripts/publication_validation.py --run --workers 2 --isolation-record <isolation.json> --raw <outside-repo-archive> --out <summary.json>
python scripts/publication_validation.py --raw <outside-repo-archive> --out <summary.json>
```

The matrix preserves every scheduled row, including infrastructure failures, censored runs and absent results. Per-run hashes identify the raw logs and replay measurements. Paired descriptions report the count, median and range of available differences. They are descriptive evidence. Three seeds cannot support a significance claim.
