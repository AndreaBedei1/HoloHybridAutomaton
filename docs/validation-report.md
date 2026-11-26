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

## Fixes found by the paired campaign

The first complete run of the matrix (archive `v25-final-paired-campaign-2026-11-25`, kept as a record) stalled. In `line_parallel_mutex`, B2 CLEAN, two vehicles held each other in separation warning. Each saw the other in two sectors. The widened association of `c2c2293` admitted every sub-pattern of those sectors, and no motion opened all of them, so both chose HOLD and neither moved. A bisection with the same seed and scenario on `6121f70`, `c2c2293` and the fix (archive `v26-liveness-bisect-2026-11-25`) confirmed the cause. `Flows._robust_choice` (`d3c4df4`) keeps the robust choice whenever one exists. Only when the robust choice is infeasible, or is a HOLD with no opening, does it take a motion that opens the measured association. That decision keeps its P1-uncertified joint class and sets `association_fallback`. `tests/test_association_ambiguity.py` reproduces the stalled observation.

Two infrastructure faults were fixed. A slot-record replace failed with a Windows sharing violation while another worker read the file. It now retries for at most 5 s and still fails closed (`426338a`). The bounded S1g queries returned UNKNOWN at the end of the complete catalogue, depending on catalogue order. They now run in a fresh Z3 context (`9ed01f6`). The complete catalogue then gave the expected verdict on all 409 checks (`c903c0d`).

## Results of the paired campaign

All 54 planned rows are present, with no `INVALID` or `MISSING` rows (`results/final_paired_validation.json`, SHA-256 of the archived copy `fd97d84283d679b38eddd8e2c411ae86f4e155d17f89fe6915cb35689c29c695`). Four isolated engines ran at once after a fresh isolation proof, on non-default ports, each with its own client and root.

In pass 1, two lanes refused 14 runs because the working tree was briefly modified while the formal results were being committed. `--require-clean` refused them before any engine started. Those 14 runs were then executed from the clean commit `c903c0d`, after a second isolation proof. The refusal record is kept (`lane_failures_pass1_dirty_tree.json`). The runs were recorded at `d3c4df4`, `9ed01f6` and `c903c0d`. These revisions differ only in `formal/`, so the comparison pairs revisions with identical runtime code (`same_runtime` in `scripts/publication_validation.py`). All 27 pairs qualify.

Safety, every run, both protocols (referee ground truth):

- P1: zero violation samples and zero physical contacts. The smallest centre distance is 2.572 m (B0, `sonar-blackout` s1).
- P2: peak occupancy one, zero double-occupancy samples and zero premature admissions.

| Case (fixed window) | B0 class | B2 class | B2 − B0 min. distance, median (range) m | B2 − B0 crossings, median (range) |
|---|---|---|---|---|
| encounter | 3 CENSORED | 3 CENSORED | +0.161 (+0.153, +0.196) | no gate |
| sonar-blackout | 3 CENSORED | 3 CENSORED | +0.089 (−0.009, +0.422) | no gate |
| formation | 3 CENSORED | 3 CENSORED | +0.019 (−0.035, +0.197) | no gate |
| gate | 3 CENSORED | 3 PASS | −0.133 (−0.233, −0.052) | +1 (+1, +2) |
| gate-acoustic-medium | 3 CENSORED | 1 PASS, 2 CENSORED | −0.054 (−0.314, +0.017) | +1 (0, +1) |
| simultaneous-clean | 3 CENSORED | 2 PASS, 1 CENSORED | +0.227 (+0.010, +0.253) | +1 (0, +1) |
| simultaneous-medium | 2 CENSORED, 1 PASS | 3 PASS | +0.266 (−0.010, +0.400) | +1 (0, +2) |
| dvl-interruption | 3 CENSORED | 3 PASS | −0.161 (−0.185, +0.040) | +1 (+1, +2) |
| integrated | 3 CENSORED | 3 PASS | −0.169 (−0.509, +0.082) | +1 (+1, +2) |

`CENSORED` means the fixed observation window ended before the criteria v9 deadline with a crossing or a recovery still open. It is not a safety failure. The encounter, sonar-blackout and formation windows are shorter than any gate passage by design. In the formation case, both protocols recovered within 8.4 s of the disturbance in every seed. In the seven gate cases, B2 completed all three crossings and recovered formation in 15 of 21 runs. B0 did so in 1 of 21, usually crossing once or twice before the window ended, because its latch waits for a positive sonar exit witness (M4g). The three B2 runs that did not finish are:

- `gate-acoustic-medium` s0: three crossings by 142.8 s, recovery still open at 185 s.
- `gate-acoustic-medium` s1: two crossings.
- `simultaneous-clean` s0: two crossings.

`P2_UNCERTIFIED` events occurred in both protocols under MEDIUM sensing and DVL interruption. The largest count was 11, in B2 `integrated` s2. None coincided with an occupancy or separation violation. Three seeds per case are descriptive evidence, not a statistical bound.

## Independent review findings

Independent reviews of the P1 and P2 changes found no critical defect. They recorded these residual premises and test gaps:

- **P1, sonar blackout.** A carried bound can last 6.4 s with every sonar missing. It is sound only if every vehicle stays under 0.70 m/s, and nothing on board checks the peer's speed.
- **P1, thrust check.** The thrust check in the certificate can never fire, because the bridge already clips the command. The link from the final setpoint to the ground speed within the interval is assumed.
- **P1, echo confirmation.** Confirmation needs two captures within `tau_max` = 0.25 s. A slower capture rate would leave dynamic echoes unconfirmed for mode switching and avoidance. The certificate itself uses every echo.
- **P1, tilt.** The vehicle tilt premise of the sector geometry is not monitored.
- **P1, propagation model.** The S5a model restates the propagation premise and does not test the partial bound.
- **P2, conceding vehicle.** For up to 2.5 s after conceding, a vehicle reporting GP_NONE from about 2.95 m before the hold may be excluded by its peers as an occupant without the timing premise that exclusion rests on. Safety then rests on that vehicle's own P2 filter.
- **P2, concession coverage.** No test exercises the `past_centre`, `passage.in_progress` or `mutex_zone` concession guards. The B2C-S2 model treats concession feasibility as a constant.

These are listed in README section 6.
