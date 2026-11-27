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

All 54 planned rows are present, with no `INVALID` or `MISSING` rows (`results/final_paired_validation.json`, SHA-256 of the archived copy `1bf37c3306cf7ad3a2bcd06f4a8d50bb1c5843213685eb4644d1eba36f50f30a`). Four isolated engines ran at once after a fresh isolation proof, on non-default ports, each with its own client and root.

In pass 1, two lanes refused 14 runs because the working tree was briefly modified while the formal results were being committed. `--require-clean` refused them before any engine started. Those 14 runs were then executed from the clean commit `c903c0d`, after a second isolation proof. The refusal record is kept (`lane_failures_pass1_dirty_tree.json`). The runs were recorded at `d3c4df4`, `9ed01f6` and `c903c0d`. These revisions differ only in `formal/`, so the comparison pairs revisions with identical runtime code (`same_runtime` in `scripts/publication_validation.py`). All 27 pairs qualify.

Safety, every run, both protocols (referee ground truth):

- P1: zero violation samples and zero physical contacts. The smallest centre distance is 2.572 m (B0, `sonar-blackout` s1).
- P2, in the 36 runs of the six gate cases (the other three cases have no gate): peak occupancy one, zero double-occupancy samples and zero premature admissions.

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

`CENSORED` means the fixed observation window ended before the criteria v9 deadline with a crossing or a recovery still open. It is not a safety failure, and it is not evidence that the run would have finished. The encounter, sonar-blackout and formation windows are shorter than any gate passage by design. In the formation case, both protocols recovered within 8.4 s of the disturbance in every seed. In the six gate cases, B2 completed all three crossings and recovered formation in 15 of 18 runs. B0 did so in 1 of 18, usually crossing once or twice before the window ended, because its latch waits for a positive sonar exit witness (M4g). The three B2 runs that did not finish are:

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

## Controller consolidation (2026-11-26/27, branch `controller-consolidation`)

### Regression and timed recovery

At `e3e31f0`, a BUSY latch was released after `t_occ_max` = 25 s without a corridor echo. Commit `6bcda80` removed that release. B0 at the outer queue point often gets no sonar exit witness (M4g), so from `6121f70` its latch stayed BUSY for the whole run. Stage A (archive `v28-regression-audit`) measured this: B0 crossed in 3/3 runs at `e3e31f0` and in 0/3 at both `6121f70` and `ebf7a29`. Commit `9998b68` adds the STALE state. A STALE latch allows committing but is never FREE, and the admission after it is timed. Stage B (`v29-stage-b-stale`) measured 3/3 crossings for both protocols.

### The near-field restart and its replacement

To remove premise S, commit `5f9f049` restarted the timed bound at every hull within 3.0 m in any sector. In the targeted HoloOcean runs of `a6e4987` (archive `v31-targeted-a6e4987`), B2 passed every case. No B0 lead drone was admitted in any of the 8 gate runs. The drone queued 3.2 m behind the stage-2 hold has an onboard bound of 2.4-3.3 m, so it renewed the restart at nearly every capture. The onboard bound is 0.40-0.54 m below the true distance at the median and up to 1.1 m below at the 95th percentile. Two injected concurrent commitments kept each other at the hold in the same way.

Commit `f583bde` replaced the restart and the review fixes `0c76653` and `a57a9b2` followed. The resulting rules:

- The timed bound runs from max(last corridor sighting, start of the dwell in the admission zone) + `t_admit_stale`.
- Every admission needs the near-field clearance: no hull with a bound below `admit_clear_m` = 3.3 m in any sector.
- The admission zone `GateRule.in_admission_zone` is explicit: along -0.4 / +0.3 m of the hold, lateral 0.35 m, vertical 0.9 m.
- Under premise Z two drones whose estimates are in the zone are at most 3.25 m apart (ST6). So with the P1 detection premise, no drone is admitted while another dwells in the zone (ST1, proved).
- Premise Z is declared and monitored by the referee as `zone_nav`: an estimate in the zone is within 0.65 m horizontally and 0.25 m vertically of the truth.
- With the committed navigation premise alone, 1.0 m per axis, two drones in the zone can be 4.68 m apart. This is ST6l, a declared limit.
- A drone that is not committed backs off along the axis while a hull ahead in its FRONT sector is nearer than `queue_standoff_m` = 3.8 m (ST7, a progress margin). The queue points stay abreast for M2 and the exit-witness geometry.
- The hold give-up runs from max(hold start + `t_admit_stale`, commitment + wait, corridor bound) (ST3).
- The standoff of a committed drone ends beyond the clearance.

### Independent review of `f583bde`

The adversarial review raised four major findings, all addressed:

- **Zone-distance errors.** The bound used the fit tolerances and a 0.05 m depth error, which are not declared error bounds. Premise Z now states the bounds and the referee monitors them; ST6l records the limit.
- **Give-up outside the zone.** A timed drone held just outside the zone never gave up. The `v32` injected run reproduced this: one drone was out of the zone at l = 2.16 m and the other was in it without clearance. Fixed by ST3 and its test.
- **Standoff below the clearance.** The committed standoff limit was below the clearance. Fixed.
- **Seabed blind ranges.** A seabed blind range under the DOWN sonar withholds the clearance. None was observed at G06; the case is recorded as a limit.

The staggered middle column of `f583bde` cut the M2 bearing margin from 30 to about 11 deg. Moving the whole queue line back broke the exit-witness geometry. Both were dropped for the queue standoff. Filtered structure-window echoes are part of the P1 detection premise, and a latched admission pushed back by avoidance is outside Pi2. Both are listed in README section 6.

### Targeted evidence

| Archive, commit | Case | B0 | B2 |
|---|---|---|---|
| `v31`, `a6e4987` | gate CLEAN, LOW, MEDIUM (2 seeds each), integrated MEDIUM, injected CLEAN | 0/8 crossed (INCOMPLETE) | 8/8 PASS |
| `v32`, `f583bde` | gate CLEAN, LOW, MEDIUM (2 seeds each) | 6/6 PASS, min true separation 3.34-3.50 m | MEDIUM 1/1 PASS |
| `v35`, `0c76653` | gate MEDIUM s0 | PASS: three crossings in 172 s, P3 met at 217 s | CLEAN s0 PASS |
| `v35`, `0c76653` | injected concurrent commitments, CLEAN s0 | PASS (minimum true separation 2.61 m) | - |
| `v32`, `f583bde` | injected concurrent commitments, CLEAN s0 | INCOMPLETE (review finding 2: endless hold out of the zone) | - |
| `v32`, `f583bde` | integrated MEDIUM s0 | INCOMPLETE (localization about 200 s, then the bar echo below) | - |

In `v35`, the nearest bound at the hold was at least 4.04 m at the 1st percentile, so the clearance held at every capture. `zone_nav` reached 0.21 m horizontally and 0.19 m vertically, the measurement that raised the vertical part of premise Z to 0.25 m in `a57a9b2`. The P2 latch released once by a sonar witness and once by STALE, with occupancy at most 1.

**Bar echo at the CR entry (open, both protocols).** An admitted vehicle about 1 m before the CR entry under MEDIUM sensing can
see the bars of the frame in its FRONT sonar at 0.7 to 0.9 m outside the predicted structure window. The bar echo is then
classified UNKNOWN or DYNAMIC, so the vehicle enters collision avoidance, retreats, and approaches again. Episodes: B2 integrated
MEDIUM 135 s (`v31`, the run passed), B0 gate MEDIUM 13 s (`v32`), B0 integrated MEDIUM 467 s (`v32`, INCOMPLETE). This
predates these fixes and is a progress cost; P1 and P2 hold. It is left unresolved: masking these echoes would rest P1 on the
P2 argument near the frame.

### Formal catalogue

On `f583bde` the complete runner had 438 checks, all with the expected outcome. The final commit adds ST3g, ST3o, ST6l and the reported ST7 to `formal/check_stale.py`. That suite passes in full on `a57a9b2`.

### Final paired campaign

The final paired campaign runs from the frozen worktree at `a57a9b2`, archive `v33-final-paired-campaign-2026-11-27`. It runs the 9 cases x 3 seeds x 2 protocols of `config/publication_validation.json`, each to the mission terminal with no fixed window. At most 3 isolated engines run at once.
