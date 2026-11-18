# Usage

All commands run from the repository root in the `holo_fleet_ha` environment.

## Demos

```bash
python scripts/run_demo.py --list
python scripts/run_demo.py --scenario gate_single
python scripts/run_demo.py --scenario lost_drone_rejoin --select 2          # start with drone 2 selected
python scripts/run_demo.py --scenario gate_single --profile MEDIUM --b2 MEDIUM --seed 1
```

A demo opens HoloOcean and the Fleet View and runs until the mission is complete (terminal SUCCESS plus 8 s) or its
deadline. The Fleet View shows the fleet from above with the planned slots, P1 / P2 / P3 status, events, and the
automaton panel of the selected drone (click a drone or press `0`-`9`).

| Option | Effect |
|---|---|
| `--headless` | no HoloOcean viewport |
| `--no-window` | no Fleet View |
| `--duration S` | stop after S simulated seconds |
| `--profile CLEAN\|LOW\|MEDIUM\|HIGH` | sensor degradation |
| `--b2 CLEAN\|LOW\|MEDIUM\|BLACKOUT` | B2 beacon on that channel level (default: none) |
| `--seed N` | seed of the wrapper noise |
| `--save [--overwrite] [--media] [--out DIR]` | keep a complete run in `results/reference/<scenario>/` |

Without `--save` a demo keeps no files.

## Scenarios

| Family | Scenarios |
|---|---|
| Separation (P1) | `p1_head_on`, `p1_vertical_escape`, `p1_two_lines`, `p1_close_encounter` |
| Formation (P3) | `formation_triangle`, `formation_square`, `formation_six`, `formation_recovery_head_current`, `formation_gust`, `em_delay_head` |
| Gate (P2) | `gate_single`, `gate_four` (four drones, stacked square), `integrated_short`, `mutex_deadlock_resolution`, `line_parallel_mutex` |
| Lost drone | `lost_drone_rejoin`, `lost_drone_rejoin_front`, `lost_drone_timeout`, `lost_drone_mutex` and its rise variants, `lost_drone_front_rise_low` |
| Adversarial | `st_rear_pusher`, `st_rear_pusher_close`: a scripted vehicle pushes the waiting drone toward the gate |
| Stress batch | `STRESS_BATCH` in `holo_fleet/sim/scenarios.py`: currents, time-varying water, queues, failures, boundary probes |
| Perception bench | `sonar_classification` (only through `run_demo.py`) |

Suggested first runs: `formation_six`, `p1_vertical_escape`, `mutex_deadlock_resolution`, `lost_drone_rejoin`,
`lost_drone_timeout`, `integrated_short`.

## Tests

```bash
python -m pytest -q -m "not slow and not holoocean"   # fast, no engine
python -m pytest -q                                   # everything, including the Z3 suites and HoloOcean tests
python scripts/run_tests.py <out dir>                 # one process per file at low priority, then the formal suite
```

## Formal verification

```bash
python formal/check_properties.py           # every suite; writes formal/results/SUMMARY.{md,json}
python formal/check_properties.py --quick   # skips the slowest suite
python formal/check_gate_guard.py           # one suite
```

Exit status 0 when every check returns its expected verdict (UNSAT for properties, SAT for mutations and witnesses).

## Campaigns

`scripts/run_sensor_robustness_campaign.py` runs each simulation in its own process, at below-normal priority, with a
watchdog, and skips runs already complete in the output directory.

```bash
python scripts/run_sensor_robustness_campaign.py --phase targeted --only gate_single lost_drone_mutex \
    --protocols B2_MEDIUM B0 --profiles MEDIUM-ALL --seeds 7300 7301 --raw results/my_batch
```

Shard a batch over several engines with `--shard i --nshards n` and `HOLO_FLEET_ENGINE_SLOTS=n` (see
[`INSTALL.md`](INSTALL.md), section 4); `--dry-run` lists the runs. Protocols, profiles and channels are listed in
[`CONFIGURATION.md`](CONFIGURATION.md).

Each run directory holds `run_config.json` (commit, engine build, configuration, seeds), per-drone state logs,
`events.jsonl`, `referee_timeseries.csv`, `referee_metrics.json` and `run_metrics.json` (class, terminal outcome,
premise audit, beacon statistics).

## Analysis

```bash
python scripts/analyze_validation.py <out dir> <raw dir>          # tables and figures of a batch
python scripts/audit_premise_c.py <out.json> <raw dir> [...]      # premise C and every CR entry, from the logs
python scripts/analyze_final_campaign.py <raw dir>                 # the v8 acceptance criteria of a final matrix
```

## Archive of the evidence

```bash
python scripts/build_archive.py --dest <archive root> --version <name> --refs <git refs> \
    --runs <raw dirs> --docs <reports> --evidence <tables, audits>
```

Writes a git bundle of the whole history, a source tarball per ref, the copied files and `MANIFEST.json` (size and
sha256 of every file). An existing version is never overwritten.
