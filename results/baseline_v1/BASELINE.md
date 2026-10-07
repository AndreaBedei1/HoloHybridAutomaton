# Baseline before the v2 changes (branch `realistic-sonar-formations-v2`, from `main` @ ad092aa)

Run on 2026-10-07 with the unmodified v1 code (conda env `holo_fleet_ha`, HoloOcean 2.3.0).

| check | command | result | time |
|---|---|---|---|
| tests | `python -m pytest -q` | **70 passed, 1 skipped, 2 failed** (see below) | 277 s |
| formal | `python formal/check_properties.py` | **102 / 102 checks as expected** (determinism 66/66, P1 9/9, P2 17/17, P3 10/10) | 305 s |

The two failures are not regressions of the v1 code: `tests/test_run_artifacts.py` parametrises over every
folder in `results/` that has a `run_config.json`, and the folder `results/pair_crossing_s0_20261007_211903/`
(an interrupted manual run of 2026-10-07 21:19, no `referee_metrics.json`) is picked up:

* `test_required_logs[pair_crossing_s0_20261007_211903]` - `referee_metrics.json` missing
* `test_referee_verdicts_in_nominal_runs[pair_crossing_s0_20261007_211903]` - same file missing

All 70 other tests pass, including every test on the seven v1 demonstrative runs.  The folder is left
untouched (it is the user's run); v2 makes the artifact tests consider completed runs only.

Files: `pytest_baseline.txt`, `formal_baseline.log`, `formal_SUMMARY.md` (copy of `formal/results/SUMMARY.md`).
