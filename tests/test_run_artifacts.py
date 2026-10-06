"""Every demonstrative run in results/ has the required logs, sane referee verdicts and non-empty figures."""

import csv
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RUNS = sorted(p for p in (ROOT / "results").glob("*") if p.is_dir() and (p / "run_config.json").exists())
REQUIRED = ["run_config.json", "events.jsonl", "referee_metrics.json", "referee_timeseries.csv", "summary.csv"]
PER_DRONE = ["state", "observations", "actions"]
NOMINAL = ("pair_crossing", "formation_current", "gate_arena")


def _runs():
    if not RUNS:
        pytest.skip("no demonstrative runs in results/ yet")
    return RUNS


@pytest.mark.parametrize("run", RUNS or [None], ids=lambda p: p.name if p else "none")
def test_required_logs(run):
    if run is None:
        pytest.skip("no runs")
    for f in REQUIRED:
        assert (run / f).exists(), f
    cfg = json.loads((run / "run_config.json").read_text())
    for k in range(cfg["n_drones"]):
        for kind in PER_DRONE:
            p = run / f"drone_{k}_{kind}.jsonl"
            if not p.exists() and kind in ("observations", "actions"):
                pytest.skip(f"{p.name} not on disk (bulky logs are not versioned)")
            assert p.exists(), p.name
            with open(p) as fh:
                json.loads(fh.readline())
    for line in open(run / "events.jsonl"):
        json.loads(line)
    rows = list(csv.DictReader(open(run / "referee_timeseries.csv")))
    assert len(rows) > 50


@pytest.mark.parametrize("run", RUNS or [None], ids=lambda p: p.name if p else "none")
def test_referee_verdicts_in_nominal_runs(run):
    if run is None:
        pytest.skip("no runs")
    cfg = json.loads((run / "run_config.json").read_text())
    m = json.loads((run / "referee_metrics.json").read_text())
    assert m["run"]["comms_enabled"] is False or cfg["comms_enabled"] is True
    assert sum(m["run"]["determinism_monitor_violations"].values()) == 0
    if cfg["scenario"] in NOMINAL:
        assert m["P1_separation"]["holds"], m["P1_separation"]["first_violation"]
        if m["P2_mutual_exclusion"]["gates"]:
            assert m["P2_mutual_exclusion"]["holds"]
            assert all(m["P2_mutual_exclusion"]["all_drones_traversed"].values())


@pytest.mark.parametrize("run", RUNS or [None], ids=lambda p: p.name if p else "none")
def test_figures_not_empty(run):
    if run is None:
        pytest.skip("no runs")
    figs = sorted((run / "figures").glob("*.png"))
    if not figs:
        pytest.skip("figures not generated (python scripts/analyze_results.py --run ...)")
    import matplotlib.image as mpimg

    for f in figs:
        img = mpimg.imread(f)
        assert img.shape[0] > 200 and img.shape[1] > 400, f.name
        assert float(np.std(img[..., :3])) > 0.02, f"{f.name} looks blank"
