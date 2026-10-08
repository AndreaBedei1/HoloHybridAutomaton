"""Demonstration runs (results/v2/demos): only COMPLETED runs are judged; an interrupted run never fails
these tests but stays recognisable (run_status.json says INCOMPLETE and it is excluded)."""

import csv
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DEMOS = ROOT / "results" / "v2" / "demos"


def _status(run: Path) -> str:
    try:
        return json.loads((run / "run_status.json").read_text(encoding="utf-8")).get("status", "INCOMPLETE")
    except (OSError, ValueError):
        return "INCOMPLETE"


ALL = sorted(p for p in DEMOS.glob("*") if p.is_dir() and (p / "run_config.json").exists()) if DEMOS.exists() else []
COMPLETE = [p for p in ALL if _status(p) == "COMPLETE"]
REQUIRED = ["run_status.json", "run_config.json", "events.jsonl", "referee_metrics.json", "referee_timeseries.csv",
            "summary.csv", "perf.json", "onboard_summary.json"]


def complete_runs():
    return COMPLETE or [None]


def test_incomplete_runs_are_excluded_and_labelled():
    for run in ALL:
        if run not in COMPLETE:
            assert _status(run) == "INCOMPLETE"
            assert run not in COMPLETE


def test_selection_logic_on_a_synthetic_interrupted_run(tmp_path):
    run = tmp_path / "interrupted"
    run.mkdir()
    (run / "run_config.json").write_text("{}")
    (run / "run_status.json").write_text(json.dumps({"status": "INCOMPLETE", "reason": "interrupted by the user"}))
    assert _status(run) == "INCOMPLETE"
    (run / "run_status.json").unlink()
    assert _status(run) == "INCOMPLETE"                      # no status file: never treated as complete


@pytest.mark.parametrize("run", complete_runs(), ids=lambda p: p.name if p else "none")
def test_required_artifacts(run):
    if run is None:
        pytest.skip("no completed demonstration run yet (python scripts/run_all_demos.py)")
    for f in REQUIRED:
        assert (run / f).exists(), f
    for line in open(run / "events.jsonl", encoding="utf-8"):
        json.loads(line)
    rows = list(csv.DictReader(open(run / "referee_timeseries.csv", encoding="utf-8")))
    cfg = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    assert rows and float(rows[-1]["t"]) >= cfg["duration_s"] - 0.2


@pytest.mark.parametrize("run", complete_runs(), ids=lambda p: p.name if p else "none")
def test_referee_verdicts(run):
    if run is None:
        pytest.skip("no completed demonstration run yet")
    m = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    r = m["run"]
    assert r["inter_agent_messages"] == 0 and r["comms_enabled"] is False
    assert r["ground_truth_used_by_controllers"] is False
    assert all(v == 0 for v in r["determinism_violations"].values())
    p1 = m["P1_separation"]
    assert p1["holds"] and p1["physical_contacts"] == 0 and not p1["collision_sensor_edges"]
    if m["P2_mutual_exclusion"]["gates"]:
        p2 = m["P2_mutual_exclusion"]
        assert p2["holds"] and max(p2["max_occupancy"].values()) == 1
        cfg = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
        assert len(p2["entry_order"][p2["gates"][0]]) == cfg["n_drones"]
    p3 = m["P3_formation_recovery"]
    if p3["enabled"]:
        assert p3["all_recovered_within_run"], p3["episodes"]


def _states(run, k):
    return [json.loads(l) for l in open(run / f"drone_{k}_state.jsonl", encoding="utf-8")]


def _run(name):
    p = DEMOS / name
    if p not in COMPLETE:
        pytest.skip(f"{name}: no completed run")
    return p


def test_vertical_escape_has_an_up_or_down_component():
    run = _run("p1_vertical_escape")
    summ = json.loads((run / "onboard_summary.json").read_text(encoding="utf-8"))
    dirs = [d for v in summ.values() for d in v["escape_directions"]]
    assert any("UP" in d or "DOWN" in d for d in dirs)
    assert max(v["depth_span_m"] for v in summ.values()) > 0.8               # a real vertical manoeuvre
    m = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    assert not m["P1_separation"]["intruder"]["below_d_safe"]


@pytest.mark.parametrize("name,n", [("formation_triangle", 3), ("formation_square", 4), ("formation_six", 6)])
def test_formation_scenarios(name, n):
    run = _run(name)
    cfg = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    m = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    assert cfg["n_drones"] == n and m["envelope"]["inside_envelope"]
    assert m["P3_formation_recovery"]["final_form_err"] < DEFAULT_E_OK
    summ = json.loads((run / "onboard_summary.json").read_text(encoding="utf-8"))
    for v in summ.values():                       # the drones themselves declare the formation recovered
        assert v["time_in_mode_s"].get("FORMATION_FOLLOW", 0.0) > 0.5 * cfg["duration_s"]


DEFAULT_E_OK = 0.5


def test_gust_formation_is_lost_and_recovered():
    run = _run("formation_gust")
    p3 = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))["P3_formation_recovery"]
    assert p3["n_episodes"] >= 1 and p3["all_recovered_within_run"]


@pytest.mark.parametrize("name", ["gate_single", "integrated_short"])
def test_gate_scenarios_use_the_static_rank_never(name):
    run = _run(name)
    m = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    assert m["run"]["static_rank_uses"] == 0 and m["run"]["occupancy_timeouts"] == 0
