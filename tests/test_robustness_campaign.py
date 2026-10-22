"""The robustness campaign orchestrator: resume rules, our processes only, run metadata.

No HoloOcean here: process lists and run directories are faked.
"""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def camp():
    spec = importlib.util.spec_from_file_location("campaign_under_test", ROOT / "scripts" / "run_sensor_robustness_campaign.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _metrics(raw, run, **kw):
    d = raw.joinpath(run[0], run[1], run[2], f"s{run[3]}")
    d.mkdir(parents=True, exist_ok=True)
    (d / "run_metrics.json").write_text(json.dumps(kw), encoding="utf-8")


def test_invalid_runs_are_retried_and_complete_runs_kept(camp, tmp_path):
    runs = [("B0", "CLEAN", "gate_single", s) for s in range(5)]
    _metrics(tmp_path, runs[0], **{"class": "PASS"})
    _metrics(tmp_path, runs[1], **{"class": "LIVENESS_FAILURE"})            # a scientific failure: kept
    _metrics(tmp_path, runs[2], **{"class": "INVALID", "invalid_reason": "started"})   # infrastructure: retried
    d = tmp_path.joinpath("B0", "CLEAN", "gate_single", "s3")
    d.mkdir(parents=True)
    (d / "run_metrics.json").write_text("{ truncated", encoding="utf-8")      # unreadable: retried
    # runs[4]: never started
    assert camp.pending(tmp_path, runs) == [runs[2], runs[3], runs[4]]


def test_only_the_engine_of_our_worker_is_terminated(camp, monkeypatch):
    killed = []
    monkeypatch.setattr(camp, "holodeck_pids", lambda: {11: 500, 12: 999, 13: 500, 14: 1})
    monkeypatch.setattr("psutil.Process", lambda pid: SimpleNamespace(kill=lambda: killed.append(pid)))
    # 11 and 13 were started by our worker 500 (13 already existed before the run: not ours to kill);
    # 12 belongs to another program's process 999, 14 to a foreign parent
    assert camp.reap(500, before={13: 500}) == [11]
    assert killed == [11]


def test_priority_is_lowered_only_for_our_worker_and_its_engine(camp, monkeypatch):
    import psutil

    touched = []

    class P:
        def __init__(self, pid):
            self.pid = pid

        def nice(self, value=None):
            if value is None:
                return psutil.NORMAL_PRIORITY_CLASS if hasattr(psutil, "NORMAL_PRIORITY_CLASS") else 0
            touched.append(self.pid)

    monkeypatch.setattr(camp, "holodeck_pids", lambda: {21: 700, 22: 42})
    monkeypatch.setattr("psutil.Process", P)
    monkeypatch.setattr(camp.os, "name", "nt")
    if not hasattr(psutil, "BELOW_NORMAL_PRIORITY_CLASS"):
        pytest.skip("Windows priority classes only")
    camp.lower_children(700)
    assert sorted(touched) == [21, 700]


def test_code_version_names_the_commit_and_the_dirty_files(camp):
    v = camp.code_version()
    assert len(v["sha"]) == 40 and all(c in "0123456789abcdef" for c in v["sha"])
    assert all(not f.startswith(" ") and len(f) > 2 for f in v["dirty_files"])


def test_no_cpu_threshold_before_a_run(camp, monkeypatch):
    """A busy machine only makes a run slower: the admission waits for memory only, never for CPU."""
    import psutil

    monkeypatch.setattr(psutil, "cpu_percent", lambda interval=None: 100.0)
    monkeypatch.setattr(psutil, "virtual_memory", lambda: SimpleNamespace(available=32 * 2**30))
    host = camp.wait_for_memory(SimpleNamespace(min_free_ram_gb=8.0), say=lambda m: pytest.fail("waited: " + m))
    assert host["waited_s"] == 0 and host["cpu_percent"] == 100
