"""scripts/run_demo.py: a plain demo keeps nothing; --save keeps a new result, --overwrite replaces it,
--media alone produces frames and GIFs.  The simulation is replaced by a fake run that writes the
same kind of files as holo_fleet.runner.run."""

import importlib.util
import json
import math
import os
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def demo(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("run_demo_under_test", ROOT / "scripts" / "run_demo.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    tmp = tmp_path / "tmp"
    tmp.mkdir()
    monkeypatch.setattr(tempfile, "tempdir", str(tmp))
    monkeypatch.setattr(mod, "OUT", tmp_path / "demos")
    calls = {"run": [], "gif": []}
    state = {"status": "COMPLETE"}

    def fake_run(name, out_root, headless=True, run_id=None, duration=None, ui=None, frame_every_s=0.5):
        calls["run"].append(frame_every_s)
        out = Path(out_root) / (run_id or name)
        (out / "frames").mkdir(parents=True, exist_ok=True)          # as the runner does
        (out / "run_status.json").write_text(json.dumps({"status": state["status"]}), encoding="utf-8")
        (out / "events.jsonl").write_text("{}\n", encoding="utf-8")
        (out / "referee_metrics.json").write_text("{}", encoding="utf-8")
        if math.isfinite(frame_every_s):                              # frames only when asked for
            (out / "fleet_view").mkdir()
            (out / "fleet_view" / "fleet_00001.jpg").write_bytes(b"jpg")
            (out / "frames" / "chase_00001.jpg").write_bytes(b"jpg")
        return {"run": {"status": state["status"]}}

    def fake_gif(frames, path, **_kw):
        calls["gif"].append(Path(path))
        Path(path).write_bytes(b"gif")
        return Path(path)

    monkeypatch.setattr(mod, "run", fake_run)
    monkeypatch.setattr(mod, "make_gif", fake_gif)
    monkeypatch.setattr(mod, "print_summary", lambda *a, **k: None)
    monkeypatch.setattr(mod, "DemoUI", lambda *a, **k: None)
    mod.calls, mod.state, mod.tmp = calls, state, tmp
    return mod


def _files(path: Path):
    return sorted(str(p.relative_to(path)) for p in path.rglob("*")) if path.exists() else []


def test_default_demo_keeps_nothing(demo):
    assert demo.main(["--scenario", "formation_triangle"]) == 0
    assert _files(demo.OUT) == []                                   # nothing under results/v2/demos
    assert _files(demo.tmp) == []                                   # the staging folder is gone
    assert demo.calls["run"] == [math.inf] and demo.calls["gif"] == []


def test_save_keeps_a_new_result_without_media(demo):
    assert demo.main(["--scenario", "formation_triangle", "--save"]) == 0
    dest = demo.OUT / "formation_triangle"
    assert _files(dest) == ["events.jsonl", "referee_metrics.json", "run_status.json"]
    assert _files(demo.tmp) == []


def test_save_refuses_to_replace_an_existing_result(demo):
    dest = demo.OUT / "formation_triangle"
    dest.mkdir(parents=True)
    (dest / "run_status.json").write_text('{"status": "COMPLETE", "canonical": true}', encoding="utf-8")
    assert demo.main(["--scenario", "formation_triangle", "--save"]) == 2
    assert demo.calls["run"] == []                                  # refused before starting anything
    assert json.loads((dest / "run_status.json").read_text(encoding="utf-8"))["canonical"] is True


def test_save_overwrite_replaces_the_result(demo):
    dest = demo.OUT / "formation_triangle"
    dest.mkdir(parents=True)
    (dest / "old_only.json").write_text("{}", encoding="utf-8")
    (dest / "formation_triangle.gif").write_bytes(b"old gif")
    assert demo.main(["--scenario", "formation_triangle", "--save", "--overwrite"]) == 0
    assert _files(dest) == ["events.jsonl", "referee_metrics.json", "run_status.json"]
    assert _files(demo.OUT) == ["formation_triangle"] + [f"formation_triangle{os.sep}{f}" for f in _files(dest)]


def test_media_only_with_media(demo):
    demo.main(["--scenario", "formation_triangle", "--save"])
    assert demo.calls["gif"] == [] and demo.calls["run"] == [math.inf]
    assert demo.main(["--scenario", "formation_triangle", "--save", "--overwrite", "--media"]) == 0
    dest = demo.OUT / "formation_triangle"
    assert demo.calls["run"][-1] == 0.5
    assert {"formation_triangle.gif", f"fleet_view{os.sep}fleet_00001.jpg", f"frames{os.sep}chase_00001.jpg"} <= set(_files(dest))


@pytest.mark.parametrize("flags", [["--media"], ["--overwrite"], ["--out", "x"]])
def test_save_only_flags_without_save_are_rejected(demo, flags):
    with pytest.raises(SystemExit):
        demo.main(["--scenario", "formation_triangle", *flags])
    assert demo.calls["run"] == []


def test_an_incomplete_run_is_never_saved(demo):
    dest = demo.OUT / "formation_triangle"
    dest.mkdir(parents=True)
    (dest / "run_status.json").write_text('{"status": "COMPLETE"}', encoding="utf-8")
    demo.state["status"] = "INCOMPLETE"
    assert demo.main(["--scenario", "formation_triangle", "--save", "--overwrite"]) == 1
    assert _files(dest) == ["run_status.json"]                      # the canonical result is untouched
    assert _files(demo.tmp) == []


def test_staging_left_by_a_killed_demo_is_removed(demo, monkeypatch):
    dead = demo.tmp / (demo.STAGE_PREFIX + "killed")
    dead.mkdir()
    (dead / "owner.pid").write_text("123456", encoding="utf-8")
    alive = demo.tmp / (demo.STAGE_PREFIX + "running")
    alive.mkdir()
    (alive / "owner.pid").write_text(str(os.getpid()), encoding="utf-8")
    monkeypatch.setattr(demo, "_alive", lambda pid: pid == os.getpid())
    demo.main(["--scenario", "formation_triangle"])
    assert not dead.exists() and alive.exists()
