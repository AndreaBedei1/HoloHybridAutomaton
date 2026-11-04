"""Launch diagnostics must refuse recycled PIDs, foreign UUIDs and foreign parents."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def diag():
    path = Path(__file__).resolve().parents[1] / "scripts/diagnose_engine_launch.py"
    spec = importlib.util.spec_from_file_location("launch_diag", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("parent,created,uuid,want", [
    (10, 123.5, "ours", True), (11, 123.5, "ours", False),
    (10, 124.5, "ours", False), (10, 123.5, "foreign", False),
])
def test_cleanup_requires_parent_uuid_and_creation_time(diag, monkeypatch, parent, created, uuid, want):
    p = SimpleNamespace(ppid=lambda: parent, create_time=lambda: created,
                        name=lambda: "Holodeck.exe", cmdline=lambda: ["Holodeck.exe", "--HolodeckUUID=" + uuid])
    monkeypatch.setattr(diag.psutil, "Process", lambda pid: p)
    record = {"pid": 20, "ppid": 10, "created": 123.5, "uuid": "ours"}
    assert (diag.owned_engine(record) is not None) == want
