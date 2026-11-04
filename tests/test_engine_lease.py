"""The project instance cap must survive competing opens, process death and PID reuse."""

import json
import os
import subprocess
import sys

import psutil
import pytest

from holo_fleet.sim import engine_lease
from holo_fleet.sim.engine_lease import EngineLease


def test_live_lease_cannot_be_taken_and_release_allows_reuse(tmp_path):
    first = EngineLease(tmp_path, capacity=1).acquire()
    try:
        with pytest.raises(RuntimeError, match="busy"):
            EngineLease(tmp_path, capacity=1).acquire()
    finally:
        first.release()
    again = EngineLease(tmp_path, capacity=1).acquire()
    again.release()


def test_two_slots_are_the_absolute_limit(tmp_path):
    leases = [EngineLease(tmp_path, capacity=2).acquire() for _ in range(2)]
    try:
        assert {p.slot for p in leases} == {0, 1}
        with pytest.raises(RuntimeError, match="busy"):
            EngineLease(tmp_path, capacity=2).acquire()
        with pytest.raises(ValueError):
            EngineLease(tmp_path, capacity=3)
    finally:
        for lease in leases:
            lease.release()


def test_recycled_owner_pid_is_stale_but_live_orphan_engine_blocks_reuse(tmp_path):
    p = psutil.Process()
    stale = {"pid": os.getpid(), "created": p.create_time() - 1, "uuid": "old", "engine": None}
    (tmp_path / "slot0.json").write_text(json.dumps(stale))
    lease = EngineLease(tmp_path, capacity=1).acquire()
    lease.release()
    stale["engine"] = {"pid": os.getpid(), "created": p.create_time()}
    (tmp_path / "slot0.json").write_text(json.dumps(stale))
    with pytest.raises(RuntimeError, match="busy"):
        EngineLease(tmp_path, capacity=1).acquire()


def test_corrupt_slot_fails_closed_without_erasing_evidence(tmp_path):
    path = tmp_path / "slot0.json"
    path.write_bytes(b"{broken")
    with pytest.raises(RuntimeError, match="unreadable"):
        EngineLease(tmp_path, capacity=1).acquire()
    assert path.read_bytes() == b"{broken"


def test_failed_atomic_update_preserves_the_last_identity(tmp_path, monkeypatch):
    lease = EngineLease(tmp_path, capacity=1).acquire()
    lease.update(uuid="ours", engine={"pending": True})
    before = (tmp_path / "slot0.json").read_bytes()
    def fail(*args):
        raise OSError("interrupted persistence")
    with monkeypatch.context() as patch:
        patch.setattr(os, "replace", fail)
        with pytest.raises(OSError, match="interrupted"):
            lease.update(engine={"pid": 123, "created": 5.0})
    assert (tmp_path / "slot0.json").read_bytes() == before
    lease.release()


def test_interrupted_launch_handoff_fails_closed_after_owner_death(tmp_path):
    pending = {"pid": os.getpid(), "created": psutil.Process().create_time() - 1,
               "uuid": "launching", "engine": {"pending": True}}
    path = tmp_path / "slot0.json"
    path.write_text(json.dumps(pending))
    with pytest.raises(RuntimeError, match="busy"):
        EngineLease(tmp_path, capacity=1).acquire()
    assert json.loads(path.read_text()) == pending


def test_watcher_uses_complete_records_of_the_exact_worker_only(tmp_path, monkeypatch):
    monkeypatch.setattr(engine_lease, "SLOTS", tmp_path)
    record = {"pid": 10, "created": 1.0, "uuid": "u", "engine": {"pending": True}}
    path = tmp_path / "slot0.json"
    path.write_text(json.dumps(record))
    assert engine_lease.worker_engines(10, 1.0) == []
    record["engine"] = {"pid": 20, "created": 2.0, "uuid": "u", "ppid": 10}
    path.write_text(json.dumps(record))
    assert engine_lease.worker_engines(10, 1.0) == [record["engine"]]
    assert engine_lease.worker_engines(10, 3.0) == []


def test_os_releases_the_slot_after_owner_exits_without_finally(tmp_path):
    code = ("import os,sys\nfrom holo_fleet.sim.engine_lease import EngineLease\n"
            "lease=EngineLease(sys.argv[1],capacity=1).acquire()\n"
            "print('ready',flush=True)\ninput()\nos._exit(0)\n")
    proc = subprocess.Popen([sys.executable, "-c", code, str(tmp_path)], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert proc.stdout.readline().strip() == "ready"
        with pytest.raises(RuntimeError, match="busy"):
            EngineLease(tmp_path, capacity=1).acquire()
    finally:
        _, err = proc.communicate("exit\n", timeout=10)
    assert proc.returncode == 0, err
    lease = EngineLease(tmp_path, capacity=1).acquire()
    lease.release()
