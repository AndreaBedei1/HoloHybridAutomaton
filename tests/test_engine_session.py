"""A refused or failed launch must not consume an engine slot or leak a client patch."""

import os
from types import SimpleNamespace

import psutil
import pytest

from holo_fleet.sim import engine_lease, engine_session


def test_busy_slot_refuses_before_calling_holoocean(tmp_path, monkeypatch):
    holoocean = pytest.importorskip("holoocean")
    monkeypatch.setattr(engine_lease, "SLOTS", tmp_path / "slots")
    monkeypatch.setenv("HOLO_FLEET_ENGINE_SLOTS", "1")
    monkeypatch.setattr(holoocean, "make", lambda **kw: pytest.fail("must refuse before starting a client"))
    lease = engine_lease.EngineLease().acquire()
    try:
        with pytest.raises(RuntimeError, match="busy"):
            engine_session.make(scenario_cfg={})
    finally:
        lease.release()


def test_failed_make_releases_lease_and_restores_client(tmp_path, monkeypatch):
    holoocean = pytest.importorskip("holoocean")
    from holoocean import holoocean as hmake
    from holo_fleet.sim import holoocean_setup as hs

    monkeypatch.setattr(engine_lease, "SLOTS", tmp_path / "slots")
    monkeypatch.setenv("HOLO_FLEET_ENGINE_SLOTS", "1")
    root = tmp_path / "root"
    private = tmp_path / "root_slot0"
    private.mkdir()
    (private / "PATCH_INFO.json").write_text("{}")
    def locked_root():
        assert engine_session._MAKE_LOCK._is_owned(), "root selection must be serialized with native make"
        return root
    monkeypatch.setattr(hs, "patched_root", locked_root)
    old_class = hmake.HoloOceanEnvironment

    def fail(**kw):
        assert hmake.HoloOceanEnvironment is not old_class
        with open(engine_lease.SLOTS / "startup.lock", "a+b") as file:
            with pytest.raises(OSError):
                engine_lease.EngineLease._lock(file, True)
        raise ValueError("bad scenario")

    monkeypatch.setattr(holoocean, "make", fail)
    with pytest.raises(ValueError, match="bad scenario"):
        engine_session.make(scenario_cfg={})
    assert hmake.HoloOceanEnvironment is old_class
    lease = engine_lease.EngineLease().acquire()
    lease.release()


@pytest.mark.parametrize("ports,allowed,launch_error", [
    ([37891], True, False), ([1985], False, False), ([], False, False), ([], False, True)])
def test_native_cleanup_and_port_audit_keep_the_lease_until_engine_exit(tmp_path, monkeypatch, ports, allowed, launch_error):
    holoocean = pytest.importorskip("holoocean")
    from holoocean import environments, holoocean as hmake
    from holo_fleet.sim import holoocean_setup as hs

    monkeypatch.setattr(engine_lease, "SLOTS", tmp_path / "slots")
    monkeypatch.setenv("HOLO_FLEET_ENGINE_SLOTS", "1")
    monkeypatch.setenv("HOLODECKPATH", "test-original")
    root = tmp_path / "root"
    private = tmp_path / "root_slot0"
    private.mkdir()
    (private / "PATCH_INFO.json").write_text("{}")
    monkeypatch.setattr(hs, "patched_root", lambda: root)
    monkeypatch.setattr(hs, "wait_for_engine_semaphores", lambda: None)
    monkeypatch.setattr(engine_session, "reserve_trace_port", lambda: None)
    alive, command = [not launch_error], []
    original_process = psutil.Process

    class Engine:
        pid = 500

        def create_time(self): return 123.5
        def ppid(self): return os.getpid()
        def name(self): return "Holodeck.exe"
        def cmdline(self): return command
        def net_connections(self, kind):
            return [SimpleNamespace(status="LISTEN", laddr=SimpleNamespace(port=port)) for port in ports]
        def kill(self): alive[0] = False
        def wait(self, timeout): return 1

    def process(pid=None):
        if pid is None:
            return SimpleNamespace(create_time=lambda: original_process().create_time(), children=lambda: [])
        if pid != 500:
            return original_process(pid)
        if not alive[0]:
            raise psutil.NoSuchProcess(pid)
        return Engine()

    def popen(args, **kw):
        if launch_error:
            raise FileNotFoundError("engine missing")
        command.extend(args)
        return Engine()

    class NativeEnvironment:
        def __init__(self, **kw):
            self._uuid = "unit-uuid"
            self.__windows_start_process__()
        def __windows_start_process__(self):
            self._world_process = environments.subprocess.Popen(["Holodeck.exe", "--HolodeckUUID=unit-uuid"])
        def __on_exit__(self):
            self._world_process.kill()
            self._world_process.wait(10)
        def __exit__(self, *args):
            self.__on_exit__()

    monkeypatch.setattr(psutil, "Process", process)
    monkeypatch.setattr(environments, "subprocess", SimpleNamespace(Popen=popen))
    monkeypatch.setattr(hmake, "HoloOceanEnvironment", NativeEnvironment)
    monkeypatch.setattr(holoocean, "make", lambda **kw: hmake.HoloOceanEnvironment(**kw))
    if launch_error:
        with pytest.raises(FileNotFoundError, match="engine missing"):
            engine_session.make(scenario_cfg={})
    elif allowed:
        env = engine_session.make(scenario_cfg={})
        with pytest.raises(RuntimeError, match="busy"):
            engine_lease.EngineLease().acquire()
        assert env._holo_engine["listen_ports"] == [37891]
        assert "-UserDir=" + str(private / "runs/unit-uuid") in command
        env.__exit__(None, None, None)
    else:
        with pytest.raises(RuntimeError, match="not verified"):
            engine_session.make(scenario_cfg={})
    assert not alive[0]
    assert hmake.HoloOceanEnvironment is NativeEnvironment
    lease = engine_lease.EngineLease().acquire()
    lease.release()
