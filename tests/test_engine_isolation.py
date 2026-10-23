"""Coexistence with other HoloOcean / Unreal programs on the same machine (CARLA, other projects).

HoloOcean's client and engine meet through shared memory and named semaphores whose names carry a fresh
per-instance UUID: no network port is involved, so an instance of another program cannot take "our" port or be
reached by us.  These tests pin that fact in the installed client (a client upgrade that introduced a fixed port
would fail here) and check that the run metadata records the sockets of OUR engine only (Unreal's TraceLog
control port, 1985 or a free high port when 1985 is taken: recorded, never used).
"""

import inspect
import re
from types import SimpleNamespace

import pytest

holoocean = pytest.importorskip("holoocean")
from holoocean import environments, holooceanclient, shmem  # noqa: E402
from holoocean import holoocean as hmake  # noqa: E402

from holo_fleet.sim.holo_env import HoloFleetSim  # noqa: E402


def test_every_environment_gets_a_fresh_uuid():
    assert re.search(r'\["uuid"\]\s*=\s*str\(uuid\.uuid4\(\)\)', inspect.getsource(hmake.make))


def test_semaphores_and_shared_memory_are_named_by_the_uuid():
    client = inspect.getsource(holooceanclient.HoloOceanClient)
    for name in ("HOLODECK_SEMAPHORE_SERVER", "HOLODECK_SEMAPHORE_CLIENT"):
        assert re.search(name + r'"\s*\+\s*self\._uuid', client), name
    assert re.search(r'"/HOLODECK_MEM"\s*\+\s*uuid', inspect.getsource(shmem.Shmem))


def test_the_engine_is_started_with_the_uuid_and_no_port():
    for meth in ("__windows_start_process__", "__linux_start_process__"):
        src = inspect.getsource(getattr(environments.HoloOceanEnvironment, meth))
        flags = re.findall(r'"(-{1,2}[A-Za-z]+)', src)
        assert "--HolodeckUUID" in flags, meth
        assert not [f for f in flags if "port" in f.lower()], (meth, flags)


def test_run_metadata_records_the_sockets_of_our_engine_only(monkeypatch):
    asked = []

    class Proc:
        def __init__(self, pid):
            asked.append(pid)

        def net_connections(self, kind="inet"):
            c = lambda port, status: SimpleNamespace(laddr=SimpleNamespace(port=port), status=status)
            return [c(52311, "LISTEN"), c(1985, "ESTABLISHED"), c(60001, "TIME_WAIT")]

    monkeypatch.setattr("psutil.Process", Proc)
    sim = SimpleNamespace(env=SimpleNamespace(_world_process=SimpleNamespace(pid=4242), _uuid="u-1"))
    ident = HoloFleetSim._engine_identity(sim)
    assert ident == {"ipc_uuid": "u-1", "pid": 4242, "listen_ports": [52311]}
    assert asked == [4242]                        # only our engine's process is inspected
