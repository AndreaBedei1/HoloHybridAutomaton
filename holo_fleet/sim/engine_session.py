"""One guarded HoloOcean launch: project slot, private writable root, UUID and socket audit."""

from __future__ import annotations

import os
import shutil
import socket
import threading
from types import SimpleNamespace

import psutil

from holo_fleet.sim.engine_lease import EngineLease, owned_engine, same_process

_MAKE_LOCK = threading.RLock()


def reserve_trace_port():
    """Stock UE 5.3.2 falls back from 1985; do not replace an existing owner of that port."""
    if os.name != "nt":
        return None
    reservation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    reservation.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    try:
        reservation.bind(("0.0.0.0", 1985))
        return reservation
    except OSError:
        reservation.close()
        return None


def make(*, clear_cache=False, **kwargs):
    """Keep the native environment API; all project launchers must use this entry point."""
    with _MAKE_LOCK:
        from holo_fleet.sim import holoocean_setup as hs
        import holoocean
        from holoocean import environments, holoocean as hmake

        lease = EngineLease().acquire()
        reservation = None
        env = None
        try:
            root = hs.patched_root()
            if root is None:
                raise RuntimeError("isolated launches require the patched HoloOcean root")
            private = root.with_name(root.name + f"_slot{lease.slot}")
            if not (private / "PATCH_INFO.json").is_file():
                raise RuntimeError(f"private engine root missing: {private}; run scripts/prepare_engine_roots.ps1")
            lease.update(root=str(private))
            os.environ["HOLODECKPATH"] = str(private)
            if clear_cache:
                cache = hs.octree_cache_dir().resolve()
                if not cache.is_relative_to(private.resolve()):
                    raise RuntimeError("refusing to clear a cache outside the private engine root")
                if cache.exists():
                    shutil.rmtree(cache)
            hs.wait_for_engine_semaphores()
            reservation = reserve_trace_port()
            base = hmake.HoloOceanEnvironment

            class LeasedEnvironment(base):
                def __windows_start_process__(self, *args, **kw):
                    previous = environments.subprocess

                    def launch(command, **options):
                        command = list(command)
                        uuid = self._uuid
                        if "--HolodeckUUID=" + uuid not in command:
                            raise RuntimeError("HoloOcean launch UUID does not match the client")
                        logs = private / "runs" / uuid
                        command += ["-UserDir=" + str(logs), "-enginesaveddirsuffix=" + uuid.replace("-", "")]
                        lease.update(uuid=uuid, engine={"pending": True})
                        try:
                            proc = previous.Popen(command, **options)
                        except OSError:
                            try:
                                children = [p.cmdline() for p in psutil.Process().children()]
                                if all(cmd and "--HolodeckUUID=" + uuid not in cmd for cmd in children):
                                    lease.update(engine=None)  # a rejected creation with no child is not an orphan
                            except psutil.Error:
                                pass  # an uncertain handoff stays reserved
                            raise
                        self._world_process = proc  # keep the native handle even if metadata recording fails
                        lease.update(engine={"pid": proc.pid, "ppid": os.getpid(), "uuid": uuid})
                        record = {"pid": proc.pid, "ppid": os.getpid(),
                                  "created": psutil.Process(proc.pid).create_time(), "uuid": uuid}
                        lease.update(engine=record, logs=str(logs), command=command)
                        return proc

                    environments.subprocess = SimpleNamespace(**{**vars(previous), "Popen": launch})
                    try:
                        return super().__windows_start_process__(*args, **kw)
                    finally:
                        environments.subprocess = previous

                def __on_exit__(self):
                    record = lease.record.get("engine")
                    if record and same_process(record) and owned_engine(record) is None:
                        raise RuntimeError("engine identity cannot be verified; refusing cleanup")
                    try:
                        return super().__on_exit__()
                    finally:
                        lease.release()

            hmake.HoloOceanEnvironment = LeasedEnvironment
            try:
                env = holoocean.make(**kwargs)
            finally:
                hmake.HoloOceanEnvironment = base
            record = lease.record.get("engine")
            proc = owned_engine(record) if record else None
            if proc is None:
                raise RuntimeError("the launched engine identity could not be verified")
            ports = sorted(c.laddr.port for c in proc.net_connections(kind="inet") if c.status == "LISTEN")
            if not ports or 1985 in ports:
                raise RuntimeError(f"non-default TraceLog listener not verified ({ports}); isolated launch refused")
            lease.update(listen_ports=ports)
            env._holo_lease = lease
            env._holo_engine = dict(lease.record)
            return env
        except BaseException:
            record = lease.record.get("engine")
            proc = owned_engine(record) if record else None
            if proc is not None:
                proc.kill()
                proc.wait(10)  # same cleanup allowance as the installed client's __on_exit__
            lease.release()
            raise
        finally:
            if reservation is not None:
                reservation.close()
