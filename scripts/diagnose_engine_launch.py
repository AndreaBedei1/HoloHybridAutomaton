"""Instrument the existing orchestrator without changing its launch defaults.

Usage: python scripts/diagnose_engine_launch.py --out results/dev/launch/baseline -- <campaign arguments>
No engine or process outside the recorded worker tree is modified. Run one probe at a time.
"""

from __future__ import annotations

import argparse
import atexit
import ctypes
import faulthandler
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import psutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.sim.engine_lease import owned_engine
from holo_fleet.sim.engine_session import reserve_trace_port


def write(path, value):
    path.write_text(json.dumps(value, indent=1, default=str), encoding="utf-8")


def snapshot(p):
    data = p.as_dict(attrs=["pid", "ppid", "create_time", "cmdline", "status", "num_handles"], ad_value=None)
    if os.name == "nt":
        in_job = ctypes.c_int()
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.GetCurrentProcess.restype = ctypes.c_void_p
        if p.pid == os.getpid():
            kernel.IsProcessInJob(ctypes.c_void_p(kernel.GetCurrentProcess()), None, ctypes.byref(in_job))
            data["in_job"] = bool(in_job.value)
    return data


def campaign():
    spec = importlib.util.spec_from_file_location("launch_campaign", ROOT / "scripts/run_sensor_robustness_campaign.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def child(out, argv):
    out.mkdir(parents=True, exist_ok=True)
    fault = open(out / "faulthandler.log", "a", encoding="utf-8")
    faulthandler.enable(file=fault, all_threads=True)
    faulthandler.dump_traceback_later(30, repeat=True, file=fault)  # diagnostic wall time, never a mission timeout
    me = psutil.Process()
    write(out / "worker.json", {"self": snapshot(me), "parents": [snapshot(p) for p in me.parents()]})
    atexit.register(lambda: write(out / "atexit.json", {"time": time.time()}))
    old_hook = sys.excepthook

    def exception(*exc):
        write(out / "exception.json", {"type": exc[0].__name__, "message": str(exc[1])})
        old_hook(*exc)

    sys.excepthook = exception
    from holo_fleet.sim import holoocean_setup as hs
    hs.use_patched_holoocean()
    from holoocean import environments

    original_popen = subprocess.Popen
    engines = []
    # UE 5.3.2 Control.cpp falls back to a high port if 1985 is occupied. Do not replace an existing owner.
    reservation = reserve_trace_port()
    write(out / "trace_reservation.json", {"acquired": reservation is not None})

    def launch(args, **kwargs):
        nonlocal reservation
        if reservation is None:
            reservation = reserve_trace_port()
        args = list(args)
        uuid = next(a.split("=", 1)[1] for a in args if a.startswith("--HolodeckUUID="))
        if not any(a.startswith("-UserDir=") for a in args):
            args += ["-UserDir=" + str((out / uuid).resolve()), "-enginesaveddirsuffix=" + uuid.replace("-", "")]
        p = original_popen(args, **kwargs)
        record = {"pid": p.pid, "ppid": os.getpid(), "created": psutil.Process(p.pid).create_time(),
                  "uuid": uuid, "args": args}
        engines.append((p, record))
        write(out / "engine.json", record)
        return p

    environments.subprocess = SimpleNamespace(Popen=launch)
    stop = threading.Event()

    def heartbeat():
        nonlocal reservation
        with open(out / "heartbeat.jsonl", "a", encoding="utf-8", buffering=1) as log:
            while not stop.wait(1):
                rows = []
                for p, record in engines:
                    info = {**record, "exit_code": p.poll()}
                    try:
                        pr = owned_engine(record)
                        if pr:
                            ports = [c.laddr.port for c in pr.net_connections(kind="inet") if c.status == "LISTEN"]
                            info["listen_ports"] = ports
                            if 1985 in ports:
                                pr = owned_engine(record)
                                if pr is not None:
                                    pr.kill()  # our exact parent/UUID/creation time only
                                info["isolation_failure"] = True
                            elif ports and reservation is not None:
                                reservation.close()
                                reservation = None
                    except psutil.Error as exc:
                        info["inspection_error"] = str(exc)  # a race must not stop monitoring later attempts
                    rows.append(info)
                log.write(json.dumps({"time": time.time(), "engines": rows}) + "\n")

    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    sys.argv = argv
    try:
        return campaign().main()
    finally:
        stop.set()
        thread.join(2)
        if reservation is not None:
            reservation.close()
        write(out / "finally.json", {"time": time.time(), "engines": [
            {**r, "exit_code": p.poll()} for p, r in engines]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--child", action="store_true")
    ap.add_argument("--launch-mode", choices=("inherit", "no-window", "new-console"), default="inherit")
    ap.add_argument("args", nargs=argparse.REMAINDER)
    opt = ap.parse_args()
    opt.out = opt.out.resolve()
    argv = opt.args[1:] if opt.args[:1] == ["--"] else opt.args
    if opt.child:
        return child(opt.out, argv)
    opt.out.mkdir(parents=True, exist_ok=True)
    mod = campaign()
    original_popen = subprocess.Popen

    def launch(args, **kwargs):
        worker_out = opt.out / (str(time.time_ns()))
        command = [sys.executable, str(Path(__file__).resolve()), "--child", "--out", str(worker_out), "--", *args[1:]]
        if os.name == "nt":
            kwargs["creationflags"] = {"inherit": 0, "no-window": subprocess.CREATE_NO_WINDOW,
                                       "new-console": subprocess.CREATE_NEW_CONSOLE}[opt.launch_mode]
            if opt.launch_mode == "new-console":
                si = subprocess.STARTUPINFO()
                si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                si.wShowWindow = 0
                kwargs["startupinfo"] = si
        return original_popen(command, **kwargs)

    # The production orchestrator supplies the shared, identity-checked cleanup and priority boundaries.
    mod.subprocess = SimpleNamespace(**{**vars(subprocess), "Popen": launch})
    sys.argv = [str(ROOT / "scripts/run_sensor_robustness_campaign.py"), *argv]
    return mod.main()


if __name__ == "__main__":
    sys.exit(main())
