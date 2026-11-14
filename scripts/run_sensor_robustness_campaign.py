"""B0 / B2 robustness campaign orchestrator (headless HoloOcean, resumable, optionally sharded).

    python scripts/run_sensor_robustness_campaign.py --phase final [--dry-run]
    python scripts/run_sensor_robustness_campaign.py --phase targeted --only lost_drone_timeout \
        --protocols B0 B2_CLEAN --profiles CLEAN LOW-ALL --seeds 0 --raw <root>

Every run executes in its own Python process (holo_fleet.runner.run, no frames, no GIF) and writes a run
directory under the raw root (default results/robustness/raw_final/, git-ignored):
    <protocol>/<profile>/<scenario>/s<seed>/
followed by run_metrics.json (holo_fleet.referee.robustness_metrics: metrics + class).  A run whose
run_metrics.json exists (and is not INVALID) is skipped, so an interrupted campaign resumes where it stopped.
Bulky per-step logs (drone_*_state.jsonl) are gzip-compressed after the analysis.  Each orchestrator runs one
HoloOcean instance at a time (``--shard i --nshards n``: n orchestrators on disjoint slices); after every run the
Holodeck.exe processes started by that run (and only those) are terminated if still alive.

Protocols: B0 (communication-free) and B2_<channel level> (one-way acoustic identity beacon, sim/acoustic_channel.py).
Profiles: CLEAN or <LEVEL>-<SCOPE> (sim/sensor_noise.py; scope ONE / SUBSET / ALL, scope_drones).  Every run ends
on SUCCESS / FAILURE / DEADLINE (referee/terminal.py, criteria v3); the deadline is a priori, from the mission plan
and the protocol bounds (referee/criteria.deadline_v3), the same for every protocol and cell.
"""

from __future__ import annotations

import argparse
import dataclasses
import gzip
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.sim.engine_lease import owned_engine, worker_engines

RAW = ROOT / "results" / "robustness" / "raw_final"        # the one authoritative output location
DEMOS = ROOT / "results" / "reference"
SEEDS = (0, 1, 2)
ALL_SCENARIOS = ("p1_head_on", "p1_vertical_escape", "p1_two_lines", "p1_close_encounter", "formation_triangle",
                 "formation_square", "formation_six", "formation_recovery_head_current", "formation_gust", "gate_single",
                 "integrated_short", "lost_drone_rejoin", "lost_drone_rejoin_front", "lost_drone_timeout", "mutex_deadlock_resolution",
                 "line_parallel_mutex", "lost_drone_mutex")
PY = sys.executable


def scope_drones(scenario: str, scope: str) -> list:
    """Drones that receive the degraded profile (deterministic, from the committed CLEAN reference run):
    ONE    = the healthy drone that comes closest to another vehicle (drone or non-cooperative vehicle) or to a gate
             bar in the reference (the drone whose sensing matters most; ties: lowest index);
    SUBSET = ceil(n / 2) healthy drones: ONE and the next healthy drones by index (cyclic);
    ALL    = every drone."""
    import csv

    import numpy as np

    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS

    sc = SCENARIOS[scenario](DEFAULT)
    n = len(sc.sim.names)
    if scope == "ALL":
        return list(range(n))
    lost = {f["drone"] for f in getattr(sc.sim, "faults", ()) or () if f.get("t_off") is None}
    healthy = [k for k in range(n) if sc.sim.names[k] not in lost]
    ref_name = {"lost_drone_rejoin_front": "lost_drone_rejoin"}.get(scenario, scenario)
    rows = list(csv.DictReader(open(DEMOS / ref_name / "referee_timeseries.csv", encoding="utf-8")))
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in rows])
    bars = [b for st in sc.plans[0].structures for b in st.bars]
    near = np.full(n, 1e9)
    for k in range(n):
        for j in range(n):
            if j != k:
                near[k] = min(near[k], np.linalg.norm(P[:, k] - P[:, j], axis=1).min())
        if rows[0].get("ix0"):
            q = np.array([[float(r["ix0"]), float(r["iy0"]), float(r["iz0"])] for r in rows])
            near[k] = min(near[k], np.linalg.norm(P[:, k] - q, axis=1).min())
        for b in bars:
            d = np.maximum(np.abs((P[:, k] - b.center) @ b.axes) - b.half, 0.0)
            near[k] = min(near[k], np.linalg.norm(d, axis=1).min())
    one = min(healthy, key=lambda k: (round(float(near[k]), 2), k))
    if scope == "ONE":
        return [one]
    order = [(one + i) % n for i in range(n)]
    return sorted([k for k in order if k in healthy][:(n + 1) // 2])


def plan(phase: str, seeds, only=None, protocols=None, profiles=None):
    """Runs (protocol, profile, scenario, seed); deadlines are a priori (criteria v3), so no run depends on another.
    ``final``: the campaign matrix; ``targeted``: --only scenarios x --protocols x --profiles (targeted validation)."""
    if phase == "final":
        runs = final_matrix(seeds, only)
    elif phase == "targeted":
        if not (only and protocols and profiles):
            raise SystemExit("--phase targeted needs --only, --protocols and --profiles")
        runs = [(proto, prof, sc, s) for sc in only for prof in profiles for proto in protocols for s in seeds]
    else:
        raise SystemExit(f"unknown phase {phase}")
    if only:
        runs = [r for r in runs if r[2] in only]
    if profiles:
        runs = [r for r in runs if r[1] in profiles]
    return list(dict.fromkeys(runs))


def final_matrix(seeds, only=None) -> list:
    """B0 vs B2 on the same scenarios and seeds (provisional: redefined when the final plan is frozen):
    * sensors CLEAN, LOW-ALL and MEDIUM-{ONE,SUBSET,ALL}: B0, and B2 with the channel at the sensors' level (LOW-ONE
      and LOW-SUBSET are dropped: LOW-ALL is the most degraded LOW case and the LOW scopes are characterised for B0
      in the sensor-robustness campaign);
    * the channel alone: sensors CLEAN with the channel MEDIUM and BLACKOUT (LOW lies between CLEAN and MEDIUM);
    * channel BLACKOUT with MEDIUM-ALL sensors (B2 must fall back to B0)."""
    scen = tuple(only) if only else FINAL_SCENARIOS
    runs = []
    for sc in scen:
        runs += [(proto, "CLEAN", sc, s) for proto in ("B0", "B2_CLEAN") for s in seeds]
    for sc in scen:
        for prof, lvl in (("LOW-ALL", "LOW"), ("MEDIUM-ONE", "MEDIUM"), ("MEDIUM-SUBSET", "MEDIUM"), ("MEDIUM-ALL", "MEDIUM")):
            runs += [(proto, prof, sc, s) for proto in ("B0", f"B2_{lvl}") for s in seeds]
        runs += [(f"B2_{ch}", "CLEAN", sc, s) for ch in ("MEDIUM", "BLACKOUT") for s in seeds]
        runs += [("B2_BLACKOUT", "MEDIUM-ALL", sc, s) for s in seeds]
    return runs


FINAL_SCENARIOS = ALL_SCENARIOS

# the gate scenarios of the standard set (the final campaign of criteria v8 is defined in scripts/final_campaign.py)
B2_GATE_SCENARIOS = ("gate_single", "integrated_short", "mutex_deadlock_resolution", "line_parallel_mutex",
                     "lost_drone_mutex")


def run_dir(raw: Path, protocol: str, prof: str, sc: str, seed: int) -> Path:
    return raw / protocol / prof / sc / f"s{seed}"


def archive_attempt(d: Path, raw: Path) -> Optional[Path]:
    """Retain invalid/interrupted evidence before retrying; never overwrite the previous worker log."""
    raw, d = raw.resolve(), d.resolve()
    if not d.is_relative_to(raw):
        raise ValueError("run directory is outside its raw root")
    log = d.parent / f"{d.name}_worker.log"
    if not d.exists() and not log.exists():
        return None
    archive = (raw / "_attempts" / d.relative_to(raw) / str(time.time_ns())).resolve()
    if not archive.is_relative_to(raw):
        raise ValueError("attempt archive is outside its raw root")
    archive.parent.mkdir(parents=True, exist_ok=True)
    if d.exists():
        d.rename(archive)
    else:
        archive.mkdir()
    if log.exists():
        target, suffix = archive / "worker.log", 0
        while target.exists():
            suffix += 1
            target = archive / f"worker_pending_{suffix}.log"
        log.rename(target)
    return archive


def worker(args) -> int:
    """One run, in this (fresh) process."""
    from holo_fleet import runner
    from holo_fleet.config import DEFAULT, BeaconConfig
    from holo_fleet.sim.sensor_noise import profile

    from holo_fleet.sim import holoocean_setup as hs

    if hs.patched_root() is None and not args.allow_unpatched:
        print(f"patched HoloOcean root not found ({hs.DEFAULT_PATCHED_ROOT}): refusing to run "
              "(the baseline needs the octree patch, docs/HOLOOCEAN_OCTREE_PATCH.md)", file=sys.stderr)
        return 4
    cfg = DEFAULT
    if args.protocol.startswith("B2_"):            # range-bearing beacon, channel level after the underscore
        cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB", channel=args.protocol[3:]))
    elif args.protocol.startswith("B2NP_"):        # ablation: the same beacon without the H6 payload
        cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB", channel=args.protocol[5:],
                                                               payload=False))
    d = Path(args.run_dir)
    prof, _, scope = args.profile.partition("-")
    # criteria v3 (referee/criteria.py): a-priori requirements, deadline from the plan and the protocol bounds
    terminal = {"profile": args.profile} if args.terminal else None
    factor = float(os.environ.get("HOLO_FLEET_HORIZON_FACTOR", "1"))
    if terminal is not None and factor != 1.0:          # exploratory: a multiple of the v5 horizon, never for results
        from holo_fleet.referee.criteria import horizon_v5
        from holo_fleet.sim.scenarios import SCENARIOS

        terminal["deadline_s"] = round(factor * horizon_v5(SCENARIOS[args.scenario](DEFAULT), DEFAULT, args.profile), 1)
    runner.run(args.scenario, d.parent, headless=True, run_id=d.name, frame_every_s=float("inf"), cfg=cfg,
               noise=profile(prof), seed=args.seed, noise_drones=scope_drones(args.scenario, scope) if scope else None,
               terminal=terminal)
    return 0


def holodeck_pids():
    import psutil

    out = {}
    for p in psutil.process_iter(["name"]):        # the name first: ppid of every process is slow on Windows
        if str(p.info.get("name") or "").lower() == "holodeck.exe":
            try:
                out[p.pid] = p.ppid()
            except Exception:  # noqa: BLE001 - gone meanwhile
                pass
    return out


def reap(worker_pid: int, before: dict, worker_created=None, records=()) -> list:
    """Terminate Holodeck.exe processes started by our worker (never anyone else's)."""
    import psutil

    killed = []
    evidence = {r["pid"]: r for r in [*records, *worker_engines(worker_pid, worker_created)]}
    for pid, record in evidence.items():
        if pid in before or record["ppid"] != worker_pid:
            continue
        try:
            p = owned_engine(record)
            if p is not None:
                p.kill()
                p.wait(10)
                killed.append(pid)
        except Exception:  # noqa: BLE001
            pass
    return killed


def be_nice(cpus=None) -> None:
    """Shared machine: the campaign and every process it starts (they inherit it) yield CPU and disk to other
    work: below-normal priority class and low I/O priority on Windows, nice 10 elsewhere; with ``cpus`` the
    orchestrator (and so its workers and engines) is also kept on those logical cores."""
    import psutil

    try:
        me = psutil.Process()
        me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS if os.name == "nt" else 10)
        if os.name == "nt":
            me.ionice(psutil.IOPRIO_LOW)
        if cpus:
            me.cpu_affinity(list(cpus))
    except Exception:  # noqa: BLE001 - courtesy only
        pass


def parse_cpus(text):
    """'0-19' or '0,2,4-7' -> sorted list of logical cores (None when empty)."""
    if not text:
        return None
    out = set()
    for part in text.split(","):
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return sorted(out)


def lower_children(worker_pid: int, worker_created=None, records=()) -> None:
    """Make sure the worker and the Holodeck.exe it started run below normal priority (in case it was not inherited)
    and on the orchestrator's cores (the Unreal engine resets its own process affinity to every core)."""
    import psutil

    try:
        cores = psutil.Process().cpu_affinity()
    except Exception:  # noqa: BLE001
        cores = None
    targets = [p for r in records if r["ppid"] == worker_pid for p in [owned_engine(r)] if p is not None]
    try:
        worker_process = psutil.Process(worker_pid)
        if worker_process.create_time() == worker_created and worker_process.ppid() == os.getpid():
            targets.append(worker_process)
    except psutil.Error:
        pass
    for pr in targets:
        try:
            if os.name == "nt" and pr.nice() not in (psutil.BELOW_NORMAL_PRIORITY_CLASS, psutil.IDLE_PRIORITY_CLASS):
                pr.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
            if cores and sorted(pr.cpu_affinity()) != sorted(cores):
                pr.cpu_affinity(cores)
        except Exception:  # noqa: BLE001
            pass


def wait_for_memory(args, say) -> dict:
    """Before a run: wait only while the free RAM is below --min-free-ram-gb (an engine start could exhaust it).  There
    is no CPU / GPU threshold: the campaign runs at low priority and a busy machine only makes a run slower (the
    simulation is lockstep: the results depend on simulation time, not on wall-clock time)."""
    import psutil

    t0 = time.time()
    while True:
        ram = psutil.virtual_memory().available / 2**30
        if ram >= args.min_free_ram_gb:
            return {"cpu_percent": round(psutil.cpu_percent(interval=1.0)), "ram_free_gb": round(ram, 1),
                    "waited_s": round(time.time() - t0)}
        say(f"{time.strftime('%H:%M:%S')} waiting for memory: free ram {ram:.1f} GB (min {args.min_free_ram_gb:g})")
        time.sleep(30.0)


def watch(proc, d: Path, dur: float, args, seen: dict):
    """Wait for the worker; kill it on the time cap, on a stall (no new controller log for --stall-min) or when the
    free RAM falls below --critical-ram-gb (the run is then INVALID and can be repeated later).  ``seen`` collects
    the Holodeck.exe started by this worker and the TCP ports they listen on (only ours: parent = our worker)."""
    import psutil

    t0, last_size, last_change = time.time(), -1, time.time()
    cpu_at_change = 0.0
    cap = args.timeout_factor * dur + 1200.0
    while True:
        try:
            return proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            pass
        records = worker_engines(proc.pid, seen.get("worker_created"))
        seen.setdefault("engine_records", {}).update({r["pid"]: r for r in records})
        lower_children(proc.pid, seen.get("worker_created"), records)
        for record in records:
            pid = record["pid"]
            engine_process = owned_engine(record)
            if engine_process is not None:
                seen.setdefault("holodeck_pids", set()).add(pid)
                try:
                    seen.setdefault("listen_ports", set()).update(
                        c.laddr.port for c in engine_process.net_connections(kind="inet") if c.status == "LISTEN")
                except Exception:  # noqa: BLE001
                    pass
        st = d / "drone_0_state.jsonl"
        size = st.stat().st_size if st.exists() else 0
        cpu = _tree_cpu(proc.pid, seen.get("holodeck_pids", ()))
        if size != last_size:
            last_size, last_change, cpu_at_change = size, time.time(), cpu
        reason = None
        # a worker starved by other (higher-priority) jobs makes no progress without being stuck: it is stalled only
        # if it (and its engine) got at least a minute of CPU since the last log growth; the time cap counts CPU too
        if time.time() - t0 > cap and cpu > 0.25 * cap:
            reason = "timeout"
        elif time.time() - last_change > 60.0 * args.stall_min and cpu - cpu_at_change > 60.0:
            reason = "stalled"
        elif psutil.virtual_memory().available / 2**30 < args.critical_ram_gb:
            reason = "low_memory"
        if reason:
            proc.kill()
            return reason


def _tree_cpu(worker_pid: int, engine_pids) -> float:
    """CPU seconds used so far by the worker and the Holodeck.exe it started."""
    import psutil

    tot = 0.0
    for pid in [worker_pid, *engine_pids]:
        try:
            ct = psutil.Process(pid).cpu_times()
            tot += ct.user + ct.system
        except Exception:  # noqa: BLE001
            pass
    return tot


def code_version() -> dict:
    """Commit of the code that produced the runs (official results must all come from one clean commit)."""
    def git(*a):
        r = subprocess.run(["git", *a], cwd=str(ROOT), capture_output=True, text=True)
        return r.stdout.rstrip("\n")

    dirty = [ln[3:] for ln in git("status", "--porcelain", "--untracked-files=all").splitlines() if ln.strip()]
    return {"sha": git("rev-parse", "HEAD").strip(), "dirty_files": dirty}


def engine_build() -> dict:
    """The engine every run of this campaign uses: the patched HoloOcean root (holoocean_setup), its PATCH_INFO and the
    sha256 of its binaries."""
    import hashlib

    from holo_fleet.sim import holoocean_setup as hs

    root = hs.patched_root()
    if root is None:
        return {"patched_root": None}
    info = json.loads((root / "PATCH_INFO.json").read_text(encoding="utf-8-sig"))
    out = {"patched_root": str(root), "patch_info": info, "binaries": {}}
    world = root / str(info.get("holoocean", "2.3.0")) / "worlds" / "Ocean" / "Windows"
    for exe in (world / "Holodeck.exe", world / "Holodeck" / "Binaries" / "Win64" / "Holodeck.exe"):
        if exe.exists():
            h = hashlib.sha256()
            with open(exe, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            out["binaries"][str(exe.relative_to(world))] = h.hexdigest()
    return out


def config_hash(d: Path) -> Optional[str]:
    """sha256 of the fleet configuration a run used (run_config.json, keys sorted)."""
    import hashlib

    try:
        rc = json.loads((d / "run_config.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    blob = json.dumps({"fleet_config": rc.get("fleet_config"), "noise": rc.get("noise"), "cfg_patch": rc.get("cfg_patch")},
                      sort_keys=True, default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def claim(d: Path) -> Optional[Path]:
    """Exclusive claim of a run directory (``<run>.lock`` next to it, created atomically with the orchestrator pid);
    None if a live orchestrator holds it.  A stale claim (its process is gone) is taken over."""
    import psutil

    d.parent.mkdir(parents=True, exist_ok=True)
    lock = d.parent / f"{d.name}.lock"
    for _ in range(2):
        try:
            fd = os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, str(os.getpid()).encode())
            os.close(fd)
            return lock
        except FileExistsError:
            try:
                pid = int(lock.read_text().strip() or "0")
            except (OSError, ValueError):
                pid = 0
            if pid and pid != os.getpid() and psutil.pid_exists(pid):
                return None
            try:
                lock.unlink()
            except OSError:
                return None
    return None


def pending(raw: Path, runs, sha: str = "") -> list:
    """Runs still to do: no run_metrics.json yet, or an INVALID one (infrastructure: engine start, killed worker,
    missing logs...), which is tried again.  COMPLETE runs, scientific failures included, are kept as they are."""
    out = []
    for r in runs:
        p = run_dir(raw, *r) / "run_metrics.json"
        try:
            m = json.loads(p.read_text(encoding="utf-8"))
            ok = m.get("class") not in (None, "INVALID")
            if ok and sha and (m.get("code_version") or {}).get("sha") not in (None, sha):
                raise SystemExit(f"refused: {p} was produced by commit {(m.get('code_version') or {}).get('sha')}, "
                                 f"not {sha}; a campaign is never resumed over another commit's runs")
        except (OSError, ValueError):
            ok = False
        if not ok:
            out.append(r)
    return out


def engine_info(d: Path) -> dict:
    """HoloOcean engine identity of a run (from run_config.json): the IPC namespace (UUID of the shared memory and
    semaphores), the engine pid and the TCP ports it listens on (Unreal TraceLog control, not used by HoloOcean)."""
    try:
        return json.loads((d / "run_config.json").read_text(encoding="utf-8"))["setup"].get("engine")
    except (OSError, ValueError, KeyError):
        return None


def finish(d: Path, profile: str) -> dict:
    from holo_fleet.referee.robustness_metrics import analyze_run

    res = analyze_run(d, profile=profile)
    for f in d.glob("drone_*_state.jsonl"):
        with open(f, "rb") as src, gzip.open(str(f) + ".gz", "wb", compresslevel=6) as dst:
            shutil.copyfileobj(src, dst)
        f.unlink()
    fr = d / "frames"
    if fr.exists() and not any(fr.iterdir()):
        fr.rmdir()
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="final", choices=("final", "targeted"))
    ap.add_argument("--raw", default=str(RAW))
    ap.add_argument("--seeds", type=int, nargs="*", default=list(SEEDS))
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--profiles", nargs="*", help="profile labels (e.g. CLEAN MEDIUM-ALL)")
    ap.add_argument("--protocols", nargs="*", help="--phase targeted: protocols (e.g. B0 B2_CLEAN)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--max-runs", type=int, default=0)
    # parallel campaign: N orchestrators on disjoint slices of the same pending list, one engine each (HoloOcean's
    # client / engine channel is shared memory + semaphores scoped by a per-instance UUID: instances cannot cross)
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--nshards", type=int, default=1)
    ap.add_argument("--cpus", default="", help="logical cores for this orchestrator and its children, e.g. 0-19")
    ap.add_argument("--retries", type=int, default=1)
    # shared-machine courtesy (other heavy jobs have priority): see be_nice / wait_for_room / watch
    ap.add_argument("--min-free-ram-gb", type=float, default=8.0)
    ap.add_argument("--critical-ram-gb", type=float, default=4.0)
    ap.add_argument("--cooldown-s", type=float, default=5.0)
    ap.add_argument("--timeout-factor", type=float, default=20.0)
    ap.add_argument("--stall-min", type=float, default=20.0)
    # worker mode
    ap.add_argument("--worker", action="store_true")
    ap.add_argument("--scenario")
    ap.add_argument("--profile")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--protocol", default="B0")
    ap.add_argument("--run-dir")
    ap.add_argument("--terminal", action="store_true", help="end on SUCCESS / FAILURE / DEADLINE (referee/terminal.py)")
    ap.add_argument("--allow-unpatched", action="store_true", help="development smoke only, never for results")
    args = ap.parse_args()
    if args.worker:
        return worker(args)

    from holo_fleet.sim.scenarios import SCENARIOS

    from holo_fleet.sim import holoocean_setup as hs

    if hs.patched_root() is None and not args.allow_unpatched and not args.dry_run:
        print(f"patched HoloOcean root not found ({hs.DEFAULT_PATCHED_ROOT}): campaign not started", flush=True)
        return 4
    raw = Path(args.raw)
    runs = plan(args.phase, args.seeds, args.only, args.protocols, args.profiles)
    todo = pending(raw, runs, "")
    print(f"phase {args.phase}: {len(runs)} runs, {len(runs) - len(todo)} done, {len(todo)} to do", flush=True)
    if args.nshards > 1:
        # slice the PLAN, not what is still pending: orchestrators started at different times then still own disjoint
        # runs (slicing the pending list made two shards run the same run into the same directory)
        mine = set(runs[args.shard::args.nshards])
        todo = [r for r in todo if r in mine]
        print(f"shard {args.shard}/{args.nshards}: {len(todo)} runs", flush=True)
    if args.dry_run:
        for r in todo:
            print(" ", *r)
        return 0
    raw.mkdir(parents=True, exist_ok=True)
    be_nice(parse_cpus(args.cpus))
    log = open(raw / "campaign_log.txt", "a", encoding="utf-8")

    def say(msg: str) -> None:
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    done = 0
    version = code_version()
    if args.phase == "final" and version["dirty_files"]:
        say(f"refused: the final campaign runs only from a clean commit; modified: {version['dirty_files']}")
        return 2
    build = engine_build()
    say(f"{time.strftime('%H:%M:%S')} phase {args.phase} start: orchestrator pid {os.getpid()}, shard {args.shard}/{args.nshards}, code {version}, engine {build}")
    for (protocol, prof, sc, seed) in todo:
        if args.max_runs and done >= args.max_runs:
            break
        d = run_dir(raw, protocol, prof, sc, seed)
        lock = claim(d)
        if lock is None:
            say(f"{time.strftime('%H:%M:%S')} {protocol} {prof} {sc} s{seed}: claimed by another orchestrator, skipped")
            continue
        if not pending(raw, [(protocol, prof, sc, seed)], version["sha"]):
            # completed by another orchestrator since this one listed its runs: never run (and archive) it again
            lock.unlink(missing_ok=True)
            say(f"{time.strftime('%H:%M:%S')} {protocol} {prof} {sc} s{seed}: already done by another orchestrator, skipped")
            continue
        from holo_fleet.config import DEFAULT
        from holo_fleet.referee.criteria import deadline_v3

        from holo_fleet.referee.criteria import horizon_v5

        dur = horizon_v5(SCENARIOS[sc](DEFAULT), DEFAULT, prof) * float(os.environ.get("HOLO_FLEET_HORIZON_FACTOR", "1"))
        res, attempt = None, 0
        while attempt <= args.retries:
            attempt += 1
            archived = archive_attempt(d, raw)
            if archived is not None and res is not None:
                (archived / "attempt_metrics.json").write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
            d.parent.mkdir(parents=True, exist_ok=True)
            host = wait_for_memory(args, say)
            before = holodeck_pids()
            t0 = time.time()
            seen: dict = {}
            cmd = [PY, str(Path(__file__).resolve()), "--worker", "--scenario", sc, "--profile", prof, "--seed",
                   str(seed), "--protocol", protocol, "--run-dir", str(d)] + (["--allow-unpatched"] if args.allow_unpatched else [])                 + ["--terminal"]
            with open(d.parent / f"{d.name}_worker.log", "w", encoding="utf-8") as wl:
                proc = subprocess.Popen(cmd, stdout=wl, stderr=subprocess.STDOUT, cwd=str(ROOT),
                                        env={**os.environ, "PYTHONUNBUFFERED": "1", "HOLO_FLEET_SLOT_WAIT_S": os.environ.get("HOLO_FLEET_SLOT_WAIT_S", "900")})
                import psutil
                try:
                    seen["worker_created"] = psutil.Process(proc.pid).create_time()
                except psutil.NoSuchProcess:
                    seen["worker_created"] = None
                code = watch(proc, d, dur, args, seen)
            killed = reap(proc.pid, before, seen["worker_created"], seen.get("engine_records", {}).values())
            wall = round(time.time() - t0, 1)
            res = finish(d, prof) if (d / "run_status.json").exists() else {"class": "INVALID", "invalid_reason": "no run dir"}
            res.update({"protocol": protocol, "profile": prof, "scenario": sc, "seed": seed, "exit_code": code,
                        "wall_s": wall, "attempt": attempt, "reaped_holodeck": killed, "host_load_at_start": host,
                        "code_version": version, "engine_build": build, "config_sha256": config_hash(d),
                        "process": {"orchestrator_pid": os.getpid(), "shard": [args.shard, args.nshards], "worker_pid": proc.pid,
                                    "holodeck_pids": sorted(seen.get("holodeck_pids", ())),
                                    "engine_listen_ports": sorted(seen.get("listen_ports", ())),
                                    "engine_records": list(seen.get("engine_records", {}).values()),
                                    "worker_created": seen["worker_created"],
                                    "start_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0))},
                        "engine": engine_info(d)})
            if isinstance(code, str):
                res.setdefault("invalid_reason", code)
            if res["class"] != "INVALID":
                break
        d.mkdir(parents=True, exist_ok=True)              # a worker that refused to start leaves no run dir
        (d / "run_metrics.json").write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
        wl_path = d.parent / f"{d.name}_worker.log"
        if wl_path.exists():
            try:
                shutil.move(str(wl_path), str(d / "worker.log"))
            except OSError:                               # still held by a dying process: copy it, leave the original
                shutil.copy(str(wl_path), str(d / "worker.log"))
        try:
            lock.unlink()                                 # the run is finished (its run_metrics.json is written)
        except OSError:
            pass
        done += 1
        line = (f"{time.strftime('%H:%M:%S')} {protocol} {prof:22s} {sc:32s} s{seed} {res['class']:16s} "
                f"P1 {res.get('P1', {}).get('min_true_separation_m')} fail {res.get('failed_requirements')} "
                f"wall {wall}s exit {code}")
        say(line)
        if args.cooldown_s:
            time.sleep(args.cooldown_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
