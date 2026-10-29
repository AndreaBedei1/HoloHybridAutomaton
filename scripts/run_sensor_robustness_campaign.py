"""Sensor-robustness and beacon-ablation campaign (headless, serial HoloOcean, resumable).

    python scripts/run_sensor_robustness_campaign.py --phase isolation|main|high|beacon [--dry-run]
    python scripts/run_sensor_robustness_campaign.py --phase main --only gate_single --seeds 0
    python scripts/run_sensor_robustness_campaign.py --phase isolation --only p1_vertical_escape --seeds 0

Every run executes in its own Python process (holo_fleet.runner.run, no frames, no GIF) and writes a run
directory under the raw root (default results/robustness/raw/, git-ignored):
    <protocol>/<profile>/<scenario>/s<seed>/
followed by run_metrics.json (holo_fleet.referee.robustness_metrics: metrics + class).  A run whose
run_metrics.json exists is skipped, so an interrupted campaign resumes where it stopped.  Bulky per-step logs
(drone_*_state.jsonl) are gzip-compressed after the analysis.  One HoloOcean instance at a time; after every run
the Holodeck.exe processes started by that run (and only those) are checked and, if still alive, terminated.

Phases (same seeds 0, 1, 2 everywhere; protocol B0 = communication-free baseline, B1 = one-way presence beacon):
* isolation: CLEAN + one phenomenon at its HIGH level (sim/sensor_noise.FACTORS) on ISOLATION_SCENARIOS
             (or on the --only scenarios: diagnosis of a failure found in another scenario);
* main:      the 16 scenarios x CLEAN, LOW, MEDIUM;
* high:      HIGH on HIGH_SCENARIOS (all 16 with --all-high);
* beacon:    B1 on BEACON_SCENARIOS x LOW, MEDIUM, HIGH (B0 comes from main/high);
* validation: the realistic operating domain: VALIDATION_SCENARIOS x (CLEAN, LOW and MEDIUM x ONE / SUBSET / ALL,
             scope_drones) x 3 seeds, each run ending on SUCCESS / FAILURE / DEADLINE (referee/terminal.py).
             main / high / beacon are the exploratory stress campaign (first version of the levels, STRESS_V1).
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

RAW = ROOT / "results" / "robustness" / "raw"
DEMOS = ROOT / "results" / "v2" / "demos"
SEEDS = (0, 1, 2)
ALL_SCENARIOS = ("p1_head_on", "p1_vertical_escape", "p1_two_lines", "p1_close_encounter", "formation_triangle",
                 "formation_square", "formation_six", "formation_recovery_head_current", "formation_gust", "gate_single",
                 "integrated_short", "lost_drone_rejoin", "lost_drone_timeout", "mutex_deadlock_resolution",
                 "line_parallel_mutex", "lost_drone_mutex")
ISOLATION_SCENARIOS = ("p1_close_encounter", "line_parallel_mutex", "lost_drone_rejoin")
HIGH_SCENARIOS = ("p1_head_on", "p1_vertical_escape", "p1_two_lines", "p1_close_encounter", "formation_six",
                  "formation_recovery_head_current", "formation_gust", "gate_single", "integrated_short",
                  "mutex_deadlock_resolution", "line_parallel_mutex", "lost_drone_rejoin", "lost_drone_timeout",
                  "lost_drone_mutex")
BEACON_SCENARIOS = ("formation_six", "gate_single", "integrated_short", "mutex_deadlock_resolution",
                    "line_parallel_mutex", "lost_drone_rejoin", "lost_drone_timeout", "lost_drone_mutex")
# validation of the realistic operating domain (LOW / MEDIUM, sim/sensor_noise.py): representative scenarios (P1
# collision avoidance, P2 gate / mutex, P3 formation and recovery, lost drones, integrated) x scope x 3 seeds
VALIDATION_SCENARIOS = ("p1_vertical_escape", "p1_two_lines", "formation_six", "formation_recovery_head_current",
                        "integrated_short", "line_parallel_mutex", "lost_drone_rejoin", "lost_drone_mutex")
# targeted check of the gate localization (relocalization hold, robust clearance) on the gate scenarios
GATE_CHECK_SCENARIOS = ("line_parallel_mutex", "lost_drone_mutex", "gate_single", "integrated_short")
# variant B2 (range-bearing beacon; protocol label B2_<channel level>, sim/acoustic_channel.py)
GATE5_SCENARIOS = ("gate_single", "integrated_short", "mutex_deadlock_resolution", "line_parallel_mutex",
                   "lost_drone_mutex")
FORMATION_B2_SCENARIOS = ("formation_six", "formation_recovery_head_current", "lost_drone_rejoin", "lost_drone_timeout",
                          "p1_vertical_escape")
TERMINAL_PHASES = ("validation", "gate_check", "b2_gate", "b2_formation", "b2_degraded", "b2_final")
PY = sys.executable


def clean_completion(protocol_root: Path, scenario: str):
    """T_clean: the slowest completion (SUCCESS time) of the CLEAN runs of ``scenario`` (None if none succeeded)."""
    ts = []
    for p in sorted((protocol_root / "CLEAN" / scenario).glob("s*/run_metrics.json")):
        term = json.loads(p.read_text(encoding="utf-8")).get("terminal") or {}
        if term.get("outcome") == "SUCCESS":
            ts.append(float(term["t_end"]))
    return max(ts) if ts else None


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
    rows = list(csv.DictReader(open(DEMOS / scenario / "referee_timeseries.csv", encoding="utf-8")))
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


def plan(phase: str, seeds, only=None, all_high=False):
    from holo_fleet.sim.sensor_noise import FACTORS

    runs = []
    if phase == "isolation":
        for f in FACTORS:
            for sc in (tuple(only) if only else ISOLATION_SCENARIOS):
                runs += [("B0", f"ISO_{f}", sc, s) for s in seeds]
    elif phase == "main":
        for prof in ("CLEAN", "LOW", "MEDIUM"):
            for sc in ALL_SCENARIOS:
                runs += [("B0", prof, sc, s) for s in seeds]
    elif phase == "high":
        for sc in (ALL_SCENARIOS if all_high else HIGH_SCENARIOS):
            runs += [("B0", "HIGH", sc, s) for s in seeds]
    elif phase == "beacon":
        for prof in ("LOW", "MEDIUM", "HIGH"):
            for sc in BEACON_SCENARIOS:
                runs += [("B1", prof, sc, s) for s in seeds]
    elif phase in ("validation", "gate_check"):
        from holo_fleet.sim.sensor_noise import SCOPES

        scen = tuple(only) if only else VALIDATION_SCENARIOS if phase == "validation" else GATE_CHECK_SCENARIOS
        for sc in scen:                                 # CLEAN first: it fixes the deadline of the degraded runs
            runs += [("B0", "CLEAN", sc, s) for s in seeds]
        for sc in scen:
            for prof in ("LOW", "MEDIUM"):
                for scope in SCOPES:
                    runs += [("B0", f"{prof}-{scope}", sc, s) for s in seeds]
    elif phase == "b2_gate":                        # B0 and B2 on the five gate scenarios, CLEAN sensors and channel
        for sc in (tuple(only) if only else GATE5_SCENARIOS):
            runs += [(proto, "CLEAN", sc, s) for proto in ("B2_CLEAN", "B0") for s in seeds]
    elif phase == "b2_formation":                   # targeted formation checks, CLEAN and MEDIUM-ALL sensors
        for sc in (tuple(only) if only else FORMATION_B2_SCENARIOS):
            for prof in ("CLEAN", "MEDIUM-ALL"):
                runs += [(proto, prof, sc, s) for proto in ("B0", "B2_CLEAN" if prof == "CLEAN" else "B2_MEDIUM")
                         for s in seeds]
    elif phase == "b2_final":                       # the documented matrix (docs/B2_ACOUSTIC_AIDED_COORDINATION.md)
        runs += final_matrix(seeds, only)
    elif phase == "beacon_clean":
        for sc in BEACON_SCENARIOS:
            runs += [("B1", "CLEAN", sc, s) for s in seeds]
    else:
        raise SystemExit(f"unknown phase {phase}")
    if only:
        runs = [r for r in runs if r[2] in only]
    return runs


def final_matrix(seeds, only=None) -> list:
    """B0 vs B2 on the same scenarios and seeds (docs/B2_ACOUSTIC_AIDED_COORDINATION.md, section 7), reduced matrix
    fixed before any B2 run with degraded sensors or channel:
    * sensors CLEAN, LOW-ALL and MEDIUM-{ONE,SUBSET,ALL}: B0, and B2 with the channel at the sensors' level (LOW-ONE
      and LOW-SUBSET are dropped: LOW-ALL is the most degraded LOW case and the LOW scopes are characterised for B0
      in the sensor-robustness campaign);
    * the channel alone: sensors CLEAN with the channel MEDIUM and BLACKOUT (LOW lies between CLEAN and MEDIUM);
    * channel BLACKOUT with MEDIUM-ALL sensors (B2 must fall back to B0).
    Runs of the b2_gate and b2_formation phases in the same raw root are reused (same code, same cells)."""
    scen = tuple(only) if only else FINAL_SCENARIOS
    runs = []
    for sc in scen:                                 # CLEAN first: B0 CLEAN fixes the deadline of every degraded run
        runs += [(proto, "CLEAN", sc, s) for proto in ("B0", "B2_CLEAN") for s in seeds]
    for sc in scen:
        for prof, lvl in (("LOW-ALL", "LOW"), ("MEDIUM-ONE", "MEDIUM"), ("MEDIUM-SUBSET", "MEDIUM"), ("MEDIUM-ALL", "MEDIUM")):
            runs += [(proto, prof, sc, s) for proto in ("B0", f"B2_{lvl}") for s in seeds]
        runs += [(f"B2_{ch}", "CLEAN", sc, s) for ch in ("MEDIUM", "BLACKOUT") for s in seeds]
        runs += [("B2_BLACKOUT", "MEDIUM-ALL", sc, s) for s in seeds]
    return runs


FINAL_SCENARIOS = ("p1_vertical_escape", "p1_two_lines", "formation_six", "formation_recovery_head_current",
                   "lost_drone_rejoin", "lost_drone_timeout") + GATE5_SCENARIOS


def run_dir(raw: Path, protocol: str, prof: str, sc: str, seed: int) -> Path:
    return raw / protocol / prof / sc / f"s{seed}"


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
    if args.protocol == "B1":
        cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_PRESENCE"))
    elif args.protocol.startswith("B2_"):            # range-bearing beacon, channel level after the underscore
        cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB", channel=args.protocol[3:]))
    d = Path(args.run_dir)
    prof, _, scope = args.profile.partition("-")
    terminal = None
    if args.terminal:
        from holo_fleet.referee.robustness_metrics import load_reference

        terminal = {"reference": load_reference(DEMOS / args.scenario)}
        if prof != "CLEAN":
            # the deadline of every protocol comes from the B0 CLEAN runs (one success criterion for B0 and B2)
            t_clean = clean_completion(Path(args.run_dir).parents[3] / "B0", args.scenario)
            if t_clean is None:
                print(f"no successful CLEAN run of {args.scenario} under {Path(args.run_dir).parents[2]}: run CLEAN "
                      "first (it fixes the deadline)", file=sys.stderr)
                return 5
            from holo_fleet.referee.terminal import deadline_s

            terminal["deadline_s"] = deadline_s(t_clean)
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


def reap(worker_pid: int, before: dict) -> list:
    """Terminate Holodeck.exe processes started by our worker (never anyone else's)."""
    import psutil

    killed = []
    for pid, ppid in holodeck_pids().items():
        if pid in before or ppid != worker_pid:
            continue
        try:
            psutil.Process(pid).kill()
            killed.append(pid)
        except Exception:  # noqa: BLE001
            pass
    return killed


def be_nice() -> None:
    """Shared machine: the campaign and every process it starts (they inherit it) yield CPU and disk to other
    work: below-normal priority class and low I/O priority on Windows, nice 10 elsewhere."""
    import psutil

    try:
        me = psutil.Process()
        me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS if os.name == "nt" else 10)
        if os.name == "nt":
            me.ionice(psutil.IOPRIO_LOW)
    except Exception:  # noqa: BLE001 - courtesy only
        pass


def lower_children(worker_pid: int) -> None:
    """Make sure the worker and the Holodeck.exe it started run below normal priority (in case it was not inherited)."""
    import psutil

    for pid in [worker_pid] + [p for p, pp in holodeck_pids().items() if pp == worker_pid]:
        try:
            pr = psutil.Process(pid)
            if os.name == "nt" and pr.nice() not in (psutil.BELOW_NORMAL_PRIORITY_CLASS, psutil.IDLE_PRIORITY_CLASS):
                pr.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
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
        lower_children(proc.pid)
        for pid, ppid in holodeck_pids().items():
            if ppid == proc.pid:
                seen.setdefault("holodeck_pids", set()).add(pid)
                try:
                    seen.setdefault("listen_ports", set()).update(
                        c.laddr.port for c in psutil.Process(pid).net_connections(kind="inet") if c.status == "LISTEN")
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

    dirty = [ln[3:] for ln in git("status", "--porcelain", "--untracked-files=no").splitlines() if ln.strip()]
    return {"sha": git("rev-parse", "HEAD").strip(), "dirty_files": dirty}


def pending(raw: Path, runs) -> list:
    """Runs still to do: no run_metrics.json yet, or an INVALID one (infrastructure: engine start, killed worker,
    missing logs...), which is tried again.  COMPLETE runs, scientific failures included, are kept as they are."""
    out = []
    for r in runs:
        p = run_dir(raw, *r) / "run_metrics.json"
        try:
            ok = json.loads(p.read_text(encoding="utf-8")).get("class") not in (None, "INVALID")
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


def finish(d: Path, reference) -> dict:
    from holo_fleet.referee.robustness_metrics import analyze_run

    res = analyze_run(d, reference)
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
    ap.add_argument("--phase", default="main")
    ap.add_argument("--raw", default=str(RAW))
    ap.add_argument("--seeds", type=int, nargs="*", default=list(SEEDS))
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--profiles", nargs="*", help="keep only these profile labels (e.g. CLEAN MEDIUM-ALL)")
    ap.add_argument("--all-high", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--max-runs", type=int, default=0)
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

    from holo_fleet.referee.robustness_metrics import load_reference
    from holo_fleet.sim.scenarios import SCENARIOS

    from holo_fleet.sim import holoocean_setup as hs

    if hs.patched_root() is None and not args.allow_unpatched and not args.dry_run:
        print(f"patched HoloOcean root not found ({hs.DEFAULT_PATCHED_ROOT}): campaign not started", flush=True)
        return 4
    raw = Path(args.raw)
    runs = plan(args.phase, args.seeds, args.only, args.all_high)
    if args.profiles:
        runs = [r for r in runs if r[1] in args.profiles]
    todo = pending(raw, runs)
    print(f"phase {args.phase}: {len(runs)} runs, {len(runs) - len(todo)} done, {len(todo)} to do", flush=True)
    if args.dry_run:
        for r in todo:
            print(" ", *r)
        return 0
    raw.mkdir(parents=True, exist_ok=True)
    be_nice()
    log = open(raw / "campaign_log.txt", "a", encoding="utf-8")

    def say(msg: str) -> None:
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    refs = {}
    done = 0
    version = code_version()
    say(f"{time.strftime('%H:%M:%S')} phase {args.phase} start: orchestrator pid {os.getpid()}, code {version}")
    for (protocol, prof, sc, seed) in todo:
        if args.max_runs and done >= args.max_runs:
            break
        d = run_dir(raw, protocol, prof, sc, seed)
        if sc not in refs:
            refs[sc] = load_reference(DEMOS / sc)
        dur = SCENARIOS[sc](__import__("holo_fleet.config", fromlist=["DEFAULT"]).DEFAULT).duration_s
        if args.phase in TERMINAL_PHASES:
            from holo_fleet.referee.terminal import clean_horizon_s

            dur = clean_horizon_s(dur)
        res, attempt = None, 0
        while attempt <= args.retries:
            attempt += 1
            if d.exists():
                shutil.rmtree(d)
            d.parent.mkdir(parents=True, exist_ok=True)
            host = wait_for_memory(args, say)
            before = holodeck_pids()
            t0 = time.time()
            seen: dict = {}
            cmd = [PY, str(Path(__file__).resolve()), "--worker", "--scenario", sc, "--profile", prof, "--seed",
                   str(seed), "--protocol", protocol, "--run-dir", str(d)] + (["--allow-unpatched"] if args.allow_unpatched else [])                 + (["--terminal"] if args.phase in TERMINAL_PHASES else [])
            with open(d.parent / f"{d.name}_worker.log", "w", encoding="utf-8") as wl:
                proc = subprocess.Popen(cmd, stdout=wl, stderr=subprocess.STDOUT, cwd=str(ROOT),
                                        env={**os.environ, "PYTHONUNBUFFERED": "1"})
                code = watch(proc, d, dur, args, seen)
            killed = reap(proc.pid, before)
            wall = round(time.time() - t0, 1)
            res = finish(d, refs[sc]) if (d / "run_status.json").exists() else {"class": "INVALID", "invalid_reason": "no run dir"}
            res.update({"protocol": protocol, "profile": prof, "scenario": sc, "seed": seed, "exit_code": code,
                        "wall_s": wall, "attempt": attempt, "reaped_holodeck": killed, "host_load_at_start": host,
                        "code_version": version,
                        "process": {"orchestrator_pid": os.getpid(), "worker_pid": proc.pid,
                                    "holodeck_pids": sorted(seen.get("holodeck_pids", ())),
                                    "engine_listen_ports": sorted(seen.get("listen_ports", ())),
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
