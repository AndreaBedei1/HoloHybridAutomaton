"""Run the fixed paired matrix and derive an auditable compact record from the raw outputs."""

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import gzip
from hashlib import sha256
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "config/publication_validation.json"


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    h = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


# Paths that never run in a campaign: a pair recorded at two revisions differing only here ran the same code.
NON_RUNTIME = ("formal", "docs", "README.md")


def same_runtime(first, second, root=ROOT):
    """True when two recorded revisions are equal or differ only outside the runtime code.

    Anything but two full hexadecimal SHAs, including an option-shaped value, and any git error fail closed.
    """
    if not all(isinstance(r, str) and re.fullmatch(r"[0-9a-f]{40}", r) for r in (first, second)):
        return False
    if first == second:
        return True
    excluded = [f":(exclude){path}" for path in NON_RUNTIME]
    code = subprocess.run(["git", "diff", "--quiet", first, second, "--", ".", *excluded], cwd=root,
                          capture_output=True).returncode
    return code == 0


def validate_isolation(path, workers=2):
    """The isolation record must show ``workers`` live engines, each with its own client, root and non-default ports."""
    proof = load(path)
    rows = proof["instances"]
    ports = [p for r in rows for p in r["listen_ports"]]
    if (not proof["all_passed"] or len(rows) != workers or not all(r["checks_passed"] for r in rows)
            or len({r["client_uuid"] for r in rows}) != workers or len({r["root"] for r in rows}) != workers
            or not all(r["listen_ports"] and 1985 not in r["listen_ports"] for r in rows)
            or len(ports) != len(set(ports))):
        raise ValueError(f"{workers}-engine isolation check failed")


def execute(raw, matrix, workers):
    def lane(shard):
        for case in matrix["cases"]:
            cmd = [sys.executable, str(ROOT / "scripts/run_sensor_robustness_campaign.py"),
                   "--only", case["scenario"], "--profiles", case["profile"], "--protocols", "B0", case["b2"],
                   "--duration-s", str(case["duration_s"]), "--seeds", *map(str, matrix["seeds"]),
                   "--raw", str(raw / case["id"]), "--nshards", str(workers), "--shard", str(shard),
                   "--retries", "0", "--require-clean"]
            if case.get("inject"):
                cmd.append("--inject-commitments")
            # A failed case is recorded (its rows stay MISSING or invalid in the summary); the lane goes on.
            code = subprocess.run(cmd, cwd=ROOT).returncode
            if code:
                failures.append({"case": case["id"], "shard": shard, "exit": code})

    failures = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        list(pool.map(lane, range(workers)))
    (raw / "lane_failures.json").write_text(json.dumps(failures, indent=1) + "\n", encoding="utf-8")


def run_record(raw, case, protocol, seed):
    path = Path(case["id"], protocol, case["profile"], case["scenario"], f"s{seed}")
    d = raw / path
    row = {"case": case["id"], "protocol": protocol, "seed": seed, "path": path.as_posix(),
           "planned_window_s": case["duration_s"]}
    metrics = d / "run_metrics.json"
    if not metrics.exists():
        return dict(row, outcome="MISSING")
    m = load(metrics)
    version = m.get("code_version") or {}
    # A run from a modified tree has no revision that identifies its code, so it is never paired.
    row.update(outcome=m["class"], commit=version.get("sha") if version.get("dirty_files") == [] else None,
               dirty_files=version.get("dirty_files"))
    if not (d / "referee_metrics.json").exists() or "P1" not in m:
        return dict(row, invalid_reason=m.get("invalid_reason"), hashes={"run_metrics.json": digest(metrics)})
    c, r = load(d / "run_config.json"), load(d / "referee_metrics.json")
    safety, mutex, recovery = (r[key] for key in ("P1_separation", "P2_mutual_exclusion", "P3_formation_recovery"))
    events = [json.loads(line) for line in (d / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    intervals, sources, modes = Counter(), Counter(), Counter()
    observations = failures = recoveries = admissions = gate_filter_steps = uncertified_steps = 0
    for trace in sorted(d.glob("drone_*_state.jsonl.gz")):
        last_sense, last_entry = True, None
        with gzip.open(trace, "rt", encoding="utf-8") as stream:
            for line in stream:
                s = json.loads(line)
                sense = bool(s.get("sense_ok"))
                entry = (s.get("gate") or {}).get("entry")
                failures += int(last_sense and not sense)
                recoveries += int(not last_sense and sense)
                admissions += int(entry == "ADMIT" and last_entry != "ADMIT")
                filt = s.get("gate_safety") or {}
                gate_filter_steps += int(bool(filt))
                uncertified_steps += int(bool(filt) and filt.get("cert") is None)
                last_sense, last_entry = sense, entry
                interval = s.get("p1_interval") or {}
                intervals[interval.get("status", "MISSING")] += 1
                if interval.get("source"):
                    sources[interval["source"]] += 1
                modes[s["mode"]] += 1
                observations += 1
    common = {key: value for key, value in c.items() if key not in ("setup", "communication", "fleet_config")}
    common["fleet_config"] = {key: value for key, value in c["fleet_config"].items() if key != "beacon"}
    beacon = r["run"].get("beacon") or {}
    p2 = m.get("P2") or {}
    injected = [e for e in events if e["type"] == "TEST_COMMIT_INJECTION"]
    with (d / "referee_timeseries.csv").open(encoding="utf-8") as series:
        samples = max(0, sum(1 for _ in series) - 1)
    n = c["n_drones"]
    dt = c["fleet_config"]["env"]["dt"]
    row.update(observed_window_s=m["horizon_s"], drones=c["n_drones"], drone_samples=observations,
               referee_samples=samples, pair_samples=samples * n * (n - 1) // 2,
               common_config_sha256=sha256(json.dumps(common, sort_keys=True).encode()).hexdigest(),
               ports=c["setup"]["engine"]["listen_ports"], terminal=m["terminal"]["outcome"],
               min_separation_m=safety["min_distance"], p1_violation_samples=safety["violations"],
               physical_contacts=safety["physical_contacts"], p1_interval_counts=dict(intervals),
               p1_interval_sources=dict(sources), conditional_interval_fraction=(
                   intervals["INTERVAL_CONDITIONAL"] / observations if observations else None),
               p2_max_occupancy=mutex["max_occupancy"] or None, p2_double_occupancy_samples=p2.get("events", {}).get("E5"),
               admissions=admissions, premature_admissions=p2.get("events", {}).get("E4"),
               p2_filter_steps=gate_filter_steps, p2_uncertified_steps=uncertified_steps,
               p2_uncertified_events=sum(e["type"] == "P2_UNCERTIFIED" for e in events),
               crossings=len(p2["passed"]) if "passed" in p2 else None, gate_completion_s=p2.get("completion_time_s"),
               queued_idle_episodes=p2.get("deadlock_events"), longest_queued_idle_s=p2.get("longest_stall_s"),
               committed_stalls=p2.get("events", {}).get("E8", {}).get("committed_stalls"),
               gate_aborts=sum(e.get("edge") == "pass_abort" for e in events if e["type"] == "transition"),
               concessions=sum(e["type"] == "COMMITMENT_CONCEDED" for e in events),
               recovery_episodes=recovery["episodes"] if recovery["enabled"] else None,
               final_formation_error_m=recovery["final_form_err"] if recovery["enabled"] else None,
               mode_samples=dict(modes), mode_drone_seconds={k: round(v * dt, 3) for k, v in modes.items()},
               sense_failures=failures, sense_recoveries=recoveries,
               sensor_drops=r["run"].get("noise_wrapper_stats"),
               beacon={k: beacon[k] for k in ("receptions", "expected_receptions", "latency_mean_ms", "latency_max_ms",
                                              "longest_absence_s") if k in beacon},
               injections=[{"t": e["t"], "drone": e["drone"]} for e in injected],
               determinism_violations=r["run"].get("determinism_violations"),
               hashes={f.name: digest(f) for f in sorted(d.iterdir()) if f.is_file()})
    return row


def summarize(raw, matrix):
    runs = []
    pairs = []
    for case in matrix["cases"]:
        comparisons = []
        for seed in matrix["seeds"]:
            b0, b2 = (run_record(raw, case, protocol, seed) for protocol in ("B0", case["b2"]))
            runs.extend((b0, b2))
            first, second = (r.get("common_config_sha256") for r in (b0, b2))
            delta = None
            if first is not None and first == second and same_runtime(b0.get("commit"), b2.get("commit")):
                delta = {k: round(b2[k] - b0[k], 4) for k in ("min_separation_m", "conditional_interval_fraction",
                         "p2_uncertified_events", "crossings",
                         "gate_completion_s", "final_formation_error_m") if b0[k] is not None and b2[k] is not None}
            comparisons.append({"seed": seed, "revisions": [b0.get("commit"), b2.get("commit")],
                                "same_configuration_and_runtime_code": delta is not None, "B2_minus_B0": delta})
        fields = {k for pair in comparisons for k in (pair["B2_minus_B0"] or {})}
        distributions = {}
        for field in sorted(fields):
            values = [p["B2_minus_B0"][field] for p in comparisons if p["B2_minus_B0"] and field in p["B2_minus_B0"]]
            distributions[field] = {"n": len(values), "median": statistics.median(values),
                                    "range": [min(values), max(values)]}
        pairs.append({"case": case["id"], "pairs": comparisons, "B2_minus_B0": distributions})
    return {"matrix_sha256": digest(MATRIX), "planned_runs": len(runs),
            "outcomes": dict(Counter(r["outcome"] for r in runs)), "runs": runs, "comparison": pairs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--workers", choices=(1, 2, 3, 4), type=int, default=1)
    parser.add_argument("--isolation-record", type=Path)
    args = parser.parse_args()
    matrix = load(MATRIX)
    if args.run:
        import os

        if not os.environ.get("HOLO_FLEET_SLOT_REGISTRY"):
            parser.error("set HOLO_FLEET_SLOT_REGISTRY to the shared engine slot registry")
        if args.workers > 1:
            if args.isolation_record is None:
                parser.error("several workers need a completed isolation record with as many engines")
            validate_isolation(args.isolation_record, args.workers)
        os.environ["HOLO_FLEET_ENGINE_SLOTS"] = str(args.workers)
        execute(args.raw.resolve(), matrix, args.workers)
    result = summarize(args.raw, matrix)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(result["outcomes"])


if __name__ == "__main__":
    main()
