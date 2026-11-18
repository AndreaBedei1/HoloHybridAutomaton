"""Offline re-audit of premise C and of every physical entry into the CR, from the logs of finished runs (no simulator).

    python scripts/audit_premise_c.py <out dir> <label>=<raw dir> [<label>=<raw dir> ...]

Every run directory below a raw dir (``_attempts`` excluded: superseded attempts) is read, never written.  The audit is
holo_fleet/referee/premise_audit.py.  The referee verdicts (class, P1, P2) are copied from run_metrics.json, not
recomputed.  Writes to <out dir>:
  runs.csv      one row per run: dataset, commit, engine build, protocol, profile, channel, scenario, seed, referee
                verdicts, pair classes, CR entry kinds, onboard acoustic counts, missing evidence;
  pairs.csv     one row per audited pair (claim episode of a drone, other drone);
  entries.csv   one row per physical CR entry;
  summary.json  totals per dataset and protocol, and the runs with missing evidence.
"""

import csv
import gzip
import json
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

PAIR_COLS = ["dataset", "run", "claimant", "drone", "class", "why", "basis", "resolution", "t_commit_claimant",
             "t_claim_usable", "t_end_usable", "t_claim_end", "t_commit", "release_at_commit", "sonar_exit_witnessed", "claim_emissions",
             "claim_uses_possible_before_commit", "t_admitted_claimant", "t_admitted_drone", "end_claimant", "end_drone", "steps_at_queue", "steps",
             "releases_while_claimed"]
ENTRY_COLS = ["dataset", "run", "gate", "drone", "t", "kind", "others_inside_at_entry", "others_inside_while_in"]


def _open(p: Path):
    gz = p.with_name(p.name + ".gz")
    return gzip.open(gz, "rt", encoding="utf-8") if gz.exists() else open(p, encoding="utf-8")


def one(job) -> dict:
    import numpy as np

    from holo_fleet.config import DEFAULT
    from holo_fleet.referee.premise_audit import audit, beacon_of_run
    from holo_fleet.sim.scenarios import SCENARIOS

    try:
        import psutil

        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass
    label, d = job
    p = Path(d)
    m = json.loads((p / "run_metrics.json").read_text(encoding="utf-8"))
    rc = json.loads((p / "run_config.json").read_text(encoding="utf-8"))
    missing = []
    n = int(rc["n_drones"])
    names = [f"drone_{k}" for k in range(n)]
    states = []
    for k in range(n):
        try:
            with _open(p / f"drone_{k}_state.jsonl") as f:
                states.append([json.loads(x) for x in f])
        except OSError:
            missing.append(f"drone_{k}_state.jsonl")
    rows = list(csv.DictReader(open(p / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    try:
        P = np.stack([np.array([[float(r[f"x{k}"]), float(r[f"y{k}"]), float(r[f"z{k}"])] for r in rows]) for k in range(n)],
                     axis=1)
    except (KeyError, ValueError):
        P, missing = None, missing + ["true positions"]
    b = (rc.get("fleet_config") or {}).get("beacon") or {}
    cv, eb = m.get("code_version") or {}, m.get("engine_build") or {}
    rm = {}
    try:
        rm = json.loads((p / "referee_metrics.json").read_text(encoding="utf-8"))
    except OSError:
        missing.append("referee_metrics.json")
    at = (rm.get("run") or {}).get("acoustic_tracker") or {}
    base = {"dataset": label, "run": p.as_posix(), "commit": cv.get("sha"), "dirty_files": len(cv.get("dirty_files") or []),
            "engine_exe_sha256": (eb.get("patch_info") or {}).get("exe_sha256"), "protocol": m.get("protocol"),
            "profile": m.get("profile"), "channel": b.get("channel") if b.get("mode") == "ACOUSTIC_RB" else None,
            "payload": b.get("payload") if b.get("mode") == "ACOUSTIC_RB" else None, "scenario": m.get("scenario"),
            "seed": m.get("seed"), "n_drones": n, "class": m.get("class"), "P1_holds": (m.get("P1") or {}).get("holds"),
            "P1_min_m": (m.get("P1") or {}).get("min_true_separation_m"), "P2_holds": (m.get("P2") or {}).get("holds"),
            "rx_received": at.get("received"), "rx_accepted": at.get("accepted"), "rx_rejected": at.get("rejected"),
            "rx_stale": at.get("stale")}
    if len(states) < n:
        return {**base, "missing": missing, "pairs": [], "entries": []}
    mm = min(len(t), min(len(s) for s in states))
    sc = SCENARIOS[rc["scenario"]](DEFAULT)
    a = audit(names, t[:mm], None if P is None else P[:mm], [s[:mm] for s in states], sc.judged_gates if P is not None else [],
              DEFAULT, beacon_of_run(rc))
    if a["b2"] and a["link"] is None:
        missing.append("channel latency")
    return {**base, "missing": missing, "link": a["link"], "pairs": a["pairs"], "entries": a["entries"],
            **{f"pc_{c}": a["counts"][c] for c in a["counts"]},
            **{f"cr_{k}": v for k, v in a["entry_counts"].items()}, "cr_with_another_inside": a["entries_with_another_inside"]}


def _jobs(specs):
    out = []
    for s in specs:
        label, _, root = s.partition("=")
        for q in sorted(Path(root).rglob("run_metrics.json")):
            if "_attempts" not in q.parts:
                out.append((label, str(q.parent)))
    return out


def _cell(v):
    return json.dumps(v) if isinstance(v, (list, dict)) else v


def main(argv) -> int:
    out = Path(argv[0])
    out.mkdir(parents=True, exist_ok=True)
    jobs = _jobs(argv[1:])
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(one, jobs, chunksize=4))
    run_cols = [c for c in res[0] if c not in ("pairs", "entries", "link")]
    run_cols += [c for r in res for c in r if c not in run_cols and c not in ("pairs", "entries", "link")]
    run_cols = list(dict.fromkeys(run_cols))
    with open(out / "runs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, run_cols)
        w.writeheader()
        for r in res:
            w.writerow({c: _cell(r.get(c)) for c in run_cols})
    with open(out / "pairs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, PAIR_COLS, extrasaction="ignore")
        w.writeheader()
        for r in res:
            for x in r["pairs"]:
                w.writerow({"dataset": r["dataset"], "run": r["run"], **{c: _cell(x.get(c)) for c in PAIR_COLS[2:]}})
    with open(out / "entries.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, ENTRY_COLS, extrasaction="ignore")
        w.writeheader()
        for r in res:
            for e in r["entries"]:
                w.writerow({"dataset": r["dataset"], "run": r["run"], **{c: _cell(e.get(c)) for c in ENTRY_COLS[2:]}})
    tot = defaultdict(lambda: defaultdict(Counter))
    for r in res:
        for key in (r["dataset"], f"{r['dataset']}|{r['protocol']}"):
            c = tot[key]
            c["runs"]["n"] += 1
            c["pairs"].update(x["class"] for x in r["pairs"])
            c["why"].update(f"{x['class']}: {x.get('why') or x.get('basis') or x.get('resolution')}" for x in r["pairs"])
            c["entries"].update(e["kind"] for e in r["entries"])
            c["entries"]["with_another_inside"] += r.get("cr_with_another_inside") or 0
            c["referee"].update(str(r["class"]) for _ in [0])
            c["P1_P2"]["P1_false"] += int(r["P1_holds"] is False)
            c["P1_P2"]["P2_false"] += int(r["P2_holds"] is False)
    summary = {"runs": len(res), "datasets": sorted({r["dataset"] for r in res}),
               "commits": {d: sorted({str(r["commit"]) for r in res if r["dataset"] == d}) for d in {r["dataset"] for r in res}},
               "engine_builds": sorted({str(r["engine_exe_sha256"]) for r in res}),
               "totals": {k: {kk: dict(vv) for kk, vv in v.items()} for k, v in sorted(tot.items())},
               "missing_evidence": [{"run": r["run"], "missing": r["missing"]} for r in res if r["missing"]]}
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    for k in sorted(tot):
        if "|" not in k:
            print(k, json.dumps({kk: dict(vv) for kk, vv in tot[k].items() if kk != "why"}))
    print("missing evidence:", len(summary["missing_evidence"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
