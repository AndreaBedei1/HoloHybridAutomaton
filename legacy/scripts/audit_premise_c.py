"""Re-audit of premise C and of every physical entry into the CR, from the logs of finished runs (no simulator).

    python scripts/audit_premise_c.py <out.json> <raw dir> [<raw dir> ...]

From the logs and the implemented TDMA / channel timing (holo_fleet/referee/premise_audit.py): per run the pairs (claim,
other drone) with the premise C class (SATISFIED / CONCURRENT / VIOLATED / INCONCLUSIVE) and the two separate verdicts
(``knowledge``: claim deliverability at the commit instants; ``blocking``: observable behaviour), and the entries of the true
positions into the CR box (ADMITTED, PUSHED, UNCOMMITTED, with the occupancy).  The summary is printed per protocol; the JSON
keeps every pair and entry and the sha256 of every input file read (the raw logs are only read).
"""

import csv
import gzip
import hashlib
import json
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _open(p: Path):
    gz = p.with_name(p.name + ".gz")
    return gzip.open(gz, "rt", encoding="utf-8") if gz.exists() else open(p, encoding="utf-8")


def _inputs(p: Path, n: int) -> dict:
    """sha256 of the files of a run the audit reads."""
    names = ["run_metrics.json", "run_config.json", "referee_timeseries.csv"]
    for k in range(n):
        f = p / f"drone_{k}_state.jsonl"
        names.append(f.name + ".gz" if f.with_name(f.name + ".gz").exists() else f.name)
    return {nm: hashlib.sha256((p / nm).read_bytes()).hexdigest() for nm in names}


def one(d: str) -> dict:
    import numpy as np

    from holo_fleet.config import DEFAULT
    from holo_fleet.referee.premise_audit import audit, beacon_of_run
    from holo_fleet.sim.scenarios import SCENARIOS

    try:
        import psutil

        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass
    p = Path(d)
    m = json.loads((p / "run_metrics.json").read_text(encoding="utf-8"))
    rc = json.loads((p / "run_config.json").read_text(encoding="utf-8"))
    n = int(rc["n_drones"])
    names = [f"drone_{k}" for k in range(n)]
    states = []
    for k in range(n):
        with _open(p / f"drone_{k}_state.jsonl") as f:
            states.append([json.loads(x) for x in f])
    rows = list(csv.DictReader(open(p / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    P = np.stack([np.array([[float(r[f"x{k}"]), float(r[f"y{k}"]), float(r[f"z{k}"])] for r in rows]) for k in range(n)], axis=1)
    mm = min(len(t), min(len(s) for s in states))
    sc = SCENARIOS[rc["scenario"]](DEFAULT)
    from holo_fleet.runner import beacon_identities

    ids = {names.index(q.drone_id): beacon_identities(sc.plans, names, q) for q in sc.plans}
    a = audit(names, t[:mm], P[:mm], [s[:mm] for s in states], sc.judged_gates, DEFAULT, beacon_of_run(rc), identities=ids)
    return {"run": p.as_posix(), "protocol": m.get("protocol"), "profile": m.get("profile"), "scenario": m.get("scenario"),
            "seed": m.get("seed"), "class": m.get("class"), "P2_holds": (m.get("P2") or {}).get("holds"),
            "counts": a["counts"], "knowledge_counts": a["knowledge_counts"], "blocking_counts": a["blocking_counts"],
            "claim_delivery_s": a["claim_delivery_s"], "link": a["link"],
            "commit_rows": sum(1 for x in a["pairs"] if x.get("t_commit") is not None),
            "released_at_commit_sample": sum(1 for x in a["pairs"] if x.get("t_commit") is not None
                                             and x.get("latch_before_commit") not in (None, "FREE")),
            "satisfied_blocked": sum(1 for x in a["pairs"] if x["class"] == "SATISFIED" and x.get("basis") == "BLOCKED"),
            "satisfied_exit_witnessed": sum(1 for x in a["pairs"] if x["class"] == "SATISFIED" and x.get("basis") == "EXIT_WITNESSED"),
            "not_satisfied": [x for x in a["pairs"] if x["class"] != "SATISFIED"], "pairs": a["pairs"],
            "entry_counts": a["entry_counts"], "inputs_sha256": _inputs(p, n),
            "entries_not_admitted": [e for e in a["entries"] if e["kind"] != "ADMITTED"],
            "entries_with_another_inside": a["entries_with_another_inside"]}


def main(argv) -> int:
    runs = sorted(str(q.parent) for r in argv[1:] for q in Path(r).rglob("run_metrics.json"))
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(one, runs, chunksize=4))
    Path(argv[0]).write_text(json.dumps(res, indent=1), encoding="utf-8")
    by = defaultdict(Counter)
    ent = defaultdict(Counter)
    kb = defaultdict(Counter)
    for r in res:
        by[r["protocol"]].update(r["counts"])
        kb[r["protocol"]].update({f"knowledge {k}": v for k, v in r["knowledge_counts"].items() if v})
        kb[r["protocol"]].update({f"blocking {k}": v for k, v in r["blocking_counts"].items() if v})
        ent[r["protocol"]].update(r["entry_counts"])
        ent[r["protocol"]]["with another inside"] += r["entries_with_another_inside"]
        ent[r["protocol"]]["runs"] += 1
        for key in ("commit_rows", "released_at_commit_sample", "satisfied_blocked", "satisfied_exit_witnessed"):
            by[r["protocol"]][key] += r[key]
    for proto in sorted(by, key=str):
        print(f"{proto}: runs {ent[proto]['runs']}; pairs {dict(by[proto])}; {dict(kb[proto])}; CR entries {dict(ent[proto])}")
    tot = Counter()
    for c in by.values():
        tot.update(c)
    print("TOTAL", len(res), "runs; pairs", dict(tot), "; P2 false runs:", sum(not r["P2_holds"] for r in res))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
