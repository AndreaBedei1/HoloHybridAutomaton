"""Re-audit of premise C and of every physical entry into the CR, from the logs of finished runs (no simulator).

    python scripts/audit_premise_c.py <out.json> <raw dir> [<raw dir> ...]

Event- and ordering-based (holo_fleet/referee/premise_audit.py): per run the pairs (claim, other drone) classified
SATISFIED / CONCURRENT / VIOLATED / INCONCLUSIVE and the entries of the true positions into the CR box (ADMITTED, PUSHED,
UNCOMMITTED, with the occupancy).  The summary is printed per protocol; the JSON keeps every non-satisfied pair and entry.
"""

import csv
import gzip
import json
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _open(p: Path):
    gz = p.with_name(p.name + ".gz")
    return gzip.open(gz, "rt", encoding="utf-8") if gz.exists() else open(p, encoding="utf-8")


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
    a = audit(names, t[:mm], P[:mm], [s[:mm] for s in states], sc.judged_gates, DEFAULT, beacon_of_run(rc))
    return {"run": p.as_posix(), "protocol": m.get("protocol"), "profile": m.get("profile"), "scenario": m.get("scenario"),
            "seed": m.get("seed"), "class": m.get("class"), "P2_holds": (m.get("P2") or {}).get("holds"),
            "counts": a["counts"], "claim_delivery_s": a["claim_delivery_s"],
            "satisfied_blocked": sum(1 for x in a["pairs"] if x["class"] == "SATISFIED" and x.get("t_evidence") is not None),
            "satisfied_never_blocked": sum(1 for x in a["pairs"] if x["class"] == "SATISFIED" and x.get("t_evidence") is None),
            "arrivals_during_claim": sum(1 for x in a["pairs"] if x.get("arrived_during_claim")),
            "releases_while_claimed": sum(int(x.get("releases_while_claimed") or 0) for x in a["pairs"]),
            "not_satisfied": [x for x in a["pairs"] if x["class"] != "SATISFIED"], "entry_counts": a["entry_counts"],
            "entries_not_admitted": [e for e in a["entries"] if e["kind"] != "ADMITTED"],
            "entries_with_another_inside": a["entries_with_another_inside"]}


def main(argv) -> int:
    runs = sorted(str(q.parent) for r in argv[1:] for q in Path(r).rglob("run_metrics.json"))
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(one, runs, chunksize=4))
    Path(argv[0]).write_text(json.dumps(res, indent=1), encoding="utf-8")
    by = defaultdict(Counter)
    ent = defaultdict(Counter)
    for r in res:
        by[r["protocol"]].update(r["counts"])
        ent[r["protocol"]].update(r["entry_counts"])
        ent[r["protocol"]]["with another inside"] += r["entries_with_another_inside"]
        ent[r["protocol"]]["runs"] += 1
        for key in ("satisfied_blocked", "satisfied_never_blocked", "arrivals_during_claim", "releases_while_claimed"):
            by[r["protocol"]][key] += r[key]
    for proto in sorted(by, key=str):
        print(f"{proto}: runs {ent[proto]['runs']}; pairs {dict(by[proto])}; CR entries {dict(ent[proto])}")
    tot = Counter()
    for c in by.values():
        tot.update(c)
    print("TOTAL", len(res), "runs; pairs", dict(tot), "; P2 false runs:", sum(not r["P2_holds"] for r in res))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
