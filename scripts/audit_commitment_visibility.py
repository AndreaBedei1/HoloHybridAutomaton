"""Premise C audit (proof-to-runtime): commitment visibility of every run of a raw directory, re-read from the logs.

    python scripts/audit_commitment_visibility.py <raw dir> <out.json>
"""

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def one(d: str) -> dict:
    import gzip

    import numpy as np
    import psutil

    from holo_fleet.referee.robustness_metrics import commitment_visibility

    try:
        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass
    p = Path(d)
    m = json.loads((p / "run_metrics.json").read_text(encoding="utf-8"))
    files = sorted(p.glob("drone_*_state.jsonl.gz"), key=lambda f: int(f.name.split("_")[1]))
    states = [[json.loads(x) for x in gzip.open(f, "rt", encoding="utf-8")] for f in files]
    n_rec = min(len(s) for s in states)
    t = np.array([float(r["t"]) for r in states[0][:n_rec]])
    cv = commitment_visibility([f.name.split("_state")[0] for f in files], t, states, n_rec)
    return {"run": p.as_posix(), "protocol": m.get("protocol"), "profile": m.get("profile"), "scenario": m.get("scenario"),
            "seed": m.get("seed"), "class": m.get("class"), "P2_holds": (m.get("P2") or {}).get("holds"), **cv}


if __name__ == "__main__":
    runs = sorted(str(p.parent) for p in Path(sys.argv[1]).rglob("run_metrics.json"))
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(one, runs))
    Path(sys.argv[2]).write_text(json.dumps(res, indent=1), encoding="utf-8")
    ep = sum(r["episodes"] for r in res)
    print(len(res), "runs; episodes", ep, "violations", sum(r["violations"] for r in res),
          "blind commits", sum(r["committed_without_evidence"] for r in res), "censored", sum(r["censored"] for r in res),
          "max", max((r["max_s"] or 0) for r in res), "within 5 s", sum(r["within_5s"] for r in res))
