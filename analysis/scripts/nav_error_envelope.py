"""Onboard navigation error versus the distance travelled, from logged runs (calibration of the payload nav bound).

For every drone of every run: the horizontal error |nav_p - truth| against the distance travelled by the estimate
since launch, and against the distance travelled since the last relocalization correction.  Reports the largest
ratio error / distance (after a minimum distance) and the per-run maxima.  Usage:
    python analysis/scripts/nav_error_envelope.py <raw root> [<raw root> ...]
"""

import csv
import gzip
import json
import sys
from pathlib import Path

import numpy as np


def runs(roots):
    for root in roots:
        for st in Path(root).rglob("run_status.json"):
            yield st.parent


def one(d):
    ts = d / "referee_timeseries.csv"
    if not ts.exists():
        return []
    with open(ts, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return []
    tt = np.array([float(r["t"]) for r in rows])
    out = []
    k = 0
    while (d / f"drone_{k}_state.jsonl.gz").exists():
        if f"x{k}" not in rows[0]:
            break
        truth = np.array([[float(r[f"x{k}"]), float(r[f"y{k}"])] for r in rows])
        est, te, aid = [], [], []
        with gzip.open(d / f"drone_{k}_state.jsonl.gz", "rt") as fh:
            for line in fh:
                s = json.loads(line)
                if s.get("nav_p") is None:
                    continue
                te.append(float(s["t"]))
                est.append(s["nav_p"][:2])
                na = s.get("nav_aid") or {}
                aid.append(float(np.linalg.norm(na.get("applied", [0.0, 0.0])[:2])) if na.get("applied") else 0.0)
        if not est:
            k += 1
            continue
        est, te, aid = np.array(est), np.array(te), np.array(aid)
        idx = np.clip(np.searchsorted(tt, te), 0, len(tt) - 1)
        err = np.linalg.norm(est - truth[idx], axis=1)
        step = np.r_[0.0, np.linalg.norm(np.diff(est, axis=0), axis=1)]
        dist = np.cumsum(step)
        e0 = err[0]
        ratio = np.where(dist > 5.0, (err - e0) / np.maximum(dist, 1e-6), 0.0)
        out.append({"run": str(d), "drone": k, "err_max": float(err.max()), "dist": float(dist[-1]),
                    "ratio_max": float(ratio.max()), "aid_total": float(aid.sum()), "e0": float(e0)})
        k += 1
    return out


def main(argv):
    res = []
    for d in runs(argv[1:]):
        res.extend(one(d))
    if not res:
        print("no runs")
        return 1
    rm = np.array([r["ratio_max"] for r in res])
    em = np.array([r["err_max"] for r in res])
    print(f"drones {len(res)}  err_max p50 {np.median(em):.3f}  p99 {np.percentile(em, 99):.3f}  max {em.max():.3f} m")
    print(f"(err - e0)/distance after 5 m: p50 {np.median(rm):.4f}  p99 {np.percentile(rm, 99):.4f}  max {rm.max():.4f}")
    for r in sorted(res, key=lambda r: -r["ratio_max"])[:8]:
        print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
