"""Calibration of the commanded-velocity model used for dead reckoning during a DVL outage.

The low level tracks the requested world velocity v_req (controller record ``v_cmd``) with a closed-loop lag.  Model:
a first-order response dv/dt = (v_req - v) / tau, started from the last DVL velocity.  For every logged run (ground
truth positions in referee_timeseries.csv, 10 Hz) this script reports, for a grid of tau, the horizontal speed error
|v_true - v_model| of 1 s predictions started at every sample (the length of a MEDIUM DVL outage), and its quantiles.

    python analysis/scripts/cmd_velocity_model.py <raw root> [<raw root> ...]
"""

import csv
import gzip
import json
import sys
from pathlib import Path

import numpy as np


def runs(roots):
    for root in roots:
        for p in Path(root).rglob("run_metrics.json"):
            if "_attempts" not in p.parts and (p.parent / "referee_timeseries.csv").exists():
                yield p.parent


def series(d):
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    k = 0
    while (d / f"drone_{k}_state.jsonl.gz").exists() and f"x{k}" in rows[0]:
        P = np.array([[float(r[f"x{k}"]), float(r[f"y{k}"])] for r in rows])
        recs = [json.loads(x) for x in gzip.open(d / f"drone_{k}_state.jsonl.gz", "rt")]
        m = min(len(recs), len(t))
        vreq = np.array([r["v_cmd"][:2] for r in recs[:m]])
        modes = [r.get("mode") for r in recs[:m]]
        yield t[:m], P[:m], vreq, modes
        k += 1


def main(argv):
    taus = (0.3, 0.5, 0.8, 1.2)
    errs = {tau: [] for tau in taus}
    held = []
    H = 10                                     # 1 s predictions at 10 Hz
    n_runs = 0
    for d in runs(argv[1:]):
        n_runs += 1
        for t, P, vreq, modes in series(d):
            if len(t) < H + 3:
                continue
            dt = np.diff(t)
            vt = np.vstack([np.zeros(2), np.diff(P, axis=0) / np.maximum(dt, 1e-6)[:, None]])
            for i in range(1, len(t) - H, 3):
                for tau in taus:
                    v = vt[i].copy()
                    for j in range(H):
                        v = v + (vreq[i + j] - v) * min(1.0, 0.1 / tau)
                        errs[tau].append(float(np.linalg.norm(vt[i + j + 1] - v)))
                held.extend(float(np.linalg.norm(vt[i + j + 1] - vt[i])) for j in range(H))
    print(f"runs {n_runs}; samples {len(held)}")
    for name, e in [("held DVL velocity", held)] + [(f"first-order tau {tau}", errs[tau]) for tau in taus]:
        e = np.asarray(e)
        print(f"{name:24s} p50 {np.percentile(e, 50):.3f}  p99 {np.percentile(e, 99):.3f}  "
              f"p99.9 {np.percentile(e, 99.9):.3f}  max {e.max():.3f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
