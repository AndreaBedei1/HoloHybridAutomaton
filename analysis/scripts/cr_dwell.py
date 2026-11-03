"""Dwell time of every drone inside the CR (truth) over the final campaign; the modes it was in while inside."""
import sys, numpy as np
from collections import Counter
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
dw, modes_in = [], Counter()
for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
    d = p.parent; rm, sc, rows, n, T, P, st, ev = load(d)
    if not sc.judged_gates: continue
    g = sc.judged_gates[0]; gf = gframe(g, P); inside = np.all(np.abs(gf) <= HALF, axis=-1)
    for k in range(n):
        idx = np.flatnonzero(inside[:, k])
        if not len(idx): continue
        # contiguous visits
        splits = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        for v in splits:
            ms = Counter(st[k].get(round(T[i], 1), {}).get("mode", "?") for i in v)
            modes_in.update(ms)
            dw.append((T[v[-1]] - T[v[0]] + 0.1, "/".join(d.relative_to(RAW).parts), k, dict(ms.most_common(3)), len(splits)))
dw.sort(key=lambda x: -x[0])
print("visits", len(dw), "dwell percentiles [s] p50/p95/p99/max:", np.percentile([x[0] for x in dw], [50, 95, 99, 100]).round(1))
print("modes while inside the CR (0.1 s steps):", dict(modes_in.most_common()))
for x in dw[:8]: print(f"  {x[0]:5.1f} s {x[1]} drone_{x[2]} modes {x[3]} visits {x[4]}")
print("multi-visit (re-entries):", sum(1 for x in dw if x[4] > 1))
