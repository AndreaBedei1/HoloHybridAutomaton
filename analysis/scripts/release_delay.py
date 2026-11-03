"""Delay between the true exit of the occupant and the latch release, by release kind and cell (B2 + BLACKOUT)."""
import sys, numpy as np
from collections import defaultdict
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
out = defaultdict(lambda: defaultdict(list))
for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
    d = p.parent; cell = "/".join(d.relative_to(RAW).parts[:2])
    if cell not in ("B2_CLEAN/CLEAN", "B2_BLACKOUT/CLEAN", "B0/CLEAN", "B2_MEDIUM/CLEAN", "B2_LOW/LOW-ALL", "B2_MEDIUM/MEDIUM-ALL", "B2_BLACKOUT/MEDIUM-ALL"): continue
    rm, sc, rows, n, T, P, st, ev = load(d)
    if not sc.judged_gates: continue
    g = sc.judged_gates[0]; gf = gframe(g, P); inside = np.all(np.abs(gf) <= HALF, axis=-1)
    for k in range(n):
        prev = None
        for i, t in enumerate(T):
            s = st[k].get(round(t, 1))
            if s is None: continue
            ga = s.get("gate") or {}; occ = ga.get("occ")
            if prev == "BUSY" and occ in ("EXITING", "FREE") and not s["committed"]:
                w = ga.get("exit_witness") or {}
                kind = "TIMEOUT" if occ == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")
                outs = [ii for ii in range(1, i + 1) for j in range(n) if j != k and inside[ii - 1, j] and not inside[ii, j]]
                if outs: out[cell][kind].append(t - T[max(outs)])
            prev = occ
for cell, D in sorted(out.items()):
    print(cell, {k: f"n={len(v)} median {np.median(v):.1f} p90 {np.percentile(v, 90):.1f} max {max(v):.1f}" for k, v in D.items()})
