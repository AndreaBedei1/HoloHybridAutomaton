"""Reviewer-B claim: gate runs with an idle interval >= 20 s (a healthy drone queued, CR truly empty, nobody committed)."""
import sys, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
for cell in sys.argv[1:]:
    hit = tot = 0; worst = []
    for p in sorted((RAW / cell).glob("*/s*/run_metrics.json")):
        rm, sc, rows, n, T, P, st, ev = load(p.parent)
        if not sc.judged_gates: continue
        g = sc.judged_gates[0]; inside = np.all(np.abs(gframe(g, P)) <= HALF, axis=-1); tot += 1
        faulty = {int(f["drone"].split("_")[1]) for f in sc.sim.faults if f.get("t_off") is None}
        run = best = 0.0
        for i, t in enumerate(T):
            S = [st[k].get(round(t, 1)) or {} for k in range(n)]
            q = any((s.get("gate") or {}).get("at_queue") in (True, "True") and not s.get("committed") for k, s in enumerate(S) if k not in faulty)
            busy = inside[i].any() or any(s.get("committed") for s in S)
            run = run + 0.1 if (q and not busy) else 0.0; best = max(best, run)
        hit += best >= 20.0; worst.append(round(best, 1))
    print(f"{cell}: {hit}/{tot} gate runs idle >= 20 s; longest idle per run {sorted(worst, reverse=True)[:6]}")
