"""Proposed gate metrics, offline, B2 cells of the final campaign (truth = referee timeseries)."""
import sys, numpy as np
from collections import defaultdict
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
agg = defaultdict(lambda: defaultdict(list))
for p in sorted(RAW.glob("B2*/*/*/s*/run_metrics.json")):
    d = p.parent; rm, sc, rows, n, T, P, st, ev = load(d)
    if not sc.judged_gates: continue
    cell = "/".join(d.relative_to(RAW).parts[:2]); g = sc.judged_gates[0]
    gf = gframe(g, P); inside = np.all(np.abs(gf) <= HALF, axis=-1)
    faulty = {int(f["drone"].split("_")[1]) for f in sc.sim.faults if f.get("t_off") is None}
    A = agg[cell]; A["runs"].append(1)
    for k in range(n):
        if k in faulty: continue
        false_free = false_busy = queued = 0
        t_q = t_c = None
        for i, t in enumerate(T):
            s = st[k].get(round(t, 1))
            if s is None: continue
            ga = s.get("gate") or {}
            if ga.get("id") != g.gate_id: continue
            other_in = any(inside[i, j] for j in range(n) if j != k)
            if ga.get("at_queue") in (True, "True") and not s["committed"]:
                queued += 1; t_q = t if t_q is None else t_q
                if ga.get("occ") == "FREE" and other_in: false_free += 1
                if ga.get("occ") != "FREE" and not other_in: false_busy += 1
            if s["committed"] and t_c is None: t_c = t
        A["queued_s"].append(queued * 0.1); A["false_free_s"].append(false_free * 0.1); A["false_busy_s"].append(false_busy * 0.1)
        if t_q is not None: A["wait_s"].append((t_c if t_c is not None else T[-1]) - t_q)
        passed = (rm.get("terminal") or {}).get("gate_passed_t") or {}
        A["passed"].append(passed.get(f"drone_{k}") is not None)
        A["committed"].append(t_c is not None)
print(f"{'cell':28s} runs drones passed% committed-not-passed  wait p50/p90/max [s]   queued-time FALSE-FREE frac   FALSE-BUSY frac")
for cell, A in sorted(agg.items()):
    q = sum(A["queued_s"]); w = np.array(A["wait_s"]) if A["wait_s"] else np.array([0.0])
    cnp = sum(1 for c, p in zip(A["committed"], A["passed"]) if c and not p)
    print(f"{cell:28s} {len(A['runs']):4d} {len(A['passed']):6d} {100*np.mean(A['passed']):6.1f} {cnp:6d}   {np.percentile(w,50):6.1f}/{np.percentile(w,90):6.1f}/{w.max():6.1f}   {sum(A['false_free_s'])/max(q,1e-9):.3f}   {sum(A['false_busy_s'])/max(q,1e-9):.3f}")
