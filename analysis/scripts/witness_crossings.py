"""Reviewer-A claims, read-only: (1) sonar exit witnesses whose range matches neither the occupant nor any drone;
(2) whole CR crossings during which a queued observer kept its latch FREE (missed corridor detection)."""
import sys, numpy as np
from collections import Counter, defaultdict
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF

TOL = 1.0
wit = defaultdict(Counter); missed = []
for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
    d = p.parent; key = d.relative_to(RAW).parts; prof = "CLEAN" if key[1] == "CLEAN" else key[1].split("-")[0]
    rm, sc, rows, n, T, P, st, ev = load(d)
    if not sc.judged_gates: continue
    g = sc.judged_gates[0]; inside = np.all(np.abs(gframe(g, P)) <= HALF, axis=-1)
    last_in = np.full(n, -1.0)                      # last time each drone was inside the CR
    prev = [None] * n
    for i, t in enumerate(T):
        last_in[inside[i]] = t
        for k in range(n):
            s = st[k].get(round(t, 1)); ga = (s or {}).get("gate") or {}
            occ = ga.get("occ") if ga.get("id") == g.gate_id else None
            w = ga.get("exit_witness") or {}
            if prev[k] == "BUSY" and occ == "EXITING" and w.get("sector") not in (None, "ACOUSTIC"):
                others = [j for j in range(n) if j != k]
                occupant = max(others, key=lambda j: last_in[j])   # most recent CR visitor
                rng = {j: float(np.linalg.norm(P[i, j] - P[i, k])) for j in others}
                c = wit[prof]; c["sonar_witnesses"] += 1
                if abs(w["range"] - rng[occupant]) > TOL: c["range_off_occupant"] += 1
                if all(abs(w["range"] - r) > TOL for r in rng.values()): c["matches_no_drone"] += 1
            prev[k] = occ
    for j in range(n):                              # whole crossings unseen by a queued observer
        idx = np.flatnonzero(inside[:, j])
        if not len(idx): continue
        runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        for r in runs:
            if r[-1] + 1 >= len(T): continue        # crossing not finished
            for k in range(n):
                if k == j: continue
                ss = [st[k].get(round(T[i], 1)) for i in r]
                if any(s is None for s in ss): continue
                ok = all((s.get("gate") or {}).get("id") == g.gate_id and (s["gate"].get("at_queue") in (True, "True"))
                         and not s["committed"] and s["gate"].get("occ") == "FREE" for s in ss)
                if ok:
                    missed.append(("/".join(key), j, k, round(T[r[0]], 1), round(T[r[-1]] - T[r[0]] + 0.1, 1),
                                   any(bool(s["gate"].get("has_prio")) for s in ss)))
print("sonar witnesses by profile (|range - truth| >", TOL, "m):")
for prof, c in sorted(wit.items()): print(f"  {prof:7s} {dict(c)}")
print(f"\nwhole crossings with a queued observer FREE throughout: {len(missed)} in {len({m[0] for m in missed})} runs")
for m in missed: print("  ", m[0], f"occupant d{m[1]} observer d{m[2]} enters {m[3]} s, {m[4]} s inside, observer had prio: {m[5]}")
