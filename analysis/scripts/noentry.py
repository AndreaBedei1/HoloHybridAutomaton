import sys, json, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
from collections import Counter
for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
    d = p.parent
    rm, sc, rows, n, T, P, st, ev = load(d)
    if not sc.judged_gates: continue
    g = sc.judged_gates[0]; gf = gframe(g, P); inside = np.all(np.abs(gf) <= HALF, axis=-1)
    for e in ev:
        if e.get("type") != "transition" or e.get("edge") != "commit": continue
        k = int(e["drone"].split("_")[1]); i = int(np.searchsorted(T, e["t"] - 1e-6))
        if i >= len(T) or any(inside[ii, k] for ii in range(i, len(T))): continue
        last = st[k][max(st[k])]
        modes = Counter(st[k][x]["mode"] for x in st[k] if x >= T[i])
        faults = [x for x in (sc.sim.faults if hasattr(sc.sim, 'faults') else [])]
        print("/".join(d.relative_to(RAW).parts), rm["class"], f"drone_{k} commit t={e['t']:.1f} end t={T[-1]:.1f} true s,l,dz={np.round(gf[-1,k],2).tolist()} min|l|={np.abs(gf[i:,k,1]).min():.2f}",
              "final mode", last["mode"], "modes after:", dict(modes.most_common(3)), "reloc_hold", last["gate"].get("reloc_hold"), "term", [x.get("reason") for x in ev if x.get("type")=="TERMINAL"])
