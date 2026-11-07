import sys, json, math, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF
CASES = [("B2_MEDIUM/MEDIUM-ONE/gate_single/s0", 0), ("B2_MEDIUM/MEDIUM-ALL/gate_single/s0", 1),
         ("B2_MEDIUM/MEDIUM-SUBSET/lost_drone_mutex/s0", 2), ("B2_MEDIUM/MEDIUM-SUBSET/integrated_short/s2", 1),
         ("B2_LOW/LOW-ALL/mutex_deadlock_resolution/s2", 2), ("B2_MEDIUM/MEDIUM-SUBSET/gate_single/s0", 1)]
step = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
for rel, k in CASES:
    d = RAW.joinpath(*rel.split("/")); rm, sc, rows, n, T, P, st, ev = load(d); g = sc.judged_gates[0]
    gf = gframe(g, P)
    print("=" * 100); print(rel, f"drone_{k}")
    nxt = -1
    for i, t in enumerate(T):
        s = st[k].get(round(t, 1))
        if s is None or t < nxt: continue
        nxt = t + step
        ga = s.get("gate") or {}
        tg = sorted(s.get("targets") or [], key=lambda x: x["d_lower"])[:2]
        na = s.get("nav_aid") or {}
        print(f" t={t:6.1f} {s['mode'][:14]:14s} c={int(s['committed'])} est(s,l)=({ga.get('s')},{ga.get('l')}) true(s,l,dz)=({gf[i,k,0]:+.2f},{gf[i,k,1]:+.2f},{gf[i,k,2]:+.2f})"
              f" occ={ga.get('occ')} prio={int(bool(ga.get('has_prio')))} loc_ok={int(bool(ga.get('loc_ok')))} hold={int(bool(ga.get('reloc_hold')))} dmin={s.get('d_min')}"
              f" tg={[(x['pattern'], x['d_lower'], x['cls']) for x in tg]} aid={ {kk: na.get(kk) for kk in list(na)[:6]} } ac={ga.get('acoustic')}")
