import sys, math, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, CASES, HALF, MOUNTS
for key, (k, t0) in list(CASES.items()):
    d = RAW.joinpath(*key); rm, sc, rows, n, T, P, st, ev = load(d); g = sc.judged_gates[0]
    gf = gframe(g, P); inside = np.all(np.abs(gf) <= HALF, axis=-1)
    i0 = int(np.searchsorted(T, t0 - 1e-6)); occ = [j for j in range(n) if j != k and inside[i0, j]][0]
    print("/".join(key), f"releaser {k} occupant {occ}")
    for i in range(i0 - 25, i0 + 8):
        s = st[k].get(round(T[i], 1))
        if s is None: continue
        yaw = math.radians(s["nav_yaw_deg"]); son = P[i, k] + np.array([math.cos(yaw), math.sin(yaw), 0]) * MOUNTS["FRONT"][0]
        r = np.linalg.norm(P[i, occ] - son)
        fr = s["sectors"]["FRONT"]
        print(f"  t={T[i]:5.1f} occ s={gf[i,occ,0]:+.2f} r_ctr={r:5.2f} FRONT healthy={int(fr['healthy'])} echoes={fr['echoes']} latch={s['gate']['occ']} w={s['gate']['exit_witness']}")
