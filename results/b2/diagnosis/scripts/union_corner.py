import math, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "formal")
exec(open(sys.argv[1]).read().split("rng = np.random.default_rng(7)")[0])
rng = np.random.default_rng(11)
for label, near in (("corner directions 1-3 m (S0 hard case)", (1.0, 3.0)), ("near field 0.8-1.5 m", (0.8, 1.5))):
    worst_u, worst_c, cnt = 1e9, 1e9, 0; bad = []
    for _ in range(60000):
        d = rng.uniform(*near); u = np.sign(rng.normal(size=3)) / math.sqrt(3.0) + 0.12 * rng.normal(size=3)
        if label.startswith("near"): u = rng.normal(size=3)
        u /= np.linalg.norm(u); P = d * u + H @ _rand_rot(rng).T
        R = sector_ranges(P)
        if len(R) < 2: continue
        e = sorted(R.items(), key=lambda x: x[1]); s0, r0 = e[0]; members = {s0: r0}
        for s, r in e[1:]:
            if abs(r - r0) <= 0.6 and all(s in [x for x in sg.SECTORS if x != m and x != sg.MIRROR[m]] for m in members):
                members[s] = r
        if len(members) != 2: continue
        cnt += 1
        for err in (-env.eps_range_near, 0.0, env.eps_range_far):
            rr = {s: r + err for s, r in members.items()}
            gu = d - union_bound(rr); gc = d - current(rr)
            worst_u = min(worst_u, gu); worst_c = min(worst_c, gc)
            if gu < 0 and len(bad) < 3: bad.append((round(d, 2), rr, round(union_bound(rr), 3)))
    print(f"{label}: 2-sector targets {cnt}; min(d - bound) current {worst_c:.3f}, union {worst_u:.3f}; violations e.g. {bad}")
