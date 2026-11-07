"""Offline feasibility check (no code change): is min_s A_s(r_s) a sound centre-distance bound for 2-sector adjacent
patterns, for one hull and for two hulls merged into one target?  Uses the S0 sampler geometry of formal/check_separation."""
import math, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "formal")
from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from formal.check_separation import _hull_points, _rand_rot
cfg = DEFAULT; env = cfg.env; M = cfg.perc.sonar
cos60 = math.cos(math.radians(M.half_angle_deg)); H = _hull_points()

def sector_ranges(P):
    out = {}
    for s in sg.SECTORS:
        sv = P - sg.MOUNTS[s]; r = np.linalg.norm(sv, axis=1); inc = (sv @ sg.AXES[s]) / r >= cos60
        if inc.any() and float(r[inc].min()) >= M.range_min:
            out[s] = float(r[inc].min())
    return out

def union_bound(ranges):
    P = frozenset(ranges)
    if len(P) == 2 and all(b in [x for x in sg.SECTORS if x != a and x != sg.MIRROR[a]] for a in P for b in P if a != b):
        return min(sg.centre_distance_lower(r, s, env.eps_range_far, M, centre_in_cone=True) for s, r in ranges.items())
    return sg.target_distance_lower(ranges, env.eps_range_far, M)

def current(ranges):
    return sg.target_distance_lower(ranges, env.eps_range_far, M)

rng = np.random.default_rng(7)
for label, two, n in (("one hull", False, 40000), ("two hulls (merged when ranges within 0.6 m)", True, 40000)):
    worst_u, worst_c, cnt2, gain = 1e9, 1e9, 0, []
    for _ in range(n):
        hulls = []
        for _h in range(2 if two else 1):
            d = rng.uniform(1.0, 6.0); u = rng.normal(size=3); u /= np.linalg.norm(u)
            hulls.append((d, d * u + H @ _rand_rot(rng).T))
        # nearest range per sector over all hulls (what the sonar reports), and the true nearest centre distance
        R = {}
        for d, P in hulls:
            for s, r in sector_ranges(P).items():
                R[s] = min(R.get(s, 1e9), r)
        if len(R) < 2:
            continue
        dtrue = min(d for d, _ in hulls)
        # targets as build_targets would merge them: take the closest echo and adjacent sectors within 0.6 m
        e = sorted(R.items(), key=lambda x: x[1]); s0, r0 = e[0]; members = {s0: r0}
        for s, r in e[1:]:
            if abs(r - r0) <= 0.6 and all(s in [x for x in sg.SECTORS if x != m and x != sg.MIRROR[m]] for m in members):
                members[s] = r
        if len(members) != 2:
            continue
        cnt2 += 1
        for err in (-env.eps_range_near, 0.0, env.eps_range_far):
            rr = {s: r + err for s, r in members.items()}
            bu, bc = union_bound(rr), current(rr)
            worst_u = min(worst_u, dtrue - bu); worst_c = min(worst_c, dtrue - bc)
            gain.append(bu - bc)
    g = np.array(gain)
    print(f"{label}: 2-sector merged targets {cnt2}; min(d - bound): current {worst_c:.3f} m, union {worst_u:.3f} m; "
          f"tightening p50 {np.percentile(g,50):.2f} p95 {np.percentile(g,95):.2f} m")
# the square-corner case of the logs: LEFT 2.92 / REAR 3.07 and LEFT 2.87 / REAR 3.12
for rr in ({"LEFT": 2.92, "REAR": 3.07}, {"LEFT": 2.87, "REAR": 3.12}, {"FRONT": 2.92, "LEFT": 3.07}):
    print(rr, "current", round(current(rr), 3), "union", round(union_bound(rr), 3))
