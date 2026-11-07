"""When a queued drone's latch stays BUSY: time since its last FRONT corridor echo (replicating gate_perception's corridor
test from the logged nav pose).  BUSY for > t_occ_max after the last corridor echo implies heard_inside() was True."""
import sys, math, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
sys.path.insert(0, r"F:\Andrea\HoloHybridAutomaton")
from gate_metrics import load, gframe, RAW, HALF
from holo_fleet.config import DEFAULT
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.sonar_geometry import R_OUT
G = DEFAULT.gate
for rel, k in [("B2_LOW/LOW-ALL/mutex_deadlock_resolution/s2", 2), ("B2_MEDIUM/MEDIUM-SUBSET/integrated_short/s2", 1),
               ("B2_MEDIUM/MEDIUM-SUBSET/mutex_deadlock_resolution/s0", 2), ("B2_MEDIUM/MEDIUM-ALL/gate_single/s0", 1)]:
    d = RAW.joinpath(*rel.split("/")); rm, sc, rows, n, T, P, st, ev = load(d); g = sc.judged_gates[0]
    gp = GatePerception(sc.plans[k], DEFAULT)
    last_c, busy_since, worst, worst_t, prev = None, None, 0.0, None, None
    spans = []
    for t in sorted(st[k]):
        s = st[k][t]; ga = s.get("gate") or {}
        if ga.get("id") != g.gate_id: continue
        p = np.array(s["nav_p"]); yaw = math.radians(s["nav_yaw_deg"])
        rn, rf = gp.cr_ranges(g, p, yaw); rn -= R_OUT + DEFAULT.env.eps_range_far; rf += DEFAULT.env.eps_range_near
        if any("FRONT" in tg["pattern"] and rn <= tg["r"] <= rf for tg in s.get("targets") or []):
            last_c = t
        if ga.get("occ") == "BUSY" and not s["committed"]:
            if busy_since is None: busy_since = t
            age = t - (last_c if last_c is not None else t)
            if age > worst: worst, worst_t = age, t
        elif busy_since is not None:
            spans.append((busy_since, t, ga.get("occ"))); busy_since = None
    if busy_since is not None: spans.append((busy_since, max(st[k]), "END"))
    print(rel, f"drone_{k}: BUSY spans (queued) {[(round(a,1), round(b,1), c) for a,b,c in spans if b-a > 5]}; "
          f"max time BUSY since last corridor echo {worst:.1f} s at t={worst_t}  (t_occ_max {G.t_occ_max})")
