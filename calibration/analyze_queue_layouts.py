"""Offline evaluation of candidate queue layouts (lateral, dz) for N = 2..6 against the P2 queue constraints.
Uses the formal helpers of formal/check_mutex.py and the deployed perception geometry; no simulation."""
import itertools, json, math, sys
import numpy as np
_ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]; sys.path.insert(0, str(_ROOT)); sys.path.insert(0, str(_ROOT / "formal"))
import check_mutex as cm
from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import pass_path_points
from holo_fleet.ha import mutex_rule
from holo_fleet.mission import structure_masked_pairs
from holo_fleet.perception.exit_witness import ExitWitness
from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS, SECTORS, centre_distance_lower
from holo_fleet.perception.sonar_processing import EchoClassifier

CFG = DEFAULT; G, ENV, PC = CFG.gate, CFG.env, CFG.perc
GATE = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
Z_GATE = float(GATE.center[2])
R_EFF = cm.R_DETECT_M - 1.0
MIN_WINDOW_S = 1.0
FUZZ = 5.0


def columns_layout(counts, c, top=None):
    """Columns left to right, c apart, each with counts[k] stacked points stack_spacing apart, top first; the stack
    is centred on the gate depth (top=None) or its top level is at dz = top."""
    k = len(counts)
    pts = []
    for i, m in enumerate(counts):
        l = ((k - 1) / 2.0 - i) * c
        for r in range(m):
            dz = ((m - 1) / 2.0 - r) * G.stack_spacing if top is None or m == 1 else top - r * G.stack_spacing
            pts.append((round(l, 3), round(dz, 3)))
    return pts


def queue_s_of(pts):
    lats = tuple(sorted({l for l, _ in pts}, reverse=True))
    q = G.queue_s(len(lats), max(abs(dz) for _, dz in pts), lats=lats)
    if len(lats) < len(pts):
        while structure_masked_pairs(pts, q, G, PC) and q > -20.0:
            q = round(q - 0.1, 6)
    return q


def relations_ok(pts, q_s):
    """M2 / M2v for an arbitrary layout: possible relations of every pair inside the queue bracket (farthest echo,
    holding tolerance both) and the unique, precedence-first leader of every occupancy."""
    P = [np.array([q_s, l, dz]) for l, dz in pts]
    n = len(P)
    inb = {(i, j): float(np.linalg.norm(P[i] - P[j])) + 2 * G.queue_tol - 0.59 <= G.queue_bracket_m
           for i in range(n) for j in range(n) if i != j}
    rels = {k: cm.possible_relations(G, P[k[0]], P[k[1]], FUZZ) for k, v in inb.items() if v}
    amb = sorted(f"{i}->{j}" for (i, j), r in rels.items() if len(r) > 1)
    bad = 0
    for r in range(1, n + 1):
        for S in itertools.combinations(range(n), r):
            leaders = []
            for i in S:
                js = [j for j in S if j != i and inb[(i, j)]]
                rel_i = [next(iter(rels[(i, j)])) for j in js]
                if mutex_rule.decide(rel_i, i, their_ranks=js) == mutex_rule.PRIORITY:
                    leaders.append(i)
            bad += leaders != [min(S)]
    return {"ambiguous": amb, "bad_occupancies": bad, "out_of_bracket_pairs": sum(not v for v in inb.values()) // 2,
            "ok": not amb and bad == 0}


def cr_bearing(pts, q_s):
    worst = 0.0
    for l, dz in pts:
        son = np.array([q_s + float(MOUNTS["FRONT"][0]), l, dz])
        for a, b, c in itertools.product((-1, 1), repeat=3):
            v = np.array([a * G.cr_half_len, b * G.cr_half_width, c * G.cr_half_height]) - son
            worst = max(worst, math.degrees(math.acos(v[0] / np.linalg.norm(v))))
    return round(worst + G.heading_tol_deg, 2)


def merge_clearance(pts, q_s):
    clear = 99.0
    for i, (l, dz) in enumerate(pts):
        path = pass_path_points(G, q_s, l, dz)
        later = [np.array([q_s, *pts[j]]) for j in range(i + 1, len(pts))]
        for a, b in zip(path[:-1], path[1:]):
            for t in np.linspace(0, 1, 201):
                x = a + t * (b - a)
                for q in later:
                    clear = min(clear, float(np.linalg.norm(x - q)))
    return round(clear, 3)


def neighbour_d_lower(pts, q_s):
    """M4c generalized: conservative centre distance of every queued neighbour, as formal M4c (echo of the near
    surface at the worst holding error of both, sector certified to contain the centre: the most aligned one)."""
    worst = 99.0
    for i, j in itertools.permutations(range(len(pts)), 2):
        d = np.array([0.0, pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]])
        sec = max(SECTORS, key=lambda k: float(AXES[k] @ d))
        r_echo = float(np.linalg.norm(d)) - 2 * G.hold_err_m - float(np.linalg.norm(MOUNTS[sec])) - 0.29 - ENV.eps_range_near
        worst = min(worst, centre_distance_lower(r_echo, sec, ENV.eps_range_far, PC.sonar, centre_in_cone=True))
    return round(worst, 3)


def exit_window(pts, q_s, lanes_of):
    """M4g with margin for an arbitrary layout: per (observer, passer, lane), seconds of the passer's path beyond the
    CR with an observable witness within R_EFF (any sector)."""
    clf, wit = EchoClassifier(gate_bars=list(GATE.bars), cfg=CFG), ExitWitness(CFG)
    yaw = math.atan2(GATE.axis[1], GATE.axis[0])
    Rq = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
    left = np.array([-GATE.axis[1], GATE.axis[0], 0.0])
    hull = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if max(abs(x), abs(y), abs(z)) == 1]) * HULL_HALF
    rows = []
    for oi, (li, zi) in enumerate(pts):
        p = GATE.from_gate_frame([q_s, li, zi])
        cones = {sec: (p + Rq @ MOUNTS[sec], Rq @ AXES[sec], None, wit.r_far(GATE, p, yaw, sec) + ENV.eps_range_far) for sec in SECTORS}
        cones = {k: (a, b, clf.structure_window(a, b), d) for k, (a, b, _, d) in cones.items()}
        for pj, (lj, zj) in enumerate(pts):
            if pj == oi:
                continue
            for lane in lanes_of[pj]:
                path = pass_path_points(G, q_s, lj, zj, lane=lane, lane_dz=0.0)
                seen, best = 0, None
                for a, b in zip(path[:-1], path[1:]):
                    k = max(1, int(np.linalg.norm(b - a) / 0.1))
                    for t in range(k):
                        cg = a + (b - a) * t / k
                        if cg[0] <= G.cr_half_len:
                            continue
                        c = GATE.from_gate_frame(cg)
                        dw = (b - a)[0] * GATE.axis + (b - a)[1] * left
                        hy = math.atan2(dw[1], dw[0])
                        Rh = np.array([[math.cos(hy), -math.sin(hy), 0], [math.sin(hy), math.cos(hy), 0], [0, 0, 1.0]])
                        surf = c + hull @ Rh.T
                        sight = [c + dl * left + np.array([0, 0, dz]) for dl in (-HULL_HALF[1], 0, HULL_HALF[1]) for dz in (-HULL_HALF[2], 0, HULL_HALF[2])]
                        for sec, (son, ax, sw, bound) in cones.items():
                            d = surf - son; r = np.linalg.norm(d, axis=1)
                            inc = np.arccos(np.clip((d @ ax) / r, -1, 1)) <= math.radians(60.0)
                            if not inc.any():
                                continue
                            rr = float(r[inc].min())
                            if rr > R_EFF or rr - ENV.eps_range_near <= bound:
                                continue
                            if not all(wit.ahead(GATE, p, yaw, sec, x) for x in (rr - ENV.eps_range_near, rr + ENV.eps_range_far)):
                                continue
                            if sw is not None and sw[0] - PC.structure_tol_m <= rr <= sw[1] + PC.structure_tol_far_m:
                                continue
                            if any(cm._segment_hits_box(son, q, bb) for q in sight for bb in GATE.bars):
                                continue
                            seen += 1; best = best or (round(float(cg[0]), 2), sec, round(rr, 2))
                            break
                rows.append((oi, pj, lane, round(seen * 0.1 / G.v_pass, 2), best))
    return rows


def evaluate(name, pts, lanes_of):
    q_s = queue_s_of(pts)
    rel = relations_ok(pts, q_s)
    ew = exit_window(pts, q_s, lanes_of)
    worst = min(ew, key=lambda x: x[3])
    zs = [Z_GATE + dz for _, dz in pts]
    out = {"layout": name, "points": pts, "queue_s": round(q_s, 2),
           "l_max": max(abs(l) for l, _ in pts), "z_range": (round(min(zs), 2), round(max(zs), 2)),
           "band_ok": min(zs) - HULL_HALF[2] >= ENV.z_min and max(zs) + HULL_HALF[2] <= ENV.z_max,
           "cr_bearing": cr_bearing(pts, q_s), "merge_clearance": merge_clearance(pts, q_s),
           "neighbour_d_lower": neighbour_d_lower(pts, q_s), "relations": rel,
           "exit_min_window_s": worst[3], "exit_worst": worst,
           "exit_blind_pairs": sum(1 for r in ew if r[3] < MIN_WINDOW_S)}
    out["ok"] = (out["band_ok"] and out["cr_bearing"] <= 60.0 - FUZZ + 1e-9 and out["merge_clearance"] >= G.merge_clearance - 1e-6
                 and out["neighbour_d_lower"] >= CFG.sep.d_warning_exit and rel["ok"] and out["exit_blind_pairs"] == 0)
    return out


def candidates(n):
    out = [("abreast 3.5", columns_layout([1] * n, G.queue_spacing))]
    for c in (3.5, 3.75, 4.0, 4.25, 4.5, 5.0):
        for k in range(math.ceil(n / 2), n):
            for counts in set(itertools.permutations([2] * (n - k) + [1] * (2 * k - n))):
                if len(counts) == k and sum(counts) == n and max(counts) <= 2:
                    out.append((f"{k} columns {counts} c={c}", columns_layout(list(counts), c)))
                    if max(counts) == 2:
                        out.append((f"{k} columns {counts} c={c} top=0", columns_layout(list(counts), c, top=0.0)))
    return out


def task(args):
    import psutil; psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    n, name, pts = args
    from holo_fleet.formations import TEMPLATES
    from holo_fleet.mission import Slot
    temps = [list(t.slots) for t in TEMPLATES.values() if len(t.slots) == n] + [[Slot(0.0, ((n - 1) / 2.0 - r) * 3.5, 0.0) for r in range(n)]]
    lanes_of = [set() for _ in pts]
    for sl in temps:
        order = sorted(range(n), key=lambda k: (-sl[k].lateral, -sl[k].along, -sl[k].dz))
        for qi, k in enumerate(order):
            lanes_of[qi].add(round(sl[k].lateral, 2))
    lanes_of = [sorted(x) for x in lanes_of]
    try:
        r = evaluate(name, pts, lanes_of)
    except Exception as e:  # noqa
        r = {"layout": name, "error": repr(e)}
    r["n"] = n
    return r


if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = [(n, name, pts) for n in range(2, 7) for name, pts in candidates(n)]
    with Pool(20) as p:
        res = p.map(task, jobs)
    json.dump(res, open(sys.argv[1], "w"), indent=1, default=str)
    for r in res:
        if "error" in r:
            print(r["n"], r["layout"], r["error"]); continue
        print(f"N={r['n']} {r['layout']:34s} ok={r['ok']!s:5} q_s {r['queue_s']:6.2f} lmax {r['l_max']:4.2f} z {r['z_range']} band {r['band_ok']!s:5} "
              f"bear {r['cr_bearing']:5.1f} merge {r['merge_clearance']:5.2f} dlow {r['neighbour_d_lower']:5.2f} rel {r['relations']['ok']!s:5} "
              f"(amb {len(r['relations']['ambiguous'])} bad {r['relations']['bad_occupancies']} outbr {r['relations']['out_of_bracket_pairs']}) "
              f"exit min {r['exit_min_window_s']:4.1f}s blind {r['exit_blind_pairs']} worst {r['exit_worst']}")
