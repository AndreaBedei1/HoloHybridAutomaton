"""Choose the G06 bar section (thickness, depth) offline (result: results/gate/bar_section_analysis.json).  Opening 2.0 x 2.0 m, 120 deg beams.
(1) observability: closed-loop gate localization at the hold zone (synthetic nominal-cone echoes, MEDIUM-like noise);
(2) masking: a real drone near the gate whose echo falls in the observer's structure window (classified STRUCTURE);
(3) phantoms: a true bar echo NOT explained by the map at a wrong estimated pose (-> DYNAMIC/UNKNOWN), within d_warning.
Truth only judges.  20 workers, BELOW_NORMAL."""
import math, sys, json, itertools
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import psutil
from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import EchoClassifier
from holo_fleet.perception.structure_aiding import StructureAiding, HULL_LAT_M
RANGES = DEFAULT.perc.sonar.ranges()
PC = DEFAULT.perc
SECTIONS = [(0.18, 0.22), (0.25, 0.30), (0.30, 0.40), (0.40, 0.50), (0.50, 0.60)]
_C = {}
def gate(sec):
    if sec not in _C:
        g = load_arena_gates(HORSESHOE_TRACK, ["G06"], bar_section={"G06": sec})[0]
        _C[sec] = (g, EchoClassifier(gate_bars=list(g.bars), cfg=DEFAULT))
    return _C[sec]
def Rz(g, yawd):
    y = math.atan2(g.axis[1], g.axis[0]) + math.radians(yawd)
    return np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1.0]])
def first_echo(clf, pos, a):
    d = np.asarray(clf.struct_pts) - pos; r = np.linalg.norm(d, axis=1)
    ang = np.arccos(np.clip((d @ a) / np.maximum(r, 1e-9), -1, 1)); m = clf.model
    sel = (ang <= np.radians(m.half_angle_deg)) & (r >= m.range_min) & (r <= m.range_max)
    return float(r[sel].min()) if sel.any() else None
def in_window(sw, r0, r1):
    return sw is not None and sw[0] - PC.structure_tol_m <= r0 and r1 <= sw[1] + PC.structure_tol_far_m

def obs_task(args):
    sec, yawd, s, noise, seed = args
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    g, clf = gate(sec); rng = np.random.default_rng(seed); R = Rz(g, yawd); W2 = g.inner_width / 2
    st = dict(n=0, validated=0, unsafe_valid=0, wrong_applied=0, fixed=0, sectors_seen=0)
    for l in np.round(np.arange(-0.6, 0.61, 0.1), 2):
        pt = g.from_gate_frame([s, l, 0.0])
        for dl in (-0.5, -0.3, 0.3, 0.5):
            pe = g.from_gate_frame([s, l + dl, 0.0]); aid = StructureAiding(clf, DEFAULT, gates=[g])
            st["n"] += 1; wrong = valid_any = False
            for k in range(8):
                profs = {}
                for q in sg.SECTORS:
                    a, _ = clf._sector_frame(q, R); w = first_echo(clf, pt + R @ sg.MOUNTS[q], a)
                    if k == 0 and w is not None: st["sectors_seen"] += 1
                    pr = np.zeros_like(RANGES)
                    if w is not None and not (noise and rng.random() < 0.08):
                        if noise: w += rng.normal(0, 0.05)
                        pr[(RANGES >= w) & (RANGES <= w + 0.15)] = 0.9
                    profs[q] = pr
                c = aid.update(pe, R, profs, {x: 0.0 for x in sg.SECTORS}, 0.25, t=0.1 * k)
                if c is not None:
                    if np.linalg.norm((pt - pe - c)[:2]) > np.linalg.norm((pt - pe)[:2]) + 0.05: wrong = True
                    pe = pe + c
                if aid.valid(0.1 * k):
                    valid_any = True
                    if W2 - (abs(g.to_gate_frame(pt)[1]) + HULL_LAT_M) < 0: st["unsafe_valid"] += 1
            st["wrong_applied"] += wrong; st["validated"] += valid_any
            st["fixed"] += valid_any and np.linalg.norm((pt - pe)[:2]) < 0.1
    return ("obs", sec, noise), st

def mask_task(args):
    """Observer A on the approach / at the queue (heading along the axis), drone B in the passage region."""
    sec, s_a, l_a = args
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    g, clf = gate(sec)
    st = dict(pairs=0, seen=0, masked_any=0, hidden=0, cr_pairs=0, cr_hidden=0)
    for yawd in (-5.0, 0.0, 5.0):
        R = Rz(g, yawd); A = g.from_gate_frame([s_a, l_a, 0.0])
        for s_b in np.arange(-3.0, 3.01, 0.2):
            for l_b in np.arange(-1.6, 1.61, 0.2):
                for dz_b in (-0.6, 0.0, 0.6):
                    B = g.from_gate_frame([s_b, l_b, dz_b])
                    if np.linalg.norm(B - A) < 1.2: continue
                    st["pairs"] += 1; cr = abs(s_b) <= 0.8 and abs(l_b) <= 1.0
                    st["cr_pairs"] += cr
                    seen = masked = 0
                    for q in sg.SECTORS:
                        a, _ = clf._sector_frame(q, R); spos = A + R @ sg.MOUNTS[q]
                        d = B - spos; rc = np.linalg.norm(d)
                        if math.degrees(math.acos(np.clip(d @ a / rc, -1, 1))) > clf.model.half_angle_deg: continue
                        r0 = rc - 0.29
                        if r0 > clf.model.range_max: continue
                        seen += 1
                        masked += in_window(clf.structure_window(spos, a), r0, r0 + 0.3)
                    if seen:
                        st["seen"] += 1; st["masked_any"] += masked > 0; st["hidden"] += masked == seen
                        st["cr_hidden"] += cr and masked == seen
    return ("mask", sec), st

def phantom_task(args):
    """True pose on the approach, estimate off by e in direction phi: true bar echoes outside the map window."""
    sec, s, e = args
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    g, clf = gate(sec)
    st = dict(n=0, echoes=0, unexplained=0, unexplained_warn=0, poses_warn=0)
    for l in (-0.5, -0.25, 0.0, 0.25, 0.5):
        for yawd in (-5.0, 0.0, 5.0):
            R = Rz(g, yawd); pt = g.from_gate_frame([s, l, 0.0])
            for phi in np.radians(np.arange(0, 360, 45)):
                pe = g.from_gate_frame([s + e * math.cos(phi), l + e * math.sin(phi), 0.0])
                st["n"] += 1; warn = False
                for q in sg.SECTORS:
                    a, _ = clf._sector_frame(q, R)
                    w = first_echo(clf, pt + R @ sg.MOUNTS[q], a)
                    if w is None: continue
                    st["echoes"] += 1
                    if not in_window(clf.structure_window(pe + R @ sg.MOUNTS[q], a), w, w + 0.15):
                        st["unexplained"] += 1
                        if w <= DEFAULT.sep.d_warning: st["unexplained_warn"] += 1; warn = True
                st["poses_warn"] += warn
    return ("phantom", sec, e), st

TASKS = {"obs": obs_task, "mask": mask_task, "phantom": phantom_task}
def run(j):
    return TASKS[j[0]](j[1])

if __name__ == "__main__":
    from multiprocessing import Pool
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    jobs = [("obs", (sec, y, s, nz, i)) for i, (sec, y, s, nz) in enumerate(itertools.product(SECTIONS, (-5.0, 0.0, 5.0), (-2.0, -1.8, -1.5, -1.2), (False, True)))]
    jobs += [("mask", (sec, float(s_a), l_a)) for sec in SECTIONS for s_a in np.arange(-6.0, -1.19, 0.4) for l_a in (-3.5, 0.0, 3.5)]
    jobs += [("phantom", (sec, float(s), e)) for sec in SECTIONS for s in np.arange(-5.0, -1.19, 0.25) for e in (0.5, 1.0)]
    out = {}; done = 0; t0 = __import__("time").time()
    log = open(sys.argv[1] + ".progress", "w")
    with Pool(20) as pool:
        for key, st in pool.imap_unordered(run, jobs):
            done += 1; log.write(f"{done}/{len(jobs)} {__import__('time').time() - t0:.0f}s {key} {json.dumps(st, default=int)}\n"); log.flush()
            acc = out.setdefault(str(key), dict.fromkeys(st, 0))
            for k, v in st.items(): acc[k] += v
    for k in sorted(out): print(k, out[k])
    json.dump(out, open(sys.argv[1], "w"), indent=1, default=int)
