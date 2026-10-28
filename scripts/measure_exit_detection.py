"""Empirical detection of a drone that has left the CR (results/gate/exit_detection.json; run from results/robustness).
in CLEAN runs.  Truth positions (referee) only judge; observer heading from its log (CLEAN, compass error ~0).
Per (sector, range bin, line-of-sight clear fraction): captures with the target in the cone vs captures with an echo
within 0.6 m of the true near-surface range (any class; and obstacle classes only)."""
import csv, gzip, json, glob, os, sys, math, collections
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS
from holo_fleet.perception import sonar_geometry as sg
LATERAL = "--lateral" in sys.argv     # only observers at a lateral queue point (|l| >= 2 m)
GATE_SC = ("gate_single", "integrated_short", "line_parallel_mutex", "lost_drone_mutex", "mutex_deadlock_resolution")
def hits(a, b, box):
    A = box.axes.T @ (a - box.center); B = box.axes.T @ (b - box.center); d = B - A; t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(A[k]) > box.half[k]: return False
        else:
            u, v = (-box.half[k] - A[k]) / d[k], (box.half[k] - A[k]) / d[k]; t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 > t1: return False
    return True
def one(args):
    run, geo = args
    import psutil; psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"], **geo)[0]
    truth = {round(float(r["t"]), 1): r for r in csv.DictReader(open(run + "/referee_timeseries.csv"))}
    ids = sorted(int(os.path.basename(f).split("_")[1]) for f in glob.glob(run + "/drone_*_state.jsonl.gz"))
    acc = collections.Counter()
    for i in ids:
        for l in gzip.open(f"{run}/drone_{i}_state.jsonl.gz", "rt"):
            r = json.loads(l)
            if round(r["t"] * 10) % 3: continue                        # every 0.3 s
            tr = truth.get(round(r["t"], 1))
            if tr is None or tr.get("absent", "") not in ("", "[]", "0"): pass
            if tr is None: continue
            P = {k: np.array([float(tr[f"x{k}"]), float(tr[f"y{k}"]), float(tr[f"z{k}"])]) for k in ids}
            fi = g.to_gate_frame(P[i])
            if fi[0] > -DEFAULT.gate.cr_half_len or (LATERAL and abs(fi[1]) < 2.0): continue   # observer before the CR
            yaw = math.radians(r["nav_yaw_deg"]); R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
            for j in ids:
                if j == i: continue
                sj = g.to_gate_frame(P[j])
                if not (DEFAULT.gate.cr_half_len < sj[0] < 12.0): continue       # target has left the CR
                for sec in sg.SECTORS:
                    son = P[i] + R @ sg.MOUNTS[sec]; d = P[j] - son; L = float(np.linalg.norm(d))
                    ang = math.degrees(math.acos(np.clip(d @ (R @ sg.AXES[sec]) / L, -1, 1)))
                    if ang > 60.0 or L > 12.5: continue                   # target centre in the nominal cone
                    rt = L - 0.25
                    left = np.array([-g.axis[1], g.axis[0], 0.0])
                    samples = [P[j] + dl * left + np.array([0, 0, dz]) for dl in (-0.29, 0.0, 0.29) for dz in (-0.12, 0.0, 0.12)]
                    clear = sum(not any(hits(son, q, b) for b in g.bars) for q in samples)
                    cb = "clear" if clear == 9 else ("partial" if clear else "blocked")
                    rb = f"{int(rt)}-{int(rt) + 1}"
                    ech = r["sectors"][sec]["echoes"] if r["sectors"].get(sec) else []
                    near = [e for e in ech if abs(e[0] - rt) <= 0.6]
                    acc[(sec, rb, cb, "n")] += 1
                    acc[(sec, rb, cb, "echo")] += bool(near)
                    acc[(sec, rb, cb, "obstacle")] += any(e[1] in ("DYNAMIC", "UNKNOWN", "UNCONFIRMED") for e in near)
    return acc
if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = []
    for root, geo in (("raw_gate_check", {"inner_size": {}, "bar_section": {}}), ("raw_gate_bars", {})):
        for sc in GATE_SC:
            jobs += [(run, geo) for run in sorted(glob.glob(f"{root}/B0/CLEAN/{sc}/s*"))]
    tot = {}
    with Pool(20) as p:
        for (run, geo), acc in zip(jobs, p.map(one, jobs)):
            key = run.replace("\\", "/").split("/")[0]
            t = tot.setdefault(key, collections.Counter()); t.update(acc)
    json.dump({k: {"|".join(kk): v for kk, v in t.items()} for k, t in tot.items()}, open([a for a in sys.argv[1:] if a != "--lateral"][0], "w"), indent=0)
    for key, t in tot.items():
        print("==", key)
        rows = sorted({k[:3] for k in t})
        for sec, rb, cb in sorted(rows, key=lambda x: (x[0], int(x[1].split("-")[0]), x[2])):
            n = t[(sec, rb, cb, "n")]
            if n >= 10: print(f"  {sec:6s} {rb:6s} {cb:8s} n {n:5d} echo {t[(sec, rb, cb, 'echo')] / n:5.2f} obstacle {t[(sec, rb, cb, 'obstacle')] / n:5.2f}")
