"""Per run, per drone: SEPARATION_WARNING entries vs truth, mode times, vacancy declarations (referee-side analysis)."""
import csv, glob, gzip, json, os, sys
from multiprocessing import Pool
import numpy as np
import psutil

R = sys.argv[1]
OUT = sys.argv[2]


def one(p):
    d = os.path.dirname(p)
    r = json.load(open(p))
    if r.get("class") == "INVALID":
        return None
    rows = list(csv.DictReader(open(os.path.join(d, "referee_timeseries.csv"))))
    rc = json.load(open(os.path.join(d, "run_config.json")))
    n = rc["n_drones"]
    lost = {int(f["drone"].split("_")[1]) for f in rc.get("faults", []) if f.get("t_off") is None}
    t_fault = min([f["t_on"] for f in rc.get("faults", [])], default=None)
    D = np.full((len(rows), n, n), np.inf)
    for i, row in enumerate(rows):
        for a in range(n):
            for b in range(a + 1, n):
                v = row.get(f"d_{a}_{b}")
                if v not in (None, ""):
                    D[i, a, b] = D[i, b, a] = float(v)
    ev = [json.loads(x) for x in open(os.path.join(d, "events.jsonl"))]
    vac = {}
    for e in ev:
        if e.get("type") == "SLOT_DECLARED_VACANT":
            vac.setdefault(e["drone"], []).append(round(e["t"], 1))
    res = {"protocol": r["protocol"], "profile": r["profile"], "scenario": r["scenario"], "seed": r["seed"],
           "class": r["class"], "failed": r.get("failed_requirements"), "drones": {}}
    for k in range(n):
        if k in lost:
            continue
        f = os.path.join(d, f"drone_{k}_state.jsonl.gz")
        st = [json.loads(x) for x in gzip.open(f, "rt")]
        m = min(len(st), len(rows))
        modes = [s["mode"] for s in st[:m]]
        dt = 0.1
        tm = {}
        for md in modes:
            tm[md] = tm.get(md, 0) + dt
        entries = []
        for i in range(1, m):
            if modes[i] == "SEPARATION_WARNING" and modes[i - 1] != "SEPARATION_WARNING":
                s = st[i]
                tg = sorted(s.get("targets") or [], key=lambda x: x.get("d_lower", 1e9))
                trig = tg[0] if tg else {}
                true_min = float(np.min(D[i, k])) if n > 1 else None
                entries.append({"t": round(s["t"], 1), "d_min": s.get("d_min"), "pattern": trig.get("pattern"),
                                "r": trig.get("r"), "true_min": round(true_min, 2)})
        res["drones"][k] = {"mode_time": {a: round(b, 1) for a, b in tm.items()}, "sw_entries": entries,
                            "vacant_declared": vac.get(f"drone_{k}", []),
                            "end_mode": modes[-1] if modes else None}
    return res


if __name__ == "__main__":
    try:
        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass
    files = sorted(glob.glob(f"{R}/*/*/*/s*/run_metrics.json"))
    with Pool(20) as pool:
        out = [x for x in pool.map(one, files, chunksize=2) if x]
    json.dump(out, open(OUT, "w"))
    print(len(out), "runs")
