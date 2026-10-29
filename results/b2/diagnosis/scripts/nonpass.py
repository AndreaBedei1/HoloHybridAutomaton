import csv, glob, gzip, json, os, sys, collections
from multiprocessing import Pool
import psutil
R = sys.argv[1]
INF = ("FORMATION_FOLLOW", "FORMATION_WAIT_REJOIN", "DEGRADED_FORMATION")
def one(p):
    d = os.path.dirname(p); r = json.load(open(p))
    if r.get("class") in ("PASS", "INVALID"): return None
    rc = json.load(open(os.path.join(d, "run_config.json")))
    lost = {int(f["drone"].split("_")[1]) for f in rc.get("faults", []) if f.get("t_off") is None}
    t0 = min([f["t_on"] for f in rc.get("faults", [])], default=0.0)
    ev = [json.loads(x) for x in open(os.path.join(d, "events.jsonl"))]
    tr = [e for e in ev if e.get("type") == "transition"]
    out = {"id": f"{r['protocol']} {r['profile']} {r['scenario']} s{r['seed']}", "class": r["class"],
           "reason": (r.get("terminal") or {}).get("reason", "").replace("required outcomes not all reached: ", ""), "drones": {}}
    for k in range(rc["n_drones"]):
        if k in lost: continue
        st = [json.loads(x) for x in gzip.open(os.path.join(d, f"drone_{k}_state.jsonl.gz"), "rt")]
        tm = collections.Counter(); 
        for s in st:
            if s["t"] >= t0: tm[s["mode"]] += 0.1
        reasons = collections.Counter(e.get("reason") for e in tr if e.get("drone") == f"drone_{k}")
        vac = [round(e["t"], 1) for e in ev if e.get("type") == "SLOT_DECLARED_VACANT" and e.get("drone") == f"drone_{k}"]
        reocc = [round(e["t"], 1) for e in ev if e.get("type") == "SLOT_REOCCUPIED" and e.get("drone") == f"drone_{k}"]
        out["drones"][k] = {"inform": round(sum(tm[m] for m in INF), 1), "SW": round(tm["SEPARATION_WARNING"], 1),
                            "REC": round(tm["FORMATION_RECOVERY"], 1), "FS": round(tm["FAILSAFE_HOLD_OR_RETREAT"], 1),
                            "CA": round(tm["COLLISION_AVOIDANCE"], 1), "WAIT": round(tm["FORMATION_WAIT_REJOIN"], 1),
                            "vac": vac, "reocc": reocc, "end": st[-1]["mode"], "end_form": {x: st[-1]["form"].get(x) for x in ("missing", "vacant", "form_err")},
                            "reasons": dict(reasons.most_common(6))}
    return out
if __name__ == "__main__":
    psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    files = sorted(glob.glob(f"{R}/*/*/*/s*/run_metrics.json"))
    with Pool(20) as pool:
        res = [x for x in pool.map(one, files) if x]
    json.dump(res, open(sys.argv[2], "w"), indent=0)
    for x in res:
        print(x["id"], x["class"][:4], "|", x["reason"])
        for k, v in x["drones"].items():
            print(f"   d{k} inForm {v['inform']:5.1f} SW {v['SW']:5.1f} REC {v['REC']:5.1f} FS {v['FS']:4.1f} CA {v['CA']:4.1f} WAIT {v['WAIT']:5.1f} vac {v['vac']} reocc {v['reocc']} end {v['end'][:14]} {v['end_form']}")
