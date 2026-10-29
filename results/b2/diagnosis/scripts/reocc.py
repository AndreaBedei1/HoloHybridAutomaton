"""At each SLOT_REOCCUPIED: which TRUE drone is nearest to the re-occupied slot (observer's true pose + template offset)."""
import csv, glob, json, collections
import numpy as np
from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS
res = collections.Counter(); ex = []
for p in sorted(glob.glob("results/robustness/raw_b2/*/*/*/s*/run_metrics.json")):
    r = json.load(open(p))
    if r["class"] == "INVALID": continue
    d = p.replace("\\", "/").rsplit("/", 1)[0]
    ev = [json.loads(l) for l in open(d + "/events.jsonl")]
    ro = [e for e in ev if e.get("type") == "SLOT_REOCCUPIED"]
    if not ro: continue
    sc = SCENARIOS[r["scenario"]](DEFAULT)
    rc = json.load(open(d + "/run_config.json"))
    lost = {int(f["drone"].split("_")[1]) for f in rc.get("faults", []) if f.get("t_off") is None}
    rows = list(csv.DictReader(open(d + "/referee_timeseries.csv")))
    for e in ro:
        k = int(e["drone"].split("_")[1]); sl = e["slot"]
        i = min(int(round(e["t"] / 0.1)) - 1, len(rows) - 1)
        P = np.array([[float(rows[i][f"{a}{j}"]) for a in "xyz"] for j in range(rc["n_drones"])])
        plan = sc.plans[k]; me, other = plan.slots[plan.slot_index], plan.slots[sl]
        s_k, _, _ = sc.path.project(P[k][:2]); _p, tan, nrm = sc.path.frame_at(s_k)
        off = (other.along - me.along) * np.append(tan, 0) + (other.lateral - me.lateral) * np.append(nrm, 0) + np.array([0, 0, other.dz - me.dz])
        exp = P[k] + off
        dist = np.linalg.norm(P - exp, axis=1); dist[k] = np.inf
        j = int(np.argmin(dist))
        owner = [q for q in range(len(sc.plans)) if sc.plans[q].slot_index == sl and sc.plans[q].path is plan.path][0]
        kind = "owner(lost)" if j == owner and owner in lost else ("owner" if j == owner else "OTHER drone")
        res[(r["protocol"].split("_")[0], r["scenario"], kind, "owner_lost" if owner in lost else "owner_healthy")] += 1
        if len(ex) < 8 and kind == "OTHER drone": ex.append((r["protocol"], r["profile"], r["scenario"], r["seed"], round(e["t"], 1), f"obs d{k} slot {sl} nearest true d{j} at {dist[j]:.2f} m; owner d{owner} at {dist[owner]:.2f} m"))
for k, v in sorted(res.items()): print(k, v)
for x in ex: print(x)
