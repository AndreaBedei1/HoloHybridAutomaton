"""True formation-error decomposition (along / lateral / vertical) of the worst drone, end of run and time-averaged."""
import csv, glob, json, sys
import numpy as np
from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS
for sc_name in sys.argv[1:]:
    sc = SCENARIOS[sc_name](DEFAULT)
    tm = sc.template.offsets(); tm = tm - tm.mean(axis=0)
    for p in sorted(glob.glob(f"results/robustness/raw_b2/*/*/{sc_name}/s*/run_metrics.json")):
        r = json.load(open(p))
        if r["class"] == "INVALID" or "MEDIUM" not in r["profile"] and r["profile"] != "CLEAN": continue
        d = p.replace("\\", "/").rsplit("/", 1)[0]
        rows = list(csv.DictReader(open(d + "/referee_timeseries.csv")))
        n = len(tm); A = []; L = []; E = []
        for row in rows[len(rows)//2:]:
            P = np.array([[float(row[f"{a}{k}"]) for a in "xyz"] for k in range(n)])
            c = P.mean(axis=0); s_c, _, _ = sc.path.project(c[:2]); _p, tan, nrm = sc.path.frame_at(s_c)
            rel = P - c; rf = np.stack([rel[:, :2] @ tan, rel[:, :2] @ nrm, rel[:, 2]], axis=1) - tm
            k = int(np.argmax(np.linalg.norm(rf, axis=1)))
            A.append(abs(rf[k, 0])); L.append(abs(rf[k, 1])); E.append(np.linalg.norm(rf[k]))
        print(f"{sc_name:16s} {r['protocol']:11s} {r['profile']:13s} s{r['seed']} {r['class'][:4]}  2nd half: worst-drone |along| {np.mean(A):.2f}  |lateral| {np.mean(L):.2f}  err {np.mean(E):.2f}  (end err {E[-1]:.2f})")
