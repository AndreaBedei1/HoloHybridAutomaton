"""Near-range obstacle targets (r < 1.2 m) per run: count of target-steps, class, sector, persistence, truth."""
import json, glob, gzip, collections, sys, csv
from multiprocessing import Pool
def one(p):
    r = json.load(open(p))
    if r["class"] == "INVALID": return None
    d = p.replace("\\", "/").rsplit("/", 1)[0]
    n = json.load(open(d + "/run_config.json"))["n_drones"]
    rows = list(csv.DictReader(open(d + "/referee_timeseries.csv")))
    cnt = collections.Counter(); sec = collections.Counter(); runs_len = []; truth = []
    for k in range(n):
        st = [json.loads(x) for x in gzip.open(f"{d}/drone_{k}_state.jsonl.gz", "rt")]
        streak = 0
        for i, s in enumerate(st):
            near = [t for t in (s.get("targets") or []) if t.get("r", 9) < 1.2]
            if near:
                streak += 1
                for t in near:
                    cnt[t.get("cls")] += 1; sec[t.get("pattern")] += 1
                if i < len(rows) and streak == 1:
                    truth.append(min(float(rows[i][f"d_{min(k,j)}_{max(k,j)}"]) for j in range(n) if j != k))
            elif streak:
                runs_len.append(streak); streak = 0
    return {"profile": r["profile"], "protocol": r["protocol"], "scenario": r["scenario"], "steps": sum(cnt.values()),
            "cls": dict(cnt), "sec": dict(sec), "episodes": len(runs_len), "ep_len": runs_len, "truth": truth}
if __name__ == "__main__":
    import psutil; psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    with Pool(20) as pool:
        out = [x for x in pool.map(one, glob.glob(sys.argv[1] + "/*/*/*/s*/run_metrics.json")) if x]
    json.dump(out, open(sys.argv[2], "w"))
    agg = collections.defaultdict(lambda: {"runs": 0, "steps": 0, "ep": 0, "len": [], "sec": collections.Counter(), "cls": collections.Counter(), "truth": []})
    for x in out:
        a = agg[x["profile"]]; a["runs"] += 1; a["steps"] += x["steps"]; a["ep"] += x["episodes"]; a["len"] += x["ep_len"]
        a["sec"].update(x["sec"]); a["cls"].update(x["cls"]); a["truth"] += x["truth"]
    import statistics
    for k, a in sorted(agg.items()):
        L = a["len"]
        print(f"{k:14s} runs {a['runs']:3d} near-target steps {a['steps']:6d} episodes {a['ep']:5d} ({a['ep']/a['runs']:.1f}/run) "
              f"median len {statistics.median(L) if L else 0} steps, max {max(L) if L else 0}; min true dist at onset: "
              f"min {min(a['truth']) if a['truth'] else '-'}; cls {dict(a['cls'])}; sectors {a['sec'].most_common(6)}")
