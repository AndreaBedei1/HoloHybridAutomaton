"""Cause tags per non-PASS run from the diagnostic outputs (sw_scan.json, nonpass.json, phantom.json)."""
import json, sys, collections
S = sys.argv[1]
sw = {(r["protocol"], r["profile"], r["scenario"], r["seed"]): r for r in json.load(open(S + "/sw_scan.json"))}
npj = {x["id"]: x for x in json.load(open(S + "/nonpass.json"))}
ph = json.load(open(S + "/phantom.json"))
tags = collections.defaultdict(set); per = {}
for key, r in sw.items():
    if r["class"] == "PASS": continue
    rid = f"{key[0]} {key[1]} {key[2]} s{key[3]}"
    nd = npj[rid]
    # C1: a healthy drone with >= 20 s in SEPARATION_WARNING whose entries are mostly multi-sector with true >= 3 m
    for k, dd in r["drones"].items():
        es = dd["sw_entries"]
        spur = [e for e in es if "+" in (e["pattern"] or "") and e["true_min"] >= 3.0]
        if dd["mode_time"].get("SEPARATION_WARNING", 0) >= 20 and len(spur) >= 0.8 * max(1, len(es)):
            tags[rid].add("C1 corner false warning")
    if key[2] == "lost_drone_timeout":
        if any(not v["vac"] or v["end_form"]["vacant"] == [] for v in nd["drones"].values()):
            tags[rid].add("P3-deg: a healthy drone without the vacancy at the deadline")
        if any(v["reocc"] for v in nd["drones"].values()):
            tags[rid].add("C3 vacancy revoked (re-occupation)")
    if any(v["CA"] > 0 for v in nd["drones"].values()) and "MEDIUM" in key[1]:
        tags[rid].add("C4 CA entries (phantom/near echoes) present")
    per[rid] = nd["reason"]
cnt = collections.Counter(t for v in tags.values() for t in v)
for t, c in cnt.most_common(): print(f"{c:3d}  {t}")
print(len(per), "non-PASS runs tagged")
