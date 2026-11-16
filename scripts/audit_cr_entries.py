"""Audit of the entries into the critical region (CR), re-read from the logs: how every drone entered the CR of a judged gate.

    python scripts/audit_cr_entries.py <out.json> <raw dir> [<raw dir> ...]

For every entry edge of a drone's true position into the CR box of a judged gate (referee geometry): ADMITTED (committed and the
stage-2 entry admitted, ``entry`` ADMIT, since its commit edge), PUSHED (committed, never admitted: the hold overridden by
separation warning or collision avoidance, st_lost_middle s5201) or UNCOMMITTED (not committed at the entry), the modes of the
last 10 s, whether another drone's true position was in the CR at the entry, and the occupancy of the CR while the entering drone
stays inside.
"""

import csv
import gzip
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

AVOID = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE")


def one(d: str) -> dict:
    import psutil

    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS
    from holo_fleet.referee.robustness_metrics import cr_entry_kind

    try:
        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass
    p = Path(d)
    m = json.loads((p / "run_metrics.json").read_text(encoding="utf-8"))
    sc = SCENARIOS[m["scenario"]](DEFAULT)
    G = DEFAULT.gate
    R = list(csv.DictReader(open(p / "referee_timeseries.csv", encoding="utf-8")))
    files = sorted(p.glob("drone_*_state.jsonl.gz"), key=lambda f: int(f.name.split("_")[1]))
    states = [[json.loads(x) for x in gzip.open(f, "rt", encoding="utf-8")] for f in files]
    n = len(states)
    t = np.array([float(r["t"]) for r in R])
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in R])
    st_t = [np.array([float(r["t"]) for r in s]) for s in states]
    entries = []
    for g in sc.judged_gates:
        gf = np.array([[g.to_gate_frame(P[i, k]) for k in range(n)] for i in range(len(t))])
        inside = (np.abs(gf[:, :, 0]) <= G.cr_half_len) & (np.abs(gf[:, :, 1]) <= G.cr_half_width) & \
                 (np.abs(gf[:, :, 2]) <= G.cr_half_height)
        for k in range(n):
            for i in np.flatnonzero(inside[1:, k] & ~inside[:-1, k]) + 1:
                j = min(int(np.searchsorted(st_t[k], t[i])), len(states[k]) - 1)
                s = states[k]
                if s[j].get("mode") == "DEAD":
                    continue
                kind = cr_entry_kind(s, j)
                j0 = int(np.searchsorted(st_t[k], t[i] - 10.0))
                modes = sorted({s[y].get("mode") for y in range(j0, j + 1)})
                i_out = i
                while i_out < len(t) and inside[i_out, k]:
                    i_out += 1
                others = inside[i:i_out].sum(axis=1).max() - 1 if i_out > i else 0
                entries.append({"gate": getattr(g, "name", None) or getattr(g, "id", None), "drone": k, "t": round(float(t[i]), 2),
                                "kind": kind, "avoidance_10s": any(x in AVOID for x in modes), "modes_10s": modes,
                                "s_est": (s[j].get("gate") or {}).get("s"), "others_inside_while_in": int(others)})
    return {"run": p.as_posix(), "protocol": m.get("protocol"), "scenario": m.get("scenario"), "seed": m.get("seed"),
            "class": m.get("class"), "P2_holds": (m.get("P2") or {}).get("holds"), "entries": entries}


if __name__ == "__main__":
    runs = sorted(str(q.parent) for r in sys.argv[2:] for q in Path(r).rglob("run_metrics.json"))
    with ProcessPoolExecutor(8) as ex:
        res = list(ex.map(one, runs, chunksize=4))
    Path(sys.argv[1]).write_text(json.dumps(res, indent=1), encoding="utf-8")
    E = [e for r in res for e in r["entries"]]
    from collections import Counter
    print(len(res), "runs;", len(E), "entries;", dict(Counter(e["kind"] for e in E)),
          "; with avoidance in the last 10 s:", dict(Counter(e["kind"] for e in E if e["avoidance_10s"])),
          "; another drone inside while in:", sum(e["others_inside_while_in"] > 0 for e in E),
          "; P2 false runs:", sum(not r["P2_holds"] for r in res))
    for r in res:
        for e in r["entries"]:
            if e["kind"] != "ADMITTED":
                print(r["protocol"], r["scenario"], r["seed"], e)
