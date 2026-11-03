"""Read-only gate-event metrics over the final campaign: commits vs the true CR occupancy, entry gaps, the 7 cases'
echo attribution.  Ground truth from referee_timeseries.csv, onboard states from drone_k_state.jsonl.gz."""
import csv, gzip, json, math, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, r"F:\Andrea\HoloHybridAutomaton")
from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT

RAW = Path(r"F:\Andrea\HoloHybridAutomaton\results\robustness\raw_final")
G = DEFAULT.gate
HALF = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
CASES = {("B0", "MEDIUM-ALL", "lost_drone_mutex", "s0"): (2, 37.3), ("B0", "MEDIUM-ALL", "mutex_deadlock_resolution", "s1"): (1, 15.5),
         ("B2_MEDIUM", "MEDIUM-ALL", "line_parallel_mutex", "s0"): (2, 39.5), ("B2_MEDIUM", "MEDIUM-ONE", "mutex_deadlock_resolution", "s2"): (1, 14.8),
         ("B2_MEDIUM", "MEDIUM-SUBSET", "gate_single", "s1"): (1, 71.0), ("B2_MEDIUM", "MEDIUM-SUBSET", "gate_single", "s2"): (1, 24.7),
         ("B2_MEDIUM", "MEDIUM-SUBSET", "integrated_short", "s1"): (1, 32.2)}


def load(d):
    rm = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    sc = SCENARIOS[rm["scenario"]](DEFAULT)
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    n = len(sc.sim.names)
    T = np.array([float(r["t"]) for r in rows])
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in rows])
    st = []
    for k in range(n):
        with gzip.open(d / f"drone_{k}_state.jsonl.gz", "rt", encoding="utf-8") as f:
            st.append({round(json.loads(x)["t"], 1): json.loads(x) for x in f})
    ev = [json.loads(l) for l in open(d / "events.jsonl", encoding="utf-8")]
    return rm, sc, rows, n, T, P, st, ev


def gframe(g, P):
    R = np.array([g.axis, [-g.axis[1], g.axis[0], 0.0], [0, 0, 1.0]])
    return np.einsum("...j,ij->...i", P - g.center, R)


def main():
    stats = Counter()
    gaps = []
    commit_rows = []
    for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
        d = p.parent
        key = tuple(d.relative_to(RAW).parts)
        try:
            rm, sc, rows, n, T, P, st, ev = load(d)
        except Exception as e:  # noqa
            print("ERR", d, e)
            continue
        if not sc.judged_gates:
            continue
        g = sc.judged_gates[0]
        gf = gframe(g, P)
        inside = np.all(np.abs(gf) <= HALF, axis=-1)
        stats["gate_runs"] += 1
        # commits: transition edge == commit; truth at that instant and at the committer's own CR entry
        for e in ev:
            if e.get("type") != "transition" or e.get("edge") != "commit":
                continue
            k = int(e["drone"].split("_")[1])
            i = int(np.searchsorted(T, e["t"] - 1e-6))
            if i >= len(T):
                continue
            stats["commits"] += 1
            occ_now = [j for j in range(n) if j != k and inside[i, j]]
            if occ_now:
                stats["commit_while_cr_occupied"] += 1
            # committer's first CR entry after the commit
            ent = next((ii for ii in range(i, len(T)) if inside[ii, k]), None)
            if ent is None:
                stats["commit_without_entry"] += 1
                continue
            others_at_entry = [j for j in range(n) if j != k and inside[ent, j]]
            stats["entry_while_occupied"] += int(bool(others_at_entry))
            # gap: time since the last other occupant left the CR, before this entry
            last_out = max((ii for ii in range(0, ent) for j in range(n) if j != k and inside[ii, j]), default=None)
            if last_out is not None:
                gaps.append((T[ent] - T[last_out], key, k))
            commit_rows.append((key, k, e["t"], T[ent], occ_now))
    print(stats)
    gaps.sort()
    print("smallest entry gaps after the previous occupant left [s]:")
    for gp in gaps[:12]:
        print(f"  {gp[0]:6.1f}  {'/'.join(gp[1])} drone_{gp[2]}")
    print("gap percentiles:", np.percentile([x[0] for x in gaps], [0, 5, 50]).round(1))
    # the 7 cases: echo attribution and timings
    print("\nCASES")
    for key, (k, t0) in CASES.items():
        d = RAW.joinpath(*key)
        rm, sc, rows, n, T, P, st, ev = load(d)
        g = sc.judged_gates[0]
        gf = gframe(g, P)
        inside = np.all(np.abs(gf) <= HALF, axis=-1)
        i = int(np.searchsorted(T, t0 - 1e-6))
        s = st[k][round(T[i], 1)]
        yaw = math.radians(s["nav_yaw_deg"])
        sonar = P[i, k] + np.array([math.cos(yaw), math.sin(yaw), 0.0]) * MOUNTS["FRONT"][0]
        ax = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        q = gframe(g, sonar)
        far = max(np.linalg.norm(q - HALF * np.array([a, b, c])) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1))
        w = s["gate"]["exit_witness"]
        print("/".join(key), f"releaser drone_{k} t={T[i]:.1f} witness r={w['range']}  CR far corner {far:.2f} (+eps {far + DEFAULT.env.eps_range_far:.2f})")
        for j in range(n):
            if j == k:
                continue
            v = P[i, j] - sonar
            r = float(np.linalg.norm(v))
            off = math.degrees(math.acos(np.clip(v @ ax / r, -1, 1)))
            print(f"   drone_{j}: centre range {r:5.2f} (near surface >= {r - R_OUT:5.2f}), off-axis {off:5.1f} deg, in CR={bool(inside[i, j])}, s={gf[i, j, 0]:+.2f}")
        occ = [j for j in range(n) if j != k and inside[i, j]][0]
        out_i = next(ii for ii in range(i, len(T)) if not inside[ii, occ])
        free_i = next((ii for ii in range(i, len(T)) if (st[k].get(round(T[ii], 1)) or {}).get("gate", {}).get("occ") == "FREE"), None)
        busy_again = next((ii for ii in range(i, free_i or len(T)) if (st[k].get(round(T[ii], 1)) or {}).get("gate", {}).get("occ") == "BUSY"), None)
        com = [c for c in commit_rows if c[0] == key and c[2] >= T[i] - 1e-6]
        c0 = com[0] if com else None
        print(f"   occupant drone_{occ} leaves CR at {T[out_i]:.1f} (+{T[out_i] - T[i]:.1f} s); releaser latch FREE at "
              f"{'-' if free_i is None else f'{T[free_i]:.1f}'}; back to BUSY before FREE: {busy_again is not None}; "
              f"next commit: {None if c0 is None else f'drone_{c0[1]} at {c0[2]:.1f}, CR entry {c0[3]:.1f}, occupied at commit {c0[4]}'}")
        if c0 is not None:
            cc = c0[1]
            ii0 = int(np.searchsorted(T, c0[2] - 1e-6))
            ii1 = int(np.searchsorted(T, c0[3]))
            dd = np.linalg.norm(P[ii0:ii1 + 1, cc] - P[ii0:ii1 + 1, occ], axis=1)
            print(f"   committer-occupant distance from commit to entry: min {dd.min():.2f} m; occupant s at committer entry {gf[ii1, occ, 0]:+.2f}")


if __name__ == "__main__":
    main()
