"""Read-only forensics of latch releases (BUSY -> EXITING/FREE) while another drone is truly inside the CR."""
import csv, gzip, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, r"F:\Andrea\HoloHybridAutomaton")
from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS

RAW = Path(r"F:\Andrea\HoloHybridAutomaton\results\robustness\raw_final")
G = DEFAULT.gate
HALF = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])


def states(d, k):
    with gzip.open(d / f"drone_{k}_state.jsonl.gz", "rt", encoding="utf-8") as f:
        return [json.loads(x) for x in f]


def analyse(d, verbose):
    rm = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    sc = SCENARIOS[rm["scenario"]](DEFAULT)
    if not sc.judged_gates:
        return []
    g = sc.judged_gates[0]
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    n = len(sc.sim.names)
    T = np.array([float(r["t"]) for r in rows])
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in rows])
    R = np.array([g.axis, [-g.axis[1], g.axis[0], 0.0], [0, 0, 1.0]])
    gf = np.einsum("mkj,ij->mki", P - g.center, R)
    inside = np.all(np.abs(gf) <= HALF, axis=-1)
    St = [{round(s["t"], 1): s for s in states(d, k)} for k in range(n)]
    out = []
    for k in range(n):
        prev = None
        for i, t in enumerate(T):
            s = St[k].get(round(t, 1))
            if s is None:
                continue
            gs = s.get("gate") or {}
            occ = gs.get("occ")
            if prev and prev[0] == "BUSY" and prev[1] == gs.get("id") and occ in ("EXITING", "FREE"):
                occupants = [j for j in range(n) if j != k and inside[i, j]]
                w = gs.get("exit_witness") or {}
                kind = "TIMEOUT" if occ == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")
                if occupants:
                    out.append(dict(run=str(d.relative_to(RAW)), k=k, t=t, i=i, kind=kind, w=w, occ=occupants,
                                    prevdiag=prev[2]))
                    if verbose:
                        report(d, sc, g, T, gf, inside, St, rows, k, i, occupants, w, prev[2], n)
            prev = (occ, gs.get("id"), gs.get("acoustic"))
    return out


def report(d, sc, g, T, gf, inside, St, rows, k, i, occ, w, diag, n):
    t = T[i]
    print("=" * 110)
    print(d.relative_to(RAW), f"release by drone_{k} at t={t:.1f}  witness={w}  truly inside: {occ}")
    sk = St[k][round(t, 1)]
    print(f"  releaser drone_{k}: mode={sk['mode']} committed={sk['committed']} est s,l=({sk['gate']['s']},{sk['gate']['l']})"
          f" true s,l,dz={np.round(gf[i, k], 2).tolist()} at_queue={sk['gate']['at_queue']} has_prio={sk['gate']['has_prio']}"
          f" decision={sk['gate']['decision']}")
    print(f"  acoustic diag (last BUSY step): {diag}  fresh fixes of: {sk['form'].get('acoustic_fixed')}")
    print(f"  sonar targets at release: {sk['targets']}")
    print(f"  sector echoes: { {s: v['echoes'] for s, v in sk['sectors'].items() if v['echoes']} }")
    for j in occ:
        sj = St[j].get(round(t, 1), {})
        print(f"  occupant drone_{j}: mode={sj.get('mode')} committed={sj.get('committed')} true s,l,dz={np.round(gf[i, j], 2).tolist()}"
              f" est s={sj.get('gate', {}).get('s')} v_cmd={sj.get('v_cmd')}")
    # occupant exit time
    for j in occ:
        jj = i
        while jj < len(T) and inside[jj, j]:
            jj += 1
        print(f"  drone_{j} leaves the CR at t={T[min(jj, len(T)-1)]:.1f} (inside since t={T[max(np.argmax(inside[:i+1, j][::-1] == False), 0)] if False else ''})")
    # timeline after release for every other drone (not the occupant): occ state, mode, commit, true s, distance
    t_end = t + 30.0
    print("  timeline (every 0.5 s while something changes):")
    last = None
    for ii in range(max(i - 20, 0), len(T)):
        if T[ii] > t_end:
            break
        line = []
        for m in range(n):
            sm = St[m].get(round(T[ii], 1))
            if sm is None:
                line.append("-")
                continue
            gsm = sm.get("gate") or {}
            line.append(f"d{m}:{sm['mode'][:9]:9s} occ={str(gsm.get('occ'))[:4]:4s} p={int(bool(gsm.get('has_prio')))} c={int(sm['committed'])}"
                        f" s={gf[ii, m][0]:+5.2f} in={int(inside[ii, m])}")
        dm = min(float(rows[ii][f"d_{a}_{b}"]) for a in range(n) for b in range(a + 1, n) if f"d_{a}_{b}" in rows[ii])
        key = tuple(x.split(" s=")[0] for x in line)
        if key != last or abs(T[ii] - t) < 0.05:
            print(f"   t={T[ii]:6.1f} {' | '.join(line)}  dmin={dm:.2f} occ_true={rows[ii].get('occ_' + g.gate_id)}")
            last = key


if __name__ == "__main__":
    verbose = "-v" in sys.argv
    allc = []
    for p in sorted(RAW.glob("*/*/*/s*/run_metrics.json")):
        try:
            allc += analyse(p.parent, verbose)
        except Exception as e:  # noqa
            print("ERR", p.parent, e)
    print("\nSUMMARY")
    from collections import Counter
    print(Counter((c["kind"]) for c in allc))
    for c in allc:
        print(c["run"], f"drone_{c['k']}", c["t"], c["kind"], c["w"], "occupants", c["occ"])
