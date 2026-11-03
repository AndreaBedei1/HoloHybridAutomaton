"""Read-only reconstruction of every non-PASS B2 run of the final campaign: formation error history, counterfactual
recovery thresholds, per-drone time budget, gate state, deadline margin."""
import csv, json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, gframe, RAW, HALF  # noqa

FAM = {"FAILSAFE_HOLD_OR_RETREAT": "FS", "FORMATION_WAIT_REJOIN": "WAIT", "MUTEX_APPROACH": "MX", "MUTEX_YIELD": "MX",
       "MUTEX_PASS": "MX", "FORMATION_RECOVERY": "REC", "COLLISION_AVOIDANCE": "CA", "SEPARATION_WARNING": "SW",
       "FORMATION_FOLLOW": "FOL", "DEGRADED_FORMATION": "DEG"}


def episodes(T, E, e_lost, e_ok, hold):
    """Referee episode logic on a formation-error series; returns (episodes, state at end)."""
    lost, ok_since, eps = False, None, []
    for t, e in zip(T, E):
        if not lost and e > e_lost:
            lost, ok_since = True, None
            eps.append([t, None])
        elif lost:
            if e < e_ok:
                ok_since = t if ok_since is None else ok_since
                if t - ok_since >= hold:
                    eps[-1][1] = ok_since
                    lost = False
            else:
                ok_since = None
    return eps, ("OK" if not lost else "RECOVERING" if ok_since is not None else "LOST")


def per_drone_dev(sc, P):
    tm = sc.template.offsets()
    out = []
    for X in P:
        c = X.mean(axis=0)
        s_c, _l, _k = sc.path.project(c[:2])
        _p, tan, nrm = sc.path.frame_at(s_c)
        rel = X - c
        rel_f = np.stack([rel[:, :2] @ tan, rel[:, :2] @ nrm, rel[:, 2]], axis=1)
        out.append(np.linalg.norm(rel_f - (tm - tm.mean(axis=0)), axis=1))
    return np.array(out)


def main():
    rows = list(csv.DictReader(open(r"F:\Andrea\HoloHybridAutomaton\results\robustness\final\runs.csv", encoding="utf-8")))
    pass_t = defaultdict(list)
    for r in rows:
        if r["class"] == "PASS":
            pass_t[(r["protocol"], r["profile"], r["scenario"])].append(float(r["t_end_s"]))
    sel = [r for r in rows if r["protocol"].startswith("B2") and r["class"] != "PASS"]
    for r in sel:
        d = RAW / r["protocol"] / r["profile"] / r["scenario"] / f"s{r['seed']}"
        rm, sc, ts, n, T, P, st, ev = load(d)
        term = rm.get("terminal") or {}
        E = np.array([float(x.get("form_err") or 0.0) for x in ts]) if sc.formation_enabled else None
        print("=" * 120)
        clean = sorted(pass_t[("B2_CLEAN", "CLEAN", r["scenario"])])
        same = sorted(pass_t[(r["protocol"], r["profile"], r["scenario"])])
        print(f"{r['protocol']}/{r['profile']}/{r['scenario']}/s{r['seed']}  failed={r['failed']}  tags={r['cause_tags']}")
        print(f"  t_ref={term.get('t_ref')} deadline={term.get('deadline_s')} t_end={term.get('t_end')} reason='{term.get('reason')}'"
              f"  disturbance windows={sc.disturbance_windows} faults={[(f['drone'], f['t_on'], f['t_off']) for f in sc.sim.faults]}")
        print(f"  B2_CLEAN PASS t_end={clean}  same-cell PASS t_end={same}")
        if E is not None:
            for th in ((1.2, 0.5, 2.0), (1.5, 1.0, 2.0), (2.0, 1.0, 2.0)):
                eps, state = episodes(T, E, *th)
                print(f"  referee e_lost/e_ok/hold={th}: {len(eps)} episodes, open at end: "
                      f"{[(round(a, 1)) for a, b in eps if b is None]}, state at end {state}")
            last = T >= T[-1] - 30
            seg = E[last]
            t_below1 = 0.0
            for t, e in zip(T[::-1], E[::-1]):
                if e >= 1.0:
                    break
                t_below1 = T[-1] - t
            print(f"  form_err: max {E.max():.2f} at t={T[E.argmax()]:.1f}; last 30 s: first10 {seg[:100].mean():.2f} "
                  f"last10 {seg[-100:].mean():.2f} min {seg.min():.2f} max {seg.max():.2f}; final {E[-1]:.2f}; "
                  f"continuously < 1.0 m for the last {t_below1:.1f} s; frac(last 30 s) < 0.5: {np.mean(seg < 0.5):.2f} < 1.0: {np.mean(seg < 1.0):.2f}")
            dev = per_drone_dev(sc, P)
            print(f"  per-drone deviation at end: {np.round(dev[-1], 2).tolist()}  (mean over last 10 s: {np.round(dev[-100:].mean(axis=0), 2).tolist()})")
        gp = term.get("gate_passed_t")
        if gp is not None:
            print(f"  gate passed t: {gp}")
        for k in range(n):
            recs = [st[k][x] for x in sorted(st[k])]
            fam = Counter(FAM.get(s["mode"], s["mode"]) for s in recs)
            tot = {f: round(c * 0.1, 1) for f, c in fam.most_common()}
            reloc = round(0.1 * sum(1 for s in recs if (s.get("gate") or {}).get("reloc_hold")), 1)
            last = recs[-1]
            fm = last.get("form") or {}
            # last 60 s mode budget
            l60 = Counter(FAM.get(s["mode"], s["mode"]) for s in recs if s["t"] >= recs[-1]["t"] - 60)
            fs_entries = sum(1 for e in ev if e.get("type") == "transition" and e.get("drone") == f"drone_{k}" and e.get("to") == "FAILSAFE_HOLD_OR_RETREAT")
            print(f"  drone_{k}: modes(s)={tot} last60={dict(l60)} FS_entries={fs_entries} reloc_hold={reloc}s end_mode={last['mode']}"
                  f" missing={fm.get('missing')} vacant={fm.get('vacant')} slot_err={fm.get('slot_err')} wait_t={fm.get('wait_t')}"
                  f" gate_s={(last.get('gate') or {}).get('s')}")
        vac = [e for e in ev if e.get("type") in ("SLOT_DECLARED_VACANT", "SLOT_REOCCUPIED")]
        if vac:
            print("  vacancy events:", [(e["type"][5:], e.get("drone"), e.get("slot"), round(e["t"], 1)) for e in vac][:8])


if __name__ == "__main__":
    main()
