"""Analysis of the frozen B2-only campaign (docs/ACCEPTANCE_CRITERIA.md v5, items of v4.3 as adapted by v5).

    python scripts/analyze_b2final.py [raw root] [out dir]

Reads <raw>/<protocol>/<profile>/<scenario>/s<seed>/run_metrics.json and writes <out>/SUMMARY.md, runs.csv and
cells.csv.  Nothing here changes a class: the items of v4.3 are evaluated on the recorded metrics.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import run_sensor_robustness_campaign as camp  # noqa: E402

STALLED_SC = "gate_stalled_occupant"


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def mcnemar_exact(b: int, c: int) -> float:
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * p)


def km(times, events):
    """Kaplan-Meier survival of 'not yet completed': list of (t, S(t)) at the event times."""
    pts = sorted(zip(times, events))
    at_risk, s, out = len(pts), 1.0, []
    i = 0
    while i < len(pts):
        t = pts[i][0]
        d = sum(1 for tt, e in pts if tt == t and e)
        n_t = sum(1 for tt, _e in pts if tt == t)
        if d:
            s *= 1 - d / at_risk
            out.append((t, s))
        at_risk -= n_t
        i += n_t
    return out


def km_median(curve):
    for t, s in curve:
        if s <= 0.5:
            return t
    return None


def load(raw: Path):
    rows = []
    for proto, prof in camp.B2_CELLS:
        for sc in camp.B2_NONGATE_SCENARIOS + camp.B2_GATE_SCENARIOS:
            seeds = camp.B2_SEEDS_GATE if sc in camp.B2_GATE_SCENARIOS else camp.B2_SEEDS_NONGATE
            for s in seeds:
                p = raw / proto / prof / sc / f"s{s}" / "run_metrics.json"
                if not p.exists():
                    rows.append({"protocol": proto, "profile": prof, "scenario": sc, "seed": s, "class": "MISSING"})
                    continue
                m = json.loads(p.read_text(encoding="utf-8"))
                ev = (m.get("P2") or {}).get("events") or {}
                term = m.get("terminal") or {}
                rows.append({
                    "protocol": proto, "profile": prof, "scenario": sc, "seed": s, "class": m.get("class"),
                    "commit": (m.get("code_version") or {}).get("sha"), "attempt": m.get("attempt"),
                    "P1": (m.get("P1") or {}).get("holds"), "contacts": (m.get("P1") or {}).get("collisions"),
                    "P2": (m.get("P2") or {}).get("holds"), "E2_to_free": (ev.get("E2") or {}).get("to_free"),
                    "E3": ev.get("E3"), "E4": ev.get("E4"), "E5": ev.get("E5"), "E7": ev.get("E7"),
                    "nav_exceeded": (m.get("nav_bound") or {}).get("exceeded"),
                    "nav_margin": (m.get("nav_bound") or {}).get("min_margin_m"),
                    "t_end": term.get("t_end"), "outcome": term.get("outcome"),
                    "deadline": (term.get("deadline_terms") and round(sum(term["deadline_terms"].values()), 1)),
                    "replay_consistent": term.get("replay_consistent"),
                    "tags": "|".join(m.get("cause_tags") or []), "invalid_reason": m.get("invalid_reason"),
                    "on_time": m.get("on_time"), "gate_bounded_response": m.get("gate_bounded_response"),
                    "false_vacancy_events": (m.get("conservative_events") or {}).get("false_vacancy"),
                })
    return rows


def unsafe(r) -> bool:
    return bool((r.get("E2_to_free") or 0) or (r.get("E3") or 0) or (r.get("E4") or 0))


def main(argv) -> int:
    raw = Path(argv[1]) if len(argv) > 1 else ROOT / "results" / "robustness" / "raw_b2final"
    out = Path(argv[2]) if len(argv) > 2 else ROOT / "results" / "b2" / "final"
    out.mkdir(parents=True, exist_ok=True)
    rows = load(raw)
    with open(out / "runs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=sorted({k for r in rows for k in r}))
        w.writeheader()
        w.writerows(rows)
    complete = [r for r in rows if r["class"] not in ("MISSING", "INVALID", "SUT_ERROR")]
    commits = Counter(r.get("commit") for r in complete)
    L = ["# B2-only validation campaign: results (criteria v5)", "",
         f"Runs planned {len(rows)}; complete {len(complete)}; "
         f"missing {sum(r['class'] == 'MISSING' for r in rows)}; INVALID {sum(r['class'] == 'INVALID' for r in rows)}; "
         f"SUT_ERROR {sum(r['class'] == 'SUT_ERROR' for r in rows)}.  Commits: {dict(commits)}.", ""]
    # ---- item 1: safety on ground truth
    def blackout_stalled(r):
        return r["scenario"] == STALLED_SC and r["protocol"] == "B2_BLACKOUT"
    judged = [r for r in complete if not blackout_stalled(r)]
    safety_fail = [r for r in judged if not r["P1"] or not r["P2"] or (r["contacts"] or 0) or unsafe(r)]
    mism = [r for r in complete if r["class"] == "REPLAY_MISMATCH"]
    n_gate = sum(1 for r in judged if r["scenario"] in camp.B2_GATE_SCENARIOS)
    L += ["## Item 1: safety (ground truth, every complete run)", "",
          f"* judged runs: {len(judged)} (gate runs {n_gate}); BLACKOUT x {STALLED_SC} reported separately.",
          f"* runs violating P1, P2, with a contact or an unsafe action (E2.to_free, E3, E4): **{len(safety_fail)}**"
          + (f"; 95 % upper bound of the per-run rate {3.0 / len(judged):.4f} (rule of three)" if not safety_fail
             and judged else ""),
          f"* REPLAY_MISMATCH: {len(mism)}", ""]
    for r in safety_fail[:40]:
        L.append(f"  - {r['protocol']} {r['profile']} {r['scenario']} s{r['seed']}: class {r['class']}, P1 {r['P1']}, "
                 f"P2 {r['P2']}, contacts {r['contacts']}, E2.to_free {r['E2_to_free']}, E3 {r['E3']}, E4 {r['E4']}")
    bs = [r for r in complete if blackout_stalled(r)]
    L += ["", f"BLACKOUT x {STALLED_SC} (predicted outside the guarantee, M3f): {len(bs)} runs, P2 violated in "
              f"{sum(1 for r in bs if not r['P2'])}, unsafe actions in {sum(1 for r in bs if unsafe(r))}.", ""]
    # ---- item 2: H6 premise
    in_env = [r for r in complete if r["nav_exceeded"] is not None]
    nb = [r for r in in_env if r["nav_exceeded"]]
    L += ["## Item 2: H6 premise (sent navigation bound contains the true error)", "",
          f"* runs with the bound logged: {len(in_env)}; runs with an exceedance: **{len(nb)}**; smallest margin "
          f"{min((r['nav_margin'] for r in in_env if r['nav_margin'] is not None), default=None)} m", ""]
    for r in nb[:20]:
        L.append(f"  - {r['protocol']} {r['profile']} {r['scenario']} s{r['seed']}: {r['nav_exceeded']} samples")
    # ---- item 3: CLEAN liveness
    clean = [r for r in complete if r["protocol"] == "B2_CLEAN" and r["profile"] == "CLEAN"]
    clean_bad = [r for r in clean if r["scenario"] != STALLED_SC and r["class"] != "PASS"]
    L += ["", "## Item 3: CLEAN liveness (B2_CLEAN / CLEAN)", "",
          f"* runs {len(clean)} ({sum(r['scenario'] != STALLED_SC for r in clean)} outside {STALLED_SC}); "
          f"non-PASS outside it: **{len(clean_bad)}**", ""]
    for r in clean_bad:
        L.append(f"  - {r['scenario']} s{r['seed']}: {r['class']} ({r['tags']})")
    # ---- item 4: per cell
    L += ["", "## Item 4: cells (16 scenarios, gate_stalled_occupant excluded; Kaplan-Meier censored at the v5 "
          "horizon, 3 x the a-priori deadline)", "",
          "| cell | runs | PASS | on time | SAFE_DEGRADED | INCOMPLETE | other | PASS+SD [95 % Wilson] | KM median t_end [s] | PASS late at the gate | PASS with transient false vacancy |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    cell_rows = []
    for proto, prof in camp.B2_CELLS:
        rs = [r for r in complete if r["protocol"] == proto and r["profile"] == prof and r["scenario"] != STALLED_SC]
        c = Counter(r["class"] for r in rs)
        ok = c["PASS"] + c["SAFE_DEGRADED"]
        lo, hi = wilson(ok, len(rs))
        curve = km([float(r["t_end"] or 0.0) for r in rs], [r["class"] in ("PASS", "SAFE_DEGRADED") for r in rs])
        med = km_median(curve)
        other = len(rs) - c["PASS"] - c["SAFE_DEGRADED"] - c["INCOMPLETE"]
        ontime = sum(1 for r in rs if r["class"] == "PASS" and r.get("on_time"))
        late = sum(1 for r in rs if r["class"] == "PASS" and r.get("gate_bounded_response") is False)
        fv = sum(1 for r in rs if r["class"] == "PASS" and (r.get("false_vacancy_events") or 0))
        L.append(f"| {proto} / {prof} | {len(rs)} | {c['PASS']} | {ontime} | {c['SAFE_DEGRADED']} | "
                 f"{c['INCOMPLETE']} | {other} | {ok}/{len(rs)} [{lo:.2f}, {hi:.2f}] | {med} | {late} | {fv} |")
        cell_rows.append({"cell": f"{proto}/{prof}", "runs": len(rs), **{k: c[k] for k in sorted(c)}, "ok": ok,
                          "wilson_lo": round(lo, 3), "wilson_hi": round(hi, 3), "km_median_s": med})
    with open(out / "cells.csv", "w", newline="", encoding="utf-8") as fh:
        keys = sorted({k for r in cell_rows for k in r})
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(cell_rows)
    # ---- item 5: stalled occupant
    st = [r for r in complete if r["scenario"] == STALLED_SC and r["protocol"] != "B2_BLACKOUT"]
    st_ok = [r for r in st if r["P1"] and r["P2"] and not (r["contacts"] or 0) and not unsafe(r) and not (r["E5"] or 0)]
    st_inv = [r for r in rows if r["scenario"] == STALLED_SC and r["class"] == "INVALID"]
    L += ["", f"## Item 5: {STALLED_SC} (v5: safety only over the whole horizon)", "",
          f"* non-BLACKOUT runs {len(st)}; accepted (P1, P2, no contact, E2.to_free = E3 = E4 = E5 = 0): "
          f"**{len(st_ok)}**; class PASS {sum(r['class'] == 'PASS' for r in st)}; INVALID (trigger never met or "
          f"infrastructure): {len(st_inv)}.  Not included in the PASS rates or the completion curves.", ""]
    # ---- item 6: McNemar, two hypotheses, Holm
    def by_key(proto, prof):
        return {(r["scenario"], r["seed"]): r["class"] == "PASS" for r in complete
                if r["protocol"] == proto and r["profile"] == prof and r["scenario"] != STALLED_SC}
    tests = []
    for a, b in ((("B2_MEDIUM", "MEDIUM-ALL"), ("B2_BLACKOUT", "MEDIUM-ALL")),
                 (("B2_CLEAN", "CLEAN"), ("B2_BLACKOUT", "CLEAN"))):
        A, B = by_key(*a), by_key(*b)
        keys = sorted(set(A) & set(B))
        b_ = sum(1 for k in keys if A[k] and not B[k])
        c_ = sum(1 for k in keys if B[k] and not A[k])
        tests.append((f"{a[0]}/{a[1]} vs {b[0]}/{b[1]}", len(keys), b_, c_, mcnemar_exact(b_, c_)))
    order = sorted(range(len(tests)), key=lambda i: tests[i][4])
    adj, prev = [None] * len(tests), 0.0
    for rank, i in enumerate(order):
        prev = max(prev, min(1.0, (len(tests) - rank) * tests[i][4]))
        adj[i] = prev
    L += ["## Item 6: acoustic benefit (B2 vs B2 under outage, endpoint PASS, exact McNemar, Holm)", "",
          "| comparison | pairs | B2 only | outage only | p | p (Holm) |", "|---|---|---|---|---|---|"]
    for (name, n, b_, c_, p), pa in zip(tests, adj):
        L.append(f"| {name} | {n} | {b_} | {c_} | {p:.4f} | {pa:.4f} |")
    L += ["", "Pairing is nominal (HoloOcean's own sensor noise is unseeded).", ""]
    # ---- classes by scenario (all cells)
    L += ["## Classes by scenario and cell", "", "| scenario | " + " | ".join(f"{p}/{q}" for p, q in camp.B2_CELLS) + " |",
          "|---|" + "---|" * len(camp.B2_CELLS)]
    for sc in camp.B2_NONGATE_SCENARIOS + camp.B2_GATE_SCENARIOS:
        cells = []
        for proto, prof in camp.B2_CELLS:
            c = Counter(r["class"] for r in rows if r["scenario"] == sc and r["protocol"] == proto and r["profile"] == prof)
            cells.append(" ".join(f"{k[:4]}{v}" for k, v in sorted(c.items())))
        L.append(f"| {sc} | " + " | ".join(cells) + " |")
    (out / "SUMMARY.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L[:40]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
