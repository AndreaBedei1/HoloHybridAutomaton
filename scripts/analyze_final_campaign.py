"""Analysis of the final campaign against the criteria v8 (docs/ACCEPTANCE_CRITERIA.md), from the run_metrics.json written by the
frozen referee:  python scripts/analyze_final_campaign.py results/robustness/raw_v11_final > <report.md>"""

import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from final_campaign import ABLATION_SCENARIOS, GATE, matrix  # noqa: E402

from holo_fleet.sim.scenarios import STRESS_BATCH  # noqa: E402

COMPLETE = ("PASS", "P3_LATE")
SAFETY = ("SAFETY_FAILURE", "UNSAFE_ACTION", "SUT_ERROR", "REPLAY_MISMATCH")
PROBES = ("st_lost_at_queue", "st_lost_committed", "st_lost_after_exit")


def cp_upper(k: int, n: int, a: float = 0.05) -> float:
    """One-sided Clopper-Pearson upper bound of a binomial rate."""
    if n == 0:
        return 1.0
    if k >= n:
        return 1.0
    lo, hi = k / n, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        cdf = sum(math.comb(n, i) * mid ** i * (1 - mid) ** (n - i) for i in range(k + 1))
        lo, hi = (mid, hi) if cdf > a else (lo, mid)
    return hi


def load(raw: Path) -> dict:
    out = {}
    for p in raw.rglob("run_metrics.json"):
        m = json.loads(p.read_text(encoding="utf-8"))
        out[(m.get("protocol"), m.get("profile"), m.get("scenario"), int(m.get("seed")))] = m
    return out


def block(key) -> str:
    proto, prof, sc, s = key
    if 5000 <= s <= 5005:
        return "A"
    if 5100 <= s <= 5139:
        return "B"
    if 5200 <= s <= 5204:
        return "C"
    if 5300 <= s <= 5302:
        return "D"
    return "E"


def main(raw: Path) -> None:
    runs = load(raw)
    plan = matrix()
    missing = [k for k in plan if k not in runs]
    L = [f"# Final campaign, criteria v8 ({raw.as_posix()})", "", f"Runs planned {len(plan)}, found {len(runs)}, missing {len(missing)} {missing[:5]}", ""]
    cls_all = Counter(m["class"] for m in runs.values())
    L.append(f"Classes, all runs: {dict(cls_all)}")
    # ------------------------------------------------------------------ item 2, safety
    bad = [k for k, m in runs.items() if m["class"] in SAFETY]
    p1 = [k for k, m in runs.items() if not (m.get("P1") or {}).get("holds", True)]
    p2 = [k for k, m in runs.items() if not (m.get("P2") or {}).get("holds", True)]
    n = len(runs)
    L += ["", "## Item 2, safety", f"- SAFETY_FAILURE, UNSAFE_ACTION, SUT_ERROR, REPLAY_MISMATCH: {len(bad)} {bad[:5]}",
          f"- P1 violated in {len(p1)} runs, P2 violated in {len(p2)} runs, of {n}",
          f"- one-sided 95 % Clopper-Pearson upper bound of the per-run rate of a safety failure: {cp_upper(len(bad) + len(set(p1) | set(p2)), n):.4f}",
          f"- smallest true separation over the campaign: {min((m.get('P1') or {}).get('min_true_separation_m', 99) for m in runs.values()):.3f} m",
          f"- dead-hull contacts recorded (criteria v7, not contacts of the fleet): {sum((m.get('P1') or {}).get('dead_hull_contacts', 0) for m in runs.values())}"]
    # ------------------------------------------------------------------ item 3, INVALID
    inv = [(k, runs[k].get("invalid_reason")) for k in runs if runs[k]["class"] == "INVALID"]
    L += ["", "## Item 3, INVALID", f"- {len(inv)} {inv[:5]}"]
    # ------------------------------------------------------------------ item 4, monitors
    pv = [k for k, m in runs.items() if (m.get("premise_v") or {}).get("live_declared_dead")]
    val = [k for k, m in runs.items() if m.get("validation_profile")]
    pv_val = [k for k in pv if k in val]
    wv = [k for k, m in runs.items() if m.get("wrong_validated_fits")]
    gn = [k for k, m in runs.items() if (m.get("gate_nav") or {}).get("committed_beyond")]
    nb = [k for k, m in runs.items() if (m.get("nav_bound") or {}).get("exceeded")]
    fair = [k for k, m in runs.items() if (m.get("fairness") or {}).get("holds") is False]
    fg = sorted((m.get("fairness") or {}).get("longest_gap_s", 0.0) for m in runs.values())
    L += ["", "## Item 4, monitors",
          f"- premise V (a live drone declared dead): {len(pv)} runs, {len(pv_val)} in the validation profiles",
          f"- wrong validated fits: {len(wv)} runs ({sum(runs[k]['wrong_validated_fits'] for k in wv)} fits): {[k[1:] for k in wv][:12]}",
          f"- gate-frame navigation premise exceeded while committed: {len(gn)} runs; H6 navigation bound exceeded: {len(nb)} runs",
          f"- fairness gap above 35 s: {len(fair)} runs {[k[:3] for k in fair][:8]}; gap p50 {fg[len(fg)//2]:.1f} s, max {fg[-1]:.1f} s"]
    # rule D decisions are in the state records (gate.rule_d): counted by scripts on request, summarised in the report
    # ------------------------------------------------------------------ item 5, block A
    L += ["", "## Item 5, standard set (block A)"]
    thr = {"CLEAN": 65, "LOW-ALL": 65, "MEDIUM-ALL": 63}
    for prof in ("CLEAN", "LOW-ALL", "MEDIUM-ALL"):
        ks = [k for k in runs if block(k) == "A" and k[1] == prof]
        c = Counter(runs[k]["class"] for k in ks)
        comp = sum(c[x] for x in COMPLETE)
        L.append(f"- {prof}: {comp} of {len(ks)} complete (threshold {thr[prof]}) {'MET' if comp >= thr[prof] else 'NOT MET'}; classes {dict(c)}")
        for k in ks:
            if runs[k]["class"] not in ("PASS",):
                L.append(f"  - {k[2]} s{k[3]}: {runs[k]['class']} {runs[k].get('failed_requirements')} {runs[k].get('cause_tags')}")
    # ------------------------------------------------------------------ item 6, block B
    ks = [k for k in runs if block(k) == "B"]
    c = Counter(runs[k]["class"] for k in ks)
    comp = sum(c[x] for x in COMPLETE)
    L += ["", "## Item 6, lost-drone family (block B)",
          f"- {comp} of {len(ks)} complete (threshold 58) {'MET' if comp >= 58 else 'NOT MET'}; classes {dict(c)}; "
          f"95 % upper bound of the failure rate {cp_upper(len(ks) - comp, len(ks)):.3f}"]
    for k in ks:
        if runs[k]["class"] != "PASS":
            L.append(f"  - {k[2]} s{k[3]}: {runs[k]['class']} {runs[k].get('failed_requirements')} {runs[k].get('cause_tags')}")
    # ------------------------------------------------------------------ item 7, block C
    ks = [k for k in runs if block(k) == "C"]
    inside = [k for k in ks if not k[2].startswith("st_bd_") and k[2] not in PROBES]
    comp = sum(runs[k]["class"] in COMPLETE for k in inside)
    p12 = [k for k in ks if runs[k]["class"] in SAFETY or k in p1 or k in p2]
    L += ["", "## Item 7, stress batch (block C)", f"- P1 and P2 violated in {len(p12)} of {len(ks)}",
          f"- inside the envelope: {comp} of {len(inside)} complete (threshold 81) {'MET' if comp >= 81 else 'NOT MET'}"]
    per = defaultdict(Counter)
    for k in ks:
        per[k[2]][runs[k]["class"]] += 1
    for sc in STRESS_BATCH:
        tag = "beyond the envelope" if sc.startswith("st_bd_") else ("declared-limit probe" if sc in PROBES else "inside")
        L.append(f"  - {sc} ({tag}): {dict(per[sc])}")
    # ------------------------------------------------------------------ item 8, blocks D and E
    L += ["", "## Item 8, ablation (block D) against B2 (block A, MEDIUM)"]
    for sc in ABLATION_SCENARIOS:
        row = []
        for proto in ("B0", "B2NP_MEDIUM"):
            c = Counter(runs[k]["class"] for k in runs if block(k) == "D" and k[0] == proto and k[2] == sc)
            row.append(f"{proto} {dict(c)}")
        c = Counter(runs[k]["class"] for k in runs if block(k) == "A" and k[1] == "MEDIUM-ALL" and k[2] == sc)
        row.append(f"B2 {dict(c)}")
        L.append(f"- {sc}: " + "; ".join(row))
    L += ["", "## Item 8, sensitivity (block E)"]
    for proto, prof in (("B2_MEDIUM", "MEDIUM_HB3-ALL"), ("B2_MEDIUM", "MEDIUM_HB4-ALL"), ("B2_MEDIUM_HT", "MEDIUM-ALL")):
        for sc in ("gate_single", "lost_drone_mutex"):
            ks = [k for k in runs if block(k) == "E" and k[0] == proto and k[1] == prof and k[2] == sc]
            L.append(f"- {proto} {prof} {sc}: {dict(Counter(runs[k]['class'] for k in ks))}")
    # ------------------------------------------------------------------ item 9, P3
    L += ["", "## Item 9, P3 verdicts"]
    for part in ("P3a", "P3b", "P3c"):
        cnt = Counter()
        for m in runs.values():
            q = (m.get("p3") or {}).get(part)
            if isinstance(q, dict) and "verdict" not in q:
                cnt.update(v.get("verdict") for v in q.values())
            elif isinstance(q, dict):
                cnt[q.get("verdict")] += 1
        L.append(f"- {part}: {dict(cnt)}")
    slack = []
    for m in runs.values():
        b = (m.get("p3") or {}).get("P3b") or {}
        if b.get("verdict") == "MET" and b.get("recovered") is not None:
            slack.append(float(b["deadline"]) - float(b["recovered"]) - 8.9)
    slack.sort()
    if slack:
        L.append(f"- P3b slack to the deadline (s), MET runs: n {len(slack)}, min {slack[0]:.1f}, p10 {slack[len(slack)//10]:.1f}, median {slack[len(slack)//2]:.1f}")
    # ------------------------------------------------------------------ item 10, every non-complete run
    L += ["", "## Item 10, every run not PASS"]
    for k in sorted(runs):
        m = runs[k]
        if m["class"] != "PASS":
            q = m.get("p3") or {}
            prem = {p: v.get("premises") for p, v in (q.get("P3a") or {}).items() if v.get("premises")}
            L.append(f"- [{block(k)}] {k[0]} {k[1]} {k[2]} s{k[3]}: {m['class']}; failed {m.get('failed_requirements')}; tags {m.get('cause_tags')}; "
                     f"P3b {(q.get('P3b') or {}).get('verdict')} {(q.get('P3b') or {}).get('premises')}; P3a premises {prem}")
    print("\n".join(L))


if __name__ == "__main__":
    main(Path(sys.argv[1]))
