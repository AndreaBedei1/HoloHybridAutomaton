"""Exact McNemar on PASS between cells over the same (scenario, seed) pairs (pairing is nominal: engine noise unseeded)."""
import json
from math import comb
from pathlib import Path
RAW = Path(r"F:\Andrea\HoloHybridAutomaton\results\robustness\raw_final")
def cell(proto, prof):
    return {(p.parts[-3], p.parts[-2]): json.loads(p.read_text())["class"] == "PASS"
            for p in (RAW / proto / prof).glob("*/s*/run_metrics.json")}
def mcnemar(a, b):
    keys = sorted(set(a) & set(b)); x = sum(a[k] and not b[k] for k in keys); y = sum(b[k] and not a[k] for k in keys)
    m = x + y; p = min(1.0, 2 * sum(comb(m, i) for i in range(min(x, y) + 1)) / 2 ** m) if m else 1.0
    return len(keys), sum(a[k] for k in keys), sum(b[k] for k in keys), x, y, p
for (pa, pb, prof) in [("B2_MEDIUM", "B2_BLACKOUT", "MEDIUM-ALL"), ("B2_MEDIUM", "B0", "MEDIUM-ALL"),
                       ("B2_BLACKOUT", "B0", "MEDIUM-ALL"), ("B2_CLEAN", "B0", "CLEAN")]:
    n, sa, sb, x, y, p = mcnemar(cell(pa, prof), cell(pb, prof))
    print(f"{pa} vs {pb} / {prof}: n={n} PASS {sa} vs {sb}; discordant {x} vs {y}; exact p = {p:.4f}")
