"""Held-out configurations of the validation experiments, chosen from the pilot by a rule fixed before the pilot results were read.

    python scripts/select_heldout.py <pilot raw dir> <out list>

Rule.  A pilot run "changes" against the LOW baseline of its scenario when it is not complete (PASS or P3_LATE), when its mission
lasts more than 1.2 times the baseline, or when a P3 part is not met.  Per factor of the channel: the first level that changes and
the level before it (the boundary); if no level changes, the most extreme level (to confirm the absence of an effect).  Always the
baseline and the blackout.  For each selected configuration: gate_single, plus every scenario that changed at that factor.  The
emergence pair (B2_LOW, B2NA_LOW) x (lost_drone_rejoin, em_delay_head).  Fresh seeds 6100-6104.
"""

import json
import sys
from collections import defaultdict
from pathlib import Path

LEVELS = {"per": [2, 4, 8], "lat": [0.5, 1, 2], "jit": [0.25, 0.5, 1], "loss": [0.3, 0.5, 0.7, 0.9], "burst": [10, 30, 60], "asym": [0.5, 0.9]}
SCEN = ("gate_single", "lost_drone_mutex", "formation_square")
SEEDS = range(6100, 6105)


def proto(f, v):
    return f"B2_SW-{f}{v:g}"


def main(raw: Path, out: Path) -> None:
    runs = {}
    for p in raw.rglob("run_metrics.json"):
        m = json.loads(p.read_text(encoding="utf-8"))
        q = m.get("p3") or {}
        bad_p3 = any(v.get("verdict") in ("NOT_MET", "LATE") for v in (q.get("P3a") or {}).values()) or \
            (q.get("P3b") or {}).get("verdict") in ("NOT_MET", "LATE") or (q.get("P3c") or {}).get("verdict") == "NOT_MET"
        runs[(m["protocol"], m["scenario"])] = (m["class"] in ("PASS", "P3_LATE"), (m.get("terminal") or {}).get("t_end"), bad_p3)
    base = {sc: runs.get(("B2_LOW", sc)) for sc in SCEN}

    def changed(pr, sc):
        r, b = runs.get((pr, sc)), base.get(sc)
        if r is None or b is None:
            return False
        return (not r[0]) or r[2] or (r[1] and b[1] and r[1] > 1.2 * b[1])

    sel = {"B2_LOW": set(SCEN), "B2_BLACKOUT": set(SCEN)}
    for f, lv in LEVELS.items():
        ch = [v for v in lv if any(changed(proto(f, v), sc) for sc in SCEN)]
        if ch:
            i = lv.index(ch[0])
            pick = lv[max(0, i - 1):i + 1]
        else:
            pick = [lv[-1]]
        for v in pick:
            sel[proto(f, v)] = {"gate_single"} | {sc for sc in SCEN if any(changed(proto(f, w), sc) for w in lv)}
    lines = [f"{p} LOW-ALL {sc} {s}" for p in sorted(sel) for sc in SCEN if sc in sel[p] for s in SEEDS]
    lines += [f"{p} LOW-ALL {sc} {s}" for p in ("B2_LOW", "B2NA_LOW") for sc in ("lost_drone_rejoin", "em_delay_head") for s in SEEDS]
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(len(lines), "runs;", {p: sorted(v) for p, v in sel.items()})


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
