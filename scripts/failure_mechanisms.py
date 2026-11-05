"""Mechanism of every non-PASS run of a campaign (diagnosis, not classification).

    python scripts/failure_mechanisms.py <raw root> [out.md]

For each run whose class is not PASS: the missing requirements at the terminal instant, the gate-liveness events
(idle and committed stalls), give-ups, and per healthy drone at the end: mode, latch state, the acoustic exit
diagnostics, gate localization, whether it passed.  Mechanisms are labelled by explicit rules on these fields.
"""

from __future__ import annotations

import gzip
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path


def states_tail(d: Path, k: int, n: int = 1):
    p = d / f"drone_{k}_state.jsonl.gz"
    if not p.exists():
        return []
    out = []
    with gzip.open(p, "rt") as fh:
        for line in fh:
            out.append(line)
            if len(out) > n:
                out.pop(0)
    return [json.loads(x) for x in out]


def mechanism(m, d: Path):
    term = m.get("terminal") or {}
    miss = list(term.get("missing") or [])
    ev = (m.get("P2") or {}).get("events") or {}
    E8 = ev.get("E8") or {}
    tags = m.get("cause_tags") or []
    labels = []
    if term.get("outcome") == "SUCCESS":
        labels.append("LATE_ONLY (mission complete; a gate bounded-response check exceeded)")
        return labels, miss, {}
    events = [json.loads(x) for x in open(d / "events.jsonl", encoding="utf-8")]
    aborts = sum(1 for e in events if e.get("decision") == "ABORT_PASS")
    faults = [e for e in events if e.get("type") == "FAULT_INJECTED"]
    end = {}
    n = (m.get("run") or {}).get("n_drones") or 0
    k = 0
    while (d / f"drone_{k}_state.jsonl.gz").exists():
        st = states_tail(d, k)
        if st:
            s = st[-1]
            g = s.get("gate") or {}
            end[k] = {"mode": s.get("mode"), "committed": s.get("committed"), "occ": g.get("occ"),
                      "loc_ok": g.get("loc_ok"), "entry": g.get("entry"), "acoustic": g.get("acoustic"),
                      "s": g.get("s"), "missing": (s.get("form") or {}).get("missing"),
                      "vacant": (s.get("form") or {}).get("vacant"), "nav_bound": s.get("nav_bound")}
        k += 1
    if "gate" in miss or "progress" in miss:
        busy = [k for k, e in end.items() if e["occ"] == "BUSY" and e["s"] is not None and e["s"] > -1e8]
        held = [k for k, e in end.items() if e["entry"] == "HOLD"]
        if held and any(not end[k]["loc_ok"] for k in held):
            labels.append("GATE_RELOCALIZATION_HOLD (committed, no gate localization)")
        if busy:
            acc = [end[k]["acoustic"] for k in busy if end[k]["acoustic"]]
            if any(a.get("not_outside") for a in acc):
                labels.append("GATE_LATCH_BUSY_POSSIBLE_OCCUPANT (a drone not confirmed outside)")
            else:
                labels.append("GATE_LATCH_BUSY_OTHER (corridor echo / no acoustic diagnostics)")
        if aborts:
            labels.append(f"GIVE_UP_CYCLES ({aborts})")
        if not labels:
            labels.append("GATE_OR_PROGRESS_OTHER")
    if "P3 recovered" in miss or "P3-deg re-formed" in miss:
        labels.append("FORMATION_NOT_RECOVERED")
    if "no false vacancy" in miss:
        labels.append("FALSE_VACANCY")
    if "P3-deg vacancy" in miss:
        labels.append("VACANCY_NOT_DECLARED")
    if "not waiting" in miss:
        labels.append("STILL_WAITING_FOR_NEIGHBOUR")
    if faults:
        labels.append("scripted_fault")
    return labels, miss, {"aborts": aborts, "idle": E8.get("idle_stalls"), "committed": E8.get("committed_stalls"),
                          "tags": tags, "end": end}


def main(argv) -> int:
    raw = Path(argv[1])
    out = Path(argv[2]) if len(argv) > 2 else None
    rows, by = [], defaultdict(Counter)
    for p in sorted(raw.rglob("run_metrics.json")):
        if "_attempts" in p.parts:
            continue
        m = json.loads(p.read_text(encoding="utf-8"))
        if m.get("class") == "PASS":
            continue
        d = p.parent
        rel = d.relative_to(raw).as_posix()
        labels, miss, info = mechanism(m, d)
        cell = "/".join(rel.split("/")[:2])
        sc = rel.split("/")[2]
        for lb in labels:
            by[lb.split(" ")[0]][f"{cell} {sc}"] += 1
        rows.append((rel, m.get("class"), labels, miss, info))
    L = ["# Mechanisms of the non-PASS runs", "", f"raw root: {raw}", "", "## Counts by mechanism", ""]
    for lb, c in sorted(by.items(), key=lambda x: -sum(x[1].values())):
        L.append(f"* **{lb}**: {sum(c.values())} runs; " + ", ".join(f"{k} x{v}" for k, v in c.most_common()))
    L += ["", "## Runs", ""]
    for rel, cls, labels, miss, info in rows:
        L.append(f"* `{rel}` {cls}: {'; '.join(labels)} | missing {miss}")
        if info.get("end"):
            for k, e in info["end"].items():
                L.append(f"    - drone {k}: {e}")
    text = "\n".join(L) + "\n"
    if out:
        out.write_text(text, encoding="utf-8")
    print("\n".join(L[:30]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
