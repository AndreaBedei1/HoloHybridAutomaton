"""Per-run audit of a paired campaign (config/publication_validation.json): mission completion, gate crossings, formation
recovery, P1 / P2 safety and every premise the referee monitors, re-derived from the raw logs with the current referee.

    python scripts/audit_final_campaign.py --raw <campaign>/raw --out <campaign>/final_run_audit.json --expect 54 [--protocol B2]

Each run directory (``run_metrics.json`` written by the campaign) is analysed again with
``holo_fleet.referee.robustness_metrics.analyze_run``; the recorded class must equal the re-derived one (the referee may only
have gained monitors since the campaign, e.g. premise Q, ``queue_cls``).  A run is ACCEPTED when its class is PASS, every drone
crossed every judged gate, the formation recovered, P1 and P2 hold with no contact, no unsafe admission (E4) and no false latch
release, and no premise monitor reports a violation.  It fails closed: a monitor missing from the metrics, a run the referee
cannot analyse (INVALID, SUT_ERROR), a gate scenario without gate metrics or without any judged queued sample, a scenario with
more than one judged gate, queue plans that the scenario code no longer reproduces (premise Q2 is judged against them), an
empty campaign or a run count different from ``--expect`` all reject.  Diagnostics that are not
declared premises (wrong validated map fits, the classification sources of pre-commit steps outside the queue) are reported,
not judged.  The premise monitors are cause tags of the referee, not classes: this audit is where they are judged.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PREMISES = ("zone_nav", "beacon_fix", "queue_cls")
MONITORS = ("nav_bound", "gate_nav", "premise_v", "premise_c", "scope") + PREMISES


def _judged_gates(scenario) -> int:
    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS

    return len(SCENARIOS[scenario](DEFAULT).judged_gates)


def plans_reproduced(rc: dict) -> bool:
    """The queue plans the referee rebuilds from the scenario code equal the ones the run recorded (``run_config.json``).

    The referee judges premise Q2 (``queue_cls``) against the rebuilt queue points; a queue layout changed since the
    campaign would shift that reference silently, so a mismatch rejects the run."""
    from holo_fleet.config import COMMUNICATION_DISABLED, DEFAULT, communication_disabled
    from holo_fleet.sim.scenarios import SCENARIOS

    cfg = communication_disabled(DEFAULT) if rc.get("communication") == COMMUNICATION_DISABLED else DEFAULT
    built = {p.drone_id: p for p in (getattr(SCENARIOS[rc["scenario"]](cfg), "plans", None) or [])}
    recorded = rc.get("plans") or []
    return len(recorded) == len(built) and all(
        pl.get("drone") in built and pl.get("slot") == built[pl["drone"]].slot_index
        and all(abs(float(pl[k]) - float(getattr(built[pl["drone"]], k))) <= 1e-3
                for k in ("queue_lateral", "queue_dz", "queue_s"))
        for pl in recorded)


def _rejected(run: str, rec: dict, m: dict) -> dict:
    """A run the referee cannot analyse (INVALID, SUT_ERROR): nothing to judge, rejected."""
    return {"run": run, "protocol": rec.get("protocol"), "profile": rec.get("profile"), "scenario": rec.get("scenario"),
            "seed": rec.get("seed"), "class": m.get("class"), "recorded_class": rec.get("class"), "t_end_s": None,
            "min_true_separation_m": None, "max_occupancy": None, "crossed": [], "unsafe_admissions": None,
            "p3_recovery_max_s": None, "premises": {}, "diagnostics": {}, "cause_tags": m.get("cause_tags"),
            "checks": {"analysed": False}, "accepted": False}


def audit_run(d: Path) -> dict:
    from holo_fleet.referee.robustness_metrics import analyze_run

    rec = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    m = analyze_run(d, profile=rec.get("profile"))
    run = str(d.relative_to(d.parents[4]))
    if "P1" not in m or "P2" not in m:
        return _rejected(run, rec, m)
    p1, p2, p3, term = m["P1"], m["P2"], m.get("P3") or {}, m.get("terminal") or {}
    ev = p2.get("events") or {}
    passed = set(p2.get("passed") or [])
    n = len((m.get("automaton") or {}).get("end_modes") or [])  # 0 (unknown) fails the crossing check
    gates = _judged_gates(rec.get("scenario"))
    present = {k: isinstance(m.get(k), dict) for k in MONITORS}
    premises = {k: (m.get(k) or {}).get("holds") for k in PREMISES}
    checks = {
        "analysed": True,
        "monitors_present": all(present.values()),
        "class_pass": m["class"] == "PASS",
        "class_reproduced": m["class"] == rec.get("class"),
        "terminal_success": term.get("outcome") == "SUCCESS",
        "one_judged_gate_at_most": gates <= 1,  # the crossing check below covers one gate
        "gate_metrics": bool(p2.get("gates")) == (gates > 0),
        "all_crossed": (bool(p2.get("completed")) and n > 0 and len(passed) == n) if gates else True,
        "formation_recovered": bool(p3.get("all_recovered")) if p3.get("enabled") else True,
        "p1": bool(p1["holds"]) and not p1.get("collisions"),
        "p2": (bool(p2["holds"]) and (p2.get("max_occupancy") or 0) <= 1) if gates else True,
        "no_unsafe_admission": not ev.get("E4"),
        "no_false_release": not p2.get("false_release_total"),
        "premise_nav_bound": present["nav_bound"] and not m["nav_bound"].get("exceeded"),
        "premise_gate_nav": present["gate_nav"] and not m["gate_nav"].get("committed_beyond"),
        "premise_v": present["premise_v"] and not m["premise_v"].get("live_declared_dead"),
        "premise_c": present["premise_c"] and not (m["premise_c"].get("counts") or {}).get("VIOLATED"),
        "scope": present["scope"] and not m["scope"].get("failures_inside_zone"),
        **{f"premise_{k}": v is True for k, v in premises.items()},  # a missing monitor fails
        "queue_cls_observed": ((m.get("queue_cls") or {}).get("samples") or 0) > 0 if gates else True,
        "plans_reproduced": plans_reproduced(json.loads((d / "run_config.json").read_text(encoding="utf-8"))),
    }
    return {"run": run, "protocol": rec.get("protocol"), "profile": rec.get("profile"),
            "scenario": rec.get("scenario"), "seed": rec.get("seed"), "class": m["class"], "recorded_class": rec.get("class"),
            "t_end_s": term.get("t_end"), "min_true_separation_m": p1.get("min_true_separation_m"),
            "max_occupancy": p2.get("max_occupancy"), "crossed": sorted(passed), "unsafe_admissions": ev.get("E4"),
            "p3_recovery_max_s": p3.get("recovery_time_max_s"),
            "premises": {k: m.get(k) for k in ("zone_nav", "beacon_fix", "queue_cls", "gate_nav", "nav_bound")},
            "diagnostics": {"wrong_validated_fits": m.get("wrong_validated_fits"),
                            "queue_cls_outside_queue_sources": (m.get("queue_cls") or {}).get("outside_queue_sources")},
            "cause_tags": m.get("cause_tags"), "checks": checks, "accepted": all(checks.values())}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--raw", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--protocol", default="", help="audit only protocols starting with this prefix (e.g. B2)")
    ap.add_argument("--expect", type=int, required=True, help="number of runs the campaign must contain (fails otherwise)")
    args = ap.parse_args(argv)
    if not args.raw.is_dir():
        raise SystemExit(f"no campaign raw directory at {args.raw}")
    rows = []
    for f in sorted(args.raw.glob("*/*/*/*/s*/run_metrics.json")):
        proto = f.parents[3].name
        if args.protocol and not proto.startswith(args.protocol):
            continue
        rows.append({"case": f.parents[4].name, **audit_run(f.parent)})
    failed = [r for r in rows if not r["accepted"]]
    count_ok = bool(rows) and (args.expect is None or len(rows) == args.expect)
    out = {"raw": args.raw.name, "runs": len(rows), "expected_runs": args.expect, "count_ok": count_ok,
           "accepted": len(rows) - len(failed),
           "failed": [{"case": r["case"], "protocol": r["protocol"], "seed": r["seed"], "class": r["class"],
                       "failed_checks": [k for k, v in r["checks"].items() if not v]} for r in failed],
           "rows": rows}
    args.out.write_text(json.dumps(out, indent=1, default=str), encoding="utf-8")
    for r in rows:
        bad = [k for k, v in r["checks"].items() if not v]
        print(f"{r['case']:22s} {r['protocol']:10s} s{r['seed']} {str(r['class']):14s} t_end {r['t_end_s']} "
              f"sep {r['min_true_separation_m']} occ {r['max_occupancy']} {'OK' if not bad else 'FAILED ' + ','.join(bad)}")
    print(f"{out['accepted']}/{out['runs']} runs accepted" + ("" if count_ok else
          f"; FAILED: {len(rows)} runs found, {args.expect if args.expect is not None else 'more than 0'} expected"))
    return 0 if not failed and count_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
