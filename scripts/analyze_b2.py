"""B0 vs B2 comparison tables from a raw campaign root (run_metrics.json + referee_metrics.json + state logs).

    python scripts/analyze_b2.py results/robustness/raw_b2_gate [--out results/b2/gate_clean]

Per run: class, completion, contacts, P1 minimum true separation, P2 (holds, max occupancy), gate completion time,
mean queue wait, occupancy-latch timeouts (onboard counter), exit witnesses by kind, FALSE RELEASES recomputed from
the state logs against the truth (a latch moving BUSY -> EXITING on a witness, or BUSY -> FREE on the t_occ_max
fallback at the same gate, while another drone is in the CR), P3 outcome, missing/vacancy figures.  Writes a CSV
per run and a Markdown summary per (protocol, profile, scenario).
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _states(d: Path, k: int):
    f = d / f"drone_{k}_state.jsonl.gz"
    op = gzip.open if f.exists() else open
    f = f if f.exists() else d / f"drone_{k}_state.jsonl"
    with op(f, "rt", encoding="utf-8") as fh:
        return [json.loads(x) for x in fh]


def false_releases(d: Path) -> dict:
    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS

    rc = json.loads((d / "run_config.json").read_text(encoding="utf-8"))
    name = rc.get("scenario") or rc.get("config", {}).get("scenario")
    sc = SCENARIOS[name](DEFAULT)
    if not sc.judged_gates:
        return {}
    g, G = sc.judged_gates[0], DEFAULT.gate
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    n = len(sc.sim.names)
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in rows])
    gf = np.einsum("mkj,ij->mki", P - g.center, np.array([g.axis, [-g.axis[1], g.axis[0], 0.0], [0, 0, 1.0]]))
    inside = (np.abs(gf[..., 0]) <= G.cr_half_len) & (np.abs(gf[..., 1]) <= G.cr_half_width) & (np.abs(gf[..., 2]) <= G.cr_half_height)
    out = {"SONAR": [0, 0], "ACOUSTIC": [0, 0], "TIMEOUT": [0, 0]}
    for k in range(n):
        st = _states(d, k)
        m = min(len(st), len(rows))
        prev = None
        for i in range(m):
            gs = st[i].get("gate") or {}
            occ, gid = gs.get("occ"), gs.get("id")
            if prev is not None and prev[0] == "BUSY" and prev[1] == gid and occ in ("EXITING", "FREE"):
                w = gs.get("exit_witness") or {}
                kind = "TIMEOUT" if occ == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")
                bad = any(inside[i, j] for j in range(n) if j != k)
                out[kind][0] += 1
                out[kind][1] += int(bad)
            prev = (occ, gid)
    return out


def collect(raw: Path):
    runs = []
    for p in sorted(raw.glob("*/*/*/s*/run_metrics.json")):
        d = p.parent
        r = json.loads(p.read_text(encoding="utf-8"))
        row = {"protocol": r.get("protocol"), "profile": r.get("profile"), "scenario": r.get("scenario"),
               "seed": r.get("seed"), "class": r.get("class"), "failed": ";".join(r.get("failed_requirements") or [])}
        if r.get("class") == "INVALID":
            runs.append(row)
            continue
        rm = json.loads((d / "referee_metrics.json").read_text(encoding="utf-8")).get("run", {})
        p1, p2, p3 = r.get("P1", {}), r.get("P2", {}), r.get("P3", {})
        term = r.get("terminal") or {}
        fr = false_releases(d)
        row.update({"outcome": term.get("outcome"), "t_end_s": term.get("t_end"),
                    "collisions": p1.get("collisions"), "p1_min_sep_m": p1.get("min_true_separation_m"),
                    "p2_holds": p2.get("holds"), "max_occupancy": p2.get("max_occupancy"),
                    "gate_completion_s": p2.get("completion_time_s"), "queue_wait_mean_s": p2.get("queue_wait_mean_s"),
                    "queue_wait_max_s": p2.get("queue_wait_max_s"),
                    "occ_timeouts": rm.get("occupancy_timeouts"),
                    "acoustic_witnesses": rm.get("acoustic_exit_witnesses"), "sonar_witnesses": rm.get("sonar_exit_witnesses"),
                    "releases_sonar": fr.get("SONAR", [None])[0], "releases_acoustic": fr.get("ACOUSTIC", [None])[0],
                    "releases_timeout": fr.get("TIMEOUT", [None])[0],
                    "false_releases": sum(v[1] for v in fr.values()) if fr else 0,
                    "p3_all_recovered": p3.get("all_recovered"), "p3_max_err_m": p3.get("max_formation_error_m"),
                    "false_missing_episodes": p3.get("false_missing_episodes"), "missing_episodes": p3.get("missing_episodes"),
                    "false_vacancy": p3.get("false_vacancy"),
                    "true_missing_delay_s": ";".join(str(x) for x in (p3.get("true_missing_delay_s") or [])),
                    "wait_rejoin_s_mean": p3.get("wait_rejoin_s_mean"),
                    "beacon_reception_rate": (rm.get("beacon") or {}).get("reception_rate")})
        runs.append(row)
    return runs


def _m(xs):
    xs = [float(x) for x in xs if x not in (None, "")]
    return round(float(np.mean(xs)), 1) if xs else None


def summarise(runs) -> str:
    groups = defaultdict(list)
    for r in runs:
        groups[(r["scenario"], r["profile"], r["protocol"])].append(r)
    lines = ["| scenario | sensors | protocol | runs | classes | contacts | P1 min [m] | P2 holds | max occ | false rel. | "
             "latch timeouts | exits sonar / acoustic | gate completion [s] | queue wait [s] | P3 recovered | false missing |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for (sc, prof, proto), rs in sorted(groups.items()):
        ok = [r for r in rs if r["class"] != "INVALID"]
        cls = defaultdict(int)
        for r in rs:
            cls[r["class"]] += 1
        lines.append(
            f"| {sc} | {prof} | {proto} | {len(rs)} | {', '.join(f'{k} {v}' for k, v in sorted(cls.items()))} | "
            f"{sum(int(r.get('collisions') or 0) for r in ok)} | {min((r['p1_min_sep_m'] for r in ok if r.get('p1_min_sep_m') is not None), default=None)} | "
            f"{sum(bool(r.get('p2_holds')) for r in ok)}/{len(ok)} | {max((r.get('max_occupancy') or 0 for r in ok), default=None)} | "
            f"{sum(int(r.get('false_releases') or 0) for r in ok)} | {sum(int(r.get('occ_timeouts') or 0) for r in ok)} | "
            f"{sum(int(r.get('releases_sonar') or 0) for r in ok)} / {sum(int(r.get('releases_acoustic') or 0) for r in ok)} | "
            f"{_m(r.get('gate_completion_s') for r in ok)} | {_m(r.get('queue_wait_mean_s') for r in ok)} | "
            f"{sum(bool(r.get('p3_all_recovered')) for r in ok if r.get('p3_all_recovered') is not None)} | "
            f"{sum(int(r.get('false_missing_episodes') or 0) for r in ok)} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("raw")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    runs = collect(Path(a.raw))
    md = summarise(runs)
    print(md)
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        keys = sorted({k for r in runs for k in r})
        with open(out / "runs.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(runs)
        (out / "SUMMARY.md").write_text(md, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
