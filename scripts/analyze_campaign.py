"""Aggregate tables of the final B0 / B2 campaign (one raw root: results/robustness/raw_final).

    python scripts/analyze_campaign.py [results/robustness/raw_final] [--out results/robustness/final]

Per run (runs.csv): class, failed requirements, cause tags, P1 minimum true separation, time in SEPARATION_WARNING /
COLLISION_AVOIDANCE, P2 (holds, occupancy), gate completion, releases of the occupancy latch by kind (sonar witness,
acoustic evidence, t_occ_max fallback) and FALSE releases recomputed from the state logs against the truth (a latch
leaving BUSY while another drone is inside the critical region), P3 recovery, false missing / vacancy, provenance
(commit, engine binary, configuration hash).  INVALID runs are counted, never aggregated.

Aggregates (SUMMARY.md, cells.csv, paired.csv): class counts and PASS rate with a Wilson 95 % interval per cell
(protocol x sensor profile) and per scenario; cause tags of the non-PASS runs; paired B0 / B2 comparisons on the same
(scenario, sensor profile, seed) - discordant pairs, the input of an exact McNemar test; the acoustic blackout
against B0.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

CLASSES = ("PASS", "SAFE_DEGRADED", "LIVENESS_FAILURE", "SAFETY_FAILURE")
# B2 cells compared with B0 under the same sensor profile
PAIRS = [("B2_CLEAN", "CLEAN"), ("B2_MEDIUM", "CLEAN"), ("B2_BLACKOUT", "CLEAN"), ("B2_LOW", "LOW-ALL"),
         ("B2_MEDIUM", "MEDIUM-ONE"), ("B2_MEDIUM", "MEDIUM-SUBSET"), ("B2_MEDIUM", "MEDIUM-ALL"),
         ("B2_BLACKOUT", "MEDIUM-ALL")]


def _states(d: Path, k: int):
    f = d / f"drone_{k}_state.jsonl.gz"
    op = gzip.open if f.exists() else open
    f = f if f.exists() else d / f"drone_{k}_state.jsonl"
    with op(f, "rt", encoding="utf-8") as fh:
        return [json.loads(x) for x in fh]


def latch_releases(d: Path, scenario: str) -> dict:
    """Releases of the occupancy latch (BUSY -> EXITING / FREE) by kind, and how many happened while another drone was
    really inside the critical region of that gate."""
    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS

    sc = SCENARIOS[scenario](DEFAULT)
    if not sc.judged_gates:
        return {}
    g, G = sc.judged_gates[0], DEFAULT.gate
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    n = len(sc.sim.names)
    P = np.array([[[float(r[f"{a}{k}"]) for a in "xyz"] for k in range(n)] for r in rows])
    gf = np.einsum("mkj,ij->mki", P - g.center, np.array([g.axis, [-g.axis[1], g.axis[0], 0.0], [0, 0, 1.0]]))
    inside = ((np.abs(gf[..., 0]) <= G.cr_half_len) & (np.abs(gf[..., 1]) <= G.cr_half_width)
              & (np.abs(gf[..., 2]) <= G.cr_half_height))
    out = {"SONAR": [0, 0], "ACOUSTIC": [0, 0], "TIMEOUT": [0, 0]}
    for k in range(n):
        st = _states(d, k)
        prev = None
        for i in range(min(len(st), len(rows))):
            gs = st[i].get("gate") or {}
            occ, gid = gs.get("occ"), gs.get("id")
            if prev is not None and prev[0] == "BUSY" and prev[1] == gid and occ in ("EXITING", "FREE"):
                w = gs.get("exit_witness") or {}
                kind = "TIMEOUT" if occ == "FREE" else ("ACOUSTIC" if w.get("sector") == "ACOUSTIC" else "SONAR")
                out[kind][0] += 1
                out[kind][1] += int(any(inside[i, j] for j in range(n) if j != k))
            prev = (occ, gid)
    return out


def collect(raw: Path) -> list:
    runs = []
    for p in sorted(raw.glob("*/*/*/s*/run_metrics.json")):
        d = p.parent
        r = json.loads(p.read_text(encoding="utf-8"))
        row = {"protocol": r.get("protocol"), "profile": r.get("profile"), "scenario": r.get("scenario"),
               "seed": r.get("seed"), "class": r.get("class"), "failed": ";".join(r.get("failed_requirements") or []),
               "cause_tags": ";".join(r.get("cause_tags") or []), "wall_s": r.get("wall_s"),
               "code_sha": (r.get("code_version") or {}).get("sha"),
               "code_dirty": ";".join((r.get("code_version") or {}).get("dirty_files") or []),
               "engine_sha256": ((r.get("engine_build") or {}).get("binaries") or {}).get("Holodeck\\Binaries\\Win64\\Holodeck.exe"),
               "config_sha256": r.get("config_sha256")}
        if r.get("class") == "INVALID":
            row["invalid_reason"] = r.get("invalid_reason")
            runs.append(row)
            continue
        try:
            rm = json.loads((d / "referee_metrics.json").read_text(encoding="utf-8")).get("run", {})
        except (OSError, ValueError):
            rm = {}
        p1, p2, p3 = r.get("P1", {}), r.get("P2", {}), r.get("P3", {})
        fm = r.get("missing") or r.get("formation_missing") or {}
        term = r.get("terminal") or {}
        fr = latch_releases(d, row["scenario"])
        row.update({
            "outcome": term.get("outcome"), "t_end_s": term.get("t_end"), "deadline_s": term.get("deadline_s"),
            "collisions": p1.get("collisions"), "p1_min_sep_m": p1.get("min_true_separation_m"),
            "intruder_min_m": p1.get("intruder_min_m"), "warning_entries": p1.get("warning_entries"),
            "time_warning_s": p1.get("time_warning_s"), "ca_entries": p1.get("ca_entries"), "time_ca_s": p1.get("time_ca_s"),
            "p2_holds": p2.get("holds"), "max_occupancy": p2.get("max_occupancy"), "gate_completed": p2.get("completed"),
            "gate_completion_s": p2.get("completion_time_s"),
            "occ_timeouts": rm.get("occupancy_timeouts"),
            "releases_sonar": fr.get("SONAR", [None])[0], "releases_acoustic": fr.get("ACOUSTIC", [None])[0],
            "releases_timeout": fr.get("TIMEOUT", [None])[0],
            "false_releases": sum(v[1] for v in fr.values()) if fr else 0,
            "p3_episodes": p3.get("episodes"), "p3_all_recovered": p3.get("all_recovered"),
            "p3_recovery_max_s": p3.get("recovery_time_max_s"), "p3_recovery_mean_s": p3.get("recovery_time_mean_s"),
            "p3_max_err_m": p3.get("max_formation_error_m"), "p3_final_err_m": p3.get("final_formation_error_m"),
            "degraded_holds": p3.get("degraded_holds"),
            "false_vacancy": (r.get("conservative_events") or {}).get("false_vacancy"),
            "beacon_reception_rate": (rm.get("beacon") or {}).get("reception_rate")})
        runs.append(row)
    return runs


def wilson(k: int, n: int, z: float = 1.96):
    """Wilson score interval of a binomial proportion k / n."""
    if n == 0:
        return (None, None)
    p = k / n
    den = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(max(0.0, c - h), 3), round(min(1.0, c + h), 3))


def mcnemar_exact(b: int, c: int) -> float:
    """Two-sided exact McNemar p-value from the discordant pairs b, c."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return round(min(1.0, 2 * p), 4)


def _num(xs):
    return [float(x) for x in xs if x not in (None, "")]


def cells(runs) -> list:
    groups = defaultdict(list)
    for r in runs:
        groups[(r["protocol"], r["profile"])].append(r)
    out = []
    for (proto, prof), rs in sorted(groups.items()):
        ok = [r for r in rs if r["class"] != "INVALID"]
        cls = Counter(r["class"] for r in ok)
        lo, hi = wilson(cls["PASS"], len(ok))
        p1 = _num(r.get("p1_min_sep_m") for r in ok)
        out.append({"protocol": proto, "profile": prof, "runs": len(ok), "invalid": len(rs) - len(ok),
                    **{c: cls.get(c, 0) for c in CLASSES}, "pass_rate": round(cls["PASS"] / len(ok), 3) if ok else None,
                    "pass_ci95": f"{lo}-{hi}", "collisions": sum(int(r.get("collisions") or 0) for r in ok),
                    "p1_min_m": round(min(p1), 3) if p1 else None,
                    "false_releases": sum(int(r.get("false_releases") or 0) for r in ok),
                    "timeout_releases": sum(int(r.get("releases_timeout") or 0) for r in ok),
                    "sonar_releases": sum(int(r.get("releases_sonar") or 0) for r in ok),
                    "acoustic_releases": sum(int(r.get("releases_acoustic") or 0) for r in ok),
                    "recovery_mean_s": round(float(np.mean(_num(r.get("p3_recovery_mean_s") for r in ok))), 1)
                    if _num(r.get("p3_recovery_mean_s") for r in ok) else None})
    return out


def paired(runs) -> list:
    by = {(r["protocol"], r["profile"], r["scenario"], r["seed"]): r for r in runs if r["class"] != "INVALID"}
    out = []
    for proto, prof in PAIRS:
        both = [(by[("B0", prof, sc, s)], r) for (p, pf, sc, s), r in by.items()
                if p == proto and pf == prof and ("B0", prof, sc, s) in by]
        b = sum(1 for x, y in both if x["class"] == "PASS" and y["class"] != "PASS")      # B0 pass, B2 not
        c = sum(1 for x, y in both if x["class"] != "PASS" and y["class"] == "PASS")      # B2 pass, B0 not
        out.append({"b2": proto, "profile": prof, "pairs": len(both),
                    "both_pass": sum(1 for x, y in both if x["class"] == y["class"] == "PASS"),
                    "b0_only_pass": b, "b2_only_pass": c,
                    "neither": sum(1 for x, y in both if x["class"] != "PASS" and y["class"] != "PASS"),
                    "mcnemar_p": mcnemar_exact(b, c),
                    "b0_safety_failures": sum(1 for x, _ in both if x["class"] == "SAFETY_FAILURE"),
                    "b2_safety_failures": sum(1 for _, y in both if y["class"] == "SAFETY_FAILURE")})
    return out


def _table(rows, cols) -> str:
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        lines.append("| " + " | ".join("" if r.get(c) is None else str(r.get(c)) for c in cols) + " |")
    return "\n".join(lines)


def summarise(runs) -> str:
    ok = [r for r in runs if r["class"] != "INVALID"]
    shas = Counter(r.get("code_sha") for r in ok)
    dirty = sum(1 for r in ok if r.get("code_dirty"))
    eng = Counter(r.get("engine_sha256") for r in ok)
    parts = [f"# Final campaign summary\n\n{len(ok)} valid runs ({len(runs) - len(ok)} INVALID, excluded).  Code "
             f"commits: {dict(shas)}; runs from a modified tree: {dirty}; engine binaries: {dict(eng)}.\n"]
    cl = cells(runs)
    parts.append("## Classes per cell (PASS rate with Wilson 95 % interval)\n\n" + _table(
        cl, ["protocol", "profile", "runs", "invalid", *CLASSES, "pass_rate", "pass_ci95", "collisions", "p1_min_m",
             "false_releases", "timeout_releases", "sonar_releases", "acoustic_releases", "recovery_mean_s"]))
    sc = defaultdict(lambda: defaultdict(Counter))
    for r in ok:
        sc[r["scenario"]][(r["protocol"], r["profile"])][r["class"]] += 1
    keys = sorted({(r["protocol"], r["profile"]) for r in ok})
    lines = ["| scenario | " + " | ".join(f"{p} {q}" for p, q in keys) + " |", "|---|" + "---|" * len(keys)]
    for s in sorted(sc):
        lines.append(f"| {s} | " + " | ".join(
            (f"{sc[s][k]['PASS']}/{sum(sc[s][k].values())}" if sum(sc[s][k].values()) else "") for k in keys) + " |")
    parts.append("## PASS per scenario and cell\n\n" + "\n".join(lines))
    tags = Counter(t for r in ok if r["class"] != "PASS" for t in (r.get("cause_tags") or "").split(";") if t)
    parts.append("## Cause tags of the non-PASS runs\n\n" + _table(
        [{"tag": t, "runs": n} for t, n in tags.most_common()], ["tag", "runs"]))
    parts.append("## Paired B0 / B2 comparison (same scenario, sensor profile and seed)\n\n" + _table(
        paired(runs), ["b2", "profile", "pairs", "both_pass", "b0_only_pass", "b2_only_pass", "neither", "mcnemar_p",
                       "b0_safety_failures", "b2_safety_failures"]))
    unsafe = [r for r in ok if r["class"] == "SAFETY_FAILURE" or int(r.get("false_releases") or 0) > 0]
    parts.append("## Safety failures and false releases\n\n" + (_table(
        unsafe, ["protocol", "profile", "scenario", "seed", "class", "p1_min_sep_m", "collisions", "false_releases",
                 "cause_tags"]) if unsafe else "None."))
    return "\n\n".join(parts) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("raw", nargs="?", default=str(ROOT / "results" / "robustness" / "raw_final"))
    ap.add_argument("--out", default=str(ROOT / "results" / "robustness" / "final"))
    a = ap.parse_args()
    runs = collect(Path(a.raw))
    md = summarise(runs)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in runs for k in r})
    with open(out / "runs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(runs)
    for name, rows in (("cells.csv", cells(runs)), ("paired.csv", paired(runs))):
        if rows:
            with open(out / name, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
    (out / "SUMMARY.md").write_text(md, encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
