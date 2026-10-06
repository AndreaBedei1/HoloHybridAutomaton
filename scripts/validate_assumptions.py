"""Empirically validate the assumptions of the formal models against HoloOcean runs (offline).

For every run directory given (default: all demonstrative runs in results/), measure:

  A_eps     per-component relative-position error of perceived neighbours vs truth   (<= eps_rel)
  A_cov     detection coverage of every drone within 8 m                              (~= 100 %)
  A_sym     symmetric detection: when i perceives j within d_warning, j perceives i   (~= 100 %)
  A_tau     staleness of the data used by the guards                                  (<= tau_max)
  A_hold    lateral / vertical hold error of yielding drones at their queue points    (<= hold_tol)
  A_mono    committed drone's along progress minus queued drones' progress in transit (>= -mono_tol)
  A_cmax    true closing speed of every pair                                          (<= c_max)
  A_track   residual velocity-tracking error in nominal modes (median / p95)          (informative)

Writes results/ASSUMPTIONS.json and results/ASSUMPTIONS.md.

    python scripts/validate_assumptions.py [run_dir ...]
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.analysis import load_run, mission_gates, perception_errors  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402


def _truth_at(rd, t):
    ts = rd.ts["t"]
    i = int(np.clip(np.searchsorted(ts, t + 1e-6), 0, len(ts) - 1))
    if i > 0 and abs(ts[i - 1] - t) < abs(ts[i] - t):
        i -= 1
    return i


def run_stats(run_dir: Path) -> dict:
    rd = load_run(run_dir)
    cfg = rd.config["fleet_config"]
    G = cfg["gate"]
    out = {"run": rd.name, "scenario": rd.config["spec_name"]}
    pe = perception_errors(rd)
    out["A_eps"] = {k: v for k, v in pe.items() if not k.startswith("_")}
    # symmetric detection within d_warning
    P = np.stack([np.stack([rd.ts[f"x_{k}"], rd.ts[f"y_{k}"], rd.ts[f"z_{k}"]], axis=1) for k in range(rd.n)])
    seen = [{} for _ in range(rd.n)]
    for k in range(rd.n):
        for o in rd.obs[k]:
            i = _truth_at(rd, o["t"])
            own = P[k, i]
            ids = set()
            for nb in o["local"]["neighbors"]:
                rel = np.asarray(nb["rel"])
                cands = [m for m in range(rd.n) if m != k]
                m_best = min(cands, key=lambda m: np.linalg.norm(P[m, i] - own - rel))
                if np.linalg.norm(P[m_best, i] - own - rel) < 1.0:
                    ids.add(m_best)
            seen[k][round(o["t"], 1)] = ids
    both = one = 0
    for t_key in seen[0]:
        i = _truth_at(rd, t_key)
        for a in range(rd.n):
            for b in range(a + 1, rd.n):
                if np.linalg.norm(P[a, i] - P[b, i]) < cfg["sep"]["d_warning"]:
                    sa = b in seen[a].get(t_key, set())
                    sb = a in seen[b].get(t_key, set())
                    if sa and sb:
                        both += 1
                    elif sa or sb:
                        one += 1
    out["A_sym"] = {"pair_samples_within_d_warning": both + one,
                    "fraction_symmetric": (both / (both + one)) if (both + one) else None}
    # staleness of the proximity sonar at decision time
    ages = [o["local"]["sensor_age"].get("ProxSonar", 0.0) for k in range(rd.n) for o in rd.obs[k]]
    out["A_tau"] = {"prox_sonar_age_p99": float(np.percentile(ages, 99)) if ages else None,
                    "prox_sonar_age_max_when_sense_ok": float(max([o["local"]["sensor_age"].get("ProxSonar", 0.0)
                                                                   for k in range(rd.n) for o in rd.obs[k]
                                                                   if o["abstract"]["sense_ok"]], default=0.0)),
                    "tau_max": cfg["env"]["tau_max"]}
    # hold tolerances at queue points (true positions, gate frame) while yielding
    _, mg = mission_gates(rd)
    gates = {g.gate_id: g for g in mg}
    lat_dev, z_dev, s_over = [], [], []
    for k in range(rd.n):
        slot_lat = rd.config["drones"][k]["slot_offset"]["lateral"]
        l_q = np.copysign(G["queue_lateral"], slot_lat) if abs(slot_lat) > 0.5 else 0.0
        for s, o in zip(rd.states[k], rd.obs[k]):
            if s["mode"] != "GATE_YIELD":
                continue
            gid = o["local"]["gate"]["gate_id"]
            if gid not in gates:
                continue
            i = _truth_at(rd, s["t"])
            sg, lg, zg = gates[gid].to_gate_frame(P[k, i])
            if o["local"]["gate"]["backoff"] == "none":
                lat_dev.append(abs(lg - l_q))
                z_dev.append(abs(zg))
            s_over.append(sg - G["s_queue"])
    out["A_hold"] = {"n_samples": len(lat_dev),
                     "lateral_dev_p99": float(np.percentile(lat_dev, 99)) if lat_dev else None,
                     "lateral_dev_max": float(max(lat_dev)) if lat_dev else None,
                     "vertical_dev_max": float(max(z_dev)) if z_dev else None,
                     "s_beyond_queue_line_max": float(max(s_over)) if s_over else None,
                     "hold_tol_lat": G["hold_tol_lat"], "hold_tol_z": G["hold_tol_z"], "hold_tol_s": G["hold_tol_s"]}
    # monotone progress during transit windows (commit -> visible in the occupied zone)
    commits = [e for e in rd.events if e.get("type") == "decision" and e.get("decision") in ("PASS", "RETRY_PASS")]
    worst = None
    for e in commits:
        k = int(e["drone"].split("_")[1])
        g = gates.get(e.get("gate"))
        if g is None:
            continue
        i0 = _truth_at(rd, e["t"])
        s0 = g.to_gate_frame(P[k, i0])[0]
        others0 = {m: g.to_gate_frame(P[m, i0])[0] for m in range(rd.n) if m != k}
        i = i0
        while i < len(rd.ts["t"]) - 1 and g.to_gate_frame(P[k, i])[0] < G["s_queue"] + G["occ_gamma"] + cfg["env"]["eps_rel"]:
            i += 1
            dk = g.to_gate_frame(P[k, i])[0] - s0
            for m, sm0 in others0.items():
                sm = g.to_gate_frame(P[m, i])[0]
                if G["s_queue"] - G["approach_len"] <= sm <= G["s_queue"] + G["hold_tol_s"]:
                    margin = dk - (sm - sm0)
                    worst = margin if worst is None else min(worst, margin)
    out["A_mono"] = {"n_commits": len(commits), "worst_margin": worst, "mono_tol": G["mono_tol"]}
    # closing speeds
    t = rd.ts["t"]
    cmax = 0.0
    for key in [k for k in rd.ts if k.startswith("d_")]:
        d = rd.ts[key]
        c = -np.diff(d) / np.maximum(np.diff(t), 1e-6)
        cmax = max(cmax, float(np.nanmax(c)))
    out["A_cmax"] = {"max_true_closing_speed": round(cmax, 3), "c_max": DEFAULT.env.c_max}
    # tracking residual in nominal authority
    errs = []
    for k in range(rd.n):
        acts = rd.run_dir / f"drone_{k}_actions.jsonl"
        if not acts.exists():
            continue
        st = {round(s["t"], 1): s for s in rd.states[k]}
        prev = None
        for line in open(acts):
            a = json.loads(line)
            if a["flow"]["authority"] != "nominal":
                continue
            s = st.get(round(a["t"], 1))
            if s is None:
                continue
            i = _truth_at(rd, a["t"])
            if prev is not None and i > prev[0]:
                v_true = (P[k, i, :2] - P[k, prev[0], :2]) / max(t[i] - t[prev[0]], 1e-6)
                errs.append(float(np.linalg.norm(np.asarray(prev[1][:2]) - v_true)))
            prev = (i, a["flow"]["v_d"])
    out["A_track"] = {"median": float(np.median(errs)) if errs else None,
                      "p95": float(np.percentile(errs, 95)) if errs else None}
    return out


def verdicts(s: dict) -> dict:
    env, G = DEFAULT.env, DEFAULT.gate
    v = {}
    e = s["A_eps"]
    # P1 needs the bound inside the warning band (hard: max); P2 inside the gate corridor (p99 reported)
    b1 = e.get("band_le_2.8m")
    v["A_eps"] = b1 is None or max(b1["max_abs_xyz"]) <= env.eps_rel
    cov = [x for k, x in e.items() if k.startswith("coverage")]
    v["A_cov"] = bool(cov and cov[0] is not None and cov[0] >= 0.99)
    v["A_sym"] = s["A_sym"]["fraction_symmetric"] is None or s["A_sym"]["fraction_symmetric"] >= 0.98
    v["A_tau"] = s["A_tau"]["prox_sonar_age_max_when_sense_ok"] <= env.tau_max + 1e-6
    h = s["A_hold"]
    v["A_hold"] = h["n_samples"] == 0 or (h["lateral_dev_max"] <= G.hold_tol_lat and h["vertical_dev_max"] <= G.hold_tol_z
                                          and h["s_beyond_queue_line_max"] <= G.hold_tol_s)
    m = s["A_mono"]
    v["A_mono"] = m["worst_margin"] is None or m["worst_margin"] >= -G.mono_tol
    v["A_cmax"] = s["A_cmax"]["max_true_closing_speed"] <= env.c_max
    return v


def main(argv=None) -> int:
    args = (argv if argv is not None else sys.argv[1:])
    runs = [Path(a) for a in args] or sorted(p for p in (ROOT / "results").glob("*")
                                              if p.is_dir() and (p / "referee_metrics.json").exists()
                                              and (p / "drone_0_observations.jsonl").exists())
    allres = []
    for r in runs:
        s = run_stats(r)
        s["verdicts"] = verdicts(s)
        allres.append(s)
        print(f"{s['run']}: " + ", ".join(f"{k}={'ok' if ok else 'VIOLATED'}" for k, ok in s["verdicts"].items()))
    (ROOT / "results" / "ASSUMPTIONS.json").write_text(json.dumps(allres, indent=2, default=float))
    lines = ["# Empirical validation of the formal assumptions", "",
             "Measured offline on the HoloOcean runs (ground truth vs onboard logs). 'VIOLATED' means the run left "
             "the assumption set, so the formal guarantee does not cover that run (its referee verdict is still empirical evidence).", "",
             "| run | A_eps max abs err xyz, d <= 2.8 m (P1) | p99 abs err xyz, 2.8-8 m (P2 corridor) | A_cov | A_sym | A_tau max (s) | A_hold (lat/z/s max, m) | A_mono worst (m) | A_cmax (m/s) |",
             "|---|---|---|---|---|---|---|---|---|"]
    for s in allres:
        e, h, vv = s["A_eps"], s["A_hold"], s["verdicts"]
        cov = [x for k, x in e.items() if k.startswith("coverage")][0]
        cov_s = f"{cov * 100:.1f}%" if cov is not None else "n/a"
        f = lambda ok: "" if ok else " **(!)**"  # noqa: E731
        b1 = e.get("band_le_2.8m", {}).get("max_abs_xyz", "-")
        b2 = e.get("band_2.8_to_8m", {}).get("p99_abs_xyz", "-")
        lines.append(
            f"| {s['run']} | {b1}{f(vv['A_eps'])} | {b2} | {cov_s}{f(vv['A_cov'])} | "
            f"{(s['A_sym']['fraction_symmetric'] or 1.0) * 100:.1f}%{f(vv['A_sym'])} | {s['A_tau']['prox_sonar_age_max_when_sense_ok']:.2f}{f(vv['A_tau'])} | "
            f"{h['lateral_dev_max'] if h['lateral_dev_max'] is not None else '-'} / {h['vertical_dev_max'] if h['vertical_dev_max'] is not None else '-'} / "
            f"{h['s_beyond_queue_line_max'] if h['s_beyond_queue_line_max'] is not None else '-'}{f(vv['A_hold'])} | "
            f"{s['A_mono']['worst_margin'] if s['A_mono']['worst_margin'] is not None else '-'}{f(vv['A_mono'])} | "
            f"{s['A_cmax']['max_true_closing_speed']}{f(vv['A_cmax'])} |")
    lines += ["", f"Formal parameters: eps_rel={DEFAULT.env.eps_rel} m, tau_max={DEFAULT.env.tau_max} s, "
                  f"hold_tol lat/z/s = {DEFAULT.gate.hold_tol_lat}/{DEFAULT.gate.hold_tol_z}/{DEFAULT.gate.hold_tol_s} m, "
                  f"mono_tol={DEFAULT.gate.mono_tol} m, c_max={DEFAULT.env.c_max} m/s."]
    (ROOT / "results" / "ASSUMPTIONS.md").write_text("\n".join(lines) + "\n")
    print("-> results/ASSUMPTIONS.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
