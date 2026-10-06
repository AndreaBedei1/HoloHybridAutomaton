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
    # A_hold exactly as used by lemma M3: during each transit window (a drone commits -> it becomes
    # visible in the occupied zone), every OTHER drone that is queued (s <= s_queue + hold_tol_s region,
    # inside the approach zone) keeps its lateral / vertical position within 2*hold_tol of its value at
    # the commit time, and does not pass s_queue + hold_tol_s.
    _, mg = mission_gates(rd)
    gates = {g.gate_id: g for g in mg}
    commits = [e for e in rd.events if e.get("type") == "decision" and e.get("decision") in ("PASS", "RETRY_PASS")]
    lat_dev, z_dev, s_over = [], [], []
    for e in commits:
        k = int(e["drone"].split("_")[1])
        g = gates.get(e.get("gate"))
        if g is None:
            continue
        i0 = _truth_at(rd, e["t"])
        ref = {m: g.to_gate_frame(P[m, i0]) for m in range(rd.n) if m != k}
        sk0 = g.to_gate_frame(P[k, i0])[0]
        # lemma M3 needs A_hold only for queued drones ALONG-TIED with the committing drone at t0
        queued = [m for m, (s0, l0, z0) in ref.items()
                  if G["s_queue"] - G["approach_len"] - 1.0 <= s0 <= G["s_queue"] + G["hold_tol_s"]
                  and abs(s0 - sk0) <= G["mu_s_lo"] + cfg["env"]["eps_rel"]]
        i = i0
        while i < len(rd.ts["t"]) - 1 and g.to_gate_frame(P[k, i])[0] < G["s_queue"] + G["occ_gamma"] + cfg["env"]["eps_rel"]:
            i += 1
            for m in queued:
                s1, l1, z1 = g.to_gate_frame(P[m, i])
                lat_dev.append(abs(l1 - ref[m][1]))
                z_dev.append(abs(z1 - ref[m][2]))
                s_over.append(s1 - G["s_queue"])
    out["A_hold"] = {"n_samples": len(lat_dev),
                     "lateral_change_max": float(max(lat_dev)) if lat_dev else None,
                     "vertical_change_max": float(max(z_dev)) if z_dev else None,
                     "s_beyond_queue_line_max": float(max(s_over)) if s_over else None,
                     "bound_lat": 2 * G["hold_tol_lat"], "bound_z": 2 * G["hold_tol_z"], "hold_tol_s": G["hold_tol_s"]}
    # monotone progress during transit windows (commit -> visible in the occupied zone)
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
    # detection is required by P1 inside the warning band (hard) and by P2 in the gate corridor
    c1 = e.get("coverage_le_2.8m")
    v["A_cov"] = c1 is None or c1 >= 0.999
    v["A_sym"] = s["A_sym"]["fraction_symmetric"] is None or s["A_sym"]["fraction_symmetric"] >= 0.98
    v["A_tau"] = s["A_tau"]["prox_sonar_age_max_when_sense_ok"] <= env.tau_max + 1e-6
    h = s["A_hold"]
    v["A_hold"] = h["n_samples"] == 0 or (h["lateral_change_max"] <= 2 * G.hold_tol_lat
                                          and h["vertical_change_max"] <= 2 * G.hold_tol_z
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
             "| run | A_eps: max abs err xyz, d <= 2.8 m (P1) | p99 abs err xyz, 2.8-8 m | A_cov: detection d <= 2.8 m / 2.8-8 m | A_sym | A_tau max [s] | A_hold: max lateral / vertical change of along-tied queued drones in transit windows, max s past queue line [m] | A_mono worst margin [m] | A_cmax [m/s] |",
             "|---|---|---|---|---|---|---|---|---|"]
    r3 = lambda x: "-" if x is None else (f"{x:.2f}" if isinstance(x, float) else str(x))  # noqa: E731
    for s in allres:
        e, h, vv = s["A_eps"], s["A_hold"], s["verdicts"]
        c1, c2 = e.get("coverage_le_2.8m"), e.get("coverage_2.8_to_8m")
        cov_s = f"{'-' if c1 is None else f'{c1 * 100:.1f}%'} / {'-' if c2 is None else f'{c2 * 100:.1f}%'}"
        f = lambda ok: "" if ok else " **(!)**"  # noqa: E731
        b1 = e.get("band_le_2.8m", {}).get("max_abs_xyz", "-")
        b2 = e.get("band_2.8_to_8m", {}).get("p99_abs_xyz", "-")
        lines.append(
            f"| {s['run']} | {b1}{f(vv['A_eps'])} | {b2} | {cov_s}{f(vv['A_cov'])} | "
            f"{(s['A_sym']['fraction_symmetric'] or 1.0) * 100:.1f}%{f(vv['A_sym'])} | {s['A_tau']['prox_sonar_age_max_when_sense_ok']:.2f}{f(vv['A_tau'])} | "
            + ("not invoked (no along-tied queued drone at any commit)" if h["n_samples"] == 0 and s["A_mono"]["n_commits"]
               else "-" if h["n_samples"] == 0 else
               f"{r3(h['lateral_change_max'])} / {r3(h['vertical_change_max'])} / {r3(h['s_beyond_queue_line_max'])}{f(vv['A_hold'])}")
            + " | "
            f"{r3(s['A_mono']['worst_margin'])}{f(vv['A_mono'])} | "
            f"{s['A_cmax']['max_true_closing_speed']}{f(vv['A_cmax'])} |")
    lines += ["", f"Formal parameters: eps_rel={DEFAULT.env.eps_rel} m, tau_max={DEFAULT.env.tau_max} s, "
                  f"hold bounds lat/z = {2 * DEFAULT.gate.hold_tol_lat:.2f}/{2 * DEFAULT.gate.hold_tol_z:.2f} m (2 x hold_tol), "
                  f"hold_tol_s = {DEFAULT.gate.hold_tol_s} m, mono_tol={DEFAULT.gate.mono_tol} m, c_max={DEFAULT.env.c_max} m/s.",
              "Errors are measured against the truth at the acquisition time of the sonar data (data age is checked "
              "separately as A_tau, as in the P1 model).  Detection coverage is a hard requirement inside the warning "
              "band; at 2.8-8 m it is reported for the gate corridor."]
    (ROOT / "results" / "ASSUMPTIONS.md").write_text("\n".join(lines) + "\n")
    print("-> results/ASSUMPTIONS.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
