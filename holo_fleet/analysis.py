"""Offline analysis of a run directory: loaders, perception-vs-truth statistics, figures.

Offline only: this module reads the referee's ground-truth time series and the
controllers' logs after the run; nothing here feeds back into a controller.
Colours follow the dataviz reference palette (validated, see REPORT.md):
drones use categorical slots 1-3, pairs a separate validated triple.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

DRONE_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]          # slots 1-3 (blue, orange, aqua)
PAIR_COLORS = {"01": "#4a3aa7", "02": "#e87ba4", "12": "#008300"}   # violet, magenta, green
INK = {"primary": "#0b0b0b", "secondary": "#52514e", "muted": "#898781", "grid": "#e1e0d9",
       "axis": "#c3c2b7", "surface": "#fcfcfb"}
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
MODE_ORDER = ["FORMATION_FOLLOW", "FORMATION_RECOVERY", "GATE_APPROACH", "GATE_YIELD", "GATE_PASS",
              "SEPARATION_WARNING", "COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT"]
MODE_SHORT = {"FORMATION_FOLLOW": "FOLLOW", "FORMATION_RECOVERY": "RECOVERY", "GATE_APPROACH": "GATE APPROACH",
              "GATE_YIELD": "GATE YIELD", "GATE_PASS": "GATE PASS", "SEPARATION_WARNING": "SEP. WARNING",
              "COLLISION_AVOIDANCE": "COLL. AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT": "FAILSAFE"}


def _style():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "figure.facecolor": INK["surface"], "axes.facecolor": INK["surface"], "savefig.facecolor": INK["surface"],
        "axes.edgecolor": INK["axis"], "axes.labelcolor": INK["secondary"], "xtick.color": INK["muted"],
        "ytick.color": INK["muted"], "text.color": INK["primary"], "axes.grid": True, "grid.color": INK["grid"],
        "grid.linewidth": 0.6, "grid.linestyle": "-", "axes.spines.top": False, "axes.spines.right": False,
        "font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"], "font.size": 9, "axes.titlesize": 10,
        "axes.titleweight": "bold", "legend.frameon": False, "lines.linewidth": 1.6,
    })
    return plt


@dataclass
class RunData:
    run_dir: Path
    config: Dict
    metrics: Dict
    ts: Dict[str, np.ndarray]
    states: List[List[Dict]]
    obs: List[List[Dict]]
    events: List[Dict]
    n: int

    @property
    def name(self) -> str:
        return self.run_dir.name


def load_run(run_dir) -> RunData:
    run_dir = Path(run_dir)
    config = json.loads((run_dir / "run_config.json").read_text())
    metrics = json.loads((run_dir / "referee_metrics.json").read_text())
    rows = list(csv.DictReader(open(run_dir / "referee_timeseries.csv")))
    keys = rows[0].keys() if rows else []
    ts = {k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows]) for k in keys}
    n = config["n_drones"]
    states, obs = [], []
    for k in range(n):
        p = run_dir / f"drone_{k}_state.jsonl"
        states.append([json.loads(line) for line in open(p)] if p.exists() else [])
        p = run_dir / f"drone_{k}_observations.jsonl"
        obs.append([json.loads(line) for line in open(p)] if p.exists() else [])
    events = [json.loads(line) for line in open(run_dir / "events.jsonl")] if (run_dir / "events.jsonl").exists() else []
    return RunData(run_dir, config, metrics, ts, states, obs, events, n)


def mission_gates(rd: RunData):
    from holo_fleet.arena_bridge import load_arena_gates

    all_g = load_arena_gates()
    return all_g, [g for g in all_g if g.gate_id in rd.config.get("mission_gates", [])]


# --------------------------------------------------------------------------- perception statistics
def _truth_interp(t_ts: np.ndarray, P: np.ndarray, t: float) -> np.ndarray:
    """Ground-truth positions of all drones at time t (linear interpolation between referee rows)."""
    j = int(np.clip(np.searchsorted(t_ts, t), 1, len(t_ts) - 1))
    t0, t1 = t_ts[j - 1], t_ts[j]
    w = float(np.clip((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0))
    return (1 - w) * P[:, j - 1] + w * P[:, j]


def perception_errors(rd: RunData, max_range: float = 8.0) -> Dict:
    """Compare every perceived neighbour (relative position) with the true relative position AT THE TIME
    THE SONAR DATA WAS ACQUIRED (decision time minus the logged data age).  This isolates the static
    error bound eps_rel of the formal model; the effect of the data age is covered separately by
    tau_max (checked as A_tau) through the term c_max * tau in the P1 model."""
    t_ts = rd.ts["t"]
    P = np.stack([np.stack([rd.ts[f"x_{k}"], rd.ts[f"y_{k}"], rd.ts[f"z_{k}"]], axis=1) for k in range(rd.n)])
    comp_err, dist_err, stale, true_rng = [], [], [], []
    detected, eligible = 0, 0
    band_cnt = {"le_2.8m": [0, 0], "2.8_to_8m": [0, 0]}     # [detected, eligible]
    for k in range(rd.n):
        for o in rd.obs[k]:
            t = o["t"]
            age = float(o["local"]["sensor_age"].get("ProxSonar", 0.0) or 0.0)
            Pt = _truth_interp(t_ts, P, t)              # for coverage (who is near now)
            used = set()
            for nb in o["local"]["neighbors"]:
                rel = np.asarray(nb["rel"])
                Pa = _truth_interp(t_ts, P, t - age)   # stale tracks are velocity-extrapolated to t - age
                trues = {m: Pa[m] - Pa[k] for m in range(rd.n) if m != k}
                m_best = min(trues, key=lambda m: np.linalg.norm(trues[m] - rel))
                err = rel - trues[m_best]
                if np.linalg.norm(err) > 2.0:
                    continue                                   # clutter / association failure
                used.add(m_best)
                comp_err.append(err)
                dist_err.append(nb["distance"] - float(np.linalg.norm(trues[m_best])))
                stale.append(nb["staleness"])
                true_rng.append(float(np.linalg.norm(trues[m_best])))
            for m in range(rd.n):
                dist_true = float(np.linalg.norm(Pt[m] - Pt[k])) if m != k else 1e9
                if dist_true <= max_range:
                    eligible += 1
                    detected += int(m in used)
                    band = "le_2.8m" if dist_true <= 2.8 else "2.8_to_8m"
                    band_cnt[band][1] += 1
                    band_cnt[band][0] += int(m in used)
    comp_err = np.array(comp_err) if comp_err else np.zeros((0, 3))
    eps = float(rd.config["fleet_config"]["env"]["eps_rel"])
    out = {"n_samples": int(len(comp_err)), "eps_rel": eps,
           "coverage_within_%.0fm" % max_range: (detected / eligible) if eligible else None}
    for band, (dct, elg) in band_cnt.items():
        out[f"coverage_{band}"] = (dct / elg) if elg else None
    if len(comp_err):
        a = np.abs(comp_err)
        out.update({
            "abs_err_p95_xyz": np.percentile(a, 95, axis=0).round(3).tolist(),
            "abs_err_p99_xyz": np.percentile(a, 99, axis=0).round(3).tolist(),
            "abs_err_max_xyz": a.max(axis=0).round(3).tolist(),
            "frac_component_err_le_eps": float(np.mean(np.all(a <= eps, axis=1))),
            "dist_err_mean": float(np.mean(dist_err)), "dist_err_p99_abs": float(np.percentile(np.abs(dist_err), 99)),
            "staleness_p99": float(np.percentile(stale, 99)), "staleness_max": float(np.max(stale)),
        })
        rng = np.array(true_rng)
        for name, lo, hi in (("le_2.8m", 0.0, 2.8), ("2.8_to_8m", 2.8, 8.0)):
            sel = a[(rng >= lo) & (rng < hi)]
            if len(sel):
                out[f"band_{name}"] = {"n": int(len(sel)), "max_abs_xyz": sel.max(axis=0).round(3).tolist(),
                                       "p99_abs_xyz": np.percentile(sel, 99, axis=0).round(3).tolist(),
                                       "share_within_eps": float(np.mean(np.all(sel <= eps, axis=1)))}
    out["_comp_err"] = comp_err
    out["_dist_err"] = np.array(dist_err)
    return out


def structure_clearance(rd: RunData, step: int = 5) -> Dict:
    """Minimum true distance from each drone's reference point to any arena gate bar (offline)."""
    all_g, _ = mission_gates(rd)
    boxes = [(g.gate_id, b) for g in all_g for b in g.bars]
    out = {}
    for k in range(rd.n):
        P = np.stack([rd.ts[f"x_{k}"], rd.ts[f"y_{k}"], rd.ts[f"z_{k}"]], axis=1)[::step]
        best, best_gate = 1e9, None
        for gid, b in boxes:
            local = (P - b.center) @ b.axes
            d = np.linalg.norm(np.maximum(np.abs(local) - b.half, 0.0), axis=1)
            j = int(np.argmin(d))
            if d[j] < best:
                best, best_gate = float(d[j]), gid
        out[f"drone_{k}"] = {"min_clearance_m": round(best, 3), "gate": best_gate}
    edges = rd.metrics["P1_separation"].get("collision_sensor_rising_edges", [])
    out["collision_sensor_contacts"] = len(edges)
    return out


# --------------------------------------------------------------------------- figures
def _mode_series(rd: RunData, k: int):
    return np.array([s["t"] for s in rd.states[k]]), [s["mode"] for s in rd.states[k]]


def _segments(t: np.ndarray, labels: List[str]):
    segs = []
    if len(t) == 0:
        return segs
    start, cur = t[0], labels[0]
    for i in range(1, len(t)):
        if labels[i] != cur:
            segs.append((cur, start, t[i]))
            start, cur = t[i], labels[i]
    segs.append((cur, start, t[-1] + 0.1))
    return segs


def fig_trajectories(rd: RunData, out: Path) -> Path:
    plt = _style()
    from matplotlib.patches import Polygon

    all_g, mg = mission_gates(rd)
    xs = np.concatenate([rd.ts[f"x_{k}"] for k in range(rd.n)])
    ys = np.concatenate([rd.ts[f"y_{k}"] for k in range(rd.n)])
    pad = 3.0
    xlim = (np.nanmin(xs) - pad, np.nanmax(xs) + pad)
    ylim = (np.nanmin(ys) - pad, np.nanmax(ys) + pad)
    aspect = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    fig, ax = plt.subplots(figsize=(9.5, float(np.clip(8.2 * aspect + 1.4, 3.6, 8.5))))
    wp = np.array(rd.config["path_waypoints"])
    ax.plot(wp[:, 0], wp[:, 1], color=INK["muted"], lw=1.0, ls="--", label="survey line (plan)")
    ax.plot([], [], color=INK["secondary"], lw=0.8, ls=":", label="onboard estimate (dotted)")
    G = rd.config["fleet_config"]["gate"]
    mission_ids = {x.gate_id for x in mg}
    for g in all_g:
        if not (xlim[0] - 2 <= g.center[0] <= xlim[1] + 2 and ylim[0] - 2 <= g.center[1] <= ylim[1] + 2):
            continue
        left = np.array([-g.axis[1], g.axis[0], 0.0])
        a = g.center + left * (g.inner_width / 2 + 0.18)
        b = g.center - left * (g.inner_width / 2 + 0.18)
        is_m = g.gate_id in mission_ids
        ax.plot([a[0], b[0]], [a[1], b[1]], color=INK["primary"] if is_m else INK["muted"], lw=3.0,
                solid_capstyle="round", zorder=4)
        ax.annotate(g.gate_id, (g.center[0], g.center[1]), xytext=(0, 9), textcoords="offset points",
                    ha="center", fontsize=8, color=INK["secondary"])
        if is_m:
            corners = [g.from_gate_frame([s, l, 0])[:2] for s, l in
                       ((-G["cr_half_len"], -G["cr_half_width"]), (-G["cr_half_len"], G["cr_half_width"]),
                        (G["cr_half_len"], G["cr_half_width"]), (G["cr_half_len"], -G["cr_half_width"]))]
            ax.add_patch(Polygon(corners, closed=True, facecolor="#f0efec", edgecolor=INK["axis"], lw=0.8, zorder=1))
            ql = [g.from_gate_frame([G["s_queue"], l, 0])[:2] for l in (-3.5, 3.5)]
            ax.plot([ql[0][0], ql[1][0]], [ql[0][1], ql[1][1]], color=INK["axis"], lw=0.8, zorder=1)
    # formation snapshots (true positions) every ~20 s while the formation is intact, hairline triangles
    t = rd.ts["t"]
    e_lost = rd.config["fleet_config"]["form"]["e_lost"]
    if rd.n == 3:
        for ts_snap in np.arange(10.0, t[-1], 20.0):
            i = int(np.searchsorted(t, ts_snap))
            if i >= len(t):
                break
            if "form_err" in rd.ts and not (rd.ts["form_err"][i] < e_lost):
                continue
            pts = np.array([[rd.ts[f"x_{k}"][i], rd.ts[f"y_{k}"][i]] for k in range(3)])
            ax.add_patch(Polygon(pts, closed=True, fill=False, edgecolor=INK["muted"], lw=0.6, zorder=2))
    for k in range(rd.n):
        c = DRONE_COLORS[k % 3]
        ax.plot(rd.ts[f"x_{k}"], rd.ts[f"y_{k}"], color=c, lw=1.6, label=f"drone_{k} (true)", zorder=3)
        nav = np.array([s["nav_p"] for s in rd.states[k]]) if rd.states[k] else np.zeros((0, 3))
        if len(nav):
            ax.plot(nav[:, 0], nav[:, 1], color=c, lw=0.8, ls=":", zorder=3)
        ax.plot(rd.ts[f"x_{k}"][0], rd.ts[f"y_{k}"][0], "o", color=c, ms=7, mec=INK["surface"], mew=1.5, zorder=5)
        ax.plot(rd.ts[f"x_{k}"][-1], rd.ts[f"y_{k}"][-1], "s", color=c, ms=7, mec=INK["surface"], mew=1.5, zorder=5)
        xk, yk = rd.ts[f"x_{k}"], rd.ts[f"y_{k}"]
        j0 = max(0, len(xk) - 30)
        dx = float(np.sign(xk[-1] - xk[j0])) if abs(xk[-1] - xk[j0]) > 0.05 else 1.0
        ax.annotate(f"drone_{k}", (xk[-1], yk[-1]), xytext=(8 * dx, 6 + 8 * k), ha="left" if dx > 0 else "right",
                    textcoords="offset points", fontsize=8, color=INK["primary"])
    # current arrows (field at peak time) on a coarse grid
    comps = rd.config.get("current_components", [])
    if comps:
        from holo_fleet.sim.currents import CurrentComponent, CurrentField

        cf = CurrentField([CurrentComponent(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in c.items()})
                           for c in comps])
        tt = np.linspace(0, t[-1], 60)
        drifts = [np.linalg.norm(cf.drift_at(np.array([np.mean(xlim), np.mean(ylim), -4.0]), x)) for x in tt]
        t_peak = float(tt[int(np.argmax(drifts))]) if drifts else 0.0
        gx, gy = np.meshgrid(np.linspace(xlim[0] + 1, xlim[1] - 1, 10),
                             np.linspace(ylim[0] + 1, ylim[1] - 1, max(3, int(round(10 * aspect)))))
        U = np.zeros_like(gx)
        V = np.zeros_like(gx)
        best = (0.0, t_peak)
        for x_ in tt:
            m = max(np.linalg.norm(cf.drift_at(np.array([gx.flat[i], gy.flat[i], -4.0]), x_)) for i in range(gx.size))
            if m > best[0]:
                best = (m, x_)
        for i in range(gx.size):
            w = cf.drift_at(np.array([gx.flat[i], gy.flat[i], -4.0]), best[1])
            U.flat[i], V.flat[i] = w[0], w[1]
        if np.any(np.hypot(U, V) > 1e-3):
            q = ax.quiver(gx, gy, U, V, color=INK["axis"], alpha=0.8, scale=8, width=0.002, zorder=0)
            ax.quiverkey(q, 0.97, 0.05, 0.4, f"current field at t={best[1]:.0f} s (key arrow = 0.4 m/s)",
                         labelpos="W", coordinates="axes", fontproperties={"size": 7}, color=INK["axis"])
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Top-down trajectories (ground truth) - {rd.config['spec_name']}")
    ax.legend(loc="upper left", fontsize=8, ncol=3)
    path = out / "trajectories_topdown.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _shade_modes(ax, rd: RunData, modes=("SEPARATION_WARNING", "COLLISION_AVOIDANCE"), color="#f0efec"):
    for k in range(rd.n):
        t, lab = _mode_series(rd, k)
        for m, a, b in _segments(t, lab):
            if m in modes:
                ax.axvspan(a, b, color=color, lw=0, zorder=0)


def fig_distances(rd: RunData, out: Path) -> Path:
    plt = _style()
    sep = rd.config["fleet_config"]["sep"]
    fig, ax = plt.subplots(figsize=(9.5, 4.0))
    t = rd.ts["t"]
    _shade_modes(ax, rd)
    for n_pair, key in enumerate([k for k in rd.ts if k.startswith("d_")]):
        pair = key[2:]
        ax.plot(t, rd.ts[key], color=PAIR_COLORS.get(pair, INK["secondary"]), lw=1.5, label=f"drone_{pair[0]} - drone_{pair[1]}")
        i = int(np.nanargmin(rd.ts[key]))
        ax.plot(t[i], rd.ts[key][i], "o", color=PAIR_COLORS.get(pair, INK["secondary"]), ms=5, zorder=5)
        ax.annotate(f"min {rd.ts[key][i]:.2f} m ({pair[0]}-{pair[1]})", (t[i], rd.ts[key][i]),
                    xytext=(6, -12 - 11 * n_pair), textcoords="offset points", fontsize=7, color=INK["secondary"],
                    arrowprops=dict(arrowstyle="-", color=INK["axis"], lw=0.6))
    for name, val, col in (("d_warning", sep["d_warning"], STATUS["warning"]), ("d_ca", sep["d_ca"], STATUS["serious"]),
                           ("d_safe (P1)", sep["d_safe"], STATUS["critical"]), ("d_collision", sep["d_collision"], INK["primary"])):
        ax.axhline(val, color=col, lw=1.0, ls="--", zorder=1)
        ax.annotate(f"{name} = {val} m", (t[0], val), xytext=(2, 2), textcoords="offset points", fontsize=7, color=INK["secondary"])
    ymax = min(12.0, float(np.nanmax([rd.ts[k] for k in rd.ts if k.startswith("d_")])) + 0.5)
    ax.set_ylim(0, ymax)
    ax.set_xlim(t[0], t[-1])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("true centre distance [m]")
    ax.set_title("P1 - pairwise distances (ground truth); shaded: some drone in warning/avoidance")
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    path = out / "pairwise_distances.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_occupancy(rd: RunData, out: Path) -> Optional[Path]:
    keys = [k for k in rd.ts if k.startswith("occ_")]
    if not keys:
        return None
    plt = _style()
    fig, axes = plt.subplots(len(keys), 1, figsize=(9.5, 1.9 * len(keys) + 0.6), sharex=True)
    axes = np.atleast_1d(axes)
    t = rd.ts["t"]
    order = rd.metrics["P2_mutual_exclusion"]["entry_order"]
    for ax, key in zip(axes, keys):
        gid = key[4:]
        occ = rd.ts[key]
        ax.step(t, occ, where="post", color=INK["secondary"], lw=1.4)
        bad = occ > 1
        if bad.any():
            ax.fill_between(t, 0, occ, where=bad, step="post", color=STATUS["critical"], alpha=0.5, label="violation")
        # who is inside: colour bars per drone along the bottom
        G = rd.config["fleet_config"]["gate"]
        _, mg = mission_gates(rd)
        g = next(x for x in mg if x.gate_id == gid)
        for k in range(rd.n):
            P = np.stack([rd.ts[f"x_{k}"], rd.ts[f"y_{k}"], rd.ts[f"z_{k}"]], axis=1)
            inside = np.array([abs(s) <= G["cr_half_len"] and abs(l) <= G["cr_half_width"] and abs(dz) <= G["cr_half_height"]
                               for s, l, dz in (g.to_gate_frame(p) for p in P)])
            for m, a, b in _segments(t, ["in" if v else "out" for v in inside]):
                if m == "in":
                    ax.axvspan(a, b, ymin=0.0, ymax=0.18, color=DRONE_COLORS[k % 3], lw=0)
                    ax.annotate(f"drone_{k}", ((a + b) / 2, 0.05), ha="center", fontsize=7, color=INK["primary"],
                                xycoords=("data", "axes fraction"))
        ax.axhline(1, color=STATUS["critical"], lw=1.0, ls="--")
        ax.annotate("P2 limit: 1", (t[0], 1), xytext=(2, 2), textcoords="offset points", fontsize=7, color=INK["secondary"])
        ax.set_ylim(0, max(2.2, float(np.nanmax(occ)) + 0.4))
        ax.set_yticks([0, 1, 2])
        ax.set_ylabel(f"{gid} occupancy")
        ax.set_title(f"P2 - drones inside the critical region of {gid} (entry order: {', '.join(order.get(gid, []))})")
    axes[-1].set_xlabel("time [s]")
    axes[-1].set_xlim(t[0], t[-1])
    path = out / "critical_region_occupancy.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_formation(rd: RunData, out: Path) -> Optional[Path]:
    if "form_err" not in rd.ts:
        return None
    plt = _style()
    F = rd.config["fleet_config"]["form"]
    fig, ax = plt.subplots(figsize=(9.5, 3.8))
    t = rd.ts["t"]
    pert = rd.ts.get("perturbation")
    if pert is not None:
        for m, a, b in _segments(t, ["p" if v > 0.5 else "-" for v in np.nan_to_num(pert)]):
            if m == "p":
                ax.axvspan(a, b, color="#f0efec", lw=0, zorder=0)
    ax.plot(t, rd.ts["form_err"], color=INK["primary"], lw=1.8, label="true formation error (referee)", zorder=4)
    for k in range(rd.n):
        tt = [o["t"] for o in rd.obs[k]]
        fe = [min(o["abstract"]["form_err"], 5.0) for o in rd.obs[k]]
        ax.plot(tt, fe, color=DRONE_COLORS[k % 3], lw=0.9, alpha=0.9, label=f"drone_{k} onboard estimate", zorder=3)
    for name, val, col in (("e_lost", F["e_lost"], STATUS["serious"]), ("e_ok", F["e_ok"], STATUS["good"])):
        ax.axhline(val, color=col, lw=1.0, ls="--")
        ax.annotate(f"{name} = {val} m", (t[-1], val), xytext=(-60, 3), textcoords="offset points", fontsize=7,
                    color=INK["secondary"])
    for ep in rd.metrics["P3_formation_recovery"]["episodes"]:
        if ep.get("t_recovered") is not None:
            ax.axvline(ep["t_recovered"], color=STATUS["good"], lw=1.0)
            lab = ep.get("recovery_after_perturbation")
            ax.annotate(f"recovered\n{lab:.1f}s after\nperturbation" if lab is not None else "recovered",
                        (ep["t_recovered"], 2.6), xytext=(3, 0), textcoords="offset points",
                        fontsize=7, color=INK["secondary"], va="top")
    ax.set_ylim(0, 3.0)
    ax.set_xlim(t[0], t[-1])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("formation error [m]")
    ax.set_title("P3 - formation error; shaded: perturbation (gate/avoidance/gust); estimates clipped at 5 m")
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    path = out / "formation_error.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_modes(rd: RunData, out: Path) -> Path:
    plt = _style()
    fig, axes = plt.subplots(rd.n, 1, figsize=(9.5, 1.55 * rd.n + 0.6), sharex=True)
    axes = np.atleast_1d(axes)
    for k, ax in enumerate(axes):
        t, lab = _mode_series(rd, k)
        for m, a, b in _segments(t, lab):
            y = MODE_ORDER.index(m) if m in MODE_ORDER else 0
            ax.broken_barh([(a, b - a)], (y - 0.35, 0.7), color=DRONE_COLORS[k % 3], lw=0)
        ax.set_yticks(range(len(MODE_ORDER)))
        ax.set_yticklabels([MODE_SHORT[m] for m in MODE_ORDER], fontsize=7)
        ax.set_ylim(-0.6, len(MODE_ORDER) - 0.4)
        ax.invert_yaxis()
        ax.set_title(f"drone_{k} - hybrid automaton mode", fontsize=9)
        ax.grid(axis="y", visible=False)
    axes[-1].set_xlabel("time [s]")
    path = out / "automaton_modes.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_depth(rd: RunData, out: Path) -> Path:
    plt = _style()
    fig, ax = plt.subplots(figsize=(9.5, 2.8))
    t = rd.ts["t"]
    _shade_modes(ax, rd, modes=("COLLISION_AVOIDANCE", "FAILSAFE_HOLD_OR_RETREAT"))
    for k in range(rd.n):
        ax.plot(t, rd.ts[f"z_{k}"], color=DRONE_COLORS[k % 3], lw=1.4, label=f"drone_{k}")
    ax.set_xlim(t[0], t[-1])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("depth z [m] (up +)")
    ax.set_title("Vertical motion (3D escape / failsafe layering); shaded: avoidance or failsafe active")
    ax.legend(loc="lower right", fontsize=8, ncol=3)
    path = out / "depth_profile.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_perception(rd: RunData, out: Path, stats: Optional[Dict] = None) -> Optional[Path]:
    stats = stats or perception_errors(rd)
    E = stats["_comp_err"]
    if len(E) == 0:
        return None
    plt = _style()
    eps = rd.config["fleet_config"]["env"]["eps_rel"]
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 2.9), sharey=True)
    bins = np.linspace(-0.6, 0.6, 49)
    for i, (ax, name) in enumerate(zip(axes, ("x", "y", "z"))):
        ax.hist(np.clip(E[:, i], -0.6, 0.6), bins=bins, color=DRONE_COLORS[0], lw=0)
        for v in (-eps, eps):
            ax.axvline(v, color=STATUS["critical"], lw=1.0, ls="--")
        inside = float(np.mean(np.abs(E[:, i]) <= eps))
        ax.set_title(f"relative {name} error: {inside * 100:.1f}% within eps", fontsize=9)
        ax.set_xlabel("estimate - truth [m]")
    axes[0].set_ylabel("samples")
    cov = [v for k, v in stats.items() if k.startswith("coverage")][0]
    fig.suptitle(f"Perception vs ground truth (offline): {stats['n_samples']} neighbour estimates, "
                 f"detection coverage within 8 m = {cov * 100:.1f}%  (eps_rel = {eps} m)", fontsize=9)
    path = out / "perception_error.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def make_all_figures(run_dir, copy_to: Optional[Path] = None) -> Dict[str, str]:
    rd = load_run(run_dir)
    out = rd.run_dir / "figures"
    out.mkdir(exist_ok=True)
    stats = perception_errors(rd)
    made = {}
    for fn in (fig_trajectories, fig_distances, fig_occupancy, fig_formation, fig_modes, fig_depth):
        p = fn(rd, out)
        if p is not None:
            made[p.stem] = str(p)
    p = fig_perception(rd, out, stats)
    if p is not None:
        made[p.stem] = str(p)
    clean = {k: v for k, v in stats.items() if not k.startswith("_")}
    (rd.run_dir / "perception_stats.json").write_text(json.dumps(clean, indent=2))
    if copy_to is not None:
        import shutil

        copy_to.mkdir(parents=True, exist_ok=True)
        for stem, p in made.items():
            shutil.copy(p, copy_to / f"{rd.name}__{stem}.png")
    return made
