"""v2 figures from the demonstration runs (results/v2/demos) and the sensor geometry.

    python scripts/make_figures_v2.py            # all figures that have data
    python scripts/make_figures_v2.py sensor     # only the sensor figures (no runs needed)

Writes figures/v2/{sensor,p1,p2,p3}/*.png and copies the demo GIFs to figures/v2/gifs/.  Ground truth
is read here for judging and plotting only (offline), never by a controller.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception import sonar_geometry as sg  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402

DEMOS = ROOT / "results" / "v2" / "demos"
FIG = ROOT / "figures" / "v2"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INK = {"primary": "#0b0b0b", "secondary": "#52514e", "muted": "#898781", "grid": "#e1e0d9", "axis": "#c3c2b7",
       "surface": "#fcfcfb"}
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
MODE_COLOR = {"FORMATION_FOLLOW": "#0ca30c", "FORMATION_RECOVERY": "#fab219", "SEPARATION_WARNING": "#ec835a",
              "COLLISION_AVOIDANCE": "#d03b3b", "FAILSAFE_HOLD_OR_RETREAT": "#4a3aa7", "GATE_APPROACH": "#86b6ef",
              "GATE_YIELD": "#2a78d6", "GATE_PASS": "#184f95"}
MODE_SHORT = {"FORMATION_FOLLOW": "follow", "FORMATION_RECOVERY": "recovery", "SEPARATION_WARNING": "sep. warning",
              "COLLISION_AVOIDANCE": "coll. avoidance", "FAILSAFE_HOLD_OR_RETREAT": "failsafe", "GATE_APPROACH": "gate approach",
              "GATE_YIELD": "gate yield", "GATE_PASS": "gate pass"}


def style():
    plt.rcParams.update({
        "figure.facecolor": INK["surface"], "axes.facecolor": INK["surface"], "savefig.facecolor": INK["surface"],
        "axes.edgecolor": INK["axis"], "axes.labelcolor": INK["secondary"], "xtick.color": INK["muted"],
        "ytick.color": INK["muted"], "text.color": INK["primary"], "axes.grid": True, "grid.color": INK["grid"],
        "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
        "font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"], "font.size": 9, "axes.titlesize": 10,
        "axes.titleweight": "bold", "legend.frameon": False, "lines.linewidth": 1.6})


def save(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  ", path.relative_to(ROOT))


# ---------------------------------------------------------------------------------------------- data
def load(name):
    d = DEMOS / name
    if not (d / "referee_timeseries.csv").exists():
        return None
    status = json.loads((d / "run_status.json").read_text(encoding="utf-8")).get("status")
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    cfg = json.loads((d / "run_config.json").read_text(encoding="utf-8"))
    met = json.loads((d / "referee_metrics.json").read_text(encoding="utf-8"))
    n = cfg["n_drones"]
    f = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else np.nan  # noqa: E731
    ts = {"t": np.array([f(r, "t") for r in rows])}
    for k in range(n):
        for a in "xyz":
            ts[f"{a}{k}"] = np.array([f(r, f"{a}{k}") for r in rows])
    for key in rows[0]:
        if key.startswith(("d_", "occ_", "ix", "iy", "iz")) or key in ("form_err", "cur_x", "cur_y", "cur_z"):
            ts[key] = np.array([f(r, key) for r in rows])
    ts["modes"] = [r["modes"].split("|") for r in rows]
    states = []
    for k in range(n):
        p = d / f"drone_{k}_state.jsonl"
        states.append([json.loads(l) for l in open(p, encoding="utf-8")] if p.exists() else [])
    return {"name": name, "status": status, "n": n, "ts": ts, "cfg": cfg, "met": met, "states": states}


def pair_min(ts, n):
    keys = [f"d_{i}_{j}" for i in range(n) for j in range(i + 1, n)]
    return np.nanmin(np.stack([ts[k] for k in keys]), axis=0)


def onboard_min(run):
    t = np.array([r["t"] for r in run["states"][0]])
    vals = np.full((run["n"], len(t)), np.nan)
    for k, st in enumerate(run["states"]):
        for i, r in enumerate(st[:len(t)]):
            if r.get("d_min") is not None:
                vals[k, i] = r["d_min"]
    return t, np.nanmin(vals, axis=0)


def thresholds(ax, labels=True):
    sep = DEFAULT.sep
    for v, c, lab in ((sep.d_warning, STATUS["serious"], "d_warning (onboard guard)"), (sep.d_ca, STATUS["critical"], "d_ca"),
                      (sep.d_safe, INK["primary"], "d_safe (P1, ground truth)")):
        ax.axhline(v, color=c, lw=1.0, ls="--" if v != sep.d_safe else "-")
        if labels:
            ax.annotate(lab, (1.0, v), xycoords=("axes fraction", "data"), xytext=(-4, 3), textcoords="offset points",
                        ha="right", fontsize=8, color=INK["secondary"])


def mode_timeline(ax, run, names=None):
    t = run["ts"]["t"]
    for k in range(run["n"]):
        modes = [m[k] for m in run["ts"]["modes"]]
        start = 0
        for i in range(1, len(modes) + 1):
            if i == len(modes) or modes[i] != modes[start]:
                ax.barh(k, t[min(i, len(t) - 1)] - t[start], left=t[start], height=0.7, color=MODE_COLOR.get(modes[start], "#ccc"),
                        edgecolor=INK["surface"], linewidth=0.4)
                start = i
    ax.set_yticks(range(run["n"]))
    ax.set_yticklabels(names or [f"drone {k}" for k in range(run["n"])])
    ax.invert_yaxis()
    ax.grid(False)
    used = sorted({m for row in run["ts"]["modes"] for m in row}, key=list(MODE_COLOR).index)
    ax.legend(handles=[Patch(color=MODE_COLOR[m], label=MODE_SHORT[m]) for m in used], ncol=min(4, len(used)),
              loc="upper center", bbox_to_anchor=(0.5, -0.32), fontsize=8)
    ax.set_xlabel("time [s]")


# ---------------------------------------------------------------------------------------------- sensor
def fig_sensor():
    fig = plt.figure(figsize=(13, 5.2))
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    hx, hy, hz = sg.HULL_HALF
    corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    edges = [(a, b) for a in range(8) for b in range(a + 1, 8) if np.sum(np.abs(corners[a] - corners[b]) > 1e-9) == 1]
    for a, b in edges:
        ax.plot(*zip(corners[a], corners[b]), color=INK["primary"], lw=1.0)
    L = 1.0
    for k, s in enumerate(sg.SECTORS):
        m, a = sg.MOUNTS[s], sg.AXES[s]
        u = np.cross(a, [0, 0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(a, u)
        th = np.linspace(0, 2 * np.pi, 48)
        rr = np.linspace(0.0, L, 8)
        T, Rr = np.meshgrid(th, rr)
        rad = Rr * math.tan(math.radians(60.0))
        X = m[0] + Rr * a[0] + rad * (np.cos(T) * u[0] + np.sin(T) * v[0])
        Y = m[1] + Rr * a[1] + rad * (np.cos(T) * u[1] + np.sin(T) * v[1])
        Z = m[2] + Rr * a[2] + rad * (np.cos(T) * u[2] + np.sin(T) * v[2])
        ax.plot_surface(X, Y, Z, color=SERIES[k], alpha=0.10, linewidth=0, shade=False)
        ax.plot(X[-1], Y[-1], Z[-1], color=SERIES[k], lw=0.9)
        ax.scatter(*m, color=SERIES[k], s=22, depthshade=False)
        ax.text(*(m + (L + 0.25) * a), s.lower(), color=SERIES[k], fontsize=8, ha="center")
    for rad, c, lab in ((DEFAULT.sep.d_safe, INK["primary"], "d_safe 1.0 m"), (DEFAULT.sep.d_warning, STATUS["serious"], "d_warning 2.4 m")):
        ph = np.linspace(0, 2 * np.pi, 80)
        ax.plot(rad * np.cos(ph), rad * np.sin(ph), 0 * ph, color=c, lw=1.0, ls="--")
        ax.plot(rad * np.cos(ph), 0 * ph, rad * np.sin(ph), color=c, lw=0.6, ls=":")
        ax.text(rad * 0.75, -rad * 0.75, 0, lab, color=c, fontsize=8)
    lim = 2.6
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x fwd [m]"); ax.set_ylabel("y left [m]"); ax.set_zlabel("z up [m]")
    ax.set_title("six identical 120 deg sonars on the hull faces\n(cones drawn to 1 m; centre-distance shells d_safe, d_warning)")
    # coverage map at 1 m: how many sectors see a hull
    ax2 = fig.add_subplot(1, 3, 2)
    az = np.radians(np.linspace(-180, 180, 361))
    el = np.radians(np.linspace(-89, 89, 179))
    A, E = np.meshgrid(az, el)
    U = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1).reshape(-1, 3)
    cnt = np.zeros(len(U))
    for s in sg.SECTORS:
        cnt += sg.ball_touches_cone(1.0 * U, s, sg.R_IN)
    cnt = cnt.reshape(A.shape)
    cmap = matplotlib.colors.ListedColormap(["#d03b3b", "#cde2fb", "#6da7ec", "#1c5cab"])
    im = ax2.pcolormesh(np.degrees(A), np.degrees(E), np.clip(cnt, 0, 3), cmap=cmap, vmin=-0.5, vmax=3.5, shading="auto")
    for name, d in sg.diagonal_directions().items():
        ax2.plot(math.degrees(math.atan2(d[1], d[0])), math.degrees(math.asin(d[2])), "o", ms=4, color=INK["primary"])
    cb = fig.colorbar(im, ax=ax2, ticks=[0, 1, 2, 3], fraction=0.046)
    cb.ax.set_yticklabels(["blind", "1 sector", "2 (overlap)", "3 (corner)"])
    ax2.set_xlabel("azimuth [deg] (left +)")
    ax2.set_ylabel("elevation [deg]")
    ax2.set_title("sectors that see a BlueROV2 at 1 m\n(dots: the 8 cube diagonals)")
    ax2.grid(False)
    # coverage vs centre distance: near-field pockets
    ax3 = fig.add_subplot(1, 3, 3)
    rho = np.linspace(0.45, 3.0, 52)
    for rad, lab, c in ((sg.R_IN, "hull = inscribed ball (worst)", SERIES[0]), (sg.R_OUT, "hull = circumscribed ball", SERIES[1])):
        cov = [1.0 - sg.coverage(r, rad, n=6000) for r in rho]
        ax3.plot(rho, np.array(cov) * 100.0, color=c, label=lab)
    ax3.axvline(DEFAULT.sep.d_safe, color=INK["primary"], lw=1.0)
    ax3.annotate("d_safe", (DEFAULT.sep.d_safe, 0.95), xycoords=("data", "axes fraction"), xytext=(3, 0),
                 textcoords="offset points", fontsize=8)
    ax3.set_xlabel("centre distance of the other drone [m]")
    ax3.set_ylabel("directions not seen by any sonar [%]")
    ax3.set_title("near-field pockets close to the hull\n(none beyond 0.8 m; HoloOcean bench: 204/204 detected)")
    ax3.legend(fontsize=8)
    fig.tight_layout()
    save(fig, FIG / "sensor" / "six_sonar_coverage.png")


# ---------------------------------------------------------------------------------------------- P1
def fig_p1():
    run = load("p1_head_on")
    if run:
        ts = run["ts"]
        fig, ax = plt.subplots(figsize=(7.5, 3.4))
        t_on, d_on = onboard_min(run)
        ax.plot(ts["t"], pair_min(ts, run["n"]), color=SERIES[0], label="true distance (referee)")
        ax.plot(t_on, d_on, color=SERIES[1], label="onboard conservative distance (min of both drones)")
        thresholds(ax)
        ax.set_ylim(0, 13)
        ax.set_xlabel("time [s]"); ax.set_ylabel("centre distance [m]")
        ax.set_title(f"p1_head_on: both give way to the right, closest {run['met']['P1_separation']['min_distance']:.2f} m")
        ax.legend(loc="upper right", fontsize=8)
        save(fig, FIG / "p1" / "p1_head_on_distance.png")
    run = load("p1_vertical_escape")
    if run:
        ts = run["ts"]
        fig, axs = plt.subplots(3, 1, figsize=(8, 8.2), gridspec_kw={"height_ratios": [1.1, 1.1, 0.8]}, sharex=True)
        ax = axs[0]
        ax.plot(ts["t"], pair_min(ts, run["n"]), color=SERIES[0], label="closest pair of drones (P1)")
        if "d_intruder" in ts:
            ax.plot(ts["t"], ts["d_intruder"], color=SERIES[6], label="clearance to the scripted vehicle")
        thresholds(ax)
        ax.set_ylim(0, 8)
        ax.set_ylabel("distance [m]")
        ax.set_title("p1_vertical_escape: traffic rule off, safety layer only")
        ax.legend(loc="upper right", fontsize=8)
        ax = axs[1]
        for k in range(run["n"]):
            ax.plot(ts["t"], ts[f"z{k}"], color=SERIES[k], label=f"drone {k}" + (" (centre, boxed in)" if k == 1 else ""))
        if "iz0" in ts:
            ax.plot(ts["t"], ts["iz0"], color=INK["muted"], ls="--", label="scripted vehicle")
        ax.set_ylabel("depth z [m]")
        ax.legend(ncol=3, fontsize=8, loc="lower left")
        mode_timeline(axs[2], run)
        fig.tight_layout()
        save(fig, FIG / "p1" / "p1_vertical_escape.png")
    run = load("p1_two_lines")
    if run:
        ts = run["ts"]
        fig, axs = plt.subplots(1, 2, figsize=(11, 3.4))
        axs[0].plot(ts["t"], pair_min(ts, run["n"]), color=SERIES[0], label="closest pair (true)")
        thresholds(axs[0])
        axs[0].set_ylim(0, 8); axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("distance [m]")
        axs[0].set_title("p1_two_lines: six drones, two lines head-on")
        for k in range(run["n"]):
            axs[1].plot(ts["t"], ts[f"z{k}"], color=SERIES[k], label=f"drone {k}")
        axs[1].set_xlabel("time [s]"); axs[1].set_ylabel("depth z [m]")
        axs[1].set_title("flanked drones give way vertically, outer ones to the right")
        axs[1].legend(ncol=3, fontsize=8)
        fig.tight_layout()
        save(fig, FIG / "p1" / "p1_two_lines.png")


# ---------------------------------------------------------------------------------------------- P2
def fig_p2(name):
    run = load(name)
    if not run:
        return
    ts = run["ts"]
    sc = SCENARIOS[name](DEFAULT)
    g = sc.judged_gates[0]
    G = DEFAULT.gate
    fig = plt.figure(figsize=(12, 7.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1.0])
    ax = fig.add_subplot(gs[0, 0])

    def gf(k):
        P = np.stack([ts[f"x{k}"], ts[f"y{k}"], ts[f"z{k}"]], axis=1)
        return np.array([g.to_gate_frame(p) for p in P])

    for b in g.bars:
        q = g.to_gate_frame(b.center)
        ax.add_patch(plt.Rectangle((q[0] - 0.11, q[1] - (0.93 if b.half[1] > 0.5 else 0.09)), 0.22,
                                   1.86 if b.half[1] > 0.5 else 0.18, color=INK["secondary"]))
    ax.add_patch(plt.Rectangle((-G.cr_half_len, -G.cr_half_width), 2 * G.cr_half_len, 2 * G.cr_half_width, fill=False,
                               ec=SERIES[0], lw=1.0, ls="--"))
    ax.annotate("critical region", (G.cr_half_len, G.cr_half_width), xytext=(3, 3), textcoords="offset points", fontsize=8,
                color=SERIES[0])
    for p in sc.plans:
        ax.plot(p.queue_s, p.queue_lateral, marker="x", color=INK["muted"], ms=7)
    for k in range(run["n"]):
        q = gf(k)
        ax.plot(q[:, 0], q[:, 1], color=SERIES[k], lw=1.4, label=f"drone {k}")
        ax.plot(q[0, 0], q[0, 1], "o", color=SERIES[k], ms=4)
    ax.set_aspect("equal")
    ax.set_xlim(-14, 12); ax.set_ylim(-6, 6)
    ax.set_xlabel("s along the gate axis [m]"); ax.set_ylabel("l lateral, left + [m]")
    ax.set_title(f"{name}: trajectories in the gate frame (x = queue points)")
    ax.legend(fontsize=8, ncol=3, loc="lower left")
    ax = fig.add_subplot(gs[0, 1])
    occ = ts[f"occ_{g.gate_id}"]
    ax.step(ts["t"], occ, where="post", color=SERIES[0], label="drones inside the CR (referee)")
    ax.axhline(1, color=STATUS["critical"], lw=1.0, ls="--")
    ax.annotate("P2 bound: 1", (0.0, 1), xycoords=("axes fraction", "data"), xytext=(3, 3), textcoords="offset points",
                fontsize=8, color=STATUS["critical"])
    order = run["met"]["P2_mutual_exclusion"]["entry_order"][g.gate_id]
    for k in range(run["n"]):
        inside = np.array([abs(a) <= G.cr_half_len and abs(b) <= G.cr_half_width and abs(c) <= G.cr_half_height
                           for a, b, c in gf(k)])
        if inside.any():
            t_in = ts["t"][np.argmax(inside)]
            ax.annotate(f"drone {k}", (t_in, 1.05), xytext=(0, 6), textcoords="offset points", fontsize=8, color=SERIES[k],
                        ha="left", rotation=0)
    ax.set_ylim(-0.1, 2.2); ax.set_yticks([0, 1, 2])
    ax.set_xlabel("time [s]"); ax.set_ylabel("CR occupancy")
    ax.set_title(f"one at a time: max occupancy {max(run['met']['P2_mutual_exclusion']['max_occupancy'].values())}, "
                 f"order {' > '.join(o[-1] for o in order)}")
    ax.legend(fontsize=8, loc="upper right")
    mode_timeline(fig.add_subplot(gs[1, :]), run)
    fig.tight_layout()
    save(fig, FIG / "p2" / f"{name}.png")


# ---------------------------------------------------------------------------------------------- P3
def fig_p3_gust():
    run = load("formation_gust")
    if not run:
        return
    ts = run["ts"]
    rc = DEFAULT.ref
    fig, axs = plt.subplots(3, 1, figsize=(8, 8.4), sharex=True, gridspec_kw={"height_ratios": [1.2, 1.0, 0.8]})
    ax = axs[0]
    for a, b in run["cfg"]["disturbance_windows"]:
        ax.axvspan(a, b, color=STATUS["warning"], alpha=0.15, lw=0)
        ax.annotate("0.85 m/s jet (beyond the envelope)", (a, 1.0), xycoords=("data", "axes fraction"), xytext=(3, -12),
                    textcoords="offset points", fontsize=8, color=INK["secondary"])
    ax.plot(ts["t"], ts["form_err"], color=SERIES[0], label="formation error (referee, translation-invariant)")
    ax.axhline(rc.e_lost, color=STATUS["critical"], ls="--", lw=1.0)
    ax.axhline(rc.e_ok, color=STATUS["good"], ls="--", lw=1.0)
    ax.annotate("lost above", (1.0, rc.e_lost), xycoords=("axes fraction", "data"), xytext=(-4, 3), textcoords="offset points",
                ha="right", fontsize=8, color=STATUS["critical"])
    ax.annotate("recovered below (2 s)", (1.0, rc.e_ok), xycoords=("axes fraction", "data"), xytext=(-4, 3),
                textcoords="offset points", ha="right", fontsize=8, color=STATUS["good"])
    for e in run["met"]["P3_formation_recovery"]["episodes"]:
        ax.axvline(e["t_lost"], color=STATUS["critical"], lw=0.8)
        if e["t_recovered"] is not None:
            ax.axvline(e["t_recovered"], color=STATUS["good"], lw=0.8)
            ax.annotate(f"recovered after {e['recovery_time_s']:.1f} s", (e["t_recovered"], 0.8), xycoords=("data", "axes fraction"),
                        xytext=(4, 0), textcoords="offset points", fontsize=8)
    ax.set_ylabel("error [m]")
    ax.set_title("formation_gust: lost under the jet, recovered after it (P3)")
    ax.legend(fontsize=8, loc="center right")
    ax = axs[1]
    jet_k = 0
    st = run["states"][jet_k]
    t_s = np.array([r["t"] for r in st])
    ce = np.array([r["current_est"] for r in st])
    ax.plot(t_s, np.linalg.norm(ce[:, :2], axis=1), color=SERIES[jet_k], label=f"drone {jet_k}: onboard current estimate")
    ax.set_ylabel("|current| [m/s]")
    ax.set_title("the drone only knows its own estimate (integrator of the velocity loop)")
    ax.legend(fontsize=8)
    mode_timeline(axs[2], run)
    fig.tight_layout()
    save(fig, FIG / "p3" / "formation_gust.png")


def fig_p3_overview():
    runs = [r for r in (load("formation_triangle"), load("formation_square"), load("formation_six")) if r]
    if not runs:
        return
    fig, axs = plt.subplots(2, len(runs), figsize=(4.2 * len(runs), 6.4))
    axs = np.atleast_2d(axs).reshape(2, len(runs))
    for c, run in enumerate(runs):
        ts = run["ts"]
        ax = axs[0, c]
        for k in range(run["n"]):
            ax.plot(ts[f"x{k}"], ts[f"y{k}"], color=SERIES[k], lw=1.2, label=f"drone {k}")
            ax.plot(ts[f"x{k}"][0], ts[f"y{k}"][0], "o", color=SERIES[k], ms=3)
        cur = run["cfg"]["current"][0]["drift"]
        ax.annotate("", xy=(0.9, 0.12 + 0.3 * cur[1]), xytext=(0.9, 0.12), xycoords="axes fraction",
                    arrowprops={"arrowstyle": "->", "color": INK["secondary"]})
        ax.set_aspect("equal")
        ax.set_title(f"{run['name']} ({run['n']} drones)")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        if run["n"] <= 4:
            ax.legend(fontsize=7, ncol=2)
        ax = axs[1, c]
        ax.plot(ts["t"], ts["form_err"], color=SERIES[0])
        ax.axhline(DEFAULT.ref.e_lost, color=STATUS["critical"], ls="--", lw=1.0)
        ax.axhline(DEFAULT.ref.e_ok, color=STATUS["good"], ls="--", lw=1.0)
        ax.set_ylim(0, 1.4)
        ax.set_xlabel("time [s]"); ax.set_ylabel("formation error [m]")
        ax.set_title(f"max {np.nanmax(ts['form_err']):.2f} m (lost above {DEFAULT.ref.e_lost} m)")
    fig.tight_layout()
    save(fig, FIG / "p3" / "formations_triangle_square_six.png")


def copy_gifs():
    for gif in DEMOS.glob("*/*.gif"):
        dst = FIG / "gifs" / gif.name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(gif, dst)
        print("  ", dst.relative_to(ROOT), f"{gif.stat().st_size / 1e6:.1f} MB")


def main(argv) -> int:
    style()
    what = set(argv) or {"sensor", "p1", "p2", "p3", "gifs"}
    if "sensor" in what:
        fig_sensor()
    if "p1" in what:
        fig_p1()
    if "p2" in what:
        fig_p2("gate_single")
        fig_p2("integrated_short")
    if "p3" in what:
        fig_p3_gust()
        fig_p3_overview()
    if "gifs" in what:
        copy_gifs()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
