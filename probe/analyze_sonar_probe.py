"""Figures and numbers for the single-beam sonar probe (probe/probe_singlebeam.py output).

Writes figures/v2/sonar_probe/*.png and results/v2/sonar_probe/summary.json.  Ground truth is used freely:
this is the calibration bench, not the controller.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "probe"))
sys.path.insert(0, str(ROOT))

from sonar_probe_lib import (BIN_W, RANGES, first_return, group, hull_near_distance, load,  # noqa: E402
                             sonar_origin)

OUT_RAW = ROOT / "probe" / "out" / "sonar_probe"
FIG = ROOT / "figures" / "v2" / "sonar_probe"
RES = ROOT / "results" / "v2" / "sonar_probe"
THR_CLEAN = 1e-3          # noise-free output: any echo
INK = {"primary": "#1f2328", "secondary": "#59636e", "muted": "#8c959f", "grid": "#e6e8eb"}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4", "#008300"]
STATUS = {"critical": "#d1242f", "warning": "#bf8700"}


def style():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK["muted"], "axes.labelcolor": INK["primary"],
                         "xtick.color": INK["secondary"], "ytick.color": INK["secondary"], "axes.grid": True,
                         "grid.color": INK["grid"], "grid.linewidth": 0.6, "axes.spines.top": False,
                         "axes.spines.right": False, "legend.frameon": False})
    return plt


def gate_truth(r) -> float:
    """Distance from the sonar origin to the nearest face of the G06 bar it is aimed at (gate frame)."""
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates

    g = {x.gate_id: x for x in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    p = sonar_origin(r["obs_pose"])
    from sonar_probe_lib import box_surface_distance

    return min(box_surface_distance(p, b.center, b.axes, b.half) for b in g.bars)


def box_truth(r) -> float:
    from probe_singlebeam import BOX_POS, BOX_SIZE  # noqa: WPS433 (probe constants)

    from sonar_probe_lib import box_surface_distance

    return box_surface_distance(sonar_origin(r["obs_pose"]), BOX_POS, np.eye(3), np.full(3, BOX_SIZE / 2))


def range_table(rows, exp, truth_fn, key_fn, thr=THR_CLEAN):
    out = []
    for _case, rs in group(rows, exp).items():
        est = [first_return(r["sonar"], thr) for r in rs]
        det = [e for e in est if e is not None]
        truth = truth_fn(rs[0])
        out.append({"key": key_fn(rs[0]["case"]), "truth": truth, "n": len(rs), "detected": len(det),
                    "est_mean": float(np.mean(det)) if det else None,
                    "err_mean": float(np.mean(det) - truth) if det else None,
                    "err_max_abs": float(np.max(np.abs(np.asarray(det) - truth))) if det else None})
    return out


def main() -> int:
    plt = style()
    FIG.mkdir(parents=True, exist_ok=True)
    RES.mkdir(parents=True, exist_ok=True)
    summary = {"bin_width_m": BIN_W}
    sess = {s: load(OUT_RAW / s) for s in ("k1", "k2", "k3", "k2_oa120", "k2_oa30")
            if (OUT_RAW / s / "samples.jsonl").exists()}
    events = {s: json.loads((OUT_RAW / s / "events.json").read_text()) for s in sess if (OUT_RAW / s / "events.json").exists()}
    summary["events"] = events
    k2 = sess.get("k2", [])

    # ---------------------------------------------------------------- visibility matrix (k1 / k2 / k3)
    vis = {}
    for s, rows in sess.items():
        if s not in ("k1", "k2", "k3"):
            continue
        for exp in ("gate", "box", "drone"):
            for _case, rs in group(rows, exp).items():
                det = np.mean([first_return(r["sonar"], THR_CLEAN) is not None for r in rs])
                case = rs[0]["case"]
                label = exp + ":" + ",".join(f"{k}={v}" for k, v in sorted(case.items()))
                vis.setdefault(label, {})[s] = round(float(det), 2)
    summary["visibility"] = vis

    # ---------------------------------------------------------------- range accuracy (noise-free)
    tables = {
        "drone_head_on": range_table([r for r in k2 if r["case"].get("aspect") == "head_on"], "drone",
                                     lambda r: hull_near_distance(r["obs_pose"], r["tgt_pose"]),
                                     lambda c: c["d_centre_from_sonar"]),
        "drone_broadside": range_table([r for r in k2 if r["case"].get("aspect") == "broadside"], "drone",
                                       lambda r: hull_near_distance(r["obs_pose"], r["tgt_pose"]),
                                       lambda c: c["d_centre_from_sonar"]),
        "box": range_table(k2, "box", box_truth, lambda c: c["d_face"]),
        "gate_left_pillar": range_table([r for r in k2 if r["case"].get("aim") == "left_pillar"], "gate", gate_truth,
                                        lambda c: c["d_plane"]),
        "gate_top_bar": range_table([r for r in k2 if r["case"].get("aim") == "top_bar"], "gate", gate_truth,
                                    lambda c: c["d_plane"]),
        "gate_centre": range_table([r for r in k2 if r["case"].get("aim") == "centre"], "gate", gate_truth,
                                   lambda c: c["d_plane"]),
    }
    summary["range_tables"] = tables
    if k2:
        fig, (ax, ax2) = plt.subplots(2, 1, figsize=(7.2, 6.4), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
        lim = 12.5
        ax.plot([0, lim], [0, lim], color=INK["muted"], lw=1, ls="--", label="ideal (estimate = truth)")
        for k, (name, tab) in enumerate(tables.items()):
            pts = [(t["truth"], t["est_mean"]) for t in tab if t["est_mean"] is not None]
            miss = [t["truth"] for t in tab if t["est_mean"] is None]
            if pts:
                x, y = np.array(pts).T
                ax.plot(x, y, marker="o", ms=4, lw=1.2, color=SERIES[k], label=name.replace("_", " "))
                ax2.plot(x, y - x, marker="o", ms=4, lw=1.2, color=SERIES[k])
            if miss:
                ax.scatter(miss, np.zeros(len(miss)) + 0.15, marker="x", s=28, color=SERIES[k])
        ax.set_ylabel("sonar first-return range [m]")
        ax.set_title("SinglebeamSonar (OpeningAngle 120 deg, 5 cm bins): range vs true near-face distance\n"
                     "x on the axis = no detection", fontsize=9)
        ax.legend(fontsize=7.5, ncol=2, loc="upper left")
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax2.axhline(0, color=INK["muted"], lw=1)
        ax2.axhspan(-BIN_W, BIN_W, color=INK["grid"], alpha=0.8, lw=0, label="+/- one range bin")
        ax2.set_xlabel("true distance from the sonar to the nearest surface of the target [m]")
        ax2.set_ylabel("error [m]")
        ax2.legend(fontsize=7.5, loc="lower left")
        fig.tight_layout()
        fig.savefig(FIG / "range_true_vs_sonar.png", dpi=140)
        plt.close(fig)

    # ---------------------------------------------------------------- field of view
    fov = {}
    panels = [("k2", 3.0, 120.0), ("k2", 6.0, 120.0), ("k2_oa30", 4.0, 30.0)]
    panels = [(s_, R, oa) for s_, R, oa in panels if s_ in sess]
    if panels:
        fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 3.4), sharey=True)
        axes = np.atleast_1d(axes)
        for ax, (s_, R, oa) in zip(axes, panels):
            rows_s = sess[s_]
            for k, plane in enumerate(("horizontal", "vertical")):
                pts = []
                sel = [r for r in rows_s if r["case"].get("R") == R and r["case"].get("plane") == plane]
                for _c, rs in group(sel, "fov").items():
                    pts.append((rs[0]["case"]["angle_deg"],
                                float(np.mean([first_return(r["sonar"], THR_CLEAN) is not None for r in rs]))))
                if not pts:
                    continue
                pts.sort()
                a, pr = np.array(pts).T
                ax.plot(a, pr, marker="o", ms=3.5, lw=1.2, color=SERIES[k], label=f"{plane} offset")
                seen = a[pr > 0.5]
                fov[f"OA{oa:.0f}_R{R:.0f}_{plane}_max_detect_deg"] = float(seen.max()) if seen.size else None
            ax.axvline(oa / 2, color=STATUS["warning"], lw=1, ls="--", label=f"configured half-angle {oa / 2:.0f} deg")
            ax.set_title(f"OpeningAngle {oa:.0f} deg, target at {R:.0f} m", fontsize=9)
            ax.set_xlabel("off-axis angle of the target centre [deg]")
            ax.set_xlim(-2, 82)
        axes[0].set_ylabel("detection rate")
        for ax in axes:
            ax.legend(fontsize=7, loc="lower left")
        fig.suptitle("Field of view: detection of a BlueROV2 vs off-axis angle (noise off)", fontsize=10)
        fig.tight_layout()
        fig.savefig(FIG / "fov.png", dpi=140)
        plt.close(fig)
    summary["fov"] = fov

    # ---------------------------------------------------------------- timing
    timing = {}
    for s_, evs in events.items():
        for e in evs:
            if e.get("event") == "timing":
                timing[f"{s_}:{e['label']}"] = {k: e[k] for k in ("mean_tick_ms", "p95_tick_ms", "real_time_factor")}
    summary["timing"] = timing

    # ---------------------------------------------------------------- echogram of the distance sweep
    if k2:
        rs = [r for r in k2 if r["exp"] == "drone" and r["case"]["aspect"] == "head_on"]
        S = np.array([r["sonar"] for r in rs]).T
        truth = [hull_near_distance(r["obs_pose"], r["tgt_pose"]) for r in rs]
        fig, ax = plt.subplots(figsize=(8.4, 3.4))
        ax.imshow(S, aspect="auto", origin="lower", cmap="Greys", vmin=0, vmax=max(1e-3, float(S.max())),
                  extent=[0, S.shape[1], RANGES[0] - BIN_W / 2, RANGES[-1] + BIN_W / 2])
        ax.scatter(np.arange(len(truth))[::6] + 3.0, np.asarray(truth)[::6] + 0.45, marker="v", s=22,
                   color=SERIES[1], label="true near-face distance (marker drawn 0.45 m above)")
        ax.set_xlabel("capture index (target moved away step by step, 6 captures per position)")
        ax.set_ylabel("range [m]")
        ax.set_title("Raw SinglebeamSonar output (intensity per 5 cm range bin) - BlueROV2 head-on sweep", fontsize=9)
        ax.legend(fontsize=7.5, loc="upper left")
        fig.tight_layout()
        fig.savefig(FIG / "echogram_drone_sweep.png", dpi=140)
        plt.close(fig)

    # ---------------------------------------------------------------- k1 / k2 / k3 same-pose comparison
    def pick(rows, exp, **case):
        for r in rows:
            if r["exp"] == exp and all(r["case"].get(k) == v for k, v in case.items()):
                return r
        return None

    comp = [("gate", {"aim": "left_pillar", "d_plane": 3.0}, "G06 left pillar, 3 m"),
            ("box", {"d_face": 3.0}, "steel box 0.5 m, 3 m")]
    if sess:
        fig, axes = plt.subplots(1, 2, figsize=(10, 3.2), sharey=True)
        names = {"k1": "k1: sonar created before the props (v1 order)",
                 "k2": "k2: props spawned before the sonar, fresh cache",
                 "k3": "k3: cache from k2, props NOT spawned (ghost)"}
        for ax, (exp, case, title) in zip(axes, comp):
            styles = {"k1": dict(lw=1.6, ls="-"), "k2": dict(lw=4.0, ls="-", alpha=0.45), "k3": dict(lw=1.3, ls="--")}
            for k, s in enumerate(("k1", "k2", "k3")):
                r = pick(sess.get(s, []), exp, **case)
                if r is None:
                    continue
                ax.plot(RANGES, r["sonar"] + 0.0, color=SERIES[k], label=names[s], **styles[s])
            ax.set_title(title, fontsize=9)
            ax.set_xlabel("range [m]")
            ax.set_xlim(0, 8)
            ax.set_ylim(-0.05, 1.25)
        axes[0].set_ylabel("echo intensity")
        axes[1].legend(fontsize=7.2, loc="upper right")
        axes[0].annotate("k1: flat zero (props invisible)", xy=(5.2, 0.02), xytext=(4.6, 0.25), fontsize=7.5,
                         color=INK["secondary"], arrowprops=dict(arrowstyle="->", color=INK["muted"], lw=0.8))
        fig.suptitle("Runtime-spawned props and the HoloOcean sonar octree cache (same pose, three sessions)", fontsize=10)
        fig.tight_layout()
        fig.savefig(FIG / "octree_props_k1_k2_k3.png", dpi=140)
        plt.close(fig)

    # ---------------------------------------------------------------- noise, dropout, false alarms
    noise = {}
    nrows = [r for r in k2 if r["exp"] in ("noise", "noise_empty")]
    if nrows:
        empty = np.array([r["sonar"] for r in nrows if r["exp"] == "noise_empty"])
        thrs = np.round(np.arange(0.05, 0.81, 0.05), 2)
        fa = [float(np.mean([(p > t).any() for p in empty])) for t in thrs]
        noise["floor_p99"] = float(np.percentile(empty, 99))
        noise["false_alarm_per_capture"] = dict(zip(map(str, thrs), fa))
        det_by_d = {}
        for _c, rs in group(nrows, "noise").items():
            d = rs[0]["case"]["d_centre_from_sonar"]
            truth = hull_near_distance(rs[0]["obs_pose"], rs[0]["tgt_pose"])
            per = {}
            for t in thrs:
                est = [first_return(r["sonar"], t) for r in rs]
                ok = [e for e in est if e is not None]
                err = np.asarray(ok) - truth if ok else np.array([])
                per[str(t)] = {"p_detect": len(ok) / len(rs), "err_mean": float(err.mean()) if ok else None,
                               "err_p05": float(np.percentile(err, 5)) if ok else None,
                               "err_p95": float(np.percentile(err, 95)) if ok else None}
            det_by_d[str(d)] = {"truth": truth, "by_threshold": per}
        noise["detection"] = det_by_d
        # chosen threshold: smallest with zero false alarms in empty water
        ok_thr = [t for t, f in zip(thrs, fa) if f == 0.0]
        noise["threshold_zero_false_alarm"] = float(min(ok_thr)) if ok_thr else None
        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 3.3))
        ax.plot(thrs, fa, marker="o", ms=3.5, color=STATUS["critical"], label="false alarm (empty water)")
        for k, (d, v) in enumerate(sorted(det_by_d.items(), key=lambda kv: float(kv[0]))):
            ax.plot(thrs, [v["by_threshold"][str(t)]["p_detect"] for t in thrs], marker="o", ms=3, lw=1.1,
                    color=SERIES[k], label=f"detection, drone at {float(d):.0f} m")
        ax.set_xlabel("detection threshold on echo intensity")
        ax.set_ylabel("probability per capture")
        ax.set_title("Noise ON (AddSigma 0.05, MultSigma 0.1, RangeSigma 0.05)", fontsize=9)
        ax.legend(fontsize=7, loc="center right")
        t_sel = noise["threshold_zero_false_alarm"] or 0.3
        xs, ms, lo, hi = [], [], [], []
        for d, v in sorted(det_by_d.items(), key=lambda kv: float(kv[0])):
            x = v["by_threshold"][str(t_sel)]
            if x["err_mean"] is None:
                continue
            xs.append(v["truth"])
            ms.append(x["err_mean"])
            lo.append(max(0.0, x["err_mean"] - x["err_p05"]))
            hi.append(max(0.0, x["err_p95"] - x["err_mean"]))
        ax2.errorbar(xs, ms, yerr=[lo, hi], fmt="o", ms=5, color=SERIES[0], ecolor=SERIES[0], capsize=3,
                     label=f"mean and 5-95 % (40 captures each), threshold {t_sel:.2f}")
        ax2.axhline(0, color=INK["muted"], lw=1)
        ax2.axhspan(-BIN_W, BIN_W, color=INK["grid"], alpha=0.8, lw=0, label="+/- one range bin")
        ax2.set_xlabel("true near-face distance [m]")
        ax2.set_ylabel("range error [m] (+ = looks farther)")
        ax2.set_ylim(-0.15, 0.2)
        ax2.legend(fontsize=7, loc="upper left")
        fig.tight_layout()
        fig.savefig(FIG / "noise_dropout.png", dpi=140)
        plt.close(fig)
    summary["noise"] = noise

    # ---------------------------------------------------------------- surface / seabed
    env_cases = {}
    for exp in ("surface_horizontal", "surface_horizontal_target", "vertical_look", "seabed_vertical",
                "seabed_horizontal", "seabed_horizontal_target"):
        for _c, rs in group(k2, exp).items():
            est = [first_return(r["sonar"], THR_CLEAN) for r in rs]
            det = [e for e in est if e is not None]
            bs = rs[0]["obs_pose"][:3, 0]
            env_cases[exp + ":" + json.dumps(rs[0]["case"], sort_keys=True)] = {
                "detected": f"{len(det)}/{len(rs)}", "first_return_mean": round(float(np.mean(det)), 3) if det else None,
                "boresight_z": round(float(bs[2]), 3), "altimeter_gt": round(float(rs[0]["altimeter"]), 2),
                "depth_gt": round(float(-rs[0]["obs_pose"][2, 3]), 2)}
    summary["surface_seabed"] = env_cases

    # ---------------------------------------------------------------- screenshots
    shots = [("k2", "gate_left_pillar_3m", "k2: G06 left pillar at 3 m (cone drawn in green)"),
             ("k2", "drone_head_on_3m", "k2: BlueROV2 head-on at 3 m"),
             ("k2", "box_3m", "k2: runtime box at 3 m"),
             ("k1", "gate_left_pillar_3m", "k1: same gate pose, sonar created before the props"),
             ("k2", "surface_target", "k2: 0.6 m below the surface, drone at 4 m"),
             ("k2", "seabed_horizontal", "k2: 1 m above the seabed, horizontal beam")]
    have = [(s, n, t) for s, n, t in shots if (OUT_RAW / s / f"{n}.png").exists()]
    if have:
        import matplotlib.image as mpimg

        cols = 3
        rows_n = math.ceil(len(have) / cols)
        fig, axes = plt.subplots(rows_n, cols, figsize=(13, 2.6 * rows_n + 0.4))
        axes = np.atleast_1d(axes).ravel()
        for ax, (s, n, t) in zip(axes, have):
            img = mpimg.imread(OUT_RAW / s / f"{n}.png")
            ax.imshow(img[:, :, :3])
            ax.set_title(t, fontsize=8.5)
            ax.axis("off")
        for ax in axes[len(have):]:
            ax.axis("off")
        fig.suptitle("Probe scenes (HoloOcean RGB camera behind the observer, visualisation only)", fontsize=10)
        fig.tight_layout()
        fig.savefig(FIG / "probe_screenshots.png", dpi=110)
        plt.close(fig)

    (RES / "summary.json").write_text(json.dumps(summary, indent=1, default=float), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("visibility", "fov", "surface_seabed")}, indent=1, default=float)[:6000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
