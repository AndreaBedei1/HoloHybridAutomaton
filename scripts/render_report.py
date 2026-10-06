"""Render the visual artifacts of a run: GIFs of the HoloOcean scene, a sensor contact sheet and a
per-run Markdown report embedding the analysis figures and the referee verdicts.

Usage::

    python scripts/render_report.py --run results/gate_arena_s0 [--copy-figures]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.analysis import load_run, make_all_figures  # noqa: E402


def _overlay_text(img, lines):
    import cv2

    out = img.copy()
    y = 22
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 20
    return out


def make_gif(run_dir: Path, key: str, rd, every: int = 3, width: int = 400, fps: int = 6):
    import cv2
    import imageio.v2 as imageio

    frames = sorted((run_dir / "frames").glob(f"{key}_*.*"))
    if not frames:
        return None
    states = [{round(s["t"], 1): s["mode"] for s in rd.states[k]} for k in range(rd.n)]
    imgs = []
    for f in frames[::every]:
        img = cv2.imread(str(f))
        if img is None:
            continue
        t = int(f.stem.split("_")[-1]) / 10.0
        modes = []
        for k in range(rd.n):
            m = states[k].get(round(t - 0.1, 1)) or states[k].get(round(t, 1)) or ""
            modes.append(f"d{k}:{m.replace('FORMATION_', 'F_').replace('SEPARATION_', 'SEP_').replace('COLLISION_', 'COLL_')[:16]}")
        img = _overlay_text(img, [f"{key}  t={t:5.1f}s"] + modes)
        h = int(img.shape[0] * width / img.shape[1])
        img = cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA)
        imgs.append(img[:, :, ::-1])
    if not imgs:
        return None
    out = run_dir / f"{key.lower()}.gif"
    imageio.mimsave(out, imgs, duration=1.0 / fps, loop=0)
    return out


def sensor_sheet(run_dir: Path, rd):
    """Front camera + forward-looking sonar of each drone at three moments of the run."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt

    sens = run_dir / "sensors"
    times = sorted({int(p.stem.split("_")[-1]) for p in sens.glob("drone_0_FrontCamera_*.png")})
    if not times:
        return None
    picks = [times[0], times[len(times) // 2], times[-1]] if len(times) >= 3 else times
    fig, axes = plt.subplots(rd.n, 2 * len(picks), figsize=(3.0 * len(picks) * 2 * 0.62, 2.1 * rd.n))
    axes = np.atleast_2d(axes)
    for k in range(rd.n):
        for j, t in enumerate(picks):
            for i, kind in enumerate(("FrontCamera", "FrontSonar")):
                ax = axes[k, 2 * j + i]
                p = sens / f"drone_{k}_{kind}_{t:04d}.png"
                if p.exists():
                    ax.imshow(mpimg.imread(p))
                else:
                    ax.text(0.5, 0.5, "n/a\n(sensor disabled)", ha="center", va="center", fontsize=8)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_title(f"drone_{k} {kind} t={t}s", fontsize=7)
    fig.suptitle("Onboard sensor samples (forward-looking sonar: range up, azimuth left-right)", fontsize=9)
    fig.tight_layout()
    out = run_dir / "figures" / "sensor_samples.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def scene_snapshots(run_dir: Path, rd, n: int = 4):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt

    shots = []
    for key in ("ChaseCamera", "SideCamera"):
        frames = sorted((run_dir / "frames").glob(f"{key}_*.*"))
        if frames:
            idx = np.linspace(0, len(frames) - 1, n).astype(int)
            shots.append((key, [frames[i] for i in idx]))
    if not shots:
        return None
    fig, axes = plt.subplots(len(shots), n, figsize=(2.6 * n, 2.0 * len(shots) + 0.4))
    axes = np.atleast_2d(axes)
    for r, (key, fs) in enumerate(shots):
        for c, f in enumerate(fs):
            ax = axes[r, c]
            img = mpimg.imread(f)
            ax.imshow(img)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_title(f"{key} t={int(f.stem.split('_')[-1]) / 10:.0f}s", fontsize=7)
    fig.suptitle(f"HoloOcean scene - {rd.config['spec_name']} (visualisation cameras, never used by controllers)", fontsize=9)
    fig.tight_layout()
    out = run_dir / "figures" / "scene_snapshots.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def write_markdown(run_dir: Path, rd, figs, gifs):
    m = rd.metrics
    p1, p2, p3, env = m["P1_separation"], m["P2_mutual_exclusion"], m["P3_formation_recovery"], m["envelope"]
    stats = json.loads((run_dir / "perception_stats.json").read_text()) if (run_dir / "perception_stats.json").exists() else {}
    decisions = {}
    for e in rd.events:
        if e.get("type") == "decision":
            decisions[e["decision"]] = decisions.get(e["decision"], 0) + 1
    env_events = [e for e in rd.events if e.get("type") in ("ENVELOPE_VIOLATION", "ENVELOPE_RESTORED")]
    lines = [f"# Run `{rd.name}`", "", rd.config["description"], "",
             f"- status: {m['run']['status']}, simulated {m['run']['sim_time_s']} s in {m['run']['wall_time_s']} s wall time",
             f"- drones: {rd.n}, inter-agent communication: {'ON' if rd.config['comms_enabled'] else 'OFF'}",
             f"- determinism monitor violations: {m['run']['determinism_monitor_violations']}", "",
             "| property | verdict (referee, ground truth) | key numbers |", "|---|---|---|",
             f"| P1 separation | {'HOLDS' if p1['holds'] else 'VIOLATED'} | min distance {p1['min_distance_overall']} m "
             f"(d_safe {p1['d_safe']} m), violations {p1['violations_d_lt_d_safe']} |",
             f"| P2 mutual exclusion | {('HOLDS' if p2['holds'] else 'VIOLATED') if p2['holds'] is not None else 'n/a (no gate)'} | "
             f"max occupancy {p2['max_occupancy']}, entry order {p2['entry_order']} |",
             f"| P3 formation recovery | {('HOLDS' if p3['holds'] else 'VIOLATED') if p3['holds'] is not None else 'n/a'} | "
             f"{p3['n_episodes']} episode(s), max recovery after perturbation {p3['max_recovery_time_s']} s "
             f"(deadline {p3['T_s']} s), final error {p3['final_form_err']} m |",
             f"| operational envelope | {'inside' if env['inside_envelope'] else 'EXCEEDED (flagged)'} | "
             f"max effective drift {env['max_effective_drift_applied']} m/s vs claimed {env['current_drift_max_claimed']} m/s |",
             "", f"Gate/formation decisions logged by the automata: `{json.dumps(decisions)}`", ""]
    if env_events:
        lines += ["Envelope declarations by the drones themselves: " +
                  ", ".join(f"{e['drone']} {e['type']} t={e['t']:.1f}s" for e in env_events[:12]), ""]
    if stats:
        cov = [v for k, v in stats.items() if k.startswith("coverage")]
        lines += [f"Perception vs ground truth: {stats.get('n_samples')} neighbour estimates, coverage "
                  f"{cov[0] * 100:.1f}% within 8 m, |error| p99 per axis {stats.get('abs_err_p99_xyz')} m, "
                  f"max {stats.get('abs_err_max_xyz')} m.", ""]
    for name, p in figs.items():
        lines += [f"![{name}](figures/{Path(p).name})", ""]
    for g in gifs:
        lines += [f"![{g.stem}]({g.name})", ""]
    (run_dir / "RUN_REPORT.md").write_text("\n".join(lines))
    return run_dir / "RUN_REPORT.md"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--copy-figures", action="store_true")
    args = ap.parse_args(argv)
    run_dir = Path(args.run)
    if not run_dir.is_dir():
        run_dir = ROOT / "results" / args.run
    figs = make_all_figures(run_dir, copy_to=(ROOT / "figures") if args.copy_figures else None)
    rd = load_run(run_dir)
    extra = [sensor_sheet(run_dir, rd), scene_snapshots(run_dir, rd)]
    for p in extra:
        if p is not None:
            figs[p.stem] = str(p)
            if args.copy_figures:
                import shutil

                shutil.copy(p, ROOT / "figures" / f"{rd.name}__{p.stem}.png")
    gifs = [g for g in (make_gif(run_dir, "ChaseCamera", rd), make_gif(run_dir, "SideCamera", rd)) if g is not None]
    if args.copy_figures:
        import shutil

        for g in gifs:
            shutil.copy(g, ROOT / "figures" / f"{rd.name}__{g.name}")
    md = write_markdown(run_dir, rd, figs, gifs)
    print(f"report: {md}")
    for g in gifs:
        print(f"gif: {g} ({g.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
