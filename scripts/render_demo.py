"""Re-render the dashboard of a finished run from its logs (no simulator needed) and make its GIF.

    python scripts/render_demo.py results/v2/demos/gate_single
    python scripts/render_demo.py results/v2/demos/gate_single --every 0.5 --fps 4

The left half is rebuilt from drone_<k>_state.jsonl (what the controllers logged), the right half
from referee_timeseries.csv (ground truth), so the frames are the ones the live window showed.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402
from holo_fleet.ui import dashboard  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402


def _f(row, k, default=0.0):
    v = row.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def render_run(run_dir: Path, every: float = 0.5, fps: float = 4.0, width: int = 960) -> Path:
    cfg_run = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    status = json.loads((run_dir / "run_status.json").read_text(encoding="utf-8")).get("status", "?")
    env_label = status
    if (run_dir / "referee_metrics.json").exists():
        mm = json.loads((run_dir / "referee_metrics.json").read_text(encoding="utf-8"))
        env_label = ("inside (whole run)" if mm["envelope"]["inside_envelope"] else "OUT (disturbance beyond the claimed envelope)")
        if status != "COMPLETE":
            env_label += f" - run {status}"
    sc = SCENARIOS[cfg_run["scenario"]](DEFAULT)
    names = [f"drone_{k}" for k in range(cfg_run["n_drones"])]
    meta0 = dashboard.scenario_meta(sc, DEFAULT, names)
    recs = [[json.loads(l) for l in open(run_dir / f"{nm}_state.jsonl", encoding="utf-8")] for nm in names]
    rows = list(csv.DictReader(open(run_dir / "referee_timeseries.csv", encoding="utf-8")))
    n = len(names)
    G = DEFAULT.gate
    gates = sc.judged_gates
    inside_prev = {g.gate_id: set() for g in gates}
    order = {g.gate_id: [] for g in gates}
    max_occ = {g.gate_id: 0 for g in gates}
    trails = [[] for _ in range(n)]
    out_dir = run_dir / "dashboard_replay"
    out_dir.mkdir(exist_ok=True)
    for f in out_dir.glob("*.jpg"):
        f.unlink()
    cams = sorted((run_dir / "frames").glob("chase_*.jpg"))
    cam_t = np.array([int(p.stem.split("_")[1]) / 10.0 for p in cams]) if cams else np.array([])
    last_t, last_trail = -1e9, -1e9
    p1_ok = True
    for i, row in enumerate(rows):
        t = _f(row, "t")
        P = np.array([[_f(row, f"x{k}"), _f(row, f"y{k}"), _f(row, f"z{k}")] for k in range(n)])
        for g in gates:
            inside = {names[k] for k in range(n) if (lambda q: abs(q[0]) <= G.cr_half_len and abs(q[1]) <= G.cr_half_width
                                                     and abs(q[2]) <= G.cr_half_height)(g.to_gate_frame(P[k]))}
            for nm in sorted(inside - inside_prev[g.gate_id]):
                if nm not in order[g.gate_id]:
                    order[g.gate_id].append(nm)
            max_occ[g.gate_id] = max(max_occ[g.gate_id], len(inside))
            inside_prev[g.gate_id] = inside
        dpair = [_f(row, f"d_{a}_{b}", 1e9) for a in range(n) for b in range(a + 1, n)]
        p1_ok = p1_ok and (min(dpair, default=1e9) >= DEFAULT.sep.d_safe)
        if t - last_trail >= 0.5:
            last_trail = t
            for k in range(n):
                trails[k].append(P[k][:2].copy())
        if t - last_t < every - 1e-6:
            continue
        last_t = t
        live = {"d_min_true": min(dpair, default=None), "p1_ok": bool(int(_f(row, "p1_ok", 1))) if "p1_ok" in row else p1_ok,
                "occupancy": {g.gate_id: int(_f(row, f"occ_{g.gate_id}")) for g in gates},
                "p2_ok": all(v <= 1 for v in max_occ.values()), "form_err": _f(row, "form_err", None) if row.get("form_err") else None,
                "formation": (row.get("formation_state") or None) if sc.formation_enabled else None,
                "episodes": int(_f(row, "episodes", 0))}
        if live["formation"] is None and sc.formation_enabled and row.get("form_err"):
            live["formation"] = "LOST" if _f(row, "form_err") > DEFAULT.ref.e_lost else "OK"
        rowf = {k: (_f(row, k) if (k[:2] in ("ix", "iy", "iz") or k == "d_intruder") and row.get(k) not in (None, "") else v)
                for k, v in row.items() if not ((k[:2] in ("ix", "iy", "iz") or k == "d_intruder") and row.get(k) in (None, ""))}
        ref = {"row": rowf, "live": live, "entry_order": {g: list(v) for g, v in order.items()}, "max_occ": dict(max_occ),
               "true_current": np.array([_f(row, "cur_x"), _f(row, "cur_y"), _f(row, "cur_z")]), "contacts": 0}
        rr = [recs[k][min(i, len(recs[k]) - 1)] for k in range(n)]
        cam = None
        if cams:
            j = int(np.argmin(np.abs(cam_t - t)))
            if abs(cam_t[j] - t) < 1.0:
                cam = cv2.imread(str(cams[j]))
        meta = dict(meta0, t=t, envelope=env_label)
        img = dashboard.render(meta, rr, ref, cam, trails)
        cv2.imwrite(str(out_dir / f"dash_{int(round(t * 10)):05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
    gif = make_gif(out_dir.glob("dash_*.jpg"), run_dir / f"{cfg_run['scenario']}.gif", width=width, fps=fps)
    return gif


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--every", type=float, default=0.5)
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--width", type=int, default=960)
    a = ap.parse_args()
    gif = render_run(Path(a.run_dir), a.every, a.fps, a.width)
    print(gif)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
