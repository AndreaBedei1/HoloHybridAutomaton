"""Re-render the fleet view of a finished run from its logs (no simulator needed) and make its GIF.

    python scripts/render_demo.py results/v2/demos/lost_drone_rejoin
    python scripts/render_demo.py results/v2/demos/gate_single --every 0.5 --fps 6
    python scripts/render_demo.py results/v2/demos/lost_drone_timeout --at 30 50     # single frames (PNG)

States, missing / vacant slots and queue decisions come from drone_<k>_state.jsonl and events.jsonl (what
the controllers logged), positions and the P1 / P2 / P3 status from referee_timeseries.csv (ground
truth), so the frames are the ones the live window showed (holo_fleet/ui/fleet_view.py).
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
from holo_fleet.ui.fleet_view import FleetView  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402


def _f(row, k, default=0.0):
    v = row.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def referee_live(row, n, gates, state) -> dict:
    """The referee's live status rebuilt from one row of referee_timeseries.csv (``state`` keeps the run so far)."""
    dpair = [_f(row, f"d_{a}_{b}", 1e9) for a in range(n) for b in range(a + 1, n)]
    occ = {g.gate_id: int(_f(row, f"occ_{g.gate_id}")) for g in gates}
    state["max_occ"] = max([state.get("max_occ", 0)] + list(occ.values()))
    absent = [f"drone_{k}" for k in str(row.get("absent") or "").split("+") if k != ""]
    return {"d_min_true": min(dpair, default=None), "p1_ok": bool(int(_f(row, "p1_ok", 1))),
            "occupancy": occ, "p2_ok": state["max_occ"] <= 1,
            "form_err": _f(row, "form_err", None) if row.get("form_err") not in (None, "") else None,
            "formation": row.get("formation_state") or None, "absent": absent,
            "form_err_present": _f(row, "form_err_present", None) if row.get("form_err_present") not in (None, "") else None,
            "row": {k: v for k, v in row.items() if k[:2] in ("ix", "iy", "iz") and v not in (None, "")}}


def replay(run_dir: Path, every: float = 0.5, at=None):
    """Yield (t, image) of the fleet view along a logged run (every ``every`` s, or only at the times ``at``)."""
    cfg_run = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    sc = SCENARIOS[cfg_run["scenario"]](DEFAULT)
    n = cfg_run["n_drones"]
    names = [f"drone_{k}" for k in range(n)]
    view = FleetView(sc, DEFAULT, names)
    recs = [[json.loads(l) for l in open(run_dir / f"{nm}_state.jsonl", encoding="utf-8")] for nm in names]
    rows = list(csv.DictReader(open(run_dir / "referee_timeseries.csv", encoding="utf-8")))
    events = sorted((json.loads(l) for l in open(run_dir / "events.jsonl", encoding="utf-8")), key=lambda e: e.get("t", 0.0))
    events = [e for e in events if e.get("drone")]
    state, ei, last = {}, 0, -1e9
    targets = sorted(at) if at else None
    for i, row in enumerate(rows):
        t = _f(row, "t")
        P = np.array([[_f(row, f"x{k}"), _f(row, f"y{k}"), _f(row, f"z{k}")] for k in range(n)])
        ev = []
        while ei < len(events) and events[ei].get("t", 0.0) <= t + 1e-6:
            ev.append(events[ei])
            ei += 1
        view.step(t, [recs[k][min(i, len(recs[k]) - 1)] for k in range(n)], P, referee_live(row, n, sc.judged_gates, state),
                  ev)
        if targets is not None:
            if targets and t >= targets[0] - 1e-6:
                targets.pop(0)
                yield t, view.render()
            continue
        if t - last >= every - 1e-6:
            last = t
            yield t, view.render()


def render_run(run_dir: Path, every: float = 0.5, fps: float = 6.0, width: int = 1280) -> Path:
    out_dir = run_dir / "fleet_view"
    out_dir.mkdir(exist_ok=True)
    for f in out_dir.glob("*.jpg"):
        f.unlink()
    for t, img in replay(run_dir, every):
        cv2.imwrite(str(out_dir / f"fleet_{int(round(t * 10)):05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    cfg_run = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    return make_gif(out_dir.glob("fleet_*.jpg"), run_dir / f"{cfg_run['scenario']}.gif", width=width, fps=fps)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--every", type=float, default=0.5, help="simulated seconds between frames")
    ap.add_argument("--fps", type=float, default=6.0, help="GIF frames per second (6 fps at 0.5 s: 3x real time)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--at", type=float, nargs="*", help="only write PNG frames at these times")
    a = ap.parse_args()
    run_dir = Path(a.run_dir)
    if a.at:
        for t, img in replay(run_dir, at=a.at):
            p = run_dir / f"fleet_t{int(round(t)):03d}.png"
            cv2.imwrite(str(p), img)
            print(p)
        return 0
    print(render_run(run_dir, a.every, a.fps, a.width))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
