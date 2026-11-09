"""Ascent of a completely failed BlueROV2 in HoloOcean: the validation of config.FailureModel.

    python calibration/probe_buoyant_ascent.py [--out results/calibration/buoyant_ascent.json]

The engine's BlueROV2 is exactly neutral (engine source, BlueROV2.cpp: Volume = MassInKG / WaterDensity), so a failed
drone would hang in the water.  The deployed vehicles are ballasted positively buoyant; the simulator applies the ascent
as a vertical current on that drone only (holo_env.ascent_command).  This probe measures, with zero thrust and the real
gate geometry:

* free ascent: the terminal speed for vertical current commands around the calibrated value, against the target
  FailureModel.v_rise_mps, the attitude while it rises and the stop at the surface;
* under the top bar of G06: whether a drone that fails inside the critical region is stopped by the bar or passes it,
  as a function of its along-track position in the opening, i.e. what "the CR is physically clear" really depends on;
* with a horizontal current: the ascent is not disturbed by the drift of the current.

Ground truth only (it is a bench): nothing here reaches a controller.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.sim.currents import CurrentComponent, CurrentField  # noqa: E402
from holo_fleet.sim.holo_env import HoloFleetSim  # noqa: E402
from holo_fleet.sim.spec import bench_spec  # noqa: E402

DT = 0.1


def run_case(name, pos, yaw, cfg, seconds, gate_ids=(), current=None):
    spec = bench_spec(name, [pos], [yaw], gate_ids=gate_ids)
    spec.faults = ({"drone": "drone_0", "type": "TOTAL_FAILURE", "t_on": 1.0, "t_off": None},)
    spec.duration_s = seconds + 1.0
    if current is not None:
        spec.current = current
    sim = HoloFleetSim(spec, cfg, headless=True)
    sim.start()
    rows = []
    try:
        for _ in range(int(seconds / DT)):
            sim.step({"drone_0": np.zeros(8)}, DT)
            tr = sim.truth()
            R = tr.rotations[0]
            tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
            rows.append([round(sim.t, 2), *[round(float(x), 3) for x in tr.positions[0]],
                         round(float(tr.velocities[0][2]), 3), round(tilt, 1), int(tr.collision[0])])
    finally:
        sim.close()
    return rows


def terminal_speed(rows, t0, t1):
    a = np.array(rows, float)
    m = (a[:, 0] >= t0) & (a[:, 0] <= t1)
    return float(np.mean(a[m, 4])) if m.any() else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "results" / "calibration" / "buoyant_ascent.json"))
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    yaw = float(np.degrees(np.arctan2(g.axis[1], g.axis[0])))
    out = {"target_v_rise_mps": DEFAULT.failure.v_rise_mps, "surface_z_m": DEFAULT.failure.surface_z_m, "free": [], "gate": [],
           "current": []}
    p0 = g.from_gate_frame([-12.0, 8.0, -3.0]).tolist()    # open water, away from the gate
    for v in ([0.05, 0.15, 0.25] if not args.quick else [0.15]):
        cfg = dataclasses.replace(DEFAULT, failure=dataclasses.replace(DEFAULT.failure, v_rise_mps=v))
        rows = run_case(f"ascent_{v}", p0, yaw, cfg, 40.0 if v >= 0.1 else 45.0)
        a = np.array(rows, float)
        out["free"].append({"v_target": v, "z0": round(float(a[0, 3]), 3), "z_end": round(float(a[-1, 3]), 3),
                            "v_terminal_mean": round(terminal_speed(rows, 15.0, 30.0), 3),
                            "tilt_max_deg": float(a[:, 5].max())})
    # failure inside the opening at several along-track positions under the top bar: stopped by the bar or not
    for s in ([-0.6, -0.2, 0.0, 0.2, 0.5, 0.75] if not args.quick else [0.0, 0.5]):
        rows = run_case(f"ascent_gate_{s}", g.from_gate_frame([s, 0.0, 0.0]).tolist(), yaw, DEFAULT, 40.0, gate_ids=("G06",))
        a = np.array(rows, float)
        dz = a[:, 3] - float(g.center[2])
        idx = np.flatnonzero(dz >= 1.0)
        out["gate"].append({"s0": s, "dz0": round(float(dz[0]), 3), "dz_end": round(float(dz[-1]), 3),
                            "t_leaves_cr_top_s": round(float(a[idx[0], 0]), 1) if idx.size else None,
                            "s_end": round(float(g.to_gate_frame(a[-1, 1:4])[0]), 3), "contact_steps": int(a[:, 6].sum()),
                            "z_trace": a[::20, 3].round(2).tolist()})
    cur = CurrentField([CurrentComponent("uniform", drift=(0.15, 0.0, 0.0))])
    rows = run_case("ascent_current", p0, yaw, DEFAULT, 30.0, current=cur)
    a = np.array(rows, float)
    out["current"].append({"drift_x": 0.15, "v_terminal_mean": round(terminal_speed(rows, 15.0, 28.0), 3),
                           "dx_end": round(float(a[-1, 1] - a[0, 1]), 2)})
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
