"""Stage-2 entry admission (control/flows.py, perception/gate_perception.py): what the hold point can observe.

M4h (reported, numeric grid): from the hold point before the critical region (on the gate axis, at the deployed
s_hold, with an own lateral position error up to ``nav_err_gate_m``), which hull centres inside the CR produce, in
the FRONT sonar, an echo that the classifier cannot explain by the mapped bars (outside the structure window grown by
its tolerances) on a line of sight not blocked by a bar.  Such an occupant makes the stage-2 check hold the entry
(GateObs.entry_ok); an occupant whose hull echo falls inside the structure window is MASKED there.

This is a statement of observability, not a guarantee: the stage-2 sonar check protects P2 against an occupant in the
observable part (whatever the latch believed), the B2 veto (``heard_inside``) against an occupant that pings, and the
latch argument of formal M3 (sound witness, healthy crossing) covers the masked part of a silent occupant.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import CheckResult, Report  # noqa: E402
from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates  # noqa: E402
from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS  # noqa: E402
from holo_fleet.perception.sonar_processing import EchoClassifier  # noqa: E402

ENC = "numeric grid over hull centres in the CR (deployed classifier structure window, bars as occluders)"


def _segment_hits_box(a, b, bar) -> bool:
    d = b - a
    lo, hi = 0.0, 1.0
    p = bar.axes.T @ (a - bar.center)
    v = bar.axes.T @ d
    for k in range(3):
        if abs(v[k]) < 1e-12:
            if abs(p[k]) > bar.half[k]:
                return False
            continue
        t1, t2 = (-bar.half[k] - p[k]) / v[k], (bar.half[k] - p[k]) / v[k]
        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
        if lo > hi:
            return False
    return True


def observability(cfg: FleetConfig = DEFAULT, n_s: int = 17, n_l: int = 9, n_z: int = 9):
    G, pc = cfg.gate, cfg.perc
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    clf = EchoClassifier(gate_bars=list(g.bars), cfg=cfg)
    s_hold = -(G.cr_half_len + float(HULL_HALF[0]) + 0.15)            # = control/flows.py RELOC_MARGIN_M
    yaw = math.atan2(g.axis[1], g.axis[0])
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    half = math.radians(pc.sonar.half_angle_deg)
    hull = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
                     if (x, y, z) != (0, 0, 0)], float) * HULL_HALF
    out = []
    for e in (-cfg.acoustic.nav_err_gate_m, 0.0, cfg.acoustic.nav_err_gate_m):
        p = g.from_gate_frame([s_hold, e, 0.0])
        son, ax = p + R @ MOUNTS["FRONT"], R @ AXES["FRONT"]
        sw = clf.structure_window(son, ax)
        for s in np.linspace(-G.cr_half_len, G.cr_half_len, n_s):
            for l in np.linspace(-G.cr_half_width, G.cr_half_width, n_l):
                for dz in np.linspace(-G.cr_half_height, G.cr_half_height, n_z):
                    c = g.from_gate_frame([s, l, dz])
                    surf = c + hull @ R.T
                    d = surf - son
                    r = np.linalg.norm(d, axis=1)
                    inc = np.arccos(np.clip((d @ ax) / np.maximum(r, 1e-9), -1.0, 1.0)) <= half
                    seen = False
                    for x, rr in zip(surf[inc], r[inc]):
                        if any(_segment_hits_box(son, x, b) for b in g.bars):
                            continue
                        if sw is None or rr < sw[0] - pc.structure_tol_m or rr > sw[1] + pc.structure_tol_far_m:
                            seen = True
                            break
                    out.append((e, float(s), float(l), float(dz), seen))
    return np.array(out, float), s_hold


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    rep = Report("stage2")
    A, s_hold = observability(cfg)
    frac = float(A[:, 4].mean())
    masked = A[A[:, 4] < 0.5]
    s_mask = (round(float(masked[:, 1].min()), 2), round(float(masked[:, 1].max()), 2)) if len(masked) else None
    seen_axis = A[(np.abs(A[:, 2]) < 1e-9) & (np.abs(A[:, 3]) < 1e-9) & (A[:, 0] == 0.0)]
    s_seen_axis = [round(float(x), 2) for x in seen_axis[seen_axis[:, 4] > 0.5][:, 1]]
    rep.add(CheckResult("M4h stage-2 observability of the CR from the hold point (reported)",
                        "fraction of CR hull centres with an unexplained, unoccluded FRONT echo", ENC, "reported",
                        verdict="reported", passed=True,
                        note=f"hold point s = {s_hold:.2f}; observable fraction {frac:.2f} over {len(A)} samples "
                             f"(own lateral error -1/0/+1 m); masked centres at s in {s_mask}; on the axis observable "
                             f"at s = {s_seen_axis}"), verbose)
    return rep


if __name__ == "__main__":
    r = run()
    print("      ", r.results[-1].note)
    sys.exit(0 if r.ok else 1)
