"""Gate G06 before / after widening: front and top views to scale, with the BlueROV2 hull and the lateral margin.

The bars come from the same geometry the perception map and the simulator spawner use (holo_fleet/arena_bridge.py);
the hull is sonar_geometry.HULL_HALF; the lateral errors are the dead-reckoning errors (true - estimated, gate frame)
of the drones that held before the critical region in results/robustness/raw_gate_check (MEDIUM), when that analysis
file is given (figures/gate/gate_check_holds.json).  Usage: make_gate_width_figure.py OUT.png [holds.json]
"""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.perception.sonar_geometry import HULL_HALF
from holo_fleet.perception.structure_aiding import HULL_LAT_M

INK, BAR, OPEN, HULL, OK, BAD, CR = "#222222", "#4a4a4a", "#f4f6f8", "#2a6fdb", "#2e9e5b", "#d1495b", "#8a8a8a"


def _front(ax, g, err, title):
    W = g.inner_width
    margin = W / 2.0 - HULL_LAT_M
    ax.add_patch(Rectangle((-W / 2, -g.inner_height / 2), W, g.inner_height, color=OPEN, zorder=0))
    for b in g.bars:
        s, l, dz = g.to_gate_frame(b.center)
        ax.add_patch(Rectangle((l - b.half[1], dz - b.half[2]), 2 * b.half[1], 2 * b.half[2], color=BAR, zorder=2))
    cw = DEFAULT.gate.cr_half_width
    for x in (-cw, cw):
        ax.axvline(x, color=CR, ls="--", lw=1.0, zorder=1)
    ax.text(cw, 1.13, "CR edge", color=CR, fontsize=8, ha="center")
    ax.add_patch(Rectangle((-margin, -0.06), 2 * margin, 0.12, color=OK, alpha=0.25, zorder=1))
    ax.annotate("", xy=(margin, 0.0), xytext=(-margin, 0.0), arrowprops=dict(arrowstyle="<->", color=OK, lw=1.4))
    ax.text(0.0, -0.17, f"hull-centre corridor ±{margin:.2f} m", color=OK, fontsize=9, ha="center", weight="bold")
    hy, hz = HULL_HALF[1], HULL_HALF[2]
    ax.add_patch(Rectangle((-hy, -0.45 - hz), 2 * hy, 2 * hz, fc=HULL, ec=HULL, alpha=0.85, zorder=3))
    ax.text(0.0, -0.45, "BlueROV2\n0.58 m", color="white", fontsize=7.5, ha="center", va="center", zorder=4)
    touch = err + HULL_LAT_M > W / 2.0
    c = BAD if touch else OK
    ax.add_patch(Rectangle((err - hy, 0.30 - hz), 2 * hy, 2 * hz, fc="none", ec=c, lw=2.0, ls="-", zorder=3))
    ax.text(err + hy, 0.30 + hz + 0.15, f"true pose, {err:.2f} m lateral error:", color=c, fontsize=8, ha="right")
    ax.text(err + hy, 0.30 + hz + 0.04, "hull touches the bar" if touch else f"{W / 2 - err - HULL_LAT_M:.2f} m to the bar",
            color=c, fontsize=8, ha="right", weight="bold")
    ax.set_title(title, fontsize=11, color=INK, loc="left")
    ax.set_xlim(-1.45, 1.45)
    ax.set_ylim(-1.05, 1.25)
    ax.set_aspect("equal")
    ax.set_xlabel("lateral l [m]  (front view, looking along the passage)")
    ax.set_ylabel("dz [m]")


def _top(ax, g, errs, title):
    W = g.inner_width
    margin = W / 2.0 - HULL_LAT_M
    L, cw = DEFAULT.gate.cr_half_len, DEFAULT.gate.cr_half_width
    ax.add_patch(Rectangle((-L, -cw), 2 * L, 2 * cw, fc="none", ec=CR, ls="--", lw=1.0, zorder=1))
    ax.text(-L + 0.03, cw + 0.05, "critical region", color=CR, fontsize=8)
    ax.add_patch(Rectangle((-2.2, -margin), 4.4, 2 * margin, color=OK, alpha=0.15, zorder=0))
    for b in g.bars:
        s, l, dz = g.to_gate_frame(b.center)
        if abs(l) > 0.1:
            ax.add_patch(Rectangle((s - b.half[0], l - b.half[1]), 2 * b.half[0], 2 * b.half[1], color=BAR, zorder=2))
    hx, hy = HULL_HALF[0], HULL_HALF[1]
    s_hold = -(L + hx + 0.15)
    ax.add_patch(Rectangle((s_hold - hx, -hy), 2 * hx, 2 * hy, fc=HULL, alpha=0.85, zorder=3))
    ax.text(s_hold, -hy - 0.22, "hold point\n(estimate on axis)", color=HULL, fontsize=7.5, ha="center")
    for e in errs:
        c = BAD if abs(e) + HULL_LAT_M > W / 2.0 else OK
        ax.plot([s_hold - 0.55], [e], marker="o", ms=5, color=c, zorder=4)
    ax.text(s_hold - 0.55, 0.62, "true l of the\nheld drones", fontsize=7.5, ha="center", color=INK)
    n_bad = sum(abs(e) + HULL_LAT_M > W / 2.0 for e in errs)
    ax.text(-2.15, -1.32, f"{n_bad}/{len(errs)} held drones would touch", fontsize=8.5, color=BAD if n_bad else OK,
            weight="bold")
    ax.annotate("", xy=(2.1, 0.0), xytext=(-2.1, 0.0), arrowprops=dict(arrowstyle="->", color=INK, lw=0.8))
    ax.set_title(title, fontsize=11, color=INK, loc="left")
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-1.45, 1.45)
    ax.set_aspect("equal")
    ax.set_xlabel("along the passage s [m]  (top view)")
    ax.set_ylabel("lateral l [m]")


def main(out, holds=None):
    old = load_arena_gates(HORSESHOE_TRACK, ["G06"], inner_size={})[0]
    new = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    errs = []
    if holds:
        errs = [r["nav_err_l"] for r in json.load(open(holds)) if r["hold_s"] >= 10.0]
    worst = max(abs(e) for e in errs) if errs else 0.58
    fig, axs = plt.subplots(2, 2, figsize=(13, 10.5))
    for col, (g, tag) in enumerate(((old, "BEFORE"), (new, "AFTER"))):
        m = g.inner_width / 2.0 - HULL_LAT_M
        _front(axs[0, col], g, worst, f"{tag}: opening {g.inner_width:.2f} x {g.inner_height:.2f} m, margin ±{m:.2f} m")
        _top(axs[1, col], g, errs, f"{tag}: top view through the gate")
    fig.suptitle("Gate G06 widened from 1.5 m to 2.0 m (height unchanged): hull-centre lateral margin "
                 f"{old.inner_width / 2 - HULL_LAT_M:.2f} -> {new.inner_width / 2 - HULL_LAT_M:.2f} m",
                 fontsize=12.5, color=INK)
    fig.text(0.5, 0.01, f"Hull half-width with 5 deg yaw allowance {HULL_LAT_M:.3f} m; bars 0.18 m; CR |s| <= "
             f"{DEFAULT.gate.cr_half_len} m, |l| <= {DEFAULT.gate.cr_half_width} m (unchanged).  Dots: lateral DR error "
             "of the drones that held >= 10 s before the CR in the MEDIUM gate check (cf43452).",
             ha="center", fontsize=8.5, color=INK)
    fig.tight_layout(rect=(0, 0.025, 1, 0.97))
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
