"""Sensor layout figure: mounting, orientation and field of view of every onboard sensor, drawn from the
actual sensor configuration (holo_fleet.perception.sensor_suite) -> figures/sensor_layout.png"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.analysis import DRONE_COLORS, INK, STATUS, _style  # noqa: E402
from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception.sensor_suite import FLS_GEOMETRY  # noqa: E402

HULL = (0.58, 0.46, 0.25)   # BlueROV2-like length, width, height [m]


def main() -> int:
    plt = _style()
    from matplotlib.patches import Rectangle, Wedge

    pc, sep = DEFAULT.perc, DEFAULT.sep
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.2))
    # ---------------- top view
    ax = axes[0]
    R = pc.ring_range_m
    for r, name, col in ((sep.d_warning, f"d_warning {sep.d_warning} m", STATUS["warning"]),
                         (sep.d_ca, f"d_ca {sep.d_ca} m", STATUS["serious"]),
                         (sep.d_safe, f"d_safe {sep.d_safe} m", STATUS["critical"])):
        ax.add_patch(plt.Circle((0, 0), r, fill=False, ls="--", lw=1.0, ec=col))
        a = math.radians(135)
        ax.annotate(name, (r * math.cos(a), r * math.sin(a)), xytext=(-3, 3), textcoords="offset points",
                    fontsize=7, color=INK["secondary"], ha="right", va="bottom",
                    bbox=dict(boxstyle="round,pad=0.15", fc=INK["surface"], ec="none"))
    for k in range(pc.ring_beams):
        a = -math.radians(k * 360.0 / pc.ring_beams)
        ax.plot([0, 4.0 * math.cos(a)], [0, 4.0 * math.sin(a)], color=INK["axis"], lw=0.4, zorder=0)
    ax.annotate(f"proximity sonar: {pc.ring_beams} beams x {len(pc.ring_elevations_deg)} rings, 5 deg az,\n"
                f"range {R:.0f} m (rays drawn to 4 m)", (0, 4.25), ha="center", fontsize=7.5, color=INK["secondary"])
    az = FLS_GEOMETRY["azimuth_deg"]
    ax.add_patch(Wedge((0.25, 0), 3.4, -az / 2, az / 2, width=None, fc=DRONE_COLORS[0], alpha=0.18, ec=DRONE_COLORS[0], lw=1.2))
    ax.annotate("forward-looking imaging sonar\n90 deg x 20 deg, 0.5-12 m", (2.3, 1.35), fontsize=7.5, color=INK["primary"])
    ax.add_patch(Wedge((0.29, 0), 2.2, -45, 45, fc="none", ec=DRONE_COLORS[1], lw=1.4, ls="-"))
    ax.annotate("front camera 90 deg HFOV", (1.0, -1.6), fontsize=7.5, color=INK["primary"])
    ax.add_patch(Rectangle((-HULL[0] / 2, -HULL[1] / 2), HULL[0], HULL[1], fc=INK["primary"], ec="none", zorder=5))
    ax.annotate("BlueROV2 hull (x forward, y left)", (-0.2, -0.2), xytext=(-4.3, -3.9), fontsize=7.5,
                color=INK["primary"], arrowprops=dict(arrowstyle="-", color=INK["secondary"], lw=0.8),
                bbox=dict(boxstyle="round,pad=0.15", fc=INK["surface"], ec="none"))
    ax.arrow(0, 0, 0.6, 0, width=0.02, color=INK["surface"], zorder=6)
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-4.5, 4.8)
    ax.set_aspect("equal")
    ax.set_title("Top view: horizontal coverage and separation thresholds")
    ax.set_xlabel("body x [m]")
    ax.set_ylabel("body y [m]")
    # ---------------- side view
    ax = axes[1]
    for e in pc.ring_elevations_deg:
        a = math.radians(e)
        ax.plot([0, 4.0 * math.cos(a)], [0, 4.0 * math.sin(a)], color=INK["axis"], lw=0.5, zorder=0)
        ax.plot([0, -4.0 * math.cos(a)], [0, 4.0 * math.sin(a)], color=INK["axis"], lw=0.5, zorder=0)
    ax.annotate("proximity-sonar rings every 5 deg, -60...+60 deg elevation (front and rear)", (0, 3.75), ha="center",
                fontsize=7.5, color=INK["secondary"])
    ax.add_patch(Wedge((0.25, 0), 3.4, -10, 10, fc=DRONE_COLORS[0], alpha=0.18, ec=DRONE_COLORS[0], lw=1.2))
    ax.annotate("imaging sonar 20 deg elevation", (2.0, 0.75), fontsize=7.5)
    for s in (-1, 1):
        a = math.radians(-90 + s * 22.5)
        ax.plot([0, 1.4 * math.cos(a)], [-0.12, -0.12 + 1.4 * math.sin(a)], color=DRONE_COLORS[2], lw=1.4)
    ax.annotate("DVL (4 beams, 22.5 deg)", (0.35, -1.35), fontsize=7.5)
    ax.plot([0, 0], [0.12, 2.6], color=DRONE_COLORS[2], lw=1.0, ls=":")
    ax.plot([0, 0], [-0.12, -2.6], color=DRONE_COLORS[2], lw=1.0, ls=":")
    ax.annotate("altimeters up / down (single beam)", (0.1, 2.3), fontsize=7.5)
    ax.annotate("depth (pressure)", (-1.6, -0.45), fontsize=7.5)
    ax.add_patch(Rectangle((-HULL[0] / 2, -HULL[2] / 2), HULL[0], HULL[2], fc=INK["primary"], ec="none", zorder=5))
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-3.0, 4.1)
    ax.set_aspect("equal")
    ax.set_title("Side view: vertical coverage (3D escape needs it)")
    ax.set_xlabel("body x [m]")
    ax.set_ylabel("body z [m] (up)")
    fig.suptitle("Onboard sensor suite of every drone (identical on all drones; drawn from the configuration)", fontsize=10)
    fig.tight_layout()
    out = ROOT / "figures" / "sensor_layout.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=150)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
