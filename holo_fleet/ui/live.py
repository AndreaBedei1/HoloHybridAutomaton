"""Live demo hook for :func:`holo_fleet.runner.run`: fleet view window and frames, viewport drawing.

The hook only *reads*: controller records and events (onboard knowledge), the referee (ground truth
for the viewer) and draws the onboard belief in the HoloOcean viewport.  It never writes into a
controller.  The fleet view (holo_fleet/ui/fleet_view.py) is the same one scripts/render_demo.py
rebuilds from the logs.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

from holo_fleet.perception.sonar_geometry import AXES
from holo_fleet.ui.fleet_view import FleetView

# viewport colours (RGB 0-255) of the echo classes; the escape arrow is magenta
VP_CLS = {"STRUCTURE": [150, 150, 150], "SEABED": [140, 109, 70], "DYNAMIC": [230, 40, 40], "UNKNOWN": [250, 178, 25]}
VP_ESCAPE = [230, 60, 230]


class DemoUI:
    def __init__(self, scenario, cfg, names, show: bool = True, draw_viewport: bool = True,
                 window: str = "holo_fleet - fleet view"):
        self.view = FleetView(scenario, cfg, names)
        self.names = list(names)
        self.show = show
        self.draw_viewport = draw_viewport
        self.window = window
        self.frames_written = 0

    def __call__(self, sim, scenario, ctrls, referee, truth, t, out, save_frame, events=()) -> None:
        recs = [ctrls[nm].last_record for nm in self.names]
        live = dict(referee.live(), row=referee.rows[-1] if referee.rows else {})
        self.view.step(t, recs, truth.positions, live, list(events))
        if self.draw_viewport and not sim.headless:
            self._viewport(sim, recs, scenario)
        if not (self.show or save_frame):
            return
        img = self.view.render()
        if save_frame:
            d = Path(out) / "fleet_view"
            d.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(d / f"fleet_{int(round(t * 10)):05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.frames_written += 1
        if self.show:
            cv2.imshow(self.window, img)
            cv2.waitKey(1)

    def _viewport(self, sim, recs, scenario) -> None:
        """Onboard belief drawn at the drone's ESTIMATED pose: nearest echo per sector, escape arrow."""
        for rec in recs:
            p = np.asarray(rec.get("nav_p") or [0, 0, 0], float)
            yaw = math.radians(rec.get("nav_yaw_deg", 0.0))
            R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0, 0, 1.0]])
            for s, sec in (rec.get("sectors") or {}).items():
                ech = [e for e in sec.get("echoes", []) if e[1] in VP_CLS]
                if not ech:
                    continue
                r, cls = min(ech, key=lambda e: e[0])
                sim.draw_line(p, p + R @ AXES[s] * r, VP_CLS[cls], 6.0, 0.12)
            esc = rec.get("escape") or {}
            v = np.asarray(rec.get("v_cmd") or [0, 0, 0], float)
            if esc.get("dir") and np.linalg.norm(v) > 0.05:
                sim.draw_arrow(p, p + 3.0 * v / max(np.linalg.norm(v), 1e-6), VP_ESCAPE, 10.0, 0.12)
        G = sim.cfg.gate
        for g in scenario.judged_gates:
            sim.draw_box(g.center, [G.cr_half_len, G.cr_half_width, G.cr_half_height], [60, 130, 230], 4.0, 0.12)
