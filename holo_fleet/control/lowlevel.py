"""Low-level velocity/heading loop: desired map-frame velocity -> normalised body command.

Feedback uses the DVL body velocity and the compass/gyro heading only.  Output is
the arena's normalised (surge, sway, heave, yaw) command, later mapped to the 8
BlueROV2 thrusters with the arena's own mapping.
"""

from __future__ import annotations

import math
from typing import Dict

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.nav import NavState, wrap

AUTHORITY = {"nominal": 0.40, "brake": 0.50, "escape": 0.60}   # max |(surge, sway)| normalised command


class LowLevelController:
    def __init__(self, cfg: FleetConfig = DEFAULT, k_v: float = 0.5, k_i: float = 0.4, k_vz: float = 0.25,
                 k_psi: float = 0.20, k_r: float = 0.35):
        self.cfg = cfg
        self.k_v, self.k_i, self.k_vz, self.k_psi, self.k_r = k_v, k_i, k_vz, k_psi, k_r
        self.i_xy = np.zeros(2)
        self.saturated = False                     # last horizontal command hit the authority limit

    def command(self, nav: NavState, v_d: np.ndarray, yaw_d: float, authority: str, dt: float) -> Dict[str, float]:
        pl = self.cfg.plant
        c, s = math.cos(nav.yaw), math.sin(nav.yaw)
        vb_d = np.array([c * v_d[0] + s * v_d[1], -s * v_d[0] + c * v_d[1]])
        vb = np.asarray(nav.v_body[:2], dtype=float)
        err = vb_d - vb
        lim = AUTHORITY.get(authority, 0.30)
        u = vb_d / pl.surge_speed_per_cmd + self.k_v * err + self.k_i * self.i_xy
        n = float(np.linalg.norm(u))
        self.saturated = n > lim
        if n > lim:
            u *= lim / n
        else:
            # conditional integration (anti-windup): integrate only while not saturated;
            # the integral term is what rejects a steady current measured through the DVL
            self.i_xy = np.clip(self.i_xy + err * dt, -0.75, 0.75)
        heave = v_d[2] / pl.heave_speed_per_cmd + self.k_vz * (v_d[2] - float(nav.v_body[2]))
        heave = float(np.clip(heave, -0.35, 0.35))
        yaw = float(np.clip(self.k_psi * wrap(yaw_d - nav.yaw) - self.k_r * nav.yaw_rate, -0.12, 0.12))
        return {"surge": float(u[0]), "sway": float(u[1]), "heave": heave, "yaw": yaw}
