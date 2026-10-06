"""Onboard navigation: compass heading + DVL dead reckoning + pressure depth.

The estimate starts from the surveyed launch pose in the mission plan (plus a
possibly non-zero initial error injected by the scenario) and is propagated with
onboard measurements only.  It is the drone's *belief*, not ground truth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class NavState:
    p: np.ndarray                       # estimated position (map frame), z from depth sensor
    yaw: float                          # estimated heading [rad]
    yaw_rate: float = 0.0               # [rad/s], CCW positive
    v_body: np.ndarray = field(default_factory=lambda: np.zeros(3))   # DVL, x fwd y left z up
    v_world: np.ndarray = field(default_factory=lambda: np.zeros(3))
    t_dvl: float = -1e9
    t_depth: float = -1e9
    t_compass: float = -1e9


class DeadReckoning:
    def __init__(self, launch_position: np.ndarray, launch_yaw_deg: float):
        self.state = NavState(p=np.array(launch_position, dtype=float), yaw=math.radians(launch_yaw_deg))

    def update(self, t: float, dt: float, compass: Optional[np.ndarray], dvl: Optional[np.ndarray],
               depth: Optional[float], imu: Optional[np.ndarray]) -> NavState:
        s = self.state
        if compass is not None:
            m = np.asarray(compass, dtype=float).ravel()
            s.yaw = math.atan2(m[1], m[0])
            s.t_compass = t
        if imu is not None:
            arr = np.asarray(imu, dtype=float)
            if arr.ndim == 2 and arr.shape[0] >= 2:
                s.yaw_rate = -float(arr[1][2])
        if dvl is not None:
            v = np.asarray(dvl, dtype=float).ravel()[:3]
            if np.all(np.isfinite(v)):
                s.v_body = v
                s.t_dvl = t
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        s.v_world = np.array([c * s.v_body[0] - sn * s.v_body[1], sn * s.v_body[0] + c * s.v_body[1], s.v_body[2]])
        s.p[0] += s.v_world[0] * dt
        s.p[1] += s.v_world[1] * dt
        if depth is not None:
            s.p[2] = float(np.asarray(depth).ravel()[0])
            s.t_depth = t
        else:
            s.p[2] += s.v_world[2] * dt
        return s

    def body_to_world_rot(self) -> np.ndarray:
        c, sn = math.cos(self.state.yaw), math.sin(self.state.yaw)
        return np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
