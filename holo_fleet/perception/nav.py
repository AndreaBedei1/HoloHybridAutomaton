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


class NavBound:
    """Conservative onboard bound of the own dead-reckoning position error (shared mission frame), sent in the B2
    payload (H6).  It grows with the declared envelope of the navigation sensors, never with the profile of a run:

      b_h(t) = nav_b0_m                                         launch survey of the shared frame [REQ]
             + sum (sin(heading_err) + dvl_scale_err) |v_h| dt   compass bias and DVL scale error: a drift
                                                                 proportional to the distance travelled
             + k_sigma dvl_sigma_body sqrt(sum dt^2)             white DVL noise integrated by the dead reckoning
                                                                 (a random walk; k_sigma as in the acoustic envelope)
             + sum a_max tau dt                                  DVL outage: the held velocity is wrong by at most
                                                                 a_max tau after tau seconds without a sample
                                                                 (a_max: the calibrated closed-loop deceleration)
             + sum |correction|                                  a map-aided correction may itself be wrong: the
                                                                 bound never shrinks
      b_z    = nav_depth_err_m while the depth sensor is fresh, growing with the speed bound while it is not.

    The bound is the sum of worst cases of independent terms (no cancellation assumed), at k_sigma for the one white
    term.  It is an envelope assumption like the acoustic one: a sensor fault outside the declared envelope (a
    larger compass bias, a DVL fault that still reports) violates it; every run logs the true error against it."""

    DVL_STALE_S = 0.15                       # one 10 Hz DVL sample plus jitter: older means the velocity is held

    def __init__(self, cfg):
        a, env = cfg.acoustic, cfg.env
        self.k_lin = math.sin(math.radians(a.heading_err_deg)) + a.dvl_scale_err
        self.k_rw = a.k_sigma * a.dvl_sigma_body_mps
        self.a_max = env.a_brake_measured
        self.depth_err = a.nav_depth_err_m
        self.v_bound = max(env.v_escape, env.v_max_nominal, cfg.gate.v_pass) + env.w_drift_max
        self.lin = a.nav_b0_m
        self.var = 0.0                       # sum dt^2 of the integrated white velocity noise
        self.outage = 0.0
        self.aid = 0.0
        self.z = self.depth_err

    def update(self, t: float, dt: float, s: NavState) -> None:
        v_h = math.hypot(float(s.v_world[0]), float(s.v_world[1]))
        self.lin += self.k_lin * v_h * dt
        tau = t - s.t_dvl
        if tau <= self.DVL_STALE_S:
            self.var += dt * dt
        else:
            self.outage += self.a_max * min(tau, 1e3) * dt
        self.z = self.depth_err if t - s.t_depth <= self.DVL_STALE_S else self.z + self.v_bound * dt

    def corrected(self, corr) -> None:
        self.aid += float(np.linalg.norm(np.asarray(corr, dtype=float)[:2]))

    @property
    def horizontal(self) -> float:
        return self.lin + self.k_rw * math.sqrt(self.var) + self.outage + self.aid

    @property
    def value(self) -> float:
        """A ball radius containing the true position: the larger of the horizontal and vertical bounds."""
        return max(self.horizontal, self.z)
