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
               depth: Optional[float], imu: Optional[np.ndarray], stamps: Optional[dict] = None) -> NavState:
        """``stamps``: the onboard time of the last update of each sensor (SensorFrame.stamp).  A frame holds the
        LATEST value of a sensor with its own stamp: the freshness times are those stamps, not t (a held DVL
        sample is not a new measurement)."""
        s = self.state
        st = stamps or {}
        if compass is not None:
            m = np.asarray(compass, dtype=float).ravel()
            s.yaw = math.atan2(m[1], m[0])
            s.t_compass = float(st.get("Compass", t))
        if imu is not None:
            arr = np.asarray(imu, dtype=float)
            if arr.ndim == 2 and arr.shape[0] >= 2:
                s.yaw_rate = -float(arr[1][2])
        if dvl is not None:
            v = np.asarray(dvl, dtype=float).ravel()[:3]
            if np.all(np.isfinite(v)):
                s.v_body = v
                s.t_dvl = float(st.get("DVLSensor", t))
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        s.v_world = np.array([c * s.v_body[0] - sn * s.v_body[1], sn * s.v_body[0] + c * s.v_body[1], s.v_body[2]])
        s.p[0] += s.v_world[0] * dt
        s.p[1] += s.v_world[1] * dt
        if depth is not None:
            s.p[2] = float(np.asarray(depth).ravel()[0])
            s.t_depth = float(st.get("DepthSensor", t))
        else:
            s.p[2] += s.v_world[2] * dt
        return s

    def body_to_world_rot(self) -> np.ndarray:
        c, sn = math.cos(self.state.yaw), math.sin(self.state.yaw)
        return np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])



RAYLEIGH_3SIGMA = math.sqrt(-2.0 * math.log(1.0 - 0.9973))   # 3.44: 99.73 % of a 2-D isotropic normal error
DVL_LEVER_M = 0.25            # [ASSUMPTION] bound of the DVL offset from the vehicle centre (omega x r not compensated)
SENSOR_LATENCY_S = 0.05       # [PROF] MEDIUM sensor latency (sim/sensor_noise.py)
A_MAX_MPS2 = 1.5              # [CAL] largest speed change rate: reversal from +0.45 to -0.30 m/s in 0.5 s
                              # (results/calibration/plant_calibration.json, "reversal")
A_MAX_NOTE = ("the reversal average; for an outage shorter than 0.5 s a higher peak rate could exceed it, which the "
              "cap |v_held| + v_bound does not cover: a stated limit")


class NavBound:
    """Conservative onboard bound of the own dead-reckoning position error (shared mission frame), sent in the B2
    payload (H6).  It grows with the declared envelope of the navigation sensors, never with the profile of a run:

      b_h(t) = nav_b0_m                                         launch survey of the shared frame [REQ]
             + sum (2 sin(heading_err / 2) + s)(1 + s) |v_h| dt  compass bias and DVL scale error s on the true
                                                                 speed (<= (1 + s) |v_h|): drift proportional to
                                                                 the distance travelled
             + 3.44 sigma_v sqrt(sum dt^2)                       white DVL noise integrated by the dead reckoning
                                                                 (a 2-D random walk at the 99.73 % level)
             + sum min(a_max (tau - T_nom), |v_held| + v_bound) dt DVL outage: beyond the nominal hold T_nom the
                                                                 held velocity is wrong by at most a_max (tau - T_nom)
             + sum |unvalidated corrections|                     an applied map fit may itself be wrong
             + 2 r_DVL + 2 (T_dvl + latency) v_bound             lever arm (bounded rotation of r), hold + latency
      b_z    = nav_depth_err_m while the depth sensor delivers new samples, growing with the speed bound otherwise.
      bound  = hypot(b_h, b_z): the radius of a BALL containing every error with |e_h| <= b_h and |e_z| <= b_z.

    The bound NEVER decreases.  A validated gate localization is not used to restart it: the fit uncertainty u is
    not an error bound (the near-optimal spread under-reports, structure_aiding's docstring), and a wrong fit would
    become a mutual-exclusion premise of the whole fleet through the payload.  Soundness is kept at the cost of a
    looser bound on long missions.  Integrity: an applied correction larger than the current horizontal bound shows that the bound was violated;
    the bound is then INVALID for the rest of the run (the payload is sent marked unusable).

    The sum of worst cases of independent terms (no cancellation assumed).  It is an envelope assumption like the
    acoustic one: a sensor fault outside the declared envelope (a larger compass bias - the HIGH profile's 4 deg
    exceeds the declared 3 deg -, a DVL that reports wrong velocities) violates it; every run logs the bound
    against the true error (the evaluation's nav-bound coverage)."""

    # [PROT] a sample is "held" (outage) when older than one 10 Hz DVL period (0.1 s) + the declared sensor latency
    # (0.05 s, MEDIUM).  In the simulator an ordinary sample is at most 0.133 s old at a control instant, so every
    # dropped sample counts (a 0.25 s threshold, tried on 2026-11-05 on a wrong diagnosis, ignored isolated dropouts)
    DVL_STALE_S = 0.15

    def __init__(self, cfg):
        a, env = cfg.acoustic, cfg.env
        s = a.dvl_scale_err
        self.k_lin = (2.0 * math.sin(math.radians(a.heading_err_deg) / 2.0) + s) * (1.0 + s)
        self.k_rw = RAYLEIGH_3SIGMA * a.dvl_sigma_body_mps
        self.depth_err = a.nav_depth_err_m
        self.v_bound = max(env.v_escape, env.v_max_nominal, cfg.gate.v_pass) + env.w_drift_max
        # zero-order hold of a sample plus its latency: an effective delay of at most one period + latency, at the
        # speed bound, both ways
        self.const = 2.0 * DVL_LEVER_M + 2.0 * (0.1 + SENSOR_LATENCY_S) * self.v_bound
        self.ref = a.nav_b0_m
        self.lin = self.var = self.outage = self.aid = 0.0
        self.z = self.depth_err
        self.invalid = False

    def update(self, t: float, dt: float, s: NavState) -> None:
        v_h = math.hypot(float(s.v_world[0]), float(s.v_world[1]))
        self.lin += self.k_lin * v_h * dt
        tau = t - s.t_dvl
        if tau <= self.DVL_STALE_S:
            self.var += dt * dt
        else:
            # beyond the nominal hold (covered by the hold + latency constant) the held velocity drifts from the true
            # one by at most a_max (tau - T_nominal), and never by more than |v_held| + v_bound
            self.outage += min(A_MAX_MPS2 * (tau - self.DVL_STALE_S), v_h + self.v_bound) * dt
        self.z = self.depth_err if t - s.t_depth <= self.DVL_STALE_S else self.z + self.v_bound * dt

    def corrected(self, corr) -> None:
        c = float(np.linalg.norm(np.asarray(corr, dtype=float)[:2]))
        if c > self.horizontal:
            self.invalid = True                 # the estimate was farther off than its bound: integrity lost
        self.aid += c

    @property
    def horizontal(self) -> float:
        return self.ref + self.lin + self.k_rw * math.sqrt(self.var) + self.outage + self.aid + self.const

    @property
    def value(self) -> float:
        """Radius of a ball containing the true position (inf when the integrity monitor has fired)."""
        return math.inf if self.invalid else math.hypot(self.horizontal, self.z)
