"""Referee-side sensor audit: what the degraded sensors really delivered, judged against ground truth.

Fed by the runner once per control step with the ground truth at the frame time, the frames the controllers
received and the controllers' own perception output (read only).  Nothing goes back to a controller.

* sonar: per sector, every other hull whose inscribed ball touches the cone within ``R_EXPECT`` m of the sensor
  must produce an obstacle echo (DYNAMIC/UNKNOWN) near its true near-surface range (recall / missed dynamic
  associations); an obstacle echo that no hull explains is a false dynamic association; range error = first bin of
  the matched echo - exact distance from the sonar to the target hull box (truth pose), for targets within 45 deg of
  the boresight; staleness = age of the data beyond tau_max, longest stale interval per drone;
* DVL: body velocity vs the true body velocity at the frame time (every new sample: latency included); heading: compass heading vs true yaw; depth:
  pressure depth vs true z (offset of the DepthSocket removed with the median); gyro: yaw rate vs the finite
  difference of the true yaw; navigation: onboard position / heading estimate vs truth.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS, R_IN, R_OUT, SECTORS, ball_touches_cone, sonar_sensor_name
from holo_fleet.perception.sonar_processing import OBSTACLE_CLASSES, UNCONFIRMED

R_EXPECT = 5.0          # completeness region of A1 (results/v2/ASSUMPTIONS.md)
MATCH_TOL = 0.6         # an echo within this of a hull's near-surface range is that hull's echo
BORESIGHT_DEG = 45.0


def _wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def box_distance(p: np.ndarray, centre: np.ndarray, R: np.ndarray, half=HULL_HALF) -> float:
    q = R.T @ (np.asarray(p, float) - np.asarray(centre, float))
    return float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0)))


def _pct(x: Sequence[float], q) -> List[float]:
    return [round(float(v), 4) for v in np.percentile(np.asarray(x, float), q)] if len(x) else []


class SensorAudit:
    def __init__(self, names: Sequence[str], cfg: FleetConfig = DEFAULT, dt: float = 0.1):
        self.names = list(names)
        self.cfg = cfg
        self.dt = dt
        n = len(self.names)
        self.range_err: List[float] = []
        self.expected = self.matched = self.missed = self.unconfirmed = 0
        self.obstacle_echoes = self.false_dynamic = 0
        self.steps = 0
        self.stale_steps = np.zeros(n, int)
        self.stale_run = np.zeros(n)
        self.longest_stale = np.zeros(n)
        self.max_age = 0.0
        self.dvl_err: List[List[float]] = [[] for _ in range(n)]
        self.dvl_fresh = np.zeros(n, int)
        self.dvl_gap = np.zeros(n)
        self.dvl_longest_gap = np.zeros(n)
        self.heading_err: List[List[float]] = [[] for _ in range(n)]
        self.depth_err: List[List[float]] = [[] for _ in range(n)]
        self.gyro_err: List[float] = []
        self.nav_err: List[List[float]] = [[] for _ in range(n)]
        self.nav_yaw_err: List[float] = []
        self._yaw_prev = None
        self._stamp = [dict() for _ in range(n)]      # last stamp seen per stream: a new sample = a new stamp

    def _new(self, k: int, fr, key: str) -> bool:
        st = fr.stamp.get(key)
        if key not in fr.data or st is None or st == self._stamp[k].get(key):
            return False
        self._stamp[k][key] = st
        return True

    def update(self, truth, frames: Dict, ctrls: Dict) -> None:
        tau = self.cfg.env.tau_max
        # attitude from the true yaw only (roll/pitch stay within a few degrees): PoseSensor's rotation matrix is not in
        # the body x-fwd / y-left / z-up convention of the sonar mounts (validate_assumptions_v2 does the same)
        P = truth.positions
        yr = np.radians(np.asarray(truth.yaw_deg, float))
        R = np.array([[[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]] for a in yr])
        hulls = [(P[j], R[j], j) for j in range(len(self.names))]
        if truth.intruders is not None:
            hulls += [(np.asarray(q, float), np.eye(3), -1 - m) for m, q in enumerate(truth.intruders)]
        self.steps += 1
        yaw_rate = None
        if self._yaw_prev is not None:
            yaw_rate = np.radians(_wrap_deg(truth.yaw_deg - self._yaw_prev)) / self.dt
        self._yaw_prev = np.array(truth.yaw_deg, float)
        for k, name in enumerate(self.names):
            fr = frames[name]
            c = ctrls.get(name)
            obs = None if c is None else c.last_obs
            # ---- sonar freshness
            ages = [fr.age(sonar_sensor_name(s)) if sonar_sensor_name(s) in fr.data else 1e9 for s in SECTORS]
            stale = max(ages) > tau + 1e-9
            if fr.t >= 0.5:                            # start-up: no capture yet
                self.max_age = max(self.max_age, min(max(ages), 99.0))
            if stale:
                self.stale_steps[k] += 1
                self.stale_run[k] += self.dt
                self.longest_stale[k] = max(self.longest_stale[k], self.stale_run[k])
            else:
                self.stale_run[k] = 0.0
            # ---- sonar detections (controller perception output vs truth)
            if obs is not None:
                for s in SECTORS:
                    rd = obs.readings.get(s)
                    if rd is None or not rd.healthy:
                        continue
                    spos = P[k] + R[k] @ MOUNTS[s]
                    axis_w = R[k] @ AXES[s]
                    cand = []
                    for (c_h, R_h, j) in hulls:
                        if j == k:
                            continue
                        body = R[k].T @ (c_h - P[k])
                        d_s = box_distance(spos, c_h, R_h)
                        cand.append((j, d_s, bool(ball_touches_cone(body[None], s, R_IN)[0]),
                                     bool(ball_touches_cone(body[None], s, R_OUT)[0]), c_h))
                    obst = [e for e in rd.echoes if e.cls in OBSTACLE_CLASSES]
                    self.obstacle_echoes += len(obst)
                    for e in obst:
                        if not any(touch_out and abs(e.r0 - d_s) <= 1.0 for (_j, d_s, _ti, touch_out, _c) in cand):
                            self.false_dynamic += 1
                    for (j, d_s, touch_in, _to, c_h) in cand:
                        if not touch_in or np.linalg.norm(c_h - spos) > R_EXPECT:
                            continue
                        self.expected += 1
                        hit = [e for e in obst if abs(e.r0 - d_s) <= MATCH_TOL]
                        if hit:
                            self.matched += 1
                            u = (c_h - spos) / max(np.linalg.norm(c_h - spos), 1e-9)
                            if math.degrees(math.acos(float(np.clip(u @ axis_w, -1.0, 1.0)))) <= BORESIGHT_DEG:
                                self.range_err.append(min(hit, key=lambda e: abs(e.r0 - d_s)).r0 - d_s)
                        elif any(e.cls == UNCONFIRMED and abs(e.r0 - d_s) <= MATCH_TOL for e in rd.echoes):
                            self.unconfirmed += 1
                        else:
                            self.missed += 1
                self.nav_err[k].append(float(np.linalg.norm(obs.p - P[k])))
                self.nav_yaw_err.append(float(_wrap_deg(math.degrees(obs.yaw) - truth.yaw_deg[k])))
            # ---- DVL / compass / depth / gyro
            dv = fr.data.get("DVLSensor")
            if dv is not None and self._new(k, fr, "DVLSensor"):
                v_true = R[k].T @ np.asarray(truth.velocities[k], float)
                v = np.asarray(dv, float).ravel()[:3]
                self.dvl_err[k].append(float(np.linalg.norm(v[:2] - v_true[:2])))
                self.dvl_fresh[k] += 1
                self.dvl_gap[k] = 0.0
            else:
                self.dvl_gap[k] += self.dt
                self.dvl_longest_gap[k] = max(self.dvl_longest_gap[k], self.dvl_gap[k])
            m = fr.data.get("Compass")
            if m is not None and self._new(k, fr, "Compass"):
                m = np.asarray(m, float).ravel()
                self.heading_err[k].append(float(_wrap_deg(math.degrees(math.atan2(m[1], m[0])) - truth.yaw_deg[k])))
            dz = fr.data.get("DepthSensor")
            if dz is not None and self._new(k, fr, "DepthSensor"):
                self.depth_err[k].append(float(np.asarray(dz).ravel()[0]) - float(P[k][2]))
            imu = fr.data.get("IMUSensor")
            if imu is not None and yaw_rate is not None:
                arr = np.asarray(imu, float)
                if arr.ndim == 2 and arr.shape[0] >= 2:
                    self.gyro_err.append(float(-arr[1][2] - yaw_rate[k]))

    def summary(self) -> Dict:
        env = self.cfg.env
        re = np.asarray(self.range_err, float)
        hd = [x for h in self.heading_err for x in h]
        dz = [x - np.median(h) for h in self.depth_err if h for x in h]
        dvl = [x for h in self.dvl_err for x in h]
        nav = [x for h in self.nav_err for x in h]
        return {
            "sonar": {
                "range_err_m_p01_p05_p50_p95_p99": _pct(re, [1, 5, 50, 95, 99]),
                "range_err_max_m": round(float(re.max()), 4) if re.size else None,
                "range_err_min_m": round(float(re.min()), 4) if re.size else None,
                "range_err_samples": int(re.size),
                "frac_beyond_eps_far": round(float(np.mean(re > env.eps_range_far)), 5) if re.size else None,
                "frac_beyond_eps_near": round(float(np.mean(re < -env.eps_range_near)), 5) if re.size else None,
                "expected_detections": self.expected, "matched": self.matched, "missed": self.missed,
                "unconfirmed": self.unconfirmed,
                "recall": round(self.matched / self.expected, 4) if self.expected else None,
                "obstacle_echoes": self.obstacle_echoes, "false_dynamic": self.false_dynamic,
                "false_dynamic_frac": round(self.false_dynamic / self.obstacle_echoes, 4) if self.obstacle_echoes else None,
                "stale_fraction": round(float(self.stale_steps.sum()) / max(1, self.steps * len(self.names)), 5),
                "longest_stale_s": round(float(self.longest_stale.max()), 2) if len(self.names) else None,
                "max_age_s": round(float(self.max_age), 3)},
            "dvl": {"vel_err_rms_m_s": round(float(np.sqrt(np.mean(np.square(dvl)))), 4) if dvl else None,
                    "vel_err_p95_m_s": _pct(dvl, [95])[0] if dvl else None,
                    "fresh_fraction": round(float(self.dvl_fresh.sum()) / max(1, self.steps * len(self.names)), 4),
                    "longest_gap_s": round(float(self.dvl_longest_gap.max()), 2)},
            "heading": {"err_rms_deg": round(float(np.sqrt(np.mean(np.square(hd)))), 3) if hd else None,
                        "err_p95_abs_deg": _pct(np.abs(hd), [95])[0] if hd else None,
                        "bias_per_drone_deg": [round(float(np.mean(h)), 2) if h else None for h in self.heading_err]},
            "depth": {"err_std_m": round(float(np.std(dz)), 4) if dz else None,
                      "err_p95_abs_m": _pct(np.abs(dz), [95])[0] if dz else None},
            "gyro": {"yaw_rate_err_rms_rad_s": round(float(np.sqrt(np.mean(np.square(self.gyro_err)))), 4)
                     if self.gyro_err else None},
            "navigation": {"pos_err_max_m": round(float(max(nav)), 3) if nav else None,
                           "pos_err_rms_m": round(float(np.sqrt(np.mean(np.square(nav)))), 3) if nav else None,
                           "pos_err_final_max_m": round(float(max(h[-1] for h in self.nav_err if h)), 3) if nav else None,
                           "yaw_err_rms_deg": round(float(np.sqrt(np.mean(np.square(self.nav_yaw_err)))), 3)
                           if self.nav_yaw_err else None},
        }
