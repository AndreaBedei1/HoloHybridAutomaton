"""One-interval P1 margin with a sound nearest-centre distance and ground-speed bounds.

The calculation covers the entire interval. It does not establish echo completeness
or a terminal relative hold. A missing echo cannot supply a distance bound.
"""

import math

import numpy as np


def swept_margin(distance, own_speed, peer_speed, safety_distance, interval):
    return distance - safety_distance - (own_speed + peer_speed) * interval


def interval_record(obs, frame, velocity, thrust, dt, cfg):
    env = cfg.env
    speed = max(env.v_escape, env.v_max_nominal, cfg.gate.v_pass) + env.w_drift_max
    if not obs.targets or obs.unhealthy or not obs.sense_ok or not math.isfinite(obs.d_min):
        return {"status": "UNOBSERVED"}
    dvl = frame.sample("DVLSensor", env.tau_max, 3)
    imu = frame.sample("IMUSensor", env.tau_max, 6)
    if dvl is None or imu is None or imu.ndim != 2 or imu.shape[0] < 2 or imu.shape[1] != 3:
        return {"status": "MOTION_UNOBSERVED"}
    if (not np.isfinite(obs.v_world).all() or not np.isfinite(velocity).all() or not np.isfinite(thrust).all()
            or np.linalg.norm(obs.v_world) > speed or np.linalg.norm(velocity) > speed - env.w_drift_max
            or np.max(abs(thrust)) > cfg.plant.thruster_limit or not 0.0 < dt <= env.dt + 1e-9):
        return {"status": "MOTION_OUTSIDE"}
    # The existing echo correction budgets 1.20 m/s. Allow two peers at the full speed cap.
    correction = max(0.0, 2 * speed - cfg.perc.v_close_staleness) * env.tau_max
    margin = swept_margin(obs.d_min - correction, speed, speed, cfg.sep.d_safe, env.dt)
    return {"status": "INTERVAL_CONDITIONAL" if margin > 0.0 else "NO_MARGIN", "margin_m": margin,
            "horizon_s": env.dt, "speed_cap_m_s": speed}
